// SPDX-License-Identifier: MIT
#include "steamvr_reset.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QSet>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <unistd.h>

#include "steamvr.h"

namespace pibridge::app {

namespace {

QByteArray readFile(const QString& path) {
    QFile f(path);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

// A path under `dir` (dir itself does not count; "SteamVR2/..." does not match "SteamVR").
bool under(const QString& path, const QString& dir) {
    return !dir.isEmpty() && path.size() > dir.size() + 1 && path.startsWith(dir) && path.at(dir.size()) == QLatin1Char('/');
}

bool underAny(const QString& path, const QStringList& dirs) {
    return std::any_of(dirs.begin(), dirs.end(), [&](const QString& d) { return under(path, d); });
}

// Steam's own client binaries: ubuntu12_32/steam, steamwebhelper (ubuntu12_64/ or ubuntu12_32/), reaper, ...
bool steamClient(const QString& exe, const QString& arg0, const QString& comm) {
    if (comm == QLatin1String("steam") || comm == QLatin1String("steamwebhelper") || comm == QLatin1String("reaper")) {
        return true;
    }
    for (const QString& p : {exe, arg0}) {
        if (p.contains(QLatin1String("/ubuntu12_32/")) || p.contains(QLatin1String("/ubuntu12_64/")) ||
            p.endsWith(QLatin1String("/steamwebhelper"))) {
            return true;
        }
    }
    return false;
}

// /proc/<pid>/stat: "pid (comm) state ppid ...": field 22 is the start time. comm may hold spaces and ")", so the
// fields are counted after the last ")".
qint64 statStartTicks(const QByteArray& stat) {
    const qsizetype close = stat.lastIndexOf(')');
    if (close < 0) {
        return -1;
    }
    const QList<QByteArray> f = stat.mid(close + 1).simplified().split(' ');
    // f[0] is field 3 (state); field 22 is f[19]
    if (f.size() < 20) {
        return -1;
    }
    bool ok = false;
    const qint64 v = f[19].toLongLong(&ok);
    return ok ? v : -1;
}

char statState(const QByteArray& stat) {
    const qsizetype close = stat.lastIndexOf(')');
    if (close < 0 || close + 2 >= stat.size()) {
        return '?';
    }
    return stat.at(close + 2);
}

QStringList splitCmdline(const QByteArray& raw) {
    QStringList args;
    for (const QByteArray& a : raw.split('\0')) {
        if (!a.isEmpty()) {
            args << QString::fromLocal8Bit(a);
        }
    }
    return args;
}

QString baseName(const QString& p) {
    const qsizetype slash = p.lastIndexOf(QLatin1Char('/'));
    return slash < 0 ? p : p.mid(slash + 1);
}

QString pidList(const std::vector<SteamVrCloser::Item>& items, SteamVrCloser::Outcome o, bool showKill) {
    QStringList l;
    for (const auto& i : items) {
        if (i.outcome != o) {
            continue;
        }
        QString s = QStringLiteral("%1 (PID %2").arg(i.proc.name).arg(i.proc.pid);
        if (showKill) {
            s += QStringLiteral(", needed a forced stop");
        }
        if (!i.error.isEmpty()) {
            s += QStringLiteral(", ") + i.error;
        }
        l << s + QLatin1Char(')');
    }
    return l.join(QStringLiteral(", "));
}

} // namespace

// ---------------------------------------------------------------- detection

std::vector<SteamVrProc> SteamVrScan::targets() const {
    std::vector<SteamVrProc> t;
    for (const SteamVrProc& p : procs) {
        if (p.inRuntime) {
            t.push_back(p);
        }
    }
    return t;
}

bool SteamVrScan::vrserverNamedRight() const {
    return std::any_of(procs.begin(), procs.end(), [](const SteamVrProc& p) {
        return p.inRuntime && p.name == QLatin1String("vrserver") && p.comm == QLatin1String("vrserver");
    });
}

QString steamVrRuntimeDir() {
    const QString rt = openVrRuntimeDir(openVrPathsFile());
    if (!rt.isEmpty()) {
        return rt;
    }
    return QDir::homePath() + QStringLiteral("/.local/share/Steam/steamapps/common/SteamVR");
}

std::vector<std::pair<qint64, QString>> steamVrSockets(const QByteArray& procNetUnix) {
    // "Num RefCount Protocol Flags Type St Inode Path": the path is the 8th column (abstract names start with "@")
    std::vector<std::pair<qint64, QString>> out;
    const QList<QByteArray> lines = procNetUnix.split('\n');
    for (qsizetype i = 1; i < lines.size(); i++) {
        const QList<QByteArray> f = lines[i].simplified().split(' ');
        if (f.size() < 8 || !f[7].startsWith("@/steamvr/")) {
            continue;
        }
        bool ok = false;
        const qint64 inode = f[6].toLongLong(&ok);
        if (!ok) {
            continue;
        }
        QByteArray path = f[7];
        for (qsizetype k = 8; k < f.size(); k++) { // a path with spaces
            path += ' ' + f[k];
        }
        out.emplace_back(inode, QString::fromLocal8Bit(path));
    }
    return out;
}

SteamVrScan scanSteamVr(const ScanOptions& opt) {
    SteamVrScan scan;
    scan.runtimeDir = QDir::cleanPath(opt.runtimeDir.isEmpty() ? steamVrRuntimeDir() : opt.runtimeDir);
    // the dir as configured and as it really is (~/.steam/steam is a link to ~/.local/share/Steam; exe links resolve)
    QStringList dirs{scan.runtimeDir};
    if (const QString canon = QFileInfo(scan.runtimeDir).canonicalFilePath(); !canon.isEmpty() && canon != scan.runtimeDir) {
        dirs << canon;
    }
    const long tps = opt.ticksPerS > 0 ? opt.ticksPerS : std::max(1L, sysconf(_SC_CLK_TCK));
    const int self = opt.selfPid > 0 ? opt.selfPid : static_cast<int>(getpid());
    const QString root = opt.procRoot;

    // uptime in seconds (first number of /proc/uptime), for the running time
    double uptime = -1;
    {
        const QList<QByteArray> u = readFile(root + QStringLiteral("/uptime")).simplified().split(' ');
        bool ok = false;
        if (!u.isEmpty()) {
            uptime = u[0].toDouble(&ok);
        }
        if (!ok) {
            uptime = -1;
        }
    }

    // inode -> socket path of every @/steamvr/* socket
    QHash<qint64, QString> socketByInode;
    for (const auto& [inode, path] : steamVrSockets(readFile(root + QStringLiteral("/net/unix")))) {
        socketByInode.insert(inode, path);
    }
    QSet<QString> heldSockets;

    QStringList pids = QDir(root).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QString& pidText : pids) {
        bool ok = false;
        const int pid = pidText.toInt(&ok);
        if (!ok || pid <= 0 || pid == self) {
            continue;
        }
        const QString dir = root + QLatin1Char('/') + pidText;
        QString exe = QFile::symLinkTarget(dir + QStringLiteral("/exe"));
        if (exe.endsWith(QLatin1String(" (deleted)"))) { // SteamVR was updated while it ran
            exe.chop(10);
        }
        const QStringList args = splitCmdline(readFile(dir + QStringLiteral("/cmdline")));
        const QString arg0 = args.isEmpty() ? QString() : args.first();
        const QString comm = QString::fromLocal8Bit(readFile(dir + QStringLiteral("/comm"))).trimmed();

        // the @/steamvr/* sockets it holds (fd links "socket:[inode]")
        QStringList sockets;
        if (!socketByInode.isEmpty()) {
            const QString fdDir = dir + QStringLiteral("/fd");
            for (const QString& fd : QDir(fdDir).entryList(QDir::AllEntries | QDir::System | QDir::NoDotAndDotDot)) {
                const QString target = QFile::symLinkTarget(fdDir + QLatin1Char('/') + fd);
                // symLinkTarget makes relative targets absolute: look at the file name only
                const QString t = baseName(target);
                if (!t.startsWith(QLatin1String("socket:[")) || !t.endsWith(QLatin1Char(']'))) {
                    continue;
                }
                const qint64 inode = t.mid(8, t.size() - 9).toLongLong(&ok);
                if (ok && socketByInode.contains(inode)) {
                    const QString& path = socketByInode[inode];
                    if (!sockets.contains(path)) {
                        sockets << path;
                    }
                }
            }
        }
        for (const QString& s : sockets) {
            heldSockets.insert(s);
        }

        if (steamClient(exe, arg0, comm)) {
            if (comm == QLatin1String("steam") || exe.endsWith(QLatin1String("/ubuntu12_32/steam"))) {
                scan.steamRunning = true;
            }
            continue; // never Steam itself, even when it holds a SteamVR socket
        }

        SteamVrProc p;
        p.pid = pid;
        p.comm = comm;
        p.exe = exe;
        p.cmdline = args.join(QLatin1Char(' '));
        p.sockets = sockets;
        if (underAny(exe, dirs)) {
            p.name = baseName(exe);
        } else if (underAny(QDir::cleanPath(arg0), dirs)) {
            p.name = baseName(arg0);
        } else if (std::any_of(args.begin() + (args.isEmpty() ? 0 : 1), args.end(),
                               [&](const QString& a) { return underAny(QDir::cleanPath(a), dirs) || a.contains(scan.runtimeDir + QLatin1Char('/')); })) {
            // pressure-vessel, a shell running vrstartup.sh, ...
            p.wrapper = true;
            p.name = baseName(exe.isEmpty() ? arg0 : exe);
        } else if (!sockets.isEmpty()) {
            p.inRuntime = false; // holds a SteamVR socket but is not SteamVR: named, never signalled
            p.name = baseName(exe.isEmpty() ? arg0 : exe);
        } else {
            continue;
        }
        if (p.name.isEmpty()) {
            p.name = comm;
        }
        const QByteArray stat = readFile(dir + QStringLiteral("/stat"));
        p.startTicks = statStartTicks(stat);
        if (p.startTicks >= 0 && uptime >= 0) {
            p.runningS = std::max<qint64>(0, static_cast<qint64>(uptime - double(p.startTicks) / double(tps)));
        }
        scan.procs.push_back(p);
    }
    std::sort(scan.procs.begin(), scan.procs.end(), [](const SteamVrProc& a, const SteamVrProc& b) { return a.pid < b.pid; });
    QSet<QString> seen;
    for (auto it = socketByInode.cbegin(); it != socketByInode.cend(); ++it) {
        if (!heldSockets.contains(it.value()) && !seen.contains(it.value())) {
            seen.insert(it.value());
            scan.orphanSockets << it.value();
        }
    }
    scan.orphanSockets.sort();
    return scan;
}

bool steamClientRunning(const QString& procRoot) {
    for (const QString& pidText : QDir(procRoot).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        if (pidText.isEmpty() || !pidText.front().isDigit()) {
            continue;
        }
        const QString dir = procRoot + QLatin1Char('/') + pidText;
        if (QString::fromLocal8Bit(readFile(dir + QStringLiteral("/comm"))).trimmed() == QLatin1String("steam") ||
            QFile::symLinkTarget(dir + QStringLiteral("/exe")).endsWith(QLatin1String("/ubuntu12_32/steam"))) {
            // a zombie is gone
            if (statState(readFile(dir + QStringLiteral("/stat"))) != 'Z') {
                return true;
            }
        }
    }
    return false;
}

QString runningWords(qint64 s) {
    if (s < 0) {
        return {};
    }
    if (s < 60) {
        return QStringLiteral("%1 s").arg(s);
    }
    const qint64 min = s / 60;
    if (min < 60) {
        return QStringLiteral("%1 min").arg(min);
    }
    const qint64 h = min / 60;
    if (h < 24) {
        return min % 60 ? QStringLiteral("%1 h %2 min").arg(h).arg(min % 60) : QStringLiteral("%1 h").arg(h);
    }
    return h % 24 ? QStringLiteral("%1 d %2 h").arg(h / 24).arg(h % 24) : QStringLiteral("%1 d").arg(h / 24);
}

QString procLine(const SteamVrProc& p) {
    QStringList parts{QStringLiteral("PID %1").arg(p.pid)};
    if (p.runningS >= 0) {
        parts << QStringLiteral("running ") + runningWords(p.runningS);
    }
    if (!p.comm.isEmpty() && p.comm != p.name.left(15)) {
        parts << QStringLiteral("shows as \"%1\"").arg(p.comm);
    }
    if (p.wrapper) {
        parts << QStringLiteral("starts SteamVR");
    }
    if (p.sockets.contains(QStringLiteral("@/steamvr/SteamVR_Namespace"))) {
        parts << QStringLiteral("holds the SteamVR socket");
    } else if (!p.sockets.isEmpty()) {
        parts << QStringLiteral("holds a SteamVR socket");
    }
    if (!p.inRuntime) {
        parts << QStringLiteral("not part of SteamVR: left alone");
    }
    return QStringLiteral("%1 (%2)").arg(p.name, parts.join(QStringLiteral(", ")));
}

// ---------------------------------------------------------------- the dialog's words

ResetDialogModel resetDialogModel(const SteamVrScan& scan, bool steamVrConnected) {
    ResetDialogModel m;
    for (const SteamVrProc& p : scan.procs) {
        m.lines << procLine(p);
    }
    for (const QString& s : scan.orphanSockets) {
        m.lines << QStringLiteral("The socket %1 is held by a process PiBridge cannot see.").arg(s);
    }
    const auto targets = scan.targets();
    if (targets.empty()) {
        m.mode = ResetDialogModel::Mode::RestartSteam;
        m.title = QStringLiteral("Reset SteamVR");
        m.intro = QStringLiteral("No SteamVR process is running. If Steam still says SteamVR is running, Steam's own "
                                 "state is stuck: restart Steam.");
        if (scan.steamRunning) {
            m.question = QStringLiteral("Restart Steam now?");
            m.warning = QStringLiteral("Games started from Steam will close.");
            m.confirm = QStringLiteral("Restart Steam");
        } else {
            m.question = QStringLiteral("Steam is not running. Start it?");
            m.confirm = QStringLiteral("Start Steam");
        }
        return m;
    }
    m.mode = ResetDialogModel::Mode::CloseSteamVr;
    m.confirm = QStringLiteral("Close SteamVR");
    m.warning = QStringLiteral("Games running in VR will close.");
    const bool healthy = scan.vrserverNamedRight() && steamVrConnected;
    if (healthy) {
        m.title = QStringLiteral("SteamVR looks fine");
        m.intro = QStringLiteral("SteamVR is running and PiBridge is connected to it. These are its processes:");
        m.question = QStringLiteral("Close SteamVR anyway?");
        return m;
    }
    m.title = QStringLiteral("Reset SteamVR");
    const bool socketHeld = std::any_of(targets.begin(), targets.end(), [](const SteamVrProc& p) { return !p.sockets.isEmpty(); });
    if (socketHeld && !scan.vrserverNamedRight()) {
        m.intro = QStringLiteral("SteamVR is left over from an earlier start and holds SteamVR's socket: new SteamVR "
                                 "starts fail until it is closed. These SteamVR processes are running:");
    } else {
        m.intro = QStringLiteral("These SteamVR processes are running:");
    }
    m.question = QStringLiteral("Close them? PiBridge asks each one to quit and stops the ones still there after 5 s.");
    return m;
}

// ---------------------------------------------------------------- closing

ProcSignals realProcSignals(const QString& procRoot) {
    ProcSignals s;
    s.send = [](int pid, int sig) { return ::kill(pid, sig) == 0 ? 0 : errno; };
    s.alive = [procRoot](int pid, qint64 startTicks) {
        const QByteArray stat = readFile(procRoot + QStringLiteral("/%1/stat").arg(pid));
        if (stat.isEmpty() || statState(stat) == 'Z' || statState(stat) == 'X') {
            return false;
        }
        return startTicks < 0 || statStartTicks(stat) == startTicks; // another process under a reused PID = gone
    };
    return s;
}

SteamVrCloser::SteamVrCloser(std::vector<SteamVrProc> targets, ProcSignals sig) : sig_(std::move(sig)) {
    for (SteamVrProc& p : targets) {
        if (p.inRuntime && p.pid > 0) {
            items_.push_back({std::move(p), Outcome::Pending, {}});
        }
    }
}

void SteamVrCloser::begin(qint64 nowMs) {
    startMs_ = nowMs;
    for (Item& i : items_) {
        if (!sig_.alive(i.proc.pid, i.proc.startTicks)) {
            i.outcome = Outcome::Closed; // already gone (or the PID is someone else's now)
            continue;
        }
        const int e = sig_.send(i.proc.pid, SIGTERM);
        if (e == ESRCH) {
            i.outcome = Outcome::Closed;
        } else if (e != 0) {
            i.outcome = Outcome::Failed;
            i.error = e == EPERM ? QStringLiteral("not allowed") : QString::fromLocal8Bit(std::strerror(e));
        }
    }
    step(nowMs);
}

bool SteamVrCloser::step(qint64 nowMs) {
    if (finished_) {
        return true;
    }
    bool pending = false;
    for (Item& i : items_) {
        if (i.outcome != Outcome::Pending) {
            continue;
        }
        if (!sig_.alive(i.proc.pid, i.proc.startTicks)) {
            i.outcome = killMs_ >= 0 ? Outcome::Killed : Outcome::Closed;
        } else {
            pending = true;
        }
    }
    if (!pending) {
        finished_ = true;
        return true;
    }
    if (killMs_ < 0 && nowMs - startMs_ >= kTermWaitMs) {
        killMs_ = nowMs;
        for (Item& i : items_) {
            if (i.outcome != Outcome::Pending) {
                continue;
            }
            const int e = sig_.send(i.proc.pid, SIGKILL);
            if (e == ESRCH) {
                i.outcome = Outcome::Killed;
            } else if (e != 0) {
                i.outcome = Outcome::Failed;
                i.error = e == EPERM ? QStringLiteral("not allowed") : QString::fromLocal8Bit(std::strerror(e));
            }
        }
        return false;
    }
    if (killMs_ >= 0 && nowMs - killMs_ >= kKillWaitMs) {
        for (Item& i : items_) {
            if (i.outcome == Outcome::Pending) {
                i.outcome = Outcome::StillRunning;
            }
        }
        finished_ = true;
        return true;
    }
    return false;
}

bool SteamVrCloser::allGone() const {
    return std::all_of(items_.begin(), items_.end(),
                       [](const Item& i) { return i.outcome == Outcome::Closed || i.outcome == Outcome::Killed; });
}

QString SteamVrCloser::report() const {
    if (items_.empty()) {
        return QStringLiteral("Nothing to close.");
    }
    QStringList parts;
    QString closed = pidList(items_, Outcome::Closed, false);
    const QString killed = pidList(items_, Outcome::Killed, true);
    if (!killed.isEmpty()) {
        closed = closed.isEmpty() ? killed : closed + QStringLiteral(", ") + killed;
    }
    if (!closed.isEmpty()) {
        parts << QStringLiteral("Closed: %1.").arg(closed);
    }
    if (const QString s = pidList(items_, Outcome::StillRunning, false); !s.isEmpty()) {
        parts << QStringLiteral("Still running: %1.").arg(s);
    }
    if (const QString f = pidList(items_, Outcome::Failed, false); !f.isEmpty()) {
        parts << QStringLiteral("Could not close: %1.").arg(f);
    }
    if (allGone()) {
        parts << QStringLiteral("SteamVR can be started again.");
    }
    return parts.join(QLatin1Char(' '));
}

// ---------------------------------------------------------------- restarting Steam

SteamRestarter::SteamRestarter(Runner& runner, std::function<bool()> steamRunning)
    : runner_(runner), steamRunning_(std::move(steamRunning)) {}

void SteamRestarter::begin(qint64 nowMs) {
    waitFromMs_ = nowMs;
    if (!steamRunning_()) {
        start();
        return;
    }
    state_ = State::ShuttingDown;
    message_ = QStringLiteral("Closing Steam...");
    runner_.run(QStringLiteral("steam"), {QStringLiteral("-shutdown")}, [this](int, const QString&) {
        if (state_ == State::ShuttingDown) {
            state_ = State::Waiting; // the exit code says little: Steam closes in the background
        }
    });
}

void SteamRestarter::start() {
    if (runner_.startDetached(QStringLiteral("steam"), {})) {
        state_ = State::Started;
        message_ = QStringLiteral("Steam is starting again. Start SteamVR once its window is back.");
    } else {
        state_ = State::Failed;
        message_ = QStringLiteral("Could not start Steam. Is it installed?");
    }
}

bool SteamRestarter::step(qint64 nowMs) {
    if (finished() || state_ == State::Idle) {
        return finished();
    }
    if (state_ == State::Waiting && !steamRunning_()) {
        start();
        return true;
    }
    if (nowMs - waitFromMs_ >= kSteamExitWaitMs) {
        state_ = State::Failed;
        message_ = QStringLiteral("Steam did not close within 20 s. Close it from its menu (Steam, Exit) and start it "
                                  "again.");
        return true;
    }
    return false;
}

} // namespace pibridge::app
