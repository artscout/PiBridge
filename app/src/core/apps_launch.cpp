// SPDX-License-Identifier: MIT
#include "apps_launch.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <chrono>
#include <csignal>
#include <unistd.h>

#include "apps_steam.h"
#include "steamvr_ready.h"

namespace pibridge::app {

QString expandHome(const QString& path, const QString& home) {
    const QString h = home.isEmpty() ? QDir::homePath() : home;
    if (path == QLatin1String("~")) {
        return h;
    }
    if (path.startsWith(QLatin1String("~/"))) {
        return h + path.mid(1);
    }
    return path;
}

LaunchSpec ownAppSpec(const OwnApp& a, const QString& logDir, const QDateTime& utc, const QString& home) {
    LaunchSpec s;
    s.program = expandHome(a.exec.trimmed(), home);
    s.args = a.args;
    s.workdir = expandHome(a.workdir.trimmed(), home);
    if (s.workdir.isEmpty()) {
        s.workdir = QFileInfo(s.program).absolutePath();
    }
    for (const QString& e : a.env) {
        if (!e.trimmed().isEmpty()) {
            s.env << e.trimmed();
        }
    }
    if (!a.quadViews) {
        s.env << QString::fromLatin1(kQuadViewsOffVar) + QStringLiteral("=1");
    }
    const QString id = a.id.isEmpty() ? appSlug(a.name) : a.id;
    s.logPath = QStringLiteral("%1/%2-%3.log").arg(logDir, id, utc.toUTC().toString(QStringLiteral("yyyyMMdd-HHmmss'Z'")));
    s.logLink = QStringLiteral("%1/%2-latest.log").arg(logDir, id);
    return s;
}

bool prepareLog(const LaunchSpec& s, QString* error) {
    const QString dir = QFileInfo(s.logPath).absolutePath();
    if (!QDir().mkpath(dir)) {
        if (error) {
            *error = QStringLiteral("Could not create %1").arg(dir);
        }
        return false;
    }
    QFile f(s.logPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Append)) {
        if (error) {
            *error = QStringLiteral("Could not write %1").arg(s.logPath);
        }
        return false;
    }
    const QString head = QStringLiteral("# PiBridge started: %1 %2\n# in: %3\n")
                             .arg(s.program, s.args.join(QLatin1Char(' ')), s.workdir);
    f.write(head.toUtf8());
    f.close();
    // the -latest link: replace only a link (never a real file someone put there)
    const QFileInfo link(s.logLink);
    if (link.isSymLink() || !link.exists()) {
        QFile::remove(s.logLink);
        QFile::link(QFileInfo(s.logPath).fileName(), s.logLink); // relative, so the folder can move
    }
    return true;
}

// ---------------------------------------------------------------- QProcess

QtAppProcesses::~QtAppProcesses() {
    // PiBridge closes, the games stay: QProcess's destructor would kill them, so the objects are let go of.
    for (auto& [pid, p] : procs_) {
        p->disconnect();
        p->setParent(nullptr);
        (void)pid;
    }
    procs_.clear();
}

qint64 QtAppProcesses::start(const LaunchSpec& s, Exited onExit, QString* error) {
    QString err;
    if (!prepareLog(s, &err)) {
        if (error) {
            *error = err;
        }
        return 0;
    }
    auto* p = new QProcess(this);
    p->setProgram(s.program);
    p->setArguments(s.args);
    p->setWorkingDirectory(s.workdir);
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    for (const QString& kv : s.env) {
        const qsizetype eq = kv.indexOf(QLatin1Char('='));
        if (eq > 0) {
            env.insert(kv.left(eq), kv.mid(eq + 1));
        }
    }
    p->setProcessEnvironment(env);
    p->setStandardInputFile(QProcess::nullDevice());
    p->setProcessChannelMode(QProcess::MergedChannels);
    p->setStandardOutputFile(s.logPath, QIODevice::Append);
    // its own session: a terminal's Ctrl+C on PiBridge does not reach the game
    p->setChildProcessModifier([] { ::setsid(); });
    p->start();
    if (!p->waitForStarted(5000)) {
        if (error) {
            *error = QStringLiteral("Could not start %1: %2").arg(QFileInfo(s.program).fileName(), p->errorString());
        }
        QFile log(s.logPath);
        if (log.open(QIODevice::WriteOnly | QIODevice::Append)) {
            log.write(QStringLiteral("# could not start: %1\n").arg(p->errorString()).toUtf8());
        }
        p->deleteLater();
        return 0;
    }
    const qint64 pid = p->processId();
    procs_[pid] = p;
    connect(p, &QProcess::finished, this, [this, p, pid, onExit](int code, QProcess::ExitStatus st) {
        procs_.erase(pid);
        p->deleteLater();
        if (onExit) {
            onExit(code, st != QProcess::NormalExit);
        }
    });
    return pid;
}

bool QtAppProcesses::stop(qint64 pid) {
    const auto it = procs_.find(pid);
    if (it == procs_.end() || it->second->state() != QProcess::Running || it->second->processId() != pid) {
        return false;
    }
    return ::kill(static_cast<pid_t>(pid), SIGTERM) == 0; // that PID only
}

// ---------------------------------------------------------------- fake

qint64 FakeAppProcesses::start(const LaunchSpec& s, Exited onExit, QString* error) {
    if (!failWith.isEmpty()) {
        if (error) {
            *error = failWith;
        }
        return 0;
    }
    started.push_back(s);
    const qint64 pid = nextPid++;
    running_[pid] = std::move(onExit);
    return pid;
}

bool FakeAppProcesses::stop(qint64 pid) {
    if (!running_.contains(pid)) {
        return false;
    }
    stopped.push_back(pid);
    return true;
}

void FakeAppProcesses::exit(qint64 pid, int code, bool crashed) {
    const auto it = running_.find(pid);
    if (it == running_.end()) {
        return;
    }
    Exited cb = std::move(it->second);
    running_.erase(it);
    if (cb) {
        cb(code, crashed);
    }
}

// ---------------------------------------------------------------- words

ChipText appChip(const AppRunState& s, bool runtimeReady) {
    switch (s.run) {
    case AppRun::Running: return {QStringLiteral("RUNNING"), ChipKind::Info};
    case AppRun::StartingSteamVr: return {QStringLiteral("STARTING STEAMVR"), ChipKind::Warn};
    case AppRun::StartingTracking: return {QStringLiteral("STARTING TRACKING"), ChipKind::Warn};
    case AppRun::Starting: return {QStringLiteral("STARTING"), ChipKind::Warn};
    case AppRun::Error: return {QStringLiteral("ERROR"), ChipKind::Error};
    case AppRun::Idle:
    case AppRun::Exited: break;
    }
    return runtimeReady ? ChipText{QStringLiteral("READY"), ChipKind::Off} : ChipText{QString(), ChipKind::Off};
}

QString appRunWords(const AppRunState& s) {
    switch (s.run) {
    case AppRun::Exited:
        return s.crashed ? QStringLiteral("Crashed") : QStringLiteral("Exited (code %1)").arg(s.exitCode);
    case AppRun::Error: return s.message;
    case AppRun::Running: return s.pid > 0 ? QStringLiteral("PID %1").arg(s.pid) : QString();
    case AppRun::StartingSteamVr:
    case AppRun::StartingTracking: return s.message;
    case AppRun::Idle:
    case AppRun::Starting: break;
    }
    return {};
}

// ---------------------------------------------------------------- launcher

namespace {
qint64 steadyMs() {
    return static_cast<qint64>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}
} // namespace

AppLauncher::AppLauncher(Runner& runner, AppProcesses& procs, SteamVrReady& ready, std::function<bool()> vrserverRunning,
                         QString logDir, Clock clock, QObject* parent)
    : QObject(parent), runner_(runner), procs_(procs), ready_(ready), vrserverRunning_(std::move(vrserverRunning)),
      logDir_(std::move(logDir)), clock_(clock ? std::move(clock) : Clock(&steadyMs)) {
    monadoTimer_.setInterval(250);
    connect(&monadoTimer_, &QTimer::timeout, this, &AppLauncher::pollMonado);
}

void AppLauncher::set(const QString& id, AppRunState s) {
    own_[id] = std::move(s);
    emit changed(id);
}

AppRunState AppLauncher::ownState(const QString& id) const {
    return own_.value(id);
}

void AppLauncher::setOwnStateForShow(const QString& id, const AppRunState& s) {
    set(id, s);
}

void AppLauncher::launchOwn(const OwnApp& a) {
    const AppRun now = own_.value(a.id).run;
    if (now == AppRun::Running || now == AppRun::Starting || now == AppRun::StartingSteamVr ||
        now == AppRun::StartingTracking) {
        return;
    }
    if (target() == AppTarget::Monado) {
        launchOnMonado(a); // never SteamVR in this mode
        return;
    }
    if (ready_.current().ready()) {
        startNow(a);
        return;
    }
    if (!vrserverRunning_()) {
        QString why;
        if (!startSteamVr(&why)) {
            AppRunState s;
            s.run = AppRun::Error;
            s.message = why;
            set(a.id, s);
            return;
        }
    }
    AppRunState s;
    s.run = AppRun::StartingSteamVr;
    s.message = ready_.current().words;
    set(a.id, s);
    const int token = ++waitToken_[a.id];
    ready_.waitReady(kReadyTimeoutMs, [this, a, token](bool ok, const QString& reason) {
        if (waitToken_.value(a.id) != token || own_.value(a.id).run != AppRun::StartingSteamVr) {
            return; // stopped by the user meanwhile
        }
        if (ok) {
            startNow(a);
            return;
        }
        AppRunState e;
        e.run = AppRun::Error;
        e.message = reason.isEmpty() ? QStringLiteral("SteamVR did not become ready") : reason;
        e.message[0] = e.message[0].toUpper();
        set(a.id, e);
    });
}

bool AppLauncher::startSteamVr(QString* why) {
    if (runtime_ && runtime_->steamVrThroughTracking()) {
        // inside-out: SteamVR comes with START TRACKING (driver_monado on, CustomHeadsetOpenVR off); a SteamVR started
        // through Steam would load the Lighthouse setup's driver
        QString w;
        if (!runtime_->startTracking(&w)) {
            *why = w.isEmpty() ? QStringLiteral("Tracking could not start.") : QStringLiteral("Tracking could not start: %1").arg(w);
            return false;
        }
        return true;
    }
    if (!runner_.startDetached(QStringLiteral("steam"), steamRunArgs(QString::fromLatin1(kSteamVrAppId)))) {
        *why = QStringLiteral("Could not start Steam. Is it installed?");
        return false;
    }
    return true;
}

void AppLauncher::launchOnMonado(const OwnApp& a) {
    MonadoReadiness m = runtime_->monado();
    if (m.ready()) {
        startNow(a, m.runtimeJson);
        return;
    }
    if (m.step == MonadoStep::NoRuntimeFile) {
        AppRunState e;
        e.run = AppRun::Error;
        e.message = m.words;
        set(a.id, e);
        return;
    }
    if (m.step == MonadoStep::Off || m.step == MonadoStep::Failed) {
        QString why;
        if (!runtime_->startTracking(&why)) {
            AppRunState e;
            e.run = AppRun::Error;
            e.message = why.isEmpty() ? QStringLiteral("Tracking could not start.")
                                      : QStringLiteral("Tracking could not start: %1").arg(why);
            set(a.id, e);
            return;
        }
        m = runtime_->monado();
    }
    MonadoWait w;
    w.app = a;
    w.deadline = clock_() + kReadyTimeoutMs;
    w.sawStart = m.step != MonadoStep::Off && m.step != MonadoStep::Failed;
    monadoWait_.insert(a.id, w);
    AppRunState s;
    s.run = AppRun::StartingTracking;
    s.message = m.words;
    set(a.id, s);
    if (QCoreApplication::instance() && !monadoTimer_.isActive()) {
        monadoTimer_.start();
    }
}

void AppLauncher::pollMonado() {
    if (monadoWait_.isEmpty() || !runtime_) {
        monadoTimer_.stop();
        return;
    }
    const MonadoReadiness m = runtime_->monado();
    const qint64 now = clock_();
    for (const QString& id : monadoWait_.keys()) {
        MonadoWait& w = monadoWait_[id];
        if (m.ready()) {
            const OwnApp a = w.app;
            monadoWait_.remove(id);
            startNow(a, m.runtimeJson);
            continue;
        }
        const bool started = m.step != MonadoStep::Off && m.step != MonadoStep::Failed;
        const bool ended = (m.step == MonadoStep::Failed || m.step == MonadoStep::NoRuntimeFile) ||
                           (w.sawStart && m.step == MonadoStep::Off);
        w.sawStart = w.sawStart || started;
        if (ended || now >= w.deadline) {
            monadoWait_.remove(id);
            AppRunState e;
            e.run = AppRun::Error;
            e.message = ended ? m.words
                              : QStringLiteral("Monado was not ready after %1 s: %2").arg(kReadyTimeoutMs / 1000).arg(m.words);
            set(id, e);
            continue;
        }
        AppRunState s = own_.value(id);
        if (s.message != m.words) {
            s.message = m.words;
            set(id, s);
        }
    }
    if (monadoWait_.isEmpty()) {
        monadoTimer_.stop();
    }
}

void AppLauncher::startNow(const OwnApp& a, const QString& runtimeJson) {
    LaunchSpec spec = ownAppSpec(a, logDir_, QDateTime::currentDateTimeUtc());
    if (!runtimeJson.isEmpty()) {
        // this app only: the system's active runtime (SteamVR) stays as it is
        spec.env << QStringLiteral("XR_RUNTIME_JSON=") + runtimeJson;
    }
    QString err;
    const QString id = a.id;
    const qint64 pid = procs_.start(
        spec,
        [this, id](int code, bool crashed) {
            AppRunState s = own_.value(id);
            s.run = AppRun::Exited;
            s.exitCode = code;
            s.crashed = crashed;
            s.pid = 0;
            set(id, s);
        },
        &err);
    AppRunState s;
    s.logPath = spec.logPath;
    if (pid <= 0) {
        s.run = AppRun::Error;
        s.message = err.isEmpty() ? QStringLiteral("Could not start the program.") : err;
        set(id, s);
        return;
    }
    s.run = AppRun::Running;
    s.pid = pid;
    set(id, s);
    emit played(id);
}

bool AppLauncher::stopOwn(const QString& id) {
    AppRunState s = own_.value(id);
    if (s.run == AppRun::StartingTracking) {
        monadoWait_.remove(id); // the Tracking session itself keeps running
        s.run = AppRun::Idle;
        s.message.clear();
        set(id, s);
        return true;
    }
    if (s.run == AppRun::StartingSteamVr) {
        waitToken_[id]++;
        s.run = AppRun::Idle;
        s.message.clear();
        set(id, s);
        return true;
    }
    if (s.run == AppRun::Running && s.pid > 0) {
        return procs_.stop(s.pid);
    }
    return false;
}

bool AppLauncher::launchSteam(const QString& appId, bool vr) {
    if (steamWait_.contains(appId)) {
        return true; // already waiting for SteamVR
    }
    if (const QString why = steamBlockedWhy(target()); !why.isEmpty()) {
        steamError_[appId] = why; // nothing is started: no OpenVR-to-OpenXR layer for Monado here
        emit changed(appId);
        return false;
    }
    if (vr && !ready_.current().ready()) {
        if (QString why; !vrserverRunning_() && !startSteamVr(&why)) {
            steamError_[appId] = why;
            emit changed(appId);
            return false;
        }
        steamError_.remove(appId);
        const int token = steamWait_[appId] = ++waitToken_[appId];
        emit changed(appId);
        ready_.waitReady(kReadyTimeoutMs, [this, appId, token](bool ok, const QString& reason) {
            if (steamWait_.value(appId, -1) != token) {
                return; // cancelled meanwhile
            }
            steamWait_.remove(appId);
            if (ok) {
                launchSteam(appId, false);
                return;
            }
            QString m = reason.isEmpty() ? QStringLiteral("SteamVR did not become ready") : reason;
            m[0] = m[0].toUpper();
            steamError_[appId] = m;
            emit changed(appId);
        });
        return true;
    }
    if (!runner_.startDetached(QStringLiteral("steam"), steamRunArgs(appId))) {
        steamError_[appId] = QStringLiteral("Could not start Steam. Is it installed?");
        steamStarted_.remove(appId);
        emit changed(appId);
        return false;
    }
    steamError_.remove(appId);
    steamStarted_[appId] = clock_();
    emit changed(appId);
    emit played(appId);
    return true;
}

bool AppLauncher::cancelSteam(const QString& appId) {
    if (!steamWait_.remove(appId)) {
        return false;
    }
    emit changed(appId);
    return true;
}

AppRunState AppLauncher::steamState(const QString& appId, const QSet<QString>& running) const {
    AppRunState s;
    if (steamWait_.contains(appId)) {
        s.run = AppRun::StartingSteamVr;
        s.message = ready_.current().words;
        return s;
    }
    if (running.contains(appId)) {
        steamStarted_.remove(appId);
        s.run = AppRun::Running;
        return s;
    }
    if (steamError_.contains(appId)) {
        s.run = AppRun::Error;
        s.message = steamError_.value(appId);
        return s;
    }
    if (const auto it = steamStarted_.constFind(appId); it != steamStarted_.cend() && clock_() - *it < kSteamStartShowMs) {
        s.run = AppRun::Starting;
    }
    return s;
}

} // namespace pibridge::app
