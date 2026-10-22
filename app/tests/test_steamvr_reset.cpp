// SPDX-License-Identifier: MIT
// Tests of "Reset SteamVR" (core/steamvr_reset.*): detection against a fake /proc tree in a temporary dir, the
// SIGTERM-then-SIGKILL sequence with a fake signal sender and an injected clock, the dialog's words, and the Steam
// restart with the FakeRunner. No real process is ever signalled or started.
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <csignal>
#include <map>
#include <set>

#include "core/runner.h"
#include "core/steamvr_reset.h"
#include "pibridge_test.h"

using namespace pibridge::app;

namespace {

const QString kRt = QStringLiteral("/home/u/.local/share/Steam/steamapps/common/SteamVR");
const QString kSteam = QStringLiteral("/home/u/.local/share/Steam");
constexpr long kTps = 100;
constexpr double kUptime = 10000.0;

void writeAll(const QString& p, const QByteArray& b) {
    QDir().mkpath(QFileInfo(p).absolutePath());
    QFile f(p);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(b);
    }
}

// A /proc tree: proc(pid, ...) writes comm, cmdline, stat and the exe and fd links.
struct FakeProc {
    QTemporaryDir dir;
    QString root() const { return dir.path(); }

    FakeProc() {
        writeAll(root() + QStringLiteral("/uptime"), QByteArray::number(kUptime, 'f', 2) + " 5000.00\n");
    }
    void netUnix(const QByteArray& body) {
        writeAll(root() + QStringLiteral("/net/unix"),
                 "Num       RefCount Protocol Flags    Type St Inode Path\n" + body);
    }
    // runningS: seconds since start (start ticks from the uptime); exe empty = no exe link (not readable)
    void proc(int pid, const QString& comm, const QString& exe, const QStringList& args, qint64 runningS,
              const std::vector<qint64>& socketInodes = {}, char state = 'S') {
        const QString d = root() + QLatin1Char('/') + QString::number(pid);
        QDir().mkpath(d + QStringLiteral("/fd"));
        writeAll(d + QStringLiteral("/comm"), comm.toLocal8Bit() + '\n');
        QByteArray cmd;
        for (const QString& a : args) {
            cmd += a.toLocal8Bit() + '\0';
        }
        writeAll(d + QStringLiteral("/cmdline"), cmd);
        const qint64 start = static_cast<qint64>((kUptime - double(runningS)) * kTps);
        QByteArray stat = QByteArray::number(pid) + " (" + comm.toLocal8Bit() + ") " + state;
        for (int field = 4; field <= 21; field++) {
            stat += " 0";
        }
        stat += ' ' + QByteArray::number(start) + " 0 0 0\n";
        writeAll(d + QStringLiteral("/stat"), stat);
        if (!exe.isEmpty()) {
            QFile::link(exe, d + QStringLiteral("/exe"));
        }
        int fd = 3;
        QFile::link(QStringLiteral("/dev/null"), d + QStringLiteral("/fd/0"));
        for (qint64 inode : socketInodes) {
            QFile::link(QStringLiteral("socket:[%1]").arg(inode), d + QStringLiteral("/fd/%1").arg(fd++));
        }
    }
    SteamVrScan scan(int self = 999999) const {
        ScanOptions o;
        o.procRoot = root();
        o.runtimeDir = kRt;
        o.ticksPerS = kTps;
        o.selfPid = self;
        return scanSteamVr(o);
    }
};

// The machine of 2026-10-19: Steam, a stuck vrserver under an odd comm holding the socket, and everything around it.
void machine(FakeProc& p) {
    p.netUnix("0000000000000000: 00000002 00000000 00010000 0001 01 111 @/steamvr/SteamVR_Namespace\n"
           "0000000000000000: 00000003 00000000 00000000 0001 03 112 @/steamvr/steam_client\n"
           "0000000000000000: 00000003 00000000 00000000 0001 03 113 @/tmp/.X11-unix/X0\n"
           "0000000000000000: 00000002 00000000 00010000 0001 01 114 @/steamvr/nobody\n"
           "0000000000000000: 00000003 00000000 00000000 0001 03 115 @/steamvr/other\n"
           "0000000000000000: 00000003 00000000 00000000 0001 03 116\n");
    p.proc(100, QStringLiteral("steam"), kSteam + QStringLiteral("/ubuntu12_32/steam"),
           {kSteam + QStringLiteral("/ubuntu12_32/steam"), QStringLiteral("-srt-logger-opened")}, 9000, {112});
    p.proc(101, QStringLiteral("steamwebhelper"), kSteam + QStringLiteral("/ubuntu12_64/steamwebhelper"),
           {QStringLiteral("./steamwebhelper"), QStringLiteral("--vr=") + kRt + QStringLiteral("/bin/x")}, 8000);
    p.proc(102, QStringLiteral("reaper"), kSteam + QStringLiteral("/ubuntu12_32/reaper"),
           {kSteam + QStringLiteral("/ubuntu12_32/reaper"), QStringLiteral("SteamLaunch"), QStringLiteral("AppId=250820"),
            QStringLiteral("--"), kRt + QStringLiteral("/bin/vrstartup.sh")}, 5100);
    // the stuck one: 1 h 24 min, comm "36674: vrstartu", holds the socket
    p.proc(200, QStringLiteral("36674: vrstartu"), kRt + QStringLiteral("/bin/linux64/vrserver"),
           {kRt + QStringLiteral("/bin/linux64/vrserver"), QStringLiteral("--keepalive")}, 5040, {111});
    // exe not readable: found through argv[0]
    p.proc(201, QStringLiteral("vrcompositor"), {}, {kRt + QStringLiteral("/bin/linux64/vrcompositor")}, 5030);
    p.proc(202, QStringLiteral("steamtours"),
           kRt + QStringLiteral("/tools/steamvr_environments/game/bin/linux64/steamtours"),
           {QStringLiteral("steamtours")}, 600);
    // pressure-vessel wrapper: only its command line names SteamVR
    p.proc(203, QStringLiteral("pressure-vessel"),
           kSteam + QStringLiteral("/steamapps/common/SteamLinuxRuntime_sniper/pressure-vessel/bin/pressure-vessel-wrap"),
           {QStringLiteral("pressure-vessel-wrap"), QStringLiteral("--"), kRt + QStringLiteral("/bin/vrstartup.sh")}, 5050);
    p.proc(204, QStringLiteral("bash"), QStringLiteral("/usr/bin/bash"), {QStringLiteral("bash")}, 100);
    // a look-alike dir next to SteamVR
    p.proc(205, QStringLiteral("vrserver"), kRt + QStringLiteral("2/bin/linux64/vrserver"),
           {kRt + QStringLiteral("2/bin/linux64/vrserver")}, 50);
    // holds a SteamVR socket but lives elsewhere: named, never a target
    p.proc(206, QStringLiteral("someclient"), QStringLiteral("/usr/bin/someclient"), {QStringLiteral("someclient")}, 30,
           {115});
    p.proc(207, QStringLiteral("vrwebhelper"), kRt + QStringLiteral("/bin/vrwebhelper/linux64/vrwebhelper"),
           {QStringLiteral("./vrwebhelper")}, 5000);
    // SteamVR was updated while it ran
    p.proc(208, QStringLiteral("vrmonitor"), kRt + QStringLiteral("/bin/linux64/vrmonitor (deleted)"),
           {QStringLiteral("vrmonitor")}, 70);
    // the app itself would never be under SteamVR, but the own PID is skipped anyway
    p.proc(4242, QStringLiteral("vrserver"), kRt + QStringLiteral("/bin/linux64/vrserver"), {}, 10);
}

const SteamVrProc* find(const SteamVrScan& s, int pid) {
    for (const auto& p : s.procs) {
        if (p.pid == pid) {
            return &p;
        }
    }
    return nullptr;
}

} // namespace

// ---------------------------------------------------------------- detection

TEST(sockets_from_proc_net_unix) {
    const auto s = steamVrSockets("Num       RefCount Protocol Flags    Type St Inode Path\n"
                                  "0000000000000000: 00000002 00000000 00010000 0001 01 111 @/steamvr/SteamVR_Namespace\n"
                                  "0000000000000000: 00000003 00000000 00000000 0001 03 113 @/tmp/.X11-unix/X0\n"
                                  "0000000000000000: 00000003 00000000 00000000 0001 03 116\n"
                                  "0000000000000000: 00000003 00000000 00000000 0001 03 117 @/steamvr/with space\n"
                                  "garbage\n");
    CHECK(s.size() == 2);
    CHECK(s[0].first == 111 && s[0].second == QStringLiteral("@/steamvr/SteamVR_Namespace"));
    CHECK(s[1].first == 117 && s[1].second == QStringLiteral("@/steamvr/with space"));
    CHECK(steamVrSockets({}).empty());
}

TEST(scan_finds_steamvr_by_path_not_name) {
    FakeProc p;
    CHECK(p.dir.isValid());
    machine(p);
    const SteamVrScan s = p.scan(4242);
    CHECK(s.runtimeDir == kRt);
    CHECK(s.steamRunning);
    std::set<int> pids;
    for (const auto& x : s.procs) {
        pids.insert(x.pid);
    }
    CHECK((pids == std::set<int>{200, 201, 202, 203, 206, 207, 208}));
    // Steam, steamwebhelper, reaper (its command line names SteamVR), bash, the look-alike dir, the own PID: never
    for (int never : {100, 101, 102, 204, 205, 4242}) {
        CHECK(find(s, never) == nullptr);
    }
    const SteamVrProc* vs = find(s, 200);
    CHECK(vs && vs->name == QStringLiteral("vrserver"));
    CHECK(vs && vs->comm == QStringLiteral("36674: vrstartu"));
    CHECK(vs && vs->runningS == 5040);
    CHECK(vs && vs->sockets == QStringList{QStringLiteral("@/steamvr/SteamVR_Namespace")});
    CHECK(vs && vs->inRuntime && !vs->wrapper);
    CHECK(vs && vs->startTicks == static_cast<qint64>((kUptime - 5040) * kTps));
    const SteamVrProc* comp = find(s, 201);
    CHECK(comp && comp->name == QStringLiteral("vrcompositor") && comp->exe.isEmpty());
    const SteamVrProc* tours = find(s, 202);
    CHECK(tours && tours->name == QStringLiteral("steamtours"));
    const SteamVrProc* pv = find(s, 203);
    CHECK(pv && pv->wrapper && pv->inRuntime && pv->name == QStringLiteral("pressure-vessel-wrap"));
    const SteamVrProc* other = find(s, 206);
    CHECK(other && !other->inRuntime && other->sockets == QStringList{QStringLiteral("@/steamvr/other")});
    const SteamVrProc* web = find(s, 207);
    CHECK(web && web->name == QStringLiteral("vrwebhelper"));
    const SteamVrProc* mon = find(s, 208);
    CHECK(mon && mon->name == QStringLiteral("vrmonitor") && mon->exe == kRt + QStringLiteral("/bin/linux64/vrmonitor"));
    // held by Steam (112) is not an orphan; 114 has no holder
    CHECK(s.orphanSockets == QStringList{QStringLiteral("@/steamvr/nobody")});
    // targets: everything but the outside holder
    std::set<int> t;
    for (const auto& x : s.targets()) {
        t.insert(x.pid);
    }
    CHECK((t == std::set<int>{200, 201, 202, 203, 207, 208}));
    // the vrserver runs under an odd name: not "named right"
    CHECK(!s.vrserverNamedRight());
}

TEST(scan_healthy_and_empty) {
    FakeProc p;
    p.netUnix("0000000000000000: 00000002 00000000 00010000 0001 01 111 @/steamvr/SteamVR_Namespace\n");
    p.proc(300, QStringLiteral("vrserver"), kRt + QStringLiteral("/bin/linux64/vrserver"), {}, 60, {111});
    const SteamVrScan s = p.scan();
    CHECK(s.procs.size() == 1);
    CHECK(s.vrserverNamedRight());
    CHECK(s.orphanSockets.isEmpty());
    CHECK(!s.steamRunning);

    FakeProc empty;
    const SteamVrScan e = empty.scan();
    CHECK(e.procs.empty() && e.orphanSockets.isEmpty() && !e.steamRunning);
}

TEST(steam_client_running) {
    FakeProc p;
    CHECK(!steamClientRunning(p.root()));
    p.proc(100, QStringLiteral("steamwebhelper"), kSteam + QStringLiteral("/ubuntu12_64/steamwebhelper"), {}, 10);
    CHECK(!steamClientRunning(p.root()));
    p.proc(101, QStringLiteral("steam"), kSteam + QStringLiteral("/ubuntu12_32/steam"), {}, 10, {}, 'Z');
    CHECK(!steamClientRunning(p.root())); // a zombie is gone
    p.proc(102, QStringLiteral("odd"), kSteam + QStringLiteral("/ubuntu12_32/steam"), {}, 10);
    CHECK(steamClientRunning(p.root()));
}

TEST(running_words_and_lines) {
    CHECK(runningWords(-1).isEmpty());
    CHECK(runningWords(40) == QStringLiteral("40 s"));
    CHECK(runningWords(12 * 60 + 5) == QStringLiteral("12 min"));
    CHECK(runningWords(5040) == QStringLiteral("1 h 24 min"));
    CHECK(runningWords(7200) == QStringLiteral("2 h"));
    CHECK(runningWords(2 * 86400 + 3 * 3600 + 7) == QStringLiteral("2 d 3 h"));

    SteamVrProc v;
    v.pid = 36868;
    v.name = v.comm = QStringLiteral("vrserver");
    v.runningS = 5040;
    v.sockets = {QStringLiteral("@/steamvr/SteamVR_Namespace")};
    CHECK(procLine(v) == QStringLiteral("vrserver (PID 36868, running 1 h 24 min, holds the SteamVR socket)"));
    v.comm = QStringLiteral("36674: vrstartu");
    CHECK(procLine(v) ==
          QStringLiteral("vrserver (PID 36868, running 1 h 24 min, shows as \"36674: vrstartu\", holds the SteamVR socket)"));
    SteamVrProc w;
    w.pid = 5;
    w.name = QStringLiteral("pressure-vessel-wrap");
    w.comm = QStringLiteral("pressure-vessel");
    w.wrapper = true;
    CHECK(procLine(w) == QStringLiteral("pressure-vessel-wrap (PID 5, starts SteamVR)")); // comm = name cut at 15
    SteamVrProc o;
    o.pid = 6;
    o.name = o.comm = QStringLiteral("someclient");
    o.inRuntime = false;
    o.sockets = {QStringLiteral("@/steamvr/other")};
    CHECK(procLine(o) == QStringLiteral("someclient (PID 6, holds a SteamVR socket, not part of SteamVR: left alone)"));
}

// ---------------------------------------------------------------- dialog words

TEST(dialog_stuck_vrserver) {
    FakeProc p;
    machine(p);
    const ResetDialogModel m = resetDialogModel(p.scan(4242), false);
    CHECK(m.mode == ResetDialogModel::Mode::CloseSteamVr);
    CHECK(m.title == QStringLiteral("Reset SteamVR"));
    CHECK(m.intro.startsWith(QStringLiteral("SteamVR is left over from an earlier start and holds SteamVR's socket")));
    CHECK(m.confirm == QStringLiteral("Restart SteamVR"));
    CHECK(m.lines.size() == 8); // 7 processes + the unheld socket
    CHECK(m.lines.contains(QStringLiteral(
        "vrserver (PID 200, running 1 h 24 min, shows as \"36674: vrstartu\", holds the SteamVR socket)")));
    CHECK(m.lines.last() == QStringLiteral("The socket @/steamvr/nobody is held by a process PiBridge cannot see."));
    CHECK(m.warning == QStringLiteral("Games running in VR will close."));
}

TEST(dialog_healthy_asks_anyway) {
    SteamVrScan s;
    SteamVrProc v;
    v.pid = 10;
    v.name = v.comm = QStringLiteral("vrserver");
    s.procs = {v};
    const ResetDialogModel h = resetDialogModel(s, true);
    CHECK(h.title == QStringLiteral("SteamVR looks fine"));
    CHECK(h.question == QStringLiteral("Restart SteamVR anyway?"));
    CHECK(h.mode == ResetDialogModel::Mode::CloseSteamVr);
    // vrserver named right but the connection is not up: not healthy
    const ResetDialogModel n = resetDialogModel(s, false);
    CHECK(n.title == QStringLiteral("Reset SteamVR"));
    CHECK(n.intro == QStringLiteral("These SteamVR processes are running:"));
}

TEST(dialog_nothing_running_offers_steam_restart) {
    SteamVrScan s;
    s.steamRunning = true;
    const ResetDialogModel m = resetDialogModel(s, false);
    CHECK(m.mode == ResetDialogModel::Mode::RestartSteam);
    CHECK(m.intro == QStringLiteral("No SteamVR process is running. If Steam still says SteamVR is running, Steam's own "
                                    "state is stuck: restart Steam."));
    CHECK(m.confirm == QStringLiteral("Restart Steam"));
    CHECK(m.warning == QStringLiteral("Games started from Steam will close."));
    CHECK(m.lines.isEmpty());
    s.steamRunning = false;
    const ResetDialogModel off = resetDialogModel(s, false);
    CHECK(off.confirm == QStringLiteral("Start Steam"));
    // only a holder outside SteamVR: nothing to close, so the Steam restart, with the holder named
    SteamVrProc o;
    o.pid = 6;
    o.name = o.comm = QStringLiteral("someclient");
    o.inRuntime = false;
    o.sockets = {QStringLiteral("@/steamvr/other")};
    s.procs = {o};
    const ResetDialogModel h = resetDialogModel(s, false);
    CHECK(h.mode == ResetDialogModel::Mode::RestartSteam && h.lines.size() == 1);
}

// ---------------------------------------------------------------- closing

namespace {

// Each pid: alive until the signal it obeys (SIGTERM, SIGKILL, or 0 = never dies); errors per pid.
struct FakeSignals {
    std::map<int, int> obeys;   // pid -> the signal that ends it
    std::map<int, bool> alive;  // pid -> alive
    std::map<int, int> error;   // pid -> errno answered
    std::vector<std::pair<int, int>> sent;
    ProcSignals make() {
        ProcSignals s;
        s.send = [this](int pid, int sig) {
            sent.emplace_back(pid, sig);
            if (error.count(pid)) {
                return error[pid];
            }
            if (obeys[pid] == sig || sig == SIGKILL) {
                if (obeys[pid] != 0) {
                    alive[pid] = false;
                }
            }
            return 0;
        };
        s.alive = [this](int pid, qint64) { return alive[pid]; };
        return s;
    }
};

SteamVrProc target(int pid, const QString& name) {
    SteamVrProc p;
    p.pid = pid;
    p.name = p.comm = name;
    return p;
}

} // namespace

TEST(close_term_then_kill_only_survivors) {
    FakeSignals f;
    f.obeys = {{1, SIGTERM}, {2, SIGKILL}};
    f.alive = {{1, true}, {2, true}};
    SteamVrProc outside = target(9, QStringLiteral("someclient"));
    outside.inRuntime = false;
    SteamVrCloser c({target(1, QStringLiteral("vrcompositor")), target(2, QStringLiteral("vrserver")), outside}, f.make());
    CHECK(c.items().size() == 2); // the outside holder is never a target
    c.begin(1000);
    CHECK((f.sent == std::vector<std::pair<int, int>>{{1, SIGTERM}, {2, SIGTERM}}));
    CHECK(!c.finished());
    CHECK(!c.step(1000 + kTermWaitMs - 1));
    CHECK(f.sent.size() == 2); // no SIGKILL before 5 s
    CHECK(!c.step(1000 + kTermWaitMs));
    CHECK((f.sent.back() == std::pair<int, int>{2, SIGKILL}));
    CHECK(f.sent.size() == 3); // SIGKILL only to the survivor
    CHECK(c.step(1000 + kTermWaitMs + 200));
    CHECK(c.items()[0].outcome == SteamVrCloser::Outcome::Closed);
    CHECK(c.items()[1].outcome == SteamVrCloser::Outcome::Killed);
    CHECK(c.allGone());
    CHECK(c.report() == QStringLiteral("Closed: vrcompositor (PID 1), vrserver (PID 2, needed a forced stop). SteamVR "
                                       "can be started again."));
}

TEST(close_all_obey_term_finish_early) {
    FakeSignals f;
    f.obeys = {{1, SIGTERM}};
    f.alive = {{1, true}};
    SteamVrCloser c({target(1, QStringLiteral("vrserver"))}, f.make());
    c.begin(0);
    CHECK(c.finished()); // gone right away: no waiting, no SIGKILL
    CHECK(f.sent.size() == 1);
}

TEST(close_gone_reused_denied_and_immortal) {
    FakeSignals f;
    f.obeys = {{3, 0}};
    f.alive = {{1, false}, {2, true}, {3, true}};
    f.error = {{2, EPERM}};
    SteamVrCloser c({target(1, QStringLiteral("vrmonitor")), target(2, QStringLiteral("vrserver")),
                     target(3, QStringLiteral("vrcompositor"))},
                    f.make());
    c.begin(0);
    // pid 1 is already gone (or someone else's now): never signalled
    for (const auto& [pid, sig] : f.sent) {
        CHECK(pid != 1);
    }
    CHECK(c.items()[0].outcome == SteamVrCloser::Outcome::Closed);
    CHECK(c.items()[1].outcome == SteamVrCloser::Outcome::Failed);
    CHECK(!c.step(kTermWaitMs));
    CHECK((f.sent.back() == std::pair<int, int>{3, SIGKILL}));
    CHECK(!c.step(kTermWaitMs + kKillWaitMs - 1));
    CHECK(c.step(kTermWaitMs + kKillWaitMs));
    CHECK(c.items()[2].outcome == SteamVrCloser::Outcome::StillRunning);
    CHECK(!c.allGone());
    CHECK(c.report() == QStringLiteral("Closed: vrmonitor (PID 1). Still running: vrcompositor (PID 3). Could not "
                                       "close: vrserver (PID 2, not allowed)."));
    // SIGKILL went only to pid 3
    int kills = 0;
    for (const auto& [pid, sig] : f.sent) {
        kills += sig == SIGKILL ? 1 : 0;
    }
    CHECK(kills == 1);
}

TEST(real_alive_reads_stat_identity) {
    // the real `alive` against the fake tree: same start time = alive; another start time (PID reused) = gone
    FakeProc p;
    p.proc(50, QStringLiteral("vrserver"), kRt + QStringLiteral("/bin/linux64/vrserver"), {}, 100);
    p.proc(51, QStringLiteral("vrserver"), {}, {}, 100, {}, 'Z');
    const ProcSignals s = realProcSignals(p.root());
    const qint64 ticks = static_cast<qint64>((kUptime - 100) * kTps);
    CHECK(s.alive(50, ticks));
    CHECK(!s.alive(50, ticks + 1));
    CHECK(!s.alive(51, ticks));
    CHECK(!s.alive(52, ticks));
}

// ---------------------------------------------------------------- restarting Steam

TEST(restart_steam_shutdown_wait_start) {
    FakeRunner r;
    bool running = true;
    SteamRestarter s(r, [&] { return running; });
    s.begin(0);
    CHECK(r.calls.size() == 1 && r.calls[0].line() == QStringLiteral("steam -shutdown") && !r.calls[0].detached);
    CHECK(s.state() == SteamRestarter::State::Waiting);
    CHECK(!s.step(5000));
    CHECK(r.calls.size() == 1); // not started while the old Steam is there
    running = false;
    CHECK(s.step(6000));
    CHECK(s.state() == SteamRestarter::State::Started);
    CHECK(r.calls.size() == 2 && r.calls[1].line() == QStringLiteral("steam") && r.calls[1].detached);
}

TEST(restart_steam_times_out) {
    FakeRunner r;
    SteamRestarter s(r, [] { return true; });
    s.begin(100);
    CHECK(!s.step(100 + kSteamExitWaitMs - 1));
    CHECK(s.step(100 + kSteamExitWaitMs));
    CHECK(s.state() == SteamRestarter::State::Failed);
    CHECK(s.message().startsWith(QStringLiteral("Steam did not close within 20 s")));
    CHECK(r.calls.size() == 1); // never started a second Steam
}

TEST(restart_steam_shutdown_still_running_times_out) {
    FakeRunner r;
    r.answer = false; // `steam -shutdown` never returns
    bool running = true;
    SteamRestarter s(r, [&] { return running; });
    s.begin(0);
    CHECK(s.state() == SteamRestarter::State::ShuttingDown);
    running = false;
    CHECK(!s.step(1000)); // waits for the command first
    CHECK(s.step(kSteamExitWaitMs));
    CHECK(s.state() == SteamRestarter::State::Failed);
}

TEST(restart_steam_not_running_just_starts) {
    FakeRunner r;
    SteamRestarter s(r, [] { return false; });
    s.begin(0);
    CHECK(s.finished() && s.state() == SteamRestarter::State::Started);
    CHECK(r.calls.size() == 1 && r.calls[0].line() == QStringLiteral("steam") && r.calls[0].detached);
}

PIBRIDGE_TEST_MAIN()
