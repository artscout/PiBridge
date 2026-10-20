// SPDX-License-Identifier: MIT
// "Reset SteamVR": find what is left of SteamVR, close exactly those processes, or restart Steam when Steam's own
// state is stuck.
//
// Two cases this is for (seen on the Crystal Super machine, 2026-10-19):
//  1. a half-started vrserver stayed alive for 1.5 h holding SteamVR's abstract socket @/steamvr/SteamVR_Namespace.
//     Its /proc comm was "36674: vrstartu", not "vrserver", so a comm check missed it, and every new SteamVR start
//     failed ("Unable to bind server socket errno=98"). It ignored SIGTERM and needed SIGKILL.
//  2. after steamwebhelper crashed, Steam said "SteamVR is already running" with no SteamVR process at all. Only a
//     Steam restart helps.
//
// So a process is found by where its program lives (exe link or argv[0] under the SteamVR runtime dir, or a wrapper
// whose command line names that dir), never by its name, and the holders of the @/steamvr/* sockets are named.
// Steam's own client (ubuntu12_32/, ubuntu12_64/: steam, steamwebhelper, reaper) is never matched. Only the PIDs
// that were listed to the user are signalled; a PID whose start time changed since the scan (reused) is left alone.
//
// Everything reads an injectable proc root and takes an injected clock and signal sender, so the tests run against a
// fake /proc tree and never signal a real process.
#pragma once

#include <QString>
#include <QStringList>
#include <functional>
#include <vector>

#include "runner.h"

namespace pibridge::app {

// ---------------------------------------------------------------- detection

struct SteamVrProc {
    int pid = 0;
    QString name;     // what the user reads: the program's file name ("vrserver", "vrcompositor", "steamtours")
    QString comm;     // /proc/<pid>/comm as it is ("36674: vrstartu")
    QString exe;      // readlink /proc/<pid>/exe ("" when not readable)
    QString cmdline;  // argv joined with spaces
    qint64 startTicks = 0;  // /proc/<pid>/stat field 22 (clock ticks after boot): the identity check before a signal
    qint64 runningS = -1;   // seconds since it started (-1 unknown)
    bool wrapper = false;   // matched only through its command line (pressure-vessel, a shell running vrstartup.sh)
    bool inRuntime = true;  // false: only listed because it holds a SteamVR socket; never signalled
    QStringList sockets;    // the @/steamvr/* socket paths it holds
};

struct SteamVrScan {
    QString runtimeDir;              // the SteamVR runtime dir the scan used
    std::vector<SteamVrProc> procs;  // sorted by PID
    QStringList orphanSockets;       // @/steamvr/* sockets no readable process holds
    bool steamRunning = false;       // Steam's client (comm "steam" or exe .../ubuntu12_32/steam)

    // The processes that would be signalled (inRuntime ones).
    std::vector<SteamVrProc> targets() const;
    // vrserver under its own name: a process named vrserver whose comm is "vrserver".
    bool vrserverNamedRight() const;
};

// The SteamVR runtime dir: runtime[0] of ~/.config/openvr/openvrpaths.vrpath (core/steamvr's lookup), else
// ~/.local/share/Steam/steamapps/common/SteamVR.
QString steamVrRuntimeDir();

// The abstract socket paths starting with "@/steamvr/" in a /proc/net/unix text, as inode -> path.
std::vector<std::pair<qint64, QString>> steamVrSockets(const QByteArray& procNetUnix);

struct ScanOptions {
    QString procRoot = QStringLiteral("/proc");
    QString runtimeDir;     // empty: steamVrRuntimeDir()
    long ticksPerS = 0;     // 0: sysconf(_SC_CLK_TCK)
    int selfPid = 0;        // 0: getpid(); never listed
};

SteamVrScan scanSteamVr(const ScanOptions& opt = {});

// Steam's client process, by comm "steam" or an exe ending in /ubuntu12_32/steam.
bool steamClientRunning(const QString& procRoot = QStringLiteral("/proc"));

// "1 h 24 min", "12 min", "40 s", "2 d 3 h"; "" for < 0.
QString runningWords(qint64 seconds);
// "vrserver (PID 36868, running 1 h 24 min, holds the SteamVR socket)".
QString procLine(const SteamVrProc& p);

// ---------------------------------------------------------------- the dialog's words

struct ResetDialogModel {
    enum class Mode { CloseSteamVr, RestartSteam } mode = Mode::CloseSteamVr;
    QString title;
    QString intro;      // one paragraph above the list
    QStringList lines;  // one per process (and the unheld sockets, the left-alone holders)
    QString question;   // the line above the buttons
    QString warning;    // shown in amber, may be empty
    QString confirm;    // the confirm button's text
};

// `steamVrConnected`: the app's OpenVR background connection is connected.
// Healthy = vrserver under its own name runs and the connection is connected.
ResetDialogModel resetDialogModel(const SteamVrScan& scan, bool steamVrConnected);

// ---------------------------------------------------------------- closing the listed processes

inline constexpr qint64 kTermWaitMs = 5000;  // SIGTERM, then SIGKILL to the ones alive after this
inline constexpr qint64 kKillWaitMs = 2000;  // after SIGKILL: how long to wait for them to vanish

struct ProcSignals {
    // kill(pid, sig): 0, or the errno.
    std::function<int(int pid, int sig)> send;
    // Is this process (pid with this start time) still there? (a zombie counts as gone)
    std::function<bool(int pid, qint64 startTicks)> alive;
};
// The real ones (::kill, /proc/<pid>/stat under `procRoot`).
ProcSignals realProcSignals(const QString& procRoot = QStringLiteral("/proc"));

class SteamVrCloser {
public:
    enum class Outcome { Pending, Closed, Killed, StillRunning, Failed };
    struct Item {
        SteamVrProc proc;
        Outcome outcome = Outcome::Pending;
        QString error;
    };

    SteamVrCloser(std::vector<SteamVrProc> targets, ProcSignals sig);
    // SIGTERM to every target that is still the same process.
    void begin(qint64 nowMs);
    // Checks who is gone; at kTermWaitMs SIGKILL to the survivors. True when finished.
    bool step(qint64 nowMs);
    bool finished() const { return finished_; }
    const std::vector<Item>& items() const { return items_; }
    // "Closed: vrcompositor (PID 2), vrserver (PID 1, needed SIGKILL)." ...
    QString report() const;
    bool allGone() const;

private:
    std::vector<Item> items_;
    ProcSignals sig_;
    qint64 startMs_ = 0;
    qint64 killMs_ = -1;
    bool finished_ = false;
};

// ---------------------------------------------------------------- restarting Steam

inline constexpr qint64 kSteamExitWaitMs = 20000;

class SteamRestarter {
public:
    enum class State { Idle, ShuttingDown, Waiting, Started, Failed };
    // `steamRunning`: the process check (steamClientRunning in the app).
    SteamRestarter(Runner& runner, std::function<bool()> steamRunning);
    // `steam -shutdown` (only when Steam runs), then wait for it to be gone.
    void begin(qint64 nowMs);
    // Starts `steam` detached once Steam is gone; fails after kSteamExitWaitMs. True when finished.
    bool step(qint64 nowMs);
    State state() const { return state_; }
    bool finished() const { return state_ == State::Started || state_ == State::Failed; }
    QString message() const { return message_; }

private:
    void start();
    Runner& runner_;
    std::function<bool()> steamRunning_;
    State state_ = State::Idle;
    qint64 waitFromMs_ = -1; // when `steam -shutdown` returned
    QString message_;
};

} // namespace pibridge::app
