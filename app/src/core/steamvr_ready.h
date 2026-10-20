// SPDX-License-Identifier: MIT
// "SteamVR fully ready": the moment an own app (Apps page) may start. vrserver running is not enough (a game started
// then finds no headset or no compositor and quits or renders to nothing). Every step must hold, in this order:
//   1. vrserver runs and the app's background OpenVR connection (core/steamvr.h, the same one the dashboard uses;
//      no second connection) is up: the server takes clients;
//   2. the headset (device 0, class HMD) is connected, its pose valid with TrackingResult_Running_OK;
//   3. the compositor runs: IVRApplications::GetApplicationProcessId("openvr.component.vrcompositor") != 0 through
//      that connection, or, when IVRApplications is not available, a vrcompositor process in /proc;
//   4. (optional) the last SteamVRSystemState_ transition in ~/.local/share/Steam/logs/vrmonitor.txt is not one of
//      a start still in progress (Startup, Connecting, NotReady, Restart); a missing or stale log does not block;
//   5. all of the above held without a break for the settle time (1.5 s).
// waitReady() gives up after its timeout (60 s for an app start) with the plain reason of the step that never held.
//
//   ReadyInputs      what is known now (built once a second from the app's state)
//   evaluateReady()  the first step that does not hold (pure)
//   ReadyTracker     adds the settle time (injected clock)
//   SteamVrReady     the Qt side: the current readiness for the UI, waiters with timeouts
#pragma once

#include <QObject>
#include <QString>
#include <QTimer>
#include <functional>
#include <optional>
#include <vector>

#include "steamvr.h"

namespace pibridge::app {

struct AppState;

inline constexpr qint64 kReadySettleMs = 1500;
inline constexpr int kReadyTimeoutMs = 60000;

struct ReadyInputs {
    bool installed = true;          // a SteamVR runtime is registered (openvrpaths.vrpath names one)
    bool vrserverRunning = false;   // the app's /proc check
    SteamVrStatus link;             // the background connection's last status (devices, compositorPid)
    bool compositorProcess = false; // a vrcompositor process exists (asked only when link.compositorPid < 0)
    QString monitorState;           // the last SteamVRSystemState_ name in vrmonitor.txt ("Ready"), "" unknown
};

enum class ReadyStep {
    NotInstalled,      // no SteamVR runtime
    NoServer,          // vrserver does not run
    NoClients,         // vrserver runs, the background connection is not up yet
    NoHeadset,         // no HMD at device 0
    HeadsetNotTracked, // the HMD is there but its pose is not valid / not Running_OK
    NoCompositor,      // the compositor does not run
    MonitorNotReady,   // vrmonitor still reports a start in progress
    Settling,          // everything holds, the settle time has not passed yet
    Ready,
};

struct Readiness {
    ReadyStep step = ReadyStep::NoServer;
    QString words;  // the step in plain words for the UI ("Waiting for the headset...")
    QString detail; // more for a tooltip (the connection error, the tracking state)
    bool ready() const { return step == ReadyStep::Ready; }
};

// The first step that does not hold; `Settling` when all hold (the tracker turns it into Ready).
Readiness evaluateReady(const ReadyInputs& in);
// Why a wait ended at this step, in plain words ("SteamVR does not see the headset").
QString readyFailureReason(const Readiness& r);

// The last "SteamVRSystemState_<X>" a transition went *to* in the text ("Ready"), "" when there is none.
QString lastSystemState(QStringView logText);
// The same from the end of a log file (the last 64 KB), "" when the file is missing.
QString lastSystemStateOfFile(const QString& path);
// <first Steam root>/logs/vrmonitor.txt, "" without Steam.
QString vrmonitorLogPath();

// The inputs from the app's state: in fake mode only from the fake state (nothing is read); otherwise the
// vrcompositor /proc check only when the connection cannot tell, and vrmonitor.txt only while vrserver runs.
ReadyInputs readyInputsFrom(const AppState& s, bool fake);

class ReadyTracker {
public:
    using Clock = std::function<qint64()>; // monotonic ms
    explicit ReadyTracker(Clock clock, qint64 settleMs = kReadySettleMs);
    // Evaluate `in`; Ready only once every step has held for settleMs (a break starts the settle time again).
    Readiness update(const ReadyInputs& in);
    // Re-check the settle time with the last inputs.
    Readiness recheck();
    const Readiness& current() const { return current_; }
    qint64 now() const { return clock_(); }

private:
    Clock clock_;
    qint64 settleMs_;
    ReadyInputs last_;
    std::optional<qint64> allHeldSince_;
    Readiness current_;
};

class SteamVrReady : public QObject {
    Q_OBJECT
public:
    using Clock = ReadyTracker::Clock;
    using Callback = std::function<void(bool ready, const QString& reason)>;
    // `clock` empty: the steady clock.
    explicit SteamVrReady(Clock clock = {}, qint64 settleMs = kReadySettleMs, QObject* parent = nullptr);
    // New inputs (once a second from the state source).
    void setInputs(const ReadyInputs& in);
    // Re-check the settle time and the waiters' timeouts (a timer calls it every 250 ms while someone waits;
    // tests call it with their clock).
    void poll();
    const Readiness& current() const { return tracker_.current(); }
    // Calls `cb` once: (true, "") as soon as SteamVR is fully ready (at once when it is), or (false, reason) after
    // `timeoutMs` or at once when SteamVR is not installed.
    void waitReady(int timeoutMs, Callback cb);
    int waiting() const { return static_cast<int>(waiters_.size()); }
    // Ends every wait with (false, reason) (the page closes, the user pressed Stop).
    void cancelAll(const QString& reason);

signals:
    void changed();

private:
    struct Waiter {
        qint64 deadline;
        Callback cb;
    };
    void settleWaiters();

    ReadyTracker tracker_;
    std::vector<Waiter> waiters_;
    QTimer timer_;
    bool everReceived_ = false;
};

} // namespace pibridge::app
