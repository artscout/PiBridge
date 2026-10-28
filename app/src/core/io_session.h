// SPDX-License-Identifier: MIT
// The inside-out sessions. Both are the recorder's session (core/rec_session.h): the same start order, PID-exact stop
// and logs, without recording. Two variants, one class:
//
//   SteamVR + Monado  (IoVariant::SteamVr; START TRACKING on the Tracking page and the Dashboard, the inside-out mode):
//     the camera broker, the controller tracker, then SteamVR (vrstartup.sh of the runtime in openvrpaths.vrpath) with
//     driver_monado.so of the chosen Monado build running the Pimax driver inside vrserver. Before anything starts:
//     the driver settings file (PIMAX_STEAMVR_ENV_FILE) is written and SteamVR's drivers are switched for the session
//     (core/steamvr_drivers.h: the Monado build's steamvr-monado registered, CustomHeadsetOpenVR switched off; backups
//     and a state file first). It runs once vrserver runs and SteamVR reports the headset. Games start through
//     SteamVR as usual. Stop: SteamVR is closed PID-exactly (core/steamvr_reset.h's closer, only the SteamVR processes
//     that started after this start's launch), then the tracker and the broker, then the drivers are put back. When
//     the user quits SteamVR, the rest stops the same way. A restore that cannot run yet (SteamVR still there, or
//     PiBridge crashed) runs as soon as vrserver is gone, also at PiBridge's next start.
//
//   Monado directly  (IoVariant::MonadoDirect; the Development page's debug / A-B start): monado-service owns the
//     headset (the display lease retry once after 4 s); own apps on the Apps page start on it (XR_RUNTIME_JSON) while
//     it runs, and the home scene: once Monado runs, the session starts pibridge-home (tools/home: a floor grid, the
//     horizon, the room and the controllers in the headset, so the headset does not stay black) with XR_RUNTIME_JSON
//     pointing at Monado, when io_home_scene is on and pibridge-home is found. It is started once per session and never
//     again (a home that came back while a game runs would take the headset over); Stop sends SIGTERM to exactly its
//     PID first. Apps wait until it has its session (home.json says so) or kHomeSettleMs passed, so a game always
//     starts after it and is the one Monado shows.
//
// The conf key io_apps (steamvr | monado) is still read but no longer chooses anything: a pibridge.conf that says
// "monado" gets the SteamVR + Monado start on the Tracking page; Monado directly is started on the Development page.
#pragma once

#include <QObject>
#include <QTimer>
#include <expected>
#include <functional>
#include <memory>

#include "apps_runtime.h"
#include "conf.h"
#include "rec_session.h"
#include "rec_setup.h"
#include "roommap_link.h"
#include "steamvr_drivers.h"
#include "steamvr_reset.h"

namespace pibridge::app {

enum class IoVariant { SteamVr, MonadoDirect };

// Why a start cannot happen now (plain words; empty = it can). `blockers`: other parts that are busy (a recording,
// the other session), each already in plain words.
RecReadiness ioStartReadiness(const RecSetup& s, const RecSystem& sys, IoVariant v, const QStringList& blockers);
// The chip words of a live session: READY, NOT READY, STARTING, RUNNING, STOPPING, STOPPED, FAILED.
ChipText ioChip(RecState s, bool ready);

// The Dashboard's headset card while "Monado directly" owns the headset (keepalived follows vrserver and stays idle
// then, so its "Waiting for SteamVR" would be wrong): the state and the hint line. `active` false: nothing to say.
struct MonadoHeadsetWords {
    QString state;
    QString hint;
};
MonadoHeadsetWords monadoHeadsetWords(RecState s, const QString& message);

// An app waits at most this long for the home scene's session.
inline constexpr qint64 kHomeSettleMs = 8000;
// pibridge-home's state from its status file (home.json: "state", "pid") when `pid` wrote it; "" otherwise.
QString homeStateOf(const QByteArray& json, qint64 pid);
// pibridge-home next to the pibridge binary (`appDir`), in the build tree (<appDir>/../tools/home/), else in PATH; "".
QString findHomeProgram(const QString& appDir, const std::function<QString(const QString&)>& which);

// What the home scene does now (the Development page's line).
enum class HomeRun { Off, NotFound, NoRuntimeFile, Waiting, Running, Exited };
struct HomeState {
    HomeRun run = HomeRun::Off;
    qint64 pid = 0;
    QString words; // plain words
};

// The SteamVR processes of one start: those under the runtime (scanSteamVr's targets) that started at or after
// `sinceTicks` (clock ticks after boot, /proc/<pid>/stat field 22). Older ones were not started by it.
std::vector<SteamVrProc> steamVrProcsSince(const SteamVrScan& scan, qint64 sinceTicks);
// Clock ticks after boot now (from <procRoot>/uptime); -1 when unknown.
qint64 bootTicksNow(const QString& procRoot = QStringLiteral("/proc"), long ticksPerS = 0);

// ---------------------------------------------------------------- the daemons let go of the headset

// $XDG_RUNTIME_DIR/pibridge/inside-out.active (common/pibridge_pause.h): while it names a running owner, pibridge-keepalived
// releases the headset's HID (wake/keepalive) and reports "paused" so Monado's Pimax driver can take it. pibridge-eyed holds
// the Tobii eye chip (a different USB device), not the headset display; when eye tracking is on it keeps the chip and feeds the
// gaze shared memory to the driver (it does NOT pause), so the start must not wait for it. Every inside-out start writes the
// marker first and waits for keepalived to say so (or not run); every end removes it.
QString insideOutMarkerPath();
QString insideOutMarkerText(qint64 owner, IoVariant v);
// The owner and variant of a marker text (nullopt: not a marker).
std::optional<std::pair<qint64, QString>> parseInsideOutMarker(const QByteArray& text);
// Which daemons still hold the headset, in plain words ("the headset service (pibridge-keepalived)"): a status file
// that is there, written by a running pid, and not "paused". A missing file or a dead writer counts as let go. When
// eyeTrackerKeepsChip is true (eye tracking is on), pibridge-eyed is meant to keep the eye chip and is never counted.
QStringList daemonsHolding(const QByteArray& keepaliveJson, const QByteArray& eyedJson, bool eyeTrackerKeepsChip,
                           const std::function<bool(qint64)>& alive);
inline constexpr qint64 kDaemonReleaseMs = 15000; // keepalived first wakes a sleeping headset (about 5 s)

// What the app knows about SteamVR (its /proc check and its OpenVR background link).
struct IoLinks {
    std::function<bool()> steamvrRunning; // vrserver runs
    std::function<bool()> steamvrHmd;     // SteamVR's link is connected and reports the headset
};

// PiBridge World's exit code when it closes itself because a game took the headset (world/scripts/main.gd): its
// memory goes back to the game, and the dashboard overlay asks for it again once no app is left (worldReturnPath()).
inline constexpr int kHomeYieldCode = 75;
/// $XDG_RUNTIME_DIR/pibridge/world-return: the dashboard overlay's request to start the world again.
QString worldReturnPath();

class IoSessionControl : public QObject {
    Q_OBJECT
public:
    struct Deps {
        IoVariant variant = IoVariant::MonadoDirect;
        std::unique_ptr<RecProcesses> procs;
        std::function<RecSetup()> setup;   // resolved fresh before every start and readiness check
        std::function<RecSystem()> system; // the running processes
        RecProbe probe;
        std::function<qint64()> nowMs;     // monotonic ms
        bool autoTick = true;              // tick on a QTimer while active (the tests tick by hand)
        std::function<void(qint64)> advance; // fake mode: move the clock on (screenshots); empty otherwise
        // Monado directly
        std::function<QString()> homeProgram;               // pibridge-home ("" = not found)
        std::function<QString()> xrOverlayProgram;          // pibridge-xroverlay ("" = not found: no recenter ring)
        std::function<QString()> dashOverlayProgram;        // pibridge-home for --dashboard-overlay ("" = none)
        std::function<QString(const QString&)> runtimeJson; // the runtime file of a monado-service (apps_runtime.h)
        std::function<bool()> monadoSocket;                 // Monado's IPC socket exists
        std::function<QString(qint64 pid)> homeStatus;      // homeStateOf() the status file, "" unknown
        // the dashboard overlay asks for the world back (no app left in the headset after the world gave way to
        // one): true once per request, the request is consumed; false = none
        std::function<bool()> takeWorldReturn;
        // SteamVR + Monado
        std::function<bool()> vrserverRunning;              // the process check while the session runs
        std::function<std::expected<QStringList, QString>(const DriverPlan&)> applyDrivers;
        std::function<std::expected<bool, QString>()> restoreDrivers;
        std::function<bool()> driversPending;
        std::function<std::expected<QStringList, QString>(const DriverPlan&)> previewDrivers;
        std::function<QString()> writeShims;                // the PATH SteamVR gets ("" = none)
        std::function<qint64()> bootTicks;                  // clock ticks since boot now (the start's mark)
        std::function<std::vector<SteamVrProc>(qint64 sinceTicks)> steamVrProcs; // SteamVR's processes since then
        ProcSignals procSignals;
        // both variants: the room map (core/roommap_link.h): its status file's bytes ("" = none), the wall clock
        std::function<QByteArray()> roomStatus;
        std::function<qint64()> unixNow;
        // both variants: the pause marker (on: write ours; off: remove ours or a stale one) and the daemons' state
        std::function<bool(bool on)> marker;
        // the marker's variant follows setVariant()
        std::function<void(IoVariant)> markerVariant;
        std::function<QStringList(bool eyeTrackerKeepsChip)> daemonsHolding;
    };
    explicit IoSessionControl(Deps d, QObject* parent = nullptr);
    ~IoSessionControl() override;

    // The real system (QProcess, /proc, the file system, SteamVR's files), or the fake mode (FakeRecProcesses, a fake
    // setup that takes the inside-out settings from `conf`, a socket that "opens" after 0.8 s, SteamVR that "comes
    // up" 1.5 s after its launch, driver files that are never touched).
    static Deps realDeps(IoVariant v, Conf& conf, IoLinks links);
    static Deps fakeDeps(IoVariant v, Conf& conf, IoLinks links);

    IoVariant variant() const { return d_.variant; }
    // Re-reads the setup and the system; the result of readiness() / setup() after it. Puts SteamVR's drivers back
    // when that is pending and SteamVR is closed.
    void refresh();
    const RecReadiness& readiness() const { return readiness_; }
    const RecSetup& setup() const { return setup_; }
    // Another part is busy: blocks the start (`why` in plain words, "" = not any more).
    void setRecorderActive(bool on);
    void setOtherSession(const QString& why);

    bool start();
    // The VR backend's session (core/vr_backend.h): SteamVR (headtrack + CustomHeadsetOpenVR) or Monado directly. Only
    // while nothing runs; false (and unchanged) otherwise.
    bool setVariant(IoVariant v);
    void stop();
    void tick();
    bool active() const;
    RecState state() const;
    QString message() const;
    const RecSession* session() const { return session_.get(); }
    RecProcesses& processes() { return *d_.procs; }
    // Fake mode only (screenshots): start and move the fake clock on until the session runs.
    void demoRun();
    // PiBridge closes: stop exactly what it started (blocking, as the recorder does) and put SteamVR's drivers back.
    void shutdown();

    // SteamVR + Monado: what a start changes in SteamVR's files (the Tracking page shows it); the restore state.
    std::expected<QStringList, QString> driverPreview() const;
    bool driversPending() const;
    QString driverNote() const { return driverNote_; }

    // Monado directly, for the Apps page: is Monado ready for an app (the session runs, its socket is up, the home
    // has its session)?
    MonadoReadiness monadoReadiness() const;
    // The runtime file apps get (the session's monado-service; "" when its build has none).
    QString runtimeJson() const;
    HomeState home() const;
    // The home has its session, gave up, is off, or had kHomeSettleMs.
    bool homeSettled() const;

    // Both variants: the room map of this session (pibridge-roommap auto, started once the runtime is up when a saved
    // room exists; stopped by PID with the session) in plain words: "Room recognised / Searching / Room not found".
    RoomWords room() const;
    qint64 roomPid() const { return roomPid_; }

signals:
    void changed();

private:
    bool startSteamVr();
    bool continueStart();
    void endMarker();
    bool waitingRelease_ = false;
    bool markerOwned_ = false; // this control wrote the pause marker (endMarker removes only its own)
    qint64 releaseDeadline_ = 0;
    void tickSteamVr(qint64 now);
    void beginCloseSteamVr(qint64 now);
    void restoreDrivers();
    QStringList blockers() const;

    Deps d_;
    RecSetup setup_;
    RecReadiness readiness_;
    bool recorderActive_ = false;
    QString otherSession_;
    std::unique_ptr<RecSession> session_;
    QTimer timer_;
    QTimer pendingTimer_; // SteamVR's drivers wait to be put back

    // SteamVR + Monado
    QString startError_;       // the start failed before the session began
    qint64 launchTicks_ = -1;  // clock ticks since boot at the start: SteamVR's processes of this start are newer
    std::unique_ptr<SteamVrCloser> closer_;
    bool closeDone_ = false;   // SteamVR of this start was closed (or there was nothing to close)
    bool stopAfterClose_ = false;
    QString stopReason_;
    QString driverNote_;       // the last restore's result, plain words
    bool steamVrSeen_ = false; // vrserver was seen running in this session

    // the room map (both variants)
    void startRoom();
    void stopRoom(bool wait);
    void tickRoom(qint64 now);
    bool roomTried_ = false;
    qint64 roomPid_ = 0;
    qint64 roomTermMs_ = -1;
    bool roomKilled_ = false;
    qint64 roomSinceUnix_ = 0; // when it was started (its status file counts from then)
    QString roomError_;

    // Monado directly
    void startHome();
    void startXrOverlay();
    void stopXrOverlay();
    void stopHome(bool wait);
    QString homeProgram_;     // looked up at refresh(), kept while the session runs
    QString runtimeJson_;
    bool homeTried_ = false;  // once per session
    qint64 homeTermMs_ = -1;  // when it got SIGTERM
    bool homeKilled_ = false;
    qint64 homePid_ = 0;      // running pibridge-home
    qint64 xrOverlayPid_ = 0; // running pibridge-xroverlay (Monado's overlay: the recenter ring)
    qint64 dashOverlayPid_ = 0; // running pibridge-home --dashboard-overlay (the dashboard over games)
    qint64 homeStartMs_ = 0;
    bool homeExited_ = false;
    int homeCode_ = 0;
    bool homeCrashed_ = false;
    bool homeYielded_ = false; // the world closed itself for a game (kHomeYieldCode); back on takeWorldReturn
    QString homeError_;
    std::shared_ptr<bool> alive_ = std::make_shared<bool>(true);
};

// The Apps page's view of the runtime (core/apps_runtime.h): Monado while "Monado directly" (Development page) runs,
// SteamVR otherwise. In the inside-out mode SteamVR is started by START TRACKING (SteamVR + Monado), never by
// steam://rungameid/250820 (that SteamVR would load CustomHeadsetOpenVR instead of the Monado driver).
class TrackingRuntime : public AppRuntime {
public:
    TrackingRuntime(Conf& conf, IoSessionControl& tracking, IoSessionControl* direct)
        : conf_(conf), tracking_(tracking), direct_(direct) {}
    AppTarget target() const override;
    MonadoReadiness monado() const override;
    bool startTracking(QString* why) override;
    bool steamVrThroughTracking() const override;

private:
    Conf& conf_;
    IoSessionControl& tracking_;
    IoSessionControl* direct_;
};

} // namespace pibridge::app
