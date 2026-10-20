// SPDX-License-Identifier: MIT
// Starting the Apps page's entries.
//   Steam game:  steam steam://rungameid/<id>, detached through the Runner. A VR game waits for SteamVR like an own
//                app: an OpenXR game started first comes up before SteamVR does and does not find the runtime.
//   Own app:     when SteamVR is not fully ready (core/steamvr_ready.h): steam steam://rungameid/250820 (unless
//                vrserver already runs), wait until it is, then start the program with its arguments, working
//                directory and environment (+ DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1 when quad views are
//                off for it). Its stdout and stderr go to ~/.local/state/pibridge/apps/<id>-<utc>.log, with the symlink
//                <id>-latest.log. The app keeps the process to show "running" and "exited (code)"; Stop sends SIGTERM
//                to that one PID only. Closing PiBridge leaves a running app running.
// While "Monado directly" runs (core/apps_runtime.h) an own app waits for that session instead, gets
// XR_RUNTIME_JSON=<the Monado build's openxr_monado-dev.json>, and SteamVR is never started; Steam games are refused
// there with the reason. In the inside-out mode SteamVR is started by START TRACKING (SteamVR + Monado) instead of
// steam://rungameid/250820, then everything waits for SteamVR as above.
// Programs are started through AppProcesses (QProcess; FakeAppProcesses in tests and the fake mode, which never
// runs anything), Steam through the Runner (core/runner.h).
#pragma once

#include <QDateTime>
#include <QHash>
#include <QObject>
#include <QSet>
#include <QTimer>
#include <QString>
#include <QStringList>
#include <functional>
#include <map>
#include <vector>

#include "apps_own.h"
#include "apps_runtime.h"
#include "runner.h"
#include "status.h"

class QProcess;

namespace pibridge::app {

class SteamVrReady;

inline constexpr char kQuadViewsOffVar[] = "DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN";
inline constexpr char kSteamVrAppId[] = "250820";
// A Steam game shows STARTING this long after Play unless it is seen running sooner.
inline constexpr qint64 kSteamStartShowMs = 45000;

struct LaunchSpec {
    QString program;  // ~ expanded
    QStringList args;
    QString workdir;  // ~ expanded; the program's folder when the app names none
    QStringList env;  // KEY=value added to (or replacing in) PiBridge's own environment
    QString logPath;  // <logDir>/<id>-<yyyyMMdd-HHmmss>Z.log
    QString logLink;  // <logDir>/<id>-latest.log
};

// "~/x" -> "<home>/x"; everything else unchanged.
QString expandHome(const QString& path, const QString& home = {});
// What starting `a` at `utc` means (pure; tested).
LaunchSpec ownAppSpec(const OwnApp& a, const QString& logDir, const QDateTime& utc, const QString& home = {});
// Creates the log folder and the log file, and points the -latest link at it; false with `error` set when not.
bool prepareLog(const LaunchSpec& s, QString* error);

class AppProcesses {
public:
    using Exited = std::function<void(int code, bool crashed)>;
    virtual ~AppProcesses() = default;
    // Start the program; the pid (> 0), or 0 with `error` set. `onExit` is called on the GUI thread when it ends.
    virtual qint64 start(const LaunchSpec& s, Exited onExit, QString* error) = 0;
    // SIGTERM to `pid` only, if it is one this object started and it still runs.
    virtual bool stop(qint64 pid) = 0;
};

// QProcess. On destruction the running programs are left running (their QProcess objects are let go of on purpose,
// since QProcess would otherwise kill them).
class QtAppProcesses : public QObject, public AppProcesses {
    Q_OBJECT
public:
    explicit QtAppProcesses(QObject* parent = nullptr) : QObject(parent) {}
    ~QtAppProcesses() override;
    qint64 start(const LaunchSpec& s, Exited onExit, QString* error) override;
    bool stop(qint64 pid) override;

private:
    std::map<qint64, QProcess*> procs_;
};

// Records the starts; never runs anything. exit() plays a program's end.
class FakeAppProcesses : public AppProcesses {
public:
    qint64 start(const LaunchSpec& s, Exited onExit, QString* error) override;
    bool stop(qint64 pid) override;
    void exit(qint64 pid, int code, bool crashed = false);

    std::vector<LaunchSpec> started;
    std::vector<qint64> stopped;
    QString failWith; // non-empty: start() fails with this error
    qint64 nextPid = 4000;

private:
    std::map<qint64, Exited> running_;
};

enum class AppRun { Idle, StartingSteamVr, StartingTracking, Starting, Running, Exited, Error };

struct AppRunState {
    AppRun run = AppRun::Idle;
    qint64 pid = 0;
    int exitCode = 0;
    bool crashed = false;
    QString message; // the error, in plain words
    QString logPath;
};

// The tile's chip: RUNNING / STARTING STEAMVR / STARTING TRACKING / STARTING / ERROR / READY (the runtime is fully
// ready: SteamVR, or Monado in "Monado directly" mode) / "" (idle).
ChipText appChip(const AppRunState& s, bool runtimeReady);
// The tile's line under the name: "Exited (code 0)", "Crashed", the error, "" for nothing.
QString appRunWords(const AppRunState& s);

class AppLauncher : public QObject {
    Q_OBJECT
public:
    using Clock = std::function<qint64()>; // monotonic ms
    AppLauncher(Runner& runner, AppProcesses& procs, SteamVrReady& ready, std::function<bool()> vrserverRunning,
                QString logDir, Clock clock = {}, QObject* parent = nullptr);

    // The runtime own apps start on (not owned; nullptr = always SteamVR, as before).
    void setRuntime(AppRuntime* runtime) { runtime_ = runtime; }
    AppTarget target() const { return runtime_ ? runtime_->target() : AppTarget::SteamVr; }

    // Start an own app (see the top of this file). Ignored while it is starting or running.
    void launchOwn(const OwnApp& a);
    // Re-check the apps that wait for Monado (a timer calls it every 250 ms while one waits; tests call it with their
    // clock): start them once Monado is ready, give up after kReadyTimeoutMs or when the session failed.
    void pollMonado();
    int waitingForMonado() const { return static_cast<int>(monadoWait_.size()); }
    // Stop it: SIGTERM to its PID, or cancel the wait for SteamVR or Monado.
    bool stopOwn(const QString& id);
    AppRunState ownState(const QString& id) const;
    // For the fake mode's screenshots only.
    void setOwnStateForShow(const QString& id, const AppRunState& s);

    // steam steam://rungameid/<id>; a VR game only once SteamVR is fully ready (it is started first when needed).
    // False when Steam could not be started, or in "Monado directly" mode (steamBlockedWhy(); nothing is started).
    bool launchSteam(const QString& appId, bool vr = true);
    // Cancel a Steam game's wait for SteamVR.
    bool cancelSteam(const QString& appId);
    // The Steam game's state from the running set (runningSteamApps()) and the time of the last Play.
    AppRunState steamState(const QString& appId, const QSet<QString>& running) const;

signals:
    void changed(const QString& id);
    // An app was started (the page writes last_played).
    void played(const QString& id);

private:
    void startNow(const OwnApp& a, const QString& runtimeJson = {});
    void set(const QString& id, AppRunState s);
    void launchOnMonado(const OwnApp& a);
    // SteamVR is not running: start it (steam://rungameid/250820, or START TRACKING in the inside-out mode).
    bool startSteamVr(QString* why);
    struct MonadoWait {
        OwnApp app;
        qint64 deadline = 0;
        bool sawStart = false; // the session was seen starting (a later Off means it ended)
    };

    Runner& runner_;
    AppProcesses& procs_;
    SteamVrReady& ready_;
    std::function<bool()> vrserverRunning_;
    QString logDir_;
    Clock clock_;
    QHash<QString, AppRunState> own_;
    QHash<QString, int> waitToken_; // a Stop during the wait invalidates the pending start
    mutable QHash<QString, qint64> steamStarted_; // dropped once the game is seen running
    QHash<QString, QString> steamError_;
    QHash<QString, int> steamWait_; // token of a VR game's pending start (absent: not waiting)
    AppRuntime* runtime_ = nullptr;
    QHash<QString, MonadoWait> monadoWait_; // own apps waiting for the Tracking session
    QTimer monadoTimer_;
};

} // namespace pibridge::app
