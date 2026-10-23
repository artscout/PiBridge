// SPDX-License-Identifier: MIT
// One "Record tracking" session: start the three programs, wait for data, run the guided steps, stop exactly what it
// started, check the dataset.
//
// Start:
//   1. pibridge-cam broker --source <camera> --socket $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock; wait (10 s at most)
//      until the socket exists;
//   2. pibridge-ctrltrack live --source broker:<socket> --factory;
//   3. monado-service with the Pimax driver's settings (monadoEnv()) and PIMAX_SLAM_RECORD=<folder>; Monado writes
//      <folder>/euroc_<datetime>/mav0/{cam0,cam1,imu0}. The steps start once IMU data comes in (30 s at most).
// The display lease can fail (the desktop does not hand over the headset's connector: "Connector ... withdrawn",
// "vkGetDrmDisplayEXT failed", "xrt_instance_create_system failed"); monado-service then exits. That is detected from
// its log and Monado is started once more after 4 s (broker and tracker stay); a second failure is said plainly.
// Stop: SIGTERM to exactly the PIDs it started, in reverse order (Monado first, so it finishes writing), each one
// SIGKILLed when it is still there after its wait (Monado 10 s, the others 3 s).
// Logs: <folder>/logs/{broker,ctrltrack,monado,monado-2,session}.log. steps.json is rewritten at every step and at
// the end; check.json after the check (core/rec_check.h).
//
// The programs run through RecProcesses (QProcess; FakeRecProcesses in the tests and the fake mode, which never runs
// anything). Time is an injected clock that the page ticks, so the tests drive every timeout without waiting.
#pragma once

#include <QString>
#include <QStringList>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "rec_check.h"
#include "rec_setup.h"
#include "rec_steps.h"
#include "runner.h"
#include "status.h"

class QProcess;
class QObject;

namespace pibridge::app {

inline constexpr qint64 kRecSocketWaitMs = 10000;
inline constexpr qint64 kRecDataWaitMs = 30000;
inline constexpr qint64 kRecLeaseRetryMs = 4000;
inline constexpr qint64 kRecMonadoTermMs = 10000;
inline constexpr qint64 kRecOtherTermMs = 3000;
inline constexpr qint64 kRecKillWaitMs = 2000;
// imu0/data.csv is bigger than its header once samples are written
inline constexpr qint64 kRecImuHeaderBytes = 200;
// An inside-out session (Mode::Live) counts as running once Monado has stayed up this long (no dataset to watch).
inline constexpr qint64 kIoSettleMs = 5000;
// SteamVR + Monado (Mode::LiveSteamVr): how long vrstartup.sh gets until vrserver runs and SteamVR reports the
// headset (omarchyvr's wait for vrserver).
inline constexpr qint64 kSteamVrStartWaitMs = 90000;

struct RecProcSpec {
    QString name;     // "broker" | "ctrltrack" | "monado" | "steamvr" | "home"
    QString program;
    QStringList args;
    QStringList env;  // KEY=value added to PiBridge's environment
    QString logPath;
};

class RecProcesses {
public:
    using Exited = std::function<void(int code, bool crashed)>;
    virtual ~RecProcesses() = default;
    // The PID (> 0), or 0 with `error` set. `onExit` runs on the GUI thread when it ends.
    virtual qint64 start(const RecProcSpec& s, Exited onExit, QString* error) = 0;
    // kill(pid, sig) for a PID this object started and that has not ended; false otherwise.
    virtual bool signal(qint64 pid, int sig) = 0;
    // Blocks up to `ms` for it to end (only used when PiBridge closes during a recording).
    virtual bool waitExit(qint64 pid, int ms) = 0;
};

class QtRecProcesses : public RecProcesses {
public:
    QtRecProcesses() = default;
    ~QtRecProcesses() override;
    qint64 start(const RecProcSpec& s, Exited onExit, QString* error) override;
    bool signal(qint64 pid, int sig) override;
    bool waitExit(qint64 pid, int ms) override;

private:
    std::map<qint64, QProcess*> procs_;
};

// Records the starts and signals, never runs anything. exit() plays a program's end; `exitOnTerm` ends a program at
// SIGTERM (unless its name is in `ignoreTerm`), SIGKILL always ends it (unless in `survivesKill`).
class FakeRecProcesses : public RecProcesses {
public:
    qint64 start(const RecProcSpec& s, Exited onExit, QString* error) override;
    bool signal(qint64 pid, int sig) override;
    bool waitExit(qint64 pid, int ms) override;
    void exit(qint64 pid, int code, bool crashed = false);
    bool running(qint64 pid) const { return running_.contains(pid); }
    qint64 pidOf(const QString& name) const; // the last started one of that name (0)

    struct Signal {
        qint64 pid;
        int sig;
    };
    std::vector<RecProcSpec> started;
    std::vector<qint64> pids;       // in start order
    std::vector<Signal> signalled;
    bool exitOnTerm = true;
    std::set<QString> ignoreTerm;
    std::set<QString> survivesKill;
    QString failWith;               // non-empty: start() fails with this error
    QString failName;               // ... only for this name (empty: every start)
    qint64 nextPid = 7100;

private:
    std::map<qint64, std::pair<QString, Exited>> running_;
};

// True when a monado-service log shows the display lease failure (lesson: the desktop withdrew the connector).
bool leaseFailure(const QString& monadoLog);
// The last lines of a log that say what went wrong (error/warning lines first), for the message line.
QString logReason(const QString& log);

// The programs' command lines and Monado's environment (pure; tested). Monado's environment is the one builder of
// core/inside_out.h (buildIoEnv) with the setup's inside-out settings; `recordDir` "" = no recording.
RecProcSpec brokerSpec(const RecSetup& s, const QString& logDir);
RecProcSpec ctrltrackSpec(const RecSetup& s, const QString& logDir);
// The Pimax SLAM engine: `wine winbridge.exe` with the kit as PIBRIDGE_WINDATA (a Z: path). Started next to the broker,
// before Monado / SteamVR, when winbridgeWanted(); stopped by PID with the rest (wine execs into it: same PID).
RecProcSpec winbridgeSpec(const RecSetup& s, const QString& logDir);
bool winbridgeWanted(const RecSetup& s);
// pibridge-headtrack (io_headtrack with the Pimax engine) and its command line
bool headtrackWanted(const RecSetup& s);
RecProcSpec headtrackSpec(const RecSetup& s, const QString& logDir);
RecProcSpec monadoSpec(const RecSetup& s, const QString& recordDir, const QString& logPath);
QStringList monadoEnv(const RecSetup& s, const QString& recordDir);
// SteamVR + Monado: <SteamVR>/bin/vrstartup.sh with SteamVR's launch environment (buildIoEnv's steamvr list: the
// preload of the same Monado build, PIMAX_STEAMVR_ENV_FILE) plus s.steamVrExtraEnv (the PATH of the shims).
RecProcSpec steamvrSpec(const RecSetup& s, const QString& logDir);
QStringList steamvrEnv(const RecSetup& s);
// The text of the driver settings file driver_monado.so reads (s.driverEnvFile).
QString steamvrDriverEnvText(const RecSetup& s);
// Generates PiBridge's Basalt config for the setup's settings (core/inside_out.h generateSlamConfig into slamDir()) and
// points the setup's sessionSlamConfig at it. On failure the base config stays in use and `note` says why.
void prepareSessionSlam(RecSetup& s, QString* note = nullptr);

// What the session looks at outside itself; real() = the file system.
struct RecProbe {
    std::function<bool(const QString& socket)> socketReady;
    std::function<qint64(const QString& recordDir)> imuBytes; // size of the newest euroc_*/mav0/imu0/data.csv
    std::function<QString(const QString& path)> readLog;      // the tail of a log file
    std::function<qint64()> monoNs;                           // CLOCK_MONOTONIC now
    // SteamVR + Monado: vrserver runs and SteamVR reports the headset (the app's OpenVR link). Empty = never.
    std::function<bool()> steamVrUp;
    static RecProbe real();
};

enum class RecState { Idle, Starting, RetryWait, Recording, Stopping, Finished, Failed };

// Mode::Live is "Monado directly": the same programs, start order, lease retry and PID-exact stop, without
// PIMAX_SLAM_RECORD, steps.json or the dataset check; the folder (in setup.sessionsDir) holds only the logs.
// Mode::LiveSteamVr is START TRACKING (SteamVR + Monado): the broker and the tracker as in Live, then vrstartup.sh
// instead of monado-service; it runs once probe.steamVrUp() says so (kSteamVrStartWaitMs at most). The launcher may
// end by itself once SteamVR runs (that is no failure); closing SteamVR itself is the caller's part
// (core/io_session.h), the session stops what it started.
class RecSession {
public:
    enum class Mode { Guided, Free, Live, LiveSteamVr };
    // `say`: speak a line (may be empty). `folderName`: the recording folder's name (default yyyyMMdd-HHmmss now).
    RecSession(RecSetup setup, RecProcesses& procs, RecProbe probe, std::function<void(const QString&)> say = {});
    ~RecSession();

    // Creates the folder and starts the broker. False (state Failed, message set) when that already fails.
    bool start(Mode mode, std::vector<RecStep> steps, qint64 nowMs, const QString& folderName = {});
    // Advance: socket/data waits, the lease retry, the steps, the stop sequence. True when something visible changed.
    bool tick(qint64 nowMs);
    void skipStep(qint64 nowMs);
    // Stop by the user (or the end of the steps).
    void stop(qint64 nowMs, const QString& reason = QStringLiteral("stopped by the user"));
    // PiBridge closes: TERM in reverse order, waiting (blocking) as the stop sequence does.
    void shutdown();

    RecState state() const { return state_; }
    bool active() const { return state_ == RecState::Starting || state_ == RecState::RetryWait ||
                                 state_ == RecState::Recording || state_ == RecState::Stopping; }
    Mode mode() const { return mode_; }
    bool live() const { return mode_ == Mode::Live || mode_ == Mode::LiveSteamVr; }
    // The line under the button: what is going on, or what went wrong (plain words).
    const QString& message() const { return message_; }
    const QString& folder() const { return folder_; }
    QString logDir() const { return folder_ + QStringLiteral("/logs"); }
    // PIMAX_SLAM_RECORD: the folder, or "" for a live session
    QString recordDir() const { return live() ? QString() : folder_; }
    const StepRunner* steps() const { return runner_.get(); }
    qint64 recordingMs(qint64 nowMs) const; // since the data came in, 0 before
    int leaseRetries() const { return retries_; }
    const std::optional<RecordingInfo>& result() const { return result_; }
    // name -> PID of what it started and is still running.
    std::map<QString, qint64> runningPids() const;

private:
    struct Proc {
        RecProcSpec spec;
        qint64 pid = 0;
        bool exited = false;
        int code = 0;
        bool crashed = false;
        bool unhandledExit = false; // ended; the session has not reacted yet
    };
    bool startProc(const RecProcSpec& spec, qint64 nowMs);
    Proc* proc(const QString& name);
    void fail(qint64 nowMs, const QString& why);
    void beginStop(qint64 nowMs);
    void stepStop(qint64 nowMs);
    void finish(qint64 nowMs);
    void handleExits(qint64 nowMs);
    void endFailedMonado();
    void log(const QString& line);
    void writeSteps();
    void sayStep();
    void copyTrackerSlamFiles();

    enum class Phase { Broker, Monado };
    RecSetup setup_;
    RecProcesses& procs_;
    RecProbe probe_;
    std::function<void(const QString&)> say_;
    RecState state_ = RecState::Idle;
    Phase phase_ = Phase::Broker;
    Mode mode_ = Mode::Guided;
    QString message_;
    QString folder_;
    std::vector<std::shared_ptr<Proc>> procsList_; // start order
    std::shared_ptr<bool> alive_ = std::make_shared<bool>(true);
    std::unique_ptr<StepRunner> runner_;
    std::vector<RecStep> plan_;
    StepsRecord record_;
    qint64 phaseStartMs_ = 0;
    qint64 retryAtMs_ = 0;
    int retries_ = 0;
    bool dataSeen_ = false;
    bool monadoEnded_ = false; // SIGTERM sent to a Monado whose compositor failed while starting
    bool failed_ = false;
    QString failReason_;
    // stop sequence: index into procsList_ walking backwards, and when the current one got TERM / KILL
    int stopIdx_ = -1;
    qint64 termAtMs_ = -1;
    qint64 killAtMs_ = -1;
    QStringList notStopped_;
    std::optional<RecordingInfo> result_;
};

// A made-up recording for the fake mode and the tests: <folder>/euroc_<datetime>/mav0 with `seconds` of IMU at
// 1000 Hz (gravity on y) and two cameras at 25 Hz (shared timestamps, empty image files), a 35 ms IMU gap when
// `withGap`, steps.json (the built-in steps for "guided") and logs/session.log. Timestamps start at `startNs`.
bool writeFakeRecording(const QString& folder, double seconds, bool withGap, const QString& mode, qint64 startNs);

// The plain words of a state for the chip: READY, STARTING, RECORDING, STOPPING, DONE, PROBLEM, FAILED.
ChipText recChip(RecState s, const std::optional<RecordingInfo>& result, bool ready);

// Speaks through spd-say (detached, through the Runner), else plays a short tone with pw-play or paplay, else
// nothing. Tool lookup is injected (tests).
class Announcer {
public:
    Announcer(Runner& runner, std::function<QString(const QString&)> which);
    void say(const QString& text);
    QString how() const; // "spd-say", "pw-play (tone)", "none"
    // The tone file (a 0.25 s 880 Hz WAV), written once into `dir`.
    static QByteArray toneWav();

private:
    Runner& runner_;
    QString spd_;
    QString player_;
    QString tone_;
};

} // namespace pibridge::app
