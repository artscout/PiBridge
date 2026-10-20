// SPDX-License-Identifier: MIT
#include "rec_session.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QtEndian>
#include <cmath>
#include <csignal>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::app {

// ---------------------------------------------------------------- processes

QtRecProcesses::~QtRecProcesses() {
    for (auto& [pid, p] : procs_) {
        (void)pid;
        p->disconnect();
        if (p->state() == QProcess::NotRunning) {
            delete p;
        }
        // a still running one is let go of on purpose: QProcess's destructor would SIGKILL it (the session's
        // shutdown() has already sent TERM and waited)
    }
    procs_.clear();
}

qint64 QtRecProcesses::start(const RecProcSpec& s, Exited onExit, QString* error) {
    QDir().mkpath(QFileInfo(s.logPath).absolutePath());
    auto* p = new QProcess;
    p->setProgram(s.program);
    p->setArguments(s.args);
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
    // its own session: a Ctrl+C in the terminal PiBridge was started from does not reach it
    p->setChildProcessModifier([] { ::setsid(); });
    QObject::connect(p, &QProcess::finished, p, [onExit](int code, QProcess::ExitStatus st) {
        if (onExit) {
            onExit(code, st != QProcess::NormalExit);
        }
    });
    p->start();
    if (!p->waitForStarted(5000)) {
        if (error) {
            *error = QStringLiteral("Could not start %1: %2").arg(QFileInfo(s.program).fileName(), p->errorString());
        }
        p->disconnect();
        delete p;
        return 0;
    }
    const qint64 pid = p->processId();
    procs_[pid] = p;
    return pid;
}

bool QtRecProcesses::signal(qint64 pid, int sig) {
    const auto it = procs_.find(pid);
    if (it == procs_.end() || it->second->state() != QProcess::Running || it->second->processId() != pid) {
        return false;
    }
    return ::kill(static_cast<pid_t>(pid), sig) == 0; // that PID only
}

bool QtRecProcesses::waitExit(qint64 pid, int ms) {
    const auto it = procs_.find(pid);
    if (it == procs_.end() || it->second->state() == QProcess::NotRunning) {
        return true;
    }
    return it->second->waitForFinished(ms);
}

qint64 FakeRecProcesses::start(const RecProcSpec& s, Exited onExit, QString* error) {
    if (!failWith.isEmpty() && (failName.isEmpty() || failName == s.name)) {
        if (error) {
            *error = failWith;
        }
        return 0;
    }
    started.push_back(s);
    const qint64 pid = nextPid++;
    pids.push_back(pid);
    running_[pid] = {s.name, std::move(onExit)};
    return pid;
}

bool FakeRecProcesses::signal(qint64 pid, int sig) {
    const auto it = running_.find(pid);
    if (it == running_.end()) {
        return false;
    }
    signalled.push_back({pid, sig});
    const QString name = it->second.first;
    if ((sig == SIGTERM && exitOnTerm && !ignoreTerm.contains(name)) || (sig == SIGKILL && !survivesKill.contains(name))) {
        exit(pid, sig == SIGKILL ? -1 : 0, sig == SIGKILL);
    }
    return true;
}

bool FakeRecProcesses::waitExit(qint64 pid, int) {
    return !running_.contains(pid);
}

void FakeRecProcesses::exit(qint64 pid, int code, bool crashed) {
    const auto it = running_.find(pid);
    if (it == running_.end()) {
        return;
    }
    Exited cb = std::move(it->second.second);
    running_.erase(it);
    if (cb) {
        cb(code, crashed);
    }
}

qint64 FakeRecProcesses::pidOf(const QString& name) const {
    for (size_t i = started.size(); i-- > 0;) {
        if (started[i].name == name) {
            return pids[i];
        }
    }
    return 0;
}

// ---------------------------------------------------------------- logs and command lines

bool leaseFailure(const QString& log) {
    static const char* const kSigns[] = {
        "has been withdrawn",           // comp_window_direct_wayland: "Connector DP-6 has been withdrawn by the compositor"
        "vkGetDrmDisplayEXT failed",
        "xrt_instance_create_system failed",
        "Failed to init compositor",    // comp_main: the display was not taken (Monado ended by the session)
        "Connector was disconnected before it could be acquired",
    };
    for (const char* s : kSigns) {
        if (log.contains(QLatin1String(s))) {
            return true;
        }
    }
    return false;
}

QString logReason(const QString& log) {
    QStringList lines = log.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    QStringList errors;
    for (const QString& l : lines) {
        const QString t = l.trimmed();
        if (t.contains(QLatin1String("ERROR"), Qt::CaseSensitive) || t.contains(QLatin1String(" error"), Qt::CaseInsensitive) ||
            t.contains(QLatin1String("failed"), Qt::CaseInsensitive)) {
            errors << t;
        }
    }
    const QStringList& from = errors.isEmpty() ? lines : errors;
    QStringList tail = from.mid(std::max<qsizetype>(0, from.size() - 2));
    for (QString& t : tail) {
        t = t.trimmed();
        if (t.size() > 160) {
            t = t.left(157) + QStringLiteral("...");
        }
    }
    return tail.join(QStringLiteral(" / "));
}

RecProcSpec brokerSpec(const RecSetup& s, const QString& logDir) {
    return {QStringLiteral("broker"),
            s.cam.path,
            {QStringLiteral("broker"), QStringLiteral("--source"), s.camDevice, QStringLiteral("--socket"), s.brokerSocket},
            {QStringLiteral("PIMAX_CAM_CALIB=") + s.calib.path},
            logDir + QStringLiteral("/broker.log")};
}

RecProcSpec ctrltrackSpec(const RecSetup& s, const QString& logDir) {
    return {QStringLiteral("ctrltrack"),
            s.ctrl.path,
            {QStringLiteral("live"), QStringLiteral("--source"), QStringLiteral("broker:") + s.brokerSocket,
             QStringLiteral("--factory")},
            {QStringLiteral("PIMAX_CAM_CALIB=") + s.calib.path},
            logDir + QStringLiteral("/ctrltrack.log")};
}

RecProcSpec winbridgeSpec(const RecSetup& s, const QString& logDir) {
    QString kit = QStringLiteral("Z:") + s.winKit;
    kit.replace(QLatin1Char('/'), QLatin1Char('\\'));
    return {QStringLiteral("winbridge"),
            s.wine,
            {s.winbridge.path},
            {QStringLiteral("WINEDEBUG=-all"), QStringLiteral("PIBRIDGE_WINDATA=") + kit},
            logDir + QStringLiteral("/winbridge.log")};
}

bool winbridgeWanted(const RecSetup& s) {
    return s.io.slam && s.io.slamEngine == SlamEngine::Pimax && pimaxSlamReady(s);
}

QStringList monadoEnv(const RecSetup& s, const QString& recordDir) {
    return buildIoEnv(s.io, s, s.refreshHz, s.sessionSlamConfig.isEmpty() ? s.slamConfig.path : s.sessionSlamConfig,
                      recordDir)
        .monado;
}

QString exportFourCamCalib(const RecSetup& s, const QString& outDir, QString* error) {
    const auto fail = [error](const QString& e) {
        if (error) {
            *error = e;
        }
        return QString();
    };
    if (s.slamTool.isEmpty()) {
        return fail(QStringLiteral("pibridge-slam was not found."));
    }
    if (s.calib.path.isEmpty()) {
        return fail(QStringLiteral("The headset's factory calibration was not found (rec_cam_calib)."));
    }
    const QString calib = outDir + QStringLiteral("/pimax-basalt-calib.json");
    QFile::remove(calib); // never a stale one of an earlier start
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
    p.start(s.slamTool, fourCamExportArgs(outDir, s.calib.path, slamTomlFiles(s.slamConfig.path).config));
    if (!p.waitForFinished(15000) || p.exitStatus() != QProcess::NormalExit || p.exitCode() != 0) {
        p.kill();
        p.waitForFinished(1000);
        const QString out = QString::fromUtf8(p.readAll()).trimmed().section(QLatin1Char('\n'), -1);
        return fail(QStringLiteral("pibridge-slam export failed%1.").arg(out.isEmpty() ? QString() : QStringLiteral(": ") + out));
    }
    if (!QFileInfo(calib).isFile()) {
        return fail(QStringLiteral("pibridge-slam export wrote no calibration."));
    }
    return calib;
}

void prepareSessionSlam(RecSetup& s, QString* note) {
    s.sessionSlamConfig.clear();
    s.sessionSlamCams = 0;
    if (!s.io.slam || s.basalt.path.isEmpty() || s.slamConfig.path.isEmpty()) {
        return;
    }
    SlamGenInput in;
    in.baseToml = s.slamConfig.path;
    in.outDir = slamDir();
    in.grid = s.io.grid;
    in.accelBias = s.io.accelBias;
    in.gyroBias = s.io.gyroBias;
    in.factoryXml = s.calib.path;
    in.gyroCache = gyroBiasCachePath();
    QString camsNote;
    if (s.io.slamCams == 4) {
        // all four cameras: the upper pair's calibration from the factory file, appended to the base's lower pair
        QString err;
        in.camsCalib = exportFourCamCalib(s, slamDir() + QStringLiteral("/cams4"), &err);
        if (in.camsCalib.isEmpty()) {
            camsNote = QStringLiteral("Four head tracking cameras: ") + err + QStringLiteral(" The lower pair is used.");
        }
    }
    SlamGenResult r = generateSlamConfig(in);
    if (!r.ok && !in.camsCalib.isEmpty()) {
        camsNote = QStringLiteral("Four head tracking cameras: ") + r.error + QStringLiteral(" The lower pair is used.");
        in.camsCalib.clear();
        r = generateSlamConfig(in);
    }
    if (r.ok) {
        s.sessionSlamConfig = r.toml;
        s.sessionSlamCams = r.cams;
    }
    if (note) {
        QStringList n;
        if (!camsNote.isEmpty()) {
            n << camsNote;
        }
        if (!r.ok) {
            n << r.error + QStringLiteral(" The base config is used as it is.");
        }
        *note = n.join(QLatin1Char(' '));
    }
}

QStringList steamvrEnv(const RecSetup& s) {
    const IoSessionEnv e = buildIoEnv(s.io, s, s.refreshHz, s.sessionSlamConfig.isEmpty() ? s.slamConfig.path : s.sessionSlamConfig,
                                      QString(), s.driverEnvFile, s.steamVrPreload);
    return e.steamvr + s.steamVrExtraEnv;
}

QString steamvrDriverEnvText(const RecSetup& s) {
    return driverEnvFileText(buildIoEnv(s.io, s, s.refreshHz,
                                        s.sessionSlamConfig.isEmpty() ? s.slamConfig.path : s.sessionSlamConfig, QString(),
                                        s.driverEnvFile, s.steamVrPreload));
}

RecProcSpec steamvrSpec(const RecSetup& s, const QString& logDir) {
    return {QStringLiteral("steamvr"), s.steamVrStartup, {}, steamvrEnv(s), logDir + QStringLiteral("/steamvr.log")};
}

RecProcSpec monadoSpec(const RecSetup& s, const QString& recordDir, const QString& logPath) {
    return {QStringLiteral("monado"), s.monado.path, {}, monadoEnv(s, recordDir), logPath};
}

RecProbe RecProbe::real() {
    RecProbe p;
    p.socketReady = [](const QString& path) {
        struct stat st {};
        return ::stat(QFile::encodeName(path).constData(), &st) == 0 && S_ISSOCK(st.st_mode);
    };
    p.imuBytes = [](const QString& dir) -> qint64 {
        const QDir d(dir);
        const QStringList e = d.entryList({QStringLiteral("euroc_*")}, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
        if (e.isEmpty()) {
            return -1;
        }
        const QFileInfo fi(d.filePath(e.last() + QStringLiteral("/mav0/imu0/data.csv")));
        return fi.exists() ? fi.size() : -1;
    };
    p.readLog = [](const QString& path) {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) {
            return QString();
        }
        if (f.size() > 64 * 1024) {
            f.seek(f.size() - 64 * 1024);
        }
        return QString::fromLocal8Bit(f.readAll());
    };
    p.monoNs = [] {
        timespec ts{};
        ::clock_gettime(CLOCK_MONOTONIC, &ts);
        return qint64(ts.tv_sec) * 1000000000 + ts.tv_nsec;
    };
    return p;
}

// ---------------------------------------------------------------- session

RecSession::RecSession(RecSetup setup, RecProcesses& procs, RecProbe probe, std::function<void(const QString&)> say)
    : setup_(std::move(setup)), procs_(procs), probe_(std::move(probe)), say_(std::move(say)) {}

RecSession::~RecSession() {
    shutdown();
    *alive_ = false;
}

void RecSession::log(const QString& line) {
    if (folder_.isEmpty()) {
        return;
    }
    QFile f(logDir() + QStringLiteral("/session.log"));
    if (f.open(QIODevice::WriteOnly | QIODevice::Append)) {
        f.write((QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz ")) + line + QLatin1Char('\n'))
                    .toUtf8());
    }
}

void RecSession::writeSteps() {
    if (live()) {
        return; // a live session has no steps.json
    }
    record_.leaseRetries = retries_;
    if (!writeStepsJson(folder_ + QLatin1Char('/') + QString::fromLatin1(kStepsFile), record_, runner_.get())) {
        log(QStringLiteral("could not write steps.json"));
    }
}

void RecSession::sayStep() {
    if (runner_ && runner_->currentStep() && say_) {
        say_(runner_->currentStep()->text);
    }
}

RecSession::Proc* RecSession::proc(const QString& name) {
    for (auto it = procsList_.rbegin(); it != procsList_.rend(); ++it) {
        if ((*it)->spec.name == name) {
            return it->get();
        }
    }
    return nullptr;
}

std::map<QString, qint64> RecSession::runningPids() const {
    std::map<QString, qint64> m;
    for (const auto& p : procsList_) {
        if (!p->exited) {
            m[p->spec.name] = p->pid;
        }
    }
    return m;
}

qint64 RecSession::recordingMs(qint64 nowMs) const {
    if (!dataSeen_) {
        return 0;
    }
    const qint64 end = record_.recordEndMs >= 0 ? record_.recordEndMs : nowMs;
    return std::max<qint64>(0, end - record_.recordStartMs);
}

bool RecSession::startProc(const RecProcSpec& spec, qint64 nowMs) {
    auto p = std::make_shared<Proc>();
    p->spec = spec;
    QString err;
    std::weak_ptr<bool> alive = alive_;
    const qint64 pid = procs_.start(
        spec,
        [p, alive](int code, bool crashed) {
            if (alive.expired() || !*alive.lock()) {
                return;
            }
            p->exited = true;
            p->code = code;
            p->crashed = crashed;
            p->unhandledExit = true;
        },
        &err);
    if (pid <= 0) {
        log(QStringLiteral("could not start %1: %2").arg(spec.name, err));
        fail(nowMs, err.isEmpty() ? QStringLiteral("Could not start %1.").arg(QFileInfo(spec.program).fileName()) : err);
        return false;
    }
    p->pid = pid;
    procsList_.push_back(p);
    log(QStringLiteral("started %1, PID %2: %3 %4").arg(spec.name).arg(pid).arg(spec.program, spec.args.join(QLatin1Char(' '))));
    if (!spec.env.isEmpty()) {
        log(QStringLiteral("  environment: %1").arg(spec.env.join(QLatin1Char(' '))));
    }
    return true;
}

bool RecSession::start(Mode mode, std::vector<RecStep> steps, qint64 nowMs, const QString& folderName) {
    if (active()) {
        return false;
    }
    mode_ = mode;
    procsList_.clear();
    result_.reset();
    retries_ = 0;
    monadoEnded_ = false;
    dataSeen_ = false;
    failed_ = false;
    failReason_.clear();
    notStopped_.clear();
    stopIdx_ = -1;
    const QString base = folderName.isEmpty() ? QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss"))
                                              : folderName;
    const bool liveMode = mode == Mode::Live || mode == Mode::LiveSteamVr;
    const QString parent = liveMode ? setup_.sessionsDir : setup_.recordingsDir;
    QString name = base;
    for (int n = 2; QFileInfo::exists(parent + QLatin1Char('/') + name); n++) {
        name = base + QStringLiteral("-%1").arg(n);
    }
    folder_ = parent + QLatin1Char('/') + name;
    if (!QDir().mkpath(logDir())) {
        state_ = RecState::Failed;
        message_ = QStringLiteral("Could not create the folder %1.").arg(folder_);
        return false;
    }
    plan_ = std::move(steps);
    runner_ = mode == Mode::Guided ? std::make_unique<StepRunner>(plan_) : nullptr;
    record_ = {};
    record_.mode = mode == Mode::Guided ? QStringLiteral("guided")
                   : mode == Mode::Free ? QStringLiteral("free")
                   : mode == Mode::Live ? QStringLiteral("live")
                                        : QStringLiteral("live-steamvr");
    record_.monoAtMs0Ns = probe_.monoNs() - nowMs * 1000000;
    record_.endReason = QStringLiteral("recording");
    writeSteps();
    log(mode == Mode::Live          ? QStringLiteral("inside-out session (Monado directly), logs in %1").arg(folder_)
        : mode == Mode::LiveSteamVr ? QStringLiteral("inside-out session (SteamVR + Monado), logs in %1").arg(folder_)
                                    : QStringLiteral("recording into %1 (%2)").arg(folder_, record_.mode));
    state_ = RecState::Starting;
    phase_ = Phase::Broker;
    phaseStartMs_ = nowMs;
    message_ = QStringLiteral("Starting the camera broker...");
    if (say_ && !liveMode) {
        say_(QStringLiteral("Starting the recording. Please wait."));
    }
    return startProc(brokerSpec(setup_, logDir()), nowMs);
}

void RecSession::fail(qint64 nowMs, const QString& why) {
    if (failed_) {
        return;
    }
    failed_ = true;
    failReason_ = why;
    message_ = why;
    record_.endReason = why;
    record_.stoppedEarly = true;
    log(QStringLiteral("failed: %1").arg(why));
    if (say_ && !live()) {
        say_(QStringLiteral("Recording failed."));
    }
    if (state_ != RecState::Stopping) {
        beginStop(nowMs);
    }
}

// Monado's IPC server does not exit when its compositor fails to take the display (KWin withdrew the lease): it
// tears the Pimax driver down and builds the system again for the next client, without the headset (which then
// resets every few seconds with nobody keeping it awake). While starting, a failed compositor ends that Monado
// (its PID only), so the exit goes through the lease retry like any other lease failure.
void RecSession::endFailedMonado() {
    Proc* p = proc(QStringLiteral("monado"));
    if (monadoEnded_ || !p || p->exited || !probe_.readLog) {
        return;
    }
    const QString text = probe_.readLog(p->spec.logPath);
    if (!text.contains(QLatin1String("Failed to init compositor")) &&
        !text.contains(QLatin1String("xrt_instance_create_system failed"))) {
        return;
    }
    monadoEnded_ = true;
    log(QStringLiteral("Monado's compositor could not take the headset's display: ending Monado (PID %1)").arg(p->pid));
    procs_.signal(p->pid, SIGTERM);
}

void RecSession::handleExits(qint64 nowMs) {
    for (const auto& sp : procsList_) {
        Proc& p = *sp;
        if (!p.unhandledExit) {
            continue;
        }
        p.unhandledExit = false;
        log(QStringLiteral("%1 (PID %2) ended: %3").arg(p.spec.name).arg(p.pid).arg(
            p.crashed ? QStringLiteral("crashed or killed") : QStringLiteral("code %1").arg(p.code)));
        if (state_ == RecState::Stopping || !active()) {
            continue;
        }
        const QString how = p.crashed ? QStringLiteral("it crashed") : QStringLiteral("code %1").arg(p.code);
        const QString logText = probe_.readLog ? probe_.readLog(p.spec.logPath) : QString();
        const QString why = logReason(logText);
        const QString tail = why.isEmpty() ? QString() : QStringLiteral(" Last log lines: %1").arg(why);
        if (p.spec.name == QLatin1String("monado")) {
            if (leaseFailure(logText) && !dataSeen_ && retries_ < 1) {
                retries_++;
                state_ = RecState::RetryWait;
                retryAtMs_ = nowMs + kRecLeaseRetryMs;
                message_ = QStringLiteral("The headset's display could not be taken over (the display lease failed). "
                                          "Trying again in 4 s...");
                log(QStringLiteral("display lease failed; retrying in 4 s"));
            } else if (leaseFailure(logText)) {
                fail(nowMs, QStringLiteral("The headset's display could not be taken over, also on the second try "
                                           "(the desktop kept the %1 connector).%2 Try again in a minute; if it keeps "
                                           "failing, restart the headset from the Dashboard.")
                                .arg(setup_.connector, live() ? QString() : QStringLiteral(" Nothing was recorded.")));
            } else if (dataSeen_) {
                fail(nowMs, (live() ? QStringLiteral("Monado stopped by itself (%1).%2")
                                                 : QStringLiteral("Monado stopped by itself during the recording (%1).%2"))
                                .arg(how, tail));
            } else {
                fail(nowMs, QStringLiteral("Monado stopped while starting (%1).%2").arg(how, tail));
            }
        } else if (p.spec.name == QLatin1String("steamvr")) {
            // vrstartup execs into SteamVR's launcher (vrmonitor), which ends once it has handed the start over (code
            // 255 is normal, also before vrserver is up: live 2026-10-19 after a compositor crash it ended at 0.7 s and
            // SteamVR went on starting). Only a crash is a failure; otherwise the wait for the headset decides.
            if (!dataSeen_ && p.crashed) {
                fail(nowMs, QStringLiteral("SteamVR's start script ended (%1) before SteamVR came up.%2").arg(how, tail));
            } else if (!dataSeen_) {
                log(QStringLiteral("the SteamVR launcher ended (%1) before SteamVR came up; waiting for SteamVR").arg(how));
            } else {
                log(QStringLiteral("the SteamVR launcher ended; SteamVR runs on by itself"));
            }
        } else if (p.spec.name == QLatin1String("broker")) {
            fail(nowMs, (live()
                             ? QStringLiteral("The camera broker stopped (%1); without it there is no head or controller "
                                              "tracking.%2")
                             : QStringLiteral("The camera broker stopped (%1); without it no camera data is recorded.%2"))
                            .arg(how, tail));
        } else if (p.spec.name == QLatin1String("winbridge")) {
            // Monado keeps running: its VIT library loses the engine and the head stays 3DoF (orientation only)
            log(QStringLiteral("winbridge (Pimax head tracking) ended (%1); the head is 3DoF from now on.%2")
                    .arg(how, tail));
            message_ = QStringLiteral("Pimax head tracking stopped (%1); the headset follows head turns only. Stop "
                                      "and start tracking again to bring it back.%2")
                           .arg(how, tail);
        } else {
            // the controller tracker is not needed for the dataset or the head: the session goes on
            log(QStringLiteral("the controller tracker ended; the session goes on"));
        }
    }
}

bool RecSession::tick(qint64 nowMs) {
    const RecState before = state_;
    const QString msgBefore = message_;
    const int stepBefore = runner_ ? runner_->current() : -2;
    handleExits(nowMs);
    switch (state_) {
    case RecState::Starting:
        if (phase_ == Phase::Broker) {
            if (probe_.socketReady(setup_.brokerSocket)) {
                log(QStringLiteral("broker socket is up"));
                // the Pimax SLAM engine first: it needs a few seconds to load; Monado's VIT library connects to it
                // and retries every second, so nothing waits for it
                if (winbridgeWanted(setup_) && !startProc(winbridgeSpec(setup_, logDir()), nowMs)) {
                    break;
                }
                // the controller tracker, unless Pimax's LED engine in winbridge tracks the controllers
                if (setup_.io.ctrlTrack && !pimaxCtrlEngine(setup_) && !setup_.ctrl.path.isEmpty() &&
                    !startProc(ctrltrackSpec(setup_, logDir()), nowMs)) {
                    break;
                }
                if (mode_ == Mode::LiveSteamVr) {
                    if (!startProc(steamvrSpec(setup_, logDir()), nowMs)) {
                        break;
                    }
                } else if (!startProc(monadoSpec(setup_, recordDir(), logDir() + QStringLiteral("/monado.log")), nowMs)) {
                    break;
                }
                phase_ = Phase::Monado;
                phaseStartMs_ = nowMs;
                message_ = mode_ == Mode::LiveSteamVr ? QStringLiteral("Starting SteamVR with the Monado driver...")
                           : mode_ == Mode::Live      ? QStringLiteral("Starting Monado...")
                                                      : QStringLiteral("Starting Monado. The headset shows no picture; that is fine.");
            } else if (nowMs - phaseStartMs_ >= kRecSocketWaitMs) {
                fail(nowMs, QStringLiteral("The camera broker did not open its socket in 10 s (see broker.log). Is "
                                           "the headset switched on?"));
            }
        } else if (phase_ == Phase::Monado && mode_ == Mode::LiveSteamVr) {
            if (probe_.steamVrUp && probe_.steamVrUp()) {
                dataSeen_ = true;
                state_ = RecState::Recording;
                record_.recordStartMs = nowMs;
                log(QStringLiteral("SteamVR is up and reports the headset: the inside-out session runs"));
                message_ = QStringLiteral("Tracking runs in SteamVR. Start games as usual (Apps page or Steam); Stop "
                                          "closes SteamVR, the controller tracker and the camera broker.");
            } else if (nowMs - phaseStartMs_ >= kSteamVrStartWaitMs) {
                fail(nowMs, QStringLiteral("SteamVR did not come up with the headset within %1 s (see steamvr.log in "
                                           "this session's logs and Steam's logs/vrserver.txt). Is Steam logged in?")
                                .arg(kSteamVrStartWaitMs / 1000));
            }
        } else if (phase_ == Phase::Monado && mode_ == Mode::Live) {
            endFailedMonado();
            if (monadoEnded_) {
                break;
            }
            if (nowMs - phaseStartMs_ >= kIoSettleMs) {
                dataSeen_ = true;
                state_ = RecState::Recording;
                record_.recordStartMs = nowMs;
                log(QStringLiteral("Monado is up: the inside-out session runs"));
                message_ = QStringLiteral("Tracking runs. Start an app on the Apps page; Stop ends Monado, the "
                                          "controller tracker and the camera broker.");
            }
        } else if (phase_ == Phase::Monado) {
            endFailedMonado();
            if (monadoEnded_) {
                break;
            }
            if (probe_.imuBytes(folder_) > kRecImuHeaderBytes) {
                dataSeen_ = true;
                state_ = RecState::Recording;
                record_.recordStartMs = nowMs;
                record_.startWall = QDateTime::currentDateTime()
                                        .toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
                                        .toString(Qt::ISODateWithMs);
                log(QStringLiteral("data is coming in: recording"));
                if (runner_) {
                    runner_->start(nowMs);
                    sayStep();
                    message_ = QStringLiteral("Recording. Follow the steps.");
                } else {
                    message_ = QStringLiteral("Recording. Press Stop when you are done.");
                    if (say_) {
                        say_(QStringLiteral("Recording."));
                    }
                }
                writeSteps();
            } else if (nowMs - phaseStartMs_ >= kRecDataWaitMs) {
                fail(nowMs, QStringLiteral("Monado runs but wrote no camera or IMU data in 30 s (see monado.log)."));
            }
        }
        break;
    case RecState::RetryWait:
        if (nowMs >= retryAtMs_) {
            log(QStringLiteral("starting Monado again"));
            monadoEnded_ = false;
            state_ = RecState::Starting;
            phase_ = Phase::Monado;
            phaseStartMs_ = nowMs;
            message_ = QStringLiteral("Starting Monado again (second try)...");
            startProc(monadoSpec(setup_, recordDir(), logDir() + QStringLiteral("/monado-%1.log").arg(retries_ + 1)), nowMs);
        }
        break;
    case RecState::Recording:
        if (runner_ && runner_->tick(nowMs)) {
            if (runner_->finished()) {
                stop(nowMs, QStringLiteral("finished"));
            } else {
                sayStep();
                writeSteps();
            }
        }
        break;
    case RecState::Stopping:
        stepStop(nowMs);
        break;
    default:
        break;
    }
    return state_ != before || message_ != msgBefore || (runner_ ? runner_->current() : -2) != stepBefore ||
           state_ == RecState::Recording;
}

void RecSession::skipStep(qint64 nowMs) {
    if (state_ != RecState::Recording || !runner_) {
        return;
    }
    runner_->skip(nowMs);
    log(QStringLiteral("step skipped"));
    if (runner_->finished()) {
        stop(nowMs, QStringLiteral("finished"));
    } else {
        sayStep();
        writeSteps();
    }
}

void RecSession::stop(qint64 nowMs, const QString& reason) {
    if (state_ != RecState::Starting && state_ != RecState::RetryWait && state_ != RecState::Recording) {
        return;
    }
    record_.endReason = reason;
    record_.stoppedEarly = reason != QLatin1String("finished");
    log(QStringLiteral("stop: %1").arg(reason));
    beginStop(nowMs);
}

void RecSession::beginStop(qint64 nowMs) {
    if (runner_ && runner_->running()) {
        runner_->stop(nowMs);
    }
    if (dataSeen_ && record_.recordEndMs < 0) {
        record_.recordEndMs = nowMs;
    }
    state_ = RecState::Stopping;
    if (!failed_) {
        message_ = live() ? QStringLiteral("Stopping...") : QStringLiteral("Stopping and saving...");
    }
    stopIdx_ = static_cast<int>(procsList_.size()) - 1;
    termAtMs_ = -1;
    killAtMs_ = -1;
    writeSteps();
    stepStop(nowMs);
}

void RecSession::stepStop(qint64 nowMs) {
    while (stopIdx_ >= 0) {
        Proc& p = *procsList_[static_cast<size_t>(stopIdx_)];
        const auto next = [this] {
            stopIdx_--;
            termAtMs_ = -1;
            killAtMs_ = -1;
        };
        handleExits(nowMs);
        if (p.exited) {
            next();
            continue;
        }
        if (termAtMs_ < 0) {
            termAtMs_ = nowMs;
            log(QStringLiteral("SIGTERM to %1 (PID %2)").arg(p.spec.name).arg(p.pid));
            procs_.signal(p.pid, SIGTERM);
            continue; // it may be gone at once
        }
        const qint64 wait = p.spec.name == QLatin1String("monado") ? kRecMonadoTermMs : kRecOtherTermMs;
        if (killAtMs_ < 0) {
            if (nowMs - termAtMs_ < wait) {
                return;
            }
            killAtMs_ = nowMs;
            log(QStringLiteral("SIGKILL to %1 (PID %2): still there after %3 s").arg(p.spec.name).arg(p.pid).arg(wait / 1000));
            procs_.signal(p.pid, SIGKILL);
            continue;
        }
        if (nowMs - killAtMs_ < kRecKillWaitMs) {
            return;
        }
        notStopped_ << QStringLiteral("%1 (PID %2)").arg(QFileInfo(p.spec.program).fileName()).arg(p.pid);
        log(QStringLiteral("%1 (PID %2) did not end").arg(p.spec.name).arg(p.pid));
        next();
    }
    finish(nowMs);
}

void RecSession::finish(qint64) {
    if (live()) {
        QString notStopped;
        if (!notStopped_.isEmpty()) {
            notStopped = QStringLiteral(" Could not stop: %1.").arg(notStopped_.join(QStringLiteral(", ")));
        }
        state_ = failed_ ? RecState::Failed : RecState::Finished;
        const bool byUser = record_.endReason == QLatin1String("stopped by the user");
        message_ = (failed_  ? failReason_
                    : byUser ? QStringLiteral("Stopped.")
                             : QStringLiteral("Stopped: %1").arg(record_.endReason)) +
                   notStopped;
        log(QStringLiteral("session ended"));
        return;
    }
    copyTrackerSlamFiles();
    result_ = checkRecording(folder_);
    record_.imuFirstNs = result_->imuFirstNs;
    record_.imuLastNs = result_->imuLastNs;
    record_.dataset = result_->dataset;
    writeSteps();
    log(QStringLiteral("check: %1").arg(result_->summary));
    QString notStopped;
    if (!notStopped_.isEmpty()) {
        notStopped = QStringLiteral(" Could not stop: %1.").arg(notStopped_.join(QStringLiteral(", ")));
    }
    if (failed_) {
        state_ = RecState::Failed;
        message_ = failReason_;
        if (dataSeen_) {
            message_ += result_->ok ? QStringLiteral(" What was recorded until then is usable.")
                                    : QStringLiteral(" Check of what was recorded: %1").arg(result_->summary);
        }
    } else {
        state_ = RecState::Finished;
        message_ = result_->ok ? QStringLiteral("Done. OK: usable (%1).").arg(durationWords(result_->durationS))
                               : QStringLiteral("Done, but the check found a problem: %1").arg(result_->summary);
        if (say_) {
            say_(mode_ == Mode::Guided ? QStringLiteral("Done. Recording stopped.") : QStringLiteral("Recording stopped."));
        }
    }
    message_ += notStopped;
}

void RecSession::copyTrackerSlamFiles() {
    // Monado's recorder writes the driver's own calibration as <folder>/pimax-basalt-calib.json, but with SLAM_CONFIG
    // the live tracker used the calibration that config names (PiBridge's copy, with the factory accel bias). Put exactly
    // that file there, so a replay uses what the tracker used; the driver's file is kept as *.driver.json.
    const QString toml = setup_.sessionSlamConfig.isEmpty() ? setup_.slamConfig.path : setup_.sessionSlamConfig;
    if (!setup_.io.slam || setup_.basalt.path.isEmpty() || toml.isEmpty()) {
        return;
    }
    const SlamTomlFiles f = slamTomlFiles(toml);
    const auto put = [this](const QString& from, const QString& name, const QString& keepAs) {
        if (from.isEmpty() || !QFileInfo(from).isFile()) {
            return;
        }
        const QString to = folder_ + QLatin1Char('/') + name;
        if (!keepAs.isEmpty() && QFileInfo::exists(to)) {
            QFile::remove(folder_ + QLatin1Char('/') + keepAs);
            QFile::rename(to, folder_ + QLatin1Char('/') + keepAs);
        }
        QFile::remove(to);
        if (QFile::copy(from, to)) {
            log(QStringLiteral("copied the tracker's %1 from %2").arg(name, from));
        }
    };
    put(f.calib, QStringLiteral("pimax-basalt-calib.json"), QStringLiteral("pimax-basalt-calib.driver.json"));
    put(f.config, QStringLiteral("pimax-basalt-config.json"), QString());
}

void RecSession::shutdown() {
    if (!active()) {
        return;
    }
    if (runner_ && runner_->running()) {
        runner_->stop(0);
    }
    log(QStringLiteral("PiBridge is closing: stopping what it started"));
    for (auto it = procsList_.rbegin(); it != procsList_.rend(); ++it) {
        Proc& p = **it;
        if (p.exited) {
            continue;
        }
        procs_.signal(p.pid, SIGTERM);
        const int wait = p.spec.name == QLatin1String("monado") ? int(kRecMonadoTermMs) : int(kRecOtherTermMs);
        if (!procs_.waitExit(p.pid, wait)) {
            procs_.signal(p.pid, SIGKILL);
            procs_.waitExit(p.pid, int(kRecKillWaitMs));
        }
    }
    record_.endReason = QStringLiteral("PiBridge was closed");
    record_.stoppedEarly = true;
    writeSteps();
    state_ = RecState::Finished;
}

ChipText recChip(RecState s, const std::optional<RecordingInfo>& result, bool ready) {
    switch (s) {
    case RecState::Starting:
    case RecState::RetryWait: return {QStringLiteral("STARTING"), ChipKind::Warn};
    case RecState::Recording: return {QStringLiteral("RECORDING"), ChipKind::Error};
    case RecState::Stopping: return {QStringLiteral("STOPPING"), ChipKind::Warn};
    case RecState::Failed: return {QStringLiteral("FAILED"), ChipKind::Error};
    case RecState::Finished:
        if (result && !result->ok) {
            return {QStringLiteral("CHECK: PROBLEM"), ChipKind::Warn};
        }
        return {QStringLiteral("DONE"), ChipKind::Info};
    case RecState::Idle: break;
    }
    return ready ? ChipText{QStringLiteral("READY"), ChipKind::Info} : ChipText{QStringLiteral("NOT READY"), ChipKind::Off};
}

// ---------------------------------------------------------------- speech

Announcer::Announcer(Runner& runner, std::function<QString(const QString&)> which) : runner_(runner) {
    spd_ = which(QStringLiteral("spd-say"));
    if (spd_.isEmpty()) {
        player_ = which(QStringLiteral("pw-play"));
        if (player_.isEmpty()) {
            player_ = which(QStringLiteral("paplay"));
        }
    }
}

QString Announcer::how() const {
    if (!spd_.isEmpty()) {
        return QStringLiteral("spd-say");
    }
    if (!player_.isEmpty()) {
        return QStringLiteral("%1 (tone)").arg(QFileInfo(player_).fileName());
    }
    return QStringLiteral("none");
}

QByteArray Announcer::toneWav() {
    constexpr int rate = 22050;
    constexpr int n = rate / 4;
    QByteArray pcm;
    pcm.reserve(n * 2);
    for (int i = 0; i < n; i++) {
        const double env = std::min(1.0, std::min(i, n - i) / 400.0); // no clicks
        const auto v = static_cast<qint16>(std::lround(9000.0 * env * std::sin(2.0 * M_PI * 880.0 * i / rate)));
        const qint16 le = qToLittleEndian(v);
        pcm.append(reinterpret_cast<const char*>(&le), 2);
    }
    QByteArray w;
    const auto u32 = [&w](quint32 x) {
        const quint32 le = qToLittleEndian(x);
        w.append(reinterpret_cast<const char*>(&le), 4);
    };
    const auto u16 = [&w](quint16 x) {
        const quint16 le = qToLittleEndian(x);
        w.append(reinterpret_cast<const char*>(&le), 2);
    };
    w += "RIFF";
    u32(36 + quint32(pcm.size()));
    w += "WAVEfmt ";
    u32(16);
    u16(1);
    u16(1);
    u32(rate);
    u32(rate * 2);
    u16(2);
    u16(16);
    w += "data";
    u32(quint32(pcm.size()));
    w += pcm;
    return w;
}

void Announcer::say(const QString& text) {
    if (text.isEmpty()) {
        return;
    }
    if (!spd_.isEmpty()) {
        // priority "text": a new line replaces one still being read
        runner_.startDetached(spd_, {QStringLiteral("-P"), QStringLiteral("text"), QStringLiteral("-r"),
                                     QStringLiteral("-15"), text});
        return;
    }
    if (player_.isEmpty()) {
        return;
    }
    if (tone_.isEmpty()) {
        const QString p = QDir::tempPath() + QStringLiteral("/pibridge-tone-%1.wav").arg(::getuid());
        QFile f(p);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate) || f.write(toneWav()) < 0) {
            return;
        }
        tone_ = p;
    }
    runner_.startDetached(player_, {tone_});
}

// ---------------------------------------------------------------- made-up recordings

bool writeFakeRecording(const QString& folder, double seconds, bool withGap, const QString& mode, qint64 startNs) {
    const QString root = folder + QStringLiteral("/euroc_") +
                         QFileInfo(folder).fileName().replace(QLatin1Char('-'), QString());
    const qint64 n = qint64(seconds * 1000);
    std::vector<ImuRow> imu;
    imu.reserve(size_t(n));
    qint64 t = startNs;
    for (qint64 i = 0; i < n; i++) {
        t += (withGap && i == n / 2) ? 35000000 : 1000000;
        const double a = 0.02 * std::sin(double(i) * 0.01);
        imu.push_back({t, {0.001, a, 0.0}, {a, 9.81, 0.05}});
    }
    std::vector<std::pair<qint64, QString>> cam;
    for (qint64 c = startNs + 20000000; c < imu.back().ts; c += 40000000) {
        cam.emplace_back(c, QStringLiteral("%1.png").arg(c));
    }
    if (!writeEurocCsvs(root, imu, {cam, cam})) {
        return false;
    }
    for (int i = 0; i < 2; i++) {
        for (const auto& [ts, name] : cam) {
            QFile f(root + QStringLiteral("/mav0/cam%1/data/").arg(i) + name);
            if (!f.open(QIODevice::WriteOnly)) {
                return false;
            }
        }
    }
    QDir().mkpath(folder + QStringLiteral("/logs"));
    QFile log(folder + QStringLiteral("/logs/session.log"));
    if (log.open(QIODevice::WriteOnly)) {
        log.write("fake mode: made-up recording, nothing was run\n");
    }
    StepsRecord r;
    r.mode = mode;
    r.monoAtMs0Ns = startNs;
    r.recordStartMs = 0;
    r.recordEndMs = qint64(seconds * 1000);
    r.imuFirstNs = imu.front().ts;
    r.imuLastNs = imu.back().ts;
    r.dataset = QFileInfo(root).fileName();
    r.endReason = QStringLiteral("finished");
    if (mode == QLatin1String("guided")) {
        StepRunner run(defaultRecSteps());
        run.start(0);
        run.tick(r.recordEndMs);
        return writeStepsJson(folder + QLatin1Char('/') + QString::fromLatin1(kStepsFile), r, &run);
    }
    return writeStepsJson(folder + QLatin1Char('/') + QString::fromLatin1(kStepsFile), r, nullptr);
}

} // namespace pibridge::app
