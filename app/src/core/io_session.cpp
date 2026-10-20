// SPDX-License-Identifier: MIT
#include "io_session.h"

#include <QDateTime>

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcessEnvironment>
#include <QSaveFile>
#include <QStandardPaths>
#include <QThread>
#include <cerrno>
#include <csignal>
#include <unistd.h>

#include "paths.h"
#include "steamvr.h"
#include "system.h"

namespace pibridge::app {

namespace {

const QString kRecorderBusy = QStringLiteral("A tracking recording is running (Development page). Stop it first.");

bool writeText(const QString& path, const QString& text) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(text.toUtf8());
    return f.commit();
}

} // namespace

RecReadiness ioStartReadiness(const RecSetup& s, const RecSystem& sys, IoVariant v, const QStringList& blockers) {
    RecReadiness r;
    if (v == IoVariant::SteamVr) {
        if (s.io.mode != TrackingMode::InsideOut) {
            r.blocking << QStringLiteral("The tracking mode is Lighthouse. Switch it to Inside-out first.");
        }
        if (sys.vrserver) {
            r.blocking << QStringLiteral("SteamVR is running. Close it first: PiBridge starts SteamVR itself with the Monado "
                                         "driver, and SteamVR's driver settings can only be changed while it is closed.");
        }
        if (!sys.steamClient) {
            r.blocking << QStringLiteral("Steam is not running. Start Steam and log in first: a SteamVR started without "
                                         "Steam starts Steam itself with SteamVR's settings, and that Steam can hang.");
        }
        if (s.steamVrStartup.isEmpty()) {
            r.blocking << QStringLiteral("SteamVR was not found (no runtime in ~/.config/openvr/openvrpaths.vrpath). "
                                         "Install SteamVR and start it once from Steam.");
        }
        if (!s.monado.path.isEmpty() && s.steamVrDriverDir.isEmpty()) {
            r.blocking << QStringLiteral("The Monado build of %1 has no SteamVR driver (steamvr-monado/). Choose another "
                                         "build on the Development page.")
                              .arg(s.monado.path);
        } else if (!s.steamVrDriverDir.isEmpty() && s.steamVrPreload.isEmpty()) {
            r.blocking << QStringLiteral("pimax_rttime_preload.so is missing in %1/bin/linux64: without it SteamVR's "
                                         "compositor is killed about 1 s after the start.")
                              .arg(s.steamVrDriverDir);
        }
    }
    r.blocking << blockers;
    const RecReadiness base = recReadiness(s, sys, v == IoVariant::SteamVr ? RecUse::SteamVr : RecUse::Monado);
    r.blocking << base.blocking;
    r.warnings << base.warnings;
    return r;
}

ChipText ioChip(RecState s, bool ready) {
    switch (s) {
    case RecState::Starting:
    case RecState::RetryWait: return {QStringLiteral("STARTING"), ChipKind::Warn};
    case RecState::Recording: return {QStringLiteral("RUNNING"), ChipKind::Info};
    case RecState::Stopping: return {QStringLiteral("STOPPING"), ChipKind::Warn};
    case RecState::Failed: return {QStringLiteral("FAILED"), ChipKind::Error};
    case RecState::Finished: return {QStringLiteral("STOPPED"), ChipKind::Off};
    case RecState::Idle: break;
    }
    return ready ? ChipText{QStringLiteral("READY"), ChipKind::Info} : ChipText{QStringLiteral("NOT READY"), ChipKind::Off};
}

MonadoHeadsetWords monadoHeadsetWords(RecState s, const QString& message) {
    const QString where = QStringLiteral("\"Monado directly\" (Development page) drives the headset; the headset "
                                         "service waits for SteamVR meanwhile.");
    switch (s) {
    case RecState::Starting:
    case RecState::RetryWait: return {QStringLiteral("Monado: starting"), message.isEmpty() ? where : message};
    case RecState::Recording: return {QStringLiteral("Monado: running"), where};
    case RecState::Stopping: return {QStringLiteral("Monado: stopping"), message};
    case RecState::Failed: return {QStringLiteral("Monado: failed"), message};
    case RecState::Finished:
    case RecState::Idle: break;
    }
    return {};
}

QString insideOutMarkerPath() {
    return statusDir() + QStringLiteral("/inside-out.active");
}

QString insideOutMarkerText(qint64 owner, IoVariant v) {
    return QStringLiteral("pibridge-inside-out 1\nowner %1\nvariant %2\n")
        .arg(owner)
        .arg(v == IoVariant::SteamVr ? QStringLiteral("steamvr") : QStringLiteral("monado"));
}

std::optional<std::pair<qint64, QString>> parseInsideOutMarker(const QByteArray& text) {
    const QList<QByteArray> l = text.split('\n');
    if (l.isEmpty() || l[0] != "pibridge-inside-out 1") {
        return std::nullopt;
    }
    qint64 owner = 0;
    QString variant;
    for (const QByteArray& line : l) {
        if (line.startsWith("owner ")) {
            owner = line.mid(6).trimmed().toLongLong();
        } else if (line.startsWith("variant ")) {
            variant = QString::fromLatin1(line.mid(8).trimmed());
        }
    }
    if (owner <= 0) {
        return std::nullopt;
    }
    return std::pair{owner, variant};
}

QStringList daemonsHolding(const QByteArray& keepaliveJson, const QByteArray& eyedJson,
                           const std::function<bool(qint64)>& alive) {
    QStringList out;
    const auto holds = [&alive](const QByteArray& json) {
        const QJsonObject o = QJsonDocument::fromJson(json).object();
        if (o.isEmpty()) {
            return false; // no file: not running
        }
        const qint64 pid = o.value(QStringLiteral("pid")).toInteger(0);
        if (pid <= 0 || (alive && !alive(pid))) {
            return false; // left over from a daemon that is gone
        }
        return o.value(QStringLiteral("state")).toString() != QLatin1String("paused");
    };
    if (holds(keepaliveJson)) {
        out << QStringLiteral("the headset service (pibridge-keepalived)");
    }
    if (holds(eyedJson)) {
        out << QStringLiteral("the eye tracker (pibridge-eyed)");
    }
    return out;
}

std::vector<SteamVrProc> steamVrProcsSince(const SteamVrScan& scan, qint64 sinceTicks) {
    std::vector<SteamVrProc> out;
    for (const SteamVrProc& p : scan.targets()) {
        if (sinceTicks >= 0 && p.startTicks >= sinceTicks) {
            out.push_back(p);
        }
    }
    return out;
}

qint64 bootTicksNow(const QString& procRoot, long ticksPerS) {
    QFile f(procRoot + QStringLiteral("/uptime"));
    if (!f.open(QIODevice::ReadOnly)) {
        return -1;
    }
    bool ok = false;
    const double up = QString::fromLatin1(f.readAll()).section(QLatin1Char(' '), 0, 0).toDouble(&ok);
    const long tps = ticksPerS > 0 ? ticksPerS : std::max(1L, sysconf(_SC_CLK_TCK));
    // one tick earlier: a process started in the same tick as this reading counts as newer
    return ok ? static_cast<qint64>(up * double(tps)) - 1 : -1;
}

QString homeStateOf(const QByteArray& json, qint64 pid) {
    const QJsonObject o = QJsonDocument::fromJson(json).object();
    if (pid <= 0 || o.value(QStringLiteral("pid")).toInteger(-1) != pid) {
        return {}; // an older run's file, or none
    }
    return o.value(QStringLiteral("state")).toString();
}

QString findHomeProgram(const QString& appDir, const std::function<QString(const QString&)>& which) {
    for (const QString& p : {appDir + QStringLiteral("/pibridge-home"), appDir + QStringLiteral("/../tools/home/pibridge-home")}) {
        const QFileInfo f(p);
        if (f.isFile() && f.isExecutable()) {
            return QDir::cleanPath(f.absoluteFilePath());
        }
    }
    return which ? which(QStringLiteral("pibridge-home")) : QString();
}

IoSessionControl::IoSessionControl(Deps d, QObject* parent) : QObject(parent), d_(std::move(d)) {
    timer_.setInterval(100);
    connect(&timer_, &QTimer::timeout, this, &IoSessionControl::tick);
    pendingTimer_.setInterval(2000);
    connect(&pendingTimer_, &QTimer::timeout, this, [this] {
        if (!active()) {
            refresh();
            emit changed();
        }
    });
    refresh(); // also puts SteamVR's drivers back after a crash of the last PiBridge
}

IoSessionControl::~IoSessionControl() {
    shutdown();
    *alive_ = false;
}

IoSessionControl::Deps IoSessionControl::realDeps(IoVariant v, Conf& conf, IoLinks links) {
    Deps d;
    d.variant = v;
    d.procs = std::make_unique<QtRecProcesses>();
    d.setup = [&conf] {
        conf.reload();
        return resolveRecSetup(conf, RecEnv::defaults());
    };
    d.system = [links] {
        RecSystem s = readRecSystem();
        s.vrserver = s.vrserver || (links.steamvrRunning && links.steamvrRunning());
        return s;
    };
    d.probe = RecProbe::real();
    d.probe.steamVrUp = [links] {
        return links.steamvrRunning && links.steamvrRunning() && links.steamvrHmd && links.steamvrHmd();
    };
    auto clock = std::make_shared<QElapsedTimer>();
    clock->start();
    d.nowMs = [clock] { return clock->elapsed(); };
    d.homeProgram = [] {
        return findHomeProgram(QCoreApplication::applicationDirPath(),
                               [](const QString& n) { return QStandardPaths::findExecutable(n); });
    };
    d.runtimeJson = [](const QString& monado) { return monadoRuntimeJsonFor(monado); };
    d.monadoSocket = [] { return QFileInfo::exists(monadoSocketPath(RecEnv::defaults().runtimeDir)); };
    d.homeStatus = [](qint64 pid) {
        QFile f(statusDir() + QStringLiteral("/home.json"));
        return f.open(QIODevice::ReadOnly) ? homeStateOf(f.readAll(), pid) : QString();
    };
    d.vrserverRunning = [links] {
        return links.steamvrRunning ? links.steamvrRunning() : processRunning(QStringLiteral("vrserver"));
    };
    d.roomStatus = [] {
        QFile f(roomStatusPath());
        return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
    };
    d.unixNow = [] { return QDateTime::currentSecsSinceEpoch(); };
    const auto alive = [](qint64 pid) { return pid > 0 && (::kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM); };
    d.marker = [v, alive](bool on) {
        const QString path = insideOutMarkerPath();
        if (on) {
            QDir().mkpath(QFileInfo(path).absolutePath());
            return writeText(path, insideOutMarkerText(QCoreApplication::applicationPid(), v));
        }
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) {
            return true;
        }
        const auto m = parseInsideOutMarker(f.readAll());
        f.close();
        const QString mine = v == IoVariant::SteamVr ? QStringLiteral("steamvr") : QStringLiteral("monado");
        // ours (this PiBridge, this variant), broken, or stale (its owner is gone): removed
        if (!m || !alive(m->first) || (m->first == QCoreApplication::applicationPid() && m->second == mine)) {
            return QFile::remove(path);
        }
        return true;
    };
    d.daemonsHolding = [alive] {
        const auto read = [](const QString& n) {
            QFile f(statusDir() + QLatin1Char('/') + n);
            return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
        };
        return daemonsHolding(read(QStringLiteral("keepalived.json")), read(QStringLiteral("eyed.json")), alive);
    };
    if (v == IoVariant::SteamVr) {
        const auto files = [] {
            const QString vrpath = openVrPathsFile();
            return DriverSwitchFiles{vrpath, steamVrSettingsFileFor(vrpath, QDir::homePath()), steamVrDriverStateFile()};
        };
        const auto sw = [files] {
            return SteamVrDriverSwitch(files(), [] { return processRunning(QStringLiteral("vrserver")); });
        };
        d.applyDrivers = [sw](const DriverPlan& p) { return sw().apply(p); };
        d.previewDrivers = [sw](const DriverPlan& p) { return sw().preview(p); };
        d.restoreDrivers = [sw] { return sw().restore(); };
        d.driversPending = [sw] { return sw().pending(); };
        d.writeShims = [] {
            return writeSteamVrShims(steamVrShimDir(), QProcessEnvironment::systemEnvironment().toStringList(),
                                     [](const QString& n) { return QStandardPaths::findExecutable(n); });
        };
        d.bootTicks = [] { return bootTicksNow(); };
        d.steamVrProcs = [](qint64 since) { return steamVrProcsSince(scanSteamVr(), since); };
        d.procSignals = realProcSignals();
    }
    return d;
}

IoSessionControl::Deps IoSessionControl::fakeDeps(IoVariant v, Conf& conf, IoLinks links) {
    Deps d;
    d.variant = v;
    d.procs = std::make_unique<FakeRecProcesses>();
    d.setup = [&conf] {
        conf.reload();
        RecSetup s = fakeRecSetup(qEnvironmentVariable("XDG_DATA_HOME") + QStringLiteral("/pibridge/recordings"));
        s.io = ioSettings(conf);
        s.refreshHz = conf.refreshHz();
        // a synthetic base config and factory file, so the page shows what PiBridge's copy would hold
        const QString base = qEnvironmentVariable("XDG_STATE_HOME") + QStringLiteral("/pibridge-fake-slam");
        if (QFileInfo::exists(base + QStringLiteral("/base.toml")) || !writeSyntheticSlamBase(base).isEmpty()) {
            s.slamConfig.path = base + QStringLiteral("/base.toml");
            s.calib.path = base + QStringLiteral("/factory.xml");
        }
        return s;
    };
    // SteamVR "comes up" 1.5 s after its launch is first asked about and is "closed" by the stop
    auto clock = std::make_shared<QElapsedTimer>();
    clock->start();
    auto offset = std::make_shared<qint64>(0);
    const auto now = [clock, offset] { return clock->elapsed() + *offset; };
    auto vrSince = std::make_shared<qint64>(-1);
    d.system = [links, vrSince] {
        RecSystem s;
        s.vrserver = (links.steamvrRunning && links.steamvrRunning()) || *vrSince >= 0;
        return s;
    };
    d.nowMs = now;
    d.advance = [offset](qint64 ms) { *offset += ms; };
    auto startedAt = std::make_shared<qint64>(-1);
    d.probe.socketReady = [now, startedAt](const QString&) {
        if (*startedAt < 0) {
            *startedAt = now();
        }
        return now() - *startedAt >= 800;
    };
    d.probe.imuBytes = [](const QString&) -> qint64 { return -1; };
    d.probe.readLog = [](const QString&) { return QString(); };
    d.probe.monoNs = [now] { return now() * 1000000; };
    d.probe.steamVrUp = [now, vrSince] {
        if (*vrSince < 0) {
            *vrSince = now();
        }
        return now() - *vrSince >= 1500;
    };
    d.vrserverRunning = [vrSince] { return *vrSince >= 0; };
    // nothing is looked up or started: the home "runs" in FakeRecProcesses and has its session at once
    d.homeProgram = [] { return QStringLiteral("/fake/bin/pibridge-home"); };
    d.runtimeJson = [](const QString& monado) {
        return monado.isEmpty() ? QString() : QFileInfo(monado).absolutePath() + QStringLiteral("/openxr_monado-dev.json");
    };
    d.monadoSocket = [] { return true; };
    d.homeStatus = [](qint64) { return QStringLiteral("running"); };
    // SteamVR's files are never touched in fake mode: the words a real start would print
    const auto words = [](const DriverPlan& p) -> std::expected<QStringList, QString> {
        return QStringList{QStringLiteral("openvrpaths.vrpath: add %1 to external_drivers").arg(p.driverDir),
                           QStringLiteral("steamvr.vrsettings: driver_CustomHeadsetOpenVR.enable = false (was not set)")};
    };
    d.previewDrivers = words;
    auto pending = std::make_shared<bool>(false);
    d.applyDrivers = [words, pending](const DriverPlan& p) {
        *pending = true;
        return words(p);
    };
    d.restoreDrivers = [pending]() -> std::expected<bool, QString> {
        const bool was = *pending;
        *pending = false;
        return was;
    };
    d.driversPending = [pending] { return *pending; };
    d.writeShims = [] { return QString(); };
    d.bootTicks = [] { return qint64(1000); };
    d.steamVrProcs = [vrSince](qint64) {
        *vrSince = -1; // the fake SteamVR "closes" when it is asked to
        return std::vector<SteamVrProc>{};
    };
    d.procSignals.send = [](int, int) { return 0; };
    d.procSignals.alive = [](int, qint64) { return false; };
    // the room map "searches" in fake mode (the screenshots show the line)
    d.roomStatus = [] { return QByteArray(); };
    d.unixNow = [] { return qint64(1760000000); };
    d.marker = [](bool) { return true; };          // nothing is written in fake mode
    d.daemonsHolding = [] { return QStringList{}; };
    return d;
}

QStringList IoSessionControl::blockers() const {
    QStringList b;
    if (recorderActive_) {
        b << kRecorderBusy;
    }
    if (!otherSession_.isEmpty()) {
        b << otherSession_;
    }
    return b;
}

void IoSessionControl::refresh() {
    if (active()) {
        return; // the running session keeps the setup it started with
    }
    if (d_.variant == IoVariant::SteamVr) {
        restoreDrivers();
    }
    endMarker(); // not running: our marker goes (and a stale one of a crashed PiBridge)
    setup_ = d_.setup ? d_.setup() : RecSetup{};
    readiness_ = ioStartReadiness(setup_, d_.system ? d_.system() : RecSystem{}, d_.variant, blockers());
    if (d_.variant == IoVariant::SteamVr && driversPending()) {
        readiness_.blocking << QStringLiteral("SteamVR's driver settings of the last session are put back first, once "
                                              "SteamVR is closed.");
    }
    homeProgram_ = d_.homeProgram ? d_.homeProgram() : QString();
    runtimeJson_ = d_.runtimeJson ? d_.runtimeJson(setup_.monado.path) : QString();
    if (setup_.build.checking && QCoreApplication::instance()) {
        // the marker scan runs on a worker thread: look again shortly, so "being checked" goes away by itself
        QTimer::singleShot(500, this, [this] {
            if (!active()) {
                refresh();
                emit changed();
            }
        });
    }
}

void IoSessionControl::restoreDrivers() {
    if (!d_.restoreDrivers || !driversPending()) {
        pendingTimer_.stop();
        return;
    }
    const bool vr = d_.vrserverRunning ? d_.vrserverRunning() : false;
    if (vr) {
        driverNote_ = QStringLiteral("SteamVR's driver settings are put back once SteamVR is closed.");
        if (!pendingTimer_.isActive() && QCoreApplication::instance()) {
            pendingTimer_.start();
        }
        return;
    }
    auto r = d_.restoreDrivers();
    if (!r) {
        driverNote_ = QStringLiteral("SteamVR's driver settings could not be put back: %1").arg(r.error());
        if (!pendingTimer_.isActive() && QCoreApplication::instance()) {
            pendingTimer_.start();
        }
        return;
    }
    pendingTimer_.stop();
    if (*r) {
        driverNote_ = QStringLiteral("SteamVR's drivers are as before again (CustomHeadsetOpenVR on).");
    }
}

void IoSessionControl::setRecorderActive(bool on) {
    if (recorderActive_ != on) {
        recorderActive_ = on;
        refresh();
        emit changed();
    }
}

void IoSessionControl::setOtherSession(const QString& why) {
    if (otherSession_ != why) {
        otherSession_ = why;
        refresh();
        emit changed();
    }
}

bool IoSessionControl::active() const {
    return waitingRelease_ || (session_ && session_->active()) || (closer_ && !closeDone_);
}

void IoSessionControl::endMarker() {
    if (d_.marker) {
        (void)d_.marker(false);
    }
}

RecState IoSessionControl::state() const {
    if (closer_ && !closeDone_) {
        return RecState::Stopping;
    }
    if (waitingRelease_) {
        return RecState::Starting;
    }
    if (session_) {
        return session_->state();
    }
    return startError_.isEmpty() ? RecState::Idle : RecState::Failed;
}

QString IoSessionControl::message() const {
    if (closer_ && !closeDone_) {
        return QStringLiteral("Closing SteamVR...");
    }
    if (waitingRelease_) {
        return QStringLiteral("Waiting for the headset service and the eye tracker to let go of the headset...");
    }
    QString m = session_ ? session_->message() : startError_;
    if (d_.variant == IoVariant::SteamVr && !active() && !driverNote_.isEmpty() && (session_ || !startError_.isEmpty())) {
        m += (m.isEmpty() ? QString() : QStringLiteral(" ")) + driverNote_;
    }
    return m;
}

bool IoSessionControl::driversPending() const {
    return d_.driversPending && d_.driversPending();
}

std::expected<QStringList, QString> IoSessionControl::driverPreview() const {
    if (!d_.previewDrivers) {
        return QStringList{};
    }
    DriverPlan p;
    p.driverDir = setup_.steamVrDriverDir;
    return d_.previewDrivers(p);
}

bool IoSessionControl::start() {
    refresh();
    if (active() || !readiness_.ready()) {
        emit changed();
        return false;
    }
    startError_.clear();
    driverNote_.clear();
    homeTried_ = false;
    homePid_ = 0;
    homeExited_ = false;
    homeTermMs_ = -1;
    homeKilled_ = false;
    homeError_.clear();
    roomTried_ = false;
    roomPid_ = 0;
    roomTermMs_ = -1;
    roomKilled_ = false;
    roomSinceUnix_ = 0;
    roomError_.clear();
    closer_.reset();
    closeDone_ = false;
    stopAfterClose_ = false;
    steamVrSeen_ = false;
    QString note;
    prepareSessionSlam(setup_, &note); // PiBridge's Basalt config for these settings
    if (!note.isEmpty()) {
        readiness_.warnings << note;
    }
    session_.reset();
    // 0. the daemons let go of the headset (common/pibridge_pause.h): the marker first, then wait for "paused"
    if (d_.marker && !d_.marker(true)) {
        startError_ = QStringLiteral("The pause marker %1 could not be written; nothing was started.").arg(insideOutMarkerPath());
        emit changed();
        return false;
    }
    if (d_.daemonsHolding && !d_.daemonsHolding().isEmpty()) {
        waitingRelease_ = true;
        releaseDeadline_ = d_.nowMs() + kDaemonReleaseMs;
        if (d_.autoTick) {
            timer_.start();
        }
        emit changed();
        return true;
    }
    const bool ok = continueStart();
    emit changed();
    return ok;
}

bool IoSessionControl::continueStart() {
    if (d_.variant == IoVariant::SteamVr) {
        const bool ok = startSteamVr();
        if (!ok && (!session_ || !session_->active())) {
            endMarker();
        }
        return ok;
    }
    session_ = std::make_unique<RecSession>(setup_, *d_.procs, d_.probe);
    const bool ok = session_->start(RecSession::Mode::Live, {}, d_.nowMs());
    if (ok && d_.autoTick) {
        timer_.start();
    }
    if (!ok) {
        endMarker();
    }
    return ok;
}

bool IoSessionControl::startSteamVr() {
    session_.reset();
    // 1. the Pimax driver's settings for driver_monado.so (vrserver does not get PiBridge's environment)
    if (setup_.driverEnvFile.isEmpty() || !writeText(setup_.driverEnvFile, steamvrDriverEnvText(setup_))) {
        startError_ = QStringLiteral("The driver settings file %1 could not be written; nothing was started.")
                          .arg(setup_.driverEnvFile);
        return false;
    }
    // 2. what SteamVR opens must not inherit SteamVR's variables
    if (d_.writeShims) {
        if (const QString path = d_.writeShims(); !path.isEmpty()) {
            setup_.steamVrExtraEnv = {QStringLiteral("PATH=") + path};
        }
    }
    // 3. the drivers: the Monado build's steamvr-monado in, CustomHeadsetOpenVR off (backups and a record first)
    if (d_.applyDrivers) {
        DriverPlan p;
        p.driverDir = setup_.steamVrDriverDir;
        auto r = d_.applyDrivers(p);
        if (!r) {
            startError_ = QStringLiteral("SteamVR's drivers could not be switched: %1 Nothing was started.").arg(r.error());
            return false;
        }
    }
    // 4. the broker, the tracker, SteamVR (the session)
    launchTicks_ = d_.bootTicks ? d_.bootTicks() : -1;
    session_ = std::make_unique<RecSession>(setup_, *d_.procs, d_.probe);
    const bool ok = session_->start(RecSession::Mode::LiveSteamVr, {}, d_.nowMs());
    if (!ok) {
        restoreDrivers();
        return false;
    }
    if (d_.autoTick) {
        timer_.start();
    }
    return true;
}

void IoSessionControl::beginCloseSteamVr(qint64 now) {
    if (closer_) {
        return;
    }
    std::vector<SteamVrProc> procs = d_.steamVrProcs ? d_.steamVrProcs(launchTicks_) : std::vector<SteamVrProc>{};
    closer_ = std::make_unique<SteamVrCloser>(std::move(procs), d_.procSignals);
    closeDone_ = false;
    closer_->begin(now);
    if (closer_->finished()) {
        closeDone_ = true;
    }
}

void IoSessionControl::stop() {
    if (waitingRelease_) {
        waitingRelease_ = false; // nothing started yet
        endMarker();
        emit changed();
        return;
    }
    if (!session_) {
        return;
    }
    stopRoom(false); // the room map first: it reads the runtime's head ring
    if (d_.variant == IoVariant::SteamVr) {
        if (session_->active() && !closer_) {
            // SteamVR first (it is the tracker's and the broker's client), then what the session started
            stopAfterClose_ = true;
            stopReason_ = QStringLiteral("stopped by the user");
            beginCloseSteamVr(d_.nowMs());
        }
        tick();
        return;
    }
    stopHome(false); // the home first: it is Monado's client
    session_->stop(d_.nowMs());
    tick();
}

void IoSessionControl::tickSteamVr(qint64 now) {
    const RecState st = session_->state();
    const bool vr = d_.vrserverRunning ? d_.vrserverRunning() : false;
    steamVrSeen_ = steamVrSeen_ || (vr && st == RecState::Recording);
    if (closer_ && !closeDone_ && closer_->step(now)) {
        closeDone_ = true;
    }
    if (st == RecState::Recording && steamVrSeen_ && !vr && !closer_) {
        // the user closed SteamVR: what it left (vrmonitor, vrcompositor) is closed by PID, then the rest
        stopAfterClose_ = true;
        stopReason_ = QStringLiteral("SteamVR was closed.");
        beginCloseSteamVr(now);
    }
    if (stopAfterClose_ && (!closer_ || closeDone_)) {
        stopAfterClose_ = false;
        session_->stop(now, stopReason_);
        session_->tick(now);
    }
    // the session ended by itself (the broker died, SteamVR never came up): SteamVR of this start goes too
    if (!session_->active() && !closer_) {
        beginCloseSteamVr(now);
    }
    if (!session_->active() && closer_ && closeDone_) {
        restoreDrivers();
    }
}

void IoSessionControl::tick() {
    if (waitingRelease_) {
        const QStringList holding = d_.daemonsHolding ? d_.daemonsHolding() : QStringList{};
        if (holding.isEmpty()) {
            waitingRelease_ = false;
            continueStart();
        } else if (d_.nowMs() >= releaseDeadline_) {
            waitingRelease_ = false;
            endMarker();
            startError_ = QStringLiteral("%1 did not let go of the headset within %2 s; nothing was started. Press "
                                         "Restart services on the Dashboard and try again.")
                              .arg(holding.join(QStringLiteral(" and ")))
                              .arg(kDaemonReleaseMs / 1000);
            startError_[0] = startError_[0].toUpper();
            timer_.stop();
        }
        emit changed();
        return;
    }
    if (!session_) {
        timer_.stop();
        return;
    }
    const qint64 now = d_.nowMs();
    session_->tick(now);
    if (d_.variant == IoVariant::SteamVr) {
        tickSteamVr(now);
        tickRoom(now);
        if (!active() && (roomPid_ == 0 || roomKilled_)) {
            timer_.stop();
            refresh();
        }
        emit changed();
        return;
    }
    if (session_->state() == RecState::Recording && !homeTried_) {
        startHome();
    }
    if (!session_->active()) {
        stopHome(false); // Monado ended by itself: the home goes too
    }
    if (homePid_ > 0 && homeTermMs_ >= 0 && !homeKilled_ && now - homeTermMs_ >= kRecOtherTermMs) {
        d_.procs->signal(homePid_, SIGKILL); // that PID only
        homeKilled_ = true;
    }
    tickRoom(now);
    if (!session_->active() && (homePid_ == 0 || homeKilled_) && (roomPid_ == 0 || roomKilled_)) {
        timer_.stop();
        refresh();
    }
    emit changed();
}

void IoSessionControl::startHome() {
    homeTried_ = true;
    if (!setup_.io.homeScene || d_.variant != IoVariant::MonadoDirect) {
        return;
    }
    if (homeProgram_.isEmpty() || runtimeJson_.isEmpty()) {
        return; // home() says why
    }
    RecProcSpec spec;
    spec.name = QStringLiteral("home");
    spec.program = homeProgram_;
    spec.env << QStringLiteral("XR_RUNTIME_JSON=") + runtimeJson_;
    spec.logPath = session_->logDir() + QStringLiteral("/home.log");
    QString err;
    const auto alive = alive_;
    std::shared_ptr<qint64> pidBox = std::make_shared<qint64>(0);
    const qint64 pid = d_.procs->start(
        spec,
        [this, alive, pidBox](int code, bool crashed) {
            if (!*alive || *pidBox != homePid_ || homePid_ == 0) {
                return;
            }
            homePid_ = 0;
            homeExited_ = true;
            homeCode_ = code;
            homeCrashed_ = crashed;
            emit changed();
        },
        &err);
    if (pid <= 0) {
        homeExited_ = true;
        homeError_ = err.isEmpty() ? QStringLiteral("pibridge-home could not start.") : err;
        return;
    }
    *pidBox = pid;
    homePid_ = pid;
    homeStartMs_ = d_.nowMs();
}

void IoSessionControl::stopHome(bool wait) {
    if (homePid_ <= 0) {
        return;
    }
    const qint64 pid = homePid_;
    if (homeTermMs_ < 0) {
        d_.procs->signal(pid, SIGTERM); // that PID only; it ends its OpenXR session and exits
        homeTermMs_ = d_.nowMs();
    }
    if (wait && homePid_ == pid && !d_.procs->waitExit(pid, static_cast<int>(kRecOtherTermMs))) {
        d_.procs->signal(pid, SIGKILL);
        homeKilled_ = true;
        d_.procs->waitExit(pid, static_cast<int>(kRecKillWaitMs));
    }
}

// ---------------------------------------------------------------- the room map

void IoSessionControl::tickRoom(qint64 now) {
    if (session_ && session_->state() == RecState::Recording && !roomTried_ && !closer_) {
        startRoom();
    }
    if (!session_ || !session_->active() || closer_) {
        stopRoom(false); // the session ends: the room map goes too
    }
    if (roomPid_ > 0 && roomTermMs_ >= 0 && !roomKilled_ && now - roomTermMs_ >= kRecOtherTermMs) {
        d_.procs->signal(roomPid_, SIGKILL); // that PID only
        roomKilled_ = true;
    }
}

void IoSessionControl::startRoom() {
    roomTried_ = true;
    if (!setup_.roomSaved || setup_.roommap.isEmpty()) {
        return; // no saved room (or no tool): nothing changes; room() says so
    }
    RecProcSpec spec;
    spec.name = QStringLiteral("roommap");
    spec.program = setup_.roommap;
    spec.args = roomAutoArgs(setup_.roomName);
    spec.logPath = session_->logDir() + QStringLiteral("/roommap.log");
    QString err;
    const auto alive = alive_;
    std::shared_ptr<qint64> pidBox = std::make_shared<qint64>(0);
    roomSinceUnix_ = d_.unixNow ? d_.unixNow() : 0;
    const qint64 pid = d_.procs->start(
        spec,
        [this, alive, pidBox](int, bool) {
            if (!*alive || *pidBox != roomPid_ || roomPid_ == 0) {
                return;
            }
            roomPid_ = 0; // it ends by itself once the room is in place or the wait is over
            emit changed();
        },
        &err);
    if (pid <= 0) {
        roomError_ = err.isEmpty() ? QStringLiteral("pibridge-roommap could not start.") : err;
        return;
    }
    *pidBox = pid;
    roomPid_ = pid;
}

void IoSessionControl::stopRoom(bool wait) {
    if (roomPid_ <= 0) {
        return;
    }
    const qint64 pid = roomPid_;
    if (roomTermMs_ < 0) {
        d_.procs->signal(pid, SIGTERM); // that PID only
        roomTermMs_ = d_.nowMs();
    }
    if (wait && roomPid_ == pid && !d_.procs->waitExit(pid, static_cast<int>(kRecOtherTermMs))) {
        d_.procs->signal(pid, SIGKILL);
        roomKilled_ = true;
        d_.procs->waitExit(pid, static_cast<int>(kRecKillWaitMs));
    }
}

RoomWords IoSessionControl::room() const {
    if (!roomError_.isEmpty()) {
        RoomWords w;
        w.look = RoomLook::NotFound;
        w.headline = QStringLiteral("Room map did not start");
        w.detail = roomError_;
        return w;
    }
    const auto st = d_.roomStatus ? parseRoomStatus(d_.roomStatus()) : std::nullopt;
    const bool running = active() && roomTried_;
    return roomStatusWords(st, 0, running ? roomSinceUnix_ : 0, setup_.roomSaved, roomPid_ > 0, setup_.roomName);
}

bool IoSessionControl::homeSettled() const {
    if (!homeTried_) {
        // not started yet: settled only when it will not start
        return !setup_.io.homeScene || homeProgram_.isEmpty() || runtimeJson_.isEmpty();
    }
    if (homePid_ <= 0) {
        return true; // ended, or could not start
    }
    const QString st = d_.homeStatus ? d_.homeStatus(homePid_) : QString();
    if (!st.isEmpty() && st != QLatin1String("starting")) {
        return true;
    }
    return d_.nowMs() - homeStartMs_ >= kHomeSettleMs;
}

HomeState IoSessionControl::home() const {
    HomeState h;
    if (!setup_.io.homeScene) {
        h.words = QStringLiteral("Off.");
        return h;
    }
    if (homeProgram_.isEmpty()) {
        h.run = HomeRun::NotFound;
        h.words = QStringLiteral("pibridge-home was not found next to pibridge.");
        return h;
    }
    if (runtimeJson_.isEmpty()) {
        h.run = HomeRun::NoRuntimeFile;
        h.words = QStringLiteral("The Monado build has no openxr_monado-dev.json: the home scene cannot find Monado.");
        return h;
    }
    if (homePid_ > 0) {
        h.run = HomeRun::Running;
        h.pid = homePid_;
        h.words = QStringLiteral("In the headset (PID %1). A game you start is shown instead.").arg(homePid_);
        return h;
    }
    if (homeExited_) {
        h.run = HomeRun::Exited;
        h.words = !homeError_.isEmpty() ? homeError_
                  : homeCrashed_        ? QStringLiteral("Crashed; it starts again with the next session.")
                                        : QStringLiteral("Ended (code %1).").arg(homeCode_);
        return h;
    }
    h.run = HomeRun::Waiting;
    h.words = QStringLiteral("Starts once Monado runs.");
    return h;
}

QString IoSessionControl::runtimeJson() const {
    return runtimeJson_;
}

MonadoReadiness IoSessionControl::monadoReadiness() const {
    MonadoInputs in;
    in.runtimeJson = runtimeJson_;
    in.state = state();
    in.socket = in.state == RecState::Recording && d_.monadoSocket && d_.monadoSocket();
    in.homeSettled = homeSettled();
    in.message = message();
    return evaluateMonadoReady(in);
}

void IoSessionControl::demoRun() {
    if (!d_.advance || !start()) {
        return;
    }
    tick(); // the socket wait starts
    d_.advance(1000);
    tick(); // broker up: tracker and Monado (or SteamVR) start
    tick(); // SteamVR is asked about for the first time
    d_.advance(std::max(kIoSettleMs, qint64(1500)) + 100);
    tick(); // Monado stayed up / SteamVR reports the headset: running
}

void IoSessionControl::shutdown() {
    stopRoom(true);
    stopHome(true);
    if (d_.variant == IoVariant::SteamVr && session_ && session_->active()) {
        // PiBridge closes: SteamVR of this start first (TERM, KILL after 5 s), waiting here as the recorder does
        QElapsedTimer t;
        t.start();
        beginCloseSteamVr(0);
        while (closer_ && !closeDone_ && t.elapsed() < kTermWaitMs + kKillWaitMs + 500) {
            if (closer_->step(t.elapsed())) {
                closeDone_ = true;
                break;
            }
            QThread::msleep(100);
        }
        closeDone_ = true;
    }
    if (session_) {
        session_->shutdown();
    }
    if (d_.variant == IoVariant::SteamVr) {
        restoreDrivers();
    }
    waitingRelease_ = false;
    endMarker();
}

AppTarget TrackingRuntime::target() const {
    return direct_ && direct_->active() ? AppTarget::Monado : AppTarget::SteamVr;
}

MonadoReadiness TrackingRuntime::monado() const {
    return direct_ ? direct_->monadoReadiness() : MonadoReadiness{};
}

bool TrackingRuntime::steamVrThroughTracking() const {
    conf_.reload();
    return ioSettings(conf_).mode == TrackingMode::InsideOut;
}

bool TrackingRuntime::startTracking(QString* why) {
    IoSessionControl* c = target() == AppTarget::Monado ? direct_ : &tracking_;
    if (!c) {
        return false;
    }
    if (c->active()) {
        return true;
    }
    c->refresh();
    if (!c->readiness().ready()) {
        if (why) {
            *why = c->readiness().blocking.join(QLatin1Char(' '));
        }
        return false;
    }
    if (!c->start()) {
        if (why) {
            *why = c->message();
        }
        return false;
    }
    return true;
}

} // namespace pibridge::app
