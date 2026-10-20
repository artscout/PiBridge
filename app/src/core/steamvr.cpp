// SPDX-License-Identifier: MIT
// Everything of the SteamVR status but the OpenVR backend (that one is steamvr_openvr.cpp).
#include "steamvr.h"

#include "paths.h"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>

namespace pibridge::app {

// ---------------------------------------------------------------- fake source

VrConnect FakeSteamVrSource::connect() {
    const size_t i = static_cast<size_t>(connectCalls++);
    const VrConnect c = connects.empty() ? VrConnect{VrConnect::Ok, {}} : connects[std::min(i, connects.size() - 1)];
    connected = c.result == VrConnect::Ok;
    return c;
}

std::optional<std::vector<VrDevice>> FakeSteamVrSource::poll() {
    const size_t i = static_cast<size_t>(pollCalls++);
    if (polls.empty()) {
        return std::vector<VrDevice>{};
    }
    return polls[std::min(i, polls.size() - 1)];
}

void FakeSteamVrSource::disconnect() {
    disconnectCalls++;
    connected = false;
}

std::vector<VrDevice> fakeVrDevices() {
    std::vector<VrDevice> d;
    VrDevice hmd;
    hmd.index = 0;
    hmd.cls = VrClass::Hmd;
    hmd.model = QStringLiteral("Crystal Super");
    hmd.poseValid = true;
    hmd.trackingResult = vrtrack::RunningOk;
    hmd.activity = vractivity::UserInteraction;
    hmd.universe = 1111111111; // the fake chaperone file's first universe (Room page)
    hmd.serial = QStringLiteral("HMD-FAKE0000"); // made up: fake mode and screenshots
    hmd.ipdMeters = 0.0635f;
    hmd.displayHz = 90.0f;
    hmd.renderWidth = 3480;
    hmd.renderHeight = 3480;
    d.push_back(hmd);

    VrDevice left;
    left.index = 1;
    left.cls = VrClass::Controller;
    left.role = VrRole::Left;
    left.model = QStringLiteral("Knuckles Left");
    left.controllerType = QStringLiteral("knuckles");
    left.serial = QStringLiteral("LHR-FAKE0001"); // made up: fake mode and screenshots
    left.hasBattery = true;
    left.battery = 0.82f;
    left.charging = true;
    left.poseValid = true;
    left.trackingResult = vrtrack::RunningOk;
    left.activity = vractivity::UserInteraction;
    d.push_back(left);

    VrDevice right = left;
    right.index = 2;
    right.role = VrRole::Right;
    right.model = QStringLiteral("Knuckles Right");
    right.serial = QStringLiteral("LHR-FAKE0002");
    right.battery = 0.45f;
    right.charging = false;
    right.poseValid = false;
    right.trackingResult = vrtrack::RunningOutOfRange;
    right.activity = vractivity::Idle;
    d.push_back(right);

    for (int i = 3; i <= 4; i++) {
        VrDevice st;
        st.index = i;
        st.cls = VrClass::TrackingReference;
        st.model = QStringLiteral("Valve SR Imp");
        st.serial = QStringLiteral("LHB-FAKE000%1").arg(i - 2);
        st.poseValid = true;
        st.trackingResult = vrtrack::RunningOk;
        st.activity = vractivity::UserInteraction;
        d.push_back(st);
    }
    return d;
}

// ---------------------------------------------------------------- where SteamVR is

QString openVrPathsFile() {
    QString base = qEnvironmentVariable("XDG_CONFIG_HOME");
    if (base.isEmpty()) {
        base = QDir::homePath() + QStringLiteral("/.config");
    }
    return base + QStringLiteral("/openvr/openvrpaths.vrpath");
}

QString openVrRuntimeDir(const QString& vrpathFile) {
    QFile f(vrpathFile);
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) {
        return {};
    }
    const QJsonArray rt = doc.object().value(QStringLiteral("runtime")).toArray();
    if (rt.isEmpty() || !rt.first().isString() || rt.first().toString().isEmpty()) {
        return {};
    }
    return QDir::cleanPath(rt.first().toString());
}

QString openVrApiLibrary(const QString& runtimeDir) {
    return runtimeDir.isEmpty() ? QString() : runtimeDir + QStringLiteral("/bin/linux64/libopenvr_api.so");
}

// ---------------------------------------------------------------- link

SteamVrLink::~SteamVrLink() {
    stop();
}

void SteamVrLink::stop() {
    if (connected_) {
        source_.disconnect();
        connected_ = false;
    }
}

SteamVrStatus SteamVrLink::tick(bool vrserverRunning) {
    if (!vrserverRunning) {
        // vrserver is gone: drop the connection; never try while it is not running. The time of the last attempt
        // is kept, so a quick restart of SteamVR still waits out kRetryMs.
        stop();
        last_ = {};
        return last_;
    }
    const qint64 now = clock_();
    if (connected_) {
        if (auto devices = source_.poll()) {
            last_ = {VrLinkState::Connected, {}, std::move(*devices), source_.compositorPid()};
            return last_;
        }
        // VREvent_Quit or the connection broke: let go, wait before the next try.
        stop();
        lastAttemptMs_ = now;
        last_ = {VrLinkState::Waiting, QStringLiteral("SteamVR is closing."), {}};
        return last_;
    }
    if (lastAttemptMs_ && now - *lastAttemptMs_ < kRetryMs) {
        if (last_.state == VrLinkState::NotRunning) { // vrserver came back within the back-off
            last_.state = VrLinkState::Waiting;
        }
        last_.devices.clear();
        return last_;
    }
    lastAttemptMs_ = now;
    const VrConnect c = source_.connect();
    switch (c.result) {
    case VrConnect::Ok:
        connected_ = true;
        if (auto devices = source_.poll()) {
            last_ = {VrLinkState::Connected, {}, std::move(*devices), source_.compositorPid()};
        } else {
            stop();
            last_ = {VrLinkState::Waiting, QStringLiteral("SteamVR is closing."), {}};
        }
        break;
    case VrConnect::NotInstalled:
        last_ = {VrLinkState::NotInstalled, c.error, {}};
        break;
    case VrConnect::Failed:
        last_ = {VrLinkState::Waiting, c.error, {}};
        break;
    }
    return last_;
}

// ---------------------------------------------------------------- worker

SteamVrWorker::SteamVrWorker(std::unique_ptr<SteamVrSource> source, std::function<bool()> vrserverRunning,
                             Deliver deliver, int intervalMs)
    : source_(std::move(source)), vrserverRunning_(std::move(vrserverRunning)), deliver_(std::move(deliver)),
      intervalMs_(intervalMs) {
    thread_ = std::jthread([this](std::stop_token st) {
        SteamVrLink link(*source_, [] {
            return static_cast<qint64>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::steady_clock::now().time_since_epoch())
                                           .count());
        });
        std::mutex m;
        std::condition_variable_any cv;
        while (!st.stop_requested()) {
            deliver_(link.tick(vrserverRunning_()));
            std::unique_lock lk(m);
            cv.wait_for(lk, st, std::chrono::milliseconds(intervalMs_), [] { return false; });
        }
        link.stop(); // VR_ShutdownInternal on the thread that connected
    });
}

SteamVrWorker::~SteamVrWorker() {
    thread_.request_stop();
    if (thread_.joinable()) {
        thread_.join();
    }
}

// ---------------------------------------------------------------- words

QString vrDeviceName(const VrDevice& d) {
    const QString type = d.controllerType.toLower();
    if (type == QLatin1String("knuckles")) {
        return QStringLiteral("Valve Index Controller");
    }
    if (type == QLatin1String("vive_controller")) {
        return QStringLiteral("HTC Vive Controller");
    }
    if (type.startsWith(QLatin1String("vive_tracker"))) {
        return QStringLiteral("Vive Tracker");
    }
    if (type == QLatin1String("oculus_touch")) {
        return QStringLiteral("Oculus Touch Controller");
    }
    if (d.cls == VrClass::TrackingReference) {
        if (d.model.contains(QLatin1String("SR Imp"))) {
            return QStringLiteral("SteamVR Base Station 2.0");
        }
        if (d.model.startsWith(QLatin1String("HTC V2"))) {
            return QStringLiteral("SteamVR Base Station 1.0");
        }
    }
    if (!d.model.isEmpty()) {
        return d.model;
    }
    switch (d.cls) {
    case VrClass::Hmd: return QStringLiteral("Headset");
    case VrClass::Controller: return QStringLiteral("Controller");
    case VrClass::TrackingReference: return QStringLiteral("Base station");
    case VrClass::GenericTracker: return QStringLiteral("Tracker");
    case VrClass::Other: break;
    }
    return QStringLiteral("Device");
}

QString vrTrackingWords(int r) {
    switch (r) {
    case vrtrack::RunningOk: return QStringLiteral("Tracking");
    case vrtrack::RunningOutOfRange: return QStringLiteral("Out of range");
    case vrtrack::CalibratingInProgress: return QStringLiteral("Calibrating");
    case vrtrack::CalibratingOutOfRange: return QStringLiteral("Calibrating, out of range");
    case vrtrack::FallbackRotationOnly: return QStringLiteral("Rotation only");
    case vrtrack::Uninitialized: return QStringLiteral("Not seen");
    default: return QStringLiteral("Not tracked");
    }
}

namespace {

bool tracked(const VrDevice& d) {
    return d.poseValid && d.trackingResult == vrtrack::RunningOk;
}

QString batteryText(const VrDevice& d) {
    if (!d.hasBattery) {
        return {};
    }
    const int pc = static_cast<int>(std::lround(std::clamp(d.battery, 0.0f, 1.0f) * 100.0f));
    return d.charging ? QStringLiteral("%1 % charging").arg(pc) : QStringLiteral("%1 %").arg(pc);
}

ControllerRow controllerRow(const VrDevice* d) {
    ControllerRow r;
    if (!d) {
        r.chip = {QStringLiteral("DISCONNECTED"), ChipKind::Off};
        r.detail = QStringLiteral("Turn it on, or pair it in SteamVR.");
        return r;
    }
    r.present = true;
    r.name = vrDeviceName(*d);
    r.serial = d->serial;
    if (tracked(*d)) {
        r.chip = {QStringLiteral("CONNECTED"), ChipKind::Info};
    } else {
        r.chip = {QStringLiteral("NOT TRACKED"), ChipKind::Warn};
        r.detail = d->poseValid || d->trackingResult != vrtrack::RunningOk ? vrTrackingWords(d->trackingResult)
                                                                          : QStringLiteral("Not seen");
    }
    if (d->activity == vractivity::Standby) {
        r.detail = r.detail.isEmpty() ? QStringLiteral("Standby") : r.detail + QStringLiteral(", standby");
    }
    if (d->hasBattery) {
        r.batteryPercent = static_cast<int>(std::lround(std::clamp(d->battery, 0.0f, 1.0f) * 100.0f));
        r.charging = d->charging;
        r.batteryText = batteryText(*d);
    }
    return r;
}

} // namespace

ControllersView controllersView(const SteamVrStatus& s, bool steamvrRunning) {
    ControllersView v;
    const VrLinkState st = steamvrRunning ? s.state : VrLinkState::NotRunning;
    switch (st) {
    case VrLinkState::NotRunning:
        v.header = {QStringLiteral("STEAMVR OFF"), ChipKind::Off};
        v.message = QStringLiteral("Start SteamVR to see the controllers.");
        return v;
    case VrLinkState::NotInstalled:
        v.header = {QStringLiteral("NO STEAMVR"), ChipKind::Off};
        v.message = QStringLiteral("SteamVR not installed: the controllers cannot be read.");
        return v;
    case VrLinkState::Waiting:
        v.header = {QStringLiteral("CONNECTING"), ChipKind::Off};
        v.message = s.error.isEmpty() ? QStringLiteral("Connecting to SteamVR...")
                                      : QStringLiteral("Connecting to SteamVR... (%1)").arg(s.error);
        return v;
    case VrLinkState::Connected: break;
    }
    const VrDevice* left = nullptr;
    const VrDevice* right = nullptr;
    std::vector<const VrDevice*> others;
    for (const VrDevice& d : s.devices) {
        if (d.cls == VrClass::Controller && d.role == VrRole::Left && !left) {
            left = &d;
        } else if (d.cls == VrClass::Controller && d.role == VrRole::Right && !right) {
            right = &d;
        } else if (d.cls == VrClass::Controller || d.cls == VrClass::GenericTracker) {
            others.push_back(&d);
        }
    }
    v.left = controllerRow(left);
    v.right = controllerRow(right);
    const int n = (left ? 1 : 0) + (right ? 1 : 0);
    v.header = {QStringLiteral("%1/2 CONNECTED").arg(n),
                n == 2 ? ChipKind::Info : n == 1 ? ChipKind::Warn : ChipKind::Off};
    for (const VrDevice* d : others) {
        QString line = QStringLiteral("%1: %2").arg(vrDeviceName(*d), tracked(*d) ? QStringLiteral("tracking")
                                                                                 : vrTrackingWords(d->trackingResult).toLower());
        if (d->hasBattery) {
            line += QStringLiteral(", battery ") + batteryText(*d);
        }
        v.extra << line;
    }
    return v;
}

TrackingView trackingView(const SteamVrStatus& s, bool steamvrRunning) {
    TrackingView v;
    const VrLinkState st = steamvrRunning ? s.state : VrLinkState::NotRunning;
    switch (st) {
    case VrLinkState::NotRunning:
        v.message = QStringLiteral("Start SteamVR to see the base stations.");
        return v;
    case VrLinkState::NotInstalled:
        v.message = QStringLiteral("SteamVR not installed: the base stations cannot be read.");
        return v;
    case VrLinkState::Waiting:
        v.message = QStringLiteral("Connecting to SteamVR...");
        return v;
    case VrLinkState::Connected: break;
    }
    int tracking = 0;
    for (const VrDevice& d : s.devices) {
        if (d.cls != VrClass::TrackingReference) {
            continue;
        }
        StationRow r;
        r.name = QStringLiteral("Station %1").arg(v.stations.size() + 1);
        r.model = vrDeviceName(d);
        r.serial = d.serial;
        r.tracking = d.poseValid;
        r.text = r.tracking ? QStringLiteral("ok") : QStringLiteral("not seen");
        tracking += r.tracking ? 1 : 0;
        v.stations.push_back(r);
    }
    const auto n = static_cast<int>(v.stations.size());
    if (n == 0) {
        v.summary = QStringLiteral("No base stations seen");
        v.message = QStringLiteral("Turn the base stations on; SteamVR finds them once the headset sees them.");
    } else {
        v.summary = QStringLiteral("%1 base station%2, %3 tracking").arg(n).arg(n == 1 ? QLatin1String("") : QLatin1String("s")).arg(tracking);
    }
    return v;
}

} // namespace pibridge::app

namespace pibridge::app {

QString steamVrStagePath() {
    return statusDir() + QStringLiteral("/steamvr-stage");
}

QString steamVrStageText(qint64 vrserverPid, quint64 universe, const std::array<float, 12>& m) {
    QStringList v;
    for (float x : m) {
        v << QString::number(double(x), 'f', 6);
    }
    return QStringLiteral("pibridge-steamvr-stage 1\nvrserver %1\nuniverse %2\nstanding_from_raw %3\n")
        .arg(vrserverPid)
        .arg(universe)
        .arg(v.join(QLatin1Char(' ')));
}

qint64 vrserverPid(const QString& procRoot) {
    const QDir proc(procRoot);
    for (const QString& pid : proc.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        if (pid.isEmpty() || !pid.front().isDigit()) {
            continue;
        }
        QFile f(procRoot + QLatin1Char('/') + pid + QStringLiteral("/comm"));
        if (f.open(QIODevice::ReadOnly) && f.readAll().trimmed() == "vrserver") {
            return pid.toLongLong();
        }
    }
    return 0;
}

} // namespace pibridge::app
