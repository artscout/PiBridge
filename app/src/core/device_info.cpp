// SPDX-License-Identifier: MIT
#include "device_info.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

#include "state.h"

namespace pibridge::app {

namespace {

std::optional<QJsonObject> readObject(const QString& path) {
    QFile f(path);
    if (path.isEmpty() || !f.open(QIODevice::ReadOnly)) {
        return std::nullopt;
    }
    const QJsonDocument d = QJsonDocument::fromJson(f.readAll());
    if (!d.isObject()) {
        return std::nullopt;
    }
    return d.object();
}

// Numbers always with a point, whatever the locale (QString::number is locale-independent).
QString mm(double v) {
    return QString::number(v, 'f', 1) + QStringLiteral(" mm");
}

const VrDevice* findHmd(const SteamVrStatus& s) {
    for (const VrDevice& d : s.devices) {
        if (d.cls == VrClass::Hmd) {
            return &d;
        }
    }
    return nullptr;
}

} // namespace

std::optional<ChovrIpd> chovrConfiguredIpd(const QString& settingsFile) {
    const auto o = readObject(settingsFile);
    if (!o) {
        return std::nullopt;
    }
    std::optional<ChovrIpd> any;
    for (auto it = o->begin(); it != o->end(); ++it) {
        if (!it.value().isObject()) {
            continue;
        }
        const QJsonObject h = it.value().toObject();
        const QJsonValue ipd = h.value(QStringLiteral("ipd"));
        if (!ipd.isDouble() || h.value(QStringLiteral("enable")).toBool(true) == false) {
            continue;
        }
        ChovrIpd c;
        c.mm = ipd.toDouble() + h.value(QStringLiteral("ipdOffset")).toDouble(0.0);
        c.hardwareIpd = h.value(QStringLiteral("hardwareIpd")).toBool(false);
        c.headset = it.key();
        if (it.key().startsWith(QLatin1String("crystalSuper"))) {
            return c;
        }
        if (!any) {
            any = c;
        }
    }
    return any;
}

std::optional<SteamVrSupersample> steamVrSupersample(const QString& vrsettingsFile) {
    const auto o = readObject(vrsettingsFile);
    if (!o) {
        return std::nullopt;
    }
    const QJsonObject s = o->value(QStringLiteral("steamvr")).toObject();
    SteamVrSupersample r;
    r.scale = s.value(QStringLiteral("supersampleScale")).toDouble(1.0);
    r.manual = s.value(QStringLiteral("supersampleManualOverride")).toBool(false);
    return r;
}

DeviceView deviceView(const DeviceInputs& in) {
    DeviceView v;
    const bool live = in.steamvrRunning && in.steamvr.state == VrLinkState::Connected;
    const VrDevice* hmd = live ? findHmd(in.steamvr) : nullptr;
    const QString startVr = QStringLiteral("Read while VR runs (START VR on the Dashboard).");

    QString model = in.keepalive.model;
    if (model.isEmpty() && hmd) {
        model = vrDeviceName(*hmd);
    }
    v.title = model.isEmpty() ? QStringLiteral("No headset") : QStringLiteral("Pimax ") + model;
    if (live) {
        v.chip = {QStringLiteral("STEAMVR LIVE"), ChipKind::Info};
    } else if (in.steamvrRunning) {
        v.chip = {QStringLiteral("CONNECTING"), ChipKind::Off};
    } else {
        v.chip = {QStringLiteral("STEAMVR OFF"), ChipKind::Off};
    }

    // ---- headset
    v.headset.push_back({QStringLiteral("Model"), model,
                         model.isEmpty() ? QStringLiteral("The headset service does not see a headset.") : QString()});
    if (hmd && !hmd->serial.isEmpty()) {
        v.headset.push_back({QStringLiteral("Headset SN"), hmd->serial, QStringLiteral("from SteamVR"), true});
    } else {
        v.headset.push_back({QStringLiteral("Headset SN"), {},
                             hmd ? QStringLiteral("SteamVR does not report one.") : startVr});
    }
    const QString noHw = QStringLiteral("The headset reports no hardware IPD on Linux.");
    if (hmd && hmd->ipdMeters > 0) {
        v.headset.push_back({QStringLiteral("IPD"), mm(hmd->ipdMeters * 1000.0),
                             QStringLiteral("What SteamVR renders with. ") + noHw});
    } else if (in.chovrIpd) {
        v.headset.push_back({QStringLiteral("IPD"), mm(in.chovrIpd->mm),
                             QStringLiteral("Configured in CustomHeadsetOpenVR, not read live. ") + noHw});
    } else {
        v.headset.push_back({QStringLiteral("IPD"), {}, QStringLiteral("Not configured. ") + noHw});
    }
    if (in.gazeIpdMm > 0) {
        v.headset.push_back({QStringLiteral("Measured IPD"), mm(in.gazeIpdMm), QStringLiteral("by the eye tracker")});
    } else {
        v.headset.push_back(
            {QStringLiteral("Measured IPD"), {}, QStringLiteral("Shown while the eye tracker streams.")});
    }

    // ---- controllers, trackers, base stations
    int station = 0;
    for (const VrDevice& d : live ? in.steamvr.devices : std::vector<VrDevice>{}) {
        const QString sn = d.serial;
        const QString note = sn.isEmpty() ? vrDeviceName(d) + QStringLiteral(", no serial reported") : vrDeviceName(d);
        if (d.cls == VrClass::Controller || d.cls == VrClass::GenericTracker) {
            const QString key = d.role == VrRole::Left    ? QStringLiteral("Left controller")
                                : d.role == VrRole::Right ? QStringLiteral("Right controller")
                                : d.cls == VrClass::GenericTracker ? QStringLiteral("Tracker")
                                                                   : QStringLiteral("Controller");
            v.controllers.push_back({key, sn, note, !sn.isEmpty()});
        } else if (d.cls == VrClass::TrackingReference) {
            v.stations.push_back({QStringLiteral("Station %1").arg(++station), sn, note, !sn.isEmpty()});
        }
    }
    if (!live) {
        v.devicesNote = in.steamvrRunning ? QStringLiteral("Connecting to SteamVR...")
                                          : QStringLiteral("The controllers' and base stations' serial numbers show "
                                                           "while VR runs (START VR on the Dashboard).");
    } else if (v.controllers.empty() && v.stations.empty()) {
        v.devicesNote = QStringLiteral("SteamVR sees no controllers or base stations. Turn the controllers on.");
    }

    // ---- rendering
    QString ss;
    if (in.supersample) {
        ss = in.supersample->manual
                 ? QStringLiteral("Supersampling %1 % (set by hand in SteamVR).")
                       .arg(QString::number(qRound(in.supersample->scale * 100)))
                 : QStringLiteral("Supersampling: SteamVR picks it (Auto).");
    }
    if (hmd && hmd->renderWidth > 0 && hmd->renderHeight > 0) {
        v.rendering.push_back({QStringLiteral("Render resolution"),
                               QStringLiteral("%1 x %2 per eye").arg(hmd->renderWidth).arg(hmd->renderHeight),
                               QStringLiteral("SteamVR's recommended size, supersampling included. ") + ss});
    } else {
        v.rendering.push_back({QStringLiteral("Render resolution"), {}, startVr + (ss.isEmpty() ? QString() : QLatin1Char(' ') + ss)});
    }
    if (hmd && hmd->displayHz > 0) {
        v.rendering.push_back({QStringLiteral("Refresh rate"),
                               QStringLiteral("%1 Hz").arg(qRound(hmd->displayHz)), QStringLiteral("from SteamVR")});
    } else if (in.keepalive.running && in.keepalive.panelOn && in.keepalive.rateHz > 0) {
        v.rendering.push_back({QStringLiteral("Refresh rate"), QStringLiteral("%1 Hz").arg(in.keepalive.rateHz),
                               QStringLiteral("the panel rate from the headset service")});
    } else {
        v.rendering.push_back({QStringLiteral("Refresh rate"), QStringLiteral("%1 Hz").arg(in.confRefreshHz),
                               QStringLiteral("set in PiBridge; the headset uses it when it wakes")});
    }
    v.rendering.push_back({QStringLiteral("Quad views"),
                           !in.qvInstalled ? QStringLiteral("Not installed")
                           : in.qvEnabled  ? QStringLiteral("On")
                                           : QStringLiteral("Off"),
                           QStringLiteral("Performance page")});
    QString mode;
    if (in.directRunning) {
        mode = QStringLiteral("Monado directly (running)");
    } else if (in.insideOut) {
        mode = in.ioRunning ? QStringLiteral("Inside-out: SteamVR + Monado (running)")
                            : QStringLiteral("Inside-out: SteamVR + Monado");
    } else {
        mode = QStringLiteral("Lighthouse (SteamVR)");
    }
    v.rendering.push_back({QStringLiteral("Tracking mode"), mode, QStringLiteral("Tracking page")});
    const bool monadoUsed = in.insideOut || in.directRunning;
    const QString use = monadoUsed ? QString() : QStringLiteral(" Used only by inside-out tracking.");
    if (in.monado.path.isEmpty()) {
        v.rendering.push_back({QStringLiteral("Monado build"), QStringLiteral("Not found"),
                               QStringLiteral("Only the old Monado session uses it.") + use});
    } else if (in.monado.checking) {
        v.rendering.push_back({QStringLiteral("Monado build"), QStringLiteral("Checking..."), use.trimmed()});
    } else if (!in.monado.checked) {
        v.rendering.push_back({QStringLiteral("Monado build"), QStringLiteral("Not readable"),
                               in.monado.error + use});
    } else {
        const QString blocking = buildBlockingReason(in.monado);
        v.rendering.push_back({QStringLiteral("Monado build"), buildKindWords(in.monado.kind),
                               (blocking.isEmpty() ? QStringLiteral("All required markers present.") : blocking) + use});
    }
    return v;
}

DeviceInputs fakeDeviceInputs(bool active) {
    const AppState s = fakeState(active);
    DeviceInputs in;
    in.steamvrRunning = s.steamvrRunning;
    in.steamvr = s.steamvr;
    in.keepalive = s.keepalive;
    in.gazeIpdMm = active ? 63.8f : 0.0f;
    in.chovrIpd = ChovrIpd{63.0, false, QStringLiteral("crystalSuperMicroOLED")};
    in.supersample = SteamVrSupersample{1.5, true};
    in.confRefreshHz = 90;
    in.qvInstalled = true;
    in.qvEnabled = true;
    in.monado.checked = true;
    in.monado.path = QStringLiteral("/fake/monado-service");
    in.monado.kind = BuildKind::PiBridge;
    return in;
}

} // namespace pibridge::app
