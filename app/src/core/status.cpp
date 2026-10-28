// SPDX-License-Identifier: MIT
#include "status.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

namespace pibridge::app {

namespace {

std::optional<QJsonObject> object(const QByteArray& json) {
    QJsonParseError err{};
    const QJsonDocument d = QJsonDocument::fromJson(json, &err);
    if (err.error != QJsonParseError::NoError || !d.isObject()) {
        return std::nullopt;
    }
    return d.object();
}

// A count that a writer may give as a number or as a boolean.
int count(const QJsonValue& v) {
    if (v.isBool()) {
        return v.toBool() ? 1 : 0;
    }
    return v.toInt(static_cast<int>(v.toDouble()));
}

qint64 int64(const QJsonValue& v) {
    return static_cast<qint64>(v.toDouble());
}

std::optional<QByteArray> readFile(const QString& path, qint64& mtimeMs) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        return std::nullopt;
    }
    mtimeMs = QFileInfo(path).lastModified().toMSecsSinceEpoch();
    return f.readAll();
}

bool fresh(qint64 updatedMs, qint64 nowMs) {
    return nowMs - updatedMs <= kStaleMs;
}

} // namespace

std::optional<KeepaliveStatus> parseKeepalive(const QByteArray& json) {
    const auto o = object(json);
    if (!o) {
        return std::nullopt;
    }
    KeepaliveStatus k;
    k.running = true;
    k.state = o->value("state").toString();
    k.model = o->value("model").toString();
    k.headsetPresent = o->value("headset_present").toBool();
    k.panelOn = o->value("panel_on").toBool();
    k.rateHz = count(o->value("rate_hz"));
    k.follow = o->value("follow").toString();
    k.followRunning = o->value("follow_running").toBool();
    k.audioOnHeadset = o->value("audio_on_headset").toBool();
    k.lensIpdMm = o->value("lens_ipd_x100").toDouble() / 100.0;
    k.crystalPresent = o->value("crystal_present").toBool();
    k.crystalStage = o->value("crystal_stage").toString();
    k.crystalLighthouse = o->value("crystal_lighthouse").toString();
    k.crystalTracked = o->value("crystal_tracked").toBool();
    k.crystalBattery = o->value("crystal_battery").isDouble() ? o->value("crystal_battery").toInt() : -1;
    k.error = o->value("error").toString();
    k.ctrlId = int64(o->value("ctrl_id"));
    k.ctrlOp = o->value("ctrl_op").toString();
    k.ctrlSide = o->value("ctrl_side").toString();
    k.ctrlState = o->value("ctrl_state").toString();
    k.ctrlDetail = o->value("ctrl_detail").toString();
    k.leftConnected = o->value("left_connected").toBool();
    k.rightConnected = o->value("right_connected").toBool();
    k.hapticSeq = int64(o->value("haptic_seq"));
    k.hapticResult = o->value("haptic_result").toString();
    k.pid = int64(o->value("pid"));
    k.updatedMs = int64(o->value("updated_unix_ms"));
    return k;
}

std::optional<EyedStatus> parseEyed(const QByteArray& json) {
    const auto o = object(json);
    if (!o) {
        return std::nullopt;
    }
    EyedStatus e;
    e.running = true;
    e.state = o->value("state").toString();
    e.profile = o->value("profile").toString();
    e.eyesSeen = count(o->value("eyes_seen"));
    e.samplesPerS = o->value("samples_per_s").toDouble();
    e.calibration = o->value("calibration").toString();
    e.follow = o->value("follow").toString();
    e.followRunning = o->value("follow_running").toBool();
    e.error = o->value("error").toString();
    e.pid = int64(o->value("pid"));
    e.updatedMs = int64(o->value("updated_unix_ms"));
    return e;
}

KeepaliveStatus readKeepalive(const QString& path, qint64 nowMs) {
    qint64 mtime = 0;
    const auto bytes = readFile(path, mtime);
    if (!bytes) {
        return {};
    }
    auto k = parseKeepalive(*bytes);
    if (!k) {
        return {};
    }
    if (k->updatedMs == 0) {
        k->updatedMs = mtime;
    }
    k->running = fresh(k->updatedMs, nowMs);
    return *k;
}

EyedStatus readEyed(const QString& path, qint64 nowMs) {
    qint64 mtime = 0;
    const auto bytes = readFile(path, mtime);
    if (!bytes) {
        return {};
    }
    auto e = parseEyed(*bytes);
    if (!e) {
        return {};
    }
    if (e->updatedMs == 0) {
        e->updatedMs = mtime;
    }
    e->running = fresh(e->updatedMs, nowMs);
    return *e;
}

ChipText headsetChip(const KeepaliveStatus& k) {
    if (!k.running) {
        return {QStringLiteral("SERVICE OFF"), ChipKind::Off};
    }
    if (k.state == QLatin1String("error")) {
        return {QStringLiteral("ERROR"), ChipKind::Error};
    }
    if (k.state == QLatin1String("paused")) {
        return {QStringLiteral("PAUSED: INSIDE-OUT"), ChipKind::Info};
    }
    if (k.state == QLatin1String("no-headset") || !k.headsetPresent) {
        return {QStringLiteral("NO HEADSET"), ChipKind::Warn};
    }
    if (k.panelOn) {
        return {QStringLiteral("SCREEN ON"), ChipKind::Info};
    }
    return {QStringLiteral("SCREEN OFF"), ChipKind::Warn};
}

QString headsetHint(const KeepaliveStatus& k, bool steamvrRunning) {
    if (!k.running) {
        return QStringLiteral("Service not running. Press \"Restart services\" to start it.");
    }
    if (k.state == QLatin1String("error")) {
        return k.error.isEmpty() ? QStringLiteral("The headset service reported an error.") : k.error;
    }
    if (k.state == QLatin1String("paused")) {
        return QString(); // the STATE row says it (Paused: inside-out tracking)
    }
    if (k.state == QLatin1String("no-headset") || !k.headsetPresent) {
        return QStringLiteral("No headset found. Check that the headset is plugged in and powered.");
    }
    if (k.state == QLatin1String("waking")) {
        return QStringLiteral("Waking the headset...");
    }
    if (k.state == QLatin1String("active")) {
        return k.panelOn ? QStringLiteral("The headset is on.") : QStringLiteral("Turning the screen on...");
    }
    // idle
    return steamvrRunning ? QStringLiteral("SteamVR is running; the headset wakes in a moment.")
                          : QStringLiteral("Press START VR on the Dashboard to wake the headset.");
}

QString eyeLine(const EyedStatus& e) {
    if (!e.running) {
        return QStringLiteral("Service not running");
    }
    if (e.state == QLatin1String("error")) {
        return e.error.isEmpty() ? QStringLiteral("Error") : QStringLiteral("Error: %1").arg(e.error);
    }
    if (e.state == QLatin1String("streaming")) {
        return QStringLiteral("Streaming: %1 %2, %3 samples/s")
            .arg(e.eyesSeen)
            .arg(e.eyesSeen == 1 ? QStringLiteral("eye") : QStringLiteral("eyes"))
            .arg(qRound(e.samplesPerS));
    }
    if (e.state == QLatin1String("paused")) {
        return QStringLiteral("Paused: inside-out tracking runs, pibridge-headtrack owns the headset");
    }
    if (e.state == QLatin1String("searching")) {
        return QStringLiteral("Looking for the eye tracker...");
    }
    if (e.state == QLatin1String("connecting")) {
        return QStringLiteral("Connecting to the eye tracker...");
    }
    return QStringLiteral("Idle: starts with SteamVR");
}

ChipText eyeChip(const EyedStatus& e) {
    if (!e.running) {
        return {QStringLiteral("SERVICE OFF"), ChipKind::Off};
    }
    if (e.state == QLatin1String("error")) {
        return {QStringLiteral("ERROR"), ChipKind::Error};
    }
    if (e.state == QLatin1String("streaming")) {
        return {QStringLiteral("STREAMING"), ChipKind::Info};
    }
    if (e.state == QLatin1String("paused")) {
        return {QStringLiteral("PAUSED: INSIDE-OUT"), ChipKind::Info};
    }
    if (e.state.isEmpty()) {
        return {QStringLiteral("IDLE"), ChipKind::Off};
    }
    return {e.state.toUpper(), e.state == QLatin1String("idle") ? ChipKind::Off : ChipKind::Warn};
}

} // namespace pibridge::app
