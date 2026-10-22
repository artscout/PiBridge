// SPDX-License-Identifier: MIT
#include "inside_out.h"
#include "video_record.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSaveFile>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>

#include "paths.h"
#include "rec_setup.h"

namespace pibridge::app {

namespace {

QString word(const Conf& c, std::string_view key) {
    return c.value(key, QString()).trimmed().toLower();
}

int intOr(const Conf& c, std::string_view key, int def, int lo, int hi) {
    bool ok = false;
    const int v = c.value(key, QString()).trimmed().toInt(&ok);
    return ok ? std::clamp(v, lo, hi) : def;
}

std::string onOff(bool on) {
    return on ? "on" : "off";
}

QString num(double v) {
    return QString::number(v, 'g', 12);
}

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

bool writeAtomic(const QString& path, const QByteArray& data) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(data);
    return f.commit();
}

std::optional<std::array<double, 3>> three(const QString& text, double maxAbs) {
    const QStringList w = text.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (w.size() != 3) {
        return std::nullopt;
    }
    std::array<double, 3> v{};
    for (int i = 0; i < 3; i++) {
        bool ok = false;
        v[size_t(i)] = w[i].toDouble(&ok);
        if (!ok || !std::isfinite(v[size_t(i)]) || std::fabs(v[size_t(i)]) > maxAbs) {
            return std::nullopt;
        }
    }
    return v;
}

// The first three numbers of a JSON array `"key": [ ... ]` in `text`, or nullopt.
std::optional<std::array<double, 3>> jsonArray3(const QString& text, const QString& key) {
    const QRegularExpression re(QStringLiteral("\"%1\"\\s*:\\s*\\[([^\\]]*)\\]").arg(QRegularExpression::escape(key)));
    const auto m = re.match(text);
    if (!m.hasMatch()) {
        return std::nullopt;
    }
    QStringList parts = m.captured(1).split(QLatin1Char(','), Qt::SkipEmptyParts);
    if (parts.size() < 3) {
        return std::nullopt;
    }
    std::array<double, 3> v{};
    for (int i = 0; i < 3; i++) {
        bool ok = false;
        v[size_t(i)] = parts[i].trimmed().toDouble(&ok);
        if (!ok) {
            return std::nullopt;
        }
    }
    return v;
}

// Replace the whole array of `"key": [...]` by `values`; false when the key is not there.
bool setJsonArray(QString& text, const QString& key, const QStringList& values) {
    const QRegularExpression re(QStringLiteral("(\"%1\"\\s*:\\s*)\\[[^\\]]*\\]").arg(QRegularExpression::escape(key)));
    const auto m = re.match(text);
    if (!m.hasMatch()) {
        return false;
    }
    text.replace(m.capturedStart(), m.capturedLength(),
                 m.captured(1) + QLatin1Char('[') + values.join(QStringLiteral(", ")) + QLatin1Char(']'));
    return true;
}

// The text of the JSON array value of "key" (brackets matched; strings in it may hold brackets), "" when absent.
std::pair<qsizetype, qsizetype> jsonArraySpan(const QString& text, const QString& key) {
    const QRegularExpression re(QStringLiteral("\"%1\"\\s*:\\s*\\[").arg(QRegularExpression::escape(key)));
    const auto m = re.match(text);
    if (!m.hasMatch()) {
        return {-1, -1};
    }
    const qsizetype start = m.capturedEnd() - 1;
    int depth = 0;
    bool str = false;
    for (qsizetype i = start; i < text.size(); i++) {
        const QChar c = text.at(i);
        if (str) {
            if (c == QLatin1Char('\\')) {
                i++;
            } else if (c == QLatin1Char('"')) {
                str = false;
            }
        } else if (c == QLatin1Char('"')) {
            str = true;
        } else if (c == QLatin1Char('[')) {
            depth++;
        } else if (c == QLatin1Char(']') && --depth == 0) {
            return {start, i + 1};
        }
    }
    return {-1, -1};
}

QString jsonArrayText(const QString& text, const QString& key) {
    const auto [a, b] = jsonArraySpan(text, key);
    return a < 0 ? QString() : text.mid(a, b - a);
}

bool replaceJsonArray(QString& text, const QString& key, const QString& array) {
    const auto [a, b] = jsonArraySpan(text, key);
    if (a < 0) {
        return false;
    }
    text.replace(a, b - a, array);
    return true;
}

// The value of `key=...` / `key="..."` in a Basalt toml (the format omarchyvr-slam export writes).
QString tomlValue(const QString& text, const QString& key) {
    for (const QString& raw : text.split(QLatin1Char('\n'))) {
        const QString l = raw.trimmed();
        if (l.startsWith(QLatin1Char('#'))) {
            continue;
        }
        const qsizetype eq = l.indexOf(QLatin1Char('='));
        if (eq <= 0 || l.left(eq).trimmed() != key) {
            continue;
        }
        QString v = l.mid(eq + 1).trimmed();
        if (v.size() >= 2 && v.startsWith(QLatin1Char('"')) && v.endsWith(QLatin1Char('"'))) {
            v = v.mid(1, v.size() - 2);
        }
        return v;
    }
    return {};
}

QString resolveNear(const QString& p, const QString& baseToml) {
    if (p.isEmpty() || QDir::isAbsolutePath(p)) {
        return p;
    }
    return QFileInfo(baseToml).absoluteDir().filePath(p);
}

} // namespace

// ---------------------------------------------------------------- settings

IoSettings ioSettings(const Conf& c) {
    IoSettings s;
    const auto flag = [&c](std::string_view k, bool def) { return c.flag(k, def); };
    const QString mode = word(c, iokey::kMode);
    s.mode = mode == QLatin1String("inside-out") || mode == QLatin1String("insideout") ? TrackingMode::InsideOut
                                                                                         : TrackingMode::Lighthouse;
    s.apps = word(c, iokey::kApps) == QLatin1String("monado") ? IoApps::Monado : IoApps::SteamVr;
    s.homeScene = flag(iokey::kHome, true);
    s.slam = flag(iokey::kSlam, true);
    s.slamEngine = word(c, iokey::kSlamEngine) == QLatin1String("pimax") ? SlamEngine::Pimax : SlamEngine::Basalt;
    const int g = intOr(c, iokey::kGrid, 30, 10, 100);
    s.grid = g <= 35 ? 30 : (g <= 45 ? 40 : 50);
    s.slamCams = intOr(c, iokey::kSlamCams, 2, 1, 8) == 4 ? 4 : 2;
    s.accelBias = word(c, iokey::kAccelBias) == QLatin1String("off") ? AccelBias::Off : AccelBias::Factory;
    const QString gy = word(c, iokey::kGyroBias);
    s.gyroBias = gy == QLatin1String("factory") ? GyroBias::Factory
                 : gy == QLatin1String("off")   ? GyroBias::Off
                                                : GyroBias::Measured;
    s.driftHold = flag(iokey::kDriftHold, false);
    s.stillHold = flag(iokey::kStillHold, true);
    s.headTilt = flag(iokey::kHeadTilt, true);
    s.fuseAuto = flag(iokey::kFuseAuto, true);
    s.smooth = flag(iokey::kSmooth, true);
    s.anchor = flag(iokey::kAnchor, false);
    s.anchorGlide = flag(iokey::kAnchorGlide, true);
    const QString ae = word(c, iokey::kCamAe);
    s.camAe = ae == QLatin1String("dry") ? CamAe::Dry : (confBool(ae.toStdString(), true) ? CamAe::On : CamAe::Off);
    s.camAeCrystal = word(c, iokey::kCamAeModel) != QLatin1String("auto");
    s.camAeNewLaw = word(c, iokey::kCamAeLaw) != QLatin1String("old");
    s.ctrlTrack = flag(iokey::kCtrlTrack, true);
    s.ctrlAngle = intOr(c, iokey::kCtrlAngle, 30, kCtrlAngleMin, kCtrlAngleMax);
    s.ctrlLostS = intOr(c, iokey::kCtrlLost, 4, kCtrlLostMin, kCtrlLostMax);
    s.ctrlSteady = flag(iokey::kCtrlSteady, true);
    return s;
}

std::vector<std::pair<std::string, std::string>> ioConfLines(const IoSettings& s) {
    std::vector<std::pair<std::string, std::string>> l;
    l.emplace_back(iokey::kMode, s.mode == TrackingMode::InsideOut ? "inside-out" : "lighthouse");
    l.emplace_back(iokey::kApps, s.apps == IoApps::Monado ? "monado" : "steamvr");
    l.emplace_back(iokey::kHome, onOff(s.homeScene));
    l.emplace_back(iokey::kSlam, onOff(s.slam));
    l.emplace_back(iokey::kSlamEngine, s.slamEngine == SlamEngine::Pimax ? "pimax" : "basalt");
    l.emplace_back(iokey::kGrid, std::to_string(s.grid));
    l.emplace_back(iokey::kSlamCams, s.slamCams == 4 ? "4" : "2");
    l.emplace_back(iokey::kAccelBias, s.accelBias == AccelBias::Off ? "off" : "factory");
    l.emplace_back(iokey::kGyroBias, s.gyroBias == GyroBias::Factory ? "factory"
                                     : s.gyroBias == GyroBias::Off   ? "off"
                                                                     : "measured");
    l.emplace_back(iokey::kDriftHold, onOff(s.driftHold));
    l.emplace_back(iokey::kStillHold, onOff(s.stillHold));
    l.emplace_back(iokey::kHeadTilt, onOff(s.headTilt));
    l.emplace_back(iokey::kFuseAuto, onOff(s.fuseAuto));
    l.emplace_back(iokey::kSmooth, onOff(s.smooth));
    l.emplace_back(iokey::kAnchor, onOff(s.anchor));
    l.emplace_back(iokey::kAnchorGlide, onOff(s.anchorGlide));
    l.emplace_back(iokey::kCamAe, s.camAe == CamAe::Dry ? "dry" : onOff(s.camAe == CamAe::On));
    l.emplace_back(iokey::kCamAeModel, s.camAeCrystal ? "crystal-super" : "auto");
    l.emplace_back(iokey::kCamAeLaw, s.camAeNewLaw ? "new" : "old");
    l.emplace_back(iokey::kCtrlTrack, onOff(s.ctrlTrack));
    l.emplace_back(iokey::kCtrlAngle, std::to_string(s.ctrlAngle));
    l.emplace_back(iokey::kCtrlLost, std::to_string(s.ctrlLostS));
    l.emplace_back(iokey::kCtrlSteady, onOff(s.ctrlSteady));
    return l;
}

std::vector<std::pair<std::string, std::string>> ioRecommendedLines() {
    auto l = ioConfLines(IoSettings::recommended());
    std::erase_if(l, [](const auto& kv) { return kv.first == iokey::kMode || kv.first == iokey::kApps || kv.first == iokey::kHome; });
    return l;
}

QString modeWord(TrackingMode m) {
    return m == TrackingMode::InsideOut ? QStringLiteral("INSIDE-OUT") : QStringLiteral("LIGHTHOUSE");
}

// ---------------------------------------------------------------- the environment

bool pimaxSlamReady(const RecSetup& s, QString* why) {
    QStringList miss;
    if (s.winbridge.path.isEmpty()) {
        miss << QStringLiteral("winbridge.exe (rec_winbridge=)");
    }
    if (s.vitBridge.path.isEmpty()) {
        miss << QStringLiteral("libpibridge_vit_winbridge.so (rec_vit_winbridge=)");
    }
    if (s.wine.isEmpty()) {
        miss << QStringLiteral("Wine");
    }
    if (s.winKit.isEmpty()) {
        miss << QStringLiteral("the Pimax files from Pimax Play in ~/.local/share/pibridge/windata (docs/WINBRIDGE.md)");
    }
    if (why) {
        *why = miss.isEmpty() ? QString() : QStringLiteral("missing: ") + miss.join(QStringLiteral(", "));
    }
    return miss.isEmpty();
}

bool pimaxCtrlEngine(const RecSetup& s) {
    return s.io.ctrlTrack && s.io.slam && s.io.slamEngine == SlamEngine::Pimax && pimaxSlamReady(s) &&
           s.winCtrlModels;
}

IoSessionEnv buildIoEnv(const IoSettings& io, const RecSetup& s, int refreshHz, const QString& slamConfig,
                        const QString& recordDir, const QString& driverEnvFile, const QString& preload) {
    IoSessionEnv e;
    QStringList& d = e.driver;
    // The Pimax driver's settings of crystal-6dof.sh (SETTINGS=...), in its order.
    d << QStringLiteral("PIMAX_FPS=%1").arg(refreshHz == 72 ? 72 : 90) << QStringLiteral("PIMAX_LOG=info")
      << QStringLiteral("PIMAX_BIAS_TIMEOUT_MS=200");
    if (!s.lens.path.isEmpty()) {
        d << QStringLiteral("PIMAX_LENS_CONF=") + s.lens.path;
    }
    // the eye separation both the SteamVR driver (ovrd_driver.cpp pimax_ipd_meters) and native OpenXR render with
    d << QStringLiteral("OXR_DEBUG_IPD_MM=%1").arg(s.ipdMm, 0, 'f', 1);
    d << QStringLiteral("PIMAX_CAM_CALIB=") + s.calib.path << QStringLiteral("PIMAX_CAM_SOURCE=broker");
    const bool pimax = io.slamEngine == SlamEngine::Pimax;
    QString pimaxWhy;
    const bool slam = io.slam && (pimax ? pimaxSlamReady(s, &pimaxWhy) : !s.basalt.path.isEmpty());
    if (io.slam && !slam) {
        e.notes << (pimax ? QStringLiteral("Head tracking (SLAM) stays off: the Pimax engine is not complete (%1).")
                                .arg(pimaxWhy)
                          : QStringLiteral("Head tracking (SLAM) stays off: the Basalt library was not found "
                                           "(rec_basalt_lib=)."));
    }
    e.winbridge = slam && pimax;
    d << (slam ? QStringLiteral("PIMAX_SLAM=1") : QStringLiteral("PIMAX_SLAM=0"));
    // camera auto exposure (pimax_camera.c PIMAX_CAM_AE: unset/0 off, "dry", "1"; pimax-cam ae.rs for the rest)
    d << QStringLiteral("PIMAX_CAM_AE=%1").arg(io.camAe == CamAe::On ? QStringLiteral("1")
                                               : io.camAe == CamAe::Dry ? QStringLiteral("dry")
                                                                        : QStringLiteral("0"));
    if (io.camAe != CamAe::Off) {
        if (io.camAeCrystal) {
            d << QStringLiteral("PIMAX_CAM_AE_MODEL=crystal-super");
        }
        if (!io.camAeNewLaw) {
            d << QStringLiteral("PIMAX_CAM_AE_LAW=old");
        }
    }
    // controllers (pimax_ctrl.c; pimax-hands fuse.rs). With Pimax's engine the controllers come from Pimax's LED
    // engine in the same winbridge (PIMAX_CTRL_ENGINE), as in Pimax Play, and pimax-ctrltrack is not started.
    const bool ctrlEngine = pimaxCtrlEngine(s);
    e.ctrltrack = io.ctrlTrack && !ctrlEngine && !s.ctrl.path.isEmpty();
    if (io.ctrlTrack && !ctrlEngine && !e.ctrltrack) {
        e.notes << QStringLiteral("Optical controller tracking stays off: pibridge-ctrltrack was not found (rec_pimax_ctrltrack=).");
    }
    d << (e.ctrltrack ? QStringLiteral("PIMAX_CTRL_TRACK=1") : QStringLiteral("PIMAX_CTRL_TRACK=0"));
    if (ctrlEngine) {
        d << QStringLiteral("PIMAX_CTRL_ENGINE=1");
    }
    d << QStringLiteral("PIMAX_CTRL_ANGLE=%1").arg(std::clamp(io.ctrlAngle, kCtrlAngleMin, kCtrlAngleMax));
    // PIMAX_CTRL_INHAND=auto is not passed (live 2026-10-19): it marked both controllers "not in the hand" while held,
    // and pibridge-ctrltrack then stopped looking for them; the tracker's IMU hand check keeps a parked controller's
    // fixes out instead
    if (e.ctrltrack) {
        if (io.ctrlLostS != 4) {
            d << QStringLiteral("PIMAX_CTRL_TRACK_LOST_S=%1").arg(std::clamp(io.ctrlLostS, kCtrlLostMin, kCtrlLostMax));
        }
        if (!io.ctrlSteady) {
            d << QStringLiteral("PIMAX_CTRL_TRACK_HYST=0");
        }
    }
    if (slam && pimax) {
        // Pimax's engine reads its own calibration and settings from the kit (no SLAM_CONFIG) and gets all four
        // tracking cameras, as Pimax Play's service hands it every frame set (camera index 0..3)
        d << QStringLiteral("VIT_SYSTEM_LIBRARY_PATH=") + s.vitBridge.path;
        d << QStringLiteral("PIMAX_SLAM_CAMS=0,1,2,3");
        // raw frame stamps (exposure start), as Pimax Play hands them: the engine adds its own factory camera-IMU
        // delta (about half a typical exposure); the driver's mid-exposure shift is for Basalt and would count twice
        d << QStringLiteral("PIMAX_SLAM_EXPOSURE_MID=0");
        // the engine's pose as it comes, as Pimax Play uses it: every pose (IMU rate) and the driver's pass-through
        // (no supervisor, still gate, tilt correction or drift hold: those are for Basalt; the engine handles loss)
        // IMU-rate head poses (smooth rotation); without this the engine delivers one pose per camera frame (~24 Hz)
        // and the head jerks when you turn.
        d << QStringLiteral("PIMAX_SLAM_PASSTHROUGH=1") << QStringLiteral("PIBRIDGE_VIT_POSE_RATE=imu");
    } else if (slam) {
        d << QStringLiteral("VIT_SYSTEM_LIBRARY_PATH=") + s.basalt.path;
        if (!slamConfig.isEmpty()) {
            d << QStringLiteral("SLAM_CONFIG=") + slamConfig;
        } else {
            e.notes << QStringLiteral("No Basalt config: Monado hands Basalt the driver's own calibration and its "
                                      "built-in settings (the detection grid cannot be set).");
        }
        // the cameras (pimax_camera.c PIMAX_SLAM_CAMS, default 0,1): four only with a calibration of four
        if (io.slamCams == 4) {
            if (slamConfig.isEmpty() || s.sessionSlamCams == 4) {
                d << QStringLiteral("PIMAX_SLAM_CAMS=0,1,2,3");
            } else {
                e.notes << QStringLiteral("Head tracking stays on the lower camera pair: no Basalt calibration with "
                                          "four cameras could be made (needs pibridge-slam and the headset's factory "
                                          "calibration).");
            }
        }
    }
    if (slam) {
        // the output stages of pimax-cam headpose.rs (the driver's, for either engine): on by default, only a switch
        // that is off is passed
        // (as omarchyvr's launcher passes the flag of a switch that is off)
        // the drift hold is on in the driver but off by default here (Crystal Super, study 3), so its 0 is
        // passed whenever it is off
        if (!io.driftHold) {
            d << QStringLiteral("PIMAX_SLAM_DRIFT_HOLD=0");
        }
        if (!io.stillHold) {
            d << QStringLiteral("PIMAX_SLAM_STILL=0");
        }
        if (!io.headTilt) {
            d << QStringLiteral("PIMAX_HEAD_TILT=0");
        }
        if (!io.fuseAuto) {
            d << QStringLiteral("PIMAX_FUSE_AUTO=0");
        }
        if (!io.smooth) {
            d << QStringLiteral("PIMAX_SLAM_SMOOTH=0");
        }
    }
    // eye tracking (ovrd_driver.cpp PIMAX_EYE_STEAMVR): the SteamVR driver reads pibridge-eyed's gaze shared memory
    // (/dev/shm/pibridge-eye-gaze) and serves SteamVR eye tracking (Prop_SupportsXrEyeGazeInteraction, OpenXR apps get
    // XR_EXT_eye_gaze_interaction). pibridge-eyed keeps the Tobii chip and writes the shm as in Lighthouse (it does NOT
    // pause in inside-out), so Monado does NOT open the chip itself (no PIMAX_EYE, no USB contention).
    // Only when Monado is the headset driver (inside-out) does it serve SteamVR eye tracking; in Lighthouse the
    // headset (and its eye tracking) come from CustomHeadsetOpenVR, so Monado must NOT also claim eye gaze there (two
    // drivers fighting over it dropped the HMD -> HmdNotFound).
    if (s.eyeTracking && io.mode == TrackingMode::InsideOut) {
        d << QStringLiteral("PIMAX_EYE_STEAMVR=1");
    }
    // the room anchor hook (Monado's offset file and head ring): the switch, or the room map (core/roommap_link.h).
    // With the room-map tool there, every inside-out session has the hook, so "Map the room" works in the session that
    // runs and a saved room is put in place at the start; without an offset file the hook changes nothing (Monado
    // deletes a stale one at its start), it only publishes the head ring.
    if (io.anchor || !s.roommap.isEmpty()) {
        d << QStringLiteral("PIMAX_ANCHOR=1");
        if (!io.anchorGlide) {
            d << QStringLiteral("PIMAX_ANCHOR_GLIDE=0");
        }
    }

    // monado-service: the driver plus the direct-mode compositor of crystal-6dof.sh, the recorder, rec_env last
    e.monado = d;
    e.monado << QStringLiteral("XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1")
             << QStringLiteral("XRT_COMPOSITOR_WAYLAND_CONNECTOR=") + s.connector
             << QStringLiteral("XRT_COMPOSITOR_FORCE_NVIDIA=0") << QStringLiteral("XRT_COMPOSITOR_LOG=info")
             << QStringLiteral("XRT_NO_STDIN=1") << QStringLiteral("IPC_IGNORE_VERSION=1");
    // the video recorder's tap (comp_recorder.c; idle cost: one poll every 0.5 s), at the ring pibridge-record creates
    e.monado << QStringLiteral("PIMAX_RECORD=1") << QStringLiteral("PIMAX_RECORD_SHM=") + recordShmPath();
    if (!recordDir.isEmpty()) {
        e.monado << QStringLiteral("PIMAX_SLAM_RECORD=") + recordDir;
    }
    e.monado << s.extraEnv; // rec_env= last: it overrides
    e.driver << s.extraEnv;

    // SteamVR (crystal-6dof.sh steamvr): vrserver does not get this environment, so driver_monado.so reads the driver
    // settings from a file (ovrd_driver.cpp PIMAX_STEAMVR_ENV_FILE); the preload keeps vrcompositor alive (rttime).
    e.steamvr << QStringLiteral("VALVE_SKIP_RUNTIME_SAFETY=1") << QStringLiteral("QT_QPA_PLATFORM=xcb");
    if (!preload.isEmpty()) {
        e.steamvr << QStringLiteral("LD_PRELOAD=") + preload;
    }
    if (!driverEnvFile.isEmpty()) {
        e.steamvr << QStringLiteral("PIMAX_STEAMVR_ENV_FILE=") + driverEnvFile;
    }
    return e;
}

QString driverEnvFileText(const IoSessionEnv& e) {
    return e.driver.join(QLatin1Char('\n')) + QLatin1Char('\n');
}

// ---------------------------------------------------------------- Basalt config

QString slamDir() {
    return configDir() + QStringLiteral("/slam");
}

FactoryImu readFactoryImu(const QByteArray& xml) {
    FactoryImu r;
    QXmlStreamReader x(xml);
    while (!x.atEnd()) {
        if (x.readNext() != QXmlStreamReader::StartElement || x.name() != QLatin1String("Stateinit")) {
            continue;
        }
        const auto a = x.attributes();
        if (a.hasAttribute(QStringLiteral("aBias"))) {
            r.aBias = three(a.value(QStringLiteral("aBias")).toString(), 1.0);
        }
        if (a.hasAttribute(QStringLiteral("wBias"))) {
            r.wBias = three(a.value(QStringLiteral("wBias")).toString(), 0.1);
        }
        break;
    }
    return r;
}

std::optional<std::array<double, 3>> parseGyroBiasCache(const QByteArray& text) {
    const auto v = three(QString::fromLatin1(text), 5.0);
    if (!v) {
        return std::nullopt;
    }
    const double k = M_PI / 180.0;
    return std::array<double, 3>{-(*v)[0] * k, (*v)[1] * k, -(*v)[2] * k};
}

QString gyroBiasCachePath() {
    const QString x = qEnvironmentVariable("XDG_CACHE_HOME");
    return (x.isEmpty() ? QDir::homePath() + QStringLiteral("/.cache") : x) + QStringLiteral("/omarchyvr/gyro-bias");
}

SlamGenResult generateSlamConfig(const SlamGenInput& in) {
    SlamGenResult r;
    if (in.baseToml.isEmpty() || !QFileInfo(in.baseToml).isFile()) {
        r.error = QStringLiteral("No base Basalt config (pimax-basalt.toml) was found (rec_slam_config=).");
        return r;
    }
    const QString toml = QString::fromUtf8(readAll(in.baseToml));
    const QString calibPath = resolveNear(tomlValue(toml, QStringLiteral("cam-calib")), in.baseToml);
    const QString configPath = resolveNear(tomlValue(toml, QStringLiteral("config-path")), in.baseToml);
    if (calibPath.isEmpty() || !QFileInfo(calibPath).isFile()) {
        r.error = QStringLiteral("The base config %1 names no readable cam-calib file.").arg(in.baseToml);
        return r;
    }
    if (configPath.isEmpty() || !QFileInfo(configPath).isFile()) {
        r.error = QStringLiteral("The base config %1 names no readable config-path file.").arg(in.baseToml);
        return r;
    }
    QString config = QString::fromUtf8(readAll(configPath));
    QString calib = QString::fromUtf8(readAll(calibPath));

    // the detection grid (Basalt's optical flow; study 2: 30 px helps a starved view)
    const QRegularExpression gridRe(QStringLiteral("(\"config\\.optical_flow_detection_grid_size\"\\s*:\\s*)-?\\d+"));
    const auto gm = gridRe.match(config);
    if (!gm.hasMatch()) {
        r.error = QStringLiteral("The base Basalt settings %1 have no optical_flow_detection_grid_size.").arg(configPath);
        return r;
    }
    config.replace(gm.capturedStart(), gm.capturedLength(), gm.captured(1) + QString::number(in.grid));

    // more cameras (the upper pair): the camera arrays of camsCalib, whose first cameras must be the base's
    r.cams = basaltCalibCams(calib.toUtf8());
    if (!in.camsCalib.isEmpty()) {
        const QByteArray more = readAll(in.camsCalib);
        const int n = basaltCalibCams(more);
        if (n == 0) {
            r.error = QStringLiteral("The camera calibration %1 is missing or is not a Basalt calibration.").arg(in.camsCalib);
            return r;
        }
        const QJsonObject b = QJsonDocument::fromJson(calib.toUtf8()).object().value(QStringLiteral("value0")).toObject();
        const QJsonObject m = QJsonDocument::fromJson(more).object().value(QStringLiteral("value0")).toObject();
        const QString moreText = QString::fromUtf8(more);
        for (const char* k : {"T_imu_cam", "intrinsics", "resolution"}) {
            const QString key = QString::fromLatin1(k);
            const QJsonArray ba = b.value(key).toArray();
            const QJsonArray ma = m.value(key).toArray();
            bool prefix = ma.size() == n && ba.size() <= ma.size();
            for (qsizetype i = 0; prefix && i < ba.size(); i++) {
                prefix = ba.at(i) == ma.at(i);
            }
            const QString arr = jsonArrayText(moreText, key);
            if (!prefix || arr.isEmpty() || !replaceJsonArray(calib, key, arr)) {
                r.error = QStringLiteral("The %1-camera calibration %2 does not start with the cameras of the base "
                                         "calibration %3 (another headset?).")
                              .arg(n)
                              .arg(in.camsCalib, calibPath);
                return r;
            }
        }
        r.cams = basaltCalibCams(calib.toUtf8());
        if (r.cams > 2) {
            // Basalt's corners of their own in the cameras beyond cam0's view (tools/slam export.h)
            const QRegularExpression no(QStringLiteral("(\"config\\.optical_flow_detection_nonoverlap\"\\s*:\\s*)(true|false)"));
            const auto nm = no.match(config);
            if (nm.hasMatch()) {
                config.replace(nm.capturedStart(), nm.capturedLength(), nm.captured(1) + QStringLiteral("true"));
            } else {
                const QRegularExpression v0(QStringLiteral("\"value0\"\\s*:\\s*\\{"));
                const auto vm = v0.match(config);
                if (!vm.hasMatch()) {
                    r.error = QStringLiteral("The base Basalt settings %1 have no value0.").arg(configPath);
                    return r;
                }
                config.insert(vm.capturedEnd(), QStringLiteral("\n    \"config.optical_flow_detection_nonoverlap\": true,"));
            }
        }
    }

    // the bias priors
    const auto baseAccel = jsonArray3(calib, QStringLiteral("calib_accel_bias"));
    const auto baseGyro = jsonArray3(calib, QStringLiteral("calib_gyro_bias"));
    if (!baseAccel || !baseGyro) {
        r.error = QStringLiteral("The base calibration %1 has no calib_accel_bias / calib_gyro_bias.").arg(calibPath);
        return r;
    }
    const FactoryImu f = readFactoryImu(in.factoryXml.isEmpty() ? QByteArray() : readAll(in.factoryXml));
    if (in.accelBias == AccelBias::Off) {
        r.accel = {0, 0, 0};
        r.accelFrom = QStringLiteral("off");
    } else if (f.aBias) {
        r.accel = *f.aBias;
        r.accelFrom = QStringLiteral("factory calibration");
    } else {
        r.accel = *baseAccel;
        r.accelFrom = QStringLiteral("base config (no factory value found)");
    }
    const auto measured = in.gyroCache.isEmpty() ? std::nullopt : parseGyroBiasCache(readAll(in.gyroCache));
    if (in.gyroBias == GyroBias::Off) {
        r.gyro = {0, 0, 0};
        r.gyroFrom = QStringLiteral("off");
    } else if (in.gyroBias == GyroBias::Measured && measured) {
        r.gyro = *measured;
        r.gyroFrom = QStringLiteral("measured at rest (the driver's cache)");
    } else if (f.wBias) {
        r.gyro = *f.wBias;
        r.gyroFrom = in.gyroBias == GyroBias::Measured ? QStringLiteral("factory calibration (nothing measured yet)")
                                                       : QStringLiteral("factory calibration");
    } else {
        r.gyro = *baseGyro;
        r.gyroFrom = QStringLiteral("base config (no measured or factory value found)");
    }
    // [bias, scale/misalignment...]: the scale and misalignment terms stay 0 (never the factory ka / na)
    QStringList acc{num(r.accel[0]), num(r.accel[1]), num(r.accel[2])};
    QStringList gyr{num(r.gyro[0]), num(r.gyro[1]), num(r.gyro[2])};
    for (int i = 0; i < 6; i++) {
        acc << QStringLiteral("0.0");
    }
    for (int i = 0; i < 9; i++) {
        gyr << QStringLiteral("0.0");
    }
    setJsonArray(calib, QStringLiteral("calib_accel_bias"), acc);
    setJsonArray(calib, QStringLiteral("calib_gyro_bias"), gyr);

    // the IMU noise (white-noise densities): measured instead of Pimax's Windows values
    if (in.measuredImuNoise) {
        // The Crystal Super's IMU at rest (desk take 2026-10-19 05:27, `omarchyvr-slam imu-noise`, x3 inflated).
        std::array<double, 3> an{0.000714, 0.000865, 0.00079};
        std::array<double, 3> gn{9.07e-5, 8.57e-5, 8.24e-5};
        r.noiseFrom = QStringLiteral("measured (Crystal Super, built in)");
        QString file = in.imuNoiseFile;
        if (file.isEmpty()) {
            file = qEnvironmentVariable("PIMAX_SLAM_IMU");
        }
        if (file.isEmpty()) {
            file = QDir::homePath() + QStringLiteral("/.config/omarchyvr/slam-imu.json");
        }
        if (QFileInfo(file).isFile()) {
            const QString t = QString::fromUtf8(readAll(file));
            const auto fa = jsonArray3(t, QStringLiteral("accel_noise_std"));
            const auto fg = jsonArray3(t, QStringLiteral("gyro_noise_std"));
            if (fa && fg) {
                an = *fa;
                gn = *fg;
                r.noiseFrom = QStringLiteral("measured (%1)").arg(file);
            }
        }
        if (!setJsonArray(calib, QStringLiteral("accel_noise_std"), {num(an[0]), num(an[1]), num(an[2])}) ||
            !setJsonArray(calib, QStringLiteral("gyro_noise_std"), {num(gn[0]), num(gn[1]), num(gn[2])})) {
            r.noiseFrom.clear(); // a base without the arrays keeps its own (scalar) values
        }
    }

    const QString outToml = in.outDir + QStringLiteral("/pimax-basalt.toml");
    const QString outCalib = in.outDir + QStringLiteral("/pimax-basalt-calib.json");
    const QString outConfig = in.outDir + QStringLiteral("/pimax-basalt-config.json");
    QString t = QStringLiteral("# Basalt SLAM options, written by PiBridge at every inside-out start from %1 (read only).\n"
                               "# PiBridge's own settings: detection grid %2 px, accel bias %3, gyro bias %4.\n")
                    .arg(in.baseToml)
                    .arg(in.grid)
                    .arg(r.accelFrom, r.gyroFrom);
    if (!r.noiseFrom.isEmpty()) {
        t += QStringLiteral("# IMU noise %1.\n").arg(r.noiseFrom);
    }
    if (!in.camsCalib.isEmpty()) {
        t += QStringLiteral("# %1 cameras, from %2.\n").arg(r.cams).arg(in.camsCalib);
    }
    for (const QString& raw : toml.split(QLatin1Char('\n'))) {
        const QString l = raw.trimmed();
        const QString key = l.section(QLatin1Char('='), 0, 0).trimmed();
        if (l.isEmpty() || l.startsWith(QLatin1Char('#'))) {
            continue; // the base's comments describe the base, not this copy
        }
        if (key == QLatin1String("cam-calib")) {
            t += QStringLiteral("cam-calib=\"%1\"\n").arg(outCalib);
        } else if (key == QLatin1String("config-path")) {
            t += QStringLiteral("config-path=\"%1\"\n").arg(outConfig);
        } else {
            t += raw + QLatin1Char('\n');
        }
    }
    if (in.write) {
        if (!writeAtomic(outConfig, config.toUtf8()) || !writeAtomic(outCalib, calib.toUtf8()) ||
            !writeAtomic(outToml, t.toUtf8())) {
            r.error = QStringLiteral("Could not write PiBridge's Basalt config into %1.").arg(in.outDir);
            return r;
        }
    }
    r.ok = true;
    r.toml = outToml;
    return r;
}

int basaltCalibCams(const QByteArray& json) {
    const QJsonDocument d = QJsonDocument::fromJson(json);
    const QJsonValue v = d.object().value(QStringLiteral("value0"));
    if (!v.isObject()) {
        return 0;
    }
    const QJsonArray a = v.toObject().value(QStringLiteral("intrinsics")).toArray();
    return a.size() == v.toObject().value(QStringLiteral("T_imu_cam")).toArray().size() ? int(a.size()) : 0;
}

QStringList fourCamExportArgs(const QString& outDir, const QString& factoryXml, const QString& baseConfig) {
    QStringList a{QStringLiteral("export"), QStringLiteral("--out"), outDir, QStringLiteral("--calib"), factoryXml,
                  QStringLiteral("--cams"), QStringLiteral("0,1,2,3")};
    if (!baseConfig.isEmpty()) {
        a << QStringLiteral("--base") << baseConfig;
    }
    return a;
}

SlamTomlFiles slamTomlFiles(const QString& toml) {
    if (toml.isEmpty()) {
        return {};
    }
    const QString t = QString::fromUtf8(readAll(toml));
    return {resolveNear(tomlValue(t, QStringLiteral("cam-calib")), toml),
            resolveNear(tomlValue(t, QStringLiteral("config-path")), toml)};
}

QString writeSyntheticSlamBase(const QString& dir) {
    const QByteArray calib =
        "{\n    \"value0\": {\n        \"T_imu_cam\": [],\n        \"intrinsics\": [],\n"
        "        \"calib_accel_bias\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\n"
        "        \"calib_gyro_bias\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\n"
        "        \"imu_update_rate\": 1000.0\n    }\n}\n";
    const QByteArray config = "{\n  \"value0\": {\n    \"config.optical_flow_type\": \"frame_to_frame\",\n"
                              "    \"config.optical_flow_detection_grid_size\": 40,\n"
                              "    \"config.vio_max_states\": 3\n  }\n}\n";
    const QByteArray toml = "# synthetic base config (tests, fake mode)\nshow-gui=0\ncam-calib=\"base-calib.json\"\n"
                            "config-path=\"base-config.json\"\nmarg-data=\"\"\nnum-threads=0\n";
    const QByteArray xml = "<?xml version=\"1.0\"?>\n<Calib><SFConfig><Stateinit ombc=\"0 0 0\" tbc=\"0 0 0\" "
                           "aBias=\"-0.05 0.34 -0.02\" wBias=\"-0.001 0.004 0.023\" ka=\"0.05 0.014 0.012\" "
                           "na=\"0.01 0.02 0.03\"/></SFConfig></Calib>\n";
    if (!writeAtomic(dir + QStringLiteral("/base-calib.json"), calib) ||
        !writeAtomic(dir + QStringLiteral("/base-config.json"), config) ||
        !writeAtomic(dir + QStringLiteral("/factory.xml"), xml) || !writeAtomic(dir + QStringLiteral("/base.toml"), toml)) {
        return {};
    }
    return dir + QStringLiteral("/base.toml");
}

// ---------------------------------------------------------------- helpers

QStringList drmConnectors(const QString& drmRoot) {
    QStringList on, off;
    const QDir d(drmRoot);
    for (const QString& e : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::System, QDir::Name)) {
        const qsizetype dash = e.indexOf(QLatin1Char('-'));
        if (!e.startsWith(QLatin1String("card")) || dash < 0) {
            continue;
        }
        const QString name = e.mid(dash + 1);
        const bool connected =
            QString::fromLatin1(readAll(drmRoot + QLatin1Char('/') + e + QStringLiteral("/status"))).trimmed() ==
            QLatin1String("connected");
        QStringList& into = connected ? on : off;
        if (!on.contains(name) && !off.contains(name)) {
            into << name;
        }
    }
    return on + off;
}

QString rttimePreloadFor(const QString& monadoService) {
    if (monadoService.isEmpty()) {
        return {};
    }
    QDir d = QFileInfo(monadoService).absoluteDir();
    for (int i = 0; i <= 4; i++) {
        const QString p = d.filePath(QStringLiteral("steamvr-monado/bin/linux64/pimax_rttime_preload.so"));
        if (QFileInfo(p).isFile()) {
            return p;
        }
        if (!d.cdUp()) {
            break;
        }
    }
    return {};
}

} // namespace pibridge::app
