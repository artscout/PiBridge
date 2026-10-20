// SPDX-License-Identifier: MIT
// Tests of the inside-out settings (core/inside_out.h, core/io_session.h): defaults == recommendations, the pibridge.conf
// round trip, the one environment builder for each setting, the generation of PiBridge's Basalt config from synthetic
// base files, the recorder's copy of the tracker's calibration, and START TRACKING (RecSession Mode::Live) with
// FakeRecProcesses and an injected clock. Nothing is ever started or signalled; every file is in a temporary dir.
#include <cmath>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <algorithm>
#include <csignal>

#include "core/apps_runtime.h"
#include "core/conf.h"
#include "core/inside_out.h"
#include "core/io_session.h"
#include "core/rec_session.h"
#include "core/rec_setup.h"
#include "core/roommap_link.h"
#include "pibridge_test.h"

using namespace pibridge::app;

namespace {

void writeAll(const QString& p, const QByteArray& b) {
    QDir().mkpath(QFileInfo(p).absolutePath());
    QFile f(p);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(b);
    }
}

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

// Points every XDG dir into a temporary dir, so slamDir() and the caches are never the user's.
struct Home {
    QTemporaryDir dir;
    Home() {
        for (const char* v : {"XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_STATE_HOME", "XDG_DATA_HOME", "XDG_RUNTIME_DIR"}) {
            const QString d = dir.path() + QLatin1Char('/') + QString::fromLatin1(v).toLower();
            QDir().mkpath(d);
            qputenv(v, QFile::encodeName(d));
        }
    }
};

Conf confWith(const QString& path, const std::vector<std::pair<std::string, std::string>>& lines) {
    std::string text;
    for (const auto& [k, v] : lines) {
        text = confSet(text, k, v);
    }
    writeAll(path, QByteArray::fromStdString(text));
    return Conf(path);
}

bool has(const QStringList& l, const char* s) {
    return l.contains(QString::fromLatin1(s));
}

bool hasPrefix(const QStringList& l, const char* p) {
    for (const QString& s : l) {
        if (s.startsWith(QLatin1String(p))) {
            return true;
        }
    }
    return false;
}

} // namespace

// ---------------------------------------------------------------- settings

TEST(defaults_are_the_recommendations) {
    Home h;
    const Conf empty(h.dir.path() + QStringLiteral("/none.conf"));
    const IoSettings s = ioSettings(empty);
    CHECK(s == IoSettings::recommended());
    // the values the sources recommend (docs/research/slam-imu-intrinsics.md, crystal-6dof.sh, pimax_ctrl.c, fuse.rs)
    CHECK(s.mode == TrackingMode::Lighthouse); // what works today
    CHECK(s.apps == IoApps::SteamVr);
    CHECK(s.homeScene); // the home scene in the headset: on
    CHECK(s.slam && s.grid == 30);
    CHECK(s.accelBias == AccelBias::Factory && s.gyroBias == GyroBias::Measured);
    CHECK(!s.driftHold); // Crystal Super: off (third recording: 273 mm mark error with it, 74 mm without)
    CHECK(s.stillHold && s.headTilt && s.fuseAuto && s.smooth);
    CHECK(!s.anchor && s.anchorGlide);
    CHECK(s.camAe == CamAe::On && s.camAeCrystal && s.camAeNewLaw);
    CHECK(s.ctrlTrack && s.ctrlAngle == 30 && s.ctrlLostS == 4 && s.ctrlSteady);
    // "Reset to recommended" writes every io_* key but not the mode or how apps run
    const auto lines = ioRecommendedLines();
    CHECK(lines.size() == ioConfLines(s).size() - 3); // not the mode, apps or the home scene
    for (const auto& [k, v] : lines) {
        CHECK(k.starts_with("io_"));
        CHECK(k != iokey::kApps && k != iokey::kHome);
    }
}

TEST(conf_round_trip_and_clamping) {
    Home h;
    IoSettings s;
    s.mode = TrackingMode::InsideOut;
    s.apps = IoApps::Monado;
    s.homeScene = false;
    s.slam = false;
    s.grid = 40;
    s.accelBias = AccelBias::Off;
    s.gyroBias = GyroBias::Factory;
    s.driftHold = true;
    s.stillHold = false;
    s.headTilt = false;
    s.fuseAuto = false;
    s.smooth = false;
    s.anchor = true;
    s.anchorGlide = false;
    s.camAe = CamAe::Dry;
    s.camAeCrystal = false;
    s.camAeNewLaw = false;
    s.ctrlTrack = false;
    s.ctrlAngle = -15;
    s.ctrlLostS = 7;
    s.ctrlSteady = false;
    const Conf c = confWith(h.dir.path() + QStringLiteral("/a.conf"), ioConfLines(s));
    CHECK(ioSettings(c) == s);
    // the written words are the documented ones
    CHECK(c.value(iokey::kMode, {}) == QStringLiteral("inside-out") && c.value(iokey::kCamAe, {}) == QStringLiteral("dry"));
    CHECK(c.value(iokey::kGyroBias, {}) == QStringLiteral("factory") && c.value(iokey::kCamAeLaw, {}) == QStringLiteral("old"));
    // out of range and broken values
    const Conf b = confWith(h.dir.path() + QStringLiteral("/b.conf"),
                            {{iokey::kCtrlAngle, "99"}, {iokey::kCtrlLost, "0"}, {iokey::kGrid, "37"},
                             {iokey::kCamAe, "maybe"}, {iokey::kGyroBias, "x"}, {iokey::kMode, "INSIDE-OUT"}});
    const IoSettings t = ioSettings(b);
    CHECK(t.ctrlAngle == kCtrlAngleMax && t.ctrlLostS == kCtrlLostMin && t.grid == 40);
    CHECK(t.camAe == CamAe::On && t.gyroBias == GyroBias::Measured && t.mode == TrackingMode::InsideOut);
    CHECK(ioSettings(confWith(h.dir.path() + QStringLiteral("/c.conf"), {{iokey::kCamAe, "off"}})).camAe == CamAe::Off);
    CHECK(ioSettings(confWith(h.dir.path() + QStringLiteral("/d.conf"), {{iokey::kGrid, "55"}})).grid == 50);
}

// ---------------------------------------------------------------- the environment

TEST(env_of_the_recommended_settings) {
    RecSetup s = fakeRecSetup(QStringLiteral("/r"));
    const IoSessionEnv e = buildIoEnv(IoSettings::recommended(), s, 90, QStringLiteral("/c/slam/pimax-basalt.toml"), {});
    // crystal-6dof.sh's SETTINGS, in its order, then the switches
    CHECK(e.driver.mid(0, 3) == (QStringList{QStringLiteral("PIMAX_FPS=90"), QStringLiteral("PIMAX_LOG=info"),
                                             QStringLiteral("PIMAX_BIAS_TIMEOUT_MS=200")}));
    for (const char* want : {"PIMAX_LENS_CONF=/fake/lens/crystal.conf", "PIMAX_CAM_SOURCE=broker", "PIMAX_SLAM=1",
                             "PIMAX_CAM_AE=1", "PIMAX_CAM_AE_MODEL=crystal-super", "PIMAX_CTRL_TRACK=1",
                             "PIMAX_CTRL_ANGLE=30", "VIT_SYSTEM_LIBRARY_PATH=/fake/basalt/libbasalt.so",
                             "SLAM_CONFIG=/c/slam/pimax-basalt.toml", "PIMAX_SLAM_DRIFT_HOLD=0"}) {
        CHECK(has(e.driver, want));
    }
    // switches at the driver's own default are not passed
    for (const char* none : {"PIMAX_SLAM_STILL", "PIMAX_HEAD_TILT", "PIMAX_FUSE_AUTO", "PIMAX_SLAM_SMOOTH", "PIMAX_ANCHOR_GLIDE",
                             "PIMAX_CTRL_TRACK_LOST_S", "PIMAX_CTRL_TRACK_HYST", "PIMAX_CAM_AE_LAW", "PIMAX_SLAM_RECORD"}) {
        CHECK(!hasPrefix(e.monado, none));
    }
    // the room anchor hook: on because the room-map tool is there (fakeRecSetup has one); off without it and the switch
    CHECK(has(e.driver, "PIMAX_ANCHOR=1"));
    RecSetup nr = s;
    nr.roommap.clear();
    CHECK(!hasPrefix(buildIoEnv(IoSettings::recommended(), nr, 90, {}, {}).driver, "PIMAX_ANCHOR"));
    CHECK(e.ctrltrack && e.notes.isEmpty());
    // monado-service: the driver plus the compositor of crystal-6dof.sh
    CHECK(e.monado.mid(0, e.driver.size()) == e.driver);
    for (const char* want : {"XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1", "XRT_COMPOSITOR_WAYLAND_CONNECTOR=DP-6",
                             "XRT_COMPOSITOR_FORCE_NVIDIA=0", "XRT_COMPOSITOR_LOG=info", "XRT_NO_STDIN=1",
                             "IPC_IGNORE_VERSION=1"}) {
        CHECK(has(e.monado, want));
        CHECK(!has(e.driver, want));
    }
    // the recorder builds its env with the same builder
    s.io = IoSettings::recommended();
    s.sessionSlamConfig = QStringLiteral("/c/slam/pimax-basalt.toml");
    CHECK(monadoEnv(s, QStringLiteral("/r/x")) ==
          buildIoEnv(s.io, s, 90, s.sessionSlamConfig, QStringLiteral("/r/x")).monado);
    CHECK(has(monadoEnv(s, QStringLiteral("/r/x")), "PIMAX_SLAM_RECORD=/r/x"));
}

TEST(env_follows_every_switch) {
    RecSetup s = fakeRecSetup(QStringLiteral("/r"));
    s.extraEnv = {QStringLiteral("PIMAX_LOG=debug")};
    IoSettings io;
    io.driftHold = true;
    io.stillHold = false;
    io.headTilt = false;
    io.fuseAuto = false;
    io.smooth = false;
    io.anchor = true;
    io.anchorGlide = false;
    io.camAe = CamAe::Dry;
    io.camAeNewLaw = false;
    io.camAeCrystal = false;
    io.ctrlAngle = -20;
    io.ctrlLostS = 8;
    io.ctrlSteady = false;
    const IoSessionEnv e = buildIoEnv(io, s, 72, QString(), QStringLiteral("/rec"));
    for (const char* want : {"PIMAX_FPS=72", "PIMAX_SLAM_STILL=0", "PIMAX_HEAD_TILT=0", "PIMAX_FUSE_AUTO=0",
                             "PIMAX_SLAM_SMOOTH=0", "PIMAX_ANCHOR=1", "PIMAX_ANCHOR_GLIDE=0", "PIMAX_CAM_AE=dry",
                             "PIMAX_CAM_AE_LAW=old", "PIMAX_CTRL_ANGLE=-20", "PIMAX_CTRL_TRACK_LOST_S=8",
                             "PIMAX_CTRL_TRACK_HYST=0", "PIMAX_SLAM_RECORD=/rec"}) {
        CHECK(has(e.monado, want));
    }
    CHECK(!hasPrefix(e.monado, "PIMAX_SLAM_DRIFT_HOLD") && !hasPrefix(e.monado, "PIMAX_CAM_AE_MODEL"));
    CHECK(!hasPrefix(e.monado, "SLAM_CONFIG")); // none given: the driver's calibration ...
    CHECK(e.notes.size() == 1);                 // ... and a note that says so
    CHECK(e.monado.last() == QStringLiteral("PIMAX_LOG=debug") && e.driver.last() == QStringLiteral("PIMAX_LOG=debug"));

    // camera AE off: no model, no law
    IoSettings off;
    off.camAe = CamAe::Off;
    off.camAeNewLaw = false;
    const IoSessionEnv o = buildIoEnv(off, s, 90, {}, {});
    CHECK(has(o.driver, "PIMAX_CAM_AE=0") && !hasPrefix(o.driver, "PIMAX_CAM_AE_MODEL") && !hasPrefix(o.driver, "PIMAX_CAM_AE_LAW"));

    // SLAM off: no Basalt, no output stages
    IoSettings ns;
    ns.slam = false;
    ns.stillHold = false;
    const IoSessionEnv n = buildIoEnv(ns, s, 90, QStringLiteral("/x.toml"), {});
    CHECK(has(n.driver, "PIMAX_SLAM=0") && !hasPrefix(n.driver, "VIT_") && !hasPrefix(n.driver, "SLAM_CONFIG") &&
          !hasPrefix(n.driver, "PIMAX_SLAM_STILL") && !hasPrefix(n.driver, "PIMAX_SLAM_DRIFT_HOLD"));

    // Basalt or the tracker missing: off, with a note each
    RecSetup m = s;
    m.basalt.path.clear();
    m.ctrl.path.clear();
    const IoSessionEnv x = buildIoEnv(IoSettings{}, m, 90, QStringLiteral("/x.toml"), {});
    CHECK(has(x.driver, "PIMAX_SLAM=0") && has(x.driver, "PIMAX_CTRL_TRACK=0") && !x.ctrltrack);
    CHECK(x.notes.size() == 2);

    // the SteamVR variant: driver_monado's file and SteamVR's launch env (crystal-6dof.sh steamvr)
    const IoSessionEnv v = buildIoEnv(IoSettings{}, s, 90, QStringLiteral("/c.toml"), {}, QStringLiteral("/cfg/steamvr-driver.env"),
                                      QStringLiteral("/m/steamvr-monado/bin/linux64/pimax_rttime_preload.so"));
    CHECK((v.steamvr == QStringList{QStringLiteral("VALVE_SKIP_RUNTIME_SAFETY=1"), QStringLiteral("QT_QPA_PLATFORM=xcb"),
                                    QStringLiteral("LD_PRELOAD=/m/steamvr-monado/bin/linux64/pimax_rttime_preload.so"),
                                    QStringLiteral("PIMAX_STEAMVR_ENV_FILE=/cfg/steamvr-driver.env")}));
    const QString file = driverEnvFileText(v);
    CHECK(file.startsWith(QStringLiteral("PIMAX_FPS=90\nPIMAX_LOG=info\n")) && file.endsWith(QLatin1Char('\n')));
    CHECK(!file.contains(QStringLiteral("XRT_COMPOSITOR")));
}

TEST(rttime_preload_and_connectors) {
    QTemporaryDir d;
    const QString build = d.path() + QStringLiteral("/build");
    writeAll(build + QStringLiteral("/src/xrt/targets/service/monado-service"), "x");
    writeAll(build + QStringLiteral("/steamvr-monado/bin/linux64/pimax_rttime_preload.so"), "x");
    CHECK(rttimePreloadFor(build + QStringLiteral("/src/xrt/targets/service/monado-service")) ==
          build + QStringLiteral("/steamvr-monado/bin/linux64/pimax_rttime_preload.so"));
    CHECK(rttimePreloadFor(d.path() + QStringLiteral("/other/monado-service")).isEmpty());
    const QString drm = d.path() + QStringLiteral("/drm");
    writeAll(drm + QStringLiteral("/card1-HDMI-A-1/status"), "disconnected\n");
    writeAll(drm + QStringLiteral("/card1-DP-6/status"), "connected\n");
    writeAll(drm + QStringLiteral("/card0-DP-6/status"), "disconnected\n");
    QDir().mkpath(drm + QStringLiteral("/renderD128"));
    CHECK((drmConnectors(drm) == QStringList{QStringLiteral("DP-6"), QStringLiteral("HDMI-A-1")}));
}

// ---------------------------------------------------------------- Basalt config

TEST(factory_imu_and_gyro_cache) {
    const FactoryImu f = readFactoryImu(
        "<a><SFConfig><Stateinit ombc=\"1 2 3\" aBias=\"-0.05 0.34 -0.02\" wBias=\"-0.001 0.004 0.023\"/></SFConfig></a>");
    CHECK(f.aBias && f.wBias);
    CHECK_NEAR((*f.aBias)[1], 0.34, 1e-12);
    CHECK_NEAR((*f.wBias)[2], 0.023, 1e-12);
    CHECK(!readFactoryImu("<a><Stateinit aBias=\"0 3 0\" wBias=\"0 0\"/></a>").aBias); // implausible, too few
    CHECK(!readFactoryImu("not xml").aBias);
    // the cache: raw axes deg/s -> body frame (x and z negated) rad/s
    const auto g = parseGyroBiasCache("0.5 -1.0 1.35\n");
    CHECK(g.has_value());
    CHECK_NEAR((*g)[0], -0.5 * M_PI / 180.0, 1e-12);
    CHECK_NEAR((*g)[1], -1.0 * M_PI / 180.0, 1e-12);
    CHECK_NEAR((*g)[2], -1.35 * M_PI / 180.0, 1e-12);
    CHECK(!parseGyroBiasCache("6 0 0").has_value() && !parseGyroBiasCache("1 2").has_value() &&
          !parseGyroBiasCache("a b c").has_value());
}

TEST(basalt_config_generation) {
    QTemporaryDir d;
    const QString base = writeSyntheticSlamBase(d.path() + QStringLiteral("/base"));
    CHECK(!base.isEmpty());
    const QByteArray baseCalib = readAll(d.path() + QStringLiteral("/base/base-calib.json"));
    const QByteArray baseConfig = readAll(d.path() + QStringLiteral("/base/base-config.json"));
    writeAll(d.path() + QStringLiteral("/gyro-bias"), "0.5 -1.0 1.35\n");
    SlamGenInput in;
    in.baseToml = base;
    in.outDir = d.path() + QStringLiteral("/out");
    in.factoryXml = d.path() + QStringLiteral("/base/factory.xml");
    in.gyroCache = d.path() + QStringLiteral("/gyro-bias");
    in.grid = 30;

    // a preview writes nothing
    in.write = false;
    SlamGenResult p = generateSlamConfig(in);
    CHECK(p.ok && !QFileInfo::exists(in.outDir));
    in.write = true;

    const SlamGenResult r = generateSlamConfig(in);
    CHECK(r.ok && r.toml == in.outDir + QStringLiteral("/pimax-basalt.toml"));
    const QString toml = QString::fromUtf8(readAll(r.toml));
    CHECK(toml.contains(QStringLiteral("cam-calib=\"%1/pimax-basalt-calib.json\"").arg(in.outDir)));
    CHECK(toml.contains(QStringLiteral("config-path=\"%1/pimax-basalt-config.json\"").arg(in.outDir)));
    CHECK(toml.contains(QStringLiteral("num-threads=0")) && toml.contains(QStringLiteral("show-gui=0")));
    CHECK(slamTomlFiles(r.toml).calib == in.outDir + QStringLiteral("/pimax-basalt-calib.json"));
    // the grid, every other setting kept
    const QString config = QString::fromUtf8(readAll(in.outDir + QStringLiteral("/pimax-basalt-config.json")));
    CHECK(config.contains(QStringLiteral("\"config.optical_flow_detection_grid_size\": 30,")));
    CHECK(QString(config).replace(QStringLiteral("grid_size\": 30"), QStringLiteral("grid_size\": 40")).toUtf8() == baseConfig);
    // accel = factory aBias, gyro = measured, scale and misalignment 0 (never ka / na)
    const QString calib = QString::fromUtf8(readAll(in.outDir + QStringLiteral("/pimax-basalt-calib.json")));
    CHECK(calib.contains(QStringLiteral("\"calib_accel_bias\": [-0.05, 0.34, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")));
    CHECK(r.accelFrom == QStringLiteral("factory calibration"));
    CHECK(r.gyroFrom.startsWith(QStringLiteral("measured")));
    CHECK_NEAR(r.gyro[2], -1.35 * M_PI / 180.0, 1e-12);
    CHECK(calib.contains(QStringLiteral("\"T_imu_cam\": []")) && calib.contains(QStringLiteral("\"imu_update_rate\": 1000.0")));
    CHECK(QFileInfo(base).isFile() && readAll(d.path() + QStringLiteral("/base/base-calib.json")) == baseCalib); // read only

    // no measured bias yet: the factory wBias; then the base config's value
    in.gyroCache = d.path() + QStringLiteral("/none");
    SlamGenResult f = generateSlamConfig(in);
    CHECK(f.ok && f.gyroFrom.startsWith(QStringLiteral("factory")));
    CHECK_NEAR(f.gyro[1], 0.004, 1e-12);
    in.factoryXml.clear();
    f = generateSlamConfig(in);
    CHECK(f.ok && f.gyroFrom.startsWith(QStringLiteral("base")) && f.accelFrom.startsWith(QStringLiteral("base")));
    // off: zeros
    in.accelBias = AccelBias::Off;
    in.gyroBias = GyroBias::Off;
    in.grid = 50;
    f = generateSlamConfig(in);
    const QString z = QString::fromUtf8(readAll(in.outDir + QStringLiteral("/pimax-basalt-calib.json")));
    CHECK(f.ok && z.contains(QStringLiteral("\"calib_accel_bias\": [0, 0, 0, 0.0")) &&
          z.contains(QStringLiteral("\"calib_gyro_bias\": [0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")));
    CHECK(readAll(in.outDir + QStringLiteral("/pimax-basalt-config.json")).contains("grid_size\": 50"));
    // broken bases say why
    in.baseToml = d.path() + QStringLiteral("/missing.toml");
    CHECK(!generateSlamConfig(in).ok && !generateSlamConfig(in).error.isEmpty());
    writeAll(d.path() + QStringLiteral("/b2/base.toml"), "cam-calib=\"c.json\"\nconfig-path=\"k.json\"\n");
    writeAll(d.path() + QStringLiteral("/b2/c.json"), "{\"value0\": {}}");
    writeAll(d.path() + QStringLiteral("/b2/k.json"), "{\"value0\": {\"config.optical_flow_detection_grid_size\": 40}}");
    in.baseToml = d.path() + QStringLiteral("/b2/base.toml");
    const SlamGenResult nb = generateSlamConfig(in);
    CHECK(!nb.ok && nb.error.contains(QStringLiteral("calib_accel_bias")));
}

TEST(prepare_session_slam_points_the_setup_at_pibridges_copy) {
    Home h;
    RecSetup s = fakeRecSetup(h.dir.path() + QStringLiteral("/rec"));
    s.slamConfig.path = writeSyntheticSlamBase(h.dir.path() + QStringLiteral("/base"));
    s.calib.path = h.dir.path() + QStringLiteral("/base/factory.xml");
    QString note;
    prepareSessionSlam(s, &note);
    CHECK(note.isEmpty() && s.sessionSlamConfig == slamDir() + QStringLiteral("/pimax-basalt.toml"));
    CHECK(slamDir().startsWith(h.dir.path()));
    CHECK(has(monadoEnv(s, {}), (QStringLiteral("SLAM_CONFIG=") + s.sessionSlamConfig).toLatin1().constData()));
    // SLAM off: nothing generated, the base is not named either
    s.io.slam = false;
    prepareSessionSlam(s, &note);
    CHECK(s.sessionSlamConfig.isEmpty() && !hasPrefix(monadoEnv(s, {}), "SLAM_CONFIG"));
}

// ---------------------------------------------------------------- four head tracking cameras (ROADMAP 1b)

namespace {

// A made-up Basalt calibration of `n` cameras (no headset's numbers): camera i has fx = 200 + i.
QByteArray fakeCalib(int n) {
    QByteArray t, in, res;
    for (int i = 0; i < n; i++) {
        const QByteArray sep = i ? ",\n" : "";
        t += sep + "            {\"px\": 0." + QByteArray::number(i) + ", \"py\": 0.0, \"pz\": 0.0, \"qx\": 0.0, "
             "\"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}";
        in += sep + "            {\"camera_type\": \"kb4\", \"intrinsics\": {\"fx\": " + QByteArray::number(200 + i) +
              ", \"fy\": 200.0, \"cx\": 320.0, \"cy\": 240.0, \"k1\": 0.0, \"k2\": 0.0, \"k3\": 0.0, \"k4\": 0.0}}";
        res += sep + "            [640, 480]";
    }
    return "{\n    \"value0\": {\n        \"T_imu_cam\": [\n" + t + "\n        ],\n        \"intrinsics\": [\n" + in +
           "\n        ],\n        \"resolution\": [\n" + res + "\n        ],\n"
           "        \"calib_accel_bias\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\n"
           "        \"calib_gyro_bias\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\n"
           "        \"accel_noise_std\": [0.08, 0.08, 0.08],\n        \"imu_update_rate\": 1000.0\n    }\n}\n";
}

// A base like the live one: two cameras, nonoverlap off (to see it switched on), grid 40.
QString writeTwoCamBase(const QString& dir) {
    writeAll(dir + QStringLiteral("/calib.json"), fakeCalib(2));
    writeAll(dir + QStringLiteral("/config.json"),
             "{\n  \"value0\": {\n    \"config.optical_flow_detection_grid_size\": 40,\n"
             "    \"config.optical_flow_detection_nonoverlap\": false,\n    \"config.vio_max_states\": 3\n  }\n}\n");
    writeAll(dir + QStringLiteral("/base.toml"), "show-gui=0\ncam-calib=\"calib.json\"\nconfig-path=\"config.json\"\n");
    writeAll(dir + QStringLiteral("/factory.xml"), "<Calib/>\n");
    return dir + QStringLiteral("/base.toml");
}

// A fake pibridge-slam: `export --out DIR ...` writes `calib` there as pimax-basalt-calib.json and its arguments into
// DIR/args (exit 0); with no `calib`, it fails.
QString writeFakeSlamTool(const QString& dir, const QByteArray& calib) {
    const QString tool = dir + QStringLiteral("/pibridge-slam");
    QByteArray sh = "#!/bin/sh\nout=\"\"\nprev=\"\"\nfor a in \"$@\"; do [ \"$prev\" = --out ] && out=\"$a\"; prev=\"$a\"; done\n";
    if (calib.isEmpty()) {
        sh += "echo 'pibridge-slam: no camera 3 in this calibration' >&2\nexit 1\n";
    } else {
        writeAll(dir + QStringLiteral("/calib4.json"), calib);
        sh += "mkdir -p \"$out\" && cp '" + QFile::encodeName(dir) + "/calib4.json' \"$out/pimax-basalt-calib.json\" && "
              "echo \"$@\" > \"$out/args\"\n";
    }
    writeAll(tool, sh);
    QFile::setPermissions(tool, QFile::permissions(tool) | QFileDevice::ExeOwner);
    return tool;
}

} // namespace

TEST(four_cameras_setting_and_environment) {
    Home h;
    // pibridge.conf: 2 by default, 4 only when it says 4
    CHECK(IoSettings{}.slamCams == 2 && IoSettings::recommended().slamCams == 2);
    const QString cp = h.dir.path() + QStringLiteral("/f.conf");
    CHECK(ioSettings(confWith(cp, {{iokey::kSlamCams, "4"}})).slamCams == 4);
    CHECK(ioSettings(confWith(cp, {{iokey::kSlamCams, "3"}})).slamCams == 2);
    CHECK(ioSettings(confWith(cp, {{iokey::kSlamCams, "x"}})).slamCams == 2);
    IoSettings four;
    four.slamCams = 4;
    CHECK(ioSettings(confWith(cp, ioConfLines(four))) == four);
    bool listed = false;
    for (const auto& [k, v] : ioRecommendedLines()) {
        listed = listed || (k == iokey::kSlamCams && v == "2");
    }
    CHECK(listed);

    RecSetup s = fakeRecSetup(h.dir.path() + QStringLiteral("/rec"));
    const QString toml = QStringLiteral("/c/slam/pimax-basalt.toml");
    // two cameras: no PIMAX_SLAM_CAMS (the driver's default 0,1), the environment of today
    CHECK(!hasPrefix(buildIoEnv(IoSettings{}, s, 90, toml, {}).driver, "PIMAX_SLAM_CAMS"));
    // four without a four-camera calibration: the lower pair, said in plain words
    const IoSessionEnv no = buildIoEnv(four, s, 90, toml, {});
    CHECK(!hasPrefix(no.driver, "PIMAX_SLAM_CAMS"));
    CHECK(std::any_of(no.notes.begin(), no.notes.end(), [](const QString& n) { return n.contains(QStringLiteral("lower camera pair")); }));
    // four with one (prepareSessionSlam made it), or with the driver's own calibration (no SLAM_CONFIG)
    s.sessionSlamCams = 4;
    CHECK(has(buildIoEnv(four, s, 90, toml, {}).driver, "PIMAX_SLAM_CAMS=0,1,2,3"));
    CHECK(has(buildIoEnv(four, s, 90, toml, QStringLiteral("/r")).monado, "PIMAX_SLAM_CAMS=0,1,2,3"));
    s.sessionSlamCams = 0;
    CHECK(has(buildIoEnv(four, s, 90, {}, {}).driver, "PIMAX_SLAM_CAMS=0,1,2,3"));
    // SLAM off: no cameras at all
    IoSettings off = four;
    off.slam = false;
    CHECK(!hasPrefix(buildIoEnv(off, s, 90, toml, {}).driver, "PIMAX_SLAM_CAMS"));

    CHECK(fourCamExportArgs(QStringLiteral("/o"), QStringLiteral("/f.xml"), QStringLiteral("/b.json")) ==
          (QStringList{QStringLiteral("export"), QStringLiteral("--out"), QStringLiteral("/o"), QStringLiteral("--calib"),
                       QStringLiteral("/f.xml"), QStringLiteral("--cams"), QStringLiteral("0,1,2,3"),
                       QStringLiteral("--base"), QStringLiteral("/b.json")}));
    CHECK(basaltCalibCams(fakeCalib(4)) == 4 && basaltCalibCams(fakeCalib(2)) == 2 && basaltCalibCams("{}") == 0);
}

TEST(four_camera_basalt_config) {
    QTemporaryDir d;
    SlamGenInput in;
    in.baseToml = writeTwoCamBase(d.path() + QStringLiteral("/base"));
    in.outDir = d.path() + QStringLiteral("/out");
    in.factoryXml = d.path() + QStringLiteral("/base/factory.xml");
    in.grid = 30;
    // two cameras: the base's calibration and its nonoverlap value as they are
    SlamGenResult two = generateSlamConfig(in);
    CHECK(two.ok && two.cams == 2);
    const QByteArray twoCalib = readAll(in.outDir + QStringLiteral("/pimax-basalt-calib.json"));
    CHECK(readAll(in.outDir + QStringLiteral("/pimax-basalt-config.json")).contains("nonoverlap\": false"));

    // four: the camera arrays of the four-camera file, everything else of the two-camera result kept
    writeAll(d.path() + QStringLiteral("/c4.json"), fakeCalib(4));
    in.camsCalib = d.path() + QStringLiteral("/c4.json");
    const SlamGenResult r = generateSlamConfig(in);
    CHECK(r.ok && r.cams == 4);
    const QByteArray c = readAll(in.outDir + QStringLiteral("/pimax-basalt-calib.json"));
    CHECK(basaltCalibCams(c) == 4 && c.contains("\"fx\": 203"));
    const auto tail = [](const QByteArray& b) { return b.mid(b.indexOf("\"calib_accel_bias\"")); };
    CHECK(tail(c) == tail(twoCalib));
    const QByteArray cfg = readAll(in.outDir + QStringLiteral("/pimax-basalt-config.json"));
    CHECK(cfg.contains("nonoverlap\": true") && cfg.contains("grid_size\": 30") && cfg.contains("vio_max_states\": 3"));
    CHECK(readAll(r.toml).contains("# 4 cameras"));
    // a base without the key gets it
    writeAll(d.path() + QStringLiteral("/base/config.json"), "{\n  \"value0\": {\n    \"config.optical_flow_detection_grid_size\": 40\n  }\n}\n");
    CHECK(generateSlamConfig(in).ok);
    CHECK(readAll(in.outDir + QStringLiteral("/pimax-basalt-config.json")).contains("\"config.optical_flow_detection_nonoverlap\": true,"));

    // another headset's file (its first cameras are not the base's), or no calibration at all: refused, in words
    QByteArray other = fakeCalib(4);
    other.replace("\"fx\": 200", "\"fx\": 199");
    writeAll(d.path() + QStringLiteral("/c4.json"), other);
    const SlamGenResult bad = generateSlamConfig(in);
    CHECK(!bad.ok && bad.error.contains(QStringLiteral("another headset")));
    in.camsCalib = d.path() + QStringLiteral("/none.json");
    CHECK(!generateSlamConfig(in).ok);
}

TEST(prepare_session_slam_with_four_cameras) {
    Home h;
    RecSetup s = fakeRecSetup(h.dir.path() + QStringLiteral("/rec"));
    s.slamConfig.path = writeTwoCamBase(h.dir.path() + QStringLiteral("/base"));
    s.calib.path = h.dir.path() + QStringLiteral("/base/factory.xml");
    s.io.slamCams = 4;
    // pibridge-slam writes the four-camera calibration: Basalt gets four, the driver PIMAX_SLAM_CAMS=0,1,2,3
    s.slamTool = writeFakeSlamTool(h.dir.path() + QStringLiteral("/bin"), fakeCalib(4));
    QString note;
    prepareSessionSlam(s, &note);
    CHECK(note.isEmpty() && s.sessionSlamCams == 4);
    CHECK(basaltCalibCams(readAll(slamTomlFiles(s.sessionSlamConfig).calib)) == 4);
    CHECK(has(monadoEnv(s, {}), "PIMAX_SLAM_CAMS=0,1,2,3"));
    const QString args = QString::fromUtf8(readAll(slamDir() + QStringLiteral("/cams4/args")));
    CHECK(args.contains(QStringLiteral("--cams 0,1,2,3")) && args.contains(QStringLiteral("--calib ") + s.calib.path) &&
          args.contains(QStringLiteral("--base ") + h.dir.path() + QStringLiteral("/base/config.json")));
    // pibridge-slam fails: the lower pair, the reason in the note, never a stale four-camera file
    s.slamTool = writeFakeSlamTool(h.dir.path() + QStringLiteral("/bin2"), {});
    prepareSessionSlam(s, &note);
    CHECK(s.sessionSlamCams == 2 && !s.sessionSlamConfig.isEmpty());
    CHECK(note.contains(QStringLiteral("lower pair")) && note.contains(QStringLiteral("no camera 3")));
    CHECK(!QFileInfo::exists(slamDir() + QStringLiteral("/cams4/pimax-basalt-calib.json")));
    CHECK(!hasPrefix(monadoEnv(s, {}), "PIMAX_SLAM_CAMS"));
    // no pibridge-slam at all
    s.slamTool.clear();
    prepareSessionSlam(s, &note);
    CHECK(s.sessionSlamCams == 2 && note.contains(QStringLiteral("pibridge-slam was not found")));
    // two cameras: no export is run
    s.io.slamCams = 2;
    s.slamTool = QStringLiteral("/nonexistent/pibridge-slam");
    prepareSessionSlam(s, &note);
    CHECK(note.isEmpty() && s.sessionSlamCams == 2);
}

// ---------------------------------------------------------------- the sessions

namespace {

struct LiveRig {
    Home h;
    RecSetup setup;
    FakeRecProcesses* procs = nullptr;
    qint64 now = 0;
    qint64 socketAtMs = 500;
    QString monadoLog;
    std::unique_ptr<IoSessionControl> c;
    bool vrserver = false;
    // the home scene (pibridge-home): found when `homeProgram` is set; its status file says `homeStatus`
    QString homeProgram;
    QString runtimeJson = QStringLiteral("/m/openxr_monado-dev.json");
    bool socket = true;
    QString homeStatus;
    QByteArray roomStatus; // pibridge-roommap's status file

    explicit LiveRig(bool insideOut = true, bool monado = true, bool withHome = false) {
        if (withHome) {
            homeProgram = QStringLiteral("/x/pibridge-home");
        }
        setup = fakeRecSetup(h.dir.path() + QStringLiteral("/recordings"));
        setup.io.mode = insideOut ? TrackingMode::InsideOut : TrackingMode::Lighthouse;
        setup.io.apps = monado ? IoApps::Monado : IoApps::SteamVr; // the old key: read, chooses nothing any more
        IoSessionControl::Deps d;
        d.variant = monado ? IoVariant::MonadoDirect : IoVariant::SteamVr;
        auto p = std::make_unique<FakeRecProcesses>();
        procs = p.get();
        d.procs = std::move(p);
        d.setup = [this] { return setup; };
        d.system = [this] {
            RecSystem s;
            s.vrserver = vrserver;
            return s;
        };
        d.probe.socketReady = [this](const QString&) { return now >= socketAtMs; };
        d.probe.imuBytes = [](const QString&) -> qint64 { return -1; };
        d.probe.readLog = [this](const QString& path) { return path.contains(QStringLiteral("monado")) ? monadoLog : QString(); };
        d.probe.monoNs = [this] { return now * 1000000; };
        d.nowMs = [this] { return now; };
        d.autoTick = false;
        d.homeProgram = [this] { return homeProgram; };
        d.runtimeJson = [this](const QString&) { return runtimeJson; };
        d.monadoSocket = [this] { return socket; };
        d.homeStatus = [this](qint64) { return homeStatus; };
        d.roomStatus = [this] { return roomStatus; };
        d.unixNow = [this] { return 1000000 + now / 1000; };
        c = std::make_unique<IoSessionControl>(std::move(d));
    }
    void at(qint64 ms) {
        now = ms;
        c->tick();
    }
    QStringList names() const {
        QStringList n;
        for (const auto& sp : procs->started) {
            n << sp.name;
        }
        return n;
    }
};

} // namespace

TEST(start_inside_out_readiness) {
    {
        LiveRig r(false, false); // SteamVR + Monado needs the inside-out mode
        CHECK(!r.c->readiness().ready());
        CHECK(r.c->readiness().blocking.front().contains(QStringLiteral("Lighthouse")));
        CHECK(!r.c->start() && r.procs->started.empty());
    }
    {
        LiveRig r(false, true); // "Monado directly" is a debug start: any mode
        CHECK(r.c->readiness().ready());
    }
    {
        LiveRig r(true, false); // SteamVR + Monado: the setup is complete (made-up paths)
        CHECK(r.c->readiness().ready());
        CHECK(!r.c->readiness().blocking.join(QString()).contains(QStringLiteral("next step")));
    }
    {
        LiveRig r;
        CHECK(r.c->readiness().ready());
        r.vrserver = true; // SteamVR must be closed
        r.c->refresh();
        CHECK(!r.c->readiness().ready());
        CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("SteamVR is running")));
        r.vrserver = false;
        r.c->setRecorderActive(true);
        CHECK(!r.c->readiness().ready());
        r.c->setRecorderActive(false);
        CHECK(r.c->readiness().ready());
        r.c->setOtherSession(QStringLiteral("The other session runs."));
        CHECK(r.c->readiness().blocking == QStringList{QStringLiteral("The other session runs.")});
        r.c->setOtherSession({});
        CHECK(r.c->readiness().ready());
    }
    CHECK(ioChip(RecState::Recording, true).text == QStringLiteral("RUNNING"));
    CHECK(ioChip(RecState::Idle, false).text == QStringLiteral("NOT READY"));
}

TEST(start_inside_out_runs_and_stops_exactly_what_it_started) {
    LiveRig r;
    CHECK(r.c->start());
    CHECK(r.c->state() == RecState::Starting && r.names() == QStringList{QStringLiteral("broker")});
    const QString folder = r.c->session()->folder();
    CHECK(folder.startsWith(r.setup.sessionsDir + QLatin1Char('/')));
    r.at(600);
    CHECK((r.names() == QStringList{QStringLiteral("broker"), QStringLiteral("ctrltrack"), QStringLiteral("monado")}));
    const RecProcSpec& m = r.procs->started[2];
    CHECK(!hasPrefix(m.env, "PIMAX_SLAM_RECORD")); // no recording
    CHECK(m.env == monadoEnv(r.setup, {}));       // the one builder
    CHECK(r.c->state() == RecState::Starting);
    r.at(600 + kIoSettleMs);
    CHECK(r.c->state() == RecState::Recording && r.c->active());
    CHECK(r.c->message().startsWith(QStringLiteral("Tracking runs.")));
    const qint64 broker = r.procs->pidOf(QStringLiteral("broker"));
    const qint64 ctrl = r.procs->pidOf(QStringLiteral("ctrltrack"));
    const qint64 monado = r.procs->pidOf(QStringLiteral("monado"));
    r.c->stop();
    CHECK(r.c->state() == RecState::Finished && r.c->message() == QStringLiteral("Stopped."));
    CHECK(r.procs->signalled.size() == 3);
    CHECK(r.procs->signalled[0].pid == monado && r.procs->signalled[0].sig == SIGTERM);
    CHECK(r.procs->signalled[1].pid == ctrl && r.procs->signalled[2].pid == broker);
    CHECK(!QFileInfo::exists(folder + QStringLiteral("/steps.json"))); // logs only
    CHECK(QFileInfo::exists(folder + QStringLiteral("/logs/session.log")));
    CHECK(!r.c->session()->result().has_value());
}

TEST(start_inside_out_without_controllers_and_with_lease_retry) {
    LiveRig r;
    r.setup.io.ctrlTrack = false;
    r.monadoLog = QStringLiteral("ERROR vkGetDrmDisplayEXT failed\n");
    CHECK(r.c->start());
    r.at(600);
    CHECK((r.names() == QStringList{QStringLiteral("broker"), QStringLiteral("monado")}));
    CHECK(has(r.procs->started[1].env, "PIMAX_CTRL_TRACK=0"));
    r.procs->exit(r.procs->pidOf(QStringLiteral("monado")), 1); // the display lease failed
    r.at(1000);
    CHECK(r.c->state() == RecState::RetryWait);
    r.at(1000 + kRecLeaseRetryMs);
    CHECK(r.names().count(QStringLiteral("monado")) == 2);
    r.monadoLog = QStringLiteral("all fine\n");
    r.at(1000 + kRecLeaseRetryMs + kIoSettleMs);
    CHECK(r.c->state() == RecState::Recording);
    // the broker dies: the session fails and stops the rest
    r.procs->exit(r.procs->pidOf(QStringLiteral("broker")), 1);
    r.at(20000);
    r.at(40000);
    CHECK(r.c->state() == RecState::Failed);
    CHECK(r.c->message().contains(QStringLiteral("no head or controller tracking")));
    CHECK(r.c->session()->runningPids().empty());
}

TEST(start_inside_out_ends_a_monado_whose_compositor_failed_and_retries) {
    // live 2026-10-19: KWin withdrew DP-6, Monado's compositor failed, its IPC server kept running without the
    // headset (which then reset every 5 s); the session must end that Monado and go through the lease retry
    LiveRig r;
    r.setup.io.ctrlTrack = false;
    r.monadoLog = QStringLiteral("ERROR [comp_main_create_system_compositor] Failed to init compositor 0x1\n");
    CHECK(r.c->start());
    r.at(600);
    const qint64 monado = r.procs->pidOf(QStringLiteral("monado"));
    r.at(700);
    CHECK(r.procs->signalled.size() == 1);
    CHECK(r.procs->signalled[0].pid == monado && r.procs->signalled[0].sig == SIGTERM);
    r.at(800); // the fake ends it at SIGTERM
    CHECK(r.c->state() == RecState::RetryWait);
    CHECK(r.c->state() == RecState::RetryWait);
    r.monadoLog = QStringLiteral("all fine\n");
    r.at(800 + kRecLeaseRetryMs);
    CHECK(r.names().count(QStringLiteral("monado")) == 2);
    r.at(800 + kRecLeaseRetryMs + kIoSettleMs);
    CHECK(r.c->state() == RecState::Recording);
    CHECK(r.procs->signalled.size() == 1);
}

// ---------------------------------------------------------------- the home scene and apps on Monado

TEST(home_scene_starts_once_monado_runs_and_is_stopped_first) {
    LiveRig r(true, true, true);
    CHECK(r.c->home().run == HomeRun::Waiting);
    CHECK(r.c->monadoReadiness().step == MonadoStep::Off);
    CHECK(r.c->start());
    CHECK(r.c->monadoReadiness().step == MonadoStep::Starting);
    r.at(600);
    CHECK(!r.names().contains(QStringLiteral("home"))); // Monado not up yet
    r.at(600 + kIoSettleMs);
    CHECK(r.c->state() == RecState::Recording);
    CHECK(r.names().back() == QStringLiteral("home"));
    const RecProcSpec& h = r.procs->started.back();
    CHECK(h.program == QStringLiteral("/x/pibridge-home"));
    CHECK(h.env == QStringList{QStringLiteral("XR_RUNTIME_JSON=/m/openxr_monado-dev.json")});
    CHECK(h.logPath == r.c->session()->logDir() + QStringLiteral("/home.log"));
    const qint64 home = r.procs->pidOf(QStringLiteral("home"));
    CHECK(r.c->home().run == HomeRun::Running && r.c->home().pid == home);
    // apps wait until the home has its session (or had its time): a game must be the newest session
    CHECK(r.c->monadoReadiness().step == MonadoStep::HomeStarting);
    r.homeStatus = QStringLiteral("starting");
    CHECK(!r.c->homeSettled());
    r.homeStatus = QStringLiteral("running");
    CHECK(r.c->monadoReadiness().ready());
    CHECK(r.c->monadoReadiness().runtimeJson == QStringLiteral("/m/openxr_monado-dev.json"));
    r.homeStatus.clear();
    r.at(600 + kIoSettleMs + kHomeSettleMs);
    CHECK(r.c->homeSettled()); // no status file: the time is up
    r.socket = false;
    CHECK(r.c->monadoReadiness().step == MonadoStep::NoSocket);
    r.socket = true;
    // Stop: the home first (that PID only), then Monado, the tracker and the broker
    const qint64 monado = r.procs->pidOf(QStringLiteral("monado"));
    r.c->stop();
    CHECK(r.procs->signalled.size() == 4);
    CHECK(r.procs->signalled[0].pid == home && r.procs->signalled[0].sig == SIGTERM);
    CHECK(r.procs->signalled[1].pid == monado);
    CHECK(r.c->home().run == HomeRun::Exited);
    CHECK(r.c->monadoReadiness().step == MonadoStep::Failed);
}

TEST(home_scene_is_never_started_twice_in_a_session) {
    LiveRig r(true, true, true);
    CHECK(r.c->start());
    r.at(600);
    r.at(600 + kIoSettleMs);
    const qint64 home = r.procs->pidOf(QStringLiteral("home"));
    CHECK(home > 0);
    r.procs->exit(home, 0, true); // it crashed: a new one could take the headset from a running game
    r.at(10000);
    r.at(20000);
    CHECK(r.names().count(QStringLiteral("home")) == 1);
    CHECK(r.c->home().run == HomeRun::Exited);
    CHECK(r.c->home().words.startsWith(QStringLiteral("Crashed")));
    CHECK(r.c->homeSettled() && r.c->monadoReadiness().ready());
    // a new session starts it again
    r.c->stop();
    CHECK(r.c->start());
    r.at(30000);
    r.at(30000 + kIoSettleMs);
    CHECK(r.names().count(QStringLiteral("home")) == 2);
}

TEST(home_scene_off_missing_or_ignoring_term) {
    {
        LiveRig r(true, true, true);
        r.setup.io.homeScene = false;
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        CHECK(!r.names().contains(QStringLiteral("home")));
        CHECK(r.c->home().run == HomeRun::Off && r.c->monadoReadiness().ready());
    }
    {
        LiveRig r(true, true, false); // pibridge-home not found
        r.c->refresh();
        CHECK(r.c->home().run == HomeRun::NotFound);
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        CHECK(!r.names().contains(QStringLiteral("home")) && r.c->monadoReadiness().ready());
    }
    {
        LiveRig r(true, true, true);
        r.runtimeJson.clear(); // a Monado build without openxr_monado-dev.json
        r.c->refresh();
        CHECK(r.c->home().run == HomeRun::NoRuntimeFile);
        CHECK(r.c->monadoReadiness().step == MonadoStep::NoRuntimeFile);
    }
    {
        LiveRig r(true, true, true);
        r.procs->ignoreTerm = {QStringLiteral("home")};
        r.procs->ignoreTerm.insert(QStringLiteral("monado")); // Monado takes its time too
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        const qint64 home = r.procs->pidOf(QStringLiteral("home"));
        r.c->stop();
        r.at(6000 + kRecOtherTermMs);
        bool killed = false;
        for (const auto& sg : r.procs->signalled) {
            killed = killed || (sg.pid == home && sg.sig == SIGKILL);
        }
        CHECK(killed);
        CHECK(!r.procs->running(home));
    }
}

// ---------------------------------------------------------------- the room map (pibridge-roommap auto)

TEST(room_map_auto_starts_with_a_saved_room_and_stops_by_pid) {
    LiveRig r(true, true);
    r.setup.roomSaved = true;
    r.c->refresh();
    CHECK(r.c->room().look == RoomLook::Saved);
    CHECK(r.c->start());
    r.at(600);
    CHECK(!r.names().contains(QStringLiteral("roommap"))); // Monado not up yet
    r.at(600 + kIoSettleMs);
    CHECK(r.c->state() == RecState::Recording);
    CHECK(r.names().back() == QStringLiteral("roommap"));
    const RecProcSpec& m = r.procs->started.back();
    CHECK(m.program == QStringLiteral("/fake/bin/pibridge-roommap"));
    CHECK((m.args == QStringList{QStringLiteral("auto"), QStringLiteral("--name"), QStringLiteral("home"),
                                 QStringLiteral("--wait"), QStringLiteral("120")}));
    CHECK(m.logPath == r.c->session()->logDir() + QStringLiteral("/roommap.log"));
    // the monado env has the anchor hook
    CHECK(has(r.procs->started[2].env, "PIMAX_ANCHOR=1"));
    const qint64 room = r.procs->pidOf(QStringLiteral("roommap"));
    CHECK(r.c->roomPid() == room);
    CHECK(r.c->room().look == RoomLook::Searching);
    // its status file: an old one (another session, earlier) does not count; this session's does
    r.roomStatus = "state = \"ok\"\nroom = \"home\"\npid = 77\ntext = \"old\"\nwhen = 5\n";
    CHECK(r.c->room().look == RoomLook::Searching);
    r.roomStatus = "state = \"searching\"\nroom = \"home\"\npid = 4242\ntext = \"Looking for the room\"\nwhen = 1000100\n";
    CHECK(r.c->room().look == RoomLook::Searching && r.c->room().detail == QStringLiteral("Looking for the room"));
    r.roomStatus = "state = \"ok\"\nroom = \"home\"\npid = 4242\ntext = \"Room in place\"\nwhen = 1000101\n";
    CHECK(r.c->room().look == RoomLook::Recognised && r.c->room().headline == QStringLiteral("Room recognised"));
    // Stop: the room map first (that PID only), then the home (none here) and Monado
    const qint64 monado = r.procs->pidOf(QStringLiteral("monado"));
    r.c->stop();
    CHECK(r.procs->signalled.size() >= 2);
    CHECK(r.procs->signalled[0].pid == room && r.procs->signalled[0].sig == SIGTERM);
    CHECK(r.procs->signalled[1].pid == monado);
    CHECK(r.c->roomPid() == 0);
}

TEST(room_map_auto_runs_once_and_not_without_a_room) {
    {
        LiveRig r(true, true);
        r.setup.roomSaved = true;
        r.c->refresh();
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        const qint64 room = r.procs->pidOf(QStringLiteral("roommap"));
        CHECK(room > 0);
        r.procs->exit(room, 0); // done: the room is in place (or the wait is over)
        r.at(10000);
        r.at(20000);
        CHECK(r.names().count(QStringLiteral("roommap")) == 1 && r.c->roomPid() == 0);
        r.c->stop();
        CHECK(r.procs->signalled.front().pid != room); // nothing sent to a PID that is gone
    }
    {
        LiveRig r(true, true); // no saved room: nothing changes
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        CHECK(r.c->state() == RecState::Recording && !r.names().contains(QStringLiteral("roommap")));
        CHECK(r.c->room().look == RoomLook::NoRoom);
    }
    {
        LiveRig r(true, true); // a saved room but no tool: not started, the anchor hook off
        r.setup.roomSaved = true;
        r.setup.roommap.clear();
        r.c->refresh();
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        CHECK(!r.names().contains(QStringLiteral("roommap")));
        CHECK(!hasPrefix(r.procs->started[2].env, "PIMAX_ANCHOR"));
    }
    {
        LiveRig r(true, true); // it cannot start: plain words
        r.setup.roomSaved = true;
        r.c->refresh();
        r.procs->failWith = QStringLiteral("No such file.");
        r.procs->failName = QStringLiteral("roommap");
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        CHECK(r.c->state() == RecState::Recording); // VR keeps running
        CHECK(r.c->room().headline == QStringLiteral("Room map did not start") && r.c->room().detail == QStringLiteral("No such file."));
    }
    {
        LiveRig r(true, true); // PiBridge closes: TERM, then KILL when it does not end
        r.setup.roomSaved = true;
        r.c->refresh();
        r.procs->ignoreTerm.insert(QStringLiteral("roommap"));
        CHECK(r.c->start());
        r.at(600);
        r.at(600 + kIoSettleMs);
        const qint64 room = r.procs->pidOf(QStringLiteral("roommap"));
        r.c->stop();
        r.at(600 + kIoSettleMs + kRecOtherTermMs + 10);
        bool killed = false;
        for (const auto& sg : r.procs->signalled) {
            killed = killed || (sg.pid == room && sg.sig == SIGKILL);
        }
        CHECK(killed);
    }
}

TEST(room_map_words_files_and_gates) {
    const auto st = parseRoomStatus("# omarchyvr-roommap auto / setup\nstate = \"failed\"\nroom = \"home\"\npid = 12\n"
                                    "text = \"Room home not recognised\"\nwhen = 99\n");
    CHECK(st && st->state == QStringLiteral("failed") && st->room == QStringLiteral("home") && st->pid == 12 &&
          st->text == QStringLiteral("Room home not recognised") && st->when == 99);
    CHECK(!parseRoomStatus("garbage"));
    // it counts for the session it names, or when written since the start
    CHECK(roomStatusWords(st, 12, 0, true, false, QStringLiteral("home")).look == RoomLook::NotFound);
    CHECK(roomStatusWords(st, 12, 0, true, false, QStringLiteral("home")).headline == QStringLiteral("Room not found"));
    CHECK(roomStatusWords(st, 0, 98, true, false, QStringLiteral("home")).look == RoomLook::NotFound);
    CHECK(roomStatusWords(st, 13, 100, true, false, QStringLiteral("home")).look == RoomLook::Saved);
    CHECK(roomStatusWords(st, 12, 0, true, false, QStringLiteral("office")).look == RoomLook::Saved); // another room
    CHECK(roomStatusWords(std::nullopt, 0, 0, false, false, QStringLiteral("home")).look == RoomLook::NoRoom);
    CHECK(roomStatusWords(std::nullopt, 0, 0, true, true, QStringLiteral("home")).look == RoomLook::Searching);
    // the name and the saved map
    Home h;
    const QString cp = h.dir.path() + QStringLiteral("/n.conf");
    CHECK(roomName(confWith(cp, {})) == QStringLiteral("home"));
    CHECK(roomName(confWith(h.dir.path() + QStringLiteral("/o.conf"), {{roomkey::kRoom, "office"}})) == QStringLiteral("office"));
    CHECK(roomName(confWith(h.dir.path() + QStringLiteral("/p.conf"), {{roomkey::kRoom, "../x"}})) == QStringLiteral("home"));
    const QString base = h.dir.path() + QStringLiteral("/roommap");
    CHECK(!roomSaved(base, QStringLiteral("home")));
    QDir().mkpath(base + QStringLiteral("/home"));
    QFile f(base + QStringLiteral("/home/map.bin"));
    CHECK(f.open(QIODevice::WriteOnly) && f.write("PXRM") == 4);
    f.close();
    CHECK(roomSaved(base, QStringLiteral("home")));
    qputenv("PIBRIDGE_ROOMMAP_DIR", base.toUtf8());
    CHECK(roommapBaseDir() == base);
    qunsetenv("PIBRIDGE_ROOMMAP_DIR");
    CHECK((roomSetupFastArgs(QStringLiteral("home")) ==
           QStringList{QStringLiteral("setup"), QStringLiteral("fast"), QStringLiteral("--name"), QStringLiteral("home"),
                       QStringLiteral("--no-hud")}));
    // SETUP PLAYSPACE with a body height: the eye height is passed so the floor comes from the head, not the points
    CHECK((roomSetupFastArgs(QStringLiteral("home"), 1.68).mid(5) ==
           QStringList{QStringLiteral("--eye-height"), QStringLiteral("1.680")}));
    // Map the room: inside-out, the tool, a live session with fresh head poses, not running already
    RingFeed live{4242, true, 0.1};
    CHECK(mapRoomGate(true, QStringLiteral("/t/pibridge-roommap"), live, false).ok);
    CHECK(mapRoomGate(false, QStringLiteral("/t/pibridge-roommap"), live, false).why.contains(QStringLiteral("inside-out")));
    CHECK(mapRoomGate(true, QString(), live, false).why.contains(QStringLiteral("not found")));
    CHECK(mapRoomGate(true, QStringLiteral("/t/pibridge-roommap"), live, true).why.contains(QStringLiteral("being mapped")));
    CHECK(mapRoomGate(true, QStringLiteral("/t/pibridge-roommap"), RingFeed{}, false).why.contains(QStringLiteral("START TRACKING")));
    CHECK(mapRoomGate(true, QStringLiteral("/t/pibridge-roommap"), RingFeed{4242, false, 0.1}, false).why.contains(QStringLiteral("No inside-out")));
    CHECK(mapRoomGate(true, QStringLiteral("/t/pibridge-roommap"), RingFeed{4242, true, 9.0}, false).why.contains(QStringLiteral("no head poses")));
    // ... in the headset: pibridge-roomsetup --map runs pibridge-roommap itself
    CHECK((roomMapHeadsetArgs(QStringLiteral("home"), QStringLiteral("/t/pibridge-roommap"), QStringLiteral("/run/r.head")) ==
           QStringList{QStringLiteral("--map"), QStringLiteral("--room"), QStringLiteral("home"), QStringLiteral("--roommap"),
                       QStringLiteral("/t/pibridge-roommap"), QStringLiteral("--ring"), QStringLiteral("/run/r.head")}));
    CHECK(mapRoomHeadsetGate(true, QStringLiteral("/t/pibridge-roommap"), QStringLiteral("/t/pibridge-roomsetup"), live, false).ok);
    CHECK(mapRoomHeadsetGate(true, QStringLiteral("/t/pibridge-roommap"), QString(), live, false).why.contains(QStringLiteral("pibridge-roomsetup")));
    CHECK(mapRoomHeadsetGate(true, QString(), QStringLiteral("/t/pibridge-roomsetup"), live, false).why.contains(QStringLiteral("pibridge-roommap")));
    CHECK(mapRoomHeadsetGate(false, QStringLiteral("/t/pibridge-roommap"), QString(), live, false).why.contains(QStringLiteral("inside-out")));
    // the floor from the user's height: measured eye height wins, else body x 0.936; refused outside the ranges
    CHECK(!eyeHeightOf(QString(), QString()));
    CHECK(std::fabs(*eyeHeightOf(QStringLiteral("1.78"), QString()) - 1.66608) < 1e-9);
    CHECK(*eyeHeightOf(QStringLiteral("1.78"), QStringLiteral("1,68 m")) == 1.68);
    CHECK(!eyeHeightOf(QStringLiteral("178"), QString()));            // centimetres: not taken
    CHECK(*eyeHeightOf(QStringLiteral("1.78"), QStringLiteral("x")) == *derivedEyeHeight(QStringLiteral("1.78")));
    CHECK((roomMapHeadsetArgs(QStringLiteral("home"), QStringLiteral("/t/rm"), QStringLiteral("/r"), 1.68).mid(7) ==
           QStringList{QStringLiteral("--eye-height"), QStringLiteral("1.680")}));
    CHECK((floorHeightArgs(QStringLiteral("home"), 1.68, QStringLiteral("/t/rm"), QStringLiteral("/r")) ==
           QStringList{QStringLiteral("--floor-height"), QStringLiteral("--room"), QStringLiteral("home"), QStringLiteral("--eye-height"),
                       QStringLiteral("1.680"), QStringLiteral("--roommap"), QStringLiteral("/t/rm"), QStringLiteral("--ring"), QStringLiteral("/r")}));
    const QString rm = QStringLiteral("/t/pibridge-roommap"), rs = QStringLiteral("/t/pibridge-roomsetup");
    CHECK(floorHeightGate(true, rm, rs, live, 1.68, true, false).ok);
    CHECK(floorHeightGate(true, rm, rs, live, std::nullopt, true, false).why.contains(QStringLiteral("body height")));
    CHECK(floorHeightGate(true, rm, rs, live, 1.68, false, false).why.contains(QStringLiteral("not recognised")));
    CHECK(floorHeightGate(true, rm, rs, RingFeed{}, 1.68, true, false).why.contains(QStringLiteral("START TRACKING")));
    CHECK(!floorHeightGate(true, rm, rs, live, 1.68, true, true).ok);
}

TEST(home_status_file_and_program_lookup) {
    CHECK(homeStateOf("{\"state\": \"running\", \"pid\": 42}", 42) == QStringLiteral("running"));
    CHECK(homeStateOf("{\"state\": \"running\", \"pid\": 41}", 42).isEmpty()); // an older run's file
    CHECK(homeStateOf("broken", 42).isEmpty());
    Home h;
    const QString bin = h.dir.path() + QStringLiteral("/bin");
    CHECK(findHomeProgram(bin, {}).isEmpty());
    CHECK(findHomeProgram(bin, [](const QString& n) { return QStringLiteral("/usr/bin/") + n; }) ==
          QStringLiteral("/usr/bin/pibridge-home"));
    // the build tree: <build>/app/pibridge next to <build>/tools/home/pibridge-home
    const QString tree = h.dir.path() + QStringLiteral("/tools/home/pibridge-home");
    writeAll(tree, "#!/bin/sh\n");
    QFile::setPermissions(tree, QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner);
    QDir().mkpath(h.dir.path() + QStringLiteral("/app"));
    CHECK(findHomeProgram(h.dir.path() + QStringLiteral("/app"), {}) == QDir::cleanPath(tree));
}

TEST(tracking_runtime_follows_the_sessions) {
    LiveRig direct(true, true, true);
    LiveRig tracking(true, false);
    // the old "apps through Monado" choice: still read, but the target is SteamVR until Monado directly runs
    Conf conf = confWith(direct.h.dir.path() + QStringLiteral("/pibridge.conf"), {{iokey::kMode, "inside-out"}, {iokey::kApps, "monado"}});
    CHECK(ioSettings(conf).apps == IoApps::Monado);
    TrackingRuntime rt(conf, *tracking.c, direct.c.get());
    CHECK(rt.target() == AppTarget::SteamVr);
    CHECK(rt.steamVrThroughTracking()); // inside-out: SteamVR comes with START TRACKING
    tracking.setup.io.apps = IoApps::Monado; // migrated: the Tracking page's start is SteamVR + Monado all the same
    tracking.c->refresh();
    CHECK(tracking.c->readiness().ready() && tracking.c->variant() == IoVariant::SteamVr);
    QString why;
    direct.vrserver = true; // Monado directly cannot start while SteamVR runs
    CHECK(direct.c->start() == false);
    direct.vrserver = false;
    CHECK(direct.c->start() && direct.c->active());
    CHECK(rt.target() == AppTarget::Monado);
    CHECK(rt.startTracking(&why)); // already running: nothing more
    CHECK(direct.names() == QStringList{QStringLiteral("broker")});
    direct.c->stop();
    CHECK(rt.target() == AppTarget::SteamVr);
    conf.set(iokey::kMode, "lighthouse");
    CHECK(!rt.steamVrThroughTracking());
}

TEST(recording_keeps_the_trackers_calibration) {
    Home h;
    RecSetup s = fakeRecSetup(h.dir.path() + QStringLiteral("/recordings"));
    s.slamConfig.path = writeSyntheticSlamBase(h.dir.path() + QStringLiteral("/base"));
    s.calib.path = h.dir.path() + QStringLiteral("/base/factory.xml");
    prepareSessionSlam(s);
    CHECK(!s.sessionSlamConfig.isEmpty());
    FakeRecProcesses procs;
    qint64 now = 0;
    bool written = false;
    RecProbe p;
    p.socketReady = [](const QString&) { return true; };
    p.imuBytes = [&](const QString& folder) -> qint64 {
        if (!written) {
            written = true;
            writeFakeRecording(folder, 2, false, QStringLiteral("free"), 1000000000);
            // what Monado's recorder writes: the driver's calibration (here: no accel bias)
            writeAll(folder + QStringLiteral("/pimax-basalt-calib.json"), "driver calibration");
        }
        return 5000;
    };
    p.readLog = [](const QString&) { return QString(); };
    p.monoNs = [&] { return now * 1000000; };
    RecSession sess(s, procs, p);
    CHECK(sess.start(RecSession::Mode::Free, {}, 0, QStringLiteral("20261019-120000")));
    sess.tick(now = 100);
    sess.tick(now = 200);
    CHECK(sess.state() == RecState::Recording);
    sess.stop(now = 3000);
    CHECK(sess.state() == RecState::Finished);
    const QString f = sess.folder();
    CHECK(readAll(f + QStringLiteral("/pimax-basalt-calib.json")) == readAll(slamDir() + QStringLiteral("/pimax-basalt-calib.json")));
    CHECK(readAll(f + QStringLiteral("/pimax-basalt-calib.json")).contains("0.34"));
    CHECK(readAll(f + QStringLiteral("/pimax-basalt-calib.driver.json")) == "driver calibration");
    CHECK(QFileInfo::exists(f + QStringLiteral("/pimax-basalt-config.json")));
}

PIBRIDGE_TEST_MAIN()
