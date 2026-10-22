// SPDX-License-Identifier: MIT
// Tests of "Record tracking" (core/rec_*.h): the setup lookup and the connector / camera / headset detection over fake
// /sys trees in a temporary dir, the readiness words, the step runner's timing and steps.json, the EuRoC check on
// synthetic datasets (the twins of omarchyvr-slam's euroc.rs tests plus more), the recordings list, and the session
// with FakeRecProcesses and an injected clock: start order and command lines, the display lease retry detected from a
// log sample, the PID-exact stop sequence in reverse order. Nothing is ever started or signalled.
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <csignal>

#include "core/conf.h"
#include "core/rec_check.h"
#include "core/rec_session.h"
#include "core/rec_setup.h"
#include "core/rec_steps.h"
#include "core/runner.h"
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

void makeExec(const QString& p) {
    writeAll(p, "#!/bin/sh\nexit 0\n");
    QFile::setPermissions(p, QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);
}

// A 128-byte EDID base block with a manufacturer id and a monitor name descriptor.
QByteArray edid(const char vendor[3], const QByteArray& name) {
    QByteArray e(128, '\0');
    const char hdr[] = {0, char(0xff), char(0xff), char(0xff), char(0xff), char(0xff), char(0xff), 0};
    for (int i = 0; i < 8; i++) {
        e[i] = hdr[i];
    }
    const unsigned v = (unsigned(vendor[0] - 'A' + 1) << 10) | (unsigned(vendor[1] - 'A' + 1) << 5) |
                       unsigned(vendor[2] - 'A' + 1);
    e[8] = char(v >> 8);
    e[9] = char(v & 0xff);
    // descriptor 2 (offset 72): monitor name
    e[72 + 3] = char(0xfc);
    QByteArray t = name.left(13);
    if (t.size() < 13) {
        t += '\n';
    }
    while (t.size() < 13) {
        t += ' ';
    }
    for (int i = 0; i < 13; i++) {
        e[72 + 5 + i] = t[i];
    }
    return e;
}

void connector(const QString& root, const QString& dir, const QByteArray& ed, const char* status) {
    QDir().mkpath(root + QLatin1Char('/') + dir);
    writeAll(root + QLatin1Char('/') + dir + QStringLiteral("/edid"), ed);
    writeAll(root + QLatin1Char('/') + dir + QStringLiteral("/status"), QByteArray(status) + '\n');
}

// A synthetic dataset like the Rust test's synth(): 2000 IMU samples at 1 kHz, two cameras of 40 frames at 25 Hz.
void synth(const QString& root, qint64 camShift) {
    std::vector<ImuRow> imu;
    for (int i = 0; i < 2000; i++) {
        imu.push_back({1000000000LL + i * 1000000LL, {0.001, 0, 0}, {0, 9.81, 0}});
    }
    std::vector<std::pair<qint64, QString>> cam, cam1;
    for (int i = 0; i < 40; i++) {
        const qint64 t = 1100000000LL + i * 40000000LL;
        cam.emplace_back(t, QStringLiteral("%1.png").arg(t));
        cam1.emplace_back(t + camShift, QStringLiteral("%1.png").arg(t));
    }
    writeEurocCsvs(root, imu, {cam, cam1});
}

void touchImages(const QString& root, int cams) {
    for (int c = 0; c < cams; c++) {
        QFile f(root + QStringLiteral("/mav0/cam%1/data.csv").arg(c));
        CHECK(f.open(QIODevice::ReadOnly));
        QString err;
        for (const auto& [t, name] : *parseCamCsv(f.readAll(), &err)) {
            writeAll(root + QStringLiteral("/mav0/cam%1/data/").arg(c) + name, "");
        }
    }
}

RecEnv envFor(const QTemporaryDir& d) {
    RecEnv e;
    e.home = d.path() + QStringLiteral("/home");
    // a prefix with nothing installed under it, so the <prefix>/lib|share/pibridge candidates fall through to the
    // dev-tree fallbacks these tests set up (deterministic regardless of a real ~/.local install)
    e.prefix = d.path() + QStringLiteral("/prefix");
    e.configDir = e.home + QStringLiteral("/.config/pibridge");
    e.dataHome = e.home + QStringLiteral("/.local/share");
    e.runtimeDir = d.path() + QStringLiteral("/run");
    e.drmRoot = d.path() + QStringLiteral("/sys/class/drm");
    e.v4lRoot = d.path() + QStringLiteral("/sys/class/video4linux");
    e.usbRoot = d.path() + QStringLiteral("/sys/bus/usb/devices");
    e.devRoot = d.path() + QStringLiteral("/dev");
    e.which = [](const QString&) { return QString(); };
    return e;
}

} // namespace

// ---------------------------------------------------------------- detection

TEST(edid_vendor_and_name) {
    const QByteArray e = edid("PVR", "Pimax");
    CHECK(edidVendor(e) == QStringLiteral("PVR"));
    CHECK(edidName(e) == QStringLiteral("Pimax"));
    CHECK(edidName(edid("DEL", "DELL U2720Q")) == QStringLiteral("DELL U2720Q"));
    CHECK(edidVendor(QByteArray(5, '\0')).isEmpty());
    CHECK(edidName(QByteArray(60, '\0')).isEmpty());
}

TEST(connector_detected_from_the_drm_tree) {
    QTemporaryDir d;
    const QString drm = d.path();
    CHECK(pimaxConnector(drm).isEmpty());
    connector(drm, QStringLiteral("card1-DP-4"), edid("DEL", "DELL U2720Q"), "connected");
    connector(drm, QStringLiteral("card1-DP-5"), edid("DEL", "DELL U2720Q"), "connected");
    QDir().mkpath(drm + QStringLiteral("/card1-HDMI-A-1")); // no edid file
    QDir().mkpath(drm + QStringLiteral("/card1"));
    QDir().mkpath(drm + QStringLiteral("/renderD128"));
    CHECK(pimaxConnector(drm).isEmpty());
    // a Pimax name in the descriptor: found; a connected one wins over a stale disconnected one
    connector(drm, QStringLiteral("card0-DP-1"), edid("PVR", "Pimax"), "disconnected");
    CHECK(pimaxConnector(drm) == QStringLiteral("DP-1"));
    connector(drm, QStringLiteral("card1-DP-6"), edid("ABC", "Pimax 7A2F"), "connected");
    CHECK(pimaxConnector(drm) == QStringLiteral("DP-6"));
    // only the vendor id says Pimax
    QTemporaryDir d2;
    connector(d2.path(), QStringLiteral("card2-DP-3"), edid("PVR", "HMD"), "connected");
    CHECK(pimaxConnector(d2.path()) == QStringLiteral("DP-3"));
}

TEST(camera_and_headset_detected) {
    QTemporaryDir d;
    const QString v4l = d.path() + QStringLiteral("/v4l");
    writeAll(v4l + QStringLiteral("/video2/name"), "HD Pro Webcam C920: HD Pro Webc\n");
    writeAll(v4l + QStringLiteral("/video2/index"), "0\n");
    CHECK(pimaxCameraDevice(v4l, QStringLiteral("/dev")).isEmpty());
    writeAll(v4l + QStringLiteral("/video11/name"), "PimaxSensor: PimaxSensor\n");
    writeAll(v4l + QStringLiteral("/video11/index"), "1\n"); // metadata node
    writeAll(v4l + QStringLiteral("/video10/name"), "PimaxSensor: PimaxSensor\n");
    writeAll(v4l + QStringLiteral("/video10/index"), "0\n");
    CHECK(pimaxCameraDevice(v4l, QStringLiteral("/dev")) == QStringLiteral("/dev/video10"));

    const QString usb = d.path() + QStringLiteral("/usb");
    writeAll(usb + QStringLiteral("/4-3.3.4/idVendor"), "34a4\n");
    writeAll(usb + QStringLiteral("/4-3.3.4/idProduct"), "0014\n"); // the camera alone is no headset
    writeAll(usb + QStringLiteral("/3-6/idVendor"), "046d\n");
    writeAll(usb + QStringLiteral("/3-6/idProduct"), "08e5\n");
    CHECK(!pimaxHeadsetPresent(usb));
    writeAll(usb + QStringLiteral("/3-3.3.1/idVendor"), "34a4\n");
    writeAll(usb + QStringLiteral("/3-3.3.1/idProduct"), "0040\n");
    CHECK(pimaxHeadsetPresent(usb));
}

TEST(setup_lookup_order) {
    QTemporaryDir d;
    const RecEnv env = envFor(d);
    Conf none(d.path() + QStringLiteral("/none.conf"));
    RecSetup s = resolveRecSetup(none, env);
    CHECK(s.monado.path.isEmpty() && s.cam.path.isEmpty() && s.calib.path.isEmpty());
    CHECK(s.recordingsDir == env.dataHome + QStringLiteral("/pibridge/recordings"));
    CHECK(s.brokerSocket == env.runtimeDir + QStringLiteral("/omarchyvr-cam-broker.sock"));
    CHECK(s.connector.isEmpty() && s.camDevice.isEmpty() && !s.headsetPresent);

    // PiBridge's tools from the install prefix, else the build tree (findTool); PiBridge's Monado build
    makeExec(env.prefix + QStringLiteral("/bin/pibridge-cam"));
    makeExec(d.path() + QStringLiteral("/build/tools/ctrltrack/pibridge-ctrltrack"));
    makeExec(env.home + QStringLiteral("/gamedev/pibridge-monado-build/src/xrt/targets/service/monado-service"));
    writeAll(env.home + QStringLiteral("/gamedev/ext/calib/crystal-calibration.xml"), "<x/>");
    writeAll(env.home + QStringLiteral("/gamedev/ovr-build/basalt/lib/libbasalt.so"), "");
    RecEnv envT = env;
    envT.findTool = [&d](const QString& n) {
        return n == QLatin1String("pibridge-ctrltrack") ? d.path() + QStringLiteral("/build/tools/ctrltrack/pibridge-ctrltrack")
                                                        : QString();
    };
    s = resolveRecSetup(none, envT);
    CHECK(s.cam.path == env.prefix + QStringLiteral("/bin/pibridge-cam"));
    CHECK(s.ctrl.path == d.path() + QStringLiteral("/build/tools/ctrltrack/pibridge-ctrltrack"));
    CHECK(resolveRecSetup(none, env).ctrl.path.isEmpty()); // no findTool: not found
    CHECK(s.monado.path.endsWith(QStringLiteral("/service/monado-service")));
    CHECK(s.calib.path.endsWith(QStringLiteral("/crystal-calibration.xml")));
    CHECK(s.basalt.path.endsWith(QStringLiteral("libbasalt.so")));
    CHECK(s.slamConfig.path.isEmpty() && !s.slamConfig.required);
    // the user's factory calibration in ~/.config/pibridge wins
    writeAll(env.configDir + QStringLiteral("/headset-calibration.xml"), "<x/>");
    s = resolveRecSetup(none, env);
    CHECK(s.calib.path == env.configDir + QStringLiteral("/headset-calibration.xml"));
    // PATH before the fallbacks
    RecEnv env2 = env;
    makeExec(d.path() + QStringLiteral("/bin/pibridge-cam"));
    env2.which = [&d](const QString& n) {
        return n == QLatin1String("pibridge-cam") ? d.path() + QStringLiteral("/bin/pibridge-cam") : QString();
    };
    CHECK(resolveRecSetup(none, env2).cam.path == d.path() + QStringLiteral("/bin/pibridge-cam"));

    // pibridge.conf keys: only that path counts; ~ is expanded; connector, camera, folder, env words
    const QString cp = d.path() + QStringLiteral("/pibridge.conf");
    writeAll(cp, "rec_pimax_cam=~/nowhere/pibridge-cam\nrec_connector=DP-9\nrec_dir=~/recs\n"
                 "rec_env=PIMAX_LOG=debug junk FOO=1\nrec_cam_device=/no/such/video\n");
    Conf conf(cp);
    s = resolveRecSetup(conf, env);
    CHECK(s.cam.path.isEmpty() && s.cam.fromConf && s.cam.wanted == env.home + QStringLiteral("/nowhere/pibridge-cam"));
    CHECK(s.connector == QStringLiteral("DP-9") && s.connectorFromConf);
    CHECK(s.recordingsDir == env.home + QStringLiteral("/recs"));
    CHECK((s.extraEnv == QStringList{QStringLiteral("PIMAX_LOG=debug"), QStringLiteral("FOO=1")}));
    CHECK(s.camDevice.isEmpty() && s.camDeviceFromConf);
}

TEST(system_check_from_proc) {
    QTemporaryDir d;
    writeAll(d.path() + QStringLiteral("/12/comm"), "monado-service\n");
    writeAll(d.path() + QStringLiteral("/13/comm"), "bash\n");
    RecSystem s = readRecSystem(d.path());
    CHECK(s.monado && !s.vrserver && !s.camBroker && !s.ctrltrack);
    writeAll(d.path() + QStringLiteral("/14/comm"), "vrserver\n");
    writeAll(d.path() + QStringLiteral("/15/comm"), "pibridge-ctrltr\n"); // comm: the first 15 bytes
    s = readRecSystem(d.path());
    CHECK(s.vrserver && s.ctrltrack);
}

TEST(readiness_words) {
    RecSetup s = fakeRecSetup(QStringLiteral("/tmp/x"));
    RecSystem sys;
    CHECK(recReadiness(s, sys).ready());
    sys.vrserver = true;
    RecReadiness r = recReadiness(s, sys);
    CHECK(!r.ready() && r.blocking.size() == 1 && r.blocking[0].contains(QStringLiteral("Close SteamVR first")));
    sys = {};
    sys.monado = true;
    CHECK(recReadiness(s, sys).blocking.join(QString()).contains(QStringLiteral("Another Monado")));
    sys = {};
    s.headsetPresent = false;
    s.camDevice.clear();
    s.connector.clear();
    s.cam.path.clear();
    s.basalt.path.clear();
    r = recReadiness(s, sys);
    const QString all = r.blocking.join(QLatin1Char('\n'));
    CHECK(all.contains(QStringLiteral("No headset found")));
    CHECK(all.contains(QStringLiteral("tracking cameras were not found")));
    CHECK(all.contains(QStringLiteral("rec_connector=")));
    CHECK(all.contains(QStringLiteral("pibridge-cam not found. Set rec_pimax_cam= in pibridge.conf.")));
    // basalt and the SLAM config (empty in the fake setup) are optional; the made-up monado-service has no
    // cap_sys_nice
    CHECK(r.warnings.size() == 3);
    CHECK(r.warnings.join(QString()).contains(QStringLiteral("Fix permissions")));
    CHECK(r.warnings.join(QString()).contains(QStringLiteral("the recording still works")));
    s = fakeRecSetup(QStringLiteral("/tmp/x"));
    s.monado.path.clear();
    s.monado.fromConf = true;
    s.monado.wanted = QStringLiteral("/opt/monado-service");
    CHECK(recReadiness(s, {}).blocking.join(QString()).contains(
        QStringLiteral("monado-service not found at /opt/monado-service (rec_monado_service= in pibridge.conf).")));
}

// ---------------------------------------------------------------- steps

TEST(default_protocol_is_on_the_head) {
    const auto steps = defaultRecSteps();
    CHECK(steps.size() == 15);
    CHECK(steps.front().kind == QStringLiteral("prepare"));
    CHECK(steps.front().text == QStringLiteral("Put the headset on. Get ready."));
    CHECK_NEAR(recStepsSeconds(steps), 120.0, 1e-9);
    int tilts = 0;
    for (const RecStep& s : steps) {
        CHECK(!s.text.contains(QStringLiteral("table")) && !s.text.contains(QStringLiteral("lenses")));
        CHECK(s.kind == QStringLiteral("prepare") || s.kind == QStringLiteral("still") ||
              s.kind == QStringLiteral("tilt-hold") || s.kind == QStringLiteral("move"));
        CHECK(s.settle < s.seconds);
        tilts += s.kind == QStringLiteral("tilt-hold");
    }
    CHECK(tilts == 4);
    CHECK(steps.back().text == QStringLiteral("Back on the mark. Hold still."));
}

TEST(protocol_from_json) {
    QString err;
    auto s = parseRecSteps(R"({"steps":[{"id":"a","kind":"still","text":"Hold still.","seconds":5,"settle":1},
                                         {"text":"Move.","seconds":2.5}]})",
                           &err);
    CHECK(s.size() == 2 && s[0].id == QStringLiteral("a") && s[0].settle == 1.0);
    CHECK(s[1].kind == QStringLiteral("move") && s[1].id == QStringLiteral("step-2") && s[1].seconds == 2.5);
    CHECK(parseRecSteps("{", &err).empty() && !err.isEmpty());
    CHECK(parseRecSteps(R"({"steps":[]})", &err).empty());
    CHECK(parseRecSteps(R"({"steps":[{"kind":"dance","text":"x","seconds":1}]})", &err).empty() &&
          err.contains(QStringLiteral("unknown kind")));
    CHECK(parseRecSteps(R"({"steps":[{"text":"x","seconds":0}]})", &err).empty());
    CHECK(parseRecSteps(R"({"steps":[{"text":"x","seconds":2,"settle":2}]})", &err).empty());
    CHECK(parseRecSteps(R"({"steps":[{"text":" ","seconds":2}]})", &err).empty() && err.contains(QStringLiteral("no text")));
}

TEST(step_runner_timing) {
    StepRunner r({{QStringLiteral("a"), QStringLiteral("still"), QStringLiteral("A"), 10, 2},
                  {QStringLiteral("b"), QStringLiteral("move"), QStringLiteral("B"), 5, 0},
                  {QStringLiteral("c"), QStringLiteral("still"), QStringLiteral("C"), 3, 1}});
    CHECK(!r.running() && r.current() == -1 && r.currentStep() == nullptr);
    r.start(1000);
    CHECK(r.running() && r.current() == 0 && r.currentStep()->id == QStringLiteral("a"));
    CHECK(r.nextStep()->id == QStringLiteral("b"));
    CHECK(r.secondsLeft(1000) == 10);
    CHECK(r.secondsLeft(1001) == 10); // rounded up
    CHECK(r.secondsLeft(10000) == 1);
    CHECK(!r.tick(10999) && r.current() == 0);
    CHECK(r.tick(11000) && r.current() == 1);
    CHECK(r.done().size() == 1 && r.done()[0].startMs == 1000 && r.done()[0].endMs == 11000);
    // a late tick: each step keeps its planned window
    CHECK(r.tick(17500) && r.current() == 2);
    CHECK(r.done()[1].startMs == 11000 && r.done()[1].endMs == 16000);
    CHECK(r.stepStartMs() == 16000 && r.secondsLeft(17500) == 2);
    CHECK(r.nextStep() == nullptr);
    r.tick(19000);
    CHECK(r.finished() && !r.running() && r.done().size() == 3 && r.done()[2].endMs == 19000);
    CHECK(!r.tick(30000));

    StepRunner s({{QStringLiteral("a"), QStringLiteral("still"), QStringLiteral("A"), 10, 0},
                  {QStringLiteral("b"), QStringLiteral("move"), QStringLiteral("B"), 5, 0},
                  {QStringLiteral("c"), QStringLiteral("move"), QStringLiteral("C"), 5, 0}});
    s.start(0);
    s.skip(4000);
    CHECK(s.current() == 1 && s.done()[0].skipped && s.done()[0].endMs == 4000);
    CHECK(s.secondsLeft(4000) == 5); // the next step starts at the skip
    s.stop(6500);
    CHECK(s.finished() && s.done().size() == 2 && s.done()[1].cutShort && s.done()[1].endMs == 6500);

    StepRunner e({});
    e.start(0);
    CHECK(e.finished());
}

TEST(steps_json_says_which_window_is_which) {
    StepRunner r(defaultRecSteps());
    r.start(500);
    r.tick(500 + 18000); // put-on (8 s) and still-mark (10 s) done
    r.stop(500 + 20000); // stopped 2 s into tilt-left
    StepsRecord rec;
    rec.mode = QStringLiteral("guided");
    rec.monoAtMs0Ns = 7000000000000LL; // 7000 s of uptime: a value a double would still hold, the type is integer
    rec.recordStartMs = 500;
    rec.recordEndMs = 20500;
    rec.imuFirstNs = rec.monoAtMs0Ns + 480 * 1000000LL;
    rec.imuLastNs = rec.monoAtMs0Ns + 20490 * 1000000LL;
    rec.stoppedEarly = true;
    rec.endReason = QStringLiteral("stopped by the user");
    const QJsonObject o = stepsJson(rec, &r);
    CHECK(o.value(QStringLiteral("format")).toString() == QStringLiteral("pibridge-record-steps 1"));
    CHECK(o.value(QStringLiteral("record_start_ns")).toInteger() == rec.monoAtMs0Ns + 500 * 1000000LL);
    CHECK(o.value(QStringLiteral("clock_matches_dataset")).toBool());
    CHECK(o.value(QStringLiteral("offset_ns")).toInteger() == 0);
    const QJsonArray steps = o.value(QStringLiteral("steps")).toArray();
    CHECK(steps.size() == 3);
    const QJsonObject s1 = steps[1].toObject();
    CHECK(s1.value(QStringLiteral("id")).toString() == QStringLiteral("still-mark"));
    CHECK(s1.value(QStringLiteral("kind")).toString() == QStringLiteral("still"));
    CHECK(s1.value(QStringLiteral("start_ns")).toInteger() == rec.monoAtMs0Ns + 8500 * 1000000LL);
    CHECK(s1.value(QStringLiteral("end_ns")).toInteger() == rec.monoAtMs0Ns + 18500 * 1000000LL);
    CHECK(s1.value(QStringLiteral("still_from_ns")).toInteger() == rec.monoAtMs0Ns + 10500 * 1000000LL);
    CHECK_NEAR(s1.value(QStringLiteral("start_s")).toDouble(), 8.02, 1e-9);
    CHECK(!steps[0].toObject().contains(QStringLiteral("still_from_ns"))); // prepare: no still window
    CHECK(steps[2].toObject().value(QStringLiteral("cut_short")).toBool());
    CHECK(steps[2].toObject().value(QStringLiteral("kind")).toString() == QStringLiteral("tilt-hold"));
    CHECK(o.value(QStringLiteral("not_reached")).toArray().size() == 12);
    // a dataset on another clock: offset_ns maps the steps onto it
    rec.imuFirstNs = 5000;
    rec.imuLastNs = 20005000;
    const QJsonObject o2 = stepsJson(rec, &r);
    CHECK(!o2.value(QStringLiteral("clock_matches_dataset")).toBool());
    CHECK(o2.value(QStringLiteral("offset_ns")).toInteger() == 5000 - (rec.monoAtMs0Ns + 500 * 1000000LL));
    // written and read back
    QTemporaryDir d;
    CHECK(writeStepsJson(d.path() + QStringLiteral("/r/steps.json"), rec, &r));
    QFile f(d.path() + QStringLiteral("/r/steps.json"));
    CHECK(f.open(QIODevice::ReadOnly) && QJsonDocument::fromJson(f.readAll()).object().value(QStringLiteral("mode")).toString() ==
                                             QStringLiteral("guided"));
}

// ---------------------------------------------------------------- the check (twins of euroc.rs)

TEST(euroc_good_dataset_passes) {
    QTemporaryDir d;
    const QString root = d.path() + QStringLiteral("/euroc_20261014120000");
    synth(root, 0);
    QString err;
    const auto ds = loadEuroc(d.path(), &err);
    CHECK(ds.has_value());
    CHECK(ds->root == root);
    CHECK(ds->imu.size() == 2000 && ds->cams.size() == 2);
    const EurocReport r = checkEuroc(*ds, false);
    CHECK(r.ok());
    CHECK(r.text.contains(QStringLiteral("25.00 Hz")));
    CHECK(r.text.contains(QStringLiteral("1000.0 Hz")));
    CHECK(r.text.contains(QStringLiteral("OK: usable for offline SLAM replay")));
    CHECK(r.text.contains(QStringLiteral("cam1: 40 of 40 timestamps shared with cam0")));
    CHECK_NEAR(r.medianAccel, 9.81, 1e-9);
    CHECK_NEAR(r.durationS, 1.999, 1e-9);
    CHECK(r.imuFirstNs == 1000000000LL && r.imuLastNs == 2999000000LL);
}

TEST(euroc_unsynced_and_missing_files_flagged) {
    QTemporaryDir d;
    synth(d.path(), 3000000);
    QString err;
    const auto ds = loadEuroc(d.path(), &err);
    CHECK(ds.has_value());
    const EurocReport r = checkEuroc(*ds, true);
    CHECK(r.problems.join(QLatin1Char('\n')).contains(QStringLiteral("match cam0")));
    CHECK(r.problems.join(QLatin1Char('\n')).contains(QStringLiteral("missing")));
    touchImages(d.path(), 1);
    const EurocReport r2 = checkEuroc(*loadEuroc(d.path(), &err), true);
    CHECK(r2.problems.join(QLatin1Char('\n')).contains(QStringLiteral("cam1: 40 image files missing")));
    CHECK(!r2.problems.join(QLatin1Char('\n')).contains(QStringLiteral("cam0: 40 image files missing")));
}

TEST(euroc_gaps_order_scale_overlap) {
    QTemporaryDir d;
    std::vector<ImuRow> imu;
    qint64 t = 1000000000;
    for (int i = 0; i < 1000; i++) {
        t += i == 500 ? 35000000 : 1000000;
        imu.push_back({t, {0, 0, 0}, {0, 4.9, 0}}); // half of gravity: scale wrong
    }
    imu[700].ts = imu[699].ts; // one repeated timestamp
    std::vector<std::pair<qint64, QString>> cam;
    for (int i = 0; i < 30; i++) {
        cam.emplace_back(500000000LL + i * 40000000LL, QStringLiteral("x.png")); // starts before the IMU
    }
    writeEurocCsvs(d.path(), imu, {cam});
    QString err;
    const EurocReport r = checkEuroc(*loadEuroc(d.path(), &err), false);
    const QString p = r.problems.join(QLatin1Char('\n'));
    CHECK(p.contains(QStringLiteral("imu0: 1 non-increasing timestamps")));
    CHECK(p.contains(QStringLiteral("imu0: a 35.0 ms gap (USB stall?)")));
    CHECK(p.contains(QStringLiteral("imu0: median |accel| 4.90 m/s², scale wrong?")));
    CHECK(p.contains(QStringLiteral("frames outside the IMU time span")));
    // fewer than 2 samples / no cameras
    QTemporaryDir e;
    writeEurocCsvs(e.path(), {{1, {}, {}}}, {});
    const EurocReport r2 = checkEuroc(*loadEuroc(e.path(), &err), false);
    CHECK(r2.problems.contains(QStringLiteral("fewer than 2 IMU samples")));
    CHECK(r2.problems.contains(QStringLiteral("no camN/data.csv")));
}

TEST(euroc_parse_errors_and_root) {
    QString err;
    CHECK(!parseImuCsv("#h\n1,2,3\n", &err) && err == QStringLiteral("imu line 2: 3 fields, want 7"));
    CHECK(!parseImuCsv("x,1,2,3,4,5,6\n", &err) && err.contains(QStringLiteral("bad timestamp")));
    CHECK(!parseImuCsv("1,1,2,3,4,5,z\n", &err) && err.contains(QStringLiteral("bad number")));
    const auto ok = parseImuCsv("#h\r\n\r\n5,0.5,1,2,3,4,9.8\r\n", &err);
    CHECK(ok && ok->size() == 1 && (*ok)[0].ts == 5 && (*ok)[0].accel[2] == 9.8); // a point in a German locale too
    CHECK(!parseCamCsv("12\n", &err) && err.contains(QStringLiteral("want ts,filename")));
    QTemporaryDir d;
    CHECK(!findEurocRoot(d.path(), &err) && err.contains(QStringLiteral("no mav0 directory")));
    QDir().mkpath(d.path() + QStringLiteral("/euroc_20261019101010/mav0"));
    QDir().mkpath(d.path() + QStringLiteral("/euroc_20261019121212/mav0"));
    QDir().mkpath(d.path() + QStringLiteral("/logs"));
    CHECK(findEurocRoot(d.path(), &err) == d.path() + QStringLiteral("/euroc_20261019121212"));
    CHECK(!findEurocRoot(d.path() + QStringLiteral("/nope"), &err));
}

TEST(recordings_list_check_and_delete) {
    QTemporaryDir d;
    const QString dir = d.path() + QStringLiteral("/recordings");
    CHECK(writeFakeRecording(dir + QStringLiteral("/20261019-141205"), 3, false, QStringLiteral("guided"), 1000000000));
    CHECK(writeFakeRecording(dir + QStringLiteral("/20261019-113040"), 2, true, QStringLiteral("free"), 1000000000));
    QDir().mkpath(dir + QStringLiteral("/not-a-recording"));
    auto list = listRecordings(dir);
    CHECK(list.size() == 2);
    CHECK(list[0].name == QStringLiteral("20261019-141205")); // newest first
    CHECK(list[0].ok && list[0].summary == QStringLiteral("OK: usable") && list[0].mode == QStringLiteral("guided"));
    CHECK_NEAR(list[0].durationS, 3.0, 0.01);
    CHECK(list[0].bytes > 0);
    CHECK(!list[1].ok && list[1].summary.contains(QStringLiteral("gap")) && list[1].mode == QStringLiteral("free"));
    CHECK(QFileInfo::exists(dir + QStringLiteral("/20261019-141205/check.json")));
    // check.json is reused
    QFile cj(dir + QStringLiteral("/20261019-141205/check.json"));
    CHECK(cj.open(QIODevice::ReadOnly));
    QJsonObject c = QJsonDocument::fromJson(cj.readAll()).object();
    cj.close();
    c.insert(QStringLiteral("summary"), QStringLiteral("cached"));
    writeAll(cj.fileName(), QJsonDocument(c).toJson());
    CHECK(listRecordings(dir)[0].summary == QStringLiteral("cached"));
    // a recording without a dataset
    QDir().mkpath(dir + QStringLiteral("/20261020-090000/logs"));
    writeAll(dir + QStringLiteral("/20261020-090000/steps.json"), "{}");
    list = listRecordings(dir);
    CHECK(list.size() == 3 && !list[0].ok && list[0].summary.startsWith(QStringLiteral("No usable dataset")));
    // delete: only a recording folder directly inside the recordings folder
    QString err;
    CHECK(!deleteRecording(dir, dir + QStringLiteral("/not-a-recording"), &err));
    CHECK(QFileInfo::exists(dir + QStringLiteral("/not-a-recording")));
    CHECK(!deleteRecording(dir, d.path(), &err));
    CHECK(!deleteRecording(dir, dir + QStringLiteral("/20261019-141205/logs"), &err));
    QFile::link(dir + QStringLiteral("/20261019-113040"), dir + QStringLiteral("/link"));
    CHECK(!deleteRecording(dir, dir + QStringLiteral("/link"), &err));
    CHECK(deleteRecording(dir, dir + QStringLiteral("/20261019-113040"), &err));
    CHECK(!QFileInfo::exists(dir + QStringLiteral("/20261019-113040")));
    CHECK(sizeWords(1400000000) == QStringLiteral("1.4 GB") && sizeWords(820000000) == QStringLiteral("820 MB") &&
          sizeWords(12000) == QStringLiteral("12 kB"));
    CHECK(durationWords(125) == QStringLiteral("2 min 05 s") && durationWords(48.4) == QStringLiteral("48 s"));
}

// ---------------------------------------------------------------- the session

namespace {

const char* const kLeaseLog =
    "INFO [comp_window_direct_wayland] Connector id 98 DP-6 (Pimax)\n"
    "INFO [_lease_connector_withdrawn] Connector DP-6 has been withdrawn by the compositor\n"
    "ERROR [comp_window_direct_wayland] vkGetDrmDisplayEXT failed: VK_ERROR_UNKNOWN\n"
    "ERROR [ipc_server_main] xrt_instance_create_system failed!\n";

struct Rig {
    QTemporaryDir dir;
    RecSetup setup;
    FakeRecProcesses procs;
    qint64 socketAtMs = 500;  // the broker's socket shows up then
    qint64 dataAtMs = -1;     // IMU data from then on (-1 never)
    qint64 now = 0;
    QString monadoLog;
    std::vector<QString> spoken;
    std::unique_ptr<RecSession> s;
    bool written = false;

    explicit Rig(const std::function<void(RecSetup&)>& tweak = {}) {
        setup = fakeRecSetup(dir.path() + QStringLiteral("/recordings"));
        if (tweak) {
            tweak(setup);
        }
        RecProbe p;
        p.socketReady = [this](const QString&) { return now >= socketAtMs; };
        p.imuBytes = [this](const QString& folder) -> qint64 {
            if (dataAtMs < 0 || now < dataAtMs) {
                return -1;
            }
            if (!written) {
                written = true;
                writeFakeRecording(folder, 3, false, QStringLiteral("guided"), 2000000000);
            }
            return 5000;
        };
        p.readLog = [this](const QString& path) { return path.contains(QStringLiteral("monado")) ? monadoLog : QString(); };
        p.monoNs = [this] { return 2000000000LL + now * 1000000; };
        s = std::make_unique<RecSession>(setup, procs, p, [this](const QString& t) { spoken.push_back(t); });
    }
    void at(qint64 ms) {
        now = ms;
        s->tick(ms);
    }
    QStringList names() const {
        QStringList n;
        for (const auto& sp : procs.started) {
            n << sp.name;
        }
        return n;
    }
};

std::vector<RecStep> twoSteps() {
    return {{QStringLiteral("a"), QStringLiteral("still"), QStringLiteral("Hold still."), 2, 0.5},
            {QStringLiteral("b"), QStringLiteral("move"), QStringLiteral("Nod slowly."), 1, 0}};
}

} // namespace

TEST(lease_failure_from_a_log_sample) {
    CHECK(leaseFailure(QString::fromLatin1(kLeaseLog)));
    CHECK(leaseFailure(QStringLiteral("ERROR vkGetDrmDisplayEXT failed")));
    CHECK(!leaseFailure(QStringLiteral("INFO all good\nERROR pimax: no headset\n")));
    const QString why = logReason(QString::fromLatin1(kLeaseLog));
    CHECK(why.contains(QStringLiteral("vkGetDrmDisplayEXT failed")) && why.contains(QStringLiteral("xrt_instance_create_system")));
    CHECK(logReason(QStringLiteral("one\ntwo\nthree\n")) == QStringLiteral("two / three"));
}

TEST(command_lines_and_environment) {
    const RecSetup s = fakeRecSetup(QStringLiteral("/r"));
    const RecProcSpec b = brokerSpec(s, QStringLiteral("/r/x/logs"));
    CHECK(b.program == s.cam.path);
    CHECK((b.args == QStringList{QStringLiteral("broker"), QStringLiteral("--source"), QStringLiteral("/dev/video0"),
                                 QStringLiteral("--socket"), s.brokerSocket}));
    CHECK(b.logPath == QStringLiteral("/r/x/logs/broker.log"));
    const RecProcSpec c = ctrltrackSpec(s, QStringLiteral("/r/x/logs"));
    CHECK((c.args == QStringList{QStringLiteral("live"), QStringLiteral("--source"), QStringLiteral("broker:") + s.brokerSocket,
                                 QStringLiteral("--factory")}));
    const QStringList e = monadoEnv(s, QStringLiteral("/r/x"));
    // the recorder uses the inside-out settings (core/inside_out.h): camera auto exposure on with the Crystal model
    for (const char* want : {"PIMAX_FPS=90", "PIMAX_CAM_SOURCE=broker", "PIMAX_SLAM=1", "PIMAX_CAM_AE=1",
                             "PIMAX_CAM_AE_MODEL=crystal-super", "PIMAX_CTRL_ANGLE=30",
                             "PIMAX_CTRL_TRACK=1", "XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1",
                             "XRT_COMPOSITOR_WAYLAND_CONNECTOR=DP-6", "XRT_COMPOSITOR_FORCE_NVIDIA=0", "XRT_NO_STDIN=1",
                             "IPC_IGNORE_VERSION=1", "PIMAX_SLAM_RECORD=/r/x"}) {
        CHECK(e.contains(QString::fromLatin1(want)));
    }
    CHECK(e.contains(QStringLiteral("PIMAX_CAM_CALIB=") + s.calib.path));
    CHECK(e.contains(QStringLiteral("VIT_SYSTEM_LIBRARY_PATH=") + s.basalt.path));
    CHECK(!e.join(QString()).contains(QStringLiteral("SLAM_CONFIG="))); // none in the fake setup
    RecSetup s2 = s;
    s2.basalt.path.clear();
    s2.extraEnv = {QStringLiteral("PIMAX_LOG=debug")};
    const QStringList e2 = monadoEnv(s2, QStringLiteral("/r/x"));
    CHECK(e2.contains(QStringLiteral("PIMAX_SLAM=0")) && !e2.join(QString()).contains(QStringLiteral("VIT_SYSTEM")));
    CHECK(e2.last() == QStringLiteral("PIMAX_LOG=debug")); // rec_env last: it overrides
}

TEST(pimax_slam_engine_environment) {
    RecSetup s = fakeRecSetup(QStringLiteral("/r"));
    s.io.slamEngine = SlamEngine::Pimax;
    s.io.slamCams = 2; // Basalt's camera setting does not matter for Pimax's engine
    CHECK(pimaxSlamReady(s) && winbridgeWanted(s));
    const IoSessionEnv e = buildIoEnv(s.io, s, 90, QStringLiteral("/r/pimax-basalt.toml"), QString());
    CHECK(e.winbridge);
    CHECK(e.driver.contains(QStringLiteral("PIMAX_SLAM=1")));
    CHECK(e.driver.contains(QStringLiteral("VIT_SYSTEM_LIBRARY_PATH=") + s.vitBridge.path));
    // all four tracking cameras, as Pimax Play feeds the engine; none of Basalt's files
    CHECK(e.driver.contains(QStringLiteral("PIMAX_SLAM_CAMS=0,1,2,3")));
    CHECK(e.driver.contains(QStringLiteral("PIMAX_SLAM_EXPOSURE_MID=0"))); // raw stamps: the engine has its own delta
    CHECK(e.driver.contains(QStringLiteral("PIMAX_SLAM_PASSTHROUGH=1")));  // the engine's pose as it comes
    CHECK(e.driver.contains(QStringLiteral("PIBRIDGE_VIT_POSE_RATE=imu")));
    CHECK(!e.driver.join(QString()).contains(QStringLiteral("SLAM_CONFIG=")));
    CHECK(!e.driver.join(QString()).contains(s.basalt.path));
    // the driver's output stages stay for either engine
    CHECK(e.driver.contains(QStringLiteral("PIMAX_SLAM_DRIFT_HOLD=0")));
    // the controllers from Pimax's LED engine in the same winbridge, not from pibridge-ctrltrack
    CHECK(pimaxCtrlEngine(s));
    CHECK(e.driver.contains(QStringLiteral("PIMAX_CTRL_ENGINE=1")));
    CHECK(e.driver.contains(QStringLiteral("PIMAX_CTRL_TRACK=0")) && !e.ctrltrack);
    {   // no LED models in the kit: pibridge-ctrltrack as before
        RecSetup sm = s;
        sm.winCtrlModels = false;
        const IoSessionEnv em = buildIoEnv(sm.io, sm, 90, QString(), QString());
        CHECK(!pimaxCtrlEngine(sm) && em.ctrltrack && em.driver.contains(QStringLiteral("PIMAX_CTRL_TRACK=1")));
        CHECK(!em.driver.join(QString()).contains(QStringLiteral("PIMAX_CTRL_ENGINE")));
    }
    {   // optical controller tracking off: neither
        RecSetup so = s;
        so.io.ctrlTrack = false;
        const IoSessionEnv eo = buildIoEnv(so.io, so, 90, QString(), QString());
        CHECK(!pimaxCtrlEngine(so) && !eo.ctrltrack && !eo.driver.join(QString()).contains(QStringLiteral("PIMAX_CTRL_ENGINE")));
    }
    // the server: wine winbridge.exe with the kit as a Z: path
    const RecProcSpec w = winbridgeSpec(s, QStringLiteral("/r/x/logs"));
    CHECK(w.name == QStringLiteral("winbridge") && w.program == s.wine);
    CHECK((w.args == QStringList{s.winbridge.path}));
    CHECK(w.env.contains(QStringLiteral("PIBRIDGE_WINDATA=Z:\\fake\\share\\pibridge\\windata")));
    CHECK(w.env.contains(QStringLiteral("WINEDEBUG=-all")));
    CHECK(w.logPath == QStringLiteral("/r/x/logs/winbridge.log"));
    // a missing part: SLAM off, the reason in plain words, no server
    RecSetup s2 = s;
    s2.winKit.clear();
    QString why;
    CHECK(!pimaxSlamReady(s2, &why) && why.contains(QStringLiteral("Pimax Play")));
    const IoSessionEnv e2 = buildIoEnv(s2.io, s2, 90, QString(), QString());
    CHECK(!e2.winbridge && e2.driver.contains(QStringLiteral("PIMAX_SLAM=0")));
    CHECK(!e2.driver.join(QString()).contains(QStringLiteral("VIT_SYSTEM")));
    CHECK(e2.notes.join(QString()).contains(QStringLiteral("Pimax engine is not complete")));
    // SLAM off: no server either
    RecSetup s3 = s;
    s3.io.slam = false;
    CHECK(!winbridgeWanted(s3));
    // the setting is saved and read back
    QTemporaryDir tmp;
    Conf c(tmp.path() + QStringLiteral("/pibridge.conf"));
    for (const auto& [k, v] : ioConfLines(s.io)) {
        CHECK(c.set(k, v));
    }
    CHECK(ioSettings(c).slamEngine == SlamEngine::Pimax);
    CHECK(ioSettings(Conf(tmp.path() + QStringLiteral("/none.conf"))).slamEngine == SlamEngine::Basalt); // default
}

TEST(session_pimax_engine_runs_winbridge_before_monado_and_stops_it_after) {
    Rig r([](RecSetup& s) { s.io.slamEngine = SlamEngine::Pimax; });
    r.dataAtMs = 3000;
    CHECK(r.s->start(RecSession::Mode::Guided, twoSteps(), 0, QStringLiteral("20261020-120000")));
    r.at(600);
    // no pibridge-ctrltrack: the controllers come from Pimax's LED engine inside winbridge
    CHECK((r.names() == QStringList{QStringLiteral("broker"), QStringLiteral("winbridge"), QStringLiteral("monado")}));
    CHECK(r.procs.started[1].program == r.setup.wine);
    CHECK(r.procs.started[2].env.contains(QStringLiteral("VIT_SYSTEM_LIBRARY_PATH=") + r.setup.vitBridge.path));
    CHECK(r.procs.started[2].env.contains(QStringLiteral("PIMAX_CTRL_ENGINE=1")));
    const qint64 broker = r.procs.pidOf(QStringLiteral("broker"));
    const qint64 wb = r.procs.pidOf(QStringLiteral("winbridge"));
    const qint64 monado = r.procs.pidOf(QStringLiteral("monado"));
    r.at(3000);
    CHECK(r.s->state() == RecState::Recording);
    r.at(6000); // the steps end: Monado first, the engine after it, the broker last
    CHECK(r.s->state() == RecState::Finished);
    CHECK(r.procs.signalled.size() == 3);
    CHECK(r.procs.signalled[0].pid == monado && r.procs.signalled[1].pid == wb && r.procs.signalled[2].pid == broker);
    CHECK(r.s->runningPids().empty());

    // the kit without the controllers' LED models: pibridge-ctrltrack as before
    Rig rm([](RecSetup& s) { s.io.slamEngine = SlamEngine::Pimax; s.winCtrlModels = false; });
    rm.s->start(RecSession::Mode::Guided, twoSteps(), 0);
    rm.at(600);
    CHECK((rm.names() == QStringList{QStringLiteral("broker"), QStringLiteral("winbridge"), QStringLiteral("ctrltrack"),
                                     QStringLiteral("monado")}));

    // the engine ends during the session: Monado goes on (3DoF), the message says so
    Rig r2([](RecSetup& s) { s.io.slamEngine = SlamEngine::Pimax; });
    r2.dataAtMs = 3000;
    r2.s->start(RecSession::Mode::Guided, twoSteps(), 0);
    r2.at(600);
    r2.at(3000);
    r2.procs.exit(r2.procs.pidOf(QStringLiteral("winbridge")), 1);
    r2.at(3100);
    CHECK(r2.s->state() == RecState::Recording);
    CHECK(r2.s->message().contains(QStringLiteral("Pimax head tracking stopped")));

    // the Basalt engine: no winbridge at all
    Rig r3;
    r3.s->start(RecSession::Mode::Guided, twoSteps(), 0);
    r3.at(600);
    CHECK(!r3.names().contains(QStringLiteral("winbridge")));
}

TEST(session_guided_run_start_order_steps_and_exact_stop) {
    Rig r;
    r.dataAtMs = 3000;
    CHECK(r.s->start(RecSession::Mode::Guided, twoSteps(), 0, QStringLiteral("20261019-120000")));
    CHECK(r.s->state() == RecState::Starting);
    CHECK(r.s->folder() == r.setup.recordingsDir + QStringLiteral("/20261019-120000"));
    CHECK(r.names() == QStringList{QStringLiteral("broker")}); // nothing else before the socket
    r.at(400);
    CHECK(r.names() == QStringList{QStringLiteral("broker")});
    r.at(600);
    CHECK((r.names() == QStringList{QStringLiteral("broker"), QStringLiteral("ctrltrack"), QStringLiteral("monado")}));
    CHECK(r.procs.started[2].env.contains(QStringLiteral("PIMAX_SLAM_RECORD=") + r.s->folder()));
    CHECK(r.procs.started[2].logPath == r.s->logDir() + QStringLiteral("/monado.log"));
    CHECK(r.s->state() == RecState::Starting && r.s->steps() && !r.s->steps()->running());
    r.at(3000);
    CHECK(r.s->state() == RecState::Recording && r.s->steps()->running());
    CHECK(r.spoken.back() == QStringLiteral("Hold still."));
    r.at(5000);
    CHECK(r.spoken.back() == QStringLiteral("Nod slowly.") && r.s->steps()->current() == 1);
    const qint64 broker = r.procs.pidOf(QStringLiteral("broker"));
    const qint64 ctrl = r.procs.pidOf(QStringLiteral("ctrltrack"));
    const qint64 monado = r.procs.pidOf(QStringLiteral("monado"));
    r.at(6000); // the steps end: the stop sequence, Monado first
    CHECK(r.s->state() == RecState::Finished);
    CHECK(r.procs.signalled.size() == 3);
    CHECK(r.procs.signalled[0].pid == monado && r.procs.signalled[0].sig == SIGTERM);
    CHECK(r.procs.signalled[1].pid == ctrl && r.procs.signalled[2].pid == broker);
    CHECK(r.s->runningPids().empty());
    CHECK(r.s->result() && r.s->result()->ok);
    CHECK(r.s->message().startsWith(QStringLiteral("Done. OK: usable")));
    CHECK(r.spoken.back() == QStringLiteral("Done. Recording stopped."));
    // steps.json: both steps, the dataset's IMU span, the end reason
    QFile f(r.s->folder() + QStringLiteral("/steps.json"));
    CHECK(f.open(QIODevice::ReadOnly));
    const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
    CHECK(o.value(QStringLiteral("steps")).toArray().size() == 2);
    CHECK(o.value(QStringLiteral("end_reason")).toString() == QStringLiteral("finished"));
    CHECK(!o.value(QStringLiteral("stopped_early")).toBool());
    CHECK(o.value(QStringLiteral("imu_first_ns")).toInteger() > 0);
    CHECK(o.value(QStringLiteral("dataset")).toString().startsWith(QStringLiteral("euroc_")));
    CHECK(o.value(QStringLiteral("steps")).toArray()[0].toObject().value(QStringLiteral("start_ns")).toInteger() ==
          2000000000LL + 3000 * 1000000LL);
    CHECK(QFileInfo::exists(r.s->folder() + QStringLiteral("/check.json")));
    CHECK(QFileInfo::exists(r.s->logDir() + QStringLiteral("/session.log")));
}

TEST(session_stop_kills_only_survivors_after_the_wait) {
    Rig r;
    r.dataAtMs = 1000;
    r.procs.ignoreTerm = {QStringLiteral("monado"), QStringLiteral("broker")};
    r.s->start(RecSession::Mode::Free, {}, 0);
    r.at(600);
    r.at(1000);
    CHECK(r.s->state() == RecState::Recording && r.s->steps() == nullptr);
    const qint64 monado = r.procs.pidOf(QStringLiteral("monado"));
    const qint64 ctrl = r.procs.pidOf(QStringLiteral("ctrltrack"));
    const qint64 broker = r.procs.pidOf(QStringLiteral("broker"));
    r.now = 2000;
    r.s->stop(2000);
    CHECK(r.s->state() == RecState::Stopping);
    CHECK(r.procs.signalled.size() == 1 && r.procs.signalled[0].pid == monado);
    r.at(2000 + kRecMonadoTermMs - 1);
    CHECK(r.procs.signalled.size() == 1); // Monado gets its full wait to finish writing
    r.at(2000 + kRecMonadoTermMs);
    // KILL to Monado, then at once TERM to the tracker (gone at TERM) and to the broker (ignores it)
    CHECK(r.procs.signalled.size() == 4);
    CHECK(r.procs.signalled[1].pid == monado && r.procs.signalled[1].sig == SIGKILL);
    CHECK(r.procs.signalled[2].pid == ctrl && r.procs.signalled[2].sig == SIGTERM);
    CHECK(r.procs.signalled[3].pid == broker && r.procs.signalled[3].sig == SIGTERM);
    r.at(2000 + kRecMonadoTermMs + kRecOtherTermMs - 1);
    CHECK(r.procs.signalled.size() == 4);
    r.at(2000 + kRecMonadoTermMs + kRecOtherTermMs);
    CHECK(r.procs.signalled.size() == 5 && r.procs.signalled[4].pid == broker && r.procs.signalled[4].sig == SIGKILL);
    CHECK(r.s->state() == RecState::Finished);
    for (const auto& sg : r.procs.signalled) {
        CHECK(sg.pid == monado || sg.pid == ctrl || sg.pid == broker); // never any other PID
    }
    CHECK(r.spoken.back() == QStringLiteral("Recording stopped."));
}

TEST(session_a_process_that_survives_kill_is_named) {
    Rig r;
    r.dataAtMs = 1000;
    r.procs.ignoreTerm = {QStringLiteral("monado")};
    r.procs.survivesKill = {QStringLiteral("monado")};
    r.s->start(RecSession::Mode::Free, {}, 0);
    r.at(600);
    r.at(1000);
    r.now = 1000;
    r.s->stop(1000);
    r.at(1000 + kRecMonadoTermMs);
    CHECK(r.s->state() == RecState::Stopping);
    r.at(1000 + kRecMonadoTermMs + kRecKillWaitMs);
    CHECK(r.s->state() == RecState::Finished);
    CHECK(r.s->message().contains(QStringLiteral("Could not stop: monado-service (PID")));
}

TEST(session_display_lease_retry_once) {
    Rig r;
    r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
    r.at(600);
    const qint64 first = r.procs.pidOf(QStringLiteral("monado"));
    r.monadoLog = QString::fromLatin1(kLeaseLog);
    r.procs.exit(first, 1);
    r.at(700);
    CHECK(r.s->state() == RecState::RetryWait && r.s->leaseRetries() == 1);
    CHECK(r.s->message().contains(QStringLiteral("Trying again in 4 s")));
    r.at(700 + kRecLeaseRetryMs - 1);
    CHECK(r.procs.started.size() == 3);
    r.at(700 + kRecLeaseRetryMs);
    CHECK(r.procs.started.size() == 4 && r.procs.started[3].name == QStringLiteral("monado"));
    CHECK(r.procs.started[3].logPath.endsWith(QStringLiteral("/monado-2.log")));
    // the broker and the tracker stayed
    CHECK(r.procs.running(r.procs.pidOf(QStringLiteral("broker"))) && r.procs.running(r.procs.pidOf(QStringLiteral("ctrltrack"))));
    r.monadoLog.clear();
    r.dataAtMs = 6000;
    r.at(6000);
    CHECK(r.s->state() == RecState::Recording);
}

TEST(session_display_lease_fails_twice_says_so) {
    Rig r;
    r.monadoLog = QString::fromLatin1(kLeaseLog);
    r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
    r.at(600);
    r.procs.exit(r.procs.pidOf(QStringLiteral("monado")), 1);
    r.at(700);
    r.at(700 + kRecLeaseRetryMs);
    r.procs.exit(r.procs.pidOf(QStringLiteral("monado")), 1);
    r.at(5000);
    CHECK(r.s->state() == RecState::Failed);
    CHECK(r.s->message().contains(QStringLiteral("could not be taken over, also on the second try")));
    CHECK(r.s->message().contains(QStringLiteral("DP-6")));
    CHECK(r.s->runningPids().empty()); // broker and tracker stopped too
    CHECK(r.spoken.back() == QStringLiteral("Recording failed."));
}

TEST(session_other_failures) {
    { // the socket never appears
        Rig r;
        r.socketAtMs = 1 << 30;
        r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
        r.at(kRecSocketWaitMs - 1);
        CHECK(r.s->state() == RecState::Starting);
        r.at(kRecSocketWaitMs);
        CHECK(r.s->state() == RecState::Failed && r.s->message().contains(QStringLiteral("did not open its socket")));
        CHECK(r.procs.signalled.size() == 1 && r.procs.signalled[0].pid == r.procs.pidOf(QStringLiteral("broker")));
    }
    { // Monado runs but no data
        Rig r;
        r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
        r.at(600);
        r.at(600 + kRecDataWaitMs);
        CHECK(r.s->state() == RecState::Failed && r.s->message().contains(QStringLiteral("wrote no camera or IMU data")));
    }
    { // Monado exits for another reason: no retry, the log's reason is shown
        Rig r;
        r.monadoLog = QStringLiteral("INFO x\nERROR pimax: camera calibration missing\n");
        r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
        r.at(600);
        r.procs.exit(r.procs.pidOf(QStringLiteral("monado")), 2);
        r.at(700);
        CHECK(r.s->state() == RecState::Failed && r.s->leaseRetries() == 0);
        CHECK(r.s->message().contains(QStringLiteral("Monado stopped while starting (code 2)")));
        CHECK(r.s->message().contains(QStringLiteral("camera calibration missing")));
    }
    { // the broker dies during the recording; what was recorded is still checked
        Rig r;
        r.dataAtMs = 1000;
        r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
        r.at(600);
        r.at(1000);
        r.procs.exit(r.procs.pidOf(QStringLiteral("broker")), 0, true);
        r.at(1100);
        CHECK(r.s->state() == RecState::Failed);
        CHECK(r.s->message().contains(QStringLiteral("camera broker stopped (it crashed)")));
        CHECK(r.s->message().contains(QStringLiteral("usable")));
    }
    { // the controller tracker ending does not stop the recording
        Rig r;
        r.dataAtMs = 1000;
        r.s->start(RecSession::Mode::Free, {}, 0);
        r.at(600);
        r.at(1000);
        r.procs.exit(r.procs.pidOf(QStringLiteral("ctrltrack")), 1);
        r.at(1100);
        CHECK(r.s->state() == RecState::Recording);
    }
    { // a program cannot be started
        Rig r;
        r.procs.failWith = QStringLiteral("Could not start pibridge-cam: no such file");
        CHECK(!r.s->start(RecSession::Mode::Guided, twoSteps(), 0));
        CHECK(r.s->state() == RecState::Failed && r.s->message().contains(QStringLiteral("Could not start pibridge-cam")));
    }
}

TEST(session_skip_stop_and_shutdown) {
    Rig r;
    r.dataAtMs = 1000;
    r.s->start(RecSession::Mode::Guided, twoSteps(), 0);
    r.at(600);
    r.at(1000);
    r.now = 1200;
    r.s->skipStep(1200);
    CHECK(r.s->steps()->current() == 1 && r.spoken.back() == QStringLiteral("Nod slowly."));
    r.now = 1500;
    r.s->stop(1500);
    CHECK(r.s->state() == RecState::Finished);
    QFile f(r.s->folder() + QStringLiteral("/steps.json"));
    CHECK(f.open(QIODevice::ReadOnly));
    const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
    CHECK(o.value(QStringLiteral("stopped_early")).toBool());
    CHECK(o.value(QStringLiteral("end_reason")).toString() == QStringLiteral("stopped by the user"));
    const QJsonArray steps = o.value(QStringLiteral("steps")).toArray();
    CHECK(steps.size() == 2 && steps[0].toObject().value(QStringLiteral("skipped")).toBool() &&
          steps[1].toObject().value(QStringLiteral("cut_short")).toBool());

    Rig q; // PiBridge closes during a recording
    q.dataAtMs = 1000;
    q.s->start(RecSession::Mode::Free, {}, 0);
    q.at(600);
    q.at(1000);
    q.s->shutdown();
    CHECK(q.procs.signalled.size() == 3 && q.procs.signalled[0].pid == q.procs.pidOf(QStringLiteral("monado")));
    CHECK(q.s->runningPids().empty() && !q.s->active());
}

TEST(chip_words) {
    CHECK(recChip(RecState::Idle, std::nullopt, true).text == QStringLiteral("READY"));
    CHECK(recChip(RecState::Idle, std::nullopt, false).text == QStringLiteral("NOT READY"));
    CHECK(recChip(RecState::Recording, std::nullopt, true).text == QStringLiteral("RECORDING"));
    RecordingInfo bad;
    CHECK(recChip(RecState::Finished, bad, true).text == QStringLiteral("CHECK: PROBLEM"));
    bad.ok = true;
    CHECK(recChip(RecState::Finished, bad, true).text == QStringLiteral("DONE"));
    CHECK(recChip(RecState::Failed, std::nullopt, true).kind == ChipKind::Error);
}

TEST(announcer_speech_tone_or_nothing) {
    FakeRunner fr;
    Announcer a(fr, [](const QString& n) { return n == QLatin1String("spd-say") ? QStringLiteral("/x/spd-say") : QString(); });
    CHECK(a.how() == QStringLiteral("spd-say"));
    a.say(QStringLiteral("Hold still."));
    a.say(QString());
    CHECK(fr.calls.size() == 1 && fr.calls[0].detached && fr.calls[0].program == QStringLiteral("/x/spd-say"));
    CHECK(fr.calls[0].args.last() == QStringLiteral("Hold still."));
    FakeRunner fr2;
    Announcer b(fr2, [](const QString& n) { return n == QLatin1String("paplay") ? QStringLiteral("/x/paplay") : QString(); });
    CHECK(b.how() == QStringLiteral("paplay (tone)"));
    b.say(QStringLiteral("Nod."));
    CHECK(fr2.calls.size() == 1 && fr2.calls[0].program == QStringLiteral("/x/paplay"));
    CHECK(fr2.calls[0].args.size() == 1 && QFileInfo(fr2.calls[0].args[0]).size() == Announcer::toneWav().size());
    FakeRunner fr3;
    Announcer c(fr3, [](const QString&) { return QString(); });
    c.say(QStringLiteral("x"));
    CHECK(c.how() == QStringLiteral("none") && fr3.calls.empty());
    const QByteArray w = Announcer::toneWav();
    CHECK(w.startsWith("RIFF") && w.mid(8, 4) == "WAVE" && w.size() == 44 + 22050 / 4 * 2);
}

PIBRIDGE_TEST_MAIN()
