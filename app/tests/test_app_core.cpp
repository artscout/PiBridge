// SPDX-License-Identifier: MIT
// Tests of the app's non-GUI logic: the conf writer, the status JSON readers, the quad views presets, the gaze shm
// reader (against a temporary file, never the real /dev/shm path) and the commands behind the buttons (FakeRunner).
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <clocale>
#include <cstring>
#include <string>

#include "core/actions.h"
#include "core/conf.h"
#include "core/gazeshm.h"
#include "core/paths.h"
#include "core/presets.h"
#include "core/runner.h"
#include "core/state.h"
#include "core/steamvr.h"
#include "core/status.h"
#include "core/system.h"
#include "pibridge_test.h"
#include "qv/settings.h"

using namespace pibridge::app;

namespace {

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

void writeAll(const QString& p, const QByteArray& b) {
    QDir().mkpath(QFileInfo(p).absolutePath());
    QFile f(p);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(b);
    }
}

// A decimal-comma locale, as the user may have; tests must pass under it.
void commaLocale() {
    if (!std::setlocale(LC_ALL, "de_DE.UTF-8")) {
        std::setlocale(LC_ALL, "ru_RU.UTF-8");
    }
}

// ---- gaze shm writer of the test (the layout of pimax-eye/src/shm.rs, its test sample)

template <typename T>
void put(std::array<uint8_t, kGazeSize>& f, size_t off, T v) {
    std::memcpy(f.data() + off, &v, sizeof v);
}

std::array<uint8_t, kGazeSize> sampleFile(uint64_t seqBegin, uint64_t seqEnd) {
    std::array<uint8_t, kGazeSize> f{};
    put(f, 0, seqBegin);
    put(f, 8, kGazeMagic);
    put(f, 12, kGazeVersion);
    put(f, 16, int64_t(123'456'789));
    put(f, 24, int64_t(999));
    put(f, 32, uint32_t(1 | 2 | 4 | 8)); // LEFT | RIGHT | COMBINED | LEFT_ORIGIN
    put(f, 36, 64.5f);
    const float lo[3]{-0.032f, 0, 0}, ld[3]{0.1f, 0.05f, -0.99f}, rd[3]{0.08f, 0.05f, -0.99f},
        cd[3]{0.09f, 0.05f, -0.99f};
    for (size_t i = 0; i < 3; i++) {
        put(f, 40 + i * 4, lo[i]);
        put(f, 52 + i * 4, ld[i]);
        put(f, 76 + i * 4, rd[i]);
        put(f, 100 + i * 4, cd[i]);
    }
    put(f, 112, uint64_t(42));
    put(f, 120, uint32_t(2));
    put(f, 128, seqEnd);
    return f;
}

} // namespace

// ---------------------------------------------------------------- conf writer

TEST(conf_set_keeps_comments_unknown_keys_and_order) {
    const std::string in = "# my settings\nfollow=vrserver\n\nfuture_key=42   # do not lose me\nrefresh_hz=90\n";
    const std::string out = confSet(in, "refresh_hz", "72");
    CHECK(out == "# my settings\nfollow=vrserver\n\nfuture_key=42   # do not lose me\nrefresh_hz=72\n");
}

TEST(conf_set_appends_missing_key_and_ignores_commented_line) {
    const std::string in = "#refresh_hz=72\nfollow=vrserver";
    const std::string out = confSet(in, "refresh_hz", "90");
    CHECK(out == "#refresh_hz=72\nfollow=vrserver\nrefresh_hz=90\n");
    CHECK(confGet(out, "refresh_hz") == std::optional<std::string>("90"));
}

TEST(conf_set_replaces_every_live_duplicate_and_keeps_trailing_comment) {
    const std::string in = "eye_tracking=on # first\n  eye_tracking = on\n";
    const std::string out = confSet(in, "eye_tracking", "off");
    CHECK(out == "eye_tracking=off  # first\neye_tracking=off\n");
    CHECK(confGet(out, "eye_tracking") == std::optional<std::string>("off"));
}

TEST(conf_get_trims_and_strips_comments) {
    CHECK(confGet("  audio_to_headset =  off   # note\n", "audio_to_headset") == std::optional<std::string>("off"));
    CHECK(!confGet("# audio_to_headset=off\n", "audio_to_headset").has_value());
    CHECK(!confGet("audio_to_headsetx=off\n", "audio_to_headset").has_value());
}

TEST(conf_numbers_have_a_point_in_every_locale) {
    commaLocale();
    CHECK(confNumber(0.5) == "0.5");
    CHECK(confNumber(90) == "90");
    CHECK(confNumber(-1.25) == "-1.25");
    // the copied quad views writer too
    qv::Settings s;
    s.peripheralDensity = 0.13f;
    CHECK(s.toLines().find("peripheral_density=0.13\n") != std::string::npos);
    qv::Settings back;
    CHECK(back.set("peripheral_density", "0,25") && std::fabs(back.peripheralDensity - 0.25f) < 1e-6f);
    std::setlocale(LC_ALL, "C");
}

TEST(conf_file_defaults_and_atomic_write) {
    QTemporaryDir tmp;
    const QString path = tmp.path() + QStringLiteral("/sub/pibridge.conf");
    Conf c(path);
    CHECK(c.refreshHz() == 90);
    CHECK(c.eyeTracking());
    CHECK(c.audioToHeadset());
    CHECK(c.value("follow", QStringLiteral("vrserver")) == QStringLiteral("vrserver"));
    CHECK(c.set("refresh_hz", "72"));
    CHECK(c.refreshHz() == 72);
    const QByteArray text = readAll(path);
    CHECK(text.startsWith("# pibridge settings"));
    CHECK(text.endsWith("refresh_hz=72\n"));
    // a hand edit in between survives the next write
    writeAll(path, text + "my_key=1\n");
    CHECK(c.set("eye_tracking", "off"));
    const QByteArray after = readAll(path);
    CHECK(after.contains("my_key=1\n") && after.contains("eye_tracking=off\n") && after.contains("refresh_hz=72\n"));
    CHECK(!QFileInfo::exists(path + QStringLiteral(".tmp")));
}

TEST(conf_reads_the_file_of_the_earlier_name_until_the_first_change) {
    QTemporaryDir tmp;
    const QString path = tmp.path() + QStringLiteral("/pibridge/pibridge.conf");
    const QString legacy = tmp.path() + QStringLiteral("/pimax-linux/pimax-linux.conf");
    writeAll(legacy, "# old\nrefresh_hz=72\nmy_key=1\n");
    Conf c(path, legacy);
    CHECK(c.refreshHz() == 72);
    CHECK(c.set("eye_tracking", "off"));
    const QByteArray text = readAll(path);
    CHECK(text.contains("refresh_hz=72\n") && text.contains("my_key=1\n") && text.contains("eye_tracking=off\n"));
    CHECK(readAll(legacy) == "# old\nrefresh_hz=72\nmy_key=1\n"); // never written
    // once the new file exists, the old one is no longer read
    writeAll(legacy, "refresh_hz=90\n");
    c.reload();
    CHECK(c.refreshHz() == 72);
    // paths
    qputenv("XDG_CONFIG_HOME", QFile::encodeName(tmp.path()));
    CHECK(confPath() == tmp.path() + QStringLiteral("/pibridge/pibridge.conf"));
    CHECK(legacyConfPath() == legacy);
    CHECK(eyeFileDirs().size() == 3 && eyeFileDirs()[1] == tmp.path() + QStringLiteral("/pimax-linux"));
    qunsetenv("XDG_CONFIG_HOME");
}

// ---------------------------------------------------------------- status files

TEST(status_keepalive_parses_every_field) {
    const auto k = parseKeepalive(R"({"state": "active", "model": "Crystal Super", "headset_present": true,
        "panel_on": true, "rate_hz": 90, "follow": "vrserver", "follow_running": true, "audio_on_headset": false,
        "error": "", "pid": 1234, "updated_unix_ms": 1760000000000})");
    CHECK(k.has_value());
    CHECK(k->running && k->state == QStringLiteral("active") && k->model == QStringLiteral("Crystal Super"));
    CHECK(k->headsetPresent && k->panelOn && k->rateHz == 90 && k->followRunning && !k->audioOnHeadset);
    CHECK(k->follow == QStringLiteral("vrserver") && k->error.isEmpty() && k->pid == 1234);
    CHECK(k->updatedMs == 1760000000000LL);
    CHECK(!parseKeepalive("{broken").has_value());
    CHECK(!parseKeepalive("[1,2]").has_value());
}

TEST(status_eyed_parses_every_field) {
    const auto e = parseEyed(R"({"state": "streaming", "profile": "user", "eyes_seen": 2, "samples_per_s": 119.6,
        "calibration": "/x/eye-calibration.toml", "follow": "vrserver", "follow_running": true, "error": "",
        "pid": 77, "updated_unix_ms": 5})");
    CHECK(e.has_value());
    CHECK(e->state == QStringLiteral("streaming") && e->profile == QStringLiteral("user") && e->eyesSeen == 2);
    CHECK_NEAR(e->samplesPerS, 119.6, 1e-9);
    CHECK(e->calibration == QStringLiteral("/x/eye-calibration.toml") && e->pid == 77);
    CHECK(eyeLine(*e) == QStringLiteral("Streaming: 2 eyes, 120 samples/s"));
    CHECK(eyeChip(*e).text == QStringLiteral("STREAMING"));
}

TEST(status_missing_or_stale_file_means_not_running) {
    QTemporaryDir tmp;
    const QString p = tmp.path() + QStringLiteral("/keepalived.json");
    const qint64 now = 1'760'000'010'000;
    CHECK(!readKeepalive(p, now).running); // missing
    writeAll(p, R"({"state":"idle","updated_unix_ms":1760000008000})");
    CHECK(readKeepalive(p, now).running); // 2 s old
    writeAll(p, R"({"state":"idle","updated_unix_ms":1760000000000})");
    CHECK(!readKeepalive(p, now).running); // 10 s old: the daemon died
    writeAll(p, "not json");
    CHECK(!readKeepalive(p, now).running);
    // without updated_unix_ms the file time counts
    const QString e = tmp.path() + QStringLiteral("/eyed.json");
    writeAll(e, R"({"state":"idle"})");
    CHECK(readEyed(e, QDateTime::currentMSecsSinceEpoch()).running);
    CHECK(!readEyed(e, QDateTime::currentMSecsSinceEpoch() + 60'000).running);
    CHECK(eyeLine(EyedStatus{}) == QStringLiteral("Service not running"));
}

TEST(status_headset_chip) {
    KeepaliveStatus k;
    CHECK(headsetChip(k).text == QStringLiteral("SERVICE OFF"));
    k.running = true;
    k.state = QStringLiteral("idle");
    k.headsetPresent = true;
    CHECK(headsetChip(k).text == QStringLiteral("SCREEN OFF") && headsetChip(k).kind == ChipKind::Warn);
    CHECK(headsetHint(k, false) == QStringLiteral("Start SteamVR to wake the headset."));
    k.state = QStringLiteral("active");
    k.panelOn = true;
    CHECK(headsetChip(k).text == QStringLiteral("SCREEN ON") && headsetChip(k).kind == ChipKind::Info);
    k.state = QStringLiteral("no-headset");
    CHECK(headsetChip(k).text == QStringLiteral("NO HEADSET"));
    k.state = QStringLiteral("error");
    k.error = QStringLiteral("The headset did not answer.");
    CHECK(headsetChip(k).text == QStringLiteral("ERROR") && headsetChip(k).kind == ChipKind::Error);
    CHECK(headsetHint(k, true) == k.error);
}

// ---------------------------------------------------------------- presets

TEST(presets_match_and_apply) {
    qv::Settings s; // defaults: 0.40 / 1.00 / 0.35 -> no preset
    CHECK(matchPreset(s) == -1);
    for (int i = 0; i < int(kQvPresets.size()); i++) {
        applyPreset(s, i);
        CHECK(matchPreset(s) == i);
    }
    applyPreset(s, 1); // BALANCED 0.20 / 1.0 / 0.35
    CHECK_NEAR(s.peripheralDensity, 0.20, 1e-6);
    CHECK_NEAR(s.focusDensity, 1.0, 1e-6);
    CHECK_NEAR(s.gazeSize[0], 0.35, 1e-6);
    CHECK_NEAR(s.gazeSize[1], 0.35, 1e-6);
    s.gazeSize[1] = 0.36f; // one value off: CUSTOM
    CHECK(matchPreset(s) == -1);
    // what the page writes as percent and reads back from the file still matches
    qv::Settings q;
    applyPreset(q, 2);
    const qv::Settings r = qv::parseSettings(q.toLines(), qv::AppId{});
    CHECK(matchPreset(r) == 2);
    // other settings are not touched
    s.sharpen = 0.9f;
    applyPreset(s, 3);
    CHECK_NEAR(s.sharpen, 0.9, 1e-6);
    applyPreset(s, 9); // out of range: nothing
    CHECK(matchPreset(s) == 3);
}

// ---------------------------------------------------------------- gaze shm

TEST(gaze_shm_reads_a_consistent_file) {
    QTemporaryDir tmp;
    const QString p = tmp.path() + QStringLiteral("/gaze");
    CHECK(!readGaze(QFile::encodeName(p).constData()).has_value()); // missing
    const auto f = sampleFile(4, 4);
    writeAll(p, QByteArray(reinterpret_cast<const char*>(f.data()), int(f.size())));
    const auto g = readGaze(QFile::encodeName(p).constData());
    CHECK(g.has_value());
    CHECK(g->tNs == 123'456'789 && g->writtenNs == 999);
    CHECK(g->flags == (1u | 2u | 4u | 8u));
    CHECK(g->direction[0][0] == 0.1f && g->direction[0][2] == -0.99f);
    CHECK(g->direction[1][0] == 0.08f);
    CHECK(g->origin[0][0] == -0.032f);
    CHECK(g->ipdMm == 64.5f && g->sample == 42 && g->state == 2);
    float tx = 0, ty = 0;
    CHECK(gazeTangent(*g, tx, ty));
    CHECK_NEAR(tx, 0.09 / 0.99, 1e-6);
    CHECK_NEAR(ty, 0.05 / 0.99, 1e-6);
}

TEST(gaze_shm_refuses_torn_snapshots) {
    auto f = sampleFile(4, 4);
    CHECK(decodeGaze(f).has_value());
    put(f, 0, uint64_t(5)); // writer in the middle
    CHECK(!decodeGaze(f).has_value());
    put(f, 0, uint64_t(6)); // begin newer than end
    CHECK(!decodeGaze(f).has_value());
    put(f, 0, uint64_t(4));
    f[8] ^= 1; // another layout
    CHECK(!decodeGaze(f).has_value());
    auto z = sampleFile(0, 0); // never published
    CHECK(!decodeGaze(z).has_value());
    CHECK(!decodeGaze(std::span<const uint8_t>(f.data(), 100)).has_value()); // short
    // torn on disk: the reader gives up after its retries
    QTemporaryDir tmp;
    const QString p = tmp.path() + QStringLiteral("/gaze");
    const auto t = sampleFile(7, 6);
    writeAll(p, QByteArray(reinterpret_cast<const char*>(t.data()), int(t.size())));
    CHECK(!readGaze(QFile::encodeName(p).constData()).has_value());
}

TEST(gaze_tangent_falls_back_to_the_eyes) {
    GazeSnapshot g;
    float x = 0, y = 0;
    CHECK(!gazeTangent(g, x, y)); // nothing valid
    g.flags = kGazeLeft | kGazeRight;
    g.direction[0][0] = 0.1f;
    g.direction[0][2] = -1.f;
    g.direction[1][0] = 0.3f;
    g.direction[1][2] = -1.f;
    CHECK(gazeTangent(g, x, y));
    CHECK_NEAR(x, 0.2, 1e-6);
    CHECK_NEAR(y, 0.0, 1e-6);
    g.direction[0][2] = g.direction[1][2] = 1.f; // backwards
    CHECK(!gazeTangent(g, x, y));
}

// ---------------------------------------------------------------- commands behind the buttons

TEST(actions_run_the_documented_commands) {
    QTemporaryDir tmp;
    Conf conf(tmp.path() + QStringLiteral("/pibridge.conf"));
    FakeRunner r;
    Actions a(r, conf, [](const QString& n) { return QStringLiteral("/opt/t/") + n; },
              tmp.path() + QStringLiteral("/eye-profile-crystal-super.bin"));
    a.setEyeFileDirs({});

    CHECK(a.launchSteamVR());
    CHECK(r.calls.back().detached && r.calls.back().line() == QStringLiteral("steam steam://rungameid/250820"));

    int got = -2;
    a.restartServices([&](int c, const QString&) { got = c; });
    CHECK(got == 0);
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user restart pibridge-keepalived pibridge-eyed"));
    a.restartHeadset({});
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user restart pibridge-keepalived"));

    CHECK(a.setEyeTracking(false, {}));
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user stop pibridge-eyed"));
    CHECK(!conf.eyeTracking());
    CHECK(a.setEyeTracking(true, {}));
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user start pibridge-eyed"));
    CHECK(conf.eyeTracking());

    CHECK(a.setRefreshHz(72, {}));
    CHECK(conf.refreshHz() == 72);
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user kill --signal=HUP pibridge-keepalived"));
    const size_t n = r.calls.size();
    CHECK(!a.setRefreshHz(60, {})); // not a rate of the headset
    CHECK(r.calls.size() == n);

    CHECK(a.setAudioToHeadset(false, {}));
    CHECK(!conf.audioToHeadset());
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user kill --signal=HUP pibridge-keepalived"));

    CHECK(a.calibrate({}));
    CHECK(r.calls[r.calls.size() - 2].line() == QStringLiteral("/opt/t/pibridge-eyecal"));
    CHECK(a.recenter({}));
    CHECK(r.calls[r.calls.size() - 2].line() == QStringLiteral("/opt/t/pibridge-eyecal --recenter"));

    CHECK(a.importProfile(QStringLiteral("/w/calibration.setpm"), {}));
    const QString out = tmp.path() + QStringLiteral("/eye-profile-crystal-super.bin");
    CHECK(r.calls[r.calls.size() - 2].line() ==
          QStringLiteral("/opt/t/pibridge-eye profile /w/calibration.setpm -o ") + out);
    CHECK(r.calls.back().line() == QStringLiteral("systemctl --user kill --signal=HUP pibridge-eyed"));
    CHECK(conf.value("eye_profile", {}) == out);
}

TEST(calibrate_and_recenter_take_the_eye_chip_from_pibridge_eyed_and_give_it_back) {
    QTemporaryDir tmp;
    Conf conf(tmp.path() + QStringLiteral("/pibridge.conf"));
    FakeRunner r;
    Actions a(r, conf, [](const QString& n) { return QStringLiteral("/opt/t/") + n; }, tmp.path() + QStringLiteral("/p.bin"));
    a.setEyeFileDirs({});
    int code = -2;
    r.exitCode = 3;
    CHECK(a.calibrate([&](int c, const QString&) { code = c; }));
    CHECK(r.calls.size() == 3);
    CHECK(r.calls[0].line() == QStringLiteral("systemctl --user stop pibridge-eyed"));
    CHECK(r.calls[1].line() == QStringLiteral("/opt/t/pibridge-eyecal"));
    CHECK(r.calls[2].line() == QStringLiteral("systemctl --user start pibridge-eyed"));
    CHECK(code == 3); // the tool's own exit code reaches the page
    // eye tracking switched off: the daemon stays stopped afterwards
    CHECK(conf.set("eye_tracking", "off"));
    r.calls.clear();
    CHECK(a.recenter({}));
    CHECK(r.calls.size() == 2);
    CHECK(r.calls[0].line() == QStringLiteral("systemctl --user stop pibridge-eyed"));
    CHECK(r.calls[1].line() == QStringLiteral("/opt/t/pibridge-eyecal --recenter"));
    // while the tool runs, the daemon is not started again
    CHECK(conf.set("eye_tracking", "on"));
    FakeRunner slow;
    slow.answer = false;
    Actions b(slow, conf, [](const QString& n) { return n; }, tmp.path() + QStringLiteral("/p.bin"));
    b.setEyeFileDirs({});
    CHECK(b.calibrate({}));
    CHECK(slow.calls.size() == 1 && slow.calls[0].line() == QStringLiteral("systemctl --user stop pibridge-eyed"));
}

TEST(actions_without_tools_run_nothing) {
    QTemporaryDir tmp;
    Conf conf(tmp.path() + QStringLiteral("/pibridge.conf"));
    FakeRunner r;
    Actions a(r, conf, [](const QString&) { return QString(); }, tmp.path() + QStringLiteral("/p.bin"));
    a.setEyeFileDirs({});
    CHECK(!a.calibrate({}));
    CHECK(!a.recenter({}));
    CHECK(!a.importProfile(QStringLiteral("/w/calibration.setpm"), {}));
    CHECK(r.calls.empty());
    // a failed import leaves the conf alone
    FakeRunner r2;
    r2.exitCode = 1;
    Actions b(r2, conf, [](const QString& n) { return n; }, tmp.path() + QStringLiteral("/p.bin"));
    b.setEyeFileDirs({});
    int code = 0;
    CHECK(b.importProfile(QStringLiteral("x.setpm"), [&](int c, const QString&) { code = c; }));
    CHECK(code == 1 && r2.calls.size() == 1 && !confGet(conf.text(), "eye_profile").has_value());
}

TEST(process_running_reads_comm) {
    QTemporaryDir tmp;
    writeAll(tmp.path() + QStringLiteral("/101/comm"), "bash\n");
    writeAll(tmp.path() + QStringLiteral("/202/comm"), "vrserver\n");
    writeAll(tmp.path() + QStringLiteral("/self/comm"), "vrmonitor\n");
    CHECK(processRunning(QStringLiteral("vrserver"), tmp.path()));
    CHECK(!processRunning(QStringLiteral("vrmonitor"), tmp.path())); // not a pid dir
    CHECK(!processRunning(QStringLiteral("vrcompositor"), tmp.path()));
    writeAll(tmp.path() + QStringLiteral("/303/comm"), "pibridge-keepalive\n"); // the kernel cuts at 15
    CHECK(processRunning(QStringLiteral("pibridge-keepalived"), tmp.path()));
}


TEST(pibridge_eyecal_reads_the_settings_itself) {
    QTemporaryDir tmp;
    Conf conf(tmp.path() + QStringLiteral("/pibridge.conf"));
    CHECK(conf.set("eye_start_list", "/x/list.txt"));
    CHECK(conf.set("eye_profile", "/x/p.bin"));
    FakeRunner r;
    Actions a(r, conf, [](const QString& n) { return QStringLiteral("/opt/t/") + n; }, tmp.path() + QStringLiteral("/p.bin"));
    a.setEyeFileDirs({});
    CHECK(a.eyecalTool() == QStringLiteral("/opt/t/pibridge-eyecal"));
    // no PIMAX_EYE_PROFILE, no --frames: the tool takes eye_start_list and eye_profile from pibridge.conf as pibridge-eyed
    CHECK(a.calibrate({}));
    CHECK(r.calls[1].line() == QStringLiteral("/opt/t/pibridge-eyecal"));
    r.calls.clear();
    CHECK(a.recenter({}));
    CHECK(r.calls[1].line() == QStringLiteral("/opt/t/pibridge-eyecal --recenter"));
    // only the fallback is there: pimax-eyecal
    Actions b(r, conf, [](const QString& n) { return n == QStringLiteral("pimax-eyecal") ? QStringLiteral("/o/pimax-eyecal") : QString(); },
              tmp.path() + QStringLiteral("/p.bin"));
    CHECK(b.eyecalTool() == QStringLiteral("/o/pimax-eyecal"));
}

TEST(the_pimax_eyecal_fallback_gets_the_start_list_and_the_profile_eyed_uses) {
    QTemporaryDir tmp;
    Conf conf(tmp.path() + QStringLiteral("/pibridge.conf"));
    FakeRunner r;
    // pibridge-eyecal is not found: the older tool, with the arguments it needs
    Actions a(r, conf,
              [](const QString& n) { return n == QStringLiteral("pibridge-eyecal") ? QString() : QStringLiteral("/opt/t/") + n; },
              tmp.path() + QStringLiteral("/p.bin"));
    const QString first = tmp.path() + QStringLiteral("/pl"), second = tmp.path() + QStringLiteral("/ovr");
    QDir().mkpath(first);
    QDir().mkpath(second);
    a.setEyeFileDirs({first, second});
    // only the second dir has the files: they are found there
    CHECK(QFile(second + QStringLiteral("/eye-start-crystal-super.txt")).open(QIODevice::WriteOnly));
    CHECK(QFile(second + QStringLiteral("/eye-profile-crystal-super.bin")).open(QIODevice::WriteOnly));
    CHECK(a.calibrate({}));
    CHECK(r.calls[1].line() == QStringLiteral("env PIMAX_EYE_PROFILE=") + second +
          QStringLiteral("/eye-profile-crystal-super.bin /opt/t/pimax-eyecal --pattern pimax --frames ") + second +
          QStringLiteral("/eye-start-crystal-super.txt"));
    // the conf wins over the dirs
    CHECK(conf.set("eye_start_list", "/x/list.txt"));
    CHECK(conf.set("eye_profile", "/x/p.bin"));
    r.calls.clear();
    CHECK(a.recenter({}));
    CHECK(r.calls[1].line() ==
          QStringLiteral("env PIMAX_EYE_PROFILE=/x/p.bin /opt/t/pimax-eyecal --recenter --pattern pimax --frames /x/list.txt"));
}

// ---------------------------------------------------------------- SteamVR (fakes only: never the real SteamVR)

namespace {

// HOME and XDG_CONFIG_HOME point into a temporary directory while it lives, so nothing of the user's is read.
struct TempHome {
    QTemporaryDir dir;
    QByteArray oldHome = qgetenv("HOME");
    QByteArray oldXdg = qgetenv("XDG_CONFIG_HOME");
    bool hadXdg = qEnvironmentVariableIsSet("XDG_CONFIG_HOME");
    TempHome() {
        qputenv("HOME", QFile::encodeName(dir.path()));
        qunsetenv("XDG_CONFIG_HOME");
    }
    ~TempHome() {
        qputenv("HOME", oldHome);
        if (hadXdg) {
            qputenv("XDG_CONFIG_HOME", oldXdg);
        } else {
            qunsetenv("XDG_CONFIG_HOME");
        }
    }
    QString path() const { return dir.path(); }
};

VrDevice device(int index, VrClass cls, VrRole role, bool poseValid, int result = vrtrack::RunningOk) {
    VrDevice d;
    d.index = index;
    d.cls = cls;
    d.role = role;
    d.poseValid = poseValid;
    d.trackingResult = result;
    return d;
}

SteamVrStatus connectedWith(std::vector<VrDevice> devices) {
    FakeSteamVrSource src;
    src.polls = {std::move(devices)};
    SteamVrLink link(src, [] { return qint64(0); });
    return link.tick(true);
}

} // namespace

TEST(steamvr_vrpath_file_and_runtime) {
    TempHome home;
    const QString file = openVrPathsFile();
    CHECK(file == home.path() + QStringLiteral("/.config/openvr/openvrpaths.vrpath"));
    CHECK(openVrRuntimeDir(file).isEmpty()); // missing
    writeAll(file, "{ not json");
    CHECK(openVrRuntimeDir(file).isEmpty()); // broken
    writeAll(file, R"({"config": ["/c"], "version": 1})");
    CHECK(openVrRuntimeDir(file).isEmpty()); // no runtime
    writeAll(file, R"({"runtime": []})");
    CHECK(openVrRuntimeDir(file).isEmpty());
    writeAll(file, R"({"runtime": [42]})");
    CHECK(openVrRuntimeDir(file).isEmpty());
    writeAll(file, R"({"config":["/c"],"external_drivers":["/d"],"jsonid":"vrpathreg",
                      "runtime":["/games/steamapps/common/SteamVR/", "/other/SteamVR"],"version":1})");
    CHECK(openVrRuntimeDir(file) == QStringLiteral("/games/steamapps/common/SteamVR")); // the first, cleaned
    qputenv("XDG_CONFIG_HOME", QFile::encodeName(home.path() + QStringLiteral("/xdg")));
    CHECK(openVrPathsFile() == home.path() + QStringLiteral("/xdg/openvr/openvrpaths.vrpath"));
}

TEST(steamvr_library_path) {
    CHECK(openVrApiLibrary(QStringLiteral("/games/SteamVR")) ==
          QStringLiteral("/games/SteamVR/bin/linux64/libopenvr_api.so"));
    CHECK(openVrApiLibrary(QString()).isEmpty());
}

TEST(steamvr_real_backend_without_steamvr_is_not_installed) {
    TempHome home; // the real backend only ever sees the temporary home here
    auto src = makeOpenVrSource();
    CHECK(!src->poll().has_value()); // not connected
    src->disconnect();               // safe when not connected
    VrConnect c = src->connect();
    CHECK(c.result == VrConnect::NotInstalled && !c.error.isEmpty()); // no vrpath file
    const QString rt = home.path() + QStringLiteral("/SteamVR");
    writeAll(home.path() + QStringLiteral("/.config/openvr/openvrpaths.vrpath"),
             QStringLiteral(R"({"runtime": ["%1"]})").arg(rt).toUtf8());
    c = src->connect();
    CHECK(c.result == VrConnect::NotInstalled && c.error.contains(QStringLiteral("missing"))); // no library
    writeAll(openVrApiLibrary(rt), "not an ELF file");
    c = src->connect();
    CHECK(c.result == VrConnect::NotInstalled && c.error.contains(QStringLiteral("could not load")));
}

TEST(steamvr_fake_devices_map_to_the_dashboard) {
    const SteamVrStatus s = connectedWith(fakeVrDevices());
    CHECK(s.state == VrLinkState::Connected && s.devices.size() == 5);
    const ControllersView v = controllersView(s, true);
    CHECK(v.header.text == QStringLiteral("2/2 CONNECTED") && v.header.kind == ChipKind::Info);
    CHECK(v.message.isEmpty() && v.extra.isEmpty());
    CHECK(v.left.present && v.left.name == QStringLiteral("Valve Index Controller"));
    CHECK(v.left.chip.text == QStringLiteral("CONNECTED") && v.left.chip.kind == ChipKind::Info);
    CHECK(v.left.batteryPercent == 82 && v.left.charging && v.left.batteryText == QStringLiteral("82 % charging"));
    CHECK(v.left.detail.isEmpty());
    CHECK(v.right.chip.text == QStringLiteral("NOT TRACKED") && v.right.chip.kind == ChipKind::Warn);
    CHECK(v.right.detail == QStringLiteral("Out of range"));
    CHECK(v.right.batteryPercent == 45 && !v.right.charging && v.right.batteryText == QStringLiteral("45 %"));
    const TrackingView t = trackingView(s, true);
    CHECK(t.summary == QStringLiteral("2 base stations, 2 tracking") && t.message.isEmpty());
    CHECK(t.stations.size() == 2 && t.stations[0].name == QStringLiteral("Station 1") &&
          t.stations[1].name == QStringLiteral("Station 2"));
    CHECK(t.stations[0].model == QStringLiteral("SteamVR Base Station 2.0") && t.stations[0].text == QStringLiteral("ok"));
    // the fake mode shows exactly this, through the same link
    CHECK(fakeState(true).steamvr.devices.size() == 5 && fakeState(true).steamvr.state == VrLinkState::Connected);
    CHECK(fakeState(false).steamvr.state == VrLinkState::NotRunning && fakeState(false).steamvr.devices.empty());
}

TEST(steamvr_roles_chips_battery_and_stations) {
    VrDevice left = device(1, VrClass::Controller, VrRole::Left, false, vrtrack::Uninitialized);
    left.model = QStringLiteral("Custom Ctrl");
    left.activity = vractivity::Standby;
    VrDevice second = device(2, VrClass::Controller, VrRole::Left, true); // a second left one: listed below
    second.controllerType = QStringLiteral("vive_controller");
    second.hasBattery = true;
    second.battery = 0.054f;
    VrDevice tracker = device(3, VrClass::GenericTracker, VrRole::None, true);
    tracker.controllerType = QStringLiteral("vive_tracker_waist");
    VrDevice hmd = device(0, VrClass::Hmd, VrRole::None, true);
    VrDevice st1 = device(4, VrClass::TrackingReference, VrRole::None, true);
    st1.model = QStringLiteral("HTC V2-XD/XE");
    VrDevice st2 = device(5, VrClass::TrackingReference, VrRole::None, false, vrtrack::Uninitialized);
    VrDevice st3 = device(6, VrClass::TrackingReference, VrRole::None, false, vrtrack::Uninitialized);
    const SteamVrStatus s = connectedWith({hmd, left, second, tracker, st1, st2, st3});

    const ControllersView v = controllersView(s, true);
    CHECK(v.header.text == QStringLiteral("1/2 CONNECTED") && v.header.kind == ChipKind::Warn);
    CHECK(v.left.name == QStringLiteral("Custom Ctrl") && v.left.chip.text == QStringLiteral("NOT TRACKED"));
    CHECK(v.left.detail == QStringLiteral("Not seen, standby"));
    CHECK(v.left.batteryPercent == -1 && v.left.batteryText.isEmpty());
    CHECK(!v.right.present && v.right.chip.text == QStringLiteral("DISCONNECTED") && v.right.chip.kind == ChipKind::Off);
    CHECK(v.extra.size() == 2);
    CHECK(v.extra.value(0) == QStringLiteral("HTC Vive Controller: tracking, battery 5 %"));
    CHECK(v.extra.value(1) == QStringLiteral("Vive Tracker: tracking"));

    const TrackingView t = trackingView(s, true);
    CHECK(t.summary == QStringLiteral("3 base stations, 1 tracking"));
    CHECK(t.stations.size() == 3 && t.stations[0].model == QStringLiteral("SteamVR Base Station 1.0"));
    CHECK(t.stations[0].tracking && !t.stations[1].tracking && t.stations[1].text == QStringLiteral("not seen"));
    CHECK(t.stations[1].model == QStringLiteral("Base station")); // no model number

    const SteamVrStatus one = connectedWith({st1});
    CHECK(trackingView(one, true).summary == QStringLiteral("1 base station, 1 tracking"));
    const SteamVrStatus none = connectedWith({hmd});
    CHECK(trackingView(none, true).summary == QStringLiteral("No base stations seen"));
    CHECK(controllersView(none, true).header.text == QStringLiteral("0/2 CONNECTED"));

    // without SteamVR: plain words, nothing listed
    const ControllersView off = controllersView(s, false);
    CHECK(off.header.text == QStringLiteral("STEAMVR OFF") && off.message == QStringLiteral("Start SteamVR to see the controllers."));
    CHECK(!off.left.present && off.extra.isEmpty());
    CHECK(trackingView(s, false).stations.empty() && trackingView(s, false).summary.isEmpty());
    SteamVrStatus ni;
    ni.state = VrLinkState::NotInstalled;
    CHECK(controllersView(ni, true).header.text == QStringLiteral("NO STEAMVR"));
    CHECK(controllersView(ni, true).message.startsWith(QStringLiteral("SteamVR not installed")));
    SteamVrStatus w;
    w.state = VrLinkState::Waiting;
    w.error = QStringLiteral("SteamVR is not ready yet");
    CHECK(controllersView(w, true).message == QStringLiteral("Connecting to SteamVR... (SteamVR is not ready yet)"));
    CHECK(vrTrackingWords(vrtrack::CalibratingInProgress) == QStringLiteral("Calibrating"));
    CHECK(vrTrackingWords(vrtrack::FallbackRotationOnly) == QStringLiteral("Rotation only"));
}

TEST(steamvr_link_backs_off_and_never_tries_without_vrserver) {
    qint64 now = 1'000'000;
    FakeSteamVrSource src;
    src.connects = {{VrConnect::Failed, QStringLiteral("SteamVR is not ready yet")},
                    {VrConnect::Failed, QStringLiteral("SteamVR is not ready yet")},
                    {VrConnect::Ok, {}}};
    src.polls = {fakeVrDevices(), fakeVrDevices(), std::nullopt, fakeVrDevices()};
    SteamVrLink link(src, [&] { return now; });

    for (int i = 0; i < 20; i++, now += 1000) { // vrserver not running: never a connect
        CHECK(link.tick(false).state == VrLinkState::NotRunning);
    }
    CHECK(src.connectCalls == 0);

    SteamVrStatus s = link.tick(true); // first try at once
    CHECK(src.connectCalls == 1 && s.state == VrLinkState::Waiting && s.error == QStringLiteral("SteamVR is not ready yet"));
    for (int ms = 1000; ms < 5000; ms += 1000) { // every second, but no new try before 5 s
        now += 1000;
        CHECK(link.tick(true).state == VrLinkState::Waiting);
    }
    CHECK(src.connectCalls == 1);
    now += 999; // 4999 ms after the first try
    link.tick(true);
    CHECK(src.connectCalls == 1);
    now += 1; // 5 s
    link.tick(true);
    CHECK(src.connectCalls == 2);
    now += 5000;
    s = link.tick(true); // third try works and reads the devices at once
    CHECK(src.connectCalls == 3 && s.state == VrLinkState::Connected && s.devices.size() == 5 && link.connected());
    now += 1000;
    s = link.tick(true);
    CHECK(s.state == VrLinkState::Connected && src.connectCalls == 3 && src.pollCalls == 2);

    now += 1000;
    s = link.tick(true); // VREvent_Quit: let go, back off
    CHECK(s.state == VrLinkState::Waiting && !link.connected() && src.disconnectCalls == 1 && s.devices.empty());
    now += 4000;
    link.tick(true);
    CHECK(src.connectCalls == 3);
    now += 1000;
    s = link.tick(true);
    CHECK(src.connectCalls == 4 && s.state == VrLinkState::Connected);

    // vrserver goes away while connected: disconnect at once, then no tries at all
    now += 1000;
    s = link.tick(false);
    CHECK(s.state == VrLinkState::NotRunning && s.devices.empty() && !link.connected() && src.disconnectCalls == 2);
    for (int i = 0; i < 30; i++, now += 1000) {
        link.tick(false);
    }
    CHECK(src.connectCalls == 4);
    // and back: tries again (the last try is long ago)
    CHECK(link.tick(true).state == VrLinkState::Connected && src.connectCalls == 5);
    // vrserver restarts within 5 s of the last try: waits out the back-off
    now += 1000;
    link.tick(false);
    now += 1000;
    CHECK(link.tick(true).state == VrLinkState::Waiting && src.connectCalls == 5);
    now += 3000;
    link.tick(true);
    CHECK(src.connectCalls == 6);

    // not installed is backed off the same way
    FakeSteamVrSource ni;
    ni.connects = {{VrConnect::NotInstalled, QStringLiteral("no SteamVR runtime")}};
    SteamVrLink l2(ni, [&] { return now; });
    CHECK(l2.tick(true).state == VrLinkState::NotInstalled);
    now += 2000;
    CHECK(l2.tick(true).state == VrLinkState::NotInstalled && ni.connectCalls == 1);
    now += 3000;
    l2.tick(true);
    CHECK(ni.connectCalls == 2);
}

namespace {

// A fake that reports into counters that outlive it (the worker owns and destroys its source).
struct CountingSource : FakeSteamVrSource {
    std::atomic<int>* disconnects;
    explicit CountingSource(std::atomic<int>* d) : disconnects(d) { polls = {fakeVrDevices()}; }
    void disconnect() override {
        if (connected) {
            (*disconnects)++;
        }
        FakeSteamVrSource::disconnect();
    }
};

} // namespace

TEST(steamvr_worker_polls_on_its_thread_and_disconnects_when_stopped) {
    std::atomic<int> disconnects{0};
    std::mutex m;
    std::condition_variable cv;
    int delivered = 0;
    bool connected = false;
    {
        SteamVrWorker w(std::make_unique<CountingSource>(&disconnects), [] { return true; },
                        [&](const SteamVrStatus& s) {
                            std::lock_guard lk(m);
                            delivered++;
                            connected = connected || s.state == VrLinkState::Connected;
                            cv.notify_all();
                        },
                        5);
        std::unique_lock lk(m);
        cv.wait_for(lk, std::chrono::seconds(5), [&] { return delivered >= 3; });
        CHECK(delivered >= 3 && connected);
    }
    CHECK(disconnects == 1); // the worker let go of SteamVR when it stopped
}

TEST(serials_reach_the_rows_for_support) {
    SteamVrStatus s;
    s.state = VrLinkState::Connected;
    s.devices = fakeVrDevices();
    const ControllersView v = controllersView(s, true);
    CHECK(v.left.serial == QStringLiteral("LHR-FAKE0001"));
    CHECK(v.right.serial == QStringLiteral("LHR-FAKE0002"));
    const TrackingView t = trackingView(s, true);
    CHECK(t.stations.size() == 2 && t.stations[0].serial == QStringLiteral("LHB-FAKE0001"));
    // without SteamVR nothing is shown
    const ControllersView off = controllersView(s, false);
    CHECK(!off.left.present || off.left.serial.isEmpty() || off.header.text != QStringLiteral("2/2 CONNECTED"));
}

TEST(paused_daemons_in_plain_words) {
    auto k = parseKeepalive("{\"state\": \"paused\", \"paused\": \"paused: inside-out tracking\", \"pid\": 5}");
    CHECK(k.has_value() && k->state == QStringLiteral("paused"));
    CHECK(headsetChip(*k).text == QStringLiteral("PAUSED: INSIDE-OUT"));
    CHECK(headsetHint(*k, true).startsWith(QStringLiteral("Paused: inside-out tracking runs")));
    auto e = parseEyed("{\"state\": \"paused\", \"pid\": 6}");
    CHECK(e.has_value() && eyeChip(*e).text == QStringLiteral("PAUSED: INSIDE-OUT"));
    CHECK(eyeLine(*e).startsWith(QStringLiteral("Paused")));
}

TEST(steamvr_stage_file_text_and_vrserver_pid) {
    std::array<float, 12> m{1, 0, 0, 0.1f, 0, 1, 0, 1.52f, 0, 0, 1, -0.2f};
    CHECK(steamVrStageText(4242, 2, m) ==
          QStringLiteral("pibridge-steamvr-stage 1\nvrserver 4242\nuniverse 2\nstanding_from_raw 1.000000 0.000000 "
                         "0.000000 0.100000 0.000000 1.000000 0.000000 1.520000 0.000000 0.000000 1.000000 -0.200000\n"));
    QTemporaryDir d;
    QDir().mkpath(d.path() + QStringLiteral("/77"));
    QDir().mkpath(d.path() + QStringLiteral("/self"));
    QFile f(d.path() + QStringLiteral("/77/comm"));
    CHECK(f.open(QIODevice::WriteOnly) && f.write("vrserver\n") > 0);
    f.close();
    CHECK(vrserverPid(d.path()) == 77);
    QFile::remove(d.path() + QStringLiteral("/77/comm"));
    CHECK(vrserverPid(d.path()) == 0);
}

PIBRIDGE_TEST_MAIN()
