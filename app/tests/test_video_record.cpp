// SPDX-License-Identifier: MIT
// The Dashboard's Record card without widgets (core/video_record.h): its words for every recorder state and VR
// runtime, the settings it writes (record.toml in a temporary XDG_CONFIG_HOME), the commands its buttons run
// (FakeRunner), and the tap's environment in the "Monado directly" session. Nothing is started.
#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include "core/inside_out.h"
#include "core/rec_setup.h"
#include "core/video_record.h"
#include "pibridge_test.h"

using namespace pibridge::app;
namespace rec = pibridge::record;

namespace {
struct Home {
    QTemporaryDir dir;
    Home() {
        for (const char* v : {"XDG_CONFIG_HOME", "XDG_RUNTIME_DIR", "HOME"}) {
            const QString d = dir.path() + QLatin1Char('/') + QString::fromLatin1(v).toLower();
            QDir().mkpath(d);
            qputenv(v, QFile::encodeName(d));
        }
        qunsetenv("PIMAX_RECORD_SHM");
        qunsetenv("PIBRIDGE_VR_SESSION_DIR");
        qunsetenv("PIMAX_VR_SESSION_DIR");
    }
};

rec::Status recording() {
    rec::Status s;
    s.state = "recording";
    s.file = "/v/pimax-20261019-120000.mp4";
    s.eyes = "left";
    s.fps = 30;
    s.quality = "high";
    s.encoder = "h264_nvenc";
    s.audio = "output";
    s.width = 1168;
    s.height = 1080;
    s.frames = 30 * 75;
    return s;
}
} // namespace

TEST(recording_shows_rec_the_length_and_stop) {
    RecordInputs in;
    in.status = recording();
    in.recorderRunning = true;
    in.directRunning = true;
    const RecordCardModel m = recordCardModel(in);
    CHECK(m.chip == QStringLiteral("REC") && m.chipKind == ChipKind::Error);
    CHECK(m.headline == QStringLiteral("Recording 1:15"));
    CHECK(m.detail == QStringLiteral("One eye, 1168x1080 at 30 fps, high quality, h264_nvenc, with sound."));
    CHECK(m.canStop && !m.canStart);
    CHECK(m.lastFolder == QStringLiteral("/v"));
    in.status.dropped_tap = 3;
    in.status.dropped_ring = 2;
    CHECK(recordCardModel(in).detail.contains(QStringLiteral("5 frames skipped by the recorder")));
}

TEST(the_source_follows_the_session) {
    RecordInputs in;
    in.directRunning = true;
    RecordCardModel m = recordCardModel(in);
    CHECK(m.canStart && m.chip == QStringLiteral("READY") && m.headline == QStringLiteral("Ready to record (Monado)"));
    // a monado-service started outside the app counts too
    in.directRunning = false;
    in.monadoRunning = true;
    CHECK(recordCardModel(in).canStart);
    // SteamVR (also SteamVR + Monado): SteamVR's mirror
    in.steamvrRunning = true;
    m = recordCardModel(in);
    CHECK(m.canStart && m.headline == QStringLiteral("Ready to record (SteamVR)"));
    CHECK(m.detail.contains(QStringLiteral("Vulkan capture layer"))); // the SteamVR default since the mirror crashed vrserver
    m = recordCardModel(RecordInputs{});
    CHECK(!m.canStart && m.headline == QStringLiteral("No VR session"));
    // a mirror recording says where it comes from; an unsupported mirror is the last error in plain words
    in.recorderRunning = true;
    in.status = recording();
    in.status.source = "mirror";
    CHECK(recordCardModel(in).detail.endsWith(QStringLiteral(", with sound, from SteamVR's mirror.")));
    in.recorderRunning = false;
    in.status = {};
    in.status.state = "error";
    in.status.note = "SteamVR's mirror could not start: SteamVR's mirror texture is not available: SteamVR does not "
                     "share its mirror texture with OpenGL here (VRCompositorError_SharedTexturesNotSupported)";
    m = recordCardModel(in);
    CHECK(m.canStart && m.error && m.detail.contains(QStringLiteral("SharedTexturesNotSupported")));
}

TEST(the_last_file_and_the_last_error_in_plain_words) {
    RecordInputs in;
    in.directRunning = true;
    in.status = recording();
    in.status.state = "idle";
    RecordCardModel m = recordCardModel(in);
    CHECK(m.canStart && m.detail.endsWith(QStringLiteral("Last: pimax-20261019-120000.mp4 (1:15)")));
    CHECK(m.lastFile == QStringLiteral("/v/pimax-20261019-120000.mp4"));
    in.status = {};
    in.status.state = "error";
    in.status.note = "Monado did not attach: this monado-service has no recorder tap";
    m = recordCardModel(in);
    CHECK(m.error && m.detail == QStringLiteral("Last try: Monado did not attach: this monado-service has no recorder tap"));
    CHECK(m.lastFolder.isEmpty());
}

TEST(starting_and_finishing) {
    RecordInputs in;
    in.recorderRunning = true;
    in.status.state = "starting";
    CHECK(recordCardModel(in).headline == QStringLiteral("Starting the recorder") && recordCardModel(in).canStop);
    in.status.state = "finalizing";
    in.status.file = "/v/a.mp4";
    const RecordCardModel m = recordCardModel(in);
    CHECK(m.chip == QStringLiteral("SAVING") && !m.canStop && !m.canStart);
}

TEST(the_settings_go_to_pibridges_record_toml_and_reach_pibridge_record) {
    Home h;
    CHECK(loadRecordSettings() == rec::Settings{});
    rec::Settings s = loadRecordSettings();
    CHECK(s.set("eyes", "both").has_value());
    CHECK(s.set("audio", "off").has_value());
    CHECK(saveRecordSettings(s).isEmpty());
    CHECK(recordSettingsPath() == h.dir.path() + QStringLiteral("/xdg_config_home/pibridge/record.toml"));
    CHECK(QFile::exists(recordSettingsPath()));
    CHECK(loadRecordSettings() == s);
    // a broken file: the defaults and why
    QFile f(recordSettingsPath());
    CHECK(f.open(QIODevice::WriteOnly | QIODevice::Truncate));
    f.write("fps = 45\n");
    f.close();
    QString err;
    CHECK(loadRecordSettings(&err) == rec::Settings{} && err.contains(QStringLiteral("expected 30, 60 or 72")));
}

TEST(the_buttons_run_pibridge_record_start_and_stop) {
    FakeRunner r;
    QString got;
    startRecording(r, QStringLiteral("/b/pibridge-record"), [&](int, const QString&) { got = QStringLiteral("started"); });
    stopRecording(r, QStringLiteral("/b/pibridge-record"), {});
    CHECK(r.calls.size() == 2);
    CHECK(r.calls[0].line() == QStringLiteral("/b/pibridge-record start") && !r.calls[0].detached);
    CHECK(r.calls[1].line() == QStringLiteral("/b/pibridge-record stop"));
    CHECK(got == QStringLiteral("started"));
}

TEST(monado_directly_turns_the_tap_on_at_the_ring_pibridge_record_uses) {
    Home h;
    CHECK(recordShmPath() == h.dir.path() + QStringLiteral("/xdg_runtime_dir/pimax-record.shm"));
    const IoSessionEnv e = buildIoEnv(IoSettings::recommended(), RecSetup{}, 90, {}, {});
    CHECK(e.monado.contains(QStringLiteral("PIMAX_RECORD=1")));
    CHECK(e.monado.contains(QStringLiteral("PIMAX_RECORD_SHM=") + recordShmPath()));
    // SteamVR's driver_monado gets neither (SteamVR's compositor draws the image there)
    CHECK(!e.driver.join(QLatin1Char(' ')).contains(QStringLiteral("PIMAX_RECORD")));
    qputenv("PIMAX_RECORD_SHM", "/x/ring.shm");
    CHECK(recordShmPath() == QStringLiteral("/x/ring.shm"));
    qunsetenv("PIMAX_RECORD_SHM");
}

PIBRIDGE_TEST_MAIN()
