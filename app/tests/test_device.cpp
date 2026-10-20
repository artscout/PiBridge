// SPDX-License-Identifier: MIT
// Tests of the Device page's logic (core/device_info.*: the rows from SteamVR / the status files / the configured
// values, CustomHeadsetOpenVR's IPD and steamvr.vrsettings' supersampling from temp files) and of the sound tests
// (core/sound_test.*: node picking, the WAV and the tone, the meter, the speaker and microphone sequences with a
// scripted runner and a FakeCapture). Nothing is played or recorded; every file is in a temporary directory; the
// serials are made up.
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QtEndian>
#include <deque>

#include "core/controllers.h"
#include "core/device_info.h"
#include "core/status.h"
#include "core/sound_test.h"
#include "core/state.h"
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

const InfoRow* row(const std::vector<InfoRow>& rows, const QString& key) {
    for (const InfoRow& r : rows) {
        if (r.key == key) {
            return &r;
        }
    }
    return nullptr;
}

// Answers each run() with the next scripted answer of that program (pactl / paplay), or holds it when `hold`.
struct ScriptRunner : Runner {
    struct Answer {
        int code = 0;
        QString out;
    };
    std::deque<Answer> pactl, paplay;
    std::vector<QStringList> calls; // program + args
    bool hold = false;
    std::vector<std::pair<Done, Answer>> held;
    QStringList filesSeen; // the files paplay was given, and whether they existed then
    void run(const QString& program, const QStringList& args, Done done) override {
        calls.push_back(QStringList{program} + args);
        auto& q = program == QLatin1String("pactl") ? pactl : paplay;
        Answer a = q.empty() ? Answer{} : q.front();
        if (!q.empty()) {
            q.pop_front();
        }
        if (program == QLatin1String("paplay") && !args.isEmpty()) {
            filesSeen << args.last() + (QFileInfo::exists(args.last()) ? QStringLiteral(" exists") : QStringLiteral(" missing"));
        }
        if (hold) {
            held.emplace_back(std::move(done), a);
            return;
        }
        if (done) {
            done(a.code, a.out);
        }
    }
    bool startDetached(const QString&, const QStringList&) override { return false; }
    void release() {
        auto h = std::move(held);
        held.clear();
        for (auto& [d, a] : h) {
            if (d) {
                d(a.code, a.out);
            }
        }
    }
};

const QString kSinks = QStringLiteral(
    "55\talsa_output.pci-0000_00_1f.3.analog-stereo\tPipeWire\ts32le 2ch 48000Hz\tSUSPENDED\n"
    "71\talsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-output-1\tPipeWire\ts16le 2ch 48000Hz\tIDLE\n");
const QString kSources = QStringLiteral(
    "56\talsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-output-1.monitor\tPipeWire\ts16le 2ch 48000Hz\tIDLE\n"
    "72\talsa_input.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-input-0\tPipeWire\ts16le 1ch 48000Hz\tIDLE\n");

QByteArray pcm(int samples, int16_t amplitude) {
    QByteArray b(samples * 2, '\0');
    for (int i = 0; i < samples; i++) {
        qToLittleEndian<qint16>(static_cast<qint16>(i % 2 ? amplitude : -amplitude), b.data() + 2 * i);
    }
    return b;
}

} // namespace

// ---------------------------------------------------------------- device info

TEST(device_view_live_from_steamvr) {
    DeviceInputs in = fakeDeviceInputs(true);
    const DeviceView v = deviceView(in);
    CHECK(v.title == QStringLiteral("Pimax Crystal Super"));
    CHECK(v.chip.text == QStringLiteral("STEAMVR LIVE"));
    const InfoRow* sn = row(v.headset, QStringLiteral("Headset SN"));
    CHECK(sn && sn->value == QStringLiteral("HMD-FAKE0000") && sn->copy);
    const InfoRow* ipd = row(v.headset, QStringLiteral("IPD"));
    CHECK(ipd && ipd->value == QStringLiteral("63.5 mm")); // a point in every locale (LC_ALL=de_DE in ctest)
    CHECK(ipd->note.contains(QStringLiteral("no hardware IPD")));
    CHECK(row(v.headset, QStringLiteral("Measured IPD"))->value == QStringLiteral("63.8 mm"));
    CHECK(v.controllers.size() == 2);
    CHECK(v.controllers[0].key == QStringLiteral("Left controller") && v.controllers[0].value == QStringLiteral("LHR-FAKE0001"));
    CHECK(v.controllers[0].note == QStringLiteral("Valve Index Controller") && v.controllers[0].copy);
    CHECK(v.controllers[1].key == QStringLiteral("Right controller"));
    CHECK(v.stations.size() == 2 && v.stations[1].key == QStringLiteral("Station 2"));
    CHECK(v.stations[0].value == QStringLiteral("LHB-FAKE0001") && v.stations[0].note == QStringLiteral("SteamVR Base Station 2.0"));
    CHECK(v.devicesNote.isEmpty());
    const InfoRow* res = row(v.rendering, QStringLiteral("Render resolution"));
    CHECK(res && res->value == QStringLiteral("3480 x 3480 per eye"));
    CHECK(res->note.contains(QStringLiteral("Supersampling 150 % (set by hand")));
    CHECK(row(v.rendering, QStringLiteral("Refresh rate"))->value == QStringLiteral("90 Hz"));
    CHECK(row(v.rendering, QStringLiteral("Refresh rate"))->note == QStringLiteral("from SteamVR"));
    CHECK(row(v.rendering, QStringLiteral("Quad views"))->value == QStringLiteral("On"));
    CHECK(row(v.rendering, QStringLiteral("Tracking mode"))->value == QStringLiteral("Lighthouse (SteamVR)"));
    const InfoRow* m = row(v.rendering, QStringLiteral("Monado build"));
    CHECK(m && m->value == QStringLiteral("C++ (PiBridge)") && m->note.contains(QStringLiteral("Used only by inside-out")));
}

TEST(device_view_without_steamvr_falls_back_to_the_configured_values) {
    DeviceInputs in = fakeDeviceInputs(false);
    in.gazeIpdMm = 0;
    const DeviceView v = deviceView(in);
    CHECK(v.chip.text == QStringLiteral("STEAMVR OFF"));
    const InfoRow* sn = row(v.headset, QStringLiteral("Headset SN"));
    CHECK(sn && sn->value.isEmpty() && sn->note == QStringLiteral("Start SteamVR to read it."));
    const InfoRow* ipd = row(v.headset, QStringLiteral("IPD"));
    CHECK(ipd->value == QStringLiteral("63.0 mm") && ipd->note.startsWith(QStringLiteral("Configured in CustomHeadsetOpenVR")));
    CHECK(row(v.headset, QStringLiteral("Measured IPD"))->value.isEmpty());
    CHECK(v.controllers.empty() && v.stations.empty());
    CHECK(v.devicesNote.contains(QStringLiteral("Start SteamVR")));
    CHECK(row(v.rendering, QStringLiteral("Render resolution"))->value.isEmpty());
    // the fake idle state: panel off, so the configured rate
    CHECK(row(v.rendering, QStringLiteral("Refresh rate"))->note.startsWith(QStringLiteral("set in PiBridge")));
    in.chovrIpd.reset();
    CHECK(deviceView(in).headset[2].value.isEmpty());
    in.keepalive.model.clear();
    CHECK(deviceView(in).title == QStringLiteral("No headset"));
}

TEST(device_view_modes_quad_views_and_build) {
    DeviceInputs in = fakeDeviceInputs(true);
    in.insideOut = true;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Tracking mode"))->value == QStringLiteral("Inside-out: SteamVR + Monado"));
    in.ioRunning = true;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Tracking mode"))->value.endsWith(QStringLiteral("(running)")));
    CHECK(!row(deviceView(in).rendering, QStringLiteral("Monado build"))->note.contains(QStringLiteral("Used only")));
    in.directRunning = true;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Tracking mode"))->value == QStringLiteral("Monado directly (running)"));
    in.qvEnabled = false;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Quad views"))->value == QStringLiteral("Off"));
    in.qvInstalled = false;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Quad views"))->value == QStringLiteral("Not installed"));
    in.monado.kind = BuildKind::Rust;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Monado build"))->value == QStringLiteral("Rust"));
    in.monado.missing = {QStringLiteral("PIMAX_SLAM")};
    CHECK(row(deviceView(in).rendering, QStringLiteral("Monado build"))->note.contains(QStringLiteral("PIMAX_SLAM")));
    in.monado.checked = false;
    in.monado.checking = true;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Monado build"))->value == QStringLiteral("Checking..."));
    in.monado.path.clear();
    CHECK(row(deviceView(in).rendering, QStringLiteral("Monado build"))->value == QStringLiteral("Not found"));
    // a headset without a serial or a render size
    for (VrDevice& d : in.steamvr.devices) {
        d.serial.clear();
        d.renderWidth = 0;
    }
    const DeviceView v = deviceView(in);
    CHECK(row(v.headset, QStringLiteral("Headset SN"))->note == QStringLiteral("SteamVR does not report one."));
    CHECK(!v.controllers[0].copy && v.controllers[0].note.contains(QStringLiteral("no serial")));
    CHECK(row(v.rendering, QStringLiteral("Render resolution"))->value.isEmpty());
}

TEST(chovr_ipd_and_supersampling_from_files) {
    QTemporaryDir t;
    const QString cs = t.path() + QStringLiteral("/settings.json");
    CHECK(!chovrConfiguredIpd(cs)); // missing
    writeAll(cs, "broken");
    CHECK(!chovrConfiguredIpd(cs));
    writeAll(cs, R"({"customShader":{"enable":true},"crystal":{"enable":true,"ipd":61.5},)"
                 R"("crystalSuperMicroOLED":{"enable":true,"ipd":64,"ipdOffset":0.5,"hardwareIpd":true}})");
    auto c = chovrConfiguredIpd(cs);
    CHECK(c && c->mm == 64.5 && c->hardwareIpd && c->headset == QStringLiteral("crystalSuperMicroOLED"));
    writeAll(cs, R"({"crystalSuperMicroOLED":{"enable":false,"ipd":64},"crystal":{"ipd":61.5}})");
    c = chovrConfiguredIpd(cs);
    CHECK(c && c->mm == 61.5 && !c->hardwareIpd && c->headset == QStringLiteral("crystal"));

    const QString vs = t.path() + QStringLiteral("/steamvr.vrsettings");
    CHECK(!steamVrSupersample(vs));
    writeAll(vs, R"({"steamvr":{}})");
    auto s = steamVrSupersample(vs);
    CHECK(s && s->scale == 1.0 && !s->manual);
    writeAll(vs, R"({"steamvr":{"supersampleScale":1.3,"supersampleManualOverride":true}})");
    s = steamVrSupersample(vs);
    CHECK(s && s->scale == 1.3 && s->manual);
    DeviceInputs in = fakeDeviceInputs(true);
    in.supersample = s;
    CHECK(row(deviceView(in).rendering, QStringLiteral("Render resolution"))->note.contains(QStringLiteral("130 %")));
    in.supersample = SteamVrSupersample{1.0, false};
    CHECK(row(deviceView(in).rendering, QStringLiteral("Render resolution"))->note.contains(QStringLiteral("Auto")));
}

// ---------------------------------------------------------------- sound: pieces

TEST(sound_picks_the_headset_nodes_not_monitors) {
    CHECK(pickHeadsetSink(kSinks) == QStringLiteral("alsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-output-1"));
    CHECK(pickHeadsetSource(kSources) == QStringLiteral("alsa_input.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-input-0"));
    CHECK(pickHeadsetSource(QStringLiteral("56\talsa_output.usb-Pimax_Corporation_X.monitor\tPipeWire\n")).isEmpty());
    CHECK(pickHeadsetSink(QStringLiteral("55\talsa_output.pci-0000_00_1f.3.analog-stereo\tPipeWire\n")).isEmpty());
    CHECK(pickHeadsetSink(QString()).isEmpty());
}

TEST(sound_wav_header_and_tone) {
    const std::vector<int16_t> tone = stereoTestTone(48000);
    const int perCh = 48000 * 7 / 10 * 2 + 48000 / 4;
    CHECK(tone.size() == size_t(2 * perCh));
    // left only first, right only last
    bool leftOnly = true, rightOnly = true;
    int16_t peak = 0;
    for (int i = 0; i < 48000 * 7 / 10; i++) {
        leftOnly = leftOnly && tone[size_t(2 * i + 1)] == 0;
        rightOnly = rightOnly && tone[tone.size() - size_t(2 * i) - 2] == 0;
        peak = std::max<int16_t>(peak, tone[size_t(2 * i)]);
    }
    CHECK(leftOnly && rightOnly);
    CHECK(peak > 9000 && peak <= 10000);
    CHECK(tone[0] == 0); // soft start
    const QByteArray w = wavFile(tone, 48000, 2);
    CHECK(w.size() == qsizetype(44 + tone.size() * 2));
    CHECK(w.left(4) == "RIFF" && w.mid(8, 8) == "WAVEfmt " && w.mid(36, 4) == "data");
    const auto* p = reinterpret_cast<const uchar*>(w.constData());
    CHECK(qFromLittleEndian<quint32>(p + 4) == quint32(w.size() - 8));
    CHECK(qFromLittleEndian<quint16>(p + 22) == 2);
    CHECK(qFromLittleEndian<quint32>(p + 24) == 48000);
    CHECK(qFromLittleEndian<quint32>(p + 28) == 48000 * 4);
    CHECK(qFromLittleEndian<quint16>(p + 34) == 16);
}

TEST(sound_meter_scale) {
    CHECK(meterLevel(QByteArray()) == 0.0f);
    CHECK(meterLevel(pcm(100, 0)) == 0.0f);
    CHECK(meterLevel(pcm(100, 32767)) == 1.0f);
    const float half = meterLevel(pcm(100, 1036)); // about -30 dBFS
    CHECK(half > 0.48f && half < 0.52f);
    CHECK(peakDb(pcm(100, 0)) == -96.0);
    CHECK(peakDb(pcm(100, -32768)) > -0.01); // the most negative sample counts as full scale
}

// ---------------------------------------------------------------- sound: the sequences

TEST(speaker_test_plays_on_the_headset_sink_only) {
    QTemporaryDir t;
    ScriptRunner r;
    FakeCapture cap;
    r.pactl = {{0, kSinks}};
    r.hold = true;
    SoundTest s(r, cap, t.path(), false);
    s.testSpeakers();
    CHECK(s.state() == SoundTest::State::Finding);
    r.release();
    CHECK(s.state() == SoundTest::State::Speakers && s.busy());
    s.testMicrophone(); // ignored while busy
    CHECK(r.calls.size() == 2);
    r.release();
    CHECK(s.state() == SoundTest::State::Idle && !s.error());
    CHECK(s.message().startsWith(QStringLiteral("Done.")));
    CHECK(r.calls[0].join(QLatin1Char(' ')) == QStringLiteral("pactl list short sinks"));
    CHECK(r.calls[1][0] == QStringLiteral("paplay"));
    CHECK(r.calls[1][1] == QStringLiteral("--device=alsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-output-1"));
    CHECK(r.filesSeen == QStringList{s.speakerFile() + QStringLiteral(" exists")});
    CHECK(!QFileInfo::exists(s.speakerFile())); // removed after playing
    for (const QStringList& c : r.calls) { // the default output is never touched
        CHECK(!c.join(QLatin1Char(' ')).contains(QStringLiteral("set-default")));
    }
    CHECK(cap.starts == 0);
}

TEST(speaker_test_errors_in_plain_words) {
    QTemporaryDir t;
    FakeCapture cap;
    {
        ScriptRunner r;
        r.pactl = {{0, QStringLiteral("55\talsa_output.pci-0000_00_1f.3.analog-stereo\tPipeWire\n")}};
        SoundTest s(r, cap, t.path(), false);
        s.testSpeakers();
        CHECK(s.error() && s.message().startsWith(QStringLiteral("Headset speakers not found")));
        CHECK(r.calls.size() == 1 && !s.busy());
    }
    {
        ScriptRunner r;
        r.pactl = {{-1, {}}};
        SoundTest s(r, cap, t.path(), false);
        s.testSpeakers();
        CHECK(s.error() && s.message().startsWith(QStringLiteral("pactl was not found")));
    }
    {
        ScriptRunner r;
        r.pactl = {{1, QStringLiteral("Connection failure: Connection refused\n")}};
        SoundTest s(r, cap, t.path(), false);
        s.testSpeakers();
        CHECK(s.message() == QStringLiteral("The sound server did not answer: Connection failure: Connection refused"));
    }
    {
        ScriptRunner r;
        r.pactl = {{0, kSinks}};
        r.paplay = {{1, QStringLiteral("Failed to open audio file.\n")}};
        SoundTest s(r, cap, t.path(), false);
        s.testSpeakers();
        CHECK(s.error() && s.message() == QStringLiteral("Playback failed: Failed to open audio file."));
        CHECK(!QFileInfo::exists(s.speakerFile()));
    }
    {
        ScriptRunner r;
        r.pactl = {{0, kSinks}};
        r.paplay = {{-1, {}}};
        SoundTest s(r, cap, t.path(), false);
        s.testSpeakers();
        CHECK(s.message().startsWith(QStringLiteral("paplay was not found")));
    }
}

TEST(microphone_test_records_meters_and_plays_back) {
    QTemporaryDir t;
    ScriptRunner r;
    FakeCapture cap;
    r.pactl = {{0, kSinks}, {0, kSources}};
    SoundTest s(r, cap, t.path(), false);
    std::vector<float> levels;
    QObject::connect(&s, &SoundTest::levelChanged, [&](float l) { levels.push_back(l); });
    s.testMicrophone();
    CHECK(s.state() == SoundTest::State::Recording);
    CHECK(cap.running && cap.rate == 48000);
    CHECK(cap.source == QStringLiteral("alsa_input.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-input-0"));
    CHECK(r.calls.size() == 2 && r.calls[1].join(QLatin1Char(' ')) == QStringLiteral("pactl list short sources"));
    cap.feed(pcm(4800, 1036));
    cap.feed(pcm(4800, 8000));
    CHECK(levels.size() == 2 && levels[0] < levels[1]);
    CHECK(s.level() == levels[1]);
    r.hold = true;
    s.finishRecording();
    CHECK(cap.stops == 1 && !cap.running);
    CHECK(s.state() == SoundTest::State::Playback && s.level() == 0.0f);
    CHECK(r.calls.back()[0] == QStringLiteral("paplay"));
    CHECK(r.calls.back()[1] == QStringLiteral("--device=alsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super-00.pro-output-1"));
    CHECK(r.filesSeen == QStringList{s.micFile() + QStringLiteral(" exists")});
    QFile f(s.micFile());
    CHECK(f.open(QIODevice::ReadOnly) && f.size() == 44 + 9600 * 2);
    f.close();
    cap.feed(pcm(4800, 30000)); // after the end: ignored
    r.release();
    CHECK(s.state() == SoundTest::State::Idle && !s.error());
    CHECK(s.message().startsWith(QStringLiteral("Done: you should have heard yourself")));
    CHECK(s.message().contains(QStringLiteral("-12 dB")));
    CHECK(!QFileInfo::exists(s.micFile())); // nothing of the recording is kept
}

TEST(microphone_test_errors_in_plain_words) {
    QTemporaryDir t;
    {
        ScriptRunner r;
        FakeCapture cap;
        r.pactl = {{0, kSinks}, {0, QStringLiteral("1\talsa_input.pci-0000_00_1f.3.analog-stereo\tPipeWire\n")}};
        SoundTest s(r, cap, t.path(), false);
        s.testMicrophone();
        CHECK(s.error() && s.message().startsWith(QStringLiteral("Headset microphone not found")) && cap.starts == 0);
    }
    {
        ScriptRunner r;
        FakeCapture cap;
        r.pactl = {{0, QString()}, {0, kSources}};
        SoundTest s(r, cap, t.path(), false);
        s.testMicrophone();
        CHECK(s.message().startsWith(QStringLiteral("Headset speakers not found (the recording")) && cap.starts == 0);
    }
    {
        ScriptRunner r;
        FakeCapture cap;
        cap.startOk = false;
        r.pactl = {{0, kSinks}, {0, kSources}};
        SoundTest s(r, cap, t.path(), false);
        s.testMicrophone();
        CHECK(s.error() && s.message() == QStringLiteral("parec is not installed") && !s.busy());
    }
    {
        ScriptRunner r;
        FakeCapture cap;
        r.pactl = {{0, kSinks}, {0, kSources}};
        SoundTest s(r, cap, t.path(), false);
        s.testMicrophone();
        cap.fail(QStringLiteral("Stream error: No such entity"));
        CHECK(s.error() && s.message() == QStringLiteral("Recording stopped: Stream error: No such entity") && !s.busy());
        CHECK(r.calls.size() == 2); // nothing played
    }
    {
        ScriptRunner r;
        FakeCapture cap;
        r.pactl = {{0, kSinks}, {0, kSources}};
        SoundTest s(r, cap, t.path(), false);
        s.testMicrophone();
        s.finishRecording(); // no data arrived
        CHECK(s.error() && s.message().startsWith(QStringLiteral("The headset's microphone sent no sound")));
        CHECK(r.calls.size() == 2);
    }
    {
        ScriptRunner r;
        FakeCapture cap;
        r.pactl = {{0, kSinks}, {0, kSources}};
        SoundTest s(r, cap, t.path(), false);
        s.testMicrophone();
        cap.feed(pcm(48000, 100)); // about -50 dBFS
        s.finishRecording();
        CHECK(s.error() && s.message().contains(QStringLiteral("very quiet")) && !s.busy());
    }
}

// ------------------------------------------------------------------------------------------------ controllers

namespace {

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

KeepaliveStatus liveStatus() {
    KeepaliveStatus k;
    k.running = true;
    k.state = QStringLiteral("idle");
    k.headsetPresent = true;
    return k;
}

} // namespace

TEST(keepalive_status_carries_the_controller_fields) {
    const auto k = parseKeepalive(R"({"state":"idle","ctrl_id":12,"ctrl_op":"pair","ctrl_side":"left",
        "ctrl_state":"waiting","ctrl_detail":"Press and hold","left_connected":true,"right_connected":false,
        "haptic_seq":3,"haptic_result":"Vibration sent to the left controller."})");
    CHECK(k && k->ctrlId == 12 && k->ctrlOp == QLatin1String("pair") && k->ctrlSide == QLatin1String("left"));
    CHECK(k->ctrlState == QLatin1String("waiting") && k->ctrlDetail == QLatin1String("Press and hold"));
    CHECK(k->leftConnected && !k->rightConnected && k->hapticSeq == 3);
}

TEST(pair_writes_one_request_and_follows_the_answer) {
    QTemporaryDir d;
    ControllerOps ops(d.path() + QStringLiteral("/pibridge"), d.path());
    QDir().mkpath(d.path() + QStringLiteral("/pibridge"));
    KeepaliveStatus k = liveStatus();
    CHECK(ops.pair(kSlotLeft, k, 1000).isEmpty());
    CHECK(ops.busy() && ops.side(kSlotLeft).busy);
    CHECK(readAll(ops.requestPath()) == "1000 pair left\n");
    // one action at a time
    CHECK(!ops.testVibration(kSlotRight, k, 1100).isEmpty());
    k.ctrlId = 1000;
    k.ctrlState = QStringLiteral("waiting");
    k.ctrlDetail = QStringLiteral("Press and hold the controller's pairing buttons.");
    ops.poll(k, 6000);
    CHECK(ops.side(kSlotLeft).text.contains(QStringLiteral("30 s left")) && !ops.side(kSlotLeft).error);
    ops.poll(k, 44000); // still waiting past the ack time: fine
    CHECK(ops.busy());
    k.ctrlState = QStringLiteral("paired");
    k.ctrlDetail = QStringLiteral("The left controller is paired.");
    ops.poll(k, 20000);
    CHECK(!ops.busy() && !ops.side(kSlotLeft).error);
    CHECK(ops.side(kSlotLeft).text == QLatin1String("The left controller is paired."));
    // a timeout from the service is an error in plain words
    CHECK(ops.pair(kSlotRight, k, 30000).isEmpty());
    k.ctrlId = 30000;
    k.ctrlState = QStringLiteral("timeout");
    k.ctrlDetail = QStringLiteral("No controller paired within 30 s.");
    ops.poll(k, 31000);
    CHECK(!ops.busy() && ops.side(kSlotRight).error);
}

TEST(controller_actions_are_refused_in_plain_words_before_anything_is_written) {
    QTemporaryDir d;
    QDir().mkpath(d.path() + QStringLiteral("/pibridge"));
    ControllerOps ops(d.path() + QStringLiteral("/pibridge"), d.path());
    KeepaliveStatus off;
    CHECK(ops.pair(kSlotLeft, off, 1).contains(QStringLiteral("not running")));
    KeepaliveStatus k = liveStatus();
    k.headsetPresent = false;
    CHECK(ops.unpair(kSlotLeft, k, 1).contains(QStringLiteral("No headset")));
    k = liveStatus();
    CHECK(ops.testVibration(kSlotRight, k, 1).contains(QStringLiteral("right controller is not connected")));
    k.leftConnected = true;
    CHECK(ops.pair(kSlotLeft, k, 1).contains(QStringLiteral("already paired")));
    k.state = QStringLiteral("paused");
    CHECK(ops.pair(kSlotRight, k, 1).contains(QStringLiteral("Inside-out tracking")));
    CHECK(ops.unpair(kSlotRight, k, 1).contains(QStringLiteral("Stop tracking first")));
    CHECK(!QFile::exists(ops.requestPath()) && !QFile::exists(ops.hapticPath()));
    CHECK(ops.side(kSlotRight).error && !ops.busy());
}

TEST(unpair_without_an_answer_times_out_and_removes_the_request) {
    QTemporaryDir d;
    QDir().mkpath(d.path() + QStringLiteral("/pibridge"));
    ControllerOps ops(d.path() + QStringLiteral("/pibridge"), d.path());
    const KeepaliveStatus k = liveStatus();
    CHECK(ops.unpair(kSlotRight, k, 500).isEmpty());
    CHECK(readAll(ops.requestPath()) == "500 unpair right\n");
    ops.poll(k, 2000);
    CHECK(ops.busy());
    ops.poll(k, 4000);
    CHECK(!ops.busy() && ops.side(kSlotRight).error && !QFile::exists(ops.requestPath()));
    CHECK(ops.side(kSlotRight).text.contains(QStringLiteral("did not take the request")));
}

TEST(test_vibration_goes_to_keepalived_or_to_the_vr_session) {
    QTemporaryDir d;
    QDir().mkpath(d.path() + QStringLiteral("/pibridge"));
    ControllerOps ops(d.path() + QStringLiteral("/pibridge"), d.path());
    KeepaliveStatus k = liveStatus();
    k.leftConnected = true;
    k.hapticSeq = 4;
    CHECK(ops.testVibration(kSlotLeft, k, 100).isEmpty());
    CHECK(readAll(ops.hapticPath()) == "2 60 300\n");
    k.hapticSeq = 5;
    k.hapticResult = QStringLiteral("Vibration sent to the left controller.");
    ops.poll(k, 400);
    CHECK(!ops.busy() && !ops.side(kSlotLeft).error);
    // inside-out session: Monado's driver takes the file (no connected flag needed)
    QFile::remove(ops.hapticPath());
    k.state = QStringLiteral("paused");
    k.rightConnected = false;
    CHECK(ops.testVibration(kSlotRight, k, 1000).isEmpty());
    CHECK(readAll(ops.hapticPath()) == "3 60 300\n");
    ops.poll(k, 1200);
    CHECK(ops.busy());
    QFile::remove(ops.hapticPath()); // taken
    ops.poll(k, 1400);
    CHECK(!ops.busy() && ops.side(kSlotRight).text.contains(QStringLiteral("VR session")));
    // nobody takes it: removed again, plain error
    CHECK(ops.testVibration(kSlotRight, k, 5000).isEmpty());
    ops.poll(k, 8100);
    CHECK(!ops.busy() && ops.side(kSlotRight).error && !QFile::exists(ops.hapticPath()));
}

PIBRIDGE_TEST_MAIN()
