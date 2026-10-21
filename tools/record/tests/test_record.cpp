// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record (the unit tests of settings.rs, cli.rs, encode.rs, shm.rs, status.rs, run.rs
// and tests/offline.rs; MIT OR Apache-2.0).
// Twins of the Rust tests (same cases, same expected values), the ring against the real comp_recorder_shm.h, and the
// whole recorder: the pibridge-record binary with an in-process fake producer (what Monado's tap does) and a fake
// ffmpeg (tests/fake_ffmpeg.sh). Nothing touches a GPU, a VR session or the sound server.
#include "pibridge/record/cli.h"
#include "pibridge/record/encode.h"
#include "pibridge/record/mirror.h"
#include "pibridge/record/proc.h"
#include "pibridge/record/run.h"
#include "pibridge/record/settings.h"
#include "pibridge/record/shm.h"
#include "pibridge/record/status.h"
#include "pibridge_test.h"

#include <algorithm>
#include <deque>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

#if __has_include(PIBRIDGE_MONADO_SHM_HEADER)
#define PIBRIDGE_HAVE_MONADO_SHM 1
#define _Static_assert static_assert
#include PIBRIDGE_MONADO_SHM_HEADER
#undef _Static_assert
#endif

using namespace pibridge::record;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {

struct TempDir {
    fs::path p;
    explicit TempDir(const std::string& name) {
        p = fs::temp_directory_path() / ("pibridge-record-test-" + std::to_string(getpid()) + "-" + name);
        fs::remove_all(p);
        fs::create_directories(p);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(p, ec);
    }
};

std::string readAll(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

using U2 = std::pair<std::uint32_t, std::uint32_t>;

bool contains(const std::string& h, const std::string& n) { return h.find(n) != std::string::npos; }

std::string join(const std::vector<std::string>& v) {
    std::string s;
    for (const auto& x : v) {
        s += (s.empty() ? "" : " ") + x;
    }
    return s;
}

std::string after(const std::vector<std::string>& v, const std::string& k) {
    const auto it = std::ranges::find(v, k);
    return it == v.end() || it + 1 == v.end() ? std::string() : *(it + 1);
}

Roots rootsUnder(const fs::path& base) {
    Roots r;
    r.home = base / "home";
    r.session = base / "session";
    r.runtime = base / "run";
    r.config = base / "home/.config/pibridge";
    r.legacyConfig = base / "home/.config/omarchyvr";
    r.proc = base / "proc";
    return r;
}

} // namespace

// ---------------------------------------------------------------- settings.rs

TEST(defaults_are_one_eye_30_fps_high_with_sound) {
    const Settings s;
    CHECK(s.eyes == Eyes::Left && s.fps == 30 && s.quality == Quality::High && s.codec == Codec::H264 &&
          s.encoder == EncoderPref::Auto);
    CHECK(s.audio && !s.mic);
    CHECK(s.request() == U2(1080, 0));
}

TEST(round_trips_through_the_file) {
    Settings s;
    CHECK(s.set("eyes", "both").has_value());
    CHECK(s.set("fps", "72").has_value());
    CHECK(s.set("quality", "high").has_value());
    CHECK(s.set("scale", "0.5").has_value());
    CHECK(s.set("max-height", "1440").has_value());
    CHECK(s.set("codec", "hevc").has_value());
    CHECK(s.set("encoder", "cpu").has_value());
    CHECK(s.set("dir", "/data/vr").has_value());
    CHECK(s.set("audio", "off").has_value());
    CHECK(s.set("mic", "on").has_value());
    CHECK(Settings::parse(s.toToml()).value() == s);
    CHECK(s.request() == U2(1440, 500));
}

TEST(rejects_bad_values_and_unknown_keys) {
    Settings s;
    CHECK(!s.set("fps", "45"));
    CHECK(!s.set("fps", "x"));
    CHECK(!s.set("eyes", "three"));
    CHECK(!s.set("quality", "ultra"));
    CHECK(!s.set("scale", "1.5"));
    CHECK(!s.set("scale", "0.05"));
    CHECK(!s.set("max_height", "100"));
    CHECK(!s.set("colour", "red"));
    CHECK(s == Settings{}); // a rejected value changes nothing
    const auto e = Settings::parse("fps = 30\nfsp = 60\n");
    CHECK(!e && contains(e.error(), "line 2"));
    CHECK(s.set("fps", "61").error() == "fps = 61: expected 30, 60 or 72");
    CHECK(s.set("colour", "red").error() == "unknown setting colour");
}

TEST(an_old_file_without_audio_takes_the_new_defaults_but_a_new_one_keeps_its_choice) {
    const auto old = Settings::parse("eyes = \"left\"\nfps = 30\nquality = \"medium\"\n").value();
    CHECK(old.quality == Quality::High && old.audio);
    CHECK(Settings::parse("quality = \"low\"\n").value().quality == Quality::Low);
    const auto chosen = Settings::parse("quality = \"medium\"\naudio = \"off\"\n").value();
    CHECK(chosen.quality == Quality::Medium && !chosen.audio);
    CHECK(Settings::parse("").value() == Settings{});
}

TEST(parses_comments_quotes_and_sections) {
    const auto s = Settings::parse("# hi\n[record]\neyes = \"both\" # side by side\n\nfps=60\n").value();
    CHECK(s.eyes == Eyes::Both && s.fps == 60);
}

TEST(missing_file_means_defaults_and_save_is_atomic) {
    TempDir d("settings");
    const fs::path p = settingsPath(d.p / "cfg");
    CHECK(Settings::load(p).value() == Settings{});
    Settings s;
    s.fps = 60;
    CHECK(s.save(p).empty());
    CHECK(Settings::load(p).value() == s);
    CHECK(!fs::exists(fs::path(p).replace_extension("toml.tmp")));
    std::ofstream(p) << "fps = 61\n";
    CHECK(!Settings::load(p));
}

TEST(pibridge_reads_omarchyvr_settings_until_it_has_its_own) {
    TempDir d("legacy");
    const fs::path mine = d.p / "pibridge", old = d.p / "omarchyvr";
    CHECK(settingsReadPath(mine, old) == settingsPath(mine));
    fs::create_directories(old);
    std::ofstream(settingsPath(old)) << "fps = 60\n";
    CHECK(settingsReadPath(mine, old) == settingsPath(old));
    Settings{}.save(settingsPath(mine));
    CHECK(settingsReadPath(mine, old) == settingsPath(mine));
}

TEST(expands_the_home_dir_and_names_files_by_time) {
    const Settings s;
    CHECK(s.outDir("/home/u") == fs::path("/home/u/Videos/pimax"));
    const std::string n = fileName(1'792'032'882);
    CHECK(n.starts_with("pimax-20") && n.ends_with(".mp4") && n.size() == std::string("pimax-20261015-123456.mp4").size());
}

TEST(args_round_trip) {
    Settings s;
    s.eyes = Eyes::Both;
    s.fps = 60;
    s.scale = 0.75f;
    const auto a = s.toArgs();
    Settings t;
    for (std::size_t i = 0; i + 1 < a.size(); i += 2) {
        CHECK(t.set(a[i].substr(2), a[i + 1]).has_value());
    }
    CHECK(t == s);
    CHECK(formatF32(0.0f) == "0" && formatF32(0.1f) == "0.1" && formatF32(0.75f) == "0.75" && formatF32(1.0f) == "1");
}

// ---------------------------------------------------------------- cli.rs

namespace {
std::vector<std::string> A(std::initializer_list<const char*> v) { return {v.begin(), v.end()}; }
} // namespace

TEST(parses_the_commands) {
    CHECK(parse({}).value().kind == Cmd::Kind::Help);
    CHECK(parse(A({"stop"})).value().kind == Cmd::Kind::Stop);
    CHECK(parse(A({"status"})).value().kind == Cmd::Kind::Status);
    const auto c = parse(A({"start", "--eyes", "both", "--fps=60", "--quality", "high", "--save"})).value();
    CHECK(c.kind == Cmd::Kind::Start && c.save);
    CHECK(c.overrides == Overrides({{"eyes", "both"}, {"fps", "60"}, {"quality", "high"}}));
    CHECK(parse(A({"run", "--max-height", "720"})).value().kind == Cmd::Kind::Run);
}

TEST(rejects_bad_input_before_spawning_anything) {
    CHECK(!parse(A({"start", "--fps", "45"})));
    CHECK(!parse(A({"start", "--eyes"})));
    CHECK(!parse(A({"start", "--colour", "red"})));
    CHECK(!parse(A({"start", "now"})));
    CHECK(!parse(A({"stop", "--fps", "30"})));
    CHECK(!parse(A({"record"})));
    CHECK(parse(A({"stop", "--fps", "30"})).error() == "stop takes no options");
    CHECK(parse(A({"start", "--eyes"})).error() == "--eyes needs a value");
}

TEST(flags_override_the_file) {
    TempDir d("cli-eff");
    const Roots r = rootsUnder(d.p);
    fs::create_directories(r.config);
    std::ofstream(settingsPath(r.config)) << "eyes = \"both\"\nfps = 60\n";
    const auto s = effective(r, {{"fps", "72"}}).value();
    CHECK(s.eyes == Eyes::Both && s.fps == 72);
}

TEST(the_spawn_spec_carries_the_settings_for_a_respawn) {
    Settings s;
    s.fps = 60;
    const auto args = runArgs(s);
    CHECK(args[0] == "run");
    CHECK(after(args, "--fps") == "60");
    CHECK(commOf("/b/pibridge-record") == kComm);
    const auto c = parse(args).value();
    CHECK(c.kind == Cmd::Kind::Run);
    Settings t;
    for (const auto& [k, v] : c.overrides) {
        CHECK(t.set(k, v).has_value());
    }
    CHECK(t == s);
    CHECK(after(runArgs(s, Source::Mirror), "--source") == "mirror" && c.source == "tap");
    CHECK(parse(runArgs(s, Source::Mirror)).value().source == "mirror");
    CHECK(!parse(A({"start", "--source", "screen"})));
    CHECK(!parse(A({"status", "--source", "tap"})));
}

TEST(brief_commands_are_start_and_stop_with_the_saved_settings) {
    const auto a = parse(A({"start", "--brief"})).value();
    CHECK(a.kind == Cmd::Kind::Brief && !a.stop);
    const auto b = parse(A({"stop", "--brief"})).value();
    CHECK(b.kind == Cmd::Kind::Brief && b.stop);
    CHECK(!parse(A({"status", "--brief"})));
}

TEST(audio_flags_reach_the_recorder) {
    const auto c = parse(A({"start", "--audio", "off", "--mic=on"})).value();
    CHECK(c.kind == Cmd::Kind::Start && !c.save && c.overrides == Overrides({{"audio", "off"}, {"mic", "on"}}));
    CHECK(!parse(A({"start", "--audio", "maybe"})));
}

TEST(status_without_a_recorder_says_so) {
    TempDir d("cli-status");
    const Roots r = rootsUnder(d.p);
    fs::create_directories(r.session);
    CHECK(statusLine(r) == "not recording");
    Status st;
    st.state = "recording";
    st.file = "/v/a.mp4";
    st.pid = 999'999;
    st.save(statusPath(r.session));
    CHECK(contains(statusLine(r), "exited without finishing"));
    CHECK(stop(r).value().starts_with("not recording"));
    CHECK(stopBrief(r).value() == "Not recording");
}

TEST(a_pid_with_another_comm_is_never_ours) {
    TempDir d("cli-comm");
    Roots r = rootsUnder(d.p);
    fs::create_directories(r.session);
    fs::create_directories(r.proc / "4242");
    std::ofstream(r.proc / "4242/stat") << "4242 (vrserver) S 1 2 3\n";
    std::ofstream(r.proc / "4242/comm") << "vrserver\n";
    std::ofstream(pidFile(r.session)) << "4242\n";
    CHECK(!runningPid(r));
    CHECK(steamvrRunning(r) && !monadoRunning(r));
    // the tap under SteamVR + driver_monado: refused in plain words (SteamVR's compositor draws the image)
    const auto e = start(r, {}, false, "tap");
    CHECK(!e && contains(e.error(), "SteamVR runs") && contains(e.error(), "--source layer"));
    // auto picks the Vulkan capture layer while vrserver runs (pickSource only: nothing is started here)
    CHECK(pickSource(r, "auto") == Source::Layer && pickSource(r, "layer") == Source::Layer);
    CHECK(pickSource(r, "mirror") == Source::Mirror); // the GL mirror is still available on request
    std::ofstream(r.proc / "4242/comm") << "pibridge-record\n";
    CHECK(runningPid(r) == std::optional<std::uint32_t>(4242));
    std::ofstream(r.proc / "4242/stat") << "4242 (pibridge-record) Z 1 2 3\n";
    CHECK(!runningPid(r)); // a zombie is gone
}

TEST(start_without_a_session_says_where_to_start_one) {
    TempDir d("cli-nosession");
    Roots r = rootsUnder(d.p);
    fs::create_directories(r.proc);
    const auto e = start(r, {{"fps", "60"}}, true);
    CHECK(!e && e.error() == "no VR session: start one first (\"Monado directly\" on the Development page, or SteamVR)");
    CHECK(!pickSource(r, "mirror") && pickSource(r, "mirror").error() == "SteamVR is not running: its mirror cannot be recorded");
    // monado-service alone: the tap
    fs::create_directories(r.proc / "77");
    std::ofstream(r.proc / "77/stat") << "77 (monado-service) S 1\n";
    std::ofstream(r.proc / "77/comm") << "monado-service\n";
    CHECK(pickSource(r, "auto") == Source::Tap && pickSource(r, "tap") == Source::Tap);
    // --save wrote the settings before the refusal (as omarchyvr does)
    CHECK(Settings::load(settingsPath(r.config)).value().fps == 60);
}

// ---------------------------------------------------------------- encode.rs

namespace {
const char* kEncoders = " V....D h264_nvenc           NVIDIA NVENC H.264 encoder (codec h264)\n V....D libx264              "
                        "libx264 H.264\n V....D hevc_nvenc  x\n";
}

TEST(parses_the_encoder_list) {
    CHECK(Available::parse(kEncoders) == (Available{true, true, true, false}));
}

TEST(auto_prefers_nvenc_with_a_cpu_fallback) {
    const auto a = Available::parse(kEncoders);
    using V = std::vector<Encoder>;
    CHECK(plan(EncoderPref::Auto, Codec::H264, a, true).value() == (V{Encoder::Nvenc, Encoder::Cpu}));
    CHECK(plan(EncoderPref::Auto, Codec::H264, a, false).value() == (V{Encoder::Cpu}));
    CHECK(!plan(EncoderPref::Nvenc, Codec::H264, a, false));
    CHECK(plan(EncoderPref::Cpu, Codec::H264, a, true).value() == (V{Encoder::Cpu}));
    CHECK(plan(EncoderPref::Auto, Codec::Hevc, a, true).value() == (V{Encoder::Nvenc}));
    CHECK(!plan(EncoderPref::Cpu, Codec::Hevc, a, true));
    CHECK(!plan(EncoderPref::Auto, Codec::H264, Available{}, true));
    CHECK(plan(EncoderPref::Auto, Codec::H264, Available{}, true).error() == "ffmpeg has neither h264_nvenc nor libx264");
}

TEST(quality_maps_to_cq_and_crf) {
    const auto cq = [](Quality q) { return qualityArgs(Encoder::Nvenc, Codec::H264, q); };
    CHECK(after(cq(Quality::Low), "-cq") == "30");
    CHECK(after(cq(Quality::Medium), "-cq") == "25");
    CHECK(after(cq(Quality::High), "-cq") == "20");
    CHECK(after(qualityArgs(Encoder::Cpu, Codec::H264, Quality::Medium), "-crf") == "23");
    CHECK(after(qualityArgs(Encoder::Cpu, Codec::Hevc, Quality::High), "-crf") == "22");
}

TEST(the_command_line_writes_a_fragmented_mp4_from_raw_rgba) {
    Settings s;
    s.fps = 60;
    const std::string j = join(ffmpegArgs(Encoder::Nvenc, s, 1168, 1080, "/v/x.mp4"));
    CHECK(contains(j, "-f rawvideo -pix_fmt rgba -video_size 1168x1080 -framerate 60 -i pipe:0"));
    CHECK(contains(j, "-c:v h264_nvenc"));
    CHECK(contains(j, "-movflags +frag_keyframe+empty_moov+default_base_moof -f mp4 /v/x.mp4"));
    CHECK(contains(j, "-g 120"));
    CHECK(contains(j, "-pix_fmt yuv420p"));
    CHECK(contains(j, "-f pulse -i @DEFAULT_MONITOR@"));
    CHECK(contains(j, "-map 0:v:0 -map 1:a:0 -c:a aac -b:a 160k -ar 48000 -ac 2 -shortest"));
    CHECK(!contains(j, "-an"));
}

TEST(audio_off_and_mic_on_change_the_inputs_and_the_mapping) {
    Settings off;
    off.audio = false;
    const std::string o = join(ffmpegArgs(Encoder::Cpu, off, 640, 480, "/v/x.mp4"));
    CHECK(!contains(o, "pulse") && contains(o, "-map 0:v:0 -an"));
    Settings mic;
    mic.mic = true;
    const std::string m = join(ffmpegArgs(Encoder::Cpu, mic, 640, 480, "/v/x.mp4"));
    CHECK(contains(m, "-i @DEFAULT_MONITOR@") && contains(m, "-i @DEFAULT_SOURCE@"));
    CHECK(contains(m, "amix=inputs=2") && contains(m, "-map [a]"));
    Settings onlyMic;
    onlyMic.audio = false;
    onlyMic.mic = true;
    CHECK(audioInputs(onlyMic).empty());
}

TEST(the_default_high_quality_is_cq_20_with_a_50m_ceiling) {
    const std::string q = join(qualityArgs(Encoder::Nvenc, Codec::H264, Settings{}.quality));
    CHECK(contains(q, "-cq 20") && contains(q, "-maxrate 50M"));
}

// ---------------------------------------------------------------- shm.rs

TEST(layout_matches_the_c_header) {
    CHECK(offsetof(ShmHeader, consumer_heartbeat_ns) == 48);
    CHECK(offsetof(ShmHeader, stop) == 56);
    CHECK(offsetof(ShmHeader, producer_pid) == 60);
    CHECK(offsetof(ShmHeader, producer_heartbeat_ns) == 64);
    CHECK(offsetof(ShmHeader, width) == 72);
    CHECK(offsetof(ShmHeader, produced) == 88);
    CHECK(offsetof(ShmHeader, skipped) == 112);
    CHECK(offsetof(ShmHeader, producer_note) == 120);
    CHECK(offsetof(ShmHeader, slot) == 184);
    CHECK(sizeof(ShmSlot) == 32);
    CHECK(sizeof(ShmHeader) <= kHeaderSize);
    char m[8];
    std::memcpy(m, &kMagic, 8);
    CHECK(std::string(m, 8) == "PMAXREC1");
#ifdef PIBRIDGE_HAVE_MONADO_SHM
    // the fork's own header, field for field
    CHECK(sizeof(pimax_rec_header) == sizeof(ShmHeader) && sizeof(pimax_rec_slot) == sizeof(ShmSlot));
    CHECK(PIMAX_REC_MAGIC == kMagic && PIMAX_REC_VERSION == kVersion && PIMAX_REC_MAX_SLOTS == kMaxSlots &&
          PIMAX_REC_HEADER_SIZE == kHeaderSize);
#define SAME(f) CHECK(offsetof(pimax_rec_header, f) == offsetof(ShmHeader, f))
    SAME(magic); SAME(version); SAME(header_size); SAME(eyes); SAME(fps); SAME(max_height); SAME(scale_permille);
    SAME(slot_count); SAME(slot_size); SAME(consumer_heartbeat_ns); SAME(stop); SAME(producer_pid);
    SAME(producer_heartbeat_ns); SAME(width); SAME(height); SAME(src_width); SAME(src_height); SAME(produced);
    SAME(dropped_busy); SAME(dropped_full); SAME(skipped); SAME(producer_note);
    CHECK(offsetof(pimax_rec_header, slots) == offsetof(ShmHeader, slot));
#undef SAME
    CHECK(offsetof(pimax_rec_slot, state) == offsetof(ShmSlot, state) && offsetof(pimax_rec_slot, seq) == offsetof(ShmSlot, seq) &&
          offsetof(pimax_rec_slot, timestamp_ns) == offsetof(ShmSlot, timestamp_ns));
    CHECK(PIMAX_REC_SLOT_READY == kSlotReady && PIMAX_REC_SLOT_READING == kSlotReading);
    std::printf("     (checked against %s)\n", PIBRIDGE_MONADO_SHM_HEADER);
#else
    std::printf("     (the fork's comp_recorder_shm.h was not found: the static_asserts only)\n");
#endif
}

TEST(frames_flow_in_order_and_slots_are_recycled) {
    TempDir d("flow");
    const fs::path p = d.p / "ring.shm";
    auto c = Ring::create(p, Request::make(1, 30, 64, 0)).value();
    auto prod = Ring::open(p).value();
    CHECK(c.producerPid() == 0);
    prod.attachProducer(64, 64, "fake", static_cast<std::uint32_t>(getpid()));
    CHECK(c.producerPid() == static_cast<std::uint32_t>(getpid()));
    CHECK(c.size() == U2(64, 64));
    CHECK(c.producerNote() == "fake");
    for (std::uint64_t seq = 0; seq < 3; seq++) {
        CHECK(prod.publish(64, 64, seq, 1000 + seq, [&](std::span<std::uint8_t> b) { std::ranges::fill(b, std::uint8_t(seq)); }));
    }
    for (std::uint64_t seq = 0; seq < 3; seq++) {
        auto f = c.take();
        CHECK(f && f->seq == seq && f->timestamp_ns == 1000 + seq);
        CHECK(f && std::ranges::all_of(f->pixels(), [&](std::uint8_t b) { return b == std::uint8_t(seq); }));
    }
    CHECK(!c.take());
    for (std::uint64_t seq = 3; seq < 8; seq++) {
        prod.publish(64, 64, seq, seq, [](std::span<std::uint8_t>) {});
    }
    const Counters k = c.counters();
    CHECK(k.produced == 7 && k.dropped_full == 1);
    prod.detachProducer();
    c.requestStop();
    CHECK(prod.stopRequested());
    {
        Ring gone = std::move(c);
    }
    CHECK(!fs::exists(p)); // the consumer removes the file
}

TEST(a_live_ring_is_not_taken_over_but_a_stale_one_is) {
    TempDir d("live");
    const fs::path p = d.p / "ring.shm";
    auto a = Ring::create(p, Request::make(1, 30, 64, 0)).value();
    const auto second = Ring::create(p, Request::make(1, 30, 64, 0));
    CHECK(!second && second.error() == "another recorder is running");
    a.setConsumerHeartbeat(1); // a dead consumer
    a.forget();                // as if it crashed: the file stays
    const auto b = Ring::create(p, Request::make(1, 30, 64, 0));
    CHECK(b && b->hdr().magic == kMagic);
}

TEST(eye_extent_matches_monado) {
    using P = std::pair<std::uint32_t, std::uint32_t>;
    const Request req = Request::make(1, 30, 1080, 0);
    CHECK(pickEyeExtent({3840, 3552}, 1080, 0, 1, req.slot_size) == std::optional<P>(P{1168, 1080}));
    CHECK(pickEyeExtent({2000, 2000}, 1080, 500, 1, req.slot_size) == std::optional<P>(P{1000, 1000}));
    const Request two = Request::make(2, 30, 1080, 0);
    CHECK(pickEyeExtent({3840, 3552}, 1080, 0, 2, two.slot_size) == std::optional<P>(P{1168, 1080}));
    const auto wide = pickEyeExtent({4000, 2000}, 1080, 0, 1, req.slot_size).value();
    CHECK(std::uint64_t(wide.first) * wide.second * 4 <= req.slot_size && wide.second < 1080);
    const auto big = pickEyeExtent({3840, 3552}, 2160, 0, 2, Request::make(2, 30, 2160, 0).slot_size).value();
    CHECK(big.first * 2 <= 4096);
}

// ---------------------------------------------------------------- status.rs

TEST(round_trips_and_summarises) {
    Status s;
    s.state = "recording";
    s.pid = 42;
    s.file = "/v/pimax-1.mp4";
    s.eyes = "left";
    s.fps = 30;
    s.quality = "medium";
    s.encoder = "h264_nvenc";
    s.width = 1168;
    s.height = 1080;
    s.frames = 90;
    s.note = "a\nb";
    Status back = Status::parse(s.toText());
    CHECK(back.note == "a b");
    Status want = s;
    want.note = "a b";
    CHECK(want == back);
    CHECK(back.summary().starts_with("recording 0:03 to /v/pimax-1.mp4 (1 eye(s), 1168x1080 at 30 fps"));
    Status idle = back;
    idle.state = "idle";
    idle.note = "Monado stopped";
    CHECK(contains(idle.summary(), "last: /v/pimax-1.mp4 0:03, Monado stopped"));
}

TEST(events_are_json_lines) {
    CHECK(eventLine("notice", 1.0, "saved \"x\"\n") == R"({"event":"notice","t":1.0,"id":"record","text":"saved \"x\"\n"})");
}

// ---------------------------------------------------------------- run.rs

TEST(steady_frames_are_written_once) {
    constexpr std::uint64_t MS = 1'000'000;
    Pacer p(30);
    for (std::uint64_t i = 0; i < 10; i++) {
        CHECK(p.copies(1000 * MS + i * 33'333'333) == 1);
    }
    CHECK(p.duplicated == 0);
}

TEST(a_dropped_frame_is_filled_by_repeating_the_next) {
    constexpr std::uint64_t MS = 1'000'000;
    Pacer p(30);
    CHECK(p.copies(0) == 1);
    CHECK(p.copies(33 * MS) == 1);
    CHECK(p.copies(100 * MS) == 2);
    CHECK(p.duplicated == 1);
    CHECK(p.copies(105 * MS) == 0);
    CHECK(p.early == 1);
}

TEST(a_long_gap_is_cut_not_filled) {
    constexpr std::uint64_t MS = 1'000'000;
    Pacer p(30);
    p.copies(0);
    CHECK(p.copies(10'000 * MS) == Pacer::kMaxCopies);
    CHECK_NEAR(p.lost_s, (300.0 - 3.0) / 30.0, 0.05);
    CHECK(p.copies(10'033 * MS) == 1);
}

TEST(unique_names) {
    TempDir d("uniq");
    const fs::path a = uniquePath(d.p, "pimax-x.mp4");
    std::ofstream(a) << "";
    CHECK(uniquePath(d.p, "pimax-x.mp4") == d.p / "pimax-x-2.mp4");
}


// ---------------------------------------------------------------- the SteamVR mirror source (mirror.h)

namespace {

// What the OpenVR + GL backend does, without SteamVR or a GPU: frames on demand, readbacks that finish at once
// (or never: `busy`), a quit after `quitAfter` frames.
struct FakeMirror : MirrorBackend {
    std::string initError, configError;
    std::uint32_t eyesAsked = 0, cfgEyes = 0, cfgW = 0, cfgH = 0;
    bool busy = false, noImage = false;
    long failAfter = -1, quitAfter = -1;
    long submitted = 0;
    std::deque<std::pair<std::uint64_t, std::uint64_t>> pending;
    std::vector<std::uint8_t> buf;
    std::string init(std::uint32_t eyes) override {
        eyesAsked = eyes;
        return initError;
    }
    std::pair<std::uint32_t, std::uint32_t> eyeSize() override { return {3840, 3552}; }
    std::string configure(std::uint32_t eyes, std::uint32_t w, std::uint32_t h) override {
        cfgEyes = eyes;
        cfgW = w;
        cfgH = h;
        buf.assign(std::size_t(w) * eyes * h * 4, 9);
        return configError;
    }
    Submit submit(std::uint64_t seq, std::uint64_t ts, std::string& err) override {
        if (failAfter >= 0 && submitted >= failAfter) {
            err = "OpenGL error 0x505 while reading SteamVR's mirror texture";
            return Submit::Failed;
        }
        if (busy) {
            return Submit::Busy;
        }
        if (noImage) {
            return Submit::NoImage;
        }
        submitted++;
        pending.emplace_back(seq, ts);
        return Submit::Queued;
    }
    std::optional<MirrorFrame> poll() override {
        if (pending.empty()) {
            return std::nullopt;
        }
        return MirrorFrame{pending.front().first, pending.front().second, buf};
    }
    void release() override { pending.pop_front(); }
    bool quitRequested() override { return quitAfter >= 0 && submitted >= quitAfter; }
    std::string note() const override { return "fake mirror"; }
};

struct MirrorRing {
    TempDir d;
    Ring consumer;
    std::atomic<bool> quit{false};
    std::jthread hb;
    MirrorRing(const std::string& n, std::uint32_t eyes, std::uint32_t fps)
        : d("mirror-" + n), consumer(Ring::create(d.p / "ring.shm", Request::make(eyes, fps, 240, 0)).value()) {
        hb = std::jthread([this] {
            while (!quit) {
                consumer.heartbeat();
                std::this_thread::sleep_for(50ms);
            }
        });
    }
    ~MirrorRing() {
        quit = true;
    }
};

} // namespace

TEST(the_mirror_feeds_the_ring_at_the_requested_rate_and_size) {
    MirrorRing m("rate", 2, 60);
    FakeMirror fake;
    fake.quitAfter = 30;
    std::atomic<bool> stop{false};
    std::uint64_t seen = 0, lastSeq = 0;
    bool ordered = true;
    std::jthread reader([&](std::stop_token st) {
        while (!st.stop_requested()) {
            if (auto f = m.consumer.take()) {
                ordered = ordered && (seen == 0 || f->seq > lastSeq);
                lastSeq = f->seq;
                seen++;
                CHECK(f->width == 516 && f->height == 240 && f->pixels()[0] == 9);
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    });
    const auto t0 = std::chrono::steady_clock::now();
    const MirrorResult r = runMirror(m.consumer, fake, stop);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::this_thread::sleep_for(50ms);
    reader.request_stop();
    CHECK(r.code == 0 && r.reason == "SteamVR is closing");
    CHECK(fake.eyesAsked == 2 && fake.cfgEyes == 2 && fake.cfgW == 258 && fake.cfgH == 240);
    CHECK(secs > 0.4 && secs < 1.5); // 30 frames at 60 fps
    CHECK(seen >= 25 && ordered);
    CHECK(m.consumer.producerPid() == 0);       // detached at the end
    CHECK(m.consumer.producerNote() == "fake mirror");
    CHECK(m.consumer.hdr().src_width == 3840 && m.consumer.hdr().src_height == 3552);
}

TEST(the_mirror_drops_and_counts_instead_of_waiting) {
    MirrorRing m("busy", 1, 60);
    FakeMirror fake;
    fake.busy = true;
    std::atomic<bool> stop{false};
    std::jthread stopper([&] {
        std::this_thread::sleep_for(300ms);
        stop = true;
    });
    const MirrorResult r = runMirror(m.consumer, fake, stop);
    CHECK(r.code == 0 && r.reason == "the recorder asked to stop");
    CHECK(m.consumer.counters().dropped_busy >= 10 && m.consumer.counters().produced == 0);
    // nobody takes the frames: the ring fills and the mirror goes on (dropped_full), never blocking
    MirrorRing full("full", 1, 72);
    FakeMirror f2;
    std::atomic<bool> stop2{false};
    std::jthread stopper2([&] {
        std::this_thread::sleep_for(300ms);
        stop2 = true;
    });
    runMirror(full.consumer, f2, stop2);
    CHECK(full.consumer.counters().produced == 4 && full.consumer.counters().dropped_full >= 10);
}

TEST(the_mirror_ends_when_the_recorder_or_steamvr_is_gone) {
    {
        MirrorRing m("stale", 1, 30);
        m.quit = true; // the recorder's heartbeat stops
        FakeMirror fake;
        std::atomic<bool> stop{false};
        MirrorOptions o;
        o.consumerTimeoutS = 0.3;
        const MirrorResult r = runMirror(m.consumer, fake, stop, o);
        CHECK(r.code == 0 && r.reason == "the recorder stopped answering");
    }
    {
        MirrorRing m("gone", 1, 30);
        FakeMirror fake;
        std::atomic<bool> stop{false};
        MirrorOptions o;
        o.steamvrAlive = [] { return false; };
        const MirrorResult r = runMirror(m.consumer, fake, stop, o);
        CHECK(r.code == 0 && r.reason == "SteamVR ended");
        CHECK(m.consumer.producerPid() == 0);
    }
    {
        MirrorRing m("stopreq", 1, 30);
        m.consumer.requestStop();
        FakeMirror fake;
        std::atomic<bool> stop{false};
        CHECK(runMirror(m.consumer, fake, stop).reason == "the recorder asked to stop");
    }
}

TEST(mirror_errors_in_plain_words) {
    MirrorRing m("errors", 1, 30);
    std::atomic<bool> stop{false};
    FakeMirror a;
    a.initError = "SteamVR's mirror texture is not available: " + compositorErrorWords(106);
    MirrorResult r = runMirror(m.consumer, a, stop);
    CHECK(r.code == 3 && contains(r.reason, "VRCompositorError_SharedTexturesNotSupported"));
    CHECK(m.consumer.producerPid() == 0); // never attached
    FakeMirror b;
    b.failAfter = 3;
    r = runMirror(m.consumer, b, stop);
    CHECK(r.code == 4 && r.reason == "OpenGL error 0x505 while reading SteamVR's mirror texture");
    CHECK(contains(compositorErrorWords(103), "scene applications only"));
    CHECK(compositorErrorWords(999) == "SteamVR compositor error 999");
}

TEST(the_nvidia_egl_device_comes_first) {
    // live 2026-10-19: GLVND's default display was Mesa's on the NVIDIA card ("failed to create dri2 screen"), and
    // SteamVR's mirror texture had no size in that context
    using V = std::vector<int>;
    CHECK(eglDeviceOrder({"Mesa Project", "NVIDIA"}, -1) == (V{1, 0}));
    CHECK(eglDeviceOrder({"NVIDIA", "", "Mesa Project"}, -1) == (V{0, 2})); // a device that did not start is skipped
    CHECK(eglDeviceOrder({"Mesa Project"}, -1) == (V{0}));
    CHECK(eglDeviceOrder({"Mesa Project", "NVIDIA"}, 0) == (V{0})); // PIBRIDGE_RECORD_EGL_DEVICE=0
    CHECK(eglDeviceOrder({"Mesa Project", "NVIDIA"}, 5) == (V{1, 0})); // out of range: ignored
    CHECK(eglDeviceOrder({}, -1).empty());
}

TEST(the_mirror_size_falls_back_to_the_rectangle_target_then_the_recommended_size) {
    std::string how;
    CHECK(chooseMirrorSize({2448, 2448}, {0, 0}, {3000, 3000}, how) == U2(2448, 2448) && how == "texture size");
    CHECK(chooseMirrorSize({0, 0}, {1920, 1080}, {3000, 3000}, how) == U2(1920, 1080) && how == "rectangle texture size");
    CHECK(chooseMirrorSize({0, 0}, {0, 0}, {3000, 2900}, how) == U2(3000, 2900) && contains(how, "recommended"));
    CHECK(chooseMirrorSize({0, 0}, {0, 0}, {0, 0}, how) == U2(0, 0) && how == "none");
}

TEST(no_mirror_size_says_why) {
    struct NoSize : FakeMirror {
        std::pair<std::uint32_t, std::uint32_t> eyeSize() override { return {0, 0}; }
        std::string sizeProblem() const override { return "texture 3 is unknown in the OpenGL context on EGL device 0"; }
    } fake;
    MirrorRing m("nosize", 1, 30);
    std::atomic<bool> stop{false};
    const MirrorResult r = runMirror(m.consumer, fake, stop);
    CHECK(r.code == 3 &&
          r.reason == "SteamVR's mirror texture has no size (texture 3 is unknown in the OpenGL context on EGL device 0)");
}

TEST(the_openvr_library_comes_from_openvrpaths) {
    CHECK(vrpathRuntime(R"({"config":["/c"],"runtime" : [ "/s/Steam\/steamapps/common/SteamVR/" ],"version":1})") ==
          "/s/Steam/steamapps/common/SteamVR");
    CHECK(vrpathRuntime(R"({"runtime":[]})").empty() && vrpathRuntime("{}").empty());
    TempDir d("vrpath");
    const fs::path rt = d.p / "SteamVR";
    fs::create_directories(rt / "bin/linux64");
    std::ofstream(rt / "bin/linux64/libopenvr_api.so") << "x";
    fs::create_directories(d.p / "cfg/openvr");
    std::ofstream(d.p / "cfg/openvr/openvrpaths.vrpath") << "{\"runtime\":[\"" << rt.string() << "\"]}";
    const std::string oldCfg = std::getenv("XDG_CONFIG_HOME") ? std::getenv("XDG_CONFIG_HOME") : "";
    setenv("XDG_CONFIG_HOME", (d.p / "cfg").c_str(), 1);
    unsetenv("PIBRIDGE_OPENVR_LIB");
    unsetenv("PIMAX_OPENVR_LIB");
    CHECK(openVrLibrary() == rt / "bin/linux64/libopenvr_api.so");
    setenv("PIBRIDGE_OPENVR_LIB", "/x/libopenvr_api.so", 1);
    CHECK(openVrLibrary() == fs::path("/x/libopenvr_api.so"));
    unsetenv("PIBRIDGE_OPENVR_LIB");
    if (oldCfg.empty()) {
        unsetenv("XDG_CONFIG_HOME");
    } else {
        setenv("XDG_CONFIG_HOME", oldCfg.c_str(), 1);
    }
}

// ---------------------------------------------------------------- the whole recorder (tests/offline.rs, with fakes)

namespace {

struct Offline {
    TempDir d;
    fs::path ring, ffLog;
    explicit Offline(const std::string& n) : d("e2e-" + n) {
        for (const char* s : {"session", "run", "out", "cfg", "home"}) {
            fs::create_directories(d.p / s);
        }
        ring = d.p / "run/rec.shm";
        ffLog = d.p / "ffmpeg-args.txt";
    }
    // the real pibridge-record binary, every path in the temp dir, the fake ffmpeg
    pid_t recorder(const std::vector<std::string>& extra, const std::vector<std::string>& extraEnv = {}) const {
        std::vector<std::string> args{PIBRIDGE_RECORD_BIN, "run", "--max-height", "240", "--fps", "30", "--dir",
                                      (d.p / "out").string()};
        args.insert(args.end(), extra.begin(), extra.end());
        std::vector<std::string> env{"PIMAX_RECORD_SHM=" + ring.string(),
                                     "PIBRIDGE_VR_SESSION_DIR=" + (d.p / "session").string(),
                                     "XDG_RUNTIME_DIR=" + (d.p / "run").string(),
                                     "XDG_CONFIG_HOME=" + (d.p / "cfg").string(),
                                     "HOME=" + (d.p / "home").string(),
                                     "PIBRIDGE_FFMPEG=" PIBRIDGE_FAKE_FFMPEG,
                                     "FAKE_FFMPEG_LOG=" + ffLog.string(),
                                     "PIBRIDGE_RECORD_NO_MONADO_CHECK=1",
                                     "PIBRIDGE_RECORD_NO_NOTIFY=1",
                                     std::string("PATH=") + (std::getenv("PATH") ? std::getenv("PATH") : "/usr/bin:/bin")};
        env.insert(env.end(), extraEnv.begin(), extraEnv.end());
        std::vector<char*> av, ev;
        std::vector<std::string> as = args, es = env;
        for (auto& s : as) av.push_back(s.data());
        for (auto& s : es) ev.push_back(s.data());
        av.push_back(nullptr);
        ev.push_back(nullptr);
        const fs::path log = d.p / "session/recorder.out";
        const pid_t pid = fork();
        if (pid == 0) {
            FILE* f = std::fopen(log.c_str(), "w");
            if (f) {
                dup2(fileno(f), 1);
                dup2(fileno(f), 2);
            }
            execve(av[0], av.data(), ev.data());
            _exit(127);
        }
        return pid;
    }
    Status status() const { return Status::load(statusPath(d.p / "session")).value_or(Status{}); }
};

// What Monado's tap does: attach, frames at the requested rate, heartbeat until `stop`.
struct FakeTap {
    std::atomic<bool> quit{false};
    std::atomic<long> sent{0};
    std::jthread t;
    FakeTap(const fs::path& ringPath, long frames, bool attach = true) {
        t = std::jthread([this, ringPath, frames, attach] {
            std::optional<Ring> ring;
            const auto t0 = std::chrono::steady_clock::now();
            while (!ring && !quit && std::chrono::steady_clock::now() - t0 < 10s) {
                if (auto r = Ring::open(ringPath)) {
                    ring.emplace(std::move(*r));
                } else {
                    std::this_thread::sleep_for(10ms);
                }
            }
            if (!ring || !attach) {
                return;
            }
            const ShmHeader& h = ring->hdr();
            const auto ext = pickEyeExtent({3840, 3552}, h.max_height, h.scale_permille, h.eyes, h.slot_size).value();
            const std::uint32_t w = ext.first * h.eyes, ht = ext.second;
            ring->attachProducer(w, ht, "fake tap", 4242);
            std::uint64_t ts = monotonicNs();
            const std::uint64_t iv = 1'000'000'000ull / h.fps;
            for (std::uint64_t seq = 0; !quit && !ring->stopRequested(); seq++) {
                ring->producerHeartbeat();
                if (sent < frames && ring->publish(w, ht, seq, ts, [](std::span<std::uint8_t> b) { std::ranges::fill(b, 7); })) {
                    sent++;
                    ts += iv;
                }
                std::this_thread::sleep_for(5ms);
            }
            ring->detachProducer();
        });
    }
    ~FakeTap() { quit = true; }
};

int waitExit(pid_t pid, std::chrono::seconds limit) {
    const auto t0 = std::chrono::steady_clock::now();
    int st = 0;
    while (waitpid(pid, &st, WNOHANG) == 0) {
        if (std::chrono::steady_clock::now() - t0 > limit) {
            kill(pid, SIGKILL);
            waitpid(pid, &st, 0);
            return -1;
        }
        std::this_thread::sleep_for(20ms);
    }
    return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

void waitFor(const std::function<bool()>& f, std::chrono::seconds limit) {
    const auto t0 = std::chrono::steady_clock::now();
    while (!f() && std::chrono::steady_clock::now() - t0 < limit) {
        std::this_thread::sleep_for(20ms);
    }
}

} // namespace

TEST(records_a_file_and_finishes_it_on_sigint) {
    Offline o("record");
    FakeTap tap(o.ring, 30);
    const pid_t pid = o.recorder({"--encoder", "auto"});
    waitFor([&] { return tap.sent >= 30 && o.status().frames >= 25; }, 10s);
    CHECK(o.status().state == "recording");
    kill(pid, SIGINT);
    CHECK(waitExit(pid, 30s) == 0);
    const Status st = o.status();
    CHECK(st.state == "idle" && st.note.empty());
    CHECK(st.encoder == "h264_nvenc" && st.audio == "output" && st.eyes == "left" && st.fps == 30);
    CHECK(st.width == 258 && st.height == 240);
    CHECK(st.frames >= 25);
    CHECK(fs::exists(st.file) && fs::file_size(st.file) == st.frames * 258 * 240 * 4);
    CHECK(fs::path(st.file).filename().string().starts_with("pimax-"));
    const std::string args = readAll(o.ffLog);
    CHECK(contains(args, "-video_size 258x240 -framerate 30 -i pipe:0 -thread_queue_size 1024 -f pulse -i @DEFAULT_MONITOR@ "
                         "-c:v h264_nvenc"));
    CHECK(contains(args, "-f mp4 " + st.file));
    CHECK(!fs::exists(o.ring)); // the recorder removed its ring
    CHECK(contains(readAll(eventsPath(o.d.p / "session")), "\"text\":\"saved "));
}

TEST(nvenc_that_dies_at_once_falls_back_to_the_cpu_with_the_same_name) {
    Offline o("fallback");
    FakeTap tap(o.ring, 20);
    const pid_t pid = o.recorder({"--eyes", "both", "--audio", "off"}, {"FAKE_FFMPEG_FAIL_NVENC=1"});
    waitFor([&] { return o.status().encoder == "libx264" && o.status().frames >= 10; }, 10s);
    kill(pid, SIGINT);
    CHECK(waitExit(pid, 30s) == 0);
    const Status st = o.status();
    CHECK(st.state == "idle" && st.encoder == "libx264" && st.audio.empty() && st.width == 516);
    const std::string args = readAll(o.ffLog);
    CHECK(contains(args, "-c:v h264_nvenc") && contains(args, "-c:v libx264 -preset veryfast -crf 19"));
    CHECK(contains(args, "-map 0:v:0 -an"));
    // "exit status: 1" or "its input closed", whichever the recorder notices first (as in omarchyvr)
    const std::string ev = readAll(eventsPath(o.d.p / "session"));
    CHECK(contains(ev, "h264_nvenc failed (") && contains(ev, "), falling back to libx264"));
}

TEST(no_sound_output_records_the_video_alone) {
    Offline o("nosound");
    FakeTap tap(o.ring, 10);
    const pid_t pid = o.recorder({"--mic", "on"}, {"FAKE_FFMPEG_PULSE_EXIT=1"});
    waitFor([&] { return o.status().frames >= 5; }, 10s);
    kill(pid, SIGINT);
    CHECK(waitExit(pid, 30s) == 0);
    CHECK(o.status().audio.empty());
    CHECK(contains(readAll(eventsPath(o.d.p / "session")),
                   "recording without sound: the audio output cannot be read ([in#0] Error opening input: Connection refused)"));
}

TEST(monado_gone_finishes_the_file) {
    Offline o("gone");
    std::optional<FakeTap> tap(std::in_place, o.ring, 10);
    const pid_t pid = o.recorder({});
    waitFor([&] { return o.status().frames >= 5; }, 10s);
    tap->quit = true; // the tap stops answering: heartbeat stale (or detached)
    tap.reset();
    CHECK(waitExit(pid, 30s) == 0);
    const Status st = o.status();
    CHECK(st.state == "idle" && st.note == "Monado stopped (or detached the tap)" && fs::exists(st.file));
}

TEST(without_a_tap_it_says_so_and_exits_0) {
    Offline o("notap");
    FakeTap tap(o.ring, 0, false);
    const pid_t pid = o.recorder({});
    CHECK(waitExit(pid, 20s) == 0);
    const Status st = o.status();
    CHECK(st.state == "error" && contains(st.note, "has no recorder tap"));
    CHECK(!fs::exists(o.ring));
}


// a fake proc tree in which SteamVR (vrserver) runs, for pibridge-record-mirror's own check
fs::path fakeSteamVrProc(const fs::path& base) {
    const fs::path p = base / "proc";
    fs::create_directories(p / "900");
    std::ofstream(p / "900/stat") << "900 (vrserver) S 1\n";
    std::ofstream(p / "900/comm") << "vrserver\n";
    return p;
}

TEST(steamvr_recordings_use_the_mirror_on_the_same_ring) {
    Offline o("mirror");
    // the mirror process replaced by the fake producer (what pibridge-record-mirror does on the ring)
    const pid_t pid = o.recorder({"--source", "mirror", "--eyes", "both"}, {"PIBRIDGE_RECORD_MIRROR_BIN=" PIBRIDGE_FAKE_PRODUCER});
    waitFor([&] { return o.status().frames >= 10; }, 10s);
    CHECK(o.status().state == "recording" && o.status().source == "mirror");
    kill(pid, SIGINT);
    CHECK(waitExit(pid, 30s) == 0);
    const Status st = o.status();
    CHECK(st.state == "idle" && st.source == "mirror" && st.width == 516 && fs::exists(st.file));
    CHECK(!fs::exists(o.ring));
}

TEST(a_mirror_that_cannot_start_says_why) {
    Offline o("mirror-fail");
    const fs::path proc = fakeSteamVrProc(o.d.p);
    // the real pibridge-record-mirror with a fake libopenvr_api.so: SteamVR "not running" (no GPU, no EGL reached)
    const pid_t pid = o.recorder({"--source", "mirror"},
                                 {"PIBRIDGE_RECORD_MIRROR_BIN=" PIBRIDGE_RECORD_MIRROR, "PIBRIDGE_OPENVR_LIB=" PIBRIDGE_FAKE_OPENVR,
                                  "PIBRIDGE_PROC_ROOT=" + proc.string()});
    CHECK(waitExit(pid, 30s) == 0);
    Status st = o.status();
    CHECK(st.state == "error" && st.note == "SteamVR's mirror could not start: SteamVR is not running");
    CHECK(!fs::exists(o.ring));
    // the library gives no compositor interface
    Offline o2("mirror-noiface");
    const fs::path proc2 = fakeSteamVrProc(o2.d.p);
    const pid_t pid2 = o2.recorder({"--source", "mirror"},
                                   {"PIBRIDGE_RECORD_MIRROR_BIN=" PIBRIDGE_RECORD_MIRROR, "PIBRIDGE_OPENVR_LIB=" PIBRIDGE_FAKE_OPENVR,
                                    "PIBRIDGE_PROC_ROOT=" + proc2.string(), "FAKE_OPENVR_INIT_ERROR=0"});
    CHECK(waitExit(pid2, 30s) == 0);
    st = o2.status();
    CHECK(st.note == "SteamVR's mirror could not start: SteamVR gives no compositor interface to this client");
    // no SteamVR at all: the mirror refuses before loading anything
    Offline o3("mirror-nosteamvr");
    fs::create_directories(o3.d.p / "proc");
    const pid_t pid3 = o3.recorder({"--source", "mirror"}, {"PIBRIDGE_RECORD_MIRROR_BIN=" PIBRIDGE_RECORD_MIRROR,
                                                            "PIBRIDGE_PROC_ROOT=" + (o3.d.p / "proc").string()});
    CHECK(waitExit(pid3, 30s) == 0);
    CHECK(o3.status().note == "SteamVR's mirror could not start: SteamVR is not running");
}

PIBRIDGE_TEST_MAIN()
