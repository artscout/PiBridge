// SPDX-License-Identifier: MIT
// pibridge-keepalived's state machine against the in-process fake headset, a fake process table and a fake pactl.
// Nothing here opens a device, runs pactl or reads the real /proc for the followed process. The status file goes to
// a temporary XDG_RUNTIME_DIR.
#include "keepalived.h"
#include "pibridge_pause.h"
#include "pibridge_status.h"
#include "pibridge_test.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/fake.h"

#include <atomic>
#include <functional>
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>

using namespace pibridge::hmd;
using namespace pibridge::keepalived;
using namespace std::chrono;

static const std::filesystem::path g_tmp = [] {
    const auto d = std::filesystem::temp_directory_path() / ("pibridge-test-keepalived-" + std::to_string(::getpid()));
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    ::setenv("XDG_RUNTIME_DIR", d.c_str(), 1);
    return d;
}();

static std::shared_ptr<const Allowlist> allow(const char* id) {
    const auto d = Descriptor::load(std::filesystem::path(PIBRIDGE_SOURCE_DEVICES_DIR) / (std::string(id) + ".toml"));
    if (!d) {
        std::abort();
    }
    return std::make_shared<const Allowlist>(d->allowlist);
}

static std::string slurp(const std::filesystem::path& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// pactl over a tiny in-memory PipeWire.
struct FakePactl {
    std::mutex mu;
    std::string sink = "desk-speakers";
    std::string source = "desk-mic";
    std::vector<std::string> calls;
    audio::Runner runner() {
        return [this](const std::vector<std::string>& a) -> std::optional<std::string> {
            std::lock_guard g(mu);
            std::string c;
            for (const auto& x : a) {
                c += (c.empty() ? "" : " ") + x;
            }
            calls.push_back(c);
            if (c == "get-default-sink") return sink;
            if (c == "get-default-source") return source;
            if (c == "list short sinks") return std::string("1\tdesk-speakers\tPipeWire\n2\talsa_output.usb-Pimax_Corporation_CS.pro-output-1\tPipeWire\n");
            if (c == "list short sources") return std::string("3\tdesk-mic\tPipeWire\n4\talsa_input.usb-Pimax_Corporation_CS.pro-input-0\tPipeWire\n");
            if (a.size() == 2 && a[0] == "set-default-sink") return sink = a[1], std::string{};
            if (a.size() == 2 && a[0] == "set-default-source") return source = a[1], std::string{};
            return std::nullopt;
        };
    }
};

struct Rig {
    std::atomic<bool> running{false};
    FakePactl pactl;
    std::vector<std::string> log;
    std::mutex log_mu;
    fake::FakeDevice dev;
    std::unique_ptr<Daemon> d;
    std::filesystem::path rate;

    Rig(bool running_at_start, Options o, const char* model = "crystal-super")
        : running(running_at_start), dev(nullptr) {
        auto [be, device] = fake::pair();
        dev = device;
        if (std::string(model) == "dream-air") {
            dev.set_model(DREAM_AIR);
        } else {
            dev.set_model(CRYSTAL_SUPER);
        }
        rate = g_tmp / ("rate-" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        o.rate_file = rate;
        Deps deps;
        deps.process_running = [this](std::string_view n) { return n == "vrserver" && running.load(); };
        deps.pactl = pactl.runner();
        deps.say = [this](std::string_view m) {
            std::lock_guard g(log_mu);
            log.emplace_back(m);
        };
        deps.allowlist = allow(model);
        deps.roots = discovery::Roots{g_tmp / "no-sysfs", g_tmp / "no-dev"}; // never the real sysfs
        d = std::make_unique<Daemon>(o, deps);
        CHECK(d->start(std::move(be), steady_clock::now()).has_value());
    }
    bool logged(std::string_view part) {
        std::lock_guard g(log_mu);
        return std::any_of(log.begin(), log.end(), [&](const std::string& l) { return l.find(part) != std::string::npos; });
    }
};

static Options crystal_opts() {
    Options o;
    o.video = 90;
    o.follow = "vrserver";
    o.audio = true;
    return o;
}

static int count_ka(const std::vector<Report>& v, std::uint8_t rate) {
    return static_cast<int>(std::count(v.begin(), v.end(), keepalive(rate)));
}
static bool has_prefix(const std::vector<Report>& v, std::initializer_list<std::uint8_t> p) {
    return std::any_of(v.begin(), v.end(), [&](const Report& r) { return std::equal(p.begin(), p.end(), r.begin()); });
}

static void wait_connected(Rig& r, steady_clock::time_point now) {
    for (int i = 0; i < 200 && !r.d->status().headset_present; i++) {
        std::this_thread::sleep_for(milliseconds(10));
    }
    r.d->tick(now);
}

TEST(follow_cycle_idle_active_idle) {
    Rig r(false, crystal_opts());
    const auto t0 = steady_clock::now();
    CHECK(slurp(r.rate) == "off\n");
    wait_connected(r, t0);
    // idle: the wake, one keepalive with rate 0, then nothing (green standby); IF1-IF3 are still read
    const auto idle = r.dev.drain_commands(milliseconds(700));
    CHECK(!idle.empty() && idle[0] == wake());
    CHECK(count_ka(idle, 0) == 1);
    CHECK(std::count_if(idle.begin(), idle.end(), [](const Report& c) { return c[2] == 0x39; }) == 1);
    CHECK(!has_prefix(idle, {0x58, 0x04, 0x29}));
    Status s = r.d->status();
    CHECK(s.state == "idle" && s.headset_present && !s.panel_on && s.rate_hz == 0 && !s.follow_running);
    CHECK(s.model == "Pimax Crystal Super");
    CHECK(r.pactl.sink == "desk-speakers");
    const auto status_file = pibridge::status::runtime_dir() / "keepalived.json";
    CHECK(slurp(status_file).find("\"state\": \"idle\"") != std::string::npos);

    // the runtime starts: wake, rate 90 at once; the video frames and the audio 2 s later
    r.running = true;
    r.d->tick(t0 + seconds(1));
    CHECK(slurp(r.rate) == "90\n");
    const auto waking = r.dev.drain_commands(milliseconds(500));
    CHECK(!waking.empty() && waking[0] == wake());
    CHECK(count_ka(waking, 90) >= 1);
    s = r.d->status();
    CHECK(s.state == "waking" && s.follow_running && s.rate_hz == 90 && !s.audio_on_headset);
    CHECK(slurp(status_file).find("\"state\": \"waking\"") != std::string::npos);
    r.d->tick(t0 + seconds(2));
    CHECK(!has_prefix(r.dev.drain_commands(milliseconds(50)), {0x58, 0x04, 0x29, 0x00, 0x01})); // not yet
    r.d->tick(t0 + milliseconds(3100));
    const auto video = r.dev.drain_commands(milliseconds(300));
    CHECK(has_prefix(video, {0x58, 0x05, 0x5b, 0x05, 0x02, 0x02}));
    CHECK(has_prefix(video, {0x58, 0x04, 0x29, 0x00, 0x01}));
    CHECK(has_prefix(video, {0x58, 0x0d, 0x1a, 0x05, 0x02, 0x5a}));
    CHECK(has_prefix(video, {0x58, 0x0d, 0x1a, 0x05, 0x05, 0x00}));
    s = r.d->status();
    CHECK(s.state == "active" && s.panel_on && s.audio_on_headset && s.error.empty());
    CHECK(r.pactl.sink == "alsa_output.usb-Pimax_Corporation_CS.pro-output-1");
    CHECK(r.pactl.source == "alsa_input.usb-Pimax_Corporation_CS.pro-input-0");
    CHECK(r.logged("video on at 90 Hz"));

    // the runtime ends: still busy for 5 s (SteamVR restarts its server), then idle and the audio back
    r.running = false;
    r.d->tick(t0 + seconds(4));
    r.d->tick(t0 + seconds(8));
    CHECK(r.d->status().state == "active");
    r.d->tick(t0 + milliseconds(9100));
    CHECK(slurp(r.rate) == "off\n");
    s = r.d->status();
    CHECK(s.state == "idle" && !s.panel_on && !s.audio_on_headset && s.rate_hz == 0);
    CHECK(r.pactl.sink == "desk-speakers" && r.pactl.source == "desk-mic");
    std::this_thread::sleep_for(milliseconds(250)); // let the last 90 and the closing 0 go out
    (void)r.dev.drain_commands(milliseconds(250));
    const auto quiet = r.dev.drain_commands(milliseconds(600));
    CHECK(std::count_if(quiet.begin(), quiet.end(), [](const Report& c) { return c[2] == 0x39; }) == 0);

    r.d->stop();
    CHECK(!std::filesystem::exists(status_file)); // a clean stop removes the status file
    CHECK(r.logged("stopped"));
}

TEST(sigterm_while_busy_sends_the_idle_report_and_gives_the_audio_back) {
    Rig r(true, crystal_opts());
    const auto t0 = steady_clock::now();
    CHECK(slurp(r.rate) == "90\n");
    CHECK(r.pactl.sink.starts_with("alsa_output.usb-Pimax")); // busy at start: the audio switches at once
    wait_connected(r, t0);
    r.d->tick(t0 + milliseconds(2100)); // the video frames 2 s after the connect
    const auto seen = r.dev.drain_commands(milliseconds(400));
    CHECK(has_prefix(seen, {0x58, 0x04, 0x29, 0x00, 0x01}));
    CHECK(r.d->status().state == "active");
    r.d->stop();
    const auto after = r.dev.drain_commands(milliseconds(200));
    CHECK(!after.empty() && after.back() == keepalive(0)); // the last thing the headset hears: rate 0
    CHECK(slurp(r.rate) == "off\n");
    CHECK(r.pactl.sink == "desk-speakers" && r.pactl.source == "desk-mic");
}

TEST(inside_out_pauses_releases_the_headset_and_resumes) {
    Rig r(true, crystal_opts());
    const auto t0 = steady_clock::now();
    wait_connected(r, t0);
    r.d->tick(t0 + milliseconds(2100));
    CHECK(r.d->status().state == "active");
    r.dev.drain_commands(milliseconds(300));
    r.d->pause(t0 + milliseconds(2200));
    const auto after = r.dev.drain_commands(milliseconds(300));
    // already awake with the video on: released at once, without the idle report (Monado takes it awake)
    for (const auto& c : after) {
        CHECK(c != keepalive(0));
    }
    CHECK(r.d->paused() && r.d->status().state == "paused" && r.d->status().paused == "paused: inside-out tracking");
    CHECK(!r.d->status().headset_present);
    CHECK(r.pactl.sink.starts_with("alsa_output.usb-Pimax")); // the sound stays on the headset while paused
    CHECK(slurp(pibridge::status::runtime_dir() / "keepalived.json").find("\"state\": \"paused\"") != std::string::npos);
    r.d->tick(t0 + seconds(5));
    CHECK(r.dev.drain_commands(milliseconds(300)).empty()); // nothing reaches the headset while paused
    // resume on a new fake device: the session opens again and wakes the headset
    auto [be, dev2] = fake::pair();
    dev2.set_model(CRYSTAL_SUPER);
    CHECK(r.d->resume(std::move(be), t0 + seconds(6)).has_value() && !r.d->paused());
    for (int i = 0; i < 200 && !r.d->status().headset_present; i++) {
        std::this_thread::sleep_for(milliseconds(10));
    }
    r.d->tick(t0 + seconds(6));
    CHECK(r.d->status().headset_present && r.d->status().state != "paused");
    r.d->stop();
}

TEST(inside_out_pause_wakes_a_sleeping_headset_before_releasing_it) {
    // live 2026-10-19: Monado's Pimax driver woke a standby headset itself, the headset re-enumerated on USB under it
    // and inside vrserver the driver then got no IMU; the hand-over wakes it the way a SteamVR start does first
    Rig r(false, crystal_opts()); // vrserver not running: idle
    const auto t0 = steady_clock::now();
    wait_connected(r, t0);
    r.d->tick(t0 + milliseconds(100));
    r.dev.drain_commands(milliseconds(300));
    r.d->pause(t0 + milliseconds(200));
    CHECK(!r.d->paused());
    const auto waking = r.dev.drain_commands(milliseconds(300));
    CHECK(!waking.empty() && waking[0] == wake());
    r.d->tick(t0 + milliseconds(2300)); // the video frames 2 s after the wake
    CHECK(has_prefix(r.dev.drain_commands(milliseconds(300)), {0x58, 0x04, 0x29, 0x00, 0x01}));
    r.d->pause(t0 + milliseconds(3000));
    CHECK(!r.d->paused()); // settling on USB
    r.d->pause(t0 + milliseconds(5400));
    CHECK(r.d->paused() && r.d->status().state == "paused");
    for (const auto& c : r.dev.drain_commands(milliseconds(300))) {
        CHECK(c != keepalive(0)); // left awake
    }
    r.d->stop();
}

TEST(pause_marker_is_stale_safe) {
    namespace pz = pibridge::pause;
    const auto p = g_tmp / "inside-out.active";
    CHECK(pz::marker_text(42, "steamvr") == "pibridge-inside-out 1\nowner 42\nvariant steamvr\n");
    { std::ofstream(p) << pz::marker_text(42, "steamvr"); }
    CHECK(pz::active_marker(p, [](long pid) { return pid == 42; }).has_value());
    CHECK(pz::active_marker(p, [](long pid) { return pid == 42; })->variant == "steamvr");
    CHECK(!pz::active_marker(p, [](long) { return false; })); // the owner is gone: stale
    { std::ofstream(p) << "something else\n"; }
    CHECK(!pz::active_marker(p, [](long) { return true; }));
    CHECK(!pz::active_marker(g_tmp / "none", [](long) { return true; }));
    { std::ofstream(p) << pz::marker_text(::getpid(), "monado"); }
    CHECK(pz::active_marker(p).has_value()); // this process runs
}

TEST(always_on_without_a_followed_process) {
    Options o = crystal_opts();
    o.follow.clear();
    o.audio = false;
    Rig r(false, o);
    const auto t0 = steady_clock::now();
    CHECK(slurp(r.rate) == "90\n");
    wait_connected(r, t0);
    r.d->tick(t0 + milliseconds(2100));
    const auto seen = r.dev.drain_commands(milliseconds(400));
    CHECK(has_prefix(seen, {0x58, 0x0d, 0x1a, 0x05, 0x02, 0x5a}));
    const Status s = r.d->status();
    CHECK(s.state == "active" && s.follow.empty() && !s.follow_running && !s.audio_on_headset);
    CHECK(r.pactl.calls.empty()); // audio off: pactl never runs
}

TEST(reconnect_switches_the_panel_on_again_while_busy) {
    Rig r(true, crystal_opts());
    auto t = steady_clock::now();
    wait_connected(r, t);
    r.d->tick(t += milliseconds(2100));
    CHECK(r.d->status().panel_on);
    r.dev.unplug();
    for (int i = 0; i < 200 && r.d->status().headset_present; i++) {
        std::this_thread::sleep_for(milliseconds(10));
    }
    r.d->tick(t += milliseconds(100));
    Status s = r.d->status();
    CHECK(s.state == "no-headset" && !s.headset_present && !s.panel_on);
    (void)r.dev.drain_commands(milliseconds(1));
    r.dev.plug();
    CHECK(r.dev.wait_host(milliseconds(2000)));
    wait_connected(r, t += milliseconds(100));
    CHECK(r.d->status().state == "waking");
    r.d->tick(t += milliseconds(2100));
    const auto seen = r.dev.drain_commands(milliseconds(400));
    CHECK(has_prefix(seen, {0x58, 0x04, 0x29, 0x00, 0x01}));
    CHECK(r.d->status().state == "active");
}

TEST(dream_air_uses_its_own_description) {
    Options o = crystal_opts();
    o.video = 72;
    Rig r(true, o, "dream-air");
    auto t = steady_clock::now();
    wait_connected(r, t);
    r.d->tick(t + milliseconds(2100));
    const auto seen = r.dev.drain_commands(milliseconds(400));
    CHECK(has_prefix(seen, {0x58, 0x0d, 0x1a, 0x05, 0x02, 0x48}));
    CHECK(count_ka(seen, 72) >= 1);
    CHECK(r.d->status().model == "Pimax Dream Air");
}

TEST(reload_applies_follow_and_audio_changes) {
    Rig r(false, crystal_opts());
    auto t = steady_clock::now();
    wait_connected(r, t);
    CHECK(r.d->status().state == "idle");
    (void)r.dev.drain_commands(milliseconds(300));
    // the app sets follow= (always on): busy at once, no grace
    Options o = crystal_opts();
    o.follow.clear();
    o.audio = false;
    r.d->reload(o, t += milliseconds(100));
    CHECK(slurp(r.rate) == "90\n");
    CHECK(r.d->status().state == "waking");
    r.d->tick(t += milliseconds(2100));
    CHECK(has_prefix(r.dev.drain_commands(milliseconds(300)), {0x58, 0x04, 0x29, 0x00, 0x01}));
    CHECK(r.d->status().state == "active");
    CHECK(r.pactl.sink == "desk-speakers");
    // audio on through the settings
    o.audio = true;
    r.d->reload(o, t += milliseconds(100));
    CHECK(r.d->status().audio_on_headset);
    // follow vrserver again while it does not run: idle at once
    o.follow = "vrserver";
    r.d->reload(o, t += milliseconds(100));
    CHECK(slurp(r.rate) == "off\n");
    CHECK(r.d->status().state == "idle");
    CHECK(!r.d->status().audio_on_headset);
}

TEST(status_file_is_rewritten_every_2_s) {
    Rig r(false, crystal_opts());
    auto t = steady_clock::now();
    wait_connected(r, t);
    const auto f = pibridge::status::runtime_dir() / "keepalived.json";
    const auto stamp = [&] {
        const std::string s = slurp(f);
        const auto p = s.find("\"updated_unix_ms\": ");
        return p == std::string::npos ? std::string() : s.substr(p, 40);
    };
    const std::string a = stamp();
    std::this_thread::sleep_for(milliseconds(20));
    r.d->tick(t += milliseconds(500)); // nothing changed, too soon: not rewritten
    CHECK(stamp() == a);
    r.d->tick(t += milliseconds(1600)); // 2.1 s since the last write
    CHECK(stamp() != a);
    const std::string body = slurp(f);
    for (const char* k : {"\"state\"", "\"model\"", "\"headset_present\"", "\"panel_on\"", "\"rate_hz\"", "\"follow\"",
                          "\"follow_running\"", "\"audio_on_headset\"", "\"error\"", "\"pid\""}) {
        CHECK(body.find(k) != std::string::npos);
    }
    CHECK(body.find("\"pid\": " + std::to_string(::getpid())) != std::string::npos);
}

// ------------------------------------------------------------------------------------------------ options

TEST(options_default_from_the_settings_file_and_flags_override) {
    Settings st = Settings::parse("follow=vrserver\nrefresh_hz=90\naudio_to_headset=on\n");
    auto o = parse_args({}, st);
    CHECK(o && o->video == std::optional<std::uint8_t>(90) && o->follow == "vrserver" && o->audio && o->wake);
    CHECK(o->rate_file == g_tmp / "pimax-rate");
    // the omarchyvr Lighthouse drop-in: --video 90 --follow vrserver --audio
    o = parse_args({"--video", "72", "--follow", "monado-service", "--no-audio"}, st);
    CHECK(o && o->video == std::optional<std::uint8_t>(72) && o->follow == "monado-service" && !o->audio);
    o = parse_args({"--follow=", "--rate-file=/x/rate", "--no-wake"}, st);
    CHECK(o && o->follow.empty() && o->rate_file == "/x/rate" && !o->wake);
    o = parse_args({"--no-follow"}, st);
    CHECK(o && o->follow.empty());
    // plain keepalive (omarchyvr keepalived without --video)
    o = parse_args({"--no-video"}, st);
    CHECK(o && !o->video && o->follow.empty() && !o->audio);
    st = Settings::parse("follow=\nrefresh_hz=72\naudio_to_headset=off\n");
    o = parse_args({}, st);
    CHECK(o && o->video == std::optional<std::uint8_t>(72) && o->follow.empty() && !o->audio);
}

TEST(options_refuse_what_the_rust_daemon_refuses) {
    const Settings st;
    CHECK(parse_args({"--video", "60"}, st).error() == "--video wants 72 or 90, not \"60\"");
    CHECK(parse_args({"--no-video", "--follow", "vrserver"}, st).error() == "--follow needs --video FPS");
    CHECK(parse_args({"--no-video", "--audio"}, st).error() == "--audio needs --video FPS");
    CHECK(parse_args({"--bogus"}, st).error().starts_with("unknown option"));
    CHECK(parse_args({"--video"}, st).error() == "--video needs a value");
    CHECK(parse_args({"--help"}, st).error() == "help");
    CHECK(settings_path_of({"--settings", "/a/b.conf"}) == "/a/b.conf");
    CHECK(settings_path_of({"--video", "90", "--settings=/c.conf"}) == "/c.conf");
    CHECK(parse_args({"--settings", "/a/b.conf"}, st)->settings_path == "/a/b.conf");
}

// ------------------------------------------------------------------------------------------------ controllers

static void put(const std::filesystem::path& p, const std::string& text) {
    std::filesystem::create_directories(p.parent_path());
    const auto tmp = p.string() + ".tmp";
    std::ofstream(tmp) << text;
    std::filesystem::rename(tmp, p);
}
static std::filesystem::path ctrl_req() { return pibridge::status::runtime_dir() / "controller-request"; }
static std::filesystem::path haptic_req() { return g_tmp / "pimax-haptic-request"; }
static Report heartbeat(std::uint8_t slot) {
    Report r{};
    const std::uint8_t b[] = {0x4c, 0x0f, 0x37, slot, 0x48, 1, 2, 3, 4, 0x49, 0, 2, 1, 0, 8, 0x2d};
    std::copy(std::begin(b), std::end(b), r.begin());
    return r;
}
static Report log_report(std::string_view text) {
    Report r{};
    std::copy(text.begin(), text.end(), r.begin());
    return r;
}
static bool wait_for(const std::function<bool()>& f) {
    for (int i = 0; i < 200; i++) {
        if (f()) {
            return true;
        }
        std::this_thread::sleep_for(milliseconds(10));
    }
    return false;
}

TEST(pair_sends_start_pair_once_and_reports_the_firmware_verdict) {
    Rig r(true, crystal_opts());
    const auto t0 = steady_clock::now();
    wait_connected(r, t0);
    (void)r.dev.drain_commands(milliseconds(100));
    put(ctrl_req(), "11 pair left\n");
    r.d->tick(steady_clock::now());
    CHECK(!std::filesystem::exists(ctrl_req()));
    Status s = r.d->status();
    CHECK(s.ctrl_id == 11 && s.ctrl_op == "pair" && s.ctrl_side == "left" && s.ctrl_state == "waiting");
    const auto sent = r.dev.drain_commands(milliseconds(200));
    CHECK(std::count(sent.begin(), sent.end(), start_pair(2)) == 1);
    // a second request while it listens is refused, nothing more is sent
    put(ctrl_req(), "12 unpair right\n");
    r.d->tick(steady_clock::now());
    CHECK(r.d->status().ctrl_id == 12 && r.d->status().ctrl_state == "refused");
    // the firmware log (IF3) says it worked
    CHECK(r.dev.send_report(3, log_report("pair finished\n")));
    CHECK(wait_for([&] {
        r.d->tick(steady_clock::now());
        return r.d->status().ctrl_state == "paired";
    }));
    s = r.d->status();
    CHECK(s.ctrl_id == 11 && s.ctrl_detail.find("left controller is paired") != std::string::npos);
    r.d->tick(steady_clock::now());
    const auto after = r.dev.drain_commands(milliseconds(100));
    CHECK(std::count_if(after.begin(), after.end(), [](const Report& c) { return c[2] == 0x04 || c[2] == 0x05; }) == 0);
}

TEST(pair_times_out_and_refuses_a_connected_controller) {
    Rig r(true, crystal_opts());
    const auto t0 = steady_clock::now();
    wait_connected(r, t0);
    put(ctrl_req(), "1 pair right\n");
    r.d->tick(t0);
    CHECK(r.d->status().ctrl_state == "waiting");
    r.d->tick(t0 + seconds(20));
    CHECK(r.d->status().ctrl_state == "waiting");
    r.d->tick(t0 + seconds(36));
    CHECK(r.d->status().ctrl_state == "timeout");
    // a controller that sends its heartbeat is already paired: refused, nothing sent
    CHECK(r.dev.send_report(0, heartbeat(2)));
    CHECK(wait_for([&] { return r.d->status().left_connected; }));
    (void)r.dev.drain_commands(milliseconds(100));
    put(ctrl_req(), "2 pair left\n");
    r.d->tick(steady_clock::now());
    CHECK(r.d->status().ctrl_state == "refused");
    CHECK(r.d->status().ctrl_detail.find("already paired") != std::string::npos);
    const auto c = r.dev.drain_commands(milliseconds(100));
    CHECK(std::count(c.begin(), c.end(), start_pair(2)) == 0);
}

TEST(unpair_waits_for_the_reply) {
    Rig r(true, crystal_opts());
    wait_connected(r, steady_clock::now());
    (void)r.dev.drain_commands(milliseconds(100));
    std::atomic<bool> stop{false};
    std::atomic<bool> right{false};
    std::thread dev([&] {
        while (!stop.load()) {
            if (const auto c = r.dev.recv_command(milliseconds(20)); c && (*c)[2] == 0x05) {
                right = *c == unpair(3);
                Report rep{};
                rep[0] = 0x4c, rep[1] = 0x04, rep[2] = 0x05, rep[3] = (*c)[3], rep[4] = 0x01, rep[5] = 0x07;
                r.dev.send_report(0, rep);
            }
        }
    });
    put(ctrl_req(), "3 unpair right\n");
    r.d->tick(steady_clock::now());
    stop = true;
    dev.join();
    CHECK(right.load());
    CHECK(r.d->status().ctrl_state == "unpaired");
    // no reply: a plain failure (the unpair entry allows one send per second)
    std::this_thread::sleep_for(milliseconds(1050));
    put(ctrl_req(), "4 unpair left\n");
    r.d->tick(steady_clock::now());
    CHECK(r.d->status().ctrl_state == "failed");
    CHECK(r.d->status().ctrl_detail.find("did not answer") != std::string::npos);
}

TEST(vibration_test_needs_a_connected_controller_and_attaches_it_first) {
    Rig r(true, crystal_opts());
    wait_connected(r, steady_clock::now());
    (void)r.dev.drain_commands(milliseconds(100));
    put(haptic_req(), "2 60 300\n");
    r.d->tick(steady_clock::now());
    CHECK(!std::filesystem::exists(haptic_req()));
    Status s = r.d->status();
    CHECK(s.haptic_seq == 1 && s.haptic_result.find("not connected") != std::string::npos);
    CHECK(r.dev.send_report(0, heartbeat(2)));
    CHECK(wait_for([&] { return r.d->status().left_connected; }));
    (void)r.dev.drain_commands(milliseconds(50));
    put(haptic_req(), "2 60 300\n");
    r.d->tick(steady_clock::now());
    s = r.d->status();
    CHECK(s.haptic_seq == 2 && s.haptic_result.find("sent to the left") != std::string::npos);
    const auto c = r.dev.drain_commands(milliseconds(200));
    CHECK(has_prefix(c, {0x58, 0x04, 0x16, 0x02, 0x1e}));                   // the attach burst first
    CHECK(has_prefix(c, {0x58, 0x09, 0x07, 0x02, 60, 0x2c, 0x01, 0, 0, 0})); // then 60 %, 300 ms
}

TEST(paused_refuses_pairing_and_leaves_the_vibration_to_monado) {
    Rig r(true, crystal_opts());
    const auto t0 = steady_clock::now();
    wait_connected(r, t0);
    r.d->tick(t0 + milliseconds(2100));
    for (int i = 0; i < 40 && !r.d->paused(); i++) {
        r.d->pause(t0 + seconds(3 + i));
    }
    CHECK(r.d->paused());
    put(ctrl_req(), "9 pair left\n");
    put(haptic_req(), "3 25 300\n");
    r.d->tick(t0 + seconds(60));
    CHECK(r.d->status().ctrl_id == 9 && r.d->status().ctrl_state == "refused");
    CHECK(r.d->status().ctrl_detail.find("Inside-out tracking") != std::string::npos);
    CHECK(std::filesystem::exists(haptic_req())); // Monado's Pimax driver takes it
    std::filesystem::remove(haptic_req());
}

int main(int argc, char** argv) {
    const int rc = pibridge::test::run_all(argc, argv);
    std::filesystem::remove_all(g_tmp);
    return rc;
}
