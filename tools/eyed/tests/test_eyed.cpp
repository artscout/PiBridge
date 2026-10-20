// SPDX-License-Identifier: MIT
// pibridge-eyed's state machine with FakeChip trackers (libpibridge-eye's in-process chip streaming eyemath-encoded gaze), a
// recording headset command sink, a fake process table, a temporary gaze file and temporary config and runtime dirs.
// Nothing opens a device, reads the real /proc for the followed process, or touches /dev/shm/omarchyvr-eye-gaze.
#include "eyed.h"

#include "pibridge_status.h"
#include "pibridge_test.h"

#include "pibridge/eye/fake.h"
#include "pibridge/eye/frames.h"
#include "pibridge/eye/live.h"
#include "pibridge/eye/shm.h"

#include <atomic>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>

using namespace pibridge::hmd;
using namespace pibridge::eyed;
using namespace std::chrono_literals;
namespace fs = std::filesystem;
namespace live = pibridge::eye::live;
namespace shm = pibridge::eye::shm;

namespace {

const fs::path g_tmp = [] {
    const auto d = fs::temp_directory_path() / ("pibridge-eyed-test-" + std::to_string(::getpid()));
    fs::remove_all(d);
    fs::create_directories(d);
    ::setenv("XDG_RUNTIME_DIR", (d / "run").c_str(), 1);
    ::setenv("HOME", (d / "home").c_str(), 1);
    ::setenv("XDG_CONFIG_HOME", (d / "home/.config").c_str(), 1);
    return d;
}();
// removed when the test binary exits
const struct Cleanup {
    ~Cleanup() {
        std::error_code ec;
        fs::remove_all(g_tmp, ec);
    }
} g_cleanup;

std::string slurp(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void put(const fs::path& p, std::string_view s) {
    fs::create_directories(p.parent_path());
    std::ofstream(p, std::ios::binary) << s;
}

fs::path status_file() { return pibridge::status::runtime_dir() / "eyed.json"; }

bool status_has(std::string_view field) { return slurp(status_file()).find(field) != std::string::npos; }

struct RecordingHid final : pibridge::eye::power::HeadsetCommands {
    std::mutex mu;
    std::vector<std::string> sent;
    pibridge::eye::Result<void> send(std::string_view id) override {
        std::lock_guard l(mu);
        sent.emplace_back(id);
        return {};
    }
    std::size_t count() {
        std::lock_guard l(mu);
        return sent.size();
    }
};

/// The base list's own profile body (a valid profile layout).
pibridge::eye::Bytes base_profile() {
    auto steps = pibridge::eye::frames::parse(pibridge::eye::frames::base_list());
    for (const auto& st : *steps) {
        if (const auto* t = std::get_if<pibridge::eye::frames::Ttp>(&st.action); t && t->op == pibridge::eye::frames::CAL_OP) {
            const auto m = pibridge::eye::frames::ttp_info(t->frames);
            if (const auto b = pibridge::eye::frames::profile_body(m->payload)) {
                return pibridge::eye::Bytes(b->begin(), b->end());
            }
        }
    }
    return {};
}

struct Rig {
    std::atomic<bool> running{false};
    std::shared_ptr<std::atomic<bool>> eyes = std::make_shared<std::atomic<bool>>(true);
    std::atomic<int> starts{0};
    std::shared_ptr<RecordingHid> hid = std::make_shared<RecordingHid>();
    std::mutex log_mu;
    std::vector<std::string> log;
    std::vector<live::Config> cfgs;
    bool power_on = true;
    fs::path shm_path;
    std::unique_ptr<Daemon> d;

    explicit Rig(Options o, bool running_at_start = false, bool power = true) : running(running_at_start), power_on(power) {
        static int n = 0;
        shm_path = g_tmp / ("gaze-" + std::to_string(n++));
        o.shm = shm_path;
        Deps deps;
        deps.process_running = [this](std::string_view p) { return p == "vrserver" && running.load(); };
        deps.hid = hid;
        deps.flip_x_from_env = [] { return false; };
        deps.base_config = [this](const live::LogFn&) {
            live::Config c;
            c.pace = false;
            c.retry = 20ms;
            c.power_on = power_on;
            return c;
        };
        deps.start_tracker = [this](const live::Config& c) {
            starts++;
            {
                std::lock_guard l(log_mu);
                cfgs.push_back(c);
            }
            auto eyes_on = eyes;
            return live::Tracker::start_with(
                c,
                [eyes_on](const live::Config&, std::stop_token) -> std::expected<std::unique_ptr<pibridge::eye::Transport>, std::string> {
                    auto chip = std::make_unique<pibridge::eye::fake::FakeChip>();
                    chip->start_on = pibridge::eye::session::OP_SUBSCRIBE;
                    chip->payload = [eyes_on](std::uint32_t k, std::int64_t ts) {
                        auto g = live::fake_sample(k, ts);
                        if (!eyes_on->load()) {
                            g.eyes[0] = pibridge::eyemath::Eye{};
                        }
                        return pibridge::eyemath::encode_payload(g);
                    };
                    return chip;
                },
                [](std::string_view) {});
        };
        deps.say = [this](std::string_view m) {
            std::lock_guard l(log_mu);
            log.emplace_back(m);
        };
        d = std::make_unique<Daemon>(std::move(o), std::move(deps));
    }
    bool logged(std::string_view part) {
        std::lock_guard l(log_mu);
        return std::ranges::any_of(log, [&](const std::string& x) { return x.find(part) != std::string::npos; });
    }
    int count_logged(std::string_view part) {
        std::lock_guard l(log_mu);
        return static_cast<int>(std::ranges::count_if(log, [&](const std::string& x) { return x.find(part) != std::string::npos; }));
    }
    /// Tick with the real clock (plus `offset`) until `f` holds or `max` passed.
    template <class F>
    bool run_until(F f, std::chrono::milliseconds max, Clock::duration offset = {}) {
        const auto t0 = Clock::now();
        while (Clock::now() - t0 < max) {
            d->tick(Clock::now() + offset);
            if (f()) {
                return true;
            }
            std::this_thread::sleep_for(3ms);
        }
        return f();
    }
    std::optional<shm::Gaze> gaze() const { return shm::read(shm_path); }
};

Options plain() {
    Options o;
    o.follow.clear();
    o.frames.reset();
    o.profile.reset();
    return o;
}

} // namespace

TEST(daemon_streams_gaze_to_the_shm_file_and_says_no_gaze_on_stop) {
    Rig r(plain());
    CHECK(r.d->start(Clock::now()).has_value());
    CHECK(r.starts == 1);
    CHECK(r.gaze().has_value() && r.gaze()->state == 0 && r.gaze()->flags == 0); // "no gaze" before the tracker runs
    CHECK(r.run_until([&] { const auto g = r.gaze(); return g && g->state == 2 && (g->flags & shm::LEFT) != 0; }, 5000ms));
    const auto g = *r.gaze();
    // the fake tracks eye A; eye_flip_x=1 (the default, the Crystal Super) makes it the left eye
    CHECK((g.flags & shm::RIGHT) == 0 && (g.flags & shm::COMBINED) != 0);
    CHECK(g.sample > 0 || g.t_ns > 0);
    CHECK(r.logged("eyes seen") && r.logged("eye tracker: Streaming"));
    CHECK(r.run_until([&] { return r.d->status().samples_per_s > 50; }, 3000ms));
    CHECK(r.d->status().state == "streaming" && r.d->status().eyes_seen && r.d->status().profile == "none");
    r.d->tick(Clock::now() + 3s); // the 2 s rewrite carries the rate
    CHECK(status_has("\"state\": \"streaming\"") && status_has("\"profile\": \"none\"") &&
          status_has("\"eyes_seen\": true") && status_has("\"pid\": " + std::to_string(::getpid())));
    r.d->stop();
    CHECK(r.d->tracker() == nullptr);
    const auto end = r.gaze();
    CHECK(end && end->state == 0 && end->flags == 0);
    CHECK(!fs::exists(status_file())); // the app reads a missing file as "not running"
    CHECK(r.logged("stopped"));
}

TEST(follow_starts_the_tracker_with_the_runtime_and_stops_it_5_s_after) {
    auto o = plain();
    o.follow = "vrserver";
    Rig r(o, false);
    const auto t0 = Clock::now();
    CHECK(r.d->start(t0).has_value());
    CHECK(r.starts == 0 && r.d->tracker() == nullptr);
    CHECK(r.logged("following vrserver: not running, waiting for it"));
    r.d->tick(t0 + 600ms);
    CHECK(r.gaze()->state == 4); // stopped: no tracker
    CHECK(r.d->status().state == "idle" && !r.d->status().follow_running && r.d->status().follow == "vrserver");
    CHECK(status_has("\"state\": \"idle\"") && status_has("\"follow\": \"vrserver\""));
    r.running = true;
    r.d->tick(t0 + 1200ms);
    CHECK(r.starts == 1 && r.d->tracker() != nullptr);
    CHECK(r.logged("vrserver started: starting the eye tracker"));
    CHECK(r.d->status().follow_running);
    r.running = false;
    r.d->tick(t0 + 1800ms);
    CHECK(r.d->tracker() != nullptr); // SteamVR restarts its server on its own: 5 s grace
    r.d->tick(t0 + 4000ms);
    CHECK(r.d->tracker() != nullptr);
    r.d->tick(t0 + 7000ms);
    CHECK(r.d->tracker() == nullptr);
    CHECK(r.logged("vrserver gone for 5 s: stopping the eye tracker"));
    r.d->stop();
}

TEST(inside_out_pauses_the_tracker_and_never_writes_the_gaze_file) {
    Rig r(plain());
    const auto t0 = Clock::now();
    CHECK(r.d->start(t0).has_value() && r.d->tracker() != nullptr);
    r.d->set_paused(true, t0);
    CHECK(r.d->tracker() == nullptr && r.d->paused()); // unsubscribed, the chip released
    CHECK(r.d->status().state == "paused" && r.d->status().paused == "paused: inside-out tracking");
    CHECK(status_has("\"state\": \"paused\"") && status_has("\"paused\": \"paused: inside-out tracking\""));
    const auto before = r.gaze();
    fs::remove(r.shm_path);
    r.d->tick(t0 + 1s);
    r.d->tick(t0 + 2s);
    CHECK(!fs::exists(r.shm_path) || r.gaze() == before); // Monado's eye code owns it meanwhile
    CHECK(r.starts == 1 && r.logged("paused: inside-out tracking"));
    r.d->set_paused(false, t0 + 3s);
    CHECK(r.starts == 2 && r.d->tracker() != nullptr && !r.d->paused());
    r.d->stop();
    // started while a session runs: the chip is never opened
    Rig p(plain());
    p.d->set_paused(true, t0);
    CHECK(p.d->start(t0).has_value() && p.starts == 0 && p.d->status().state == "paused");
    p.d->stop();
}

TEST(no_eye_for_4_s_sends_the_eye_module_enable_once_per_stretch) {
    Rig r(plain());
    r.eyes->store(false);
    CHECK(r.d->start(Clock::now()).has_value());
    CHECK(r.run_until([&] { return r.logged("no eye seen yet"); }, 5000ms));
    CHECK(r.hid->count() == 0);
    // 3 s later (the daemon's clock): still waiting
    CHECK(!r.run_until([&] { return r.hid->count() > 0; }, 200ms, 3s));
    // past 4 s: the enable, once
    CHECK(r.run_until([&] { return r.hid->count() > 0; }, 2000ms, 4500ms));
    CHECK(!r.run_until([&] { return r.hid->count() > 1; }, 200ms, 8s));
    CHECK(r.hid->count() == 1 && r.hid->sent[0] == "eye_power_on");
    CHECK(r.logged("no eye for 4 s: sent the captured eye-module enable 58 05 3c 05 00 01") ||
          r.logged("no eye for 5 s: sent the captured eye-module enable 58 05 3c 05 00 01"));
    // the eyes come back, go again: one more enable for the new stretch
    r.eyes->store(true);
    CHECK(r.run_until([&] { return r.logged("eyes seen again after"); }, 3000ms, 8s));
    r.eyes->store(false);
    CHECK(r.run_until([&] { return r.logged("eyes lost after"); }, 3000ms, 9s));
    CHECK(r.run_until([&] { return r.hid->count() == 2; }, 3000ms, 20s));
    r.d->stop();
}

TEST(no_enable_without_power_on) {
    Rig r(plain(), false, false);
    r.eyes->store(false);
    CHECK(r.d->start(Clock::now()).has_value());
    CHECK(r.run_until([&] { return r.logged("no eye seen yet"); }, 5000ms));
    CHECK(!r.run_until([&] { return r.hid->count() > 0; }, 300ms, 10s));
    r.d->stop();
}

TEST(sighup_restarts_the_tracker_only_when_its_settings_changed) {
    auto o = plain();
    o.flip_x = true;
    Rig r(o);
    const auto t0 = Clock::now();
    CHECK(r.d->start(t0).has_value());
    CHECK(r.starts == 1 && r.d->config().conv.flip_x);
    r.d->reload(o, t0 + 1s); // nothing changed
    CHECK(r.starts == 1);
    o.flip_x = false;
    r.d->reload(o, t0 + 2s);
    CHECK(r.starts == 2 && !r.d->config().conv.flip_x);
    CHECK(r.logged("settings changed: restarting the eye tracker"));
    o.eye_tracking = false;
    r.d->reload(o, t0 + 3s);
    CHECK(r.d->tracker() == nullptr && r.d->status().state == "idle");
    o.eye_tracking = true;
    r.d->reload(o, t0 + 4s);
    CHECK(r.starts == 3 && r.d->tracker() != nullptr);
    // a new followed process is taken in its current state (not running: the tracker stops)
    o.follow = "vrserver";
    r.d->reload(o, t0 + 5s);
    CHECK(r.d->tracker() == nullptr && r.starts == 3);
    r.running = true;
    o.follow.clear();
    r.d->reload(o, t0 + 6s); // always on again
    CHECK(r.d->tracker() != nullptr && r.starts == 4);
    // the gaze file stays where it was
    o.shm = g_tmp / "elsewhere";
    r.d->reload(o, t0 + 7s);
    CHECK(r.d->options().shm == r.shm_path && r.starts == 4);
    r.d->stop();
}

TEST(a_missing_start_list_is_an_error_state_until_fixed) {
    auto o = plain();
    o.frames = g_tmp / "lists/eye-start-crystal-super.txt";
    Rig r(o);
    const auto t0 = Clock::now();
    CHECK(r.d->start(t0).has_value());
    CHECK(r.starts == 0 && r.d->status().state == "error");
    CHECK(r.d->status().error.starts_with("start list " + o.frames->string() + ": No such file"));
    CHECK(status_has("\"state\": \"error\""));
    put(*o.frames, pibridge::eye::frames::base_list());
    r.d->reload(o, t0 + 1s); // SIGHUP after the app wrote the file
    CHECK(r.starts == 1 && r.d->status().error.empty() && r.d->status().profile == "list");
    CHECK(r.cfgs.back().frames && *r.cfgs.back().frames == pibridge::eye::frames::base_list() && r.cfgs.back().with_cal);
    r.d->stop();
}

TEST(the_status_file_is_rewritten_every_2_s) {
    Rig r(plain());
    const auto t0 = Clock::now();
    CHECK(r.d->start(t0).has_value());
    // let the state settle (searching -> connecting -> streaming each rewrite at once)
    CHECK(r.run_until([&] { return r.d->status().state == "streaming"; }, 5000ms));
    const auto t1 = Clock::now() + 10s;
    r.d->tick(t1);
    fs::remove(status_file());
    r.d->tick(t1 + 1s);
    CHECK(!fs::exists(status_file()));
    r.d->tick(t1 + 2100ms);
    CHECK(fs::exists(status_file()));
    r.d->stop();
}

TEST(make_config_takes_the_list_and_the_users_profile) {
    const auto dir = g_tmp / "cfg";
    const auto list = dir / "eye-start.txt";
    put(list, pibridge::eye::frames::base_list());
    const auto body = base_profile();
    CHECK(!body.empty());
    const auto prof = dir / "eye-profile.bin";
    put(prof, std::string_view(reinterpret_cast<const char*>(body.data()), body.size()));
    std::vector<std::string> said;
    const auto say = [&](std::string_view m) { said.emplace_back(m); };
    std::string kind;
    Options o = plain();
    o.frames = list;
    o.profile = prof;
    auto c = make_config(o, live::Config{}, say, kind);
    CHECK(c && c->frames && c->with_cal && c->profile && *c->profile == body && kind == "user");
    CHECK(std::ranges::any_of(said, [](const std::string& s) { return s.find("steps") != std::string::npos; }));
    // a missing profile: the list's own upload
    o.profile = dir / "missing.bin";
    said.clear();
    c = make_config(o, live::Config{}, say, kind);
    CHECK(c && !c->profile && c->with_cal && kind == "list");
    CHECK(std::ranges::any_of(said, [](const std::string& s) { return s.starts_with("no eye profile"); }));
    // --no-profile: no upload at all
    o.profile = prof;
    o.no_profile = true;
    c = make_config(o, live::Config{}, say, kind);
    CHECK(c && !c->profile && !c->with_cal && kind == "none");
    // a broken profile or list stops it with a reason
    o.no_profile = false;
    put(dir / "bad.bin", "xyz");
    o.profile = dir / "bad.bin";
    c = make_config(o, live::Config{}, say, kind);
    CHECK(!c && c.error().starts_with("eye profile " + (dir / "bad.bin").string()));
    put(dir / "bad.txt", "STEP 1 | X | x\nBULK_OUT ep=0x05 zz\n");
    o.frames = dir / "bad.txt";
    o.profile.reset();
    c = make_config(o, live::Config{}, say, kind);
    CHECK(!c && c.error().starts_with("start list"));
    // no list: the built-in one, no profile
    o.frames.reset();
    c = make_config(o, live::Config{}, say, kind);
    CHECK(c && !c->frames && !c->with_cal && kind == "none");
}

TEST(options_come_from_the_settings_and_the_default_files) {
    const ConfigDirs dirs{g_tmp / "opt/pibridge", g_tmp / "opt/pimax-linux", g_tmp / "opt/omarchyvr"};
    const Settings def = Settings::parse("");
    auto o = parse_args({}, def, dirs);
    CHECK(o && !o->frames && !o->profile && o->follow == "vrserver" && o->eye_tracking && o->flip_x);
    CHECK(o && o->shm == shm::default_path() && o->settings_path == Settings::default_path());
    // only the omarchyvr files exist: they are used
    put(dirs.omarchyvr / START_LIST_FILE, "x");
    put(dirs.omarchyvr / PROFILE_FILE, "x");
    o = parse_args({}, def, dirs);
    CHECK(o && o->frames == dirs.omarchyvr / START_LIST_FILE && o->profile == dirs.omarchyvr / PROFILE_FILE);
    // a file in the folder of PiBridge's earlier name comes next
    put(dirs.legacy / PROFILE_FILE, "x");
    o = parse_args({}, def, dirs);
    CHECK(o && o->frames == dirs.omarchyvr / START_LIST_FILE && o->profile == dirs.legacy / PROFILE_FILE);
    // a pibridge file wins
    put(dirs.pibridge / START_LIST_FILE, "x");
    o = parse_args({}, def, dirs);
    CHECK(o && o->frames == dirs.pibridge / START_LIST_FILE && o->profile == dirs.legacy / PROFILE_FILE);
    // settings keys
    const Settings s = Settings::parse("follow=\neye_tracking=off\neye_flip_x=0\neye_start_list=/l/list.txt\n"
                                       "eye_profile=~/p.bin\n");
    o = parse_args({}, s, dirs);
    CHECK(o && o->follow.empty() && !o->eye_tracking && !o->flip_x && o->frames == fs::path("/l/list.txt"));
    CHECK(o && o->profile == fs::path(std::getenv("HOME")) / "p.bin");
    // flags win
    o = parse_args({"--frames", "/f.txt", "--profile", "/p.bin", "--no-profile", "--shm", "/tmp/x", "--follow", "steam",
                    "--settings", "/s.conf"},
                   s, dirs);
    CHECK(o && o->frames == fs::path("/f.txt") && o->profile == fs::path("/p.bin") && o->no_profile);
    CHECK(o && o->shm == fs::path("/tmp/x") && o->follow == "steam" && o->settings_path == fs::path("/s.conf"));
    o = parse_args({"--no-follow"}, def, dirs);
    CHECK(o && o->follow.empty());
    o = parse_args({"--follow", ""}, def, dirs);
    CHECK(o && o->follow.empty());
    CHECK(!parse_args({"--frames"}, def, dirs) && parse_args({"--frames"}, def, dirs).error() == "--frames needs a value");
    CHECK(parse_args({"--bogus"}, def, dirs).error() == "unknown option --bogus");
    CHECK(parse_args({"--help"}, def, dirs).error() == "help");
    CHECK(settings_path_of({"--settings", "/a.conf"}) == fs::path("/a.conf"));
    CHECK(usage().find("pibridge-eyed") != std::string::npos);
    // ConfigDirs::from_env follows XDG_CONFIG_HOME
    const auto e = ConfigDirs::from_env();
    CHECK(e.pibridge == g_tmp / "home/.config/pibridge" && e.legacy == g_tmp / "home/.config/pimax-linux" &&
          e.omarchyvr == g_tmp / "home/.config/omarchyvr");
}

TEST(the_unit_file_is_a_user_unit_that_conflicts_with_the_older_eye_daemons) {
    const auto u = slurp(fs::path(PIBRIDGE_SOURCE_DEVICES_DIR).parent_path() / "tools/eyed/pibridge-eyed.service");
    CHECK(u.find("Conflicts=pimax-eyed.service omarchyvr-eyed.service") != std::string::npos);
    CHECK(u.find("ExecStart=%h/.local/bin/pibridge-eyed") != std::string::npos);
    CHECK(u.find("ExecReload=/bin/kill -HUP $MAINPID") != std::string::npos);
    CHECK(u.find("WantedBy=default.target") != std::string::npos);
    CHECK(u.find("\nEnvironment=") == std::string::npos); // eye_flip_x in the settings covers it
}

PIBRIDGE_TEST_MAIN()
