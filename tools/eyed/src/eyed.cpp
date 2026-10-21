// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/eyed.rs (MIT OR Apache-2.0).
#include "eyed.h"

#include "pibridge_pause.h"
#include "pibridge_env.h"

#include "hid_commands.h"
#include "pibridge_status.h"

#include "pibridge/hmd/sys.h"

#include "pibridge/eye/frames.h"

#include "pibridge/eyemath/clock.h"

#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <sstream>

#include <unistd.h>

namespace pibridge::eyed {

namespace fs = std::filesystem;
using pibridge::eye::live::State;

// ------------------------------------------------------------------------------------------------ options

ConfigDirs ConfigDirs::from_env() {
    const fs::path p = Settings::config_dir();
    return ConfigDirs{p, p.parent_path() / "pimax-linux", p.parent_path() / "omarchyvr"};
}

std::optional<fs::path> ConfigDirs::find(std::string_view name) const {
    std::error_code ec;
    for (const auto& d : {pibridge, legacy, omarchyvr}) {
        if (d.empty()) {
            continue;
        }
        const fs::path p = d / name;
        if (fs::is_regular_file(p, ec)) {
            return p;
        }
    }
    return std::nullopt;
}

namespace {

// `~/x` as the app or the user may write it; everything else as given.
fs::path expand(const std::string& s) {
    if (s.starts_with("~/")) {
        if (const char* h = std::getenv("HOME"); h && *h) {
            return fs::path(h) / s.substr(2);
        }
    }
    return fs::path(s);
}

} // namespace

std::filesystem::path settings_path_of(const std::vector<std::string>& args) {
    for (std::size_t i = 0; i + 1 < args.size(); i++) {
        if (args[i] == "--settings") {
            return args[i + 1];
        }
    }
    return Settings::default_path();
}

std::string usage() {
    return "usage: pibridge-eyed [--frames FILE] [--profile FILE | --no-profile] [--shm PATH] [--follow PROCESS | "
           "--no-follow] [--settings FILE]\n"
           "Runs the eye tracker and publishes the gaze in shared memory (default /dev/shm/pibridge-eye-gaze).\n"
           "Defaults come from ~/.config/pibridge/pibridge.conf (eye_start_list, eye_profile, eye_tracking,\n"
           "eye_flip_x, follow); the start list and profile default to eye-start-crystal-super.txt and\n"
           "eye-profile-crystal-super.bin in ~/.config/pibridge (or ~/.config/pimax-linux, then\n"
           "~/.config/omarchyvr, when only there).\n"
           "SIGHUP re-reads the settings; SIGTERM/SIGINT stop it (unsubscribe, \"no gaze\").\n";
}

std::expected<Options, std::string> parse_args(const std::vector<std::string>& args, const Settings& settings,
                                               const ConfigDirs& dirs) {
    Options o;
    o.follow = settings.follow;
    o.eye_tracking = settings.eye_tracking;
    o.flip_x = settings.eye_flip_x;
    o.frames = settings.eye_start_list.empty() ? dirs.find(START_LIST_FILE)
                                               : std::optional<fs::path>(expand(settings.eye_start_list));
    o.profile = settings.eye_profile.empty() ? dirs.find(PROFILE_FILE)
                                             : std::optional<fs::path>(expand(settings.eye_profile));
    o.settings_path = settings_path_of(args);
    for (std::size_t i = 0; i < args.size(); i++) {
        const std::string& a = args[i];
        const auto value = [&]() -> std::expected<std::string, std::string> {
            if (i + 1 >= args.size()) {
                return std::unexpected(a + " needs a value");
            }
            return args[++i];
        };
        if (a == "--help" || a == "-h") {
            return std::unexpected(std::string("help"));
        }
        if (a == "--frames" || a == "--profile" || a == "--shm" || a == "--follow" || a == "--settings") {
            auto v = value();
            if (!v) {
                return std::unexpected(v.error());
            }
            if (a == "--frames") {
                o.frames = fs::path(*v);
            } else if (a == "--profile") {
                o.profile = fs::path(*v);
            } else if (a == "--shm") {
                if (v->empty()) {
                    return std::unexpected(std::string("--shm needs a path"));
                }
                o.shm = *v;
            } else if (a == "--follow") {
                o.follow = *v;
            }
        } else if (a == "--no-profile") {
            o.no_profile = true;
        } else if (a == "--no-follow") {
            o.follow.clear();
        } else {
            return std::unexpected("unknown option " + a);
        }
    }
    return o;
}

// ------------------------------------------------------------------------------------------------ config

namespace {

std::expected<std::string, std::string> read_file(const fs::path& p, bool& missing) {
    std::error_code ec;
    missing = !fs::exists(p, ec);
    if (missing) {
        return std::unexpected(std::string("No such file or directory"));
    }
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        return std::unexpected(std::string("cannot be read (permissions?)"));
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

std::expected<pibridge::eye::live::Config, std::string> make_config(const Options& opts, pibridge::eye::live::Config cfg,
                                                               const std::function<void(std::string_view)>& say,
                                                               std::string& profile_kind) {
    profile_kind = "none";
    if (opts.frames) {
        const std::string f = opts.frames->string();
        bool missing = false;
        auto text = read_file(*opts.frames, missing);
        if (!text) {
            return std::unexpected(std::format("start list {}: {}", f, text.error()));
        }
        const auto steps = pibridge::eye::frames::parse(*text);
        if (!steps) {
            return std::unexpected(std::format("start list {}: {}", f, steps.error()));
        }
        say(std::format("start list {}: {} steps", f, steps->size()));
        cfg.frames = std::make_shared<const std::string>(std::move(*text));
        // the list's own profile upload is this user's Pimax Play eye profile
        cfg.with_cal = !opts.no_profile;
    } else {
        say("no start list file: the built-in base list (the Dream Air's start)");
    }
    if (opts.profile && !opts.no_profile) {
        const std::string f = opts.profile->string();
        bool missing = false;
        auto b = read_file(*opts.profile, missing);
        if (!b) {
            if (!missing) {
                return std::unexpected(std::format("eye profile {}: {}", f, b.error()));
            }
            say(std::format("no eye profile {}: the list's own", f));
        } else {
            pibridge::eye::Bytes body(b->begin(), b->end());
            if (auto ok = pibridge::eye::frames::check_profile(body); !ok) {
                return std::unexpected(std::format("eye profile {}: {}", f, ok.error()));
            }
            cfg.profile = std::make_shared<const pibridge::eye::Bytes>(std::move(body));
        }
    }
    if (opts.no_profile) {
        cfg.profile = nullptr;
    }
    if (cfg.profile) {
        say(std::format("eye profile: {} bytes (the user's calibration)", cfg.profile->size()));
        profile_kind = "user";
    } else if (cfg.with_cal) {
        profile_kind = "list";
    }
    return cfg;
}

// ------------------------------------------------------------------------------------------------ daemon

Daemon::Daemon(Options opts, Deps deps) : opts_(std::move(opts)), deps_(std::move(deps)) {
    if (!deps_.say) {
        deps_.say = [](std::string_view m) {
            std::printf("%s %.*s\n", local_hms().c_str(), static_cast<int>(m.size()), m.data());
            std::fflush(stdout);
        };
    }
    if (!deps_.hid) {
        deps_.hid = std::make_shared<HidHeadsetCommands>();
    }
    if (!deps_.base_config) {
        deps_.base_config = [](const pibridge::eye::live::LogFn& log) { return pibridge::eye::live::Config::from_env(log); };
    }
    if (!deps_.flip_x_from_env) {
        deps_.flip_x_from_env = [] { return pibridge::env("EYE_FLIP_X") != nullptr; };
    }
    if (!deps_.start_tracker) {
        auto hid = deps_.hid;
        auto say = deps_.say;
        deps_.start_tracker = [hid, say](const pibridge::eye::live::Config& c) {
            return pibridge::eye::live::Tracker::start(c, std::make_unique<pibridge::eye::live::RealDevice>(hid),
                                                  [say](std::string_view m) { say("pibridge-eye: " + std::string(m)); });
        };
    }
}

Daemon::~Daemon() {
    if (started_) {
        stop();
    }
}

void Daemon::say(const std::string& m) const { deps_.say(m); }

void Daemon::configure() {
    const auto log = [this](std::string_view m) { say("pibridge-eye: " + std::string(m)); };
    auto base = deps_.base_config(log);
    if (!deps_.flip_x_from_env()) {
        base.conv.flip_x = opts_.flip_x;
    }
    auto c = make_config(opts_, std::move(base), [this](std::string_view m) { say(std::string(m)); }, profile_kind_);
    if (!c) {
        cfg_ok_ = false;
        error_ = c.error();
        say(error_ + "; the eye tracker does not start (fix the setting, then reload)");
        return;
    }
    cfg_ = std::move(*c);
    cfg_ok_ = true;
    error_.clear();
}

bool Daemon::should_run() const { return !paused_ && cfg_ok_ && opts_.eye_tracking && (!fol_ || fol_->active()); }

void Daemon::set_paused(bool on, Clock::time_point now) {
    if (on == paused_ || stopped_) {
        return;
    }
    paused_ = on;
    if (on) {
        tr_.reset(); // unsubscribes and releases the chip
        say(std::string(pibridge::pause::kWords) + ": the eye chip is left to Monado's Pimax driver");
    } else {
        say("inside-out tracking ended: the eye tracker may run again");
        if (should_run() && !tr_) {
            start_tracker();
        }
    }
    write_status(now, true);
}

void Daemon::start_tracker() {
    tr_ = deps_.start_tracker(cfg_);
    last_state_.reset();
}

std::expected<void, std::string> Daemon::start(Clock::time_point now) {
    auto out = pibridge::eye::shm::Publisher::create(opts_.shm);
    if (!out) {
        return std::unexpected(std::format("{}: {}", opts_.shm.string(), out.error().to_string()));
    }
    out_.emplace(std::move(*out));
    if (auto r = out_->publish(nullptr, static_cast<std::uint32_t>(State::Searching), pibridge::eyemath::monotonic_ns()); !r) {
        return std::unexpected(std::format("{}: {}", opts_.shm.string(), r.error().to_string()));
    }
    say("publishing the gaze to " + opts_.shm.string());
    started_ = true;
    configure();
    if (!opts_.follow.empty()) {
        follow_running_ = deps_.process_running(opts_.follow);
        fol_.emplace(follow_running_);
        say(std::format("following {}: {}", opts_.follow,
                        follow_running_ ? "running" : "not running, waiting for it"));
    }
    if (!opts_.eye_tracking) {
        say("eye tracking is off (eye_tracking in pibridge.conf): idle");
    }
    if (should_run()) {
        start_tracker();
    }
    last_follow_check_ = now;
    last_write_ = now;
    since_ = now;
    rate_since_ = now;
    write_status(now, true);
    return {};
}

namespace {

std::string enable_eye_module(pibridge::eye::power::HeadsetCommands& hid) {
    if (auto r = pibridge::eye::power::send_enable(hid); !r) {
        return "eye module enable failed: " + r.error().to_string();
    }
    std::string m = "sent the captured eye-module enable 58 05 3c 05 00 01";
    if (const auto* h = dynamic_cast<const HidHeadsetCommands*>(&hid); h && !h->last_target().empty()) {
        m += " to the " + h->last_target();
    }
    return m;
}

double secs(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

} // namespace

void Daemon::tick(Clock::time_point now) {
    if (!out_ || stopped_) {
        return;
    }
    if (fol_ && (!last_follow_check_ || now - *last_follow_check_ >= FOLLOW_EVERY)) {
        last_follow_check_ = now;
        follow_running_ = deps_.process_running(opts_.follow);
        if (const auto ch = fol_->step(now, follow_running_); ch == Follow::Change::Active) {
            if (should_run() && !tr_) {
                say(opts_.follow + " started: starting the eye tracker");
                start_tracker();
            } else if (paused_) {
                say(opts_.follow + " started: the eye tracker stays paused (inside-out tracking)");
            }
        } else if (ch == Follow::Change::Idle) {
            say(opts_.follow + " gone for 5 s: stopping the eye tracker");
            tr_.reset(); // unsubscribes and releases the chip
        }
    }
    if (paused_) {
        write_status(now, false); // no gaze is published: Monado's own eye code owns the file meanwhile
        return;
    }
    if (!tr_) {
        if (!last_write_ || now - *last_write_ >= SHM_EVERY) {
            (void)out_->publish(nullptr, static_cast<std::uint32_t>(State::Stopped), pibridge::eyemath::monotonic_ns());
            last_write_ = now;
        }
        last_state_.reset();
        last_seq_.reset();
        rate_ = 0;
        rate_count_ = 0;
        rate_since_ = now;
        write_status(now, false);
        return;
    }
    const State st = tr_->state();
    if (last_state_ != st) {
        say("eye tracker: " + std::string(pibridge::eye::live::state_name(st)));
        last_state_ = st;
    }
    const auto s = st == State::Streaming ? tr_->latest() : std::nullopt;
    const std::optional<std::uint64_t> seq = s ? std::optional<std::uint64_t>(s->seq) : std::nullopt;
    const bool fresh = seq != last_seq_;
    // every new sample, and twice a second without one (readers see the state and that the writer lives)
    if (fresh || !last_write_ || now - *last_write_ >= SHM_EVERY) {
        std::optional<pibridge::eye::shm::Sample> o;
        if (s) {
            o = pibridge::eye::live::to_shm(*s);
        }
        if (auto r = out_->publish(o ? &*o : nullptr, static_cast<std::uint32_t>(st), pibridge::eyemath::monotonic_ns()); !r) {
            say(opts_.shm.string() + ": " + r.error().to_string());
        }
        last_write_ = now;
        last_seq_ = seq;
        if (fresh && s) {
            streamed_++;
            rate_count_++;
        }
    }
    if (fresh && s) {
        if (s->eyes[0].direction_valid || s->eyes[1].direction_valid) {
            last_valid_ = now;
        }
        // about half the samples carry no eye even when worn (blinks, alternating frames): "lost" only after
        // EYES_LOST_AFTER without a single valid one
        const bool seen = last_valid_ && now - *last_valid_ < EYES_LOST_AFTER;
        if (eyes_ && eyes_->first == seen) {
            const auto t = eyes_->second;
            if (!seen && !enable_sent_ && now - t >= NO_EYES_ENABLE && cfg_.power_on) {
                say(std::format("no eye for {:.0f} s: {}", secs(now - t), enable_eye_module(*deps_.hid)));
                enable_sent_ = true;
            }
        } else if (eyes_) {
            say(std::format("{} after {:.1f} s", seen ? "eyes seen again" : "eyes lost", secs(now - eyes_->second)));
            eyes_ = std::pair{seen, now};
            if (seen) {
                enable_sent_ = false;
            }
        } else {
            say(seen ? "eyes seen" : "no eye seen yet");
            eyes_ = std::pair{seen, now};
        }
    }
    if (now - since_ >= std::chrono::seconds(60) && streamed_ > 0) {
        say(std::format("{:.0f} gaze samples/s", static_cast<double>(streamed_) / secs(now - since_)));
        streamed_ = 0;
        since_ = now;
    }
    if (now - rate_since_ >= std::chrono::seconds(1)) {
        rate_ = static_cast<double>(rate_count_) / secs(now - rate_since_);
        rate_count_ = 0;
        rate_since_ = now;
    }
    write_status(now, false);
}

void Daemon::reload(Options opts, Clock::time_point now) {
    const Options old = opts_;
    opts_ = std::move(opts);
    opts_.shm = old.shm; // the gaze file changes only at the next start
    const bool restart = opts_.frames != old.frames || opts_.profile != old.profile ||
                         opts_.no_profile != old.no_profile || opts_.flip_x != old.flip_x ||
                         opts_.eye_tracking != old.eye_tracking || !cfg_ok_;
    if (opts_.follow != old.follow) {
        if (opts_.follow.empty()) {
            fol_.reset();
            follow_running_ = false;
            say("settings: not following any process (always on)");
        } else {
            follow_running_ = deps_.process_running(opts_.follow);
            fol_.emplace(follow_running_);
            say(std::format("settings: following {}: {}", opts_.follow, follow_running_ ? "running" : "not running"));
        }
        last_follow_check_ = now;
    }
    if (restart) {
        if (tr_) {
            say("settings changed: restarting the eye tracker");
        }
        tr_.reset();
        configure();
    }
    if (should_run() && !tr_) {
        start_tracker();
    } else if (!should_run() && tr_) {
        tr_.reset();
    }
    write_status(now, true);
}

void Daemon::stop() {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    tr_.reset();
    if (out_) {
        (void)out_->publish(nullptr, static_cast<std::uint32_t>(State::Searching), pibridge::eyemath::monotonic_ns());
    }
    if (!deps_.status_name.empty()) {
        std::error_code ec;
        fs::remove(pibridge::status::runtime_dir() / (deps_.status_name + ".json"), ec);
    }
    say("stopped");
}

Status Daemon::status() const {
    Status s;
    s.profile = profile_kind_;
    s.follow = opts_.follow;
    s.follow_running = follow_running_;
    if (paused_) {
        s.state = "paused";
        s.paused = std::string(pibridge::pause::kWords);
        return s;
    }
    if (cfg_ok_ && cfg_.calibration) {
        s.calibration = cfg_.calibration_path.string();
    }
    if (!cfg_ok_) {
        s.state = "error";
        s.error = error_;
        return s;
    }
    if (!tr_) {
        s.state = "idle";
        return s;
    }
    const State st = tr_->state();
    switch (st) {
    case State::Searching:
    case State::Reconnecting: s.state = "searching"; break;
    case State::Connecting: s.state = "connecting"; break;
    case State::Streaming: s.state = "streaming"; break;
    case State::Stopped: s.state = "idle"; break;
    }
    if (st == State::Streaming) {
        s.eyes_seen = eyes_ && eyes_->first;
        s.samples_per_s = rate_;
    } else if (st == State::Searching || st == State::Reconnecting) {
        s.error = tr_->last_message();
    }
    return s;
}

void Daemon::write_status(Clock::time_point now, bool force) {
    if (deps_.status_name.empty() || stopped_) {
        return;
    }
    Status st = status();
    Status cmp = st;
    cmp.samples_per_s = 0; // the rate alone is not a change: it goes out with the 2 s rewrite
    const bool changed = !last_status_ || *last_status_ != cmp;
    if (!force && !changed && last_status_write_ && now - *last_status_write_ < STATUS_EVERY) {
        return;
    }
    pibridge::status::StatusWriter w;
    w.str("state", st.state)
        .str("profile", st.profile)
        .boolean("eyes_seen", st.eyes_seen)
        .num("samples_per_s", st.samples_per_s)
        .str("calibration", st.calibration)
        .str("follow", st.follow)
        .boolean("follow_running", st.follow_running)
        .str("error", st.error)
        .str("paused", st.paused)
        .integer("pid", static_cast<long long>(::getpid()));
    (void)w.write(deps_.status_name);
    last_status_ = cmp;
    last_status_write_ = now;
}

} // namespace pibridge::eyed
