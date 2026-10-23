// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/keepalived.rs (MIT OR Apache-2.0).
#include "keepalived.h"

#include "pibridge/hmd/ctrl_serials.h"

#include "pibridge_pause.h"

#include "pibridge/hmd/discovery.h"
#include "pibridge/hmd/haptics.h"
#include "pibridge/hmd/ipd.h"
#include "pibridge_status.h"

#include <deque>
#include <cmath>
#include <atomic>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include <unistd.h>

namespace pibridge::keepalived {
namespace fs = std::filesystem;

// ------------------------------------------------------------------------------------------------ options

fs::path default_rate_file() {
    const char* x = std::getenv("XDG_RUNTIME_DIR");
    return fs::path(x && *x ? x : "/tmp") / "pimax-rate";
}

std::string usage() {
    return "usage: pibridge-keepalived [--video 72|90 | --no-video] [--follow PROCESS | --no-follow] [--audio | --no-audio]\n"
           "                        [--no-wake] [--rate-file PATH] [--settings PATH]\n"
           "Keeps a Pimax headset awake (wake + keepalive every 213 ms). Defaults come from\n"
           "~/.config/pibridge/pibridge.conf (refresh_hz, follow, audio_to_headset); flags override them.\n"
           "--follow '' (or --no-follow) keeps the headset busy all the time.\n";
}

fs::path settings_path_of(const std::vector<std::string>& args) {
    for (std::size_t i = 0; i + 1 < args.size(); i++) {
        if (args[i] == "--settings") {
            return args[i + 1];
        }
        if (args[i].starts_with("--settings=")) {
            return args[i].substr(11);
        }
    }
    if (!args.empty() && args.back().starts_with("--settings=")) {
        return args.back().substr(11);
    }
    return Settings::default_path();
}

std::expected<Options, std::string> parse_args(const std::vector<std::string>& args, const Settings& settings) {
    Options o;
    o.rate_file = default_rate_file();
    o.video = static_cast<std::uint8_t>(settings.refresh_hz);
    o.follow = settings.follow;
    o.audio_output = settings.audio_to_headset;
    o.audio_input = settings.mic_to_headset;
    o.audio = o.audio_output || o.audio_input;
    o.crystal_tracking = settings.crystal_tracking;
    o.crystal_autostart_steamvr = settings.crystal_autostart_steamvr;
    o.settings_path = settings_path_of(args);
    bool follow_flag = false;
    bool audio_flag = false;
    for (std::size_t i = 0; i < args.size(); i++) {
        std::string a = args[i];
        std::optional<std::string> inline_val;
        if (const auto eq = a.find('='); a.starts_with("--") && eq != std::string::npos) {
            inline_val = a.substr(eq + 1);
            a = a.substr(0, eq);
        }
        const auto value = [&]() -> std::expected<std::string, std::string> {
            if (inline_val) {
                return *inline_val;
            }
            if (i + 1 >= args.size()) {
                return std::unexpected(a + " needs a value");
            }
            return args[++i];
        };
        if (a == "--help" || a == "-h") {
            return std::unexpected(std::string("help"));
        } else if (a == "--no-wake") {
            o.wake = false;
        } else if (a == "--rate-file") {
            auto v = value();
            if (!v) {
                return std::unexpected(v.error());
            }
            o.rate_file = *v;
        } else if (a == "--video") {
            auto v = value();
            if (!v) {
                return std::unexpected(v.error());
            }
            if (*v == "72" || *v == "90") {
                o.video = static_cast<std::uint8_t>(std::stoi(*v));
            } else {
                return std::unexpected("--video wants 72 or 90, not \"" + *v + "\"");
            }
        } else if (a == "--no-video") {
            o.video.reset();
        } else if (a == "--follow") {
            auto v = value();
            if (!v) {
                return std::unexpected(v.error());
            }
            o.follow = *v;
            follow_flag = true;
        } else if (a == "--no-follow") {
            o.follow.clear();
            follow_flag = true;
        } else if (a == "--audio") {
            o.audio = o.audio_output = o.audio_input = true;
            audio_flag = true;
        } else if (a == "--no-audio") {
            o.audio = o.audio_output = o.audio_input = false;
        } else if (a == "--settings") {
            if (auto v = value(); !v) {
                return std::unexpected(v.error());
            }
        } else {
            return std::unexpected("unknown option \"" + args[i] + "\"");
        }
    }
    if (!o.video) {
        // plain keepalive (the Rust daemon without --video): following and audio need the panel mode
        if (follow_flag && !o.follow.empty()) {
            return std::unexpected(std::string("--follow needs --video FPS"));
        }
        if (audio_flag) {
            return std::unexpected(std::string("--audio needs --video FPS"));
        }
        o.follow.clear();
        o.audio = false;
    }
    return o;
}

// ------------------------------------------------------------------------------------------------ session events

struct Daemon::Events {
    std::mutex mu;
    int connects = 0;
    int disconnects = 0;
    Fd fd;
    std::function<void(std::string_view)> say;
    /// Firmware log lines (IF3) not looked at yet (pairing verdicts); the oldest dropped beyond 64.
    std::deque<std::string> log;
    /// Volume button presses (+1 up, -1 down) not applied yet, and the button held now (0 = none).
    int volume_steps = 0;
    int volume_held = 0;
    /// The lens IPD from the IMU reports, mm x 100 (0 = none yet); written on the session thread at the IMU rate.
    std::atomic<int> lens_ipd_x100{0};
    /// Last report / heartbeat of slot 2 and 3, and the last controller report alone.
    std::array<std::optional<Clock::time_point>, 2> heard;
    std::array<std::optional<Clock::time_point>, 2> streamed;
};

namespace {

class H : public Handler {
public:
    explicit H(std::shared_ptr<Daemon::Events> ev) : ev_(std::move(ev)) {
        // PIBRIDGE_HMD_CAPTURE=<file>: append every report this daemon does not decode (IF0 replies, IF1/IF2 reports
        // that are not IMU or controller data, IF3 log lines), one line each with milliseconds since the start, to find
        // the format of unknown reports (the headset's volume buttons). Off without the variable.
        if (const char* path = std::getenv("PIBRIDGE_HMD_CAPTURE"); path != nullptr && path[0] != '\0') {
            capture_ = std::fopen(path, "a");
            if (capture_ != nullptr) {
                std::setvbuf(capture_, nullptr, _IOLBF, 0);
                std::fprintf(capture_, "# pibridge-keepalived capture started\n");
            }
        }
        t0_ = std::chrono::steady_clock::now();
    }
    ~H() override {
        if (capture_ != nullptr) {
            std::fclose(capture_);
        }
    }
    void on_reply(const Report& d) override { capture(0, d, d.size()); }
    void on_other(std::size_t idx, const Report& d, std::size_t len) override { capture(idx, d, len); }
    void on_state(State s) override {
        {
            std::lock_guard g(ev_->mu);
            (s == State::Connected ? ev_->connects : ev_->disconnects)++;
        }
        eventfd_signal(ev_->fd.get());
    }
    void on_log(std::string_view l) override {
        if (capture_ != nullptr) {
            std::fprintf(capture_, "%lld if3 log %.*s\n", static_cast<long long>(ms()), static_cast<int>(l.size()),
                         l.data());
        }
        if (const auto b = audio::volume_button(l); b.dir != 0) {
            {
                std::lock_guard g(ev_->mu);
                if (b.pressed) {
                    ev_->volume_steps += b.dir;
                    ev_->volume_held = b.dir;
                } else if (ev_->volume_held == b.dir) {
                    ev_->volume_held = 0;
                }
            }
            eventfd_signal(ev_->fd.get());
        }
        std::lock_guard g(ev_->mu);
        ev_->log.emplace_back(l);
        if (ev_->log.size() > 64) {
            ev_->log.pop_front();
        }
    }
    void on_status(const ControllerStatus& s) override { heard(s.slot, false); }
    void on_imu(const ImuSample& s) override {
        if (s.ipd_mm >= 40.0f && s.ipd_mm <= 90.0f) {
            ev_->lens_ipd_x100.store(static_cast<int>(std::lround(s.ipd_mm * 100.0f)), std::memory_order_relaxed);
        }
    }
    void on_controller(const ControllerReport& r) override { heard(r.slot, true); }
    void on_link_closed(std::chrono::nanoseconds lived, std::uint64_t keepalives) override {
        char b[96];
        std::snprintf(b, sizeof b, "headset gone after %.1f s, %llu keepalives",
                      std::chrono::duration<double>(lived).count(), static_cast<unsigned long long>(keepalives));
        if (ev_->say) {
            ev_->say(b);
        }
    }

private:
    long long ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0_).count();
    }
    void capture(std::size_t idx, const Report& d, std::size_t len) {
        if (capture_ == nullptr) {
            return;
        }
        // trailing zeros are the read buffer's padding; keep at least the first 8 bytes
        while (len > 8 && d[len - 1] == 0) {
            len--;
        }
        std::string hex;
        char b[4];
        for (std::size_t i = 0; i < len && i < d.size(); i++) {
            std::snprintf(b, sizeof b, "%02x", d[i]);
            hex += b;
        }
        std::fprintf(capture_, "%lld if%zu %s\n", ms(), idx, hex.c_str());
    }
    FILE* capture_ = nullptr;
    std::chrono::steady_clock::time_point t0_;

    void heard(std::uint8_t slot, bool report) {
        if (!is_slot(slot)) {
            return;
        }
        const auto now = Clock::now();
        std::lock_guard g(ev_->mu);
        ev_->heard[slot - 2] = now;
        if (report) {
            ev_->streamed[slot - 2] = now;
        }
    }
    std::shared_ptr<Daemon::Events> ev_;
};

std::string hz(std::uint8_t f) { return std::to_string(static_cast<int>(f)); }

} // namespace

// ------------------------------------------------------------------------------------------------ daemon

Daemon::Daemon(Options opts, Deps deps) : opts_(std::move(opts)), deps_(std::move(deps)), ev_(std::make_shared<Events>()) {
    if (!deps_.say) {
        deps_.say = [](std::string_view m) {
            std::printf("%s %.*s\n", local_hms().c_str(), static_cast<int>(m.size()), m.data());
            std::fflush(stdout);
        };
    }
    if (auto e = eventfd()) {
        ev_->fd = std::move(*e);
    }
    ev_->say = deps_.say;
    Crystal::Deps cd;
    cd.find = deps_.crystal_find ? deps_.crystal_find : [] { return p3b::find_hidraw(discovery::Roots::from_env()); };
    cd.open = deps_.crystal_open ? deps_.crystal_open : open_hidraw;
    cd.process_running = deps_.process_running;
    cd.start_steamvr = deps_.start_steamvr ? deps_.start_steamvr : [] {
        return start_detached(in_own_scope({"steam", "steam://rungameid/250820"}));
    };
    // the app next to this binary (~/.local/bin or /usr/bin): systemd's PATH need not have ~/.local/bin
    cd.start_app = deps_.start_app ? deps_.start_app : [] {
        std::error_code ec;
        const auto self = std::filesystem::read_symlink("/proc/self/exe", ec);
        return !ec && start_detached(in_own_scope({(self.parent_path() / "pibridge").string()}));
    };
    cd.signal_steamvr = deps_.signal_steamvr ? deps_.signal_steamvr : [](int sig) { signal_steamvr_processes(sig); };
    cd.lighthouse_module = deps_.crystal_lighthouse_module ? deps_.crystal_lighthouse_module : [] { return lighthouse_module_present(); };
    cd.say = deps_.say;
    cd.request = pibridge::status::runtime_dir() / "crystal-request";
    crystal_ = std::make_unique<Crystal>(std::move(cd));
    crystal_->set({opts_.crystal_tracking, opts_.crystal_autostart_steamvr});
}

Daemon::~Daemon() { stop(); }

int Daemon::wake_fd() const { return ev_->fd.get(); }

void Daemon::say(const std::string& m) const { deps_.say(m); }

bool Daemon::busy() const {
    if (!opts_.video) {
        return true; // plain keepalive: always busy at the rate file's value
    }
    return handover_until_.has_value() || !follow_ || follow_->active();
}

// Write the keepalive rate atomically (the session re-reads the file on every keepalive); nullopt = `off`.
bool Daemon::write_rate(std::optional<std::uint8_t> fps) {
    const fs::path tmp = fs::path(opts_.rate_file).replace_extension("tmp");
    {
        std::ofstream f(tmp, std::ios::trunc);
        f << (fps ? hz(*fps) : std::string("off")) << "\n";
        if (!f) {
            error_ = "cannot write the rate file " + opts_.rate_file.string();
            return false;
        }
    }
    std::error_code ec;
    fs::rename(tmp, opts_.rate_file, ec);
    if (ec) {
        error_ = "cannot write the rate file " + opts_.rate_file.string() + ": " + ec.message();
        return false;
    }
    return true;
}

std::expected<void, std::string> Daemon::start(std::unique_ptr<Backend> backend, Clock::time_point now) {
    if (opts_.video && !opts_.follow.empty()) {
        follow_running_ = deps_.process_running(opts_.follow);
        follow_.emplace(follow_running_);
    }
    const bool busy_at_start = busy();
    if (opts_.video) {
        // the keepalive reads its rate from this file on every keepalive
        if (!write_rate(busy_at_start ? opts_.video : std::nullopt)) {
            return std::unexpected(error_);
        }
    }
    if (follow_) {
        say("following " + opts_.follow + ": " +
            (busy_at_start ? "running, headset on" : "not running, headset idle until it starts"));
    }
    if (opts_.video && opts_.audio && busy_at_start) {
        audio_on();
    }
    say("waiting for the headset (Crystal Super 34a4:0040 or Dream Air 34a4:0044)");
    Config cfg;
    cfg.wake = opts_.wake;
    cfg.rate_file = opts_.rate_file;
    cfg.allowlist = deps_.allowlist;
    cfg.devices_dirs = deps_.devices_dirs;
    cfg.log = deps_.say;
    auto s = Session::open_with_backend(std::move(cfg), std::move(backend), std::make_unique<H>(ev_));
    if (!s) {
        return std::unexpected("cannot start the session: " + s.error().message());
    }
    session_ = std::move(*s);
    write_status(now, true);
    return {};
}

void Daemon::audio_on() {
    if (audio_before_) {
        return;
    }
    auto [m, b] = audio::to_headset(deps_.pactl, opts_.audio_output, opts_.audio_input);
    say(m);
    audio_before_ = b;
}

void Daemon::audio_off() {
    if (!audio_before_) {
        return;
    }
    say(audio::restore(*audio_before_, deps_.pactl));
    audio_before_.reset();
}

void Daemon::go_active(Clock::time_point now, bool log) {
    if (log) {
        say(opts_.follow + " started: waking the headset");
    }
    // out of the green standby the way Pimax Play starts: the wake frame first, then the rate
    if (session_ && connected_) {
        if (auto w = session_->send_report(wake()); !w) {
            say("wake failed: " + w.error().message());
        }
    }
    write_rate(opts_.video);
    video_at_ = now + VIDEO_DELAY;
    audio_with_video_ = opts_.audio;
}

void Daemon::go_idle() {
    say(opts_.follow + " gone for 5 s: the headset goes idle (one keepalive with rate 0, then none)");
    write_rate(std::nullopt);
    video_at_.reset();
    audio_with_video_ = false;
    panel_on_ = false;
    audio_off();
}

void Daemon::track_volume(Clock::time_point now) {
    if (deps_.volume_file.empty() || (volume_at_ && now - *volume_at_ < VOLUME_EVERY)) {
        return;
    }
    volume_at_ = now;
    const auto sink = audio::headset_sink(deps_.pactl);
    if (!sink) {
        volume_sink_.clear();
        return;
    }
    std::string saved;
    if (std::ifstream f(deps_.volume_file); f) {
        std::getline(f, saved);
    }
    if (*sink != volume_sink_) {
        volume_sink_ = *sink;
        if (!saved.empty() && audio::sink_volume(*sink, deps_.pactl) != saved &&
            audio::set_sink_volume(*sink, saved, deps_.pactl)) {
            say("audio: headset volume " + saved + " put back");
        }
        return;
    }
    const auto v = audio::sink_volume(*sink, deps_.pactl);
    if (v && *v != saved) {
        std::error_code ec;
        std::filesystem::create_directories(deps_.volume_file.parent_path(), ec);
        const auto tmp = deps_.volume_file.string() + ".tmp";
        if (std::ofstream o(tmp); o && (o << *v << '\n')) {
            o.close();
            std::filesystem::rename(tmp, deps_.volume_file, ec);
        }
    }
}

void Daemon::read_volume_button_file() {
    std::filesystem::path p = deps_.volume_button_file;
    if (p.empty()) {
        const char* rt = std::getenv("XDG_RUNTIME_DIR");
        p = std::filesystem::path(rt != nullptr && rt[0] != '\0' ? rt : "/tmp") / "pibridge/volume-button";
    }
    std::ifstream f(p);
    unsigned long seq = 0;
    int dir = 0;
    int pressed = 0;
    if (!(f >> seq >> dir >> pressed) || (dir != 1 && dir != -1)) {
        return;
    }
    const bool first = !button_seq_;
    if (!first && seq == *button_seq_) {
        return;
    }
    button_seq_ = seq;
    if (first) {
        return; // an entry from before this daemon looked: not a press now
    }
    std::lock_guard g(ev_->mu);
    if (pressed != 0) {
        ev_->volume_steps += dir;
        ev_->volume_held = dir;
    } else if (ev_->volume_held == dir) {
        ev_->volume_held = 0;
    }
}

void Daemon::volume_buttons(Clock::time_point now) {
    int steps = 0;
    int held = 0;
    {
        std::lock_guard g(ev_->mu);
        steps = std::exchange(ev_->volume_steps, 0);
        held = ev_->volume_held;
    }
    if (held != hold_dir_) {
        hold_dir_ = held;
        hold_since_ = last_repeat_ = now;
    } else if (held != 0 && now - hold_since_ >= VOLUME_REPEAT_DELAY && now - last_repeat_ >= VOLUME_REPEAT) {
        if (now - hold_since_ < VOLUME_HOLD_MAX) {
            steps += held;
            last_repeat_ = now;
        }
    }
    if (steps == 0) {
        return;
    }
    const auto sink = audio::headset_sink(deps_.pactl);
    if (!sink) {
        say("audio: volume button, but no headset output");
        return;
    }
    if (const auto v = audio::step_sink_volume(*sink, steps, deps_.pactl); v && held == 0) {
        say("audio: headset volume " + *v + " (button)");
    }
}

std::chrono::milliseconds Daemon::poll_timeout() const {
    if (hold_dir_ != 0) {
        return std::chrono::milliseconds(20);
    }
    // paused: the volume buttons come from Monado's file, looked at on every tick
    return paused_ ? std::chrono::milliseconds(50) : std::chrono::milliseconds(500);
}

// The controllers' own serials, once per connection (ctrl_serials.h): for the app, which SteamVR cannot tell them.
void Daemon::read_controller_serials(Clock::time_point now) {
    if (!session_ || !session_->is_connected()) {
        serial_read_ = {false, false};
        headset_serials_read_ = false;
        return;
    }
    if (!headset_serials_read_) {
        headset_serials_read_ = true; // the headset's and its optical module's, once per connection
        for (const std::uint8_t slot : {std::uint8_t{0}, std::uint8_t{5}}) {
            if (auto sn = session_->read_serial(slot, std::chrono::milliseconds(800)); sn && !sn->empty()) {
                hmd::ctrl_serials::store(slot, *sn);
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        const std::uint8_t slot = i == 0 ? SLOT_LEFT : SLOT_RIGHT;
        if (!seen(slot, now)) {
            serial_read_[i] = false;
            continue;
        }
        if (serial_read_[i]) {
            continue;
        }
        serial_read_[i] = true; // one try per connection
        if (auto sn = session_->read_serial(slot, std::chrono::milliseconds(800)); sn && !sn->empty()) {
            hmd::ctrl_serials::store(slot, *sn);
        }
    }
}

void Daemon::tick(Clock::time_point now) {
    crystal_->tick(now); // the original Crystal: its own life, apart from the Crystal Super / Dream Air session
    // its sound (DisplayPort) and microphone are the defaults while it is on, as the Crystal Super's during VR
    const bool crystal_on = crystal_->stage() == "ready" && opts_.audio;
    if (crystal_on && !crystal_audio_) {
        audio_on();
        crystal_audio_ = true;
        crystal_audio_retry_ = now + std::chrono::seconds(3);
    } else if (!crystal_on && crystal_audio_) {
        audio_off();
        crystal_audio_ = false;
    } else if (crystal_on && opts_.audio_output && audio_before_ && !audio_before_->our_sink &&
               now >= crystal_audio_retry_) {
        // its sound output exists only once the GPU drives its display (SteamVR's compositor, with the non-desktop
        // EDID the desktop does not): try again until it is there
        crystal_audio_retry_ = now + std::chrono::seconds(3);
        auto [m, b] = audio::to_headset(deps_.pactl);
        if (b.our_sink) {
            say(m);
            audio_before_->sink = b.sink;
            audio_before_->our_sink = b.our_sink;
            audio_before_->card = b.card;
            audio_before_->card_profile = b.card_profile;
            audio_before_->our_card_profile = b.our_card_profile;
        }
    }
    if (paused_) {
        read_volume_button_file();
    }
    volume_buttons(now);
    track_volume(now);
    if (paused_ && !stopped_) {
        controllers(now); // pair / unpair refused with the reason; the vibration file is Monado's
        write_status(now, false); // the status file stays fresh while paused
        return;
    }
    if (stopped_ || !session_) {
        return;
    }
    eventfd_drain(ev_->fd.get());
    int connects = 0;
    int disconnects = 0;
    {
        std::lock_guard g(ev_->mu);
        connects = std::exchange(ev_->connects, 0);
        disconnects = std::exchange(ev_->disconnects, 0);
    }
    if (disconnects > 0) {
        connected_ = session_->is_connected();
        panel_on_ = false;
        video_at_.reset();
    }
    if (connects > 0) {
        connected_ = session_->is_connected();
        const Model* m = session_->model();
        const auto found = discovery::find_headset(deps_.roots);
        const std::string node = found && found->nodes[0] ? found->nodes[0]->string() : std::string();
        say("headset" + (node.empty() ? std::string() : " on " + node) + (m ? " (" + std::string(m->name) + ")" : "") +
            ", sending wake + keepalive");
        // a (re)connect: the panel only while busy; idle, the rate file already says off
        if (opts_.video) {
            if (busy()) {
                video_at_ = now + VIDEO_DELAY;
            } else {
                say("idle: one keepalive with rate 0, then none (standby)");
            }
        }
    }
    if (follow_) {
        follow_running_ = deps_.process_running(opts_.follow);
        if (const auto change = follow_->step(now, follow_running_)) {
            if (*change == Follow::Change::Active) {
                go_active(now);
            } else {
                go_idle();
            }
        }
    }
    controllers(now);
    read_controller_serials(now);
    if (video_at_ && now >= *video_at_) {
        video_at_.reset();
        if (busy() && opts_.video) {
            if (session_->is_connected()) {
                const auto r = session_->video_prepare(*opts_.video);
                if (r) {
                    panel_on_ = true;
                    video_on_at_ = now;
                    if (error_.starts_with("video on")) {
                        error_.clear();
                    }
                    say("video on at " + hz(*opts_.video) + " Hz");
                } else {
                    error_ = "video on at " + hz(*opts_.video) + " Hz failed: " + r.error().message();
                    say(error_);
                }
            }
            if (audio_with_video_) {
                audio_with_video_ = false;
                audio_on();
            }
        }
    }
    write_status(now, false);
}

void Daemon::reload(Options o, Clock::time_point now) {
    crystal_->set({o.crystal_tracking, o.crystal_autostart_steamvr});
    if (o.rate_file != opts_.rate_file || o.wake != opts_.wake) {
        say("settings: the rate file and the wake change at the next start");
        o.rate_file = opts_.rate_file;
        o.wake = opts_.wake;
    }
    if (paused_) {
        const bool audio_changed = o.audio_output != opts_.audio_output || o.audio_input != opts_.audio_input;
        opts_ = std::move(o); // applied when the session opens again
        if (audio_changed && audio_before_) {
            audio_off(); // the sound stays on the headset while paused: switched again with the new choice
            if (opts_.video && opts_.audio) {
                audio_on();
            }
        }
        say("settings reloaded (paused)");
        write_status(now, true);
        return;
    }
    const bool was_busy = busy();
    const auto old_video = opts_.video;
    const std::string old_follow = opts_.follow;
    const bool audio_changed = o.audio_output != opts_.audio_output || o.audio_input != opts_.audio_input;
    opts_ = std::move(o);
    say("settings reloaded");
    if (opts_.video && !opts_.follow.empty()) {
        follow_running_ = deps_.process_running(opts_.follow);
        if (!follow_ || opts_.follow != old_follow) {
            follow_.emplace(follow_running_); // a new process to follow: its state now, no grace
        }
    } else {
        follow_.reset();
        follow_running_ = false;
    }
    const bool now_busy = busy();
    if (opts_.video) {
        if (now_busy && (!was_busy || old_video != opts_.video)) {
            write_rate(opts_.video);
            if (session_ && session_->is_connected()) {
                if (!was_busy) {
                    (void)session_->send_report(wake());
                }
                video_at_ = now + VIDEO_DELAY;
            }
        } else if (!now_busy && was_busy) {
            write_rate(std::nullopt);
            video_at_.reset();
            panel_on_ = false;
        }
    } else {
        // plain keepalive: the panel is left as it is and the rate file is read as others wrote it
        video_at_.reset();
    }
    if (opts_.video && opts_.audio && now_busy) {
        if (audio_changed) {
            audio_off(); // the other choice of output / input: back first, then switched again
        }
        audio_on();
    } else {
        audio_off();
    }
    write_status(now, true);
}

void Daemon::close_session(bool idle_report) {
    if (session_) {
        if (idle_report && opts_.video && busy() && session_->is_connected()) {
            // the idle report Pimax Play sends when it closes: the last keepalive carries rate 0
            write_rate(std::nullopt);
            if (auto w = session_->send_report(keepalive(0)); !w) {
                say("idle report failed: " + w.error().message());
            }
        }
        session_->close();
        session_.reset();
    }
    audio_off();
    connected_ = false;
    panel_on_ = false;
    video_at_.reset();
}

void Daemon::pause(Clock::time_point now) {
    if (paused_ || stopped_) {
        return;
    }
    const bool can_wake = opts_.video && session_ && connected_ && session_->is_connected();
    if (!handover_until_ && can_wake && !panel_on_) {
        say("inside-out tracking starts: waking the headset first (video on), then it is left to Monado");
        go_active(now, false);
        handover_until_ = now + HANDOVER_MAX;
        write_status(now, true);
        return;
    }
    if (handover_until_) {
        // released once the video is on and the headset had time to settle on USB, or at the limit regardless
        if (now < *handover_until_ && (!panel_on_ || !video_on_at_ || now < *video_on_at_ + HANDOVER_SETTLE)) {
            return;
        }
        handover_until_.reset();
    }
    close_session(false);
    if (opts_.video && opts_.audio) {
        audio_on(); // the sound stays on the headset during inside-out tracking (pactl only, no HID)
    }
    paused_ = true;
    say(std::string(pibridge::pause::kWords) + ": the headset is left to Monado's Pimax driver");
    write_status(now, true);
}

std::expected<void, std::string> Daemon::resume(std::unique_ptr<Backend> backend, Clock::time_point now) {
    if (!paused_ || stopped_) {
        return {};
    }
    paused_ = false;
    say("inside-out tracking ended: taking the headset again");
    audio_off(); // the sound kept on the headset during the pause goes back to where it was before
    return start(std::move(backend), now);
}

void Daemon::stop() {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    close_session();
    if (!deps_.status_name.empty()) {
        std::error_code ec;
        fs::remove(pibridge::status::runtime_dir() / (deps_.status_name + ".json"), ec);
    }
    say("stopped");
}

Status Daemon::status() const {
    Status s;
    s.follow = opts_.follow;
    if (ctrl_) {
        s.ctrl_id = ctrl_->id;
        s.ctrl_op = std::string(pairing::op_name(ctrl_->op));
        s.ctrl_side = std::string(pairing::side_of(ctrl_->slot));
        s.ctrl_state = ctrl_->state;
        s.ctrl_detail = ctrl_->detail;
    }
    s.haptic_seq = haptic_seq_;
    s.haptic_result = haptic_result_;
    s.crystal_present = crystal_->present();
    s.crystal_stage = std::string(crystal_->stage());
    s.crystal_lighthouse = crystal_->lighthouse();
    s.crystal_tracked = crystal_->tracked();
    s.crystal_battery = crystal_->battery();
    if (paused_) {
        s.state = "paused";
        s.paused = std::string(pibridge::pause::kWords);
        s.follow_running = follow_ && follow_running_;
        return s;
    }
    s.follow_running = follow_ && follow_running_;
    s.audio_on_headset = audio_before_ && (audio_before_->our_sink || audio_before_->our_source);
    s.lens_ipd_x100 = ev_->lens_ipd_x100.load(std::memory_order_relaxed);
    if (const int c = crystal_->ipd_x100(); c > 0) {
        s.lens_ipd_x100 = c; // the original Crystal: its IPD buttons move the lenses, the slider follows
    }
    s.error = error_;
    const bool present = session_ && session_->is_connected();
    s.headset_present = present;
    if (session_) {
        if (const Model* m = session_->model()) {
            s.model = std::string(m->name);
        }
        if (!present && s.error.empty()) {
            s.error = session_->last_error();
        }
    }
    s.panel_on = present && panel_on_;
    s.left_connected = present && seen(SLOT_LEFT, Clock::now());
    s.right_connected = present && seen(SLOT_RIGHT, Clock::now());
    if (opts_.video) {
        s.rate_hz = busy() ? *opts_.video : 0;
    } else {
        std::ifstream f(opts_.rate_file);
        int v = 0;
        if (f >> v) {
            s.rate_hz = v;
        }
    }
    if (!present) {
        s.state = s.error.empty() ? "no-headset" : "error";
    } else if (!busy()) {
        s.state = "idle";
    } else if (opts_.video && !panel_on_) {
        s.state = "waking";
    } else {
        s.state = "active";
    }
    return s;
}

void Daemon::write_status(Clock::time_point now, bool force) {
    if (deps_.status_name.empty()) {
        return;
    }
    const Status s = status();
    if (!force && last_status_ == s && last_write_ && now - *last_write_ < STATUS_EVERY) {
        return;
    }
    pibridge::status::StatusWriter w;
    w.str("state", s.state)
        .str("model", s.model)
        .boolean("headset_present", s.headset_present)
        .boolean("panel_on", s.panel_on)
        .integer("rate_hz", s.rate_hz)
        .str("follow", s.follow)
        .boolean("follow_running", s.follow_running)
        .boolean("audio_on_headset", s.audio_on_headset)
        .str("error", s.error)
        .str("paused", s.paused)
        .integer("ctrl_id", static_cast<long long>(s.ctrl_id))
        .str("ctrl_op", s.ctrl_op)
        .str("ctrl_side", s.ctrl_side)
        .str("ctrl_state", s.ctrl_state)
        .str("ctrl_detail", s.ctrl_detail)
        .boolean("left_connected", s.left_connected)
        .boolean("right_connected", s.right_connected)
        .integer("haptic_seq", static_cast<long long>(s.haptic_seq))
        .str("haptic_result", s.haptic_result)
        .integer("lens_ipd_x100", s.lens_ipd_x100)
        .boolean("crystal_present", s.crystal_present)
        .str("crystal_stage", s.crystal_stage)
        .str("crystal_lighthouse", s.crystal_lighthouse)
        .boolean("crystal_tracked", s.crystal_tracked)
        .integer("crystal_battery", s.crystal_battery)
        .integer("pid", static_cast<long long>(::getpid()));
    w.write(deps_.status_name);
    last_status_ = s;
    last_write_ = now;
}

// ------------------------------------------------------------------------------------------------ controllers

fs::path Daemon::ctrl_request_path() const {
    return deps_.ctrl_request.empty() ? pibridge::status::runtime_dir() / pairing::REQUEST_FILE : deps_.ctrl_request;
}

fs::path Daemon::haptic_request_path() const {
    return deps_.haptic_request.empty() ? haptics::request_path() : deps_.haptic_request;
}

bool Daemon::seen(std::uint8_t slot, Clock::time_point now) const {
    if (!is_slot(slot)) {
        return false;
    }
    std::lock_guard g(ev_->mu);
    const auto& t = ev_->heard[slot - 2];
    return t && now - *t <= CONTROLLER_SEEN_FOR;
}

void Daemon::controllers(Clock::time_point now) {
    if (stopped_) {
        return;
    }
    if (auto r = pairing::take_request(ctrl_request_path()); !r) {
        say("controller request: " + r.error());
    } else if (*r) {
        controller_request(**r, now);
    }
    pair_progress(now);
    if (paused_) {
        return; // the vibration and IPD requests are taken by Monado's Pimax driver during an inside-out session
    }
    if (auto r = ipd::take_request(ipd::request_path()); !r) {
        say("IPD request: " + r.error());
    } else if (*r) {
        if (!session_ || !session_->is_connected()) {
            say("IPD request: no headset");
        } else if (auto w = session_->send_report(ipd_motor(**r)); !w) {
            say("IPD motor not moved: " + w.error().message());
        } else {
            char b[64];
            std::snprintf(b, sizeof b, "IPD motor set to %.1f mm", static_cast<double>(**r));
            say(b);
        }
    }
    if (auto h = haptics::take_request(haptic_request_path()); !h) {
        haptic_seq_++;
        haptic_result_ = "The vibration request was not understood: " + h.error();
        say(haptic_result_);
    } else if (*h) {
        const auto [slot, amp, ms] = **h;
        vibration_test(slot, amp, ms, now);
    }
}

void Daemon::controller_request(const pairing::Request& r, Clock::time_point now) {
    const std::string side(pairing::side_of(r.slot));
    const std::string what(pairing::op_name(r.op));
    CtrlJob j{r.id, r.op, r.slot, "failed", "", now};
    const auto done = [&](std::string state, std::string detail) {
        j.state = std::move(state);
        j.detail = std::move(detail);
        ctrl_ = j;
        say(what + " " + side + ": " + j.state + ": " + j.detail);
    };
    if (paused_) {
        return done("refused", "Inside-out tracking is running and Monado's Pimax driver owns the headset. Stop VR "
                               "first, then " + what + " the controller.");
    }
    if (!session_ || !session_->is_connected()) {
        return done("failed", "No headset: switch the headset on and try again.");
    }
    if (pairing_) {
        return done("refused", "Pairing of the " + std::string(pairing::side_of(pairing_->slot)) +
                                   " controller is still running: wait until it ends.");
    }
    if (r.op == pairing::Op::Pair) {
        if (seen(r.slot, now)) {
            return done("refused", "The " + side + " controller is already paired and connected. Press Unpair first "
                                   "to pair it again.");
        }
        {
            std::lock_guard g(ev_->mu);
            ev_->log.clear(); // only verdicts after the send count
        }
        if (auto w = session_->send_report(start_pair(r.slot)); !w) {
            return done("failed", "The pair command was not sent: " + w.error().message());
        }
        j.state = "waiting";
        j.detail = std::string(pairing::INSTRUCTION) + ".";
        ctrl_ = j;
        pairing_ = j;
        say("pair " + side + ": StartPair sent, waiting for the controller (" +
            std::to_string(pairing::TIMEOUT.count()) + " s)");
        return;
    }
    const std::uint8_t slot = r.slot;
    auto rep = session_->request_with(unpair(slot), pairing::UNPAIR_REPLY,
                                      [slot](const Report& d) { return d[2] == cmd::UNPAIR && d[3] == slot; });
    if (!rep) {
        if (rep.error().code == Errc::Timeout) {
            return done("failed", "The headset did not answer the unpair command (no reply in 1 s).");
        }
        return done("failed", "The unpair command was not sent: " + rep.error().message());
    }
    if ((*rep)[4] != 0x01) {
        return done("failed", "The headset refused to unpair the " + side + " controller.");
    }
    {
        std::lock_guard g(ev_->mu);
        ev_->heard[slot - 2].reset();
        ev_->streamed[slot - 2].reset();
    }
    done("unpaired", "The " + side + " controller was unpaired. Put it in pairing mode and press Pair to connect it "
                     "again.");
}

void Daemon::pair_progress(Clock::time_point now) {
    std::deque<std::string> lines;
    std::optional<Clock::time_point> heard;
    {
        std::lock_guard g(ev_->mu);
        lines.swap(ev_->log);
        if (pairing_) {
            heard = ev_->heard[pairing_->slot - 2];
        }
    }
    if (!pairing_) {
        return;
    }
    const std::string side(pairing::side_of(pairing_->slot));
    std::optional<pairing::LogEvent> verdict;
    for (const auto& l : lines) {
        const auto e = pairing::parse_log(l);
        if (e && *e != pairing::LogEvent::Started) {
            verdict = e;
            break;
        }
    }
    const auto end = [&](std::string state, std::string detail) {
        pairing_->state = std::move(state);
        pairing_->detail = std::move(detail);
        ctrl_ = *pairing_;
        say("pair " + side + ": " + ctrl_->state + ": " + ctrl_->detail);
        pairing_.reset();
    };
    if (verdict == pairing::LogEvent::Success || (heard && *heard > pairing_->started)) {
        end("paired", "The " + side + " controller is paired.");
    } else if (verdict == pairing::LogEvent::TimeoutOrCancel || now - pairing_->started >= pairing::TIMEOUT) {
        end("timeout", "No controller paired within 30 s. Put the " + side +
                           " controller in pairing mode, then press Pair again.");
    } else if (!session_ || !session_->is_connected()) {
        end("failed", "The headset went away during pairing.");
    }
}

void Daemon::vibration_test(std::uint8_t slot, std::uint8_t amp, std::uint16_t ms, Clock::time_point now) {
    haptic_seq_++;
    const std::string side(pairing::side_of(slot));
    if (!session_ || !session_->is_connected()) {
        haptic_result_ = "No headset: switch the headset on and try again.";
    } else if (!seen(slot, now)) {
        haptic_result_ = "The " + side + " controller is not connected: switch it on (or pair it) and try again.";
    } else {
        bool streaming = false;
        {
            std::lock_guard g(ev_->mu);
            const auto& t = ev_->streamed[slot - 2];
            streaming = t && now - *t <= CONTROLLER_STREAM_GAP;
        }
        // a controller that is not streaming gets the attach burst first, the way Pimax Play starts it
        if (!streaming) {
            if (auto a = session_->controller_attach(slot); !a) {
                say("attach " + side + " before the test pulse: " + a.error().message());
            }
        }
        if (session_->haptic(slot, haptics::Pulse{amp, ms})) {
            haptic_result_ = "Vibration sent to the " + side + " controller.";
        } else {
            haptic_result_ = "The vibration was not sent (unknown controller slot).";
        }
    }
    say("vibration test " + side + ": " + haptic_result_);
}

} // namespace pibridge::keepalived
