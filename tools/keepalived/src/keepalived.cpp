// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/keepalived.rs (MIT OR Apache-2.0).
#include "keepalived.h"

#include "pibridge_pause.h"

#include "pibridge/hmd/discovery.h"
#include "pibridge/hmd/haptics.h"
#include "pibridge_status.h"

#include <deque>

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
    o.audio = settings.audio_to_headset;
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
            o.audio = true;
            audio_flag = true;
        } else if (a == "--no-audio") {
            o.audio = false;
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
    /// Last report / heartbeat of slot 2 and 3, and the last controller report alone.
    std::array<std::optional<Clock::time_point>, 2> heard;
    std::array<std::optional<Clock::time_point>, 2> streamed;
};

namespace {

class H : public Handler {
public:
    explicit H(std::shared_ptr<Daemon::Events> ev) : ev_(std::move(ev)) {}
    void on_state(State s) override {
        {
            std::lock_guard g(ev_->mu);
            (s == State::Connected ? ev_->connects : ev_->disconnects)++;
        }
        eventfd_signal(ev_->fd.get());
    }
    void on_log(std::string_view l) override {
        std::lock_guard g(ev_->mu);
        ev_->log.emplace_back(l);
        if (ev_->log.size() > 64) {
            ev_->log.pop_front();
        }
    }
    void on_status(const ControllerStatus& s) override { heard(s.slot, false); }
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
    auto [m, b] = audio::to_headset(deps_.pactl);
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

void Daemon::tick(Clock::time_point now) {
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
    if (o.rate_file != opts_.rate_file || o.wake != opts_.wake) {
        say("settings: the rate file and the wake change at the next start");
        o.rate_file = opts_.rate_file;
        o.wake = opts_.wake;
    }
    if (paused_) {
        opts_ = std::move(o); // applied when the session opens again
        say("settings reloaded (paused)");
        write_status(now, true);
        return;
    }
    const bool was_busy = busy();
    const auto old_video = opts_.video;
    const std::string old_follow = opts_.follow;
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
    if (paused_) {
        s.state = "paused";
        s.paused = std::string(pibridge::pause::kWords);
        s.follow_running = follow_ && follow_running_;
        return s;
    }
    s.follow_running = follow_ && follow_running_;
    s.audio_on_headset = audio_before_ && (audio_before_->our_sink || audio_before_->our_source);
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
        return; // the vibration request is taken by Monado's Pimax driver during an inside-out session
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
