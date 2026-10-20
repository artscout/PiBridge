// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/keepalived.rs (MIT OR Apache-2.0).
//
// pibridge-keepalived: waits for the headset, sends the wake, then `58 04 39 00 RR 00` every 213 ms (RR from the rate
// file), keeps IF1-IF3 open and drained, and starts over whenever the device re-enumerates.
//
// With a video rate (`--video FPS`, default `refresh_hz` of pibridge.conf) it switches the panel on at every
// connect, for SteamVR drivers that do not do it themselves (the Crystal Super with its Lighthouse faceplate).
//
// `--follow PROCESS` (default `follow` of pibridge.conf, SteamVR's `vrserver`; empty = always on): busy only while
// PROCESS runs, the way Pimax Play is busy only while it runs. While it does not run: one keepalive with rate 0 and
// then none (the rate file says `off`; the session keeps reading the headset), and the headset goes to its green
// standby. When it starts: wake, rate FPS and, 2 s later, the video frames; the LED turns blue. It goes idle again
// 5 s after the process is gone.
//
// `--audio` (default `audio_to_headset`): while busy, the headset's USB audio is the default output and input; when
// idle again the previous defaults come back, unless the user picked another device meanwhile.
//
// Controllers (docs/keepalived.md "Controllers"): the app's Pair / Unpair buttons write one line to
// `$XDG_RUNTIME_DIR/pibridge/controller-request` (libpibridge-hmd pairing.h), its Test vibration button writes
// `$XDG_RUNTIME_DIR/pimax-haptic-request` (haptics.h, the file Monado's Pimax driver takes during an inside-out
// session). The daemon takes them on its own session and reports the result in keepalived.json (`ctrl_*`,
// `haptic_*`, `left_connected` / `right_connected`). While paused, pair and unpair are refused with a plain reason and
// the vibration file is left to Monado.
//
// The state machine is this class; main.cpp only wires signals and the clock. Tests drive it with a fake headset,
// a fake process table and a fake pactl.
#pragma once

#include "pibridge/hmd/audio.h"
#include "pibridge/hmd/discovery.h"
#include "pibridge/hmd/follow.h"
#include "pibridge/hmd/pairing.h"
#include "pibridge/hmd/session.h"
#include "pibridge/hmd/settings.h"

#include <chrono>
#include <expected>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::keepalived {

// The daemon is built on libpibridge-hmd (settings, discovery, session, follower, audio): its names unqualified here.
using namespace pibridge::hmd;

using Clock = std::chrono::steady_clock;

/// Wait between the connect (or the process start) and the video-prepare frames: Pimax Play announces the rate
/// about 4 s after its wake, after its identity and calibration reads; the wake itself needs about 1 s.
inline constexpr std::chrono::seconds VIDEO_DELAY{2};
/// The pause's hand-over: the headset settles this long after its video is on before it is released, and the
/// hand-over gives up waiting for the video after HANDOVER_MAX (released anyway).
inline constexpr std::chrono::seconds HANDOVER_SETTLE{3};
inline constexpr std::chrono::seconds HANDOVER_MAX{10};
/// A controller counts as connected this long after its last report or 0x37 heartbeat (the heartbeat comes about
/// every 4 s while it is awake and not streaming: Crystal Super capture 2026-10-17, take cold2).
inline constexpr std::chrono::seconds CONTROLLER_SEEN_FOR{10};
/// The vibration test pulse first sends the attach burst when the controller has not streamed for this long.
inline constexpr std::chrono::milliseconds CONTROLLER_STREAM_GAP{1000};
/// The status file is rewritten at least this often.
inline constexpr std::chrono::seconds STATUS_EVERY{2};

struct Options {
    /// Send the wake after connecting (`--no-wake` turns it off).
    bool wake = true;
    /// The keepalive rate file (`--rate-file`, default $XDG_RUNTIME_DIR/pimax-rate).
    std::filesystem::path rate_file;
    /// Panel rate (72 | 90); nullopt = plain keepalive, the panel is left alone (`--no-video`).
    std::optional<std::uint8_t> video;
    /// Process to follow; empty = always on. Only used with a video rate.
    std::string follow;
    /// Headset audio while busy. Only used with a video rate.
    bool audio = false;
    /// The settings file the defaults came from (`--settings`).
    std::filesystem::path settings_path;
};

/// $XDG_RUNTIME_DIR/pimax-rate (/tmp/pimax-rate without it): the file omarchyvr uses, so both stacks agree.
std::filesystem::path default_rate_file();

/// Command line over the settings file: flags override `pibridge.conf`. An error is a usage message.
/// `--help` gives the error "help" (main prints usage()).
std::expected<Options, std::string> parse_args(const std::vector<std::string>& args, const Settings& settings);
/// The settings path named by `--settings` in `args`, or the default one.
std::filesystem::path settings_path_of(const std::vector<std::string>& args);
std::string usage();

/// What the daemon reports in keepalived.json (docs/CONVENTIONS.md).
struct Status {
    /// "idle" | "waking" | "active" | "no-headset" | "error" | "paused" (an inside-out session owns the headset).
    std::string state = "no-headset";
    std::string model;
    bool headset_present = false;
    bool panel_on = false;
    int rate_hz = 0;
    std::string follow;
    bool follow_running = false;
    bool audio_on_headset = false;
    std::string error;
    /// "paused: inside-out tracking" while paused, "" otherwise.
    std::string paused;
    /// The last controller request (pair / unpair): its id, "pair" | "unpair", "left" | "right", and
    /// "waiting" | "paired" | "timeout" | "unpaired" | "failed" | "refused" with the plain words in ctrl_detail.
    std::uint64_t ctrl_id = 0;
    std::string ctrl_op;
    std::string ctrl_side;
    std::string ctrl_state;
    std::string ctrl_detail;
    /// A report or heartbeat of that controller within CONTROLLER_SEEN_FOR.
    bool left_connected = false;
    bool right_connected = false;
    /// Vibration test requests taken so far and the words for the last one.
    std::uint64_t haptic_seq = 0;
    std::string haptic_result;
    bool operator==(const Status&) const = default;
};

/// Everything the daemon touches outside the headset, replaceable for tests.
struct Deps {
    /// Is a process with this name running (/proc/<pid>/comm)?
    std::function<bool(std::string_view)> process_running = [](std::string_view n) { return pibridge::hmd::process_running(n); };
    /// Runs pactl.
    audio::Runner pactl = audio::run_pactl;
    /// One log line (stdout with a HH:MM:SS prefix by default; the journal keeps it).
    std::function<void(std::string_view)> say;
    /// Name of the status file in $XDG_RUNTIME_DIR/pibridge/ (empty = do not write one).
    std::string status_name = "keepalived";
    /// The session's allowlist; null = the connected model's description from `devices_dirs` (empty = defaults).
    std::shared_ptr<const Allowlist> allowlist;
    std::vector<std::filesystem::path> devices_dirs;
    /// The app's controller request file; empty = $XDG_RUNTIME_DIR/pibridge/controller-request.
    std::filesystem::path ctrl_request;
    /// The vibration test request file; empty = $XDG_RUNTIME_DIR/pimax-haptic-request (haptics::request_path()).
    std::filesystem::path haptic_request;
    /// Where the connect log line looks up the device node (sysfs, read only).
    discovery::Roots roots = discovery::Roots::from_env();
};

class Daemon {
public:
    Daemon(Options opts, Deps deps);
    ~Daemon();
    Daemon(const Daemon&) = delete;
    Daemon& operator=(const Daemon&) = delete;

    /// Write the rate file, switch the audio if busy at start, and open the session on `backend`.
    std::expected<void, std::string> start(std::unique_ptr<Backend> backend, Clock::time_point now);
    /// Readable when the session reported a (dis)connect: call tick() soon.
    int wake_fd() const;
    /// Advance the state machine: connect events, the followed process, the delayed video frames, the status file.
    void tick(Clock::time_point now);
    /// New options after SIGHUP (the rate file and the wake take effect at the next start).
    void reload(Options opts, Clock::time_point now);
    /// Clean stop: a rate-0 keepalive if the headset was busy, the audio back, the session closed, the status file
    /// removed.
    void stop();
    /// An inside-out session owns the headset (common/pibridge_pause.h). First the hand-over: a sleeping headset is woken
    /// and its video turned on the way a SteamVR start does it (live 2026-10-19: when Monado's Pimax driver woke it
    /// itself, the headset re-enumerated on USB right under the driver, which inside vrserver then got no IMU), then
    /// the session is closed without the idle report (the headset stays awake for Monado), the audio given back and
    /// the status "paused"; the sound stays on the headset (pactl). Call it again from the loop until paused() (the hand-over takes a few seconds).
    void pause(Clock::time_point now);
    /// The marker is gone: open the session again on `backend` (as start()).
    std::expected<void, std::string> resume(std::unique_ptr<Backend> backend, Clock::time_point now);
    bool paused() const { return paused_; }

    Status status() const;
    bool busy() const;
    const Options& options() const { return opts_; }

    struct Events;

private:
    void say(const std::string& m) const;
    bool write_rate(std::optional<std::uint8_t> fps);
    void go_active(Clock::time_point now, bool log = true);
    void go_idle();
    void audio_on();
    void audio_off();
    void write_status(Clock::time_point now, bool force);

    Options opts_;
    Deps deps_;
    std::shared_ptr<Events> ev_;
    std::unique_ptr<Session> session_;
    std::optional<Follow> follow_;
    bool follow_running_ = false;
    bool connected_ = false;
    bool panel_on_ = false;
    std::optional<Clock::time_point> video_at_;
    /// Switch the audio when the delayed video frames go out (the order of the Rust daemon).
    bool audio_with_video_ = false;
    std::optional<audio::Before> audio_before_;
    std::string error_;
    std::optional<Status> last_status_;
    std::optional<Clock::time_point> last_write_;
    bool stopped_ = false;
    bool paused_ = false;
    /// The pause's hand-over: until when the headset is kept busy (woken, video on) before it is released.
    std::optional<Clock::time_point> handover_until_;
    std::optional<Clock::time_point> video_on_at_;
    void close_session(bool idle_report = true);

    // controllers: pair / unpair / vibration test
    struct CtrlJob {
        std::uint64_t id = 0;
        pairing::Op op = pairing::Op::Pair;
        std::uint8_t slot = 0;
        std::string state;
        std::string detail;
        Clock::time_point started{};
    };
    void controllers(Clock::time_point now);
    void controller_request(const pairing::Request& r, Clock::time_point now);
    void pair_progress(Clock::time_point now);
    void vibration_test(std::uint8_t slot, std::uint8_t amp, std::uint16_t ms, Clock::time_point now);
    bool seen(std::uint8_t slot, Clock::time_point now) const;
    std::filesystem::path ctrl_request_path() const;
    std::filesystem::path haptic_request_path() const;
    /// The last request shown in the status; the pairing that still waits for the firmware.
    std::optional<CtrlJob> ctrl_;
    std::optional<CtrlJob> pairing_;
    std::uint64_t haptic_seq_ = 0;
    std::string haptic_result_;
};

} // namespace pibridge::keepalived
