// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/eyed.rs (MIT OR Apache-2.0).
//
// pibridge-eyed: runs the eye tracker on its own (no Monado) and publishes the gaze in shared memory (pibridge::eye::shm,
// `/dev/shm/omarchyvr-eye-gaze`) for the SteamVR driver: the setup of the Crystal Super (SteamVR Lighthouse +
// CustomHeadsetOpenVR), whose SteamVR eye tracking component then serves OpenXR `XR_EXT_eye_gaze_interaction`.
//
// Options (defaults from pibridge.conf): `--frames FILE` (the start list made by `pibridge-eye frames` from a Windows
// capture of this headset; needed for headsets without a built-in list, the Crystal Super), `--profile FILE` (the
// user's current eye profile, made by `pibridge-eye profile` from Pimax's calibration.setpm, uploaded in place of the
// list's own; a missing file is skipped), `--no-profile` (no eye profile upload at all), `--shm PATH`,
// `--follow PROCESS`. The environment of the live tracker applies (PIBRIDGE_EYE_*, eye-calibration.toml). The eye module
// is powered with the captured enable when the headset is there and the chip is not. Only one program may own the
// chip (omarchyvr-eyed, Monado's eye tracking and pimax-eyecal must not run at the same time).
//
// When the stream goes on but shows no eye for NO_EYES_ENABLE (the headset was taken off and put on again: the
// Crystal Super switches its eye module off when it is taken off, and Pimax Play sends the eye-module enable again
// when it is put back on), the captured enable `58 05 3c 05 00 01` is sent once for that stretch. Losing and finding
// the eyes is logged.
//
// `--follow PROCESS` (SteamVR's `vrserver`): the eye tracker runs only while PROCESS runs, like pibridge-keepalived: it
// starts with the runtime and stops (unsubscribes, releases the chip) 5 s after it is gone.
//
// The state machine is the Daemon class; main.cpp only wires signals and the clock. Tests drive it with FakeChip
// trackers, a fake headset command sink, a fake process table, a temporary shm path and temporary config dirs.
#pragma once

#include "pibridge/hmd/follow.h"
#include "pibridge/hmd/settings.h"

#include "pibridge/eye/live.h"
#include "pibridge/eye/power.h"
#include "pibridge/eye/shm.h"

#include <chrono>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eyed {

// The daemon is built on libpibridge-hmd (settings, discovery, session, follower, audio): its names unqualified here.
using namespace pibridge::hmd;

using Clock = std::chrono::steady_clock;

/// No eye for this long while streaming: send the eye-module enable once.
inline constexpr std::chrono::seconds NO_EYES_ENABLE{4};
/// The eyes count as lost after this long without one valid sample.
inline constexpr std::chrono::milliseconds EYES_LOST_AFTER{500};
/// The status file is rewritten at least this often.
inline constexpr std::chrono::seconds STATUS_EVERY{2};
/// The shared-memory file is rewritten at least this often without a new sample.
inline constexpr std::chrono::milliseconds SHM_EVERY{500};
/// The followed process is looked up this often.
inline constexpr std::chrono::milliseconds FOLLOW_EVERY{500};

/// File names of the Crystal Super's start list and eye profile in the config folders.
inline constexpr std::string_view START_LIST_FILE = "eye-start-crystal-super.txt";
inline constexpr std::string_view PROFILE_FILE = "eye-profile-crystal-super.bin";

/// The folders the default files are looked up in: pibridge first, then pimax-linux (PiBridge's earlier name), then
/// omarchyvr (each used only when the file is not in the ones before).
struct ConfigDirs {
    std::filesystem::path pibridge;
    std::filesystem::path legacy;
    std::filesystem::path omarchyvr;
    /// pibridge::hmd::Settings::config_dir() and its siblings `pimax-linux` and `omarchyvr`.
    static ConfigDirs from_env();
    /// The default file `name`: the first folder above that has it, else nullopt.
    std::optional<std::filesystem::path> find(std::string_view name) const;
};

struct Options {
    /// The start list; nullopt = the built-in base list.
    std::optional<std::filesystem::path> frames;
    /// The eye profile; nullopt = none (the list's own upload, if any).
    std::optional<std::filesystem::path> profile;
    /// No eye profile upload at all.
    bool no_profile = false;
    /// The gaze file.
    std::filesystem::path shm = pibridge::eye::shm::default_path();
    /// Process to follow; empty = always on.
    std::string follow;
    /// `eye_tracking` of pibridge.conf: off = the daemon stays idle (the app normally stops the unit instead).
    bool eye_tracking = true;
    /// `eye_flip_x` of pibridge.conf (the PIBRIDGE_EYE_FLIP_X environment wins when it is set).
    bool flip_x = true;
    /// The settings file the defaults came from (`--settings`).
    std::filesystem::path settings_path;

    bool operator==(const Options&) const = default;
};

/// Command line over the settings file: flags override `pibridge.conf`. An error is a usage message;
/// `--help` gives the error "help".
std::expected<Options, std::string> parse_args(const std::vector<std::string>& args, const Settings& settings,
                                               const ConfigDirs& dirs);
/// The settings path named by `--settings` in `args`, or the default one.
std::filesystem::path settings_path_of(const std::vector<std::string>& args);
std::string usage();

/// What eyed.json says (docs/CONVENTIONS.md).
struct Status {
    /// "idle" | "searching" | "connecting" | "streaming" | "error".
    std::string state = "idle";
    /// "user" (the user's profile is uploaded) | "list" (the list's own upload) | "none".
    std::string profile = "none";
    bool eyes_seen = false;
    double samples_per_s = 0;
    /// Path of the per-eye calibration in use, or "".
    std::string calibration;
    std::string follow;
    bool follow_running = false;
    /// Plain words, "" when none.
    std::string error;
    /// "paused: inside-out tracking" while paused (state "paused"), "" otherwise.
    std::string paused;
    bool operator==(const Status&) const = default;
};

/// The tracker configuration of `opts` on top of `base` (Config::from_env): the start list, the profile, flip x.
/// `profile_kind` is set to "user" | "list" | "none". An error is a plain-words reason (unreadable list, bad
/// profile); a missing profile file is only logged.
std::expected<pibridge::eye::live::Config, std::string> make_config(const Options& opts, pibridge::eye::live::Config base,
                                                               const std::function<void(std::string_view)>& say,
                                                               std::string& profile_kind);

/// Everything the daemon touches outside itself, replaceable for tests.
struct Deps {
    /// Is a process with this name running (/proc/<pid>/comm)?
    std::function<bool(std::string_view)> process_running = [](std::string_view n) { return pibridge::hmd::process_running(n); };
    /// The headset command sink for the eye-module enable (null: HidHeadsetCommands over the real sysfs).
    std::shared_ptr<pibridge::eye::power::HeadsetCommands> hid;
    /// Starts a tracker (null: Tracker::start on the real chip, RealDevice with `hid`).
    std::function<std::unique_ptr<pibridge::eye::live::Tracker>(const pibridge::eye::live::Config&)> start_tracker;
    /// The tracker's environment configuration (null: Config::from_env).
    std::function<pibridge::eye::live::Config(const pibridge::eye::live::LogFn&)> base_config;
    /// Whether PIBRIDGE_EYE_FLIP_X is set (then it wins over `eye_flip_x`); null: the real environment.
    std::function<bool()> flip_x_from_env;
    /// One log line (stdout with a HH:MM:SS prefix by default; the journal keeps it).
    std::function<void(std::string_view)> say;
    /// Name of the status file in $XDG_RUNTIME_DIR/pibridge/ (empty = do not write one).
    std::string status_name = "eyed";
};

class Daemon {
public:
    Daemon(Options opts, Deps deps);
    ~Daemon();
    Daemon(const Daemon&) = delete;
    Daemon& operator=(const Daemon&) = delete;

    /// Create the gaze file ("no gaze"), build the tracker configuration and start the tracker unless the followed
    /// process is not running. Fails only when the gaze file cannot be created.
    std::expected<void, std::string> start(Clock::time_point now);
    /// One pass of the loop (call it every few ms): the followed process, the tracker state, a new sample to the gaze
    /// file, the eyes lost / seen log, the eye-module enable, the status file.
    void tick(Clock::time_point now);
    /// New options after SIGHUP: the tracker restarts when the start list, the profile, flip x or eye tracking
    /// changed; a new followed process is taken in its current state. The gaze file path changes only at the next
    /// start.
    void reload(Options opts, Clock::time_point now);
    /// Clean stop: the tracker unsubscribes and releases the chip, the gaze file says "no gaze", the status file is
    /// removed.
    void stop();
    /// An inside-out session owns the eye chip (common/pibridge_pause.h): the tracker is stopped (unsubscribed, the
    /// chip released), nothing is published to the gaze file (Monado's PIMAX_EYE writes it then), status "paused".
    /// Off: the tracker starts again when it should run.
    void set_paused(bool on, Clock::time_point now);
    bool paused() const { return paused_; }

    Status status() const;
    const Options& options() const { return opts_; }
    /// The tracker (null while idle).
    const pibridge::eye::live::Tracker* tracker() const { return tr_.get(); }
    /// The tracker configuration in use (after start / reload).
    const pibridge::eye::live::Config& config() const { return cfg_; }

private:
    void say(const std::string& m) const;
    void configure();
    bool should_run() const;
    void start_tracker();
    void write_status(Clock::time_point now, bool force);

    Options opts_;
    Deps deps_;
    pibridge::eye::live::Config cfg_;
    bool cfg_ok_ = false;
    std::string profile_kind_ = "none";
    std::string error_;
    std::optional<pibridge::eye::shm::Publisher> out_;
    std::unique_ptr<pibridge::eye::live::Tracker> tr_;
    std::optional<Follow> fol_;
    bool follow_running_ = false;
    std::optional<Clock::time_point> last_follow_check_;
    std::optional<pibridge::eye::live::State> last_state_;
    std::optional<std::uint64_t> last_seq_;
    std::optional<Clock::time_point> last_write_;
    std::uint64_t streamed_ = 0;
    Clock::time_point since_{};
    // eyes seen or not (any eye's direction valid), and since when; whether the enable went out for this stretch
    std::optional<std::pair<bool, Clock::time_point>> eyes_;
    bool enable_sent_ = false;
    std::optional<Clock::time_point> last_valid_;
    // samples per second for the status file: samples counted since rate_since_
    std::uint64_t rate_count_ = 0;
    Clock::time_point rate_since_{};
    double rate_ = 0;
    std::optional<Status> last_status_;
    std::optional<Clock::time_point> last_status_write_;
    bool started_ = false;
    bool stopped_ = false;
    bool paused_ = false;
};

} // namespace pibridge::eyed
