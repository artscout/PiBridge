// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/live.rs (MIT OR Apache-2.0).
//
// A long-running gaze session: open and claim the EyeChip, send the captured Windows init (the start list), subscribe,
// read gaze on a thread and publish eyemath Samples in the OpenXR head frame with host-clock timestamps; unsubscribe
// and release the interface on stop; start over when the device re-enumerates or stalls.
//
// Safety model is that of frames.h: only list members go out (frames::Guard), calibration opcodes are refused, the
// single extra frame is the captured gaze unsubscribe (0x4ce, stream 0x0505, as Pimax Play sent it at the end of a
// session; see session::CAPTURED_UNSUBSCRIBE_GAZE). The eye-module enable goes to the headset only through
// power::HeadsetCommands (the device allowlist), never from this library directly.
//
// The gaze math (decode, clock map, frame conversion, calibration, filter, drift) is libpibridge-eyemath's; this file only
// wires it to the transport.
#pragma once

#include "pibridge/eye/error.h"
#include "pibridge/eye/power.h"
#include "pibridge/eye/replay.h"
#include "pibridge/eye/session.h"
#include "pibridge/eye/shm.h"
#include "pibridge/eye/transport.h"
#include "pibridge/eye/ttp.h"

#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/clock.h"
#include "pibridge/eyemath/correction.h"
#include "pibridge/eyemath/drift.h"
#include "pibridge/eyemath/filter.h"
#include "pibridge/eyemath/gaze.h"
#include "pibridge/eyemath/track.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <expected>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace pibridge::eye::live {

using namespace std::chrono_literals;

/// Connection state of a Tracker. The numbers are what the shared-memory file carries (`state` at offset 120).
enum class State : std::uint8_t {
    /// No EyeChip on the bus (or the thread has not started yet).
    Searching = 0,
    /// Opening, init sequence running.
    Connecting = 1,
    /// Gaze samples are arriving.
    Streaming = 2,
    /// The link failed; waiting to try again.
    Reconnecting = 3,
    /// Stopped.
    Stopped = 4,
};

/// "Searching", "Connecting", ... (the Rust `{:?}`).
std::string_view state_name(State s);

/// One log line.
using LogFn = std::function<void(std::string_view)>;

/// Session options.
struct Config {
    /// Send the captured calibration blob (0x456, the user's profile). Off by default: streaming works without it.
    bool with_cal = false;
    /// Leave out the 0x76c / 0x777 auth pair (the answer is rejected by the chip but harmless).
    bool skip_auth = false;
    /// Leave out the vendor control write bReq 48.
    bool skip_vendor_write = true;
    /// Device-to-head frame conversion.
    pibridge::eyemath::FrameConv conv;
    /// Host-side accuracy correction applied to every converted sample (empty: raw). The old combined-gaze
    /// correction (`eye-correction.toml`); ignored when `calibration` is set.
    std::optional<pibridge::eyemath::Correction> correction;
    /// Per-eye calibration (`eye-calibration.toml`); its device-to-head conversion replaces `conv`.
    std::optional<pibridge::eyemath::Calibration> calibration;
    /// The file `calibration` came from (empty when none; for the status file).
    std::filesystem::path calibration_path;
    /// Output smoothing (empty: the calibration's `[filter]`, or no filter without a calibration).
    std::optional<pibridge::eyemath::FilterParams> filter;
    /// Transport latency subtracted from mapped host times.
    std::int64_t latency_ns = 0;
    /// A streaming link with no gaze for this long is torn down and re-initialised.
    Duration stall = 3s;
    /// Wait between connection attempts (doubles up to 10 s while attempts fail).
    Duration retry = 2s;
    /// Timeout of every USB transfer.
    Duration timeout = 2s;
    /// How long a request may wait for its answer (the 34 KB calibration upload gets at least 10 s).
    Duration response_wait = 3s;
    /// Samples kept for Tracker::sample_at.
    std::size_t history = 256;
    /// Use this usbfs node instead of searching sysfs for 2104:0561 (taken as a 2104:0561).
    std::optional<std::filesystem::path> usb_device;
    /// Also take the eye tracker of the original Crystal (2104:0220) when no 2104:0561 is there; it always runs on
    /// its own start list (for_chip). Off by default (pibridge-eyed turns it on; the Tobii calibration of
    /// pibridge-eyecal was captured on 2104:0561 only and never reaches a 2104:0220).
    bool p3b = false;
    /// Pace the init steps like the capture (clamped to 1 s). Off only in tests.
    bool pace = true;
    /// Recover a chip that stopped answering with a host-side USB port reset (`USBDEVFS_RESET`).
    bool usb_reset = true;
    /// Consecutive "silent" link failures (a request without answer, or the gaze stream stopping) before a reset.
    std::uint32_t reset_after = 2;
    /// Minimum time between two USB resets.
    Duration reset_min_interval = 60s;
    /// Power the eye module (the captured enable through power::HeadsetCommands) when the headset is present and the
    /// chip has been absent for 5 s; at most once per headset enumeration.
    bool power_on = true;
    /// The start list (frame list text) to use instead of the built-in one: a list made by `pibridge-eye frames` from
    /// the user's own Windows capture of this headset (the Crystal Super has no built-in list). Its own 0x456 profile
    /// upload is accepted.
    std::shared_ptr<const std::string> frames;
    /// The eye profile body to upload in place of the list's own 0x456 (the user's current calibration, from Pimax's
    /// `calibration.setpm` by `pibridge-eye profile`, see frames::with_profile). Implies `with_cal`.
    std::shared_ptr<const Bytes> profile;

    /// Defaults overridden by the environment: frame conversion (`PIBRIDGE_EYE_FLIP_*`, eyemath track.h),
    /// `PIBRIDGE_EYE_CAL=1` (send the calibration blob), `PIBRIDGE_EYE_SKIP_AUTH=1`, `PIBRIDGE_EYE_LATENCY_MS`,
    /// `PIBRIDGE_EYE_USB_RESET=0` (no USB reset recovery), `PIBRIDGE_EYE_POWER_ON=0` (never power the eye module),
    /// `PIBRIDGE_EYE_USB` (usbfs node), `PIBRIDGE_EYE_FRAMES=path` (start list, see `frames`), `PIBRIDGE_EYE_PROFILE=path`
    /// (eye profile body, see `profile`; a missing file is skipped), `PIBRIDGE_EYE_CALIBRATION` and
    /// `PIBRIDGE_EYE_CORRECTION` (eyemath calib.h / correction.h), `PIBRIDGE_EYE_FILTER=none|one-euro`. Problems are
    /// logged through `log` (stderr with a `pibridge-eye: ` prefix by default) and leave the default in place.
    static Config from_env(const LogFn& log = {});
};

/// The configuration of a link to the chip with USB product id `pid`. A 2104:0220 (the original Crystal) gets its own
/// start list (frames::p3b_list) in place of the configured list and profile: those are made from and for 2104:0561
/// (its auth, platform config and XR5EYECHIP profile), and Pimax Play uploads no profile to this chip. Any other pid:
/// `cfg` as it is.
Config for_chip(const Config& cfg, std::uint16_t pid);

/// The pieces of a running drift estimate shared between the reader thread and the recentre hook.
struct SharedDrift {
    std::mutex mu;
    pibridge::eyemath::Drift drift;
};

/// Decodes the IN pipe, maps timestamps and hands out samples: device-to-head conversion, then the per-eye
/// calibration and combination (or the old combined correction), the running drift, the output filter.
class Pipeline {
public:
    using Sink = std::function<void(const pibridge::eyemath::Sample&)>;

    /// New pipeline; `sink` receives every sample.
    Pipeline(const Config& cfg, Sink sink);

    /// Start a new link: fresh framing and a fresh clock map (the chip's clock restarts when it re-enumerates).
    void reset_link();
    /// Share a running drift estimate (added after the per-eye calibration).
    void set_drift(std::shared_ptr<SharedDrift> d) { drift_ = std::move(d); }
    /// Feed one IN transfer. Gaze notifications become samples; every other message is returned.
    std::vector<ttp::Message> push(ByteView bytes);

    /// Gaze samples decoded.
    std::uint64_t samples = 0;
    /// Gaze payloads that failed to decode.
    std::uint64_t parse_errors = 0;
    /// IN framing errors.
    std::uint64_t framing_errors = 0;
    /// The host clock (CLOCK_MONOTONIC ns); replaceable in tests.
    std::function<std::int64_t()> now;

private:
    ttp::StreamDecoder dec_;
    pibridge::eyemath::ClockMap clock_;
    pibridge::eyemath::FrameConv conv_;
    std::optional<pibridge::eyemath::Correction> correction_;
    std::optional<pibridge::eyemath::Calibration> calibration_;
    pibridge::eyemath::CombineState combine_;
    pibridge::eyemath::FilterParams filter_;
    pibridge::eyemath::OneEuro one_euro_;
    std::shared_ptr<SharedDrift> drift_;
    Sink sink_;
};

/// Why a link ended.
enum class Failure {
    /// A request got no answer in time (the chip is alive on USB but silent).
    NoResponse,
    /// The gaze stream stopped (or never started after the subscribe).
    Stall,
    /// A request was answered with an unexpected status.
    Status,
    /// A USB transfer failed (device gone, protocol error).
    Io,
    /// Anything else (frame list, guard).
    Other,
};

/// The chip is reachable but does not talk: what a USB reset may cure.
bool is_silent(Failure f);

/// A failed link.
struct LinkError {
    /// Class of the failure.
    Failure kind = Failure::Other;
    /// Text for the log.
    std::string msg;
    bool operator==(const LinkError&) const = default;
};

/// When to USB-reset the chip: after Config::reset_after consecutive silent failures, at most once per
/// Config::reset_min_interval.
class Recovery {
public:
    /// Account for a finished link (`failure` empty for a clean stop); true means "reset the chip now".
    bool after_link(std::optional<Failure> failure, bool streamed, Clock::time_point now, const Config& cfg);
    /// Resets done.
    std::uint32_t resets = 0;

private:
    std::uint32_t silent_ = 0;
    std::optional<Clock::time_point> last_reset_;
};

/// How the tracker reaches a chip: open it, reset it, and react to its absence. RealDevice is the usbfs one.
class Device {
public:
    virtual ~Device() = default;
    /// Open and claim; an error text = no usable chip (absent, busy).
    virtual std::expected<std::unique_ptr<Transport>, std::string> open(const Config& cfg, std::stop_token stop) = 0;
    /// USB-reset the chip (the link is closed when this is called); the value is a log line.
    virtual std::expected<std::string, std::string> reset(const Config& cfg);
    /// Called after open() failed: may power the eye module; returns a log line when it did something.
    virtual std::optional<std::string> absent(const Config& cfg, Clock::time_point now);
    /// Called after a link failed with a silent chip (still on the bus, no answer or no gaze): may power the eye
    /// module again; returns a log line when it did something.
    virtual std::optional<std::string> silent(const Config& cfg, Clock::time_point now);
    /// USB product id of the chip the last open() reached (replay::EYE_PID unless the device knows better).
    virtual std::uint16_t pid() const;
};

/// A Device made of an open function (tests): no reset, no power control.
class FnDevice final : public Device {
public:
    using OpenFn = std::function<std::expected<std::unique_ptr<Transport>, std::string>(const Config&, std::stop_token)>;
    explicit FnDevice(OpenFn f, std::uint16_t pid = replay::EYE_PID) : f_(std::move(f)), pid_(pid) {}
    std::expected<std::unique_ptr<Transport>, std::string> open(const Config& cfg, std::stop_token stop) override {
        return f_(cfg, stop);
    }
    std::uint16_t pid() const override { return pid_; }

private:
    OpenFn f_;
    std::uint16_t pid_;
};

/// The eye-module enable after a silent chip is sent at most this often.
inline constexpr std::chrono::seconds SILENT_ENABLE_INTERVAL{5};

/// The usbfs EyeChip: sysfs discovery, USB reset, eye-module power-on through `hid` (power.h).
class RealDevice final : public Device {
public:
    /// `hid` sends the allowlisted eye-module enable (null: no power control). `usb_root` is normally
    /// `/sys/bus/usb/devices`, `dev_usb_root` `/dev/bus/usb` (tests point them at a temporary tree).
    explicit RealDevice(std::shared_ptr<power::HeadsetCommands> hid = nullptr,
                        std::filesystem::path usb_root = "/sys/bus/usb/devices",
                        std::filesystem::path dev_usb_root = "/dev/bus/usb");

    std::expected<std::unique_ptr<Transport>, std::string> open(const Config& cfg, std::stop_token stop) override;
    std::expected<std::string, std::string> reset(const Config& cfg) override;
    /// The Crystal Super switches its eye module off when the headset is taken off (the chip stays on USB but stops
    /// answering) and does not switch it on when it is put back on: Pimax Play then sends the eye-module enable
    /// again and starts the chip anew (Windows capture sleep-wake of 2026-10-17: standby at 16.4 s, enable at
    /// 22.97 s, chip start at 28.1 s). Same here: the captured enable after a silent link, at most every 5 s.
    std::optional<std::string> silent(const Config& cfg, Clock::time_point now) override;
    /// The headset is there and the chip has been missing for 5 s: the enable, once per headset enumeration.
    std::optional<std::string> absent(const Config& cfg, Clock::time_point now) override;
    std::uint16_t pid() const override { return pid_; }

private:
    /// The chip's node and USB product id.
    std::expected<std::pair<std::filesystem::path, std::uint16_t>, std::string> chip_node(const Config& cfg) const;

    std::shared_ptr<power::HeadsetCommands> hid_;
    std::filesystem::path usb_root_;
    std::filesystem::path dev_usb_root_;
    power::EnableGate gate_;
    std::optional<Clock::time_point> silent_enable_at_;
    std::uint16_t pid_ = replay::EYE_PID;
};

/// Run one link on an open transport until `stop` is requested (ok) or the link fails (error; the caller
/// reconnects). Sends the init unless gaze is already flowing, then reads; on the way out unsubscribes if it
/// subscribed.
std::expected<void, LinkError> run_link(Transport& t, const Config& cfg, std::stop_token stop, Pipeline& pipe,
                                        const std::function<void(State)>& state, const LogFn& log);

/// Counters of a running Tracker.
struct TrackerStats {
    /// Samples published.
    std::uint64_t samples = 0;
    /// Link failures followed by a new attempt.
    std::uint64_t reconnects = 0;
    /// USB resets done to recover a silent chip.
    std::uint64_t resets = 0;
};

/// The gaze session handle. Destroying it stops the thread (unsubscribe, release).
class Tracker {
public:
    /// Start the reader thread on `dev` (RealDevice for the real chip). `log` gets the tracker's lines, each only
    /// once in a row (stderr with a `pibridge-eye: ` prefix by default).
    static std::unique_ptr<Tracker> start(Config cfg, std::unique_ptr<Device> dev, LogFn log = {});
    /// Start with an open function (tests): called for every connection attempt; an error means "no device". `pid`
    /// is the chip it stands for.
    static std::unique_ptr<Tracker> start_with(Config cfg, FnDevice::OpenFn open, LogFn log = {},
                                               std::uint16_t pid = replay::EYE_PID);

    ~Tracker();
    Tracker(const Tracker&) = delete;
    Tracker& operator=(const Tracker&) = delete;

    /// Current state.
    State state() const { return static_cast<State>(state_.load(std::memory_order_relaxed)); }
    /// Counters.
    TrackerStats stats() const;
    /// Newest sample, if any arrived.
    std::optional<pibridge::eyemath::Sample> latest() const;
    /// Every sample still in the history with `seq > after` (all of them for nullopt), oldest first. Polling this once
    /// per rendered frame loses nothing while the history (default 256 samples, about 2 s) covers the gap.
    std::vector<pibridge::eyemath::Sample> samples_since(std::optional<std::uint64_t> after) const;
    /// Sample for host time `t_ns` (`CLOCK_MONOTONIC`): interpolated between neighbours, clamped to the history ends.
    std::optional<pibridge::eyemath::Sample> sample_at(std::int64_t t_ns) const;
    /// Drift-correction hook: the wearer was looking along `dir` (head frame, unit, from the head origin) at host
    /// time `at_ns`. Each calibrated eye's residual updates the running per-eye offset (eyemath drift.h). Returns how
    /// many eyes accepted the observation; an error without a calibration or a sample at that time.
    std::expected<std::size_t, std::string> observe_target(std::int64_t at_ns, const pibridge::eyemath::Vec3& dir,
                                                           const pibridge::eyemath::DriftParams& p);
    /// The running drift offsets.
    pibridge::eyemath::Drift drift() const;
    /// The last line the thread logged (why a link failed, why no chip was found); empty before any.
    std::string last_message() const;
    /// Stop the thread and wait for it (unsubscribes and releases the interface first).
    void stop();

private:
    Tracker(Config cfg, LogFn log);
    void run(std::stop_token stop, std::unique_ptr<Device> dev);
    void set(State s) { state_.store(static_cast<std::uint8_t>(s), std::memory_order_relaxed); }

    Config cfg_;
    LogFn log_;
    std::atomic<std::uint8_t> state_{static_cast<std::uint8_t>(State::Searching)};
    mutable std::mutex hist_mu_;
    std::deque<pibridge::eyemath::Sample> hist_;
    std::size_t cap_;
    std::atomic<std::uint64_t> samples_{0};
    std::atomic<std::uint64_t> reconnects_{0};
    std::atomic<std::uint64_t> resets_{0};
    std::shared_ptr<SharedDrift> drift_ = std::make_shared<SharedDrift>();
    mutable std::mutex msg_mu_;
    std::string last_msg_;
    std::jthread thread_;
};

/// The subset of a tracker sample that goes into the shared-memory file.
shm::Sample to_shm(const pibridge::eyemath::Sample& s);

/// The synthetic gaze sample of the Rust fake chip (`FakeChip::sample`): eye A tracked, looking a little around
/// straight ahead, eye B lost. For tests and the eyed daemon's tests.
pibridge::eyemath::GazeSample fake_sample(std::uint32_t n, std::int64_t timestamp_us);
/// `encode_payload(fake_sample(n, ts))`: plug into fake::FakeChip::payload for a realistic gaze stream.
Bytes fake_payload(std::uint32_t n, std::int64_t timestamp_us);

} // namespace pibridge::eye::live
