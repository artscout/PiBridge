// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/resilient.rs (MIT OR Apache-2.0).
//
// A live Source that heals itself: it reopens the camera (or reconnects to the broker) after an error or a stall
// instead of failing, so long-running readers (the broker, ctrltrack, Monado through the C ABI) survive a USB3 hub
// drop or a broker restart.
//
// * An error from the inner source (V4L2 POLLERR/ENODEV after an unplug, broker socket closed) drops it; the next
//   advance calls try to open it again with backoff (Policy::backoff_min doubling up to Policy::backoff_max).
//   Meanwhile advance returns false (a timeout) and honours the caller's timeout, so callers keep their loops and
//   only see a gap. The camera 34a4:0014 can take about 90 s to come back after a hub drop: the wrapper simply keeps
//   trying, it never gives up.
// * A stall (no frame for Policy::stall while open; live camera only) is handled like an error. A reopen that never
//   delivers a frame counts as a failure, so a camera that stays dark (headset asleep) is retried with the long
//   backoff and logged at most every Policy::log_every.
// * Replays are never wrapped (their end must stay visible), and a broker client connected to a replay broker passes
//   the broker's end through.
// * generation() changes on every reopen, so Camera resets its clock mapping and frame counter tracking.
//
// Opening sends nothing to the headset: a reopen is the same plain UVC open as the first one (or a socket connect).
// PIMAX_CAM_RECONNECT=0 turns the wrapper off; PIMAX_CAM_STALL_S sets the camera stall time (default 5 s, 0 = no
// stall detection). Log lines start with "pimax-cam: " like the Rust ones.
#pragma once

#include "pibridge/cam/source.h"

#include <chrono>
#include <functional>
#include <optional>

namespace pibridge::cam {

/// Opens (or reopens) the inner source.
using Opener = std::function<Result<std::unique_ptr<Source>>()>;

using Clock = std::chrono::steady_clock;

/// Reopen policy.
struct Policy {
    /// No frame for this long while open: treat as lost (nullopt: only errors count).
    std::optional<Clock::duration> stall;
    /// First retry delay after a loss.
    Clock::duration backoff_min{};
    /// Longest retry delay.
    Clock::duration backoff_max{};
    /// While down, repeat the "still waiting" log line at most this often.
    Clock::duration log_every{};

    /// The live camera: stall after 5 s (PIMAX_CAM_STALL_S), retry 0.25 s doubling to 2 s.
    static Policy camera();
    /// A broker client: reconnect on errors only (the broker itself watches the camera), 0.25 s doubling to 1 s.
    static Policy broker();
    /// Retry delay after failures consecutive failures (>= 1).
    Clock::duration backoff(std::uint32_t failures) const;
};

/// True unless PIMAX_CAM_RECONNECT=0 (or off/no/false).
bool reconnect_enabled();

/// Counters of a Resilient source.
struct Health {
    bool up = false;
    /// Successful opens, the first one included.
    std::uint64_t opens = 0;
    /// Times the source was lost (error or stall).
    std::uint64_t losses = 0;
    /// Of those, stalls.
    std::uint64_t stalls = 0;
    /// Failed open attempts since the last frame.
    std::uint32_t failures = 0;
};

class Resilient final : public Source {
public:
    /// Wrap an already open source; open reopens it. label names it in log lines ("camera", "camera broker").
    Resilient(std::unique_ptr<Source> first, Opener open, Policy policy, std::string label);

    /// Silence the stderr lines (tests).
    Resilient& quiet() {
        log_ = false;
        return *this;
    }
    /// Counters.
    Health health() const;

    Result<bool> advance(int timeout_ms) override;
    std::optional<RawFrame> current() const override;
    std::string describe() const override;
    bool finished() const override;
    std::uint64_t corrupt_frames() const override;
    bool is_replay() const override;
    std::uint64_t generation() const override;
    bool up() const override;

private:
    void say(Clock::time_point now, const std::string& msg);
    void lose(Clock::time_point now, std::string why, bool stall);
    void try_open(Clock::time_point now);

    Opener open_;
    std::unique_ptr<Source> inner_;
    Policy policy_;
    std::string label_;
    std::string desc_;
    std::string last_err_;
    Health health_;
    bool framed_ = false;
    Clock::time_point last_frame_;
    Clock::time_point next_try_;
    std::optional<Clock::time_point> down_since_;
    std::optional<Clock::time_point> last_log_;
    std::uint64_t corrupt_base_ = 0;
    bool log_ = true;
};

} // namespace pibridge::cam
