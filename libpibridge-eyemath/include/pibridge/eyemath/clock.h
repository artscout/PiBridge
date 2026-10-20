// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/clock.rs (MIT OR Apache-2.0).
// Host monotonic clock and the device-to-host timestamp map.
#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace pibridge::eyemath {

/// `CLOCK_MONOTONIC` in nanoseconds: the clock Monado and SteamVR drivers stamp their data with.
std::int64_t monotonic_ns();

/// Maps the device clock (microseconds, same crystal as the IMU) to the host `CLOCK_MONOTONIC`.
///
/// `arrival - device` is the clock offset plus a non-negative transport delay, so the true offset is the lower
/// envelope of that difference. The estimate follows it with a one-sided filter: it jumps down at once to any new
/// minimum and creeps up with a time constant of `1/alpha` samples, which tracks crystal drift (tens of ppm, a few
/// tens of microseconds per second) without being dragged up by USB jitter. The result is "arrival time of the
/// least-delayed sample", so it is late by the minimal transport latency (a ms or two); `latency_ns` is subtracted
/// as a configured allowance. Output is forced strictly increasing.
class ClockMap {
public:
    /// New map; `alpha` is the upward creep per sample (default 1/1024).
    explicit ClockMap(double alpha = 1.0 / 1024.0) : alpha_(alpha) {}

    /// Forget everything (device re-enumerated: its clock restarted). Keeps `latency_ns`.
    void reset();
    /// Current offset `host - device` in ns, if any sample was seen.
    std::optional<double> offset_ns() const { return offset_ns_; }
    /// Host time for a sample with device timestamp `dev_us` that arrived at host time `arrival_ns`.
    std::int64_t map(std::int64_t dev_us, std::int64_t arrival_ns);

    /// Subtracted from every mapped time (configured transport latency).
    std::int64_t latency_ns = 0;

private:
    std::optional<double> offset_ns_;
    double alpha_;
    std::int64_t last_dev_us_ = std::numeric_limits<std::int64_t>::min();
    std::int64_t last_out_ns_ = std::numeric_limits<std::int64_t>::min();
};

} // namespace pibridge::eyemath
