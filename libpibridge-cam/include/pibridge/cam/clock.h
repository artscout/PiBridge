// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/clock.rs (MIT OR Apache-2.0).
//
// Device clock -> host CLOCK_MONOTONIC.
//
// The frame metadata carries the MCU time in microseconds (64 bit); the IMU reports the low 32 bits of the same
// clock. Two mappings:
// * imu_dev_to_mono_ns: the preferred one inside Monado. The HMD driver already maps IMU device time to monotonic
//   (pimax_hmd.c keeps an unwrapped device time and a minimum-delay offset). Using the same reference puts camera
//   and IMU samples on one timeline exactly (what SLAM needs).
// * HostClockMap: fallback without the IMU (replays, camera-only runs). It tracks the minimum of
//   host_arrival - device_time (the least delayed frame), relaxing slowly so drift is followed. Same rule as the HMD
//   driver's IMU mapping.
//
// The integer arithmetic is exactly the Rust one (wrapping where Rust's release build wraps): any rounding
// difference would shift the camera/IMU alignment.
#pragma once

#include <cstdint>

namespace pibridge::cam {

/// IMU clock reference published by the HMD driver (the layout of pimax_cam_imu_clock_ref).
struct ImuClockRef {
    /// Last IMU device timestamp (u32 us, as reported).
    std::uint32_t dev_ts_u32 = 0;
    /// Non-zero when the reference is valid.
    std::uint32_t valid = 0;
    /// The same timestamp unwrapped to 64 bit by the driver.
    std::uint64_t dev_ext_us = 0;
    /// monotonic_ns = dev_ext_us * 1000 + offset_ns.
    std::int64_t offset_ns = 0;
};

/// Unwrap the low 32 bits of dev_ts_us around the IMU reference and convert to monotonic ns. Valid while the camera
/// timestamp is within +-35 min of the reference (u32 us wraps every 71.6 min).
std::uint64_t imu_dev_to_mono_ns(std::uint64_t dev_ts_us, const ImuClockRef& r);

/// Minimum-delay mapping from device us to host ns, from arrival times.
class HostClockMap {
public:
    /// Relaxation shift: the offset moves (candidate - offset) >> RELAX per frame towards later arrivals
    /// (2^8 frames = ~5 s at 50 fps).
    static constexpr unsigned RELAX = 8;

    /// Map one frame; host_ns is when the host saw it (V4L2 buffer timestamp). Output is strictly increasing.
    std::uint64_t map(std::uint64_t dev_us, std::uint64_t host_ns);
    /// Current offset (host - device), ns.
    std::int64_t offset_ns() const { return offset_ns_; }

    /// Number of resets (device time went backwards or jumped).
    std::uint32_t resets = 0;

private:
    bool have_ = false;
    std::int64_t offset_ns_ = 0;
    std::uint64_t last_dev_us_ = 0;
    std::uint64_t last_out_ns_ = 0;
};

/// Monotonic clock now, ns.
std::uint64_t monotonic_ns();

} // namespace pibridge::cam
