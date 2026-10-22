// SPDX-License-Identifier: MIT
//
// The original Crystal's own inside-out tracking for other processes: one small file in shared memory
// (`/dev/shm/pibridge-crystal-pose`), rewritten by pibridge-keepalived for every head and controller report the
// headset streams (p3b.h). Reader: the SteamVR driver (CustomHeadsetOpenVR with the PiBridge patch, its Pimax SLAM
// headset on Linux), which has no Pimax runtime to ask.
//
// Layout (little endian, SIZE bytes), the headset's tracking space (y up, metres), CLOCK_MONOTONIC nanoseconds:
//
//   off  type        field
//     0  u64         seq_begin
//     8  u32         magic MAGIC ("PBPS")
//    12  u32         version VERSION
//    16  i64         time it was published
//    24  u32         the headset's own inside-out tracking, flags: 1 orientation, 2 position (0: Lighthouse mode);
//                    head report byte 113
//    28  u32         reserved (0)
//    32  device x3   head, left controller, right controller, DEVICE_SIZE bytes each:
//          +0  i64       sample time (0 = never seen)
//          +8  u32       state: head 1 tracked / 0 no pose; controller 4 no pose, 5 IMU only, 7 optical; 0 never
//          +12 i32       battery % (-1 unknown)
//          +16 f32 x3    position
//          +28 f32 x4    orientation, quaternion x y z w
//          +44 f32 x3    linear velocity, m/s, tracking space
//          +56 f32 x3    angular velocity, rad/s, tracking space
//          +68 u32       buttons: head byte 138 (2 volume up, 1 volume down); controllers pressed (p3b BUTTON_*) | touched
//                        (p3b TOUCH_*) << 8
//          +72 f32 x4    trigger 0..1, grip 0..1, stick x, stick y -1..1 (controllers)
//          +88 u64       sample number
//   320  u64         seq_end
//
// Consistency without locks, as the gaze file (pibridge/eye/shm.h): the writer stores an odd seq_begin, the body, the
// new even value in seq_end, then in seq_begin, each with its own pwrite; a reader loads seq_end, the body, then
// seq_begin, and keeps the body only when both are equal and even.
#pragma once

#include "pibridge/hmd/p3b.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>

namespace pibridge::hmd::pose_shm {

/// "PBPS" as a little-endian u32.
inline constexpr std::uint32_t MAGIC = 0x53504250u;
inline constexpr std::uint32_t VERSION = 1;
inline constexpr std::size_t DEVICE_SIZE = 96;
inline constexpr std::size_t SIZE = 328;

/// The default file (in use by the deployed daemon and the SteamVR driver: tests must never use it).
std::filesystem::path default_path();

struct Device {
    std::int64_t t_ns = 0;
    std::uint32_t state = 0;
    std::int32_t battery = -1;
    std::array<float, 3> position{};
    std::array<float, 4> orientation{0, 0, 0, 1};
    std::array<float, 3> linear_velocity{};
    std::array<float, 3> angular_velocity{};
    std::uint32_t buttons = 0;
    std::array<float, 4> axes{}; // trigger, grip, stick x, stick y
    std::uint64_t sample = 0;
    bool operator==(const Device&) const = default;
};

struct State {
    std::int64_t written_ns = 0;
    std::uint32_t tracking = 0;
    std::array<Device, 3> devices{}; // head, left, right
    bool operator==(const State&) const = default;
};

/// The headset's clock (ns since it booted) mapped to CLOCK_MONOTONIC: the smallest receive-minus-sample offset of the
/// last 0.1-0.2 s (the report that waited least in transit). A window, not the smallest ever: the two clocks
/// run at different rates (2026-10-22: the all-time minimum had the poses 4.3 s old after a few minutes, and SteamVR
/// showed black). Forgotten after a jump (the headset rebooted).
class Clock {
public:
    std::int64_t to_host(std::uint64_t headset_ns, std::int64_t received_ns);

private:
    std::optional<std::int64_t> window_min_;  // this window's smallest offset
    std::optional<std::int64_t> previous_min_; // the window before
    std::int64_t window_start_ = 0;
};

/// A head report as it goes into the file: its angular velocity turned from the body into the tracking space, and the
/// head reports' x-up space turned to y up (the controller reports are y up already).
Device from_head(const p3b::Head& h, std::int64_t t_ns, std::uint64_t sample);
Device from_controller(const p3b::Controller& c, std::int64_t t_ns, std::uint64_t sample);

/// The body (bytes 8..320).
std::array<std::uint8_t, SIZE - 16> body(const State& s);
/// Decode a consistent snapshot (the bytes of the whole file); nullopt when torn, empty or of another layout.
std::optional<State> decode(std::span<const std::uint8_t> f);

/// The writer: keeps the state, rewrites the file on every update.
class Publisher {
public:
    /// Create (or take over) the file, mode 0644 (the SteamVR driver runs as the same user; readable is enough).
    static std::optional<Publisher> create(const std::filesystem::path& path);
    Publisher(Publisher&& o) noexcept;
    Publisher& operator=(Publisher&& o) noexcept;
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;
    ~Publisher();

    void head(const p3b::Head& h, std::int64_t received_ns);
    void controller(const p3b::Controller& c, std::int64_t received_ns);
    /// The headset is gone: every device without a pose.
    void gone(std::int64_t now_ns);
    const State& state() const { return state_; }

private:
    Publisher(int fd, std::uint64_t seq) : fd_(fd), seq_(seq) {}
    void write(std::int64_t now_ns);
    int fd_ = -1;
    std::uint64_t seq_ = 0;
    State state_;
    Clock clock_;
};

} // namespace pibridge::hmd::pose_shm
