// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/shm.rs (MIT OR Apache-2.0).
//
// Live controller poses for Monado: a 256-byte memory-mapped file with a seqlock.
//
// Layout (little endian), default path `$XDG_RUNTIME_DIR/pimax-ctrltrack.pose`. The Monado reader is
// `pimax_ctrltrack_shm.h` in the Pimax driver (keep both in sync; byte-identical to the Rust writer).
//
//   0 u32 magic 0x54435850 ("PXCT")   4 u32 version (1)   8 u32 seq (odd while writing)   12 u32 writer pid
//  16 + 96*hand (0 = left, 1 = right):
//    +0 u64 frame_ns    CLOCK_MONOTONIC time of the LED frame the pose comes from (0 = never seen)
//    +8 u64 publish_ns  CLOCK_MONOTONIC time it was written
//   +16 f32 pos[3]      head (body) frame position of the controller (LED model origin), metres;
//                       body = OpenXR head axes (x right, y up, z back), the frame of Monado's head pose
//   +28 f32 quat[4]     body_from_aim orientation, x y z w
//   +44 f32 rms_px      joint reprojection RMS
//   +48 u32 leds        matched LEDs over all cameras
//   +52 u32 cams        cameras with matches
//   +56 f32 aim_from_imu[4]  rotation x y z w (controller IMU axes in the aim frame, for gyro deltas)
// 208 heartbeat (same version: older readers ignore it; 0 = an older writer):
//   208 u64 beat_ns     CLOCK_MONOTONIC time of the writer's last loop pass (at least every 0.5 s while alive,
//                       with or without frames or controllers): the liveness signal for omarchyvrd / Monado
//   216 u64 led_frames  LED frames processed since start
//   224 u32 source_up   1 = camera frames arriving, 0 = source reopening (broker or camera gone)
//   228 u32 reopens     times the source came back (broker restart, camera reopen)
#pragma once

#include "pibridge/cam/io.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace pibridge::ctrl::track::shm {

inline constexpr std::uint32_t MAGIC = 0x5443'5850;
inline constexpr std::uint32_t VERSION = 1;
inline constexpr std::size_t SIZE = 256;
inline constexpr std::size_t HAND_OFF = 16;
inline constexpr std::size_t HAND_STRIDE = 96;
inline constexpr std::size_t BEAT_OFF = HAND_OFF + 2 * HAND_STRIDE;
static_assert(BEAT_OFF + 24 <= SIZE);

/// The writer's heartbeat (offset BEAT_OFF).
struct Beat {
    std::uint64_t beat_ns = 0;
    std::uint64_t led_frames = 0;
    bool source_up = false;
    std::uint32_t reopens = 0;
    /// Age of the heartbeat at now_ns; nullopt when the writer never wrote one (older writer).
    std::optional<std::uint64_t> age_ns(std::uint64_t now_ns) const {
        if (beat_ns == 0) {
            return std::nullopt;
        }
        return now_ns > beat_ns ? now_ns - beat_ns : 0;
    }
    bool operator==(const Beat&) const = default;
};

/// One hand's published state.
struct HandPose {
    std::uint64_t frame_ns = 0;
    std::uint64_t publish_ns = 0;
    std::array<float, 3> pos{};
    std::array<float, 4> quat{};
    float rms_px = 0;
    std::uint32_t leds = 0;
    std::uint32_t cams = 0;
    std::array<float, 4> aim_from_imu{};
    bool operator==(const HandPose&) const = default;
};

/// `$XDG_RUNTIME_DIR/pimax-ctrltrack.pose` (`/tmp` without XDG_RUNTIME_DIR).
std::string default_path();

/// Monado's in-hand status (`$XDG_RUNTIME_DIR/pimax-ctrl-inhand`, written by the Pimax controller driver,
/// pimax-hands inhand_ffi.rs).
std::string inhand_status_path();

/// Which controllers to look for (left, right): a controller is skipped only when the status file of a LIVE Monado
/// says it is `down` or `off`. No file, an unreadable one or a dead writer: both.
std::array<bool, 2> inhand_wanted(const std::string& path);

/// inhand_wanted on the file's text; `alive` tells whether the writer's pid runs.
std::array<bool, 2> parse_inhand(std::string_view text, const std::function<bool(std::uint32_t)>& alive);

std::uint64_t monotonic_ns();

/// Writer side (`pibridge-ctrltrack live`).
class PoseWriter {
public:
    static cam::Result<std::unique_ptr<PoseWriter>> create(const std::string& path);
    ~PoseWriter();
    PoseWriter(const PoseWriter&) = delete;
    PoseWriter& operator=(const PoseWriter&) = delete;
    /// Replace one hand (0 left, 1 right) and publish both.
    void set(std::size_t hand, const HandPose& h);
    /// Update the heartbeat and publish.
    void beat(const Beat& b);

private:
    PoseWriter() = default;
    void flush();
    std::uint8_t* p_ = nullptr;
    std::array<HandPose, 2> hands_{};
    Beat beat_{};
};

/// Reader side (tests, `pibridge-ctrltrack show`); Monado has the same in C.
class PoseReader {
public:
    static cam::Result<std::unique_ptr<PoseReader>> open(const std::string& path);
    ~PoseReader();
    PoseReader(const PoseReader&) = delete;
    PoseReader& operator=(const PoseReader&) = delete;
    std::optional<std::array<HandPose, 2>> read() const;
    /// Both hands and the heartbeat, one consistent snapshot.
    std::optional<std::pair<std::array<HandPose, 2>, Beat>> read_all() const;

private:
    PoseReader() = default;
    std::uint8_t* p_ = nullptr;
};

} // namespace pibridge::ctrl::track::shm
