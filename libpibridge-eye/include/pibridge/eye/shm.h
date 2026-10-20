// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/shm.rs (MIT OR Apache-2.0).
//
// The gaze for other processes: one small file in shared memory (`/dev/shm/omarchyvr-eye-gaze`), rewritten for every
// sample by the eye daemon. Readers: the SteamVR driver (CustomHeadsetOpenVR with the Linux patch, which hands the
// gaze to SteamVR's eye tracking component and so to OpenXR XR_EXT_eye_gaze_interaction) and anything else that wants
// the gaze without owning the eye chip. The C layout is omarchyvr's `linux/crystal-super/omarchyvr_eye_gaze.h`
// (struct omarchyvr_eye_gaze); this file is byte-identical to it.
//
// Layout (little endian, SIZE bytes), head frame of OpenXR (x right, y up, -z forward, metres):
//
//   off  type        field
//     0  u64         seq_begin
//     8  u32         magic MAGIC ("OVEG")
//    12  u32         version VERSION
//    16  i64         sample time, CLOCK_MONOTONIC ns (latency corrected)
//    24  i64         time it was published, CLOCK_MONOTONIC ns
//    32  u32         flags: LEFT, RIGHT, COMBINED, LEFT_ORIGIN, RIGHT_ORIGIN
//    36  f32         IPD from the chip, mm (0 = unknown)
//    40  f32 x3 x2   left origin, left direction
//    64  f32 x3 x2   right origin, right direction
//    88  f32 x3 x2   combined origin, combined direction
//   112  u64         tracker sample number
//   120  u32         tracker state (0 searching, 1 connecting, 2 streaming, ...)
//   124  u32         reserved (0)
//   128  u64         seq_end
//
// Consistency without locks: the writer stores an odd seq_begin, then the body, then the new even value in seq_end,
// then in seq_begin. A reader loads seq_end, the body, then seq_begin, and keeps the body only when both are equal and
// even (else it retries). Each store is its own pwrite, so they reach readers in this order.
#pragma once

#include "pibridge/eye/error.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>

namespace pibridge::eye::shm {

/// "OVEG" as a little-endian u32.
inline constexpr std::uint32_t MAGIC = 0x4745564fu;
/// Layout version.
inline constexpr std::uint32_t VERSION = 1;
/// Size of the file.
inline constexpr std::size_t SIZE = 136;
/// Flag: left direction valid.
inline constexpr std::uint32_t LEFT = 1;
/// Flag: right direction valid.
inline constexpr std::uint32_t RIGHT = 2;
/// Flag: combined direction valid.
inline constexpr std::uint32_t COMBINED = 4;
/// Flag: left origin valid.
inline constexpr std::uint32_t LEFT_ORIGIN = 8;
/// Flag: right origin valid.
inline constexpr std::uint32_t RIGHT_ORIGIN = 16;

/// The default file (in use by the deployed daemon and the SteamVR driver: tests must never use it).
std::filesystem::path default_path();

using Vec3 = std::array<float, 3>;

/// One eye's ray in the head frame (the fields of eyemath's per-eye ray this file carries).
struct EyeRay {
    Vec3 origin{};
    Vec3 direction{};
    bool origin_valid = false;
    bool direction_valid = false;
};

/// Combined gaze ray in the head frame.
struct CombinedRay {
    Vec3 origin{};
    Vec3 direction{};
};

/// What the writer publishes: the subset of a tracker sample (eyemath's Sample in phase 2) that goes into the file.
struct Sample {
    /// Running number of samples since the tracker started.
    std::uint64_t seq = 0;
    /// Sample time mapped to CLOCK_MONOTONIC, nanoseconds.
    std::int64_t host_ts_ns = 0;
    /// Eyes, index 0 left and 1 right.
    std::array<EyeRay, 2> eyes{};
    /// Combined ray; nullopt when no eye has a valid ray.
    std::optional<CombinedRay> gaze;
    /// Interpupillary distance reported by the chip, millimetres; 0 when absent.
    float ipd_mm = 0;
};

/// What a reader gets.
struct Gaze {
    std::int64_t t_ns = 0;
    std::int64_t written_ns = 0;
    std::uint32_t flags = 0;
    float ipd_mm = 0;
    /// left, right, combined
    std::array<Vec3, 3> origin{};
    std::array<Vec3, 3> direction{};
    std::uint64_t sample = 0;
    std::uint32_t state = 0;
    bool operator==(const Gaze&) const = default;
};

/// The body (bytes 8..128) of a sample (`s` null: no gaze right now).
std::array<std::uint8_t, 120> body(const Sample* s, std::uint32_t state, std::int64_t written_ns);

/// Decode a consistent snapshot (the bytes of the whole file); nullopt when torn, empty or of another layout.
std::optional<Gaze> decode(std::span<const std::uint8_t> f);

/// The writer.
class Publisher {
public:
    /// Create (or take over) the file, mode 0600.
    static Result<Publisher> create(const std::filesystem::path& path);

    Publisher(Publisher&& o) noexcept;
    Publisher& operator=(Publisher&& o) noexcept;
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;
    ~Publisher();

    /// Publish one state (`sample` null: no gaze right now).
    Result<void> publish(const Sample* sample, std::uint32_t state, std::int64_t written_ns);

private:
    Publisher(int fd, std::uint64_t seq) : fd_(fd), seq_(seq) {}
    int fd_ = -1;
    std::uint64_t seq_ = 0;
};

/// Read the file the way readers must (seq_end, body, seq_begin; a few retries when torn).
std::optional<Gaze> read(const std::filesystem::path& path);

} // namespace pibridge::eye::shm
