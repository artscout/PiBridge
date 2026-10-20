// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/track.rs (MIT OR Apache-2.0).
//
// Gaze samples in the OpenXR head frame: device-to-head conversion, the combined gaze ray, interpolation.
//
// Device frame (what is known and what is not): the EyeChip reports origins in millimetres and unit directions in
// one 3D frame. From the data: `+z` is the viewing direction (both eyes sit at `z` about -20 mm, behind the sensor
// plane, and look toward `+z`), eye A has `x` about +30 mm and eye B about -28 mm. Tobii's XR runtime reports in the
// Unity-style HMD frame (x right, y up, z forward, left-handed), so the working assumption is that device x points to
// the user's right and eye A is the right eye, y up. The sign of x and y needs a live test (look left, look up).
//
// OpenXR is right-handed: x right, y up, -z forward. The default conversion is therefore `(x, y, z) -> (x, y, -z)`
// (flip z only), millimetres to metres, with the eye order `[left, right]` = `[B, A]`.
//
// Every part is configurable (FrameConv, environment, FrameConv::from_env):
//
// | variable              | default | effect |
// |-----------------------|---------|--------|
// | `PIBRIDGE_EYE_FLIP_X`    | 0       | negate x; also swaps which eye is left (the eye with the larger converted x is right) |
// | `PIBRIDGE_EYE_FLIP_Y`    | 0       | negate y |
// | `PIBRIDGE_EYE_FLIP_Z`    | 1       | negate z (device +z forward to OpenXR -z forward); 0 if the device z already points back |
// | `PIBRIDGE_EYE_OFFSET_MM` | 0,0,0   | added to the converted origins (head-frame position of the device origin) |
// | `PIBRIDGE_EYE_SWAP`      | 0       | swap the left/right assignment after the above |
//
// The Crystal Super runs with `eye_flip_x=1` (pibridge.conf): its chip sees the eyes the other way round.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "pibridge/eyemath/gaze.h"
#include "pibridge/eyemath/types.h"

namespace pibridge::eyemath {

/// Device-to-head conversion.
struct FrameConv {
    /// Negate device x.
    bool flip_x = false;
    /// Negate device y.
    bool flip_y = false;
    /// Negate device z (default true: device forward is +z, OpenXR forward is -z).
    bool flip_z = true;
    /// Offset added to converted origins, millimetres (OpenXR axes).
    Vec3 offset_mm{0.0, 0.0, 0.0};
    /// Swap the left/right assignment.
    bool swap_eyes = false;

    /// Defaults overridden by the `PIBRIDGE_EYE_*` environment variables (table above). A flag is on for
    /// "1", "true", "yes", "on" (trimmed) and off for any other value; an unset variable keeps the default.
    static FrameConv from_env();

    /// Device origin (mm) to head-frame origin (mm).
    Vec3 origin(const Vec3& v) const;
    /// Device direction to head-frame direction.
    Vec3 direction(const Vec3& v) const;
    /// Index (0 = left, 1 = right) of the head-frame eye that device eye A (`a = true`) or B (`false`) is.
    std::size_t slot(bool a) const;

    friend bool operator==(const FrameConv&, const FrameConv&) = default;

private:
    Vec3 axes(const Vec3& v) const;
};

/// One eye in the head frame (OpenXR axes, metres).
struct EyeRay {
    /// Origin, metres.
    Vec3f origin{0.0F, 0.0F, 0.0F};
    /// Unit direction.
    Vec3f direction{0.0F, 0.0F, 0.0F};
    /// Origin is valid.
    bool origin_valid = false;
    /// Direction is valid (the eye is tracked).
    bool direction_valid = false;
    /// Pupil diameter in millimetres, negative when unknown.
    float pupil_mm = 0.0F;
    /// Pupil position in the eye-camera sensor area, 0..1 (-1 when unknown).
    std::array<float, 2> pupil_pos{0.0F, 0.0F};
    /// `pupil_pos` is valid.
    bool pupil_pos_valid = false;
    /// Eye openness, negative when the device does not provide it.
    float openness = 0.0F;

    friend bool operator==(const EyeRay&, const EyeRay&) = default;
};

/// Combined gaze ray in the head frame.
struct Gaze {
    /// Midpoint of the contributing origins, metres.
    Vec3f origin{0.0F, 0.0F, 0.0F};
    /// Normalised mean direction.
    Vec3f direction{0.0F, 0.0F, 0.0F};
    /// Bit 0 left eye contributed, bit 1 right eye.
    std::uint8_t eyes = 0;

    friend bool operator==(const Gaze&, const Gaze&) = default;
};

/// One gaze sample with host-clock time.
struct Sample {
    /// Running number of samples since the tracker started.
    std::uint64_t seq = 0;
    /// Device timestamp, microseconds.
    std::int64_t device_ts_us = 0;
    /// Device time mapped to `CLOCK_MONOTONIC`, nanoseconds (ClockMap).
    std::int64_t host_ts_ns = 0;
    /// Host time the USB transfer carrying the sample returned.
    std::int64_t arrival_ns = 0;
    /// Eyes, index 0 left and 1 right (after FrameConv::slot).
    std::array<EyeRay, 2> eyes{};
    /// Combined ray; empty when no eye has a valid ray.
    std::optional<Gaze> gaze;
    /// Interpupillary distance reported by the chip (field 34), millimetres; 0 when absent.
    float ipd_mm = 0.0F;

    friend bool operator==(const Sample&, const Sample&) = default;
};

/// Convert a decoded device sample. `host_ts_ns` / `arrival_ns` come from the clock map.
Sample convert(const GazeSample& g, const FrameConv& conv, std::uint64_t seq, std::int64_t host_ts_ns,
               std::int64_t arrival_ns);

/// The sample for host time `t_ns` from a history ordered by `host_ts_ns`: linear interpolation of the combined
/// ray between the two neighbours, clamped to the ends (no extrapolation). Empty for an empty history.
std::optional<Sample> sample_at(std::span<const Sample> history, std::int64_t t_ns);

} // namespace pibridge::eyemath
