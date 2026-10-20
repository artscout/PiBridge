// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/gaze.rs (MIT OR Apache-2.0).
//
// Gaze notification (TTP stream 0x0505, 1242-byte frames, about 120.5 Hz) decoder.
//
// The payload is `00 00` followed by one compound `0x230bb8` (35 elements of type `0x0bb8`). Each element is
// `0x20bb9` (compound, 2 items) `| tag2 field-number | value`, where the value is a scalar or a compound
// (`0x31f42` vec3 S15.16, `0x21f43` vec2 S15.16, `0x21f40` two S22.42). The decoder walks the elements by *field
// number*, so field order does not matter; the 364 full-size packets of the full-packet capture all use the order
// WIRE_ORDER, which encode_payload reproduces byte for byte.
//
// The field meanings come from the Windows captures (headset not worn in the only full-size capture, worn in the
// 485-byte snap-length captures where only eye A and the frame header are visible). Eye B (fields 8-13, 17, 24, 25,
// 32, 33) has never been seen with valid data on the Dream Air.
//
// The transport (TTP framing, stream ids) is libpibridge-eye's: this file only decodes the payload bytes it is handed.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <expected>

#include "pibridge/eyemath/types.h"

namespace pibridge::eyemath {

/// Type id of the 35-element container.
inline constexpr std::uint16_t TYPE_GAZE = 0x0bb8;
/// Type id of one `(field number, value)` element.
inline constexpr std::uint16_t TYPE_FIELD = 0x0bb9;
/// Type id of a vec3 of S15.16 values.
inline constexpr std::uint16_t TYPE_VEC3_F16 = 0x1f42;
/// Type id of a vec2 of S15.16 values.
inline constexpr std::uint16_t TYPE_VEC2_F16 = 0x1f43;
/// Type id of a pair of S22.42 values.
inline constexpr std::uint16_t TYPE_PAIR_F22 = 0x1f40;

/// Field numbers of the frame-level fields.
namespace field {
/// int64 device timestamp in microseconds.
inline constexpr std::uint8_t TIMESTAMP = 1;
/// int32, constant 0xFFFFF in every packet seen.
inline constexpr std::uint8_t FIELD_14 = 14;
/// int32 frame counter, +1 per packet.
inline constexpr std::uint8_t COUNTER = 15;
/// vec3, sentinel (-1,-1,-1) in every packet seen.
inline constexpr std::uint8_t AUX_POINT_1 = 18;
/// validity of AUX_POINT_1.
inline constexpr std::uint8_t AUX_VALID_1 = 19;
/// vec3, sentinel (-1,-1,-1) in every packet seen.
inline constexpr std::uint8_t AUX_POINT_2 = 20;
/// validity of AUX_POINT_2.
inline constexpr std::uint8_t AUX_VALID_2 = 21;
/// int32 flag, always 0.
inline constexpr std::uint8_t FLAG_26 = 26;
/// int32 flag, always 0.
inline constexpr std::uint8_t FLAG_27 = 27;
/// int32 flag, always 0.
inline constexpr std::uint8_t FLAG_28 = 28;
/// S15.16, -1.0 in every packet seen.
inline constexpr std::uint8_t FIELD_29 = 29;
/// S15.16, constant 57.35359 in every packet seen (the track conversion reads it as the IPD in mm).
inline constexpr std::uint8_t FIELD_34 = 34;
/// int32, constant 1 in every packet seen.
inline constexpr std::uint8_t FIELD_35 = 35;
} // namespace field

/// Field numbers of the two eye groups, in the order
/// `[origin, origin_valid, direction, direction_valid, pupil_diameter, openness, aux_pair, pupil_pos, pupil_pos_valid,
/// flag_a, flag_b]`.
inline constexpr std::array<std::array<std::uint8_t, 11>, 2> EYE_FIELDS{{
    {2, 3, 4, 5, 6, 7, 16, 23, 22, 30, 31},
    {8, 9, 10, 11, 12, 13, 17, 25, 24, 32, 33},
}};

/// Field order used on the wire by every packet seen.
inline constexpr std::array<std::uint8_t, 35> WIRE_ORDER{
    1, 2, 3, 4, 5, 6, 7, 16, 23, 22, 30, 31, 8, 9, 10, 11, 12, 13, 17, 25, 24, 32, 33, 14, 15, 18,
    19, 20, 21, 26, 27, 29, 28, 34, 35,
};

/// Nominal period of the gaze stream in microseconds (mode of the timestamp deltas, 120.48 Hz).
inline constexpr std::int64_t NOMINAL_PERIOD_US = 8300;

/// `raw / 65536` (S15.16).
inline double fixed16_16_to_f64(std::int32_t raw) { return static_cast<double>(raw) / 65536.0; }
/// `raw / 2^42` (S22.42).
inline double fixed22_42_to_f64(std::int64_t raw) { return static_cast<double>(raw) / 4'398'046'511'104.0; }

/// Tracking state of one eye derived from the validity flags.
enum class EyeState {
    /// No valid data (eye closed, not worn, or the group is missing from a truncated packet).
    Lost,
    /// The eye position is valid but there is no gaze direction (blink, partial view).
    OriginOnly,
    /// Origin, direction, pupil diameter and pupil position are all valid.
    Tracked,
};

/// One eye. Missing or invalid values hold the device sentinels (`0` for vectors, `-1` for scalars).
struct Eye {
    /// Gaze origin in millimetres (fields 2 / 8). Eye A is about (+31, -2, -22).
    Vec3 origin_mm{0.0, 0.0, 0.0};
    /// Validity of `origin_mm` (fields 3 / 9).
    bool origin_valid = false;
    /// Unit gaze direction (fields 4 / 10); +z is the viewing direction.
    Vec3 direction{0.0, 0.0, 0.0};
    /// Validity of `direction` (fields 5 / 11).
    bool direction_valid = false;
    /// Pupil diameter in millimetres (fields 6 / 12), -1 when invalid.
    double pupil_diameter_mm = -1.0;
    /// Fields 7 / 13: eye openness candidate. -1 in every packet of every capture, so "not provided".
    double openness_raw = -1.0;
    /// Pupil position in the sensor area, 0..1 (fields 23 / 25), -1 when invalid.
    Vec2 pupil_pos{-1.0, -1.0};
    /// Validity of `pupil_pos` (fields 22 / 24).
    bool pupil_pos_valid = false;
    /// Fields 16 / 17: two S22.42 values, (-1, -1) in every packet seen.
    Vec2 aux_pair{-1.0, -1.0};
    /// Fields 30,31 / 32,33: flags. The first is always 1; the second is 0 whenever the eye is fully tracked and
    /// 1 whenever only the origin is valid (observed on eye A), so it may mean "pupil/glints lost".
    std::array<std::int32_t, 2> flags{0, 0};
    /// All 11 fields of this group were present in the packet.
    bool complete = false;

    /// Tracking state from the validity flags.
    EyeState state() const;
    /// Pupil diameter when valid.
    std::optional<double> pupil_diameter() const;
    /// Eye openness when the device provides it (never seen). The Rust field is `openness`; it is `openness_raw`
    /// here because C++ cannot have a member and a function of the same name.
    std::optional<double> openness() const;
    /// Gaze ray `(origin_mm, direction)` when both are valid.
    std::optional<std::pair<Vec3, Vec3>> ray() const;
    /// `(yaw, pitch)` of the gaze direction in degrees, `yaw = atan2(x, z)`, `pitch = atan2(y, hypot(x, z))`, when
    /// valid. The axis signs are those of the device frame (x toward eye A's side, +z forward).
    std::optional<std::pair<double, double>> yaw_pitch_deg() const;

    friend bool operator==(const Eye&, const Eye&) = default;
};

/// Frame-level fields other than timestamp and counter. All unexplained constants.
struct FrameFields {
    /// Field 14.
    std::int32_t field_14 = 0;
    /// Fields 18 and 20.
    std::array<Vec3, 2> aux_points{{{-1.0, -1.0, -1.0}, {-1.0, -1.0, -1.0}}};
    /// Fields 19 and 21.
    std::array<bool, 2> aux_valid{false, false};
    /// Fields 26, 27, 28.
    std::array<std::int32_t, 3> flags{0, 0, 0};
    /// Field 29.
    double field_29 = -1.0;
    /// Field 34.
    double field_34 = 0.0;
    /// Field 35.
    std::int32_t field_35 = 0;

    friend bool operator==(const FrameFields&, const FrameFields&) = default;
};

/// One decoded gaze notification.
struct GazeSample {
    /// Device timestamp in microseconds (field 1); +8300 per packet nominally, same crystal as the IMU clock.
    std::int64_t timestamp_us = 0;
    /// Frame counter (field 15), empty when the packet was cut before it.
    std::optional<std::uint32_t> frame_counter;
    /// Eye A (fields 2-7, 16, 22, 23, 30, 31) and eye B (fields 8-13, 17, 24, 25, 32, 33).
    std::array<Eye, 2> eyes{};
    /// Remaining fields.
    FrameFields frame{};
    /// The payload ended before all announced elements were read (capture snap length, or a short read).
    bool truncated = false;
    /// Bit `n` is set when field `n` was decoded.
    std::uint64_t fields = 0;
    /// Elements with a field number this decoder does not know (kept out of the typed struct).
    std::uint32_t unknown_fields = 0;

    /// Whether field `n` was decoded.
    bool has(std::uint8_t n) const { return n < 64 && (fields & (std::uint64_t{1} << n)) != 0; }
    /// Whether all 35 known fields were decoded.
    bool is_complete() const;

    friend bool operator==(const GazeSample&, const GazeSample&) = default;
};

/// Decoder errors (the Rust `GazeError` enum).
struct GazeError {
    enum class Kind {
        /// No payload prefix, or nothing decodable (not even the timestamp).
        TooShort,
        /// The first item is not a `0x...0bb8` container.
        NotGaze,
        /// An element or a field value has the wrong shape (`field`, `reason`).
        BadField,
        /// Strict mode: the payload ends before the announced elements do (`found`, `announced`).
        Truncated,
        /// Strict mode: items remain after the announced elements (`offset` inside the TLV area).
        Trailing,
    };
    Kind kind = Kind::TooShort;
    /// BadField: field number, or -1 when the element header itself is wrong.
    std::int32_t field = 0;
    /// BadField: what was wrong.
    const char* reason = "";
    /// Truncated: elements read and announced.
    std::uint16_t found = 0;
    std::uint16_t announced = 0;
    /// Trailing: offset of the first unexpected item.
    std::size_t offset = 0;

    /// The message of the Rust `Display` impl.
    std::string message() const;

    friend bool operator==(const GazeError& a, const GazeError& b);
};

/// Strict decode of a complete payload (the 2-byte prefix plus all announced elements, nothing after them).
std::expected<GazeSample, GazeError> parse_payload(std::span<const std::uint8_t> payload);

/// Lenient decode for payloads cut by a capture snap length or a short read: every complete element before the cut
/// is kept, `truncated` is set, and absent fields keep their invalid defaults. Needs at least the timestamp.
std::expected<GazeSample, GazeError> parse_payload_lenient(std::span<const std::uint8_t> payload);

/// Re-encode a sample's payload in WIRE_ORDER (all fields; absent ones are written with their defaults).
/// Used to synthesise notifications for tests and the fake chip.
std::vector<std::uint8_t> encode_payload(const GazeSample& s);

/// Running statistics over a stream of samples.
struct GazeStats {
    /// Samples seen.
    std::uint64_t samples = 0;
    /// Samples per eye in each EyeState (`[Lost, OriginOnly, Tracked]`).
    std::array<std::array<std::uint64_t, 3>, 2> states{};
    /// Frames missing according to jumps of the frame counter.
    std::uint64_t counter_gaps = 0;
    /// Timestamp deltas above 1.5 nominal periods.
    std::uint64_t timestamp_gaps = 0;
    /// Timestamp going backwards or standing still.
    std::uint64_t timestamp_regressions = 0;
    /// First device timestamp.
    std::optional<std::int64_t> first_ts;
    /// Last device timestamp.
    std::optional<std::int64_t> last_ts;

    /// Account for one sample.
    void add(const GazeSample& s);
    /// Rate from the device timestamps, in Hz.
    std::optional<double> device_rate_hz() const;

private:
    std::optional<std::uint32_t> last_counter_;
};

/// One-line human readable form used by omarchyvr's `pimax-eye stream --print` (not ported yet).
std::string format_line(const GazeSample& s);

} // namespace pibridge::eyemath
