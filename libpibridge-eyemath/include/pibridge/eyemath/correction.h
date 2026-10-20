// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/correction.rs (MIT OR Apache-2.0).
//
// Host-side accuracy correction of the combined gaze direction (the older, single map; the per-eye calibration in
// calib.h reuses its polynomial).
//
// The EyeChip's own calibration is never touched (no calibration opcode is ever sent). The error of the factory gaze
// is instead corrected on the host, in the head frame, by a smooth map fitted from a measurement: a polynomial on the
// gaze tangent `(u, v) = (x / -z, y / -z)` of a head-frame direction (OpenXR axes: x right, y up, -z forward):
//
//     u' = cx . f(u, v)        v' = cy . f(u, v)        f(u, v) = (1, u, v, u*u, u*v, v*v)
//
// An affine map has zero coefficients in the last three places. The corrected direction is `(u', v', -1)`
// normalised. Directions that do not point forward (z >= 0) and tangents beyond MAX_TANGENT are left alone.
//
// File (`eye-correction.toml`):
//
//     model = "affine"                       # or "quadratic" (informational)
//     x = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]     # cx; 3 numbers are padded with zeros
//     y = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]     # cy
//     flip = "0,0,1"                         # PIBRIDGE_EYE_FLIP_X,Y,Z of the measurement (a warning if they differ later)
//
// Selecting: `PIBRIDGE_EYE_CORRECTION=<path>` loads that file, `none` (or `off`, `0`) disables it, unset loads the
// default file when it exists: `~/.config/pibridge/eye-correction.toml`, else `~/.config/pimax-linux/...`, else
// `~/.config/omarchyvr/...`.
#pragma once

#include <array>
#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "pibridge/eyemath/track.h"
#include "pibridge/eyemath/types.h"

namespace pibridge::eyemath {

/// Tangents with a larger magnitude are not corrected (a polynomial is not trusted that far out: about 56 degrees).
inline constexpr double MAX_TANGENT = 1.5;

/// The feature vector `f(u, v)`.
std::array<double, 6> features(double u, double v);

/// Tangent of a forward-pointing direction (x right, y up, -z forward); empty for a direction that does not point
/// forward or is not finite.
std::optional<Tangent> tangent_of(const Vec3& d);

/// Unit direction of tangent (u, v).
Vec3 dir_of(double u, double v);

/// A fitted correction map.
struct Correction {
    /// "affine" or "quadratic" (informational; the coefficients decide).
    std::string model = "affine";
    /// Coefficients of the corrected u.
    std::array<double, 6> cx{0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    /// Coefficients of the corrected v.
    std::array<double, 6> cy{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    /// `PIBRIDGE_EYE_FLIP_X/Y/Z` in force when the measurement was taken, when recorded.
    std::optional<std::array<bool, 3>> flips;

    /// The map that changes nothing (also what a default-constructed Correction is).
    static Correction identity();
    /// True when the map is the identity.
    bool is_identity() const;
    /// The corrected tangent of tangent (u, v).
    Tangent map_tangent(double u, double v) const;
    /// The corrected unit direction of the head-frame direction `d` (returned unchanged when it does not point
    /// forward or lies beyond MAX_TANGENT).
    Vec3 apply(const Vec3& d) const;
    /// apply() on float directions.
    Vec3f apply_f32(const Vec3f& d) const;
    /// Correct the combined gaze direction and the valid per-eye directions of `s` (the origins stay).
    void apply_sample(Sample& s) const;

    /// The file text; `notes` become leading comment lines.
    std::string to_toml(std::span<const std::string> notes = {}) const;
    /// Parse the file text.
    static std::expected<Correction, Error> parse(std::string_view text);
    /// Load a file.
    static std::expected<Correction, Error> load(const std::filesystem::path& path);
    /// `~/.config/pibridge/eye-correction.toml` (where a new file goes); empty without `$HOME`.
    static std::optional<std::filesystem::path> default_path();
    /// The default file that exists under `home`: the pibridge one, else the pimax-linux one, else the omarchyvr one.
    static std::optional<std::filesystem::path> find_default(const std::optional<std::filesystem::path>& home);
    /// Selection by the value of `PIBRIDGE_EYE_CORRECTION` (see above): an empty optional for "no correction".
    static std::expected<std::optional<Correction>, Error> select(std::optional<std::string_view> var,
                                                                  const std::optional<std::filesystem::path>& home);
    /// select() with the real environment (`PIBRIDGE_EYE_CORRECTION`, `HOME`).
    static std::expected<std::optional<Correction>, Error> from_env();
    /// A warning when the file was measured with other `PIBRIDGE_EYE_FLIP_*` settings than `conv`.
    std::optional<std::string> flip_warning(const FrameConv& conv) const;

    friend bool operator==(const Correction&, const Correction&) = default;
};

/// `$HOME` as a path, empty when unset.
std::optional<std::filesystem::path> home_dir();

} // namespace pibridge::eyemath
