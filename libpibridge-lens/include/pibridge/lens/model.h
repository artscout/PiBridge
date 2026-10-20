// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/model.rs (MIT OR Apache-2.0).
//
// The lens model: panel position -> client image UV, per colour channel. For a point (u, v) of the user-facing
// display (v = 0 is the top), `p = ((u-.5)*2*aspect, (.5-v)*2) - centre` in half display heights (+x right, +y up).
// Per channel c: `p' = (p - shift_c) / ca_c`, `r2 = |p'|^2`, ray tangent
// `t = base_c * scale * p' * (1 + k1 r2 + k2 r2^2 + ... + k5 r2^5)`; the client renders a pinhole image covering
// the configured FOV, so the sampled texture coordinate is the linear map of the tangent into that image. Every
// float operation keeps the Rust order (which is the old C model's order), so the results are bit-identical.
#pragma once

#include "pibridge/lens/config.h"
#include "pibridge/lens/lut.h"

#include <array>
#include <memory>
#include <optional>
#include <utility>

namespace pibridge::lens {

/// `(float)(M_PI / 180.0)`, the constant of the C `DEG2RAD` macro.
inline constexpr float RAD_PER_DEG = static_cast<float>(3.14159265358979323846 / 180.0);

inline float deg2rad(float x) {
    return x * RAD_PER_DEG;
}

/// Tangent length and its slope at radius `r` for one channel (f64).
std::pair<double, double> radial_tangent(double base, double scale, const Poly& k, double r);

using Uv = std::array<float, 2>;
using Uv3 = std::array<Uv, CHANNELS>;

/// A lens configuration ready to evaluate (read-only after construction, safe to share between threads).
class Lens {
public:
    explicit Lens(LensConfig cfg);

    /// Attach the LUTs that replace the radial model (the config's FOV and centre must already match them).
    Lens with_luts(std::array<std::shared_ptr<const Lut>, 2> luts) &&;

    const Lut* lut(std::size_t view) const { return luts_[view].get(); }
    /// Does `view` have a visibility mask (configured, or the visible area of a LUT)?
    bool has_mask(std::size_t view) const;
    const LensConfig& config() const { return cfg_; }
    const EyeConfig& eye(std::size_t view) const { return cfg_.eye[view]; }
    /// FOV half-angles in degrees: left, right, up, down.
    std::array<float, 4> fov_deg(std::size_t view) const { return cfg_.eye[view].fov; }
    /// tan of the FOV half-angles: left, right, up, down.
    std::array<float, 4> fov_tan(std::size_t view) const { return tans_[view]; }

    /// Ray tangent (x right, y up) that channel `c` shows at panel point (u, v), before the visibility mask.
    Uv tangent(std::size_t view, std::size_t c, float u, float v) const;
    Uv tangent_at(const EyeConfig& e, std::size_t c, float px, float py) const;
    /// Ray tangent -> client image UV.
    Uv tangent_to_uv(std::size_t view, Uv t) const;
    /// Client image UV -> ray tangent.
    Uv uv_to_tangent(std::size_t view, Uv uv) const;
    /// The distortion callback: for the display point (u, v) of `view` the UV to sample for R, G and B.
    Uv3 compute(std::size_t view, float u, float v) const;
    /// Signed distance (half-heights, approximate, positive inside) from the visible area boundary; +inf when the
    /// eye has no mask.
    float mask_distance(std::size_t view, float u, float v) const;
    /// 0 = fully visible, 1 = fully hidden, smooth in between.
    float hide_amount(std::size_t view, float u, float v) const;
    /// Panel radius at which channel `c` shows the ray `angle_deg` off the optical axis (nullopt beyond radius 4).
    std::optional<float> radius_for_angle(std::size_t view, std::size_t c, float angle_deg) const;
    /// First radius at which the tangent stops increasing, within `rmax`; nullopt if monotonic.
    std::optional<float> first_fold(std::size_t view, std::size_t c, float rmax) const;

private:
    std::pair<float, float> centred(const EyeConfig& e, float u, float v) const;

    LensConfig cfg_;
    std::array<std::array<float, 4>, 2> tans_{};
    std::array<std::shared_ptr<const Lut>, 2> luts_{};
};

} // namespace pibridge::lens
