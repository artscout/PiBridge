// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/kb4.rs (MIT OR Apache-2.0).
//
// Kannala-Brandt fisheye model (OpenCV fisheye, Monado T_DISTORTION_FISHEYE_KB4).
//
// theta = atan2(r, z), theta_d = theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8 [+ k5 theta^10 +
// k6 theta^12]), u = fx * theta_d * x / r + cx, v = fy * theta_d * y / r + cy (OpenCV camera axes: x right, y down,
// z forward). The factory XML says FISHEYE_4_PARAMETERS but lists 6 numbers; Kb4::k are the first four. The last
// two are kept in Kb4::extra and can be evaluated as k5, k6 with Kb4::with_extra_radial (used only to test that
// hypothesis against real images).
#pragma once

#include "pibridge/cam/geom.h"

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace pibridge::cam {

/// Intrinsics of one fisheye camera. Build with Kb4::make so that the angle limit is computed.
struct Kb4 {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
    /// k1..k4.
    std::array<double, 4> k{};
    /// Further coefficients from the XML (unused by KB4).
    std::vector<double> extra;
    /// Use extra as k5, k6 (radial theta^10, theta^12 terms). Hypothesis test only.
    bool use_extra = false;
    /// Largest incidence angle (radians) where theta_d still increases; projections beyond it are rejected.
    double theta_limit = 0;

    /// New model; computes theta_limit.
    static Kb4 make(std::uint32_t width, std::uint32_t height, double fx, double fy, double cx, double cy,
                    std::array<double, 4> k, std::vector<double> extra);
    /// Copy that also applies the two extra coefficients as radial terms.
    Kb4 with_extra_radial() const;
    /// Distorted angle for an incidence angle theta (radians).
    double theta_d(double theta) const;
    /// d theta_d / d theta.
    double dtheta_d(double theta) const;
    /// Largest incidence angle (radians, capped at 100 deg) for which theta_d is still increasing.
    double compute_theta_max() const;
    /// Project a 3D point (camera frame) to pixels. nullopt for the origin, points behind or angles beyond
    /// theta_limit. The result can lie outside the image; callers check bounds.
    std::optional<std::pair<double, double>> project(const geom::Vec3& p) const;
    /// Unit ray through pixel (u, v) (Newton iteration on theta_d). nullopt outside the model's valid disc.
    std::optional<geom::Vec3> unproject(double u, double v) const;
    /// Incidence angle of a pixel's ray in degrees.
    std::optional<double> angle_of_pixel_deg(double u, double v) const;

    bool operator==(const Kb4&) const = default;

private:
    std::size_t n_coeffs() const;
    double coeff(std::size_t i) const { return i < 4 ? k[i] : extra[i - 4]; }
};

} // namespace pibridge::cam
