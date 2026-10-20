// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/pnp.rs (MIT OR Apache-2.0).
//
// Gravity-aware absolute pose of a multi-camera fisheye rig against a 3D map (4 DoF: yaw + translation). The VIO
// supplies roll and pitch, so two 2D-3D correspondences are a minimal sample (solve_p2: the u-free equation
// cos * alpha + sin * beta = gamma intersected with the unit circle, depths by 2x2 least squares, cheirality by the
// depth along the bearing). ransac samples pairs (splitmix64), scores by truncated angular error (MSAC), polishes
// promising hypotheses (local optimisation) and refines the winner by Huber Levenberg-Marquardt (refine).
//
// Conventions: body_from_cam[c] maps camera c (OpenCV axes) into the body (IMU, OpenXR head axes); the map frame is
// gravity aligned, +Y up; map_from_body = (Ry(yaw) * r_tilt, t). Nothing panics on bad data: invalid correspondences
// (camera index outside the rig, zero or non-finite bearing or point) are ignored.
#pragma once

#include "pibridge/cam/geom.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::roommap::pnp {

using cam::geom::Mat3;
using cam::geom::Pose;
using cam::geom::Vec3;

/// One 2D-3D correspondence of the rig.
struct Corr {
    std::size_t cam = 0;
    /// Observed ray in that camera's frame (normalised internally).
    Vec3 bearing{};
    /// The 3D point in the map frame (m).
    Vec3 point{};
    bool operator==(const Corr&) const = default;
};

/// Heading and position of the body in the map.
struct Pose4 {
    double yaw = 0;
    Vec3 t{0.0, 0.0, 0.0};
    /// map_from_body = (Ry(yaw) * r_tilt, t).
    Pose to_pose(const Mat3& r_tilt) const;
    bool operator==(const Pose4&) const = default;
};

/// Rotation about +Y by `yaw` radians.
Mat3 rot_y(double yaw);

struct PnpParams {
    double thresh_deg = 1.0;
    std::size_t max_iters = 300;
    double confidence = 0.999;
    std::size_t min_inliers = 15;
    std::uint64_t seed = 0x9E3779B97F4A7C15ull;
    bool operator==(const PnpParams&) const = default;
};

struct PnpResult {
    Pose4 pose;
    /// Indices into the input correspondences, ascending.
    std::vector<std::size_t> inliers;
    double rms_deg = 0;
    std::size_t iterations = 0;
    bool operator==(const PnpResult&) const = default;
};

/// Minimal solver: up to two poses from two correspondences (empty for degenerate samples).
std::vector<Pose4> solve_p2(const std::array<Corr, 2>& c, std::span<const Pose> body_from_cam, const Mat3& r_tilt);

/// RANSAC with solve_p2; nullopt with fewer than min_inliers valid correspondences, no hypothesis, too few final
/// inliers or a non-positive threshold.
std::optional<PnpResult> ransac(std::span<const Corr> corrs, std::span<const Pose> body_from_cam, const Mat3& r_tilt,
                                const PnpParams& p);

/// Angular residual (deg) of one correspondence for a pose (inf for invalid ones).
double residual_deg(const Corr& c, std::span<const Pose> body_from_cam, const Mat3& r_tilt, const Pose4& pose);

/// Huber Levenberg-Marquardt on (yaw, t) over the correspondences `idx`: (pose, RMS of the plain angular residuals
/// in degrees over the valid entries, NaN when there are none).
std::pair<Pose4, double> refine(std::span<const Corr> corrs, std::span<const std::size_t> idx,
                                std::span<const Pose> body_from_cam, const Mat3& r_tilt, const Pose4& init,
                                double huber_deg);

/// Internals the Rust unit tests reach.
namespace detail {

struct SplitMix64 {
    std::uint64_t s;
    std::uint64_t next_u64();
    std::size_t below(std::size_t n);
    double unit();
};

double wrap_pi(double a);

struct Ray {
    Vec3 p;
    Vec3 d;
    Vec3 x;
    static std::optional<Ray> make(const Corr& c, std::span<const Pose> body_from_cam, const Mat3& r_tilt);
};

struct Lin {
    Vec3 r;
    std::array<std::array<double, 4>, 3> j;
    double cos;
};
std::optional<Lin> linearize(const Ray& ray, double c, double s, const Vec3& t);
std::optional<std::array<double, 4>> solve4(std::array<std::array<double, 4>, 4> a, std::array<double, 4> b);
std::size_t iterations_needed(double w, double confidence);

} // namespace detail

} // namespace pibridge::roommap::pnp
