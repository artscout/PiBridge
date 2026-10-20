// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/align.rs (MIT OR Apache-2.0).
//
// Relocalisation against a saved room map pooled over many frames. Each frame that matches the map (observe: ORB of
// both cameras, keyframes ranked by quick descriptor hits, brute-force matching against the points of the best ones,
// RANSAC 4-DoF PnP) contributes its 2D-3D correspondences, and ONE room_from_slam (yaw about up + translation) is fitted
// jointly over all frames of a window with each frame's tracker pose (joint_fit: robust start = median of the per-frame
// hypotheses, Huber Levenberg-Marquardt with numeric derivatives, re-gated against the joint model).
#pragma once

#include "pibridge/roommap/map.h"
#include "pibridge/roommap/pnp.h"
#include "pibridge/roommap/session.h"
#include "pibridge/roommap/stereo.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::roommap::align {

using cam::geom::Pose;
using cam::geom::Vec3;
using session::Alignment;

struct ObserveParams {
    std::size_t top_k = 4;
    std::size_t rank_sample = 250;
    std::uint32_t rank_dist = 45;
    std::uint32_t max_dist = 64;
    float ratio = 0.85f;
    pnp::PnpParams pnp{0.8, 500, 0.999, 12, 7};
    std::size_t min_inliers = 15;
    double min_inlier_frac = 0.15;
};

/// One frame matched against the map.
struct FrameObs {
    std::int64_t ts_ns = 0;
    Pose slam_from_body;
    /// Bearing in the camera frame, point in room coordinates.
    std::vector<pnp::Corr> corrs;
    /// Keyframe index (into map.keyframes) of each correspondence's point.
    std::vector<std::uint32_t> corr_kf;
    /// Per-frame RANSAC inliers (indices into corrs).
    std::vector<std::size_t> inliers;
    Pose room_from_body;
    Alignment hyp;
    double rms_deg = 0;
};

/// Match one frame against the map; nullopt when the room is not seen.
std::optional<FrameObs> observe(const map::RoomMap& map, const stereo::Rig& rig, std::span<const stereo::Features> feats,
                                std::int64_t ts_ns, const Pose& slam_from_body, const ObserveParams& p);
/// observe restricted to the keyframes `kfs` (indices into map.keyframes).
std::optional<FrameObs> observe_kfs(const map::RoomMap& map, std::span<const std::size_t> kfs, const stereo::Rig& rig,
                                    std::span<const stereo::Features> feats, std::int64_t ts_ns,
                                    const Pose& slam_from_body, const ObserveParams& p);

struct JointParams {
    double gate_deg = 1.0;
    double huber_deg = 0.4;
    std::size_t rounds = 3;
    double hyp_m = 0.25;
    double hyp_deg = 4.0;
};

struct JointFit {
    Alignment align;
    std::size_t frames = 0;
    std::size_t inliers = 0;
    double rms_deg = 0;
    double sigma_deg = 0;
    double sigma_m = 0;
    double hyp_rms_m = 0;
    double hyp_rms_deg = 0;
    std::pair<std::int64_t, std::int64_t> span_ns{0, 0};
};

/// Wrap an angle to (-pi, pi].
double wrap(double a);
/// Component-wise median of alignments (yaw unwrapped around the first).
std::optional<Alignment> median_alignment(std::span<const Alignment> v);
/// Distance of two alignments: (translation m, yaw deg).
std::pair<double, double> diff(const Alignment& a, const Alignment& b);

/// Fit one room_from_slam over all correspondences of `obs`.
std::optional<JointFit> joint_fit(std::span<const FrameObs* const> obs, std::span<const Pose> bfc, const JointParams& p);

struct AcceptParams {
    std::int64_t window_ns = 4000000000;
    std::size_t min_frames = 12;
    std::size_t min_inliers = 300;
    double min_agree = 0.6;
};

/// First accepted fit when the frames arrive in time order: (index of the frame, the fit).
std::optional<std::pair<std::size_t, JointFit>> first_fix(std::span<const FrameObs> obs, std::span<const Pose> bfc,
                                                          const JointParams& jp, const AcceptParams& ap);
/// Joint fits over consecutive windows.
std::vector<JointFit> window_fits(std::span<const FrameObs> obs, std::span<const Pose> bfc, const JointParams& jp,
                                  std::int64_t window_ns, std::size_t min_frames);

} // namespace pibridge::roommap::align
