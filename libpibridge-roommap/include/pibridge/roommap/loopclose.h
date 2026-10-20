// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/loopclose.rs (MIT OR Apache-2.0).
//
// Loop closure inside the mapping session: every keyframe is relocalised against the keyframes of other times of the
// same session (more than min_gap away; then again on the keyframes near the best one); each success is a loop edge.
// With the VIO's relative motion between consecutive keyframes as odometry edges, a 4-DoF pose graph (yaw +
// translation per keyframe; roll/pitch from gravity) is solved by Gauss-Newton (Huber on loops, outliers dropped at
// chi > reject_chi, solved again). The first keyframe stays fixed. Keyframes and points move with their node, the
// frames between keyframes get the time-interpolated correction.
#pragma once

#include "pibridge/roommap/align.h"
#include "pibridge/roommap/map.h"
#include "pibridge/roommap/refine.h"
#include "pibridge/roommap/session.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::roommap::loopclose {

using cam::geom::Pose;
using cam::geom::Vec3;
using session::Alignment;

struct LoopParams {
    std::int64_t min_gap_ns = 12000000000;
    std::int64_t local_ns = 4000000000;
    std::size_t min_inliers = 30;
    double min_frac = 0.3;
    double max_rms_deg = 0.5;
    double max_jump_m = 2.0;
    double max_jump_deg = 15.0;
    double sigma_loop_m = 0.03;
    double sigma_loop_deg = 0.3;
    double odo_frac = 0.02;
    double odo_min_m = 0.003;
    double odo_deg_per_m = 0.3;
    double odo_min_deg = 0.05;
    double reject_chi = 4.0;
};

struct LoopStats {
    std::size_t tried = 0;
    std::size_t loops = 0;
    std::size_t rejected = 0;
    double max_corr_m = 0;
    double max_corr_deg = 0;
};

/// Correction of the tracker trajectory at a time: room_new_from_room_old.
struct Correction {
    std::int64_t ts_ns = 0;
    Alignment c;
};

/// Pose of b relative to a (yaw difference, translation in a's yaw frame).
struct Edge {
    std::size_t a = 0;
    std::size_t b = 0;
    double dyaw = 0;
    Vec3 dt{};
    double wt = 0;
    double wy = 0;
    bool is_loop = false;
};

using Node = std::pair<double, Vec3>;

std::pair<double, Vec3> edge_residual(const Edge& e, std::span<const Node> x);
/// Gauss-Newton on the node poses (node 0 fixed); loop edges get a Huber weight at `huber` sigmas.
void solve_graph(std::span<Node> x, std::span<const Edge> edges, std::size_t iters, double huber);
/// Interpolated correction at ts (clamped at the ends).
Alignment correction_at(std::span<const Correction> c, std::int64_t ts);
/// Detect loops, solve the pose graph, move keyframes, points and frames: the counters and the per-keyframe
/// corrections (by time).
std::pair<LoopStats, std::vector<Correction>> close_loops(map::RoomMap& map, const stereo::Rig& rig,
                                                          std::vector<refine::MapFrame>& frames,
                                                          const align::ObserveParams& op, const LoopParams& p,
                                                          std::size_t threads);

} // namespace pibridge::roommap::loopclose
