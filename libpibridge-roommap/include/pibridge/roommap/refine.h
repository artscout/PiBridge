// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/refine.rs (MIT OR Apache-2.0).
//
// Multi-view refinement of the map points: every point is searched in the mapping session's frames within a few
// seconds of its keyframe (projected into each camera's virtual pinhole, descriptor matched near the projection) and
// re-triangulated from all observations (Gauss-Newton on the angular error, outliers gated); points without enough
// parallax or consistency can be dropped.
#pragma once

#include "pibridge/roommap/map.h"
#include "pibridge/roommap/stereo.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pibridge::roommap::refine {

using cam::geom::Pose;

/// Keypoint buckets of one image.
struct Grid {
    float cell = 16.0f;
    std::size_t cols = 0;
    std::size_t rows = 0;
    std::vector<std::vector<std::uint32_t>> cells;
    static Grid make(const stereo::Features& f, std::size_t w, std::size_t h, float cell);
    void near(float x, float y, float r, std::vector<std::uint32_t>& out) const;
};

/// A frame of the mapping session: time, room_from_body, features of the rig cameras (images dropped).
struct MapFrame {
    std::int64_t ts_ns = 0;
    Pose room_from_body;
    std::vector<stereo::Features> feats;
    std::vector<Grid> grids;
    static MapFrame make(std::int64_t ts_ns, const Pose& room_from_body, std::vector<stereo::Features> feats,
                         const stereo::Rig& rig);
};

struct RefineParams {
    std::int64_t window_ns = 6000000000;
    float radius_px = 10.0f;
    std::uint32_t max_dist = 50;
    std::size_t min_obs = 4;
    double min_parallax_deg = 3.0;
    double gate_deg = 0.8;
    double max_rms_deg = 0.35;
    double max_move_m = 1.5;
    double max_move_frac = 0.6;
    bool drop_unrefined = true;
};

struct RefineStats {
    std::size_t points = 0;
    std::size_t refined = 0;
    std::size_t dropped = 0;
    double median_move_m = 0;
    double median_obs = 0;
};

/// Refine the points of every keyframe from the frames of the mapping session (`threads` workers).
RefineStats refine_map(map::RoomMap& map, const stereo::Rig& rig, const std::vector<MapFrame>& frames,
                       const RefineParams& p, std::size_t threads);

} // namespace pibridge::roommap::refine
