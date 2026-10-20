// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/pipeline.rs (MIT OR Apache-2.0).
//
// Map building and frame observation over a recording (the per-frame work runs on `threads` workers; the results are
// merged in time order, so they do not depend on the number of threads).
#pragma once

#include "pibridge/roommap/align.h"
#include "pibridge/roommap/pngread.h"
#include "pibridge/roommap/rec.h"
#include "pibridge/roommap/refine.h"
#include "pibridge/roommap/session.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::roommap::pipeline {

using cam::geom::Pose;
using cam::geom::Vec3;

/// Mapping parameters (keyframe density; 40 stereo points per keyframe).
session::SessionParams build_params(double kf_dist, double kf_angle_deg);

/// ORB features of both cameras of a frame set.
std::vector<stereo::Features> frame_features(const stereo::Rig& rig, const pngread::Grey& a, const pngread::Grey& b,
                                             const orb::OrbParams& orb);

struct BuildStats {
    std::size_t frames = 0;
    std::size_t no_pose = 0;
};

/// Session 1: keyframes of a recording in the room frame room_from_slam.
std::expected<std::pair<map::RoomMap, BuildStats>, std::string> build_from_recording(
    const rec::Recording& rec, const stereo::Rig& rig, map::RoomMap map, const session::SessionParams& sp,
    std::size_t every, const session::Alignment& room_from_slam);

/// Match every `every`-th frame set against the map (parallel): (observations sorted by time, frame sets tried).
std::expected<std::pair<std::vector<align::FrameObs>, std::size_t>, std::string> observe_recording(
    const rec::Recording& rec, const map::RoomMap& map, const stereo::Rig& rig, std::size_t every,
    const orb::OrbParams& orb, const align::ObserveParams& op, std::size_t threads);

/// Features of every `every`-th frame set as mapping frames (room_from_slam puts them in the room frame). Parallel.
std::expected<std::vector<refine::MapFrame>, std::string> map_frames(const rec::Recording& rec, const stereo::Rig& rig,
                                                                     std::size_t every, const orb::OrbParams& orb,
                                                                     const session::Alignment& room_from_slam,
                                                                     std::size_t threads);

/// Stereo points of one frame set in the tracker world, those within max_range of the head only.
std::vector<Vec3> frame_points(const stereo::Rig& rig, const std::vector<stereo::Features>& feats,
                               const Pose& slam_from_body, double max_range);

/// A frame set's head pose and stereo points in the tracker world.
struct PointFrame {
    std::int64_t ts_ns = 0;
    Pose pose;
    std::vector<Vec3> pts;
};

/// Stereo points of every `every`-th frame set of a recording (parallel). Sorted by time.
std::expected<std::vector<PointFrame>, std::string> recording_points(const rec::Recording& rec, const stereo::Rig& rig,
                                                                     std::size_t every, double max_range,
                                                                     std::size_t threads);

} // namespace pibridge::roommap::pipeline
