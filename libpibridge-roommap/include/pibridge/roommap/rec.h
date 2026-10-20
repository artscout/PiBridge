// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/rec.rs (MIT OR Apache-2.0).
//
// Offline input: a 6DoF recording (EuRoC layout: `<dir>/euroc_<stamp>/mav0/{cam0,cam1,imu0}`, cam0/cam1 = the rig's
// stereo pair, 640x480 grey PNG tiles) and the tracker trajectory of the same session (Basalt `tracking.csv`, IMU body
// in Basalt's z-up world, converted here to Monado's y-up world).
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/cam/slam.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::roommap::rec {

using cam::geom::Pose;

/// Basalt's world is z up, ours y up: -90 deg about X (the driver's SLAM_TO_YUP).
extern const cam::slam::Quat SLAM_TO_YUP;

struct FrameRef {
    std::int64_t ts_ns = 0;
    std::string cam0;
    std::string cam1;
    bool operator==(const FrameRef&) const = default;
};

using Traj = std::vector<std::pair<std::int64_t, Pose>>;

struct Recording {
    std::string root;
    std::string tracking;
    /// Frame sets present in both cameras, in cam0's order.
    std::vector<FrameRef> frames;
    /// slam_from_body (y up), sorted by time.
    Traj traj;

    static std::expected<Recording, std::string> open(const std::string& dir, const std::optional<std::string>& tracking);
    /// Short name (the recording directory name).
    std::string name() const;
};

/// EuRoC root of a recording directory (itself if it holds mav0, else the last `*/mav0` by name).
std::expected<std::string, std::string> dataset_root(const std::string& d);
/// Newest `replay-*/tracking.csv` of a directory (by modification time).
std::optional<std::string> find_tracking(const std::string& d);
/// Parse a trajectory (`ts,px,py,pz,qw,qx,qy,qz`, Basalt z up) into y-up slam_from_body poses.
Traj parse_tracking(std::string_view text);
/// Tracker pose at a frame time (nearest sample within 5 ms).
std::optional<Pose> pose_at(const Traj& tr, std::int64_t ts);

} // namespace pibridge::roommap::rec
