// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/rest.rs (MIT OR Apache-2.0).
// Where untracked controllers are assumed to be: a fixed place in the head-yaw frame (x right, y up, -z forward,
// metres, head at the origin) that follows the head's yaw (PIMAX_CTRL_REST), the aim level or at the look point
// (PIMAX_CTRL_AIM), the grip pitched up like a held handle.
#pragma once

#include "pibridge/ctrl/pose.h"

#include <expected>
#include <string>
#include <string_view>
#include <utility>

namespace pibridge::ctrl {

/// Placement of one controller's grip in the head-yaw frame (mirrored for the left hand).
struct Rest {
    double side;
    double down;
    double fwd;
    double grip_pitch_deg;

    static constexpr Rest desk() { return Rest{0.15, 0.16, 0.38, 45.0}; }
    static constexpr Rest legacy() { return Rest{0.20, 0.30, 0.35, 0.0}; }
    /// desk, legacy or side,down,fwd[,grip_pitch_deg]; the error text of the Rust code.
    static std::expected<Rest, std::string> parse(std::string_view s);
    /// Grip position in the head-yaw frame.
    V3d position(Hand hand) const;
    /// Grip relative to the aim pose: a pure pitch about +x.
    Pose aim_to_grip() const;
    bool operator==(const Rest&) const = default;
};

/// Where the synthesized aim ray points: level (yaw only) or at the spot `distance` m ahead of the view.
struct AimMode {
    bool head = false;
    double distance = 0.0;
    static constexpr double DEFAULT_HEAD_DISTANCE = 1.0;
    static AimMode level() { return AimMode{}; }
    static AimMode head_at(double d) { return AimMode{true, d}; }
    /// level, head or head:<metres>.
    static std::expected<AimMode, std::string> parse(std::string_view s);
    bool operator==(const AimMode&) const = default;
};

/// Yaw-only part of a head orientation (identity when the head looks straight up or down).
Pose yaw_of(std::array<double, 4> head_q);
/// Aim and grip pose of one controller in the tracking-origin frame with the head at the origin.
std::pair<Pose, Pose> controller_poses(Hand hand, const Rest& rest, AimMode aim, std::array<double, 4> head_q);
/// Direction (unit vector) of a pose's -z axis.
V3d forward(const Pose& p);
/// Angles of a point seen from the head (deg): (right of centre, below centre).
std::pair<double, double> view_angles(std::array<double, 4> head_q, V3d point);

} // namespace pibridge::ctrl
