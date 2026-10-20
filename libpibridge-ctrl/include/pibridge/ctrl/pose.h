// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/pose.rs (MIT OR Apache-2.0).
// Where the wrist is relative to a controller's grip pose: Monado's ht_ctrl_emu grip rule applied to the canonical
// open hand of u_hand_sim_simulate_generic and inverted, so hand_pose = grip_pose * grip_to_wrist(hand).
#pragma once

#include <array>

namespace pibridge::ctrl {

using V3d = std::array<double, 3>;

enum class Hand { Left, Right };

/// Distance from the palm joint to the grip origin, towards the palm surface (same value as ht_ctrl_emu).
inline constexpr double GRIP_OFFSET_FROM_PALM_M = 0.03;

/// Rigid transform: rotation (unit quaternion x, y, z, w) and translation.
struct Pose {
    std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};
    V3d p{0.0, 0.0, 0.0};

    static Pose identity() { return Pose{}; }
    static Pose from_position(V3d p) { return Pose{{0.0, 0.0, 0.0, 1.0}, p}; }
    /// Rotate a vector by this pose's rotation.
    V3d rotate(V3d v) const;
    /// self * other: apply other first, then self.
    Pose compose(const Pose& other) const;
    Pose inverse() const;
    /// Rotation from orthonormal axes (columns of the rotation matrix).
    static Pose from_axes(V3d x, V3d y, V3d z, V3d p);
    /// Angle in radians between the rotations of two poses.
    double angle_to(const Pose& other) const;
    bool operator==(const Pose&) const = default;
};

/// The grip pose derived from hand joints, exactly like Monado's ht_ctrl_emu (do_grip_pose).
Pose grip_from_joints(const Pose& palm, V3d index_prox, V3d ring_prox, Hand hand);
/// Grip pose expressed in the wrist frame of the simulated open hand.
Pose grip_in_wrist(Hand hand);
/// Wrist expressed in the grip frame: wrist_pose = grip_pose * grip_to_wrist(hand).
Pose grip_to_wrist(Hand hand);

// The canonical open left hand (tests use it).
inline constexpr V3d PALM_LEFT = {0.00329, 0.00095, -0.06443};
inline constexpr V3d INDEX_PROXIMAL_LEFT = {0.02792, 0.0, -0.09429};
inline constexpr V3d RING_PROXIMAL_LEFT = {-0.01645, 0.00047, -0.08656};

} // namespace pibridge::ctrl
