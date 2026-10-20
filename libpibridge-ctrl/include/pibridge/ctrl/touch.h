// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/touch.rs (MIT OR Apache-2.0).
// Grip and aim of a tracked controller from the Oculus Touch (Quest 2) openxr_aim / openxr_grip components SteamVR
// uses: grip = aim * aim_to_grip(), so SteamVR's derived aim equals the driver's aim.
#pragma once

#include "pibridge/ctrl/pose.h"

namespace pibridge::ctrl::touch {

inline constexpr V3d OPENXR_GRIP_ORIGIN = {0.007, -0.00182941, 0.1019482};
inline constexpr double OPENXR_GRIP_PITCH_DEG = 20.6;
inline constexpr V3d OPENXR_AIM_ORIGIN = {0.007, -0.03894766, 0.00949694};
inline constexpr double OPENXR_AIM_PITCH_DEG = -39.4;

/// openxr_grip component (pose of the grip in the Touch raw frame).
Pose openxr_grip(bool left);
/// openxr_aim component.
Pose openxr_aim(bool left);
/// Grip relative to aim of a tracked controller (the same for both hands).
Pose aim_to_grip();
/// A pure pitch about +x (tests use it).
std::array<double, 4> pitch(double deg);

} // namespace pibridge::ctrl::touch
