// SPDX-License-Identifier: MIT
// The passthrough without a headset: libpibridge-roommap's synthetic room (synth.h, floor at y = 0, ceiling 2.6 m)
// rendered through the cameras of the shipped reference calibration, as the camera feed would deliver it. The self
// tests' snapshots (pibridge-roomsetup --map, pibridge-home) and the passthrough tests use it.
#pragma once

#include "pibridge/roommap/synth.h"
#include "pibridge/xrkit/passthrough.h"

#include <expected>
#include <string>

namespace pibridge::xrkit::ptsynth {

/// The embedded Dream Air reference calibration as a rig (view origin: default_head_in_imu); `upper` false: the lower
/// pair only.
std::expected<pt::Rig, std::string> referenceRig(bool upper = true);
/// Every camera image of the synthetic room for a head (view) pose in the room (= the stage).
pt::Frame synthFrame(const roommap::synth::Room& room, const pt::Rig& rig, const room::Mat34& stageFromView, std::uint64_t seq);
cam::geom::Pose geomPose(const room::Mat34& m);
/// A head pose: position, heading (rad, about +y) and pitch (rad, + = up).
room::Mat34 headPose(room::Vec3 p, double heading, double pitch);
/// The synthetic room's walls as a room shape (its shell's floor rectangle and ceiling).
pt::RoomShape roomShape(const roommap::synth::Room& room);

} // namespace pibridge::xrkit::ptsynth
