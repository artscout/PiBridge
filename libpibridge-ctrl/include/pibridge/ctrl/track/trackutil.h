// SPDX-License-Identifier: MIT
// PiBridge: small helpers shared by the controller trackers (imutrack, wintrack).
#pragma once

#include "pibridge/ctrl/track/rig.h"
#include "pibridge/ctrl/track/solve.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace pibridge::ctrl::track {

/// The observations of per-camera matches, for refine() / refine_with().
std::vector<Obs> obs_list(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                          const std::vector<Matches>& per_cam);
/// Total matches over the cameras.
std::size_t count(const std::vector<Matches>& m);
std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b);
/// Angle between two rotations (deg).
double angle_deg(const cam::geom::Mat3& a, const cam::geom::Mat3& b);
/// The rigid map taking a onto b (least squares): the rotation by the polar decomposition (Newton iterations).
std::optional<cam::geom::Pose> rigid_fit(const std::vector<cam::geom::Vec3>& a, const std::vector<cam::geom::Vec3>& b);
/// The LED model's near-symmetry: a rigid map, turned at least 120 deg, that takes the LEDs onto LEDs (and the normals
/// onto normals) within 10 mm rms; nullopt without one.
std::optional<std::pair<cam::geom::Pose, double>> model_twin(const LedModel& m);

} // namespace pibridge::ctrl::track
