// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/p3p.rs (MIT OR Apache-2.0).
//
// P3P: camera pose from three bearing vectors and three model points (Grunert's method, Haralick et al. 1994),
// with a Durand-Kerner quartic solver polished by Newton steps, then the exact triad alignment.
#pragma once

#include "pibridge/cam/geom.h"

#include <array>
#include <optional>
#include <vector>

namespace pibridge::ctrl::track {

/// Real roots of `c[0] x^4 + c[1] x^3 + c[2] x^2 + c[3] x + c[4]`.
std::vector<double> quartic_real_roots(const std::array<double, 5>& c);

/// Rigid transform that maps the triangle `m` onto the triangle `c` (exact for congruent triangles).
std::optional<cam::geom::Pose> triad_align(const std::array<cam::geom::Vec3, 3>& m,
                                           const std::array<cam::geom::Vec3, 3>& c);

/// All `cam_from_model` poses (up to 4) with `y[i]` (unit bearing in the camera) looking at `x[i]` (model points).
std::vector<cam::geom::Pose> p3p(const std::array<cam::geom::Vec3, 3>& y, const std::array<cam::geom::Vec3, 3>& x);

} // namespace pibridge::ctrl::track
