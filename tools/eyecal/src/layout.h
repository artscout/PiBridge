// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/layout.rs (MIT OR Apache-2.0).
//
// Where the targets are. Everything stays inside a comfortable eye rotation (about +-15 degrees horizontally and
// +-12 vertically, well inside the sharp centre of the lenses) so that the wearer never has to turn the head: the
// targets are head-locked, a head turn moves them along.
//
// Angles: azimuth right, elevation up, degrees, from the head origin (between the eyes); targets sit on a sphere of
// TARGET_DIST_M. The on-chip calibration's targets come from the captured Tobii requests instead (chip_targets): their
// directions are the chip's own target points, put into the head frame by the same device-to-head conversion as the
// gaze, so a target is shown where the request tells the chip it is.
#pragma once

#include "pibridge/eyemath/track.h"
#include "pibridge/eyemath/types.h"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::eyecal {

using pibridge::eyemath::Vec3;

/// Distance of the head-locked targets, metres (far enough that the per-eye parallax is small and the vergence
/// relaxed, near enough to fuse easily).
inline constexpr double TARGET_DIST_M = 2.0;

/// One target.
struct Target {
    /// Azimuth, degrees (right positive).
    double az = 0.0;
    /// Elevation, degrees (up positive).
    double el = 0.0;
    /// Shown on a white background (the pupil narrows: the fit sees another pupil size).
    bool bright = false;
    /// Index into pibridge::eye::calibration::targets() of an on-chip calibration target (-1: none).
    int chip = -1;
    /// Distance of the target point, metres (the chip's targets are farther than TARGET_DIST_M off the axis).
    double dist_m = TARGET_DIST_M;

    /// New target.
    static constexpr Target at(double az, double el) { return Target{az, el, false, -1, TARGET_DIST_M}; }
    /// New target on a white background.
    static constexpr Target on_white(double az, double el) { return Target{az, el, true, -1, TARGET_DIST_M}; }

    /// Unit head-frame direction (x right, y up, -z forward).
    Vec3 dir() const;
    /// The target point, metres, head frame.
    Vec3 point() const;
    /// Angle from straight ahead, degrees.
    double eccentricity() const;
    /// Short name, e.g. `+15,-12`.
    std::string name() const;

    bool operator==(const Target&) const = default;
};

/// Unit direction at azimuth `az` / elevation `el` (degrees).
Vec3 dir_az_el(double az, double el);

/// Horizontal extent of the calibration grid.
inline constexpr double CAL_AZ = 15.0;
/// Vertical extent of the calibration grid.
inline constexpr double CAL_EL = 12.0;

/// Calibration targets: 9 = 3 x 3 grid at (0, +-15) x (0, +-12); 13 adds the four inner diagonals (+-7.5, +-6), which
/// lets the quadratic model see the field between the grid lines. Centre first, then a short path.
std::vector<Target> calibration_targets(std::size_t points);

/// The calibration path of Pimax Play (as the user saw it on Windows, 2026-10-18): a green dot on black at the
/// centre, then the corners bottom-left, top-right, bottom-right, top-left and back to the centre; then on white
/// left, right, centre, up, down. The dot travels from one place to the next. Same extent as the grid.
std::vector<Target> pimax_targets();

/// Validation targets, all different from the calibration targets and inside the calibrated field: two near the
/// centre (5 degrees), four mid (8-10 degrees), four outer (15 degrees, between the grid points).
std::vector<Target> validation_targets();

/// Recentre (drift) targets: centre and four at +-8 / +-6. Also the short validation after the on-chip calibration.
std::vector<Target> recenter_targets();

/// Service-path check targets (through the runtime, combined gaze).
std::vector<Target> service_targets();

/// How the chip's target table maps to the screen. The table's x, y are in the chip's own frame (the frame of its gaze
/// origins and directions); `conv` (the tracker's FrameConv) turns them into the head frame like the gaze. The
/// mirrors exist for the first live test only (if the dots turn out mirrored against Pimax Play's path).
struct ChipLayout {
    pibridge::eyemath::FrameConv conv;
    bool mirror_x = false;
    bool mirror_y = false;
};

/// The nine on-chip calibration targets in the captured order (pass 1: centre and diagonals on black, pass 2: the
/// edges on white), each pointing at the chip's own target point.
std::vector<Target> chip_targets(const ChipLayout& l);

/// Region of a target for the report.
enum class Region {
    /// Up to 7 degrees from the centre.
    Centre,
    /// 7 to 12 degrees.
    Mid,
    /// Further out.
    Outer,
};

/// All regions.
inline constexpr std::array<Region, 3> ALL_REGIONS = {Region::Centre, Region::Mid, Region::Outer};
/// Name.
const char* region_name(Region r);
/// Region of a target.
Region region_of(const Target& t);

/// Radius of the pursuit circle (horizontal; vertical is 0.8 of it).
inline constexpr double PURSUIT_RADIUS_DEG = 10.0;
/// Duration of one pursuit turn.
inline constexpr double PURSUIT_PERIOD_S = 12.0;

/// Smooth-pursuit path: a circle of PURSUIT_RADIUS_DEG around the centre, one turn in PURSUIT_PERIOD_S (4.5 deg/s, an
/// easy pursuit speed), starting at the right. `t` seconds from the start.
Target pursuit_at(double t);

} // namespace pibridge::eyecal
