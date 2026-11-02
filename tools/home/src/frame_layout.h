// SPDX-License-Identifier: MIT
// The desktop's frame in C++ (the same numbers as world/scripts/layout.gd): a frame at the eyes turned to where they
// look (yaw only), the interface size's distance ahead and its scale; in it the first monitor in the middle (2.19 wide,
// 0.065 above the frame's middle), the second on the right (x 2.012, 0.727 nearer, turned 40 degrees to you), a third
// on the left; Zoom scales them about the frame's middle and Distance moves them away. Pure (no runtime): SteamVR's
// Desktop tab (tools/overlay) places its monitor overlays with it.
#pragma once

#include "pibridge/room/vmath.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::home::frame {

struct UiScale {
    double forward; // m ahead of the eyes
    double scale;
};
/// "Large" 2.3 / 1.0, "Medium" 1.55 / 0.75 (the default), "Small" 0.85 / 0.5, "Miniature" 0.4 / 0.3.
UiScale uiScale(const std::string& name);

inline constexpr double kMonW = 2.19, kMonH = 1.231;
inline constexpr double kCenterY = 0.065, kCenterZ = 0.0005; // z: towards you
inline constexpr double kSideX = 2.012, kSideZ = 0.727, kSideYawDeg = 40.0;
inline constexpr double kZoomMin = 0.5, kZoomMax = 5.0, kZoomStep = 0.25;
inline constexpr double kDistanceMin = -1.5, kDistanceMax = 1.5, kDistanceStep = 0.25;

struct Monitor {
    std::uint32_t index = 0;
    std::int32_t x = 0, y = 0;            // on the desktop (its order: left to right)
    std::uint32_t logicalW = 0, logicalH = 0;
};

struct Placed {
    std::uint32_t monitor = 0;
    room::Mat34 pose;   // the monitor's middle, its face (+z) towards you, x along its rows, y up
    double w = 0, h = 0; // metres
};

/// The shown monitors (`mask`) placed for the head pose `head` (stage / standing space).
std::vector<Placed> place(const room::Mat34& head, const std::vector<Monitor>& mons, std::uint32_t mask, double zoom,
                          double distance, const std::string& uiScaleName = "Medium");

} // namespace pibridge::home::frame
