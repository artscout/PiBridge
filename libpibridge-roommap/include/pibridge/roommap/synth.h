// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/synth.rs (MIT OR Apache-2.0).
//
// A synthetic textured room rendered through the real KB4 fisheye cameras of a factory calibration: test data for the
// whole room-memory pipeline without a headset. A box (floor at y = 0, ceiling at 2.6 m) with furniture boxes,
// textured with multi-scale value noise plus sharp-edged posters; 2x2 sub-pixel rays per pixel.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/cam/kb4.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace pibridge::roommap::synth {

using cam::geom::Pose;
using cam::geom::Vec3;

/// Unit camera rays of 2x2 sub-pixel samples per pixel (zero vector = outside the model).
struct RayTable {
    std::size_t w = 0;
    std::size_t h = 0;
    std::vector<std::array<std::array<float, 3>, 4>> rays;
    static RayTable make(const cam::Kb4& intr);
};

struct Aabb {
    Vec3 lo{};
    Vec3 hi{};
};

struct Room {
    Aabb shell{{-3.0, 0.0, -3.5}, {3.0, 2.6, 3.5}};
    std::vector<Aabb> boxes{
        {{-2.9, 0.0, -3.4}, {-1.7, 0.75, -2.6}},
        {{1.2, 0.0, -1.0}, {2.0, 1.1, 0.2}},
        {{-0.6, 0.0, 2.0}, {0.7, 0.45, 3.3}},
        {{-2.5, 0.0, 0.5}, {-2.0, 1.8, 1.5}},
    };
    std::uint64_t seed = 42;
    /// Brightness gain (lighting change between sessions).
    double gain = 1.0;

    /// First surface hit by a ray: (distance, texture value 0..255).
    std::optional<std::pair<double, double>> trace(const Vec3& o, const Vec3& d) const;
    /// Render one camera (grey) at room_from_cam (OpenCV camera axes).
    std::vector<std::uint8_t> render(const cam::Kb4& intr, const Pose& room_from_cam) const;
    std::vector<std::uint8_t> render_rays(const RayTable& rays, const Pose& room_from_cam) const;
    /// Distance along a camera ray.
    std::optional<double> depth(const Pose& room_from_cam, const Vec3& ray_cam) const;
};

} // namespace pibridge::roommap::synth
