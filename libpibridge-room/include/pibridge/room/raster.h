// SPDX-License-Identifier: MIT
// A tiny software renderer for the scene lists (draw.h): no depth, triangles alpha-blended in order, one-pixel lines.
// pibridge-roomsetup --selftest --snapshot uses it to show what the headset would see without a headset or GPU.
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/vmath.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::room {

struct Image {
    int w = 0;
    int h = 0;
    std::vector<std::uint8_t> rgb; // w * h * 3
};

/// Render `b` seen through `viewProj` (Vulkan clip conventions, as projectionVulkan) on `background`.
Image rasterize(const Batch& b, const Mat4& viewProj, int w, int h, Rgba background);
/// The same over an existing picture (pibridge-roomsetup --map's snapshots: the scene over the passthrough).
void rasterizeOnto(Image& img, const Batch& b, const Mat4& viewProj);
/// Binary PPM (P6).
bool writePpm(const Image& img, const std::string& path);
/// A view matrix (stage -> eye) for an eye at `eye` looking at `at`.
Mat4 lookAt(Vec3 eye, Vec3 at);

} // namespace pibridge::room
