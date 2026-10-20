// SPDX-License-Identifier: MIT
// The VR home's 360-degree background (backgrounds.h): an equirectangular picture (2:1, longitude 0 straight ahead
// at the start = -z, growing to the right; top row = straight up) drawn infinitely far behind everything. The GPU
// pass is xrkit's sky (shaders/sky.frag); this is the same lookup on the CPU (snapshots, tests), the picture file
// reader and the built-in "dusk" picture.
#pragma once

#include "pibridge/room/raster.h"
#include "pibridge/room/vmath.h"

#include <cstdint>
#include <expected>
#include <string>
#include <vector>

namespace pibridge::home::sky {

struct Pano {
    int w = 0, h = 0;
    std::vector<std::uint8_t> rgb; // w * h * 3, sRGB
};
/// A binary PPM (P6, 255). Pictures wider than `maxW` are halved until they fit (the GPU texture; 4096 by default).
std::expected<Pano, std::string> loadPpm(const std::string& path, int maxW = 4096);
/// The built-in picture: a dusk sky over a dark plain with a faint horizon glow and stars (deterministic).
Pano dusk(int w = 2048, int h = 1024);
/// RGBA (alpha 255) for the GPU upload.
std::vector<std::uint8_t> toRgba(const Pano& p);
/// Longitude / latitude texture coordinates of a stage direction (the shader's arithmetic), yaw in radians.
void dirToSt(room::Vec3 d, double yaw, double& s, double& t);
/// The picture in direction `d` (sRGB bytes, bilinear).
std::array<std::uint8_t, 3> sample(const Pano& p, room::Vec3 d, double yaw = 0);
/// An eye image of the sky (the tan of the fov as in OpenXR: left, right negative-left, up, down negative).
room::Image render(const Pano& p, const room::Mat34& stageFromEye, double tanL, double tanR, double tanU, double tanD,
                   int w, int h, double yaw = 0);

} // namespace pibridge::home::sky
