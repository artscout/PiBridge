// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/pvrfit.rs (MIT OR Apache-2.0): only the dump reader (`parse_dump`, the data
// types) that Lut::from_dump and its tests need. The fit itself (`fit-pvr`) is the later setup tool (plan M10).
//
// A dump (`windows/pimax-dump-distortion.ps1`) holds, per eye and colour channel, `pvr_getHmdDistortedUV` sampled on
// a grid and along four radial lines: panel UV -> UV into the undistorted render texture spanning the FOV port.
#pragma once

#include "pibridge/lens/config.h"

#include <array>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::lens {

/// One radial line: direction in panel UV (`+x`, `-x`, `+y`, `-y`) and the output UVs per channel.
struct LineData {
    std::string name;
    std::array<double, 2> dir{};
    std::array<std::vector<double>, CHANNELS> out;
};

/// Everything the fit needs of one eye.
struct EyeDump {
    std::string name;
    /// FOV port tangents: up, down, left, right (all positive).
    std::array<double, 4> fov_tan{};
    /// HmdToEyePose orientation (x, y, z, w) and position.
    std::array<double, 4> quat{};
    std::array<double, 3> pos{};
    /// Native panel size of this eye in pixels as the runtime reports it.
    std::pair<double, double> panel{};
    std::size_t grid_n = 0;
    /// Output UV pairs `[u0, v0, u1, v1, ...]`, row-major with v outer, per channel (R, G, B).
    std::array<std::vector<double>, CHANNELS> grid;
    std::vector<LineData> lines;
    std::size_t line_n = 0;
    std::int64_t failed_calls = 0;
};

struct Dump {
    std::string serial;
    std::string product;
    std::array<EyeDump, 2> eyes;
    /// `view_rotation_fix` config value read from the runtime, if present.
    std::optional<std::int64_t> view_rotation_fix;
};

/// Parse the JSON text of a `pimax-distortion-*.json`.
std::expected<Dump, std::string> parse_dump(std::string_view text);

} // namespace pibridge::lens
