// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/grid.rs (MIT OR Apache-2.0).
//
// Debug rendering of the UV grid of a lens model, without a compositor or a headset (PIMAX_LENS_DEBUG_GRID). The
// picture is the user-facing panel of one eye (3840x3552 scaled down): iso-lines of the client image UV (one colour
// per channel, so a coloured fringe means the channels disagree), the rings at known angles off the optical axis,
// the optical axis, the edge of the rendered FOV and the visibility mask.
#pragma once

#include "pibridge/lens/model.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::lens::grid {

/// Half of the user-facing display height in pixels (3552 / 2); radii in the model are in these units.
inline constexpr float HALF_HEIGHT_PX = 1776.0f;

/// Colours of the angle rings: 10, 20, 30, 40, 50 degrees.
inline constexpr std::array<std::array<std::uint8_t, 3>, 5> RING_COLORS = {{
    {255, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {0, 255, 255},
    {255, 0, 255},
}};

struct GridOptions {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    /// UV iso-lines per axis (10: one every 0.1 of the rendered FOV).
    std::uint32_t divisions = 10;
    /// Ring angles in degrees, drawn with RING_COLORS in order.
    std::vector<float> rings_deg;

    /// Panel picture of `3840/div x 3552/div` pixels with a UV line every 0.1 and rings at 10..50 degrees.
    static GridOptions for_panel(std::uint32_t div);
};

/// Render the grid picture of `view` (RGB8, `width * height * 3` bytes).
std::vector<std::uint8_t> render(const Lens& lens, std::size_t view, const GridOptions& opt);
/// Human readable summary of what the model does to one eye.
std::string summary(const Lens& lens, std::size_t view);

namespace detail {
float iso_cover(float value, float grad, float step, float halfw);
}

} // namespace pibridge::lens::grid
