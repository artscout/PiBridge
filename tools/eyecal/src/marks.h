// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/gpu.rs (the pure part: `block`, `shade`) and the matrix helpers of
// pimax-lenstest/src/geometry.rs (MIT OR Apache-2.0); the check grid from omarchyvr analysis/tools/xr_grid.c
// `draw_grid` (MIT OR Apache-2.0).
//
// The scene on the GPU is one full-screen fragment pass per eye (shaders/marks.frag) drawn straight into the swapchain
// image. Per frame only a 1.6 KB uniform block changes (the marks as directions in that eye's view space); nothing is
// rendered or uploaded by the CPU, so the frame time stays a fraction of a millisecond at 3552 x 3840. block() builds
// the uniform block; shade() is the CPU twin of the shader (tests).
//
// The check grid (Scene::grid) is xr_grid's head-locked wall, ray-cast per pixel in the same pass: a square grid every
// 0.2 m at 2 m (bright every 5th line), a red centre cross, circles at 10/20/30/40/50 degrees from the view axis
// (white, yellow, green, cyan, magenta) and an orange 4-pixel FOV border, all in this eye's own view space. The gaze
// dot (Scene::gaze) is drawn last: a filled violet disc with a thin dark outline.
#pragma once

#include "procedure.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace pibridge::eyecal {

/// Row-major 3x3 matrix (the Rust `[[f32; 3]; 3]`).
using Mat3 = std::array<std::array<float, 3>, 3>;
inline constexpr Mat3 IDENT3 = {{{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}}};

std::array<float, 3> mat_vec(const Mat3& m, const std::array<float, 3>& v);
Mat3 transpose(const Mat3& m);
/// Rotation matrix of the quaternion (x, y, z, w); normalised first, identity for a zero quaternion.
Mat3 quat_to_mat(const std::array<float, 4>& q);

/// Marks per eye (the shader's `MAX_MARKS`).
inline constexpr std::size_t MAX_MARKS = 32;
/// Offsets (floats) of the std140 uniform block `Scene` of marks.frag, in its member order: `tan_`, `info`, `bg`,
/// `grid` (on, grid step in tangent units, border pixels, -), `gaze` (unit direction, radius in rad; radius 0 = no
/// dot), `gaze_col` (sRGB, outline width in rad), then `marks[MAX_MARKS]` of 3 vec4 each.
inline constexpr std::size_t OFF_TAN = 0;
inline constexpr std::size_t OFF_INFO = 4;
inline constexpr std::size_t OFF_BG = 8;
inline constexpr std::size_t OFF_GRID = 12;
inline constexpr std::size_t OFF_GAZE = 16;
inline constexpr std::size_t OFF_GAZE_COL = 20;
inline constexpr std::size_t OFF_MARKS = 24;
/// Floats per mark (dir_r, color_kind, extra).
inline constexpr std::size_t MARK_FLOATS = 12;
/// Floats in the uniform block: 6 vec4 header + 3 vec4 per mark (std140).
inline constexpr std::size_t BLOCK_FLOATS = OFF_MARKS + MAX_MARKS * MARK_FLOATS;
using Block = std::array<float, BLOCK_FLOATS>;

/// The check grid (xr_grid.c): the wall distance, the line spacing on it, the FOV border.
inline constexpr double GRID_DIST_M = 2.0;
inline constexpr double GRID_STEP_M = 0.2;
inline constexpr float GRID_BORDER_PX = 4.0F;
/// The circles, degrees from the view axis, and their colours (sRGB): white, yellow, green, cyan, magenta.
inline constexpr std::array<float, 5> GRID_RING_DEG = {10.0F, 20.0F, 30.0F, 40.0F, 50.0F};
inline constexpr std::array<std::array<float, 3>, 5> GRID_RING_RGB = {{{1.0F, 1.0F, 1.0F},
                                                                       {1.0F, 1.0F, 0.0F},
                                                                       {0.0F, 1.0F, 0.0F},
                                                                       {0.0F, 1.0F, 1.0F},
                                                                       {1.0F, 0.0F, 1.0F}}};
/// Grid lines (0x90 grey, 0xE0 every 5th), the centre cross (red), the FOV border (orange).
inline constexpr std::array<float, 3> GRID_LINE_RGB = {144.0F / 255.0F, 144.0F / 255.0F, 144.0F / 255.0F};
inline constexpr std::array<float, 3> GRID_MAJOR_RGB = {224.0F / 255.0F, 224.0F / 255.0F, 224.0F / 255.0F};
inline constexpr std::array<float, 3> GRID_CROSS_RGB = {1.0F, 0.0F, 0.0F};
inline constexpr std::array<float, 3> GRID_BORDER_RGB = {1.0F, 128.0F / 255.0F, 0.0F};
/// The gaze dot: radius and outline width (degrees), fill colour (violet-magenta, sRGB), outline colour.
inline constexpr double GAZE_DOT_DEG = 0.7;
inline constexpr double GAZE_OUTLINE_DEG = 0.15;
inline constexpr std::array<float, 3> GAZE_RGB = {0.85F, 0.2F, 1.0F};
inline constexpr std::array<float, 3> GAZE_OUTLINE_RGB = {0.02F, 0.02F, 0.03F};

/// One eye's view: rotation eye -> head (OpenXR view orientation), position in the head frame, FOV tangents (left,
/// right, up, down; positive) and image size.
struct EyeView {
    Mat3 rot = IDENT3;
    std::array<float, 3> pos{0.0F, 0.0F, 0.0F};
    std::array<float, 4> tan{1.0F, 1.0F, 1.0F, 1.0F};
    std::array<std::uint32_t, 2> size{16, 16};
};

/// The uniform block of one eye.
Block block(const Scene& scene, const EyeView& v);

/// CPU twin of `marks.frag`: the sRGB colour of pixel (x, y).
std::array<float, 3> shade(const Block& b, std::uint32_t x, std::uint32_t y);

} // namespace pibridge::eyecal
