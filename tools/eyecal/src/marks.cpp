// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/gpu.rs and pimax-lenstest/src/geometry.rs (MIT OR Apache-2.0).
// The check grid: ported from omarchyvr analysis/tools/xr_grid.c `draw_grid` (MIT OR Apache-2.0).
#include "marks.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pibridge::eyecal {

namespace {

using V3 = std::array<float, 3>;

float cover(float d, float pix) { return std::clamp(d / pix + 0.5F, 0.0F, 1.0F); }

V3 cross(const V3& a, const V3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

float dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

float rad(double d) { return static_cast<float>(d * std::numbers::pi / 180.0); }

/// xr_grid's coverage of a line of half-width `w` at distance `d` (all in the same units as `pix`).
float line(float d, float w, float pix) { return std::clamp((w - std::fabs(d)) / pix + 0.5F, 0.0F, 1.0F); }

void mix_in(V3& c, const V3& col, float a) {
    for (std::size_t k = 0; k < 3; k++) {
        c[k] += (col[k] - c[k]) * a;
    }
}

/// The check grid at view tangent (tx, ty) over `c` (xr_grid.c draw_grid, per pixel); `tpp` is the tangent per pixel,
/// `step` the grid spacing in tangent units.
void grid(V3& c, float tx, float ty, float tpp, float step) {
    const float gx = tx / step;
    const float gy = ty / step;
    const float fx = gx - std::floor(gx + 0.5F);
    const float fy = gy - std::floor(gy + 0.5F);
    const int ix = static_cast<int>(std::floor(gx + 0.5F));
    const int iy = static_cast<int>(std::floor(gy + 0.5F));
    const bool mx = std::abs(ix) % 5 == 0;
    const bool my = std::abs(iy) % 5 == 0;
    const float ps = tpp / step; // one pixel in grid units
    const float ax = line(fx, ps * (mx ? 2.0F : 1.0F), ps);
    const float ay = line(fy, ps * (my ? 2.0F : 1.0F), ps);
    mix_in(c, GRID_LINE_RGB, ax);
    mix_in(c, GRID_LINE_RGB, ay);
    if (mx) {
        mix_in(c, GRID_MAJOR_RGB, ax);
    }
    if (my) {
        mix_in(c, GRID_MAJOR_RGB, ay);
    }
    const float r = std::sqrt(tx * tx + ty * ty);
    for (std::size_t i = 0; i < GRID_RING_DEG.size(); i++) {
        mix_in(c, GRID_RING_RGB[i], line(r - std::tan(rad(GRID_RING_DEG[i])), 2.0F * tpp, tpp));
    }
    mix_in(c, GRID_CROSS_RGB, line(tx, 3.0F * tpp, tpp));
    mix_in(c, GRID_CROSS_RGB, line(ty, 3.0F * tpp, tpp));
}

} // namespace

std::array<float, 3> mat_vec(const Mat3& m, const std::array<float, 3>& v) {
    return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}

Mat3 transpose(const Mat3& m) {
    Mat3 o{};
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            o[i][j] = m[j][i];
        }
    }
    return o;
}

Mat3 quat_to_mat(const std::array<float, 4>& q0) {
    const float n = std::sqrt(q0[0] * q0[0] + q0[1] * q0[1] + q0[2] * q0[2] + q0[3] * q0[3]);
    std::array<float, 4> q = {0.0F, 0.0F, 0.0F, 1.0F};
    if (n > 0.0F) {
        q = {q0[0] / n, q0[1] / n, q0[2] / n, q0[3] / n};
    }
    const auto [x, y, z, w] = q;
    return {{{1.0F - 2.0F * (y * y + z * z), 2.0F * (x * y - z * w), 2.0F * (x * z + y * w)},
             {2.0F * (x * y + z * w), 1.0F - 2.0F * (x * x + z * z), 2.0F * (y * z - x * w)},
             {2.0F * (x * z - y * w), 2.0F * (y * z + x * w), 1.0F - 2.0F * (x * x + y * y)}}};
}

Block block(const Scene& scene, const EyeView& v) {
    Block b{};
    std::copy(v.tan.begin(), v.tan.end(), b.begin());
    const float tpp = (v.tan[0] + v.tan[1]) / static_cast<float>(v.size[0]);
    const Mat3 inv = transpose(v.rot);
    std::size_t n = 0;
    for (const auto& m : scene.marks) {
        if (n >= MAX_MARKS) {
            break;
        }
        const V3 rel = {static_cast<float>(m.pos[0] - static_cast<double>(v.pos[0])),
                        static_cast<float>(m.pos[1] - static_cast<double>(v.pos[1])),
                        static_cast<float>(m.pos[2] - static_cast<double>(v.pos[2]))};
        const V3 d = mat_vec(inv, rel);
        const float l = std::sqrt(dot(d, d));
        if (std::isnan(l) || l <= 1e-6F || d[2] > -1e-3F * l) {
            continue; // behind the eye
        }
        const std::size_t o = OFF_MARKS + n * MARK_FLOATS;
        b[o] = d[0] / l;
        b[o + 1] = d[1] / l;
        b[o + 2] = d[2] / l;
        b[o + 3] = rad(m.radius_deg);
        b[o + 4] = m.color[0];
        b[o + 5] = m.color[1];
        b[o + 6] = m.color[2];
        b[o + 7] = m.kind == MarkKind::Disc ? 0.0F : m.kind == MarkKind::Ring ? 1.0F : 2.0F;
        b[o + 8] = rad(m.width_deg);
        n++;
    }
    b[OFF_INFO] = static_cast<float>(v.size[0]);
    b[OFF_INFO + 1] = static_cast<float>(v.size[1]);
    b[OFF_INFO + 2] = tpp;
    b[OFF_INFO + 3] = static_cast<float>(n);
    b[OFF_BG] = scene.background[0];
    b[OFF_BG + 1] = scene.background[1];
    b[OFF_BG + 2] = scene.background[2];
    if (scene.grid) {
        b[OFF_GRID] = 1.0F;
        b[OFF_GRID + 1] = static_cast<float>(GRID_STEP_M / GRID_DIST_M);
        b[OFF_GRID + 2] = GRID_BORDER_PX;
    }
    // the gaze dot: its direction in this eye's view space, as the marks
    if (scene.gaze) {
        const auto& g = *scene.gaze;
        const V3 rel = {static_cast<float>(g[0] - static_cast<double>(v.pos[0])),
                        static_cast<float>(g[1] - static_cast<double>(v.pos[1])),
                        static_cast<float>(g[2] - static_cast<double>(v.pos[2]))};
        const V3 d = mat_vec(inv, rel);
        const float l = std::sqrt(dot(d, d));
        if (!std::isnan(l) && l > 1e-6F && d[2] <= -1e-3F * l) {
            b[OFF_GAZE] = d[0] / l;
            b[OFF_GAZE + 1] = d[1] / l;
            b[OFF_GAZE + 2] = d[2] / l;
            b[OFF_GAZE + 3] = rad(GAZE_DOT_DEG);
        }
    }
    b[OFF_GAZE_COL] = GAZE_RGB[0];
    b[OFF_GAZE_COL + 1] = GAZE_RGB[1];
    b[OFF_GAZE_COL + 2] = GAZE_RGB[2];
    b[OFF_GAZE_COL + 3] = rad(GAZE_OUTLINE_DEG);
    return b;
}

std::array<float, 3> shade(const Block& b, std::uint32_t x, std::uint32_t y) {
    const float w = b[OFF_INFO];
    const float h = b[OFF_INFO + 1];
    const float tl = b[OFF_TAN];
    const float tr = b[OFF_TAN + 1];
    const float tu = b[OFF_TAN + 2];
    const float td = b[OFF_TAN + 3];
    const float tx = -tl + (static_cast<float>(x) + 0.5F) / w * (tl + tr);
    const float ty = tu - (static_cast<float>(y) + 0.5F) / h * (tu + td);
    const float r2 = tx * tx + ty * ty;
    const float n = std::sqrt(r2 + 1.0F);
    const V3 v = {tx / n, ty / n, -1.0F / n};
    const float pix = b[OFF_INFO + 2] / (1.0F + r2);
    std::array<float, 3> code = {b[OFF_BG], b[OFF_BG + 1], b[OFF_BG + 2]};
    if (b[OFF_GRID] > 0.5F) {
        grid(code, tx, ty, b[OFF_INFO + 2], b[OFF_GRID + 1]);
        const float e = b[OFF_GRID + 2];
        const float fx = static_cast<float>(x) + 0.5F;
        const float fy = static_cast<float>(y) + 0.5F;
        if (fx < e || fy < e || fx >= w - e || fy >= h - e) {
            code = GRID_BORDER_RGB;
        }
    }
    const auto count = static_cast<std::size_t>(b[OFF_INFO + 3] + 0.5F);
    for (std::size_t i = 0; i < std::min(count, MAX_MARKS); i++) {
        const std::size_t o = OFF_MARKS + i * MARK_FLOATS;
        const V3 d = {b[o], b[o + 1], b[o + 2]};
        const float r = b[o + 3];
        const auto kind = static_cast<int>(b[o + 7] + 0.5F);
        const float wd = b[o + 8];
        const float c = dot(d, v);
        if (c < std::cos(std::min(r + wd + 4.0F * pix, 3.0F))) {
            continue;
        }
        const V3 cr = cross(d, v);
        const float ang = std::atan2(std::sqrt(dot(cr, cr)), c);
        float cov = 0.0F;
        if (kind == 0) {
            cov = cover(r - ang, pix);
        } else if (kind == 1) {
            cov = cover(0.5F * wd - std::fabs(ang - r), pix);
        } else {
            V3 e1 = cross(d, {0.0F, 1.0F, 0.0F});
            const float l = std::sqrt(dot(e1, e1));
            e1 = {e1[0] / l, e1[1] / l, e1[2] / l};
            const V3 e2 = cross(e1, d);
            const float lx = dot(v, e1) / c;
            const float ly = dot(v, e2) / c;
            const float b1 = std::min(cover(0.5F * wd - std::fabs(ly), pix), cover(r - std::fabs(lx), pix));
            const float b2 = std::min(cover(0.5F * wd - std::fabs(lx), pix), cover(r - std::fabs(ly), pix));
            cov = std::max(b1, b2);
        }
        for (std::size_t k = 0; k < 3; k++) {
            code[k] += (b[o + 4 + k] - code[k]) * cov;
        }
    }
    // the gaze dot on top: the dark outline (a disc radius + outline), then the fill
    if (const float r = b[OFF_GAZE + 3]; r > 0.0F) {
        const V3 d = {b[OFF_GAZE], b[OFF_GAZE + 1], b[OFF_GAZE + 2]};
        const float ow = b[OFF_GAZE_COL + 3];
        const float c = dot(d, v);
        if (c >= std::cos(std::min(r + ow + 4.0F * pix, 3.0F))) {
            const V3 cr = cross(d, v);
            const float ang = std::atan2(std::sqrt(dot(cr, cr)), c);
            mix_in(code, GAZE_OUTLINE_RGB, cover(r + ow - ang, pix));
            mix_in(code, {b[OFF_GAZE_COL], b[OFF_GAZE_COL + 1], b[OFF_GAZE_COL + 2]}, cover(r - ang, pix));
        }
    }
    return code;
}

} // namespace pibridge::eyecal
