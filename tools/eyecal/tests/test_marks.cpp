// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/gpu.rs tests (MIT OR Apache-2.0). New: the uniform block against the
// shader's std140 layout, the check grid (xr_grid.c) and the gaze dot.
#include "marks.h"

#include "layout.h"

#include "pibridge_test.h"

#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

using namespace pibridge::eyecal;

namespace {

EyeView view(std::size_t eye, float cant_deg) {
    const float c = (eye == 0 ? cant_deg : -cant_deg) * std::numbers::pi_v<float> / 180.0F * 0.5F;
    EyeView v;
    v.rot = cant_deg == 0.0F ? IDENT3 : quat_to_mat({0.0F, std::sin(c), 0.0F, std::cos(c)});
    v.pos = {eye == 0 ? -0.032F : 0.032F, 0.0F, 0.0F};
    v.tan = {1.0F, 1.0F, 0.9F, 0.9F};
    v.size = {400, 360};
    return v;
}

Scene scene(std::vector<Mark> marks) {
    Scene s;
    s.marks = std::move(marks);
    s.background = BACKGROUND;
    return s;
}

std::array<float, 3> centroid(const Block& b, std::uint32_t w, std::uint32_t h) {
    float sx = 0.0F;
    float sy = 0.0F;
    float sw = 0.0F;
    for (std::uint32_t y = 0; y < h; y++) {
        for (std::uint32_t x = 0; x < w; x++) {
            const auto c = shade(b, x, y);
            const float v = std::max(c[0] - BACKGROUND[0], 0.0F);
            sx += v * (static_cast<float>(x) + 0.5F);
            sy += v * (static_cast<float>(y) + 0.5F);
            sw += v;
        }
    }
    return {sx / sw, sy / sw, sw};
}

} // namespace

TEST(a_disc_lands_where_each_eye_must_see_it_also_with_canted_views) {
    const Vec3 p = Target::at(12.0, -9.0).point();
    const Mark m{p, MarkKind::Disc, 1.0, 0.0, {1.0F, 1.0F, 1.0F}};
    for (const float cant : {0.0F, 2.0F}) {
        for (std::size_t eye = 0; eye < 2; eye++) {
            const auto v = view(eye, cant);
            const auto b = block(scene({m}), v);
            const auto [cx, cy, _] = centroid(b, 400, 360);
            // expected: the target in this eye's view space, projected
            const auto inv = transpose(v.rot);
            const std::array<float, 3> rel = {static_cast<float>(p[0] - v.pos[0]), static_cast<float>(p[1]),
                                              static_cast<float>(p[2])};
            const auto d = mat_vec(inv, rel);
            const float tx = d[0] / -d[2];
            const float ty = d[1] / -d[2];
            const float ex = (tx + 1.0F) / 2.0F * 400.0F;
            const float ey = (0.9F - ty) / 1.8F * 360.0F;
            CHECK(std::fabs(cx - ex) < 0.3F && std::fabs(cy - ey) < 0.3F);
        }
    }
}

TEST(abc_target_has_a_cut_cross_and_a_centre_dot) {
    const Vec3 p = {0.0, 0.0, -2.0};
    const std::vector<Mark> marks = {
        Mark{p, MarkKind::Disc, 8.0, 0.0, {1.0F, 1.0F, 1.0F}},
        Mark{p, MarkKind::Cross, 8.5, 1.5, BACKGROUND},
        Mark{p, MarkKind::Disc, 1.0, 0.0, {1.0F, 0.0F, 0.0F}},
    };
    EyeView v = view(0, 0.0F);
    v.pos = {0.0F, 0.0F, 0.0F};
    const auto b = block(scene(marks), v);
    const std::uint32_t cx = 200;
    const std::uint32_t cy = 180;
    CHECK((shade(b, cx, cy) == std::array<float, 3>{1.0F, 0.0F, 0.0F})); // red centre
    // on the horizontal bar, 4 degrees out: background; between the bars: white
    const float px_per_deg = 400.0F / 2.0F * std::numbers::pi_v<float> / 180.0F;
    const auto bar = shade(b, cx + static_cast<std::uint32_t>(4.0F * px_per_deg), cy);
    CHECK(std::fabs(bar[0] - BACKGROUND[0]) < 0.02F);
    const auto off = static_cast<std::uint32_t>(4.0F * px_per_deg);
    const auto quad = shade(b, cx + off, cy - off);
    CHECK(quad[1] > 0.95F);
    // outside the disc: background
    const auto out = shade(b, cx + static_cast<std::uint32_t>(12.0F * px_per_deg), cy);
    CHECK(out == BACKGROUND);
}

TEST(rings_are_hollow_and_marks_behind_are_dropped) {
    const Mark ring{{0.0, 0.0, -2.0}, MarkKind::Ring, 5.0, 0.5, {0.0F, 1.0F, 0.0F}};
    Mark behind = ring;
    behind.pos = {0.0, 0.0, 2.0};
    EyeView v = view(0, 0.0F);
    v.pos = {0.0F, 0.0F, 0.0F};
    const auto b = block(scene({ring, behind}), v);
    CHECK(b[7] == 1.0F); // only the ring is in the block
    CHECK(shade(b, 200, 180) == BACKGROUND);
    const float px_per_deg = 400.0F / 2.0F * std::numbers::pi_v<float> / 180.0F;
    const auto on = shade(b, 200 + static_cast<std::uint32_t>(5.0F * px_per_deg), 180);
    CHECK(on[1] > 0.5F);
    const std::vector<Mark> many(40, ring);
    CHECK(static_cast<std::size_t>(block(scene(many), v)[7]) == MAX_MARKS);
}

TEST(matrix_helpers) {
    const auto m = quat_to_mat({0.0F, std::sin(0.25F), 0.0F, std::cos(0.25F)});
    const auto t = transpose(m);
    // m * m^T = I
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            float s = 0.0F;
            for (std::size_t k = 0; k < 3; k++) {
                s += m[i][k] * t[k][j];
            }
            CHECK(std::fabs(s - IDENT3[i][j]) < 1e-6F);
        }
    }
    CHECK((quat_to_mat({0.0F, 0.0F, 0.0F, 0.0F}) == IDENT3));
    const auto f = mat_vec(m, {0.0F, 0.0F, -1.0F});
    CHECK(f[0] < 0.0F); // a positive yaw turns -z to the left
}

namespace {

bool near3(const std::array<float, 3>& a, const std::array<float, 3>& b, float tol = 0.02F) {
    return std::fabs(a[0] - b[0]) < tol && std::fabs(a[1] - b[1]) < tol && std::fabs(a[2] - b[2]) < tol;
}

/// The pixel of view tangent (tx, ty) in view(0, 0) at `w` x `h` (tan 1, 1, 0.9, 0.9).
std::pair<std::uint32_t, std::uint32_t> pixel_of(float tx, float ty, std::uint32_t w, std::uint32_t h) {
    const float x = (tx + 1.0F) / 2.0F * static_cast<float>(w) - 0.5F;
    const float y = (0.9F - ty) / 1.8F * static_cast<float>(h) - 0.5F;
    return {static_cast<std::uint32_t>(std::lround(x)), static_cast<std::uint32_t>(std::lround(y))};
}

} // namespace

TEST(the_uniform_block_matches_the_shader_std140_layout) {
    std::ifstream f(std::string(PIBRIDGE_EYECAL_SHADER_DIR) + "/marks.frag");
    CHECK(f.good());
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string src = ss.str();
    const auto open = src.find("uniform Scene {");
    const auto close = src.find("} u;", open);
    CHECK(open != std::string::npos && close != std::string::npos);
    if (open == std::string::npos || close == std::string::npos) {
        return;
    }
    // the members in order: every header member is one vec4 (16 bytes, std140), then the marks array
    std::vector<std::string> names;
    std::istringstream body(src.substr(open, close - open));
    std::string ln;
    std::getline(body, ln);
    while (std::getline(body, ln)) {
        std::istringstream w(ln);
        std::string type;
        std::string name;
        w >> type >> name;
        if (type.empty()) {
            continue;
        }
        CHECK(type == "vec4" || type == "Mark");
        names.push_back(name.substr(0, name.find_first_of("[;")));
    }
    CHECK((names == std::vector<std::string>{"tan_", "info", "bg", "grid", "gaze", "gaze_col", "marks"}));
    CHECK(OFF_TAN == 0 && OFF_INFO == 4 && OFF_BG == 8 && OFF_GRID == 12 && OFF_GAZE == 16 && OFF_GAZE_COL == 20);
    CHECK(OFF_MARKS == 4 * (names.size() - 1));
    // a Mark is three vec4, the array has MAX_MARKS of them
    CHECK(src.find("const int MAX_MARKS = " + std::to_string(MAX_MARKS) + ";") != std::string::npos);
    CHECK(MARK_FLOATS == 12 && BLOCK_FLOATS == OFF_MARKS + 12 * MAX_MARKS);
    CHECK(sizeof(Block) == (6 + 3 * MAX_MARKS) * 16);
    // the grid's constants are the twin's
    CHECK(src.find("float[5](10.0, 20.0, 30.0, 40.0, 50.0)") != std::string::npos);
    CHECK(src.find("vec3(0.02, 0.02, 0.03)") != std::string::npos); // the gaze dot's outline
}

TEST(the_check_grid_has_xr_grids_rings_cross_lines_and_border) {
    const std::uint32_t w = 400;
    const std::uint32_t h = 360;
    EyeView v = view(0, 0.0F);
    v.pos = {0.0F, 0.0F, 0.0F};
    Scene s = scene({});
    s.grid = true;
    const auto b = block(s, v);
    CHECK(b[OFF_GRID] == 1.0F && std::fabs(b[OFF_GRID + 1] - 0.1F) < 1e-7F && b[OFF_GRID + 2] == 4.0F);
    // the circles at 10..50 degrees, on the diagonal: white, yellow, green, cyan, magenta
    for (std::size_t i = 0; i < GRID_RING_DEG.size(); i++) {
        const float r = std::tan(GRID_RING_DEG[i] * std::numbers::pi_v<float> / 180.0F) / std::sqrt(2.0F);
        const auto [x, y] = pixel_of(r, r, w, h);
        CHECK(near3(shade(b, x, y), GRID_RING_RGB[i]));
    }
    // the red centre cross, over the whole view
    CHECK(near3(shade(b, w / 2, h / 2), GRID_CROSS_RGB));
    {
        const auto [x, y] = pixel_of(0.73F, 0.0F, w, h);
        CHECK(near3(shade(b, x, y), GRID_CROSS_RGB));
    }
    // a grid line every 0.2 m at 2 m (0.1 in tangent units), every 5th brighter; the background between
    {
        const auto [x, y] = pixel_of(0.1F, 0.05F, w, h);
        CHECK(near3(shade(b, x, y), GRID_LINE_RGB));
    }
    {
        const auto [x, y] = pixel_of(-0.5F, 0.35F, w, h);
        CHECK(near3(shade(b, x, y), GRID_MAJOR_RGB));
    }
    {
        const auto [x, y] = pixel_of(0.05F, 0.05F, w, h);
        CHECK(near3(shade(b, x, y), BACKGROUND));
    }
    // the orange FOV border, 4 pixels
    CHECK(shade(b, 0, 0) == GRID_BORDER_RGB && shade(b, 3, 100) == GRID_BORDER_RGB && shade(b, w - 1, h - 4) == GRID_BORDER_RGB);
    CHECK(shade(b, 4, 101) != GRID_BORDER_RGB);
    // without the flag: nothing of it
    const auto plain = block(scene({}), v);
    CHECK(plain[OFF_GRID] == 0.0F);
    CHECK(shade(plain, w / 2, h / 2) == BACKGROUND && shade(plain, 0, 0) == BACKGROUND);
    // the grid is in each eye's own view space: a canted eye sees its centre cross at its own image centre
    EyeView c = view(0, 6.0F);
    const auto bc = block(s, c);
    CHECK(near3(shade(bc, w / 2, h / 2), GRID_CROSS_RGB));
}

TEST(the_gaze_dot_is_filled_with_a_dark_outline_over_everything) {
    const std::uint32_t w = 4000;
    const std::uint32_t h = 3600;
    EyeView v = view(1, 0.0F);
    v.size = {w, h};
    const Vec3 p = Target::at(5.0, 3.0).point();
    Scene s = scene({Mark{p, MarkKind::Disc, 2.0, 0.0, {1.0F, 1.0F, 1.0F}}}); // a white mark under it
    s.grid = true;
    s.gaze = p;
    const auto b = block(s, v);
    CHECK(std::fabs(b[OFF_GAZE + 3] - static_cast<float>(GAZE_DOT_DEG * std::numbers::pi / 180.0)) < 1e-7F);
    // where this eye sees the point (as the marks)
    const std::array<float, 3> rel = {static_cast<float>(p[0] - v.pos[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
    const float tx = rel[0] / -rel[2];
    const float ty = rel[1] / -rel[2];
    const float ex = (tx + 1.0F) / 2.0F * static_cast<float>(w);
    const float ey = (0.9F - ty) / 1.8F * static_cast<float>(h);
    // the centroid of the violet fill
    float sx = 0.0F;
    float sy = 0.0F;
    float sw = 0.0F;
    const auto x0 = static_cast<std::uint32_t>(ex) - 40;
    const auto y0 = static_cast<std::uint32_t>(ey) - 40;
    for (std::uint32_t y = y0; y < y0 + 80; y++) {
        for (std::uint32_t x = x0; x < x0 + 80; x++) {
            const auto c = shade(b, x, y);
            const float k = std::max(0.0F, 1.0F - c[1] / 1.0F) * (c[0] > 0.5F ? 1.0F : 0.0F); // violet: red high, green low
            sx += k * (static_cast<float>(x) + 0.5F);
            sy += k * (static_cast<float>(y) + 0.5F);
            sw += k;
        }
    }
    CHECK(sw > 0.0F);
    CHECK(std::fabs(sx / sw - ex) < 0.5F && std::fabs(sy / sw - ey) < 0.5F);
    // filled (the centre is the dot's colour, over the white mark), a dark outline, then the white mark again
    const auto cx = static_cast<std::uint32_t>(ex);
    const auto cy = static_cast<std::uint32_t>(ey);
    CHECK(near3(shade(b, cx, cy), GAZE_RGB));
    const float px_per_deg = static_cast<float>(w) / 2.0F * std::numbers::pi_v<float> / 180.0F;
    const auto at = [&](double deg) { return cx + static_cast<std::uint32_t>(std::lround(deg * px_per_deg)); };
    CHECK(near3(shade(b, at(0.5 * GAZE_DOT_DEG), cy), GAZE_RGB));
    CHECK(near3(shade(b, at(GAZE_DOT_DEG + 0.5 * GAZE_OUTLINE_DEG), cy), GAZE_OUTLINE_RGB, 0.05F));
    CHECK(near3(shade(b, at(GAZE_DOT_DEG + GAZE_OUTLINE_DEG + 0.3), cy), {1.0F, 1.0F, 1.0F}));
    // no gaze: no dot; a gaze behind the eye is dropped like a mark
    Scene n = s;
    n.gaze.reset();
    CHECK(block(n, v)[OFF_GAZE + 3] == 0.0F);
    CHECK(near3(shade(block(n, v), cx, cy), {1.0F, 1.0F, 1.0F}));
    n.gaze = Vec3{0.0, 0.0, 2.0};
    CHECK(block(n, v)[OFF_GAZE + 3] == 0.0F);
}

PIBRIDGE_TEST_MAIN()
