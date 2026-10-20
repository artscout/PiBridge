// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/grid.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/lens/grid.h"
#include "pibridge_test.h"

#include <cstdlib>
#include <set>
#include <tuple>

using namespace pibridge::lens;
using pibridge::lens::testing::ASPECT;

namespace {

grid::GridOptions small() {
    grid::GridOptions o;
    o.width = 192;
    o.height = 178;
    o.divisions = 10;
    o.rings_deg = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f};
    return o;
}

bool contains(const std::string& s, const char* x) {
    return s.find(x) != std::string::npos;
}

} // namespace

TEST(render_has_the_right_size_and_content) {
    const Lens l(LensConfig::defaults(ASPECT));
    const auto img = grid::render(l, 0, small());
    CHECK(img.size() == 192 * 178 * 3);
    std::set<std::tuple<int, int, int>> distinct;
    for (std::size_t i = 0; i < img.size(); i += 3) {
        distinct.insert({img[i], img[i + 1], img[i + 2]});
    }
    CHECK(distinct.size() > 20);
    // deterministic
    CHECK(img == grid::render(l, 0, small()));
    // left and right eye differ (optical centre is mirrored)
    CHECK(img != grid::render(l, 1, small()));
}

TEST(ring_colours_and_axis_colour_appear) {
    const Lens l(LensConfig::defaults(ASPECT));
    auto opt = small();
    opt.width = 480;
    opt.height = 444;
    const auto img = grid::render(l, 0, opt);
    const auto any = [&](auto pred) {
        for (std::size_t i = 0; i < img.size(); i += 3) {
            if (pred(img[i], img[i + 1], img[i + 2])) {
                return true;
            }
        }
        return false;
    };
    for (const auto& c : grid::RING_COLORS) {
        const bool exact = any([&](int r, int g, int b) { return r == c[0] && g == c[1] && b == c[2]; });
        const bool near = any([&](int r, int g, int b) {
            return std::abs(r - c[0]) < 40 && std::abs(g - c[1]) < 40 && std::abs(b - c[2]) < 40;
        });
        CHECK(exact || near);
    }
    CHECK(any([](int r, int g, int b) { return r > 200 && g > 90 && g < 170 && b < 60; }));
}

TEST(mask_darkens_and_outlines) {
    auto [cfg, w] = parse("mask = ellipse 0.9 0.8\n", ASPECT);
    CHECK(w.empty());
    const Lens l(cfg);
    auto opt = small();
    opt.width = 240;
    opt.height = 222;
    const auto img = grid::render(l, 0, opt);
    // the panel corner is hidden: very dark
    const std::size_t o = (8 * 240 + 8) * 3;
    CHECK(img[o] < 12 && img[o + 1] < 12 && img[o + 2] < 12);
    bool outline = false;
    for (std::size_t i = 0; i < img.size(); i += 3) {
        const int r = img[i];
        const int g = img[i + 1];
        const int b = img[i + 2];
        outline = outline || (r == 255 && g == 90 && b == 160) || (r > 200 && g < 140 && b > 100);
    }
    CHECK(outline);
    const auto plain = grid::render(Lens(LensConfig::defaults(ASPECT)), 0, opt);
    CHECK(img != plain);
}

TEST(summary_reports_rings_corners_and_monotonicity) {
    const Lens l(LensConfig::defaults(ASPECT));
    const std::string s = grid::summary(l, 0);
    CHECK(contains(s, "eye 0 (left): distortion on"));
    CHECK(contains(s, "30 deg: r = 0.7"));
    CHECK(contains(s, "corner TL"));
    CHECK(contains(s, "OUTSIDE FOV"));
    CHECK(contains(s, "monotonic out to r = 1.6"));
    const Lens bad(parse("k = 0 -3 0\n", ASPECT).first);
    CHECK(contains(grid::summary(bad, 0), "WARNING: radial curve folds back"));
    const Lens masked(parse("blackout = 0.1\nfit_range = 1.5\n", ASPECT).first);
    const std::string ms = grid::summary(masked, 1);
    CHECK(contains(ms, "visibility mask") && contains(ms, "fit_range 1.500"));
}

TEST(iso_cover_is_one_on_the_line_and_zero_far_away) {
    CHECK(grid::detail::iso_cover(0.5f, 0.01f, 0.1f, 0.5f) == 1.0f);
    CHECK(grid::detail::iso_cover(0.55f, 0.01f, 0.1f, 0.5f) == 0.0f);
    const float half = grid::detail::iso_cover(0.5f + 0.008f, 0.01f, 0.1f, 0.5f);
    CHECK(half > 0.0f && half < 1.0f);
}

PIBRIDGE_TEST_MAIN()
