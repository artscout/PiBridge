// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/mask.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/lens/mask.h"
#include "pibridge_test.h"

#include <cmath>
#include <limits>

using namespace pibridge::lens;
using pibridge::lens::testing::ASPECT;

namespace {

Lens lens(const char* text) {
    auto [cfg, w] = parse(text, ASPECT);
    CHECK(w.empty());
    return Lens(std::move(cfg));
}

} // namespace

TEST(no_mask_means_no_custom_mesh) {
    const Lens l(LensConfig::defaults(ASPECT));
    for (const auto k : {MaskKind::HiddenTriangleMesh, MaskKind::VisibleTriangleMesh, MaskKind::LineLoop}) {
        CHECK(!visibility_mesh(l, 0, k, 32));
    }
    CHECK(mask_kind_from_i32(2) == MaskKind::LineLoop);
    CHECK(!mask_kind_from_i32(3));
}

TEST(outline_of_an_ellipse_stays_on_the_ellipse_or_the_fov) {
    const Lens l = lens("center = 0 0\nmask = ellipse 0.8 0.6\nmask_feather = 0\n");
    const auto poly = *outline_tangent(l, 0, 64);
    CHECK(poly.size() == 64);
    const auto [tl, tr, tu, td] = l.fov_tan(0);
    for (const auto& p : poly) {
        CHECK(p[0] >= -tl - 1e-4f && p[0] <= tr + 1e-4f && p[1] >= -td - 1e-4f && p[1] <= tu + 1e-4f);
    }
    // the +x ray: boundary at panel radius 0.8 -> tangent from the model
    const float want = l.tangent(0, 1, 0.5f + 0.8f / (2.0f * ASPECT), 0.5f)[0];
    CHECK(std::fabs(poly[0][0] - want) < 2e-3f);
    // a quarter turn later: +y at panel radius 0.6
    const float want_y = l.tangent(0, 1, 0.5f, 0.5f - 0.6f / 2.0f)[1];
    CHECK(std::fabs(poly[16][1] - want_y) < 2e-3f);
}

TEST(visible_plus_hidden_cover_the_fov_rectangle) {
    const Lens l = lens("mask = ellipse 1.0 0.9\nblackout = 0.02\n");
    const std::size_t n = 96;
    const auto vis = *visibility_mesh(l, 1, MaskKind::VisibleTriangleMesh, n);
    const auto hid = *visibility_mesh(l, 1, MaskKind::HiddenTriangleMesh, n);
    const auto [tl, tr, tu, td] = l.fov_tan(1);
    const float rect = (tl + tr) * (tu + td);
    const float total = mesh_area(vis) + mesh_area(hid);
    CHECK(std::fabs(total - rect) / rect < 0.03f);
    CHECK(mesh_area(vis) < rect * 0.99f && mesh_area(hid) > 0.0f);
    for (const MaskMesh* m : {&vis, &hid}) {
        for (const auto i : m->indices) {
            CHECK(i < m->vertices.size());
        }
        CHECK(m->indices.size() % 3 == 0);
    }
    const auto line = *visibility_mesh(l, 1, MaskKind::LineLoop, n);
    CHECK(line.vertices.size() == n);
    CHECK(line.indices.size() == n);
}

TEST(hidden_axis_gives_no_mesh) {
    const Lens l = lens("blackout = 5\n");
    CHECK(!visibility_mesh(l, 0, MaskKind::LineLoop, 32));
}

TEST(y_is_flipped_like_the_default_mask) {
    // a mask that only cuts the top of the panel (user-facing up): the hidden mesh has area only at y < 0
    const Lens l = lens("center = 0 0\nblackout = 0 0 0.9 0\nmask_feather = 0\n");
    const auto hid = *visibility_mesh(l, 0, MaskKind::HiddenTriangleMesh, 64);
    float above = std::numeric_limits<float>::infinity();
    float below = -std::numeric_limits<float>::infinity();
    for (const auto& v : hid.vertices) {
        above = std::fmin(above, v[1]);
        below = std::fmax(below, v[1]);
    }
    // hidden region is at the top of the image -> negative y in mask coordinates
    const auto t = l.fov_tan(0);
    CHECK(std::fabs(above + t[2]) < 1e-3f);
    CHECK(below <= t[3] + 1e-3f);
}

PIBRIDGE_TEST_MAIN()
