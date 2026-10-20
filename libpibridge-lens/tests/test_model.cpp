// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/model.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/lens/fmt.h"
#include "pibridge/lens/model.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::lens;
using pibridge::lens::testing::ASPECT;

namespace {

Lens lens_from(const char* text) {
    auto [cfg, w] = parse(text, ASPECT);
    CHECK(w.empty());
    return Lens(std::move(cfg));
}

float deg_of(float t) {
    return fmt::to_degrees(std::atan(t));
}

} // namespace

TEST(disabled_is_identity) {
    const Lens l = lens_from("distortion = off\n");
    CHECK((l.compute(0, 0.25f, 0.75f) == Uv3{Uv{0.25f, 0.75f}, Uv{0.25f, 0.75f}, Uv{0.25f, 0.75f}}));
    CHECK((l.compute(1, 0.0f, 1.0f) == Uv3{Uv{0.0f, 1.0f}, Uv{0.0f, 1.0f}, Uv{0.0f, 1.0f}}));
}

TEST(symmetric_centre_maps_to_image_centre) {
    const Lens l = lens_from("center = 0 0\nca_r = 1\nca_b = 1\n");
    for (const auto& c : l.compute(0, 0.5f, 0.5f)) {
        CHECK(std::fabs(c[0] - 0.5f) < 1e-6f && std::fabs(c[1] - 0.5f) < 1e-6f);
    }
}

TEST(default_centre_offset_moves_the_axis) {
    const Lens l(LensConfig::defaults(ASPECT));
    // the left eye optical axis is 0.0271 half-heights to the right of the panel centre, 0.009 up
    const float u = 0.5f + 0.0271f / (2.0f * ASPECT);
    const float v = 0.5f - 0.009f / 2.0f;
    const Uv g = l.compute(0, u, v)[1];
    CHECK(std::fabs(g[0] - 0.5f) < 1e-4f && std::fabs(g[1] - 0.5f) < 1e-4f);
}

TEST(mirror_symmetry_with_symmetric_parameters) {
    const Lens l = lens_from("center = 0 0\n");
    const auto a = l.compute(0, 0.2f, 0.3f);
    const auto b = l.compute(0, 0.8f, 0.3f);
    for (std::size_t c = 0; c < 3; c++) {
        CHECK(std::fabs(a[c][0] + b[c][0] - 1.0f) < 1e-5f);
        CHECK(std::fabs(a[c][1] - b[c][1]) < 1e-6f);
    }
    const auto d = l.compute(0, 0.2f, 0.7f);
    for (std::size_t c = 0; c < 3; c++) {
        CHECK(std::fabs(a[c][0] - d[c][0]) < 1e-6f);
        CHECK(std::fabs(a[c][1] + d[c][1] - 1.0f) < 1e-5f);
    }
}

TEST(ring_radii_of_the_default_fit_match_the_table_roughly) {
    const Lens l(LensConfig::defaults(ASPECT));
    // "Dream Air like Pimax" table, green: 10 deg -> 0.25641, 30 -> 0.73145, 50 -> 1.08985 (half-heights)
    for (const auto& [deg, r] : {std::pair{10.0f, 0.25641f}, std::pair{30.0f, 0.73145f}, std::pair{50.0f, 1.08985f}}) {
        const float got = *l.radius_for_angle(0, 1, deg);
        CHECK(std::fabs(got - r) < 0.012f);
    }
}

TEST(red_and_blue_are_scaled_by_ca) {
    const Lens l(LensConfig::defaults(ASPECT));
    const float g = *l.radius_for_angle(0, 1, 40.0f);
    const float r = *l.radius_for_angle(0, 0, 40.0f);
    const float b = *l.radius_for_angle(0, 2, 40.0f);
    CHECK(r > g && g > b);
    CHECK(std::fabs((r / g) - 1.0045f) < 0.003f);
}

TEST(default_polynomial_is_monotonic_to_the_corners) {
    const Lens l(LensConfig::defaults(ASPECT));
    for (std::size_t c = 0; c < 3; c++) {
        CHECK(!l.first_fold(0, c, 1.6f));
    }
    // an obviously folding polynomial is detected
    const Lens bad = lens_from("k = 0 -3 0\n");
    CHECK(bad.first_fold(0, 1, 1.6f).has_value());
}

TEST(fit_range_continues_with_the_tangent_line) {
    auto cfg = LensConfig::defaults(ASPECT);
    cfg.eye[0].fit_range = 1.0f;
    const Lens l(cfg);
    const EyeConfig e = l.eye(0);
    const auto [t1, s1] = radial_tangent(double{e.base[1]}, 1.0, e.k[1], 1.0);
    // continuity at the fit range and linear growth beyond it (green: ca = 1, p' = p)
    const auto at = [&](float r) { return double{l.tangent_at(e, 1, r, 0.0f)[0]}; };
    CHECK(std::fabs(at(1.0f) - t1) < 1e-4);
    CHECK(std::fabs(at(1.3f) - (t1 + s1 * 0.3)) < 1e-3);
    CHECK(std::fabs(at(1.001f) - at(1.0f) - s1 * 0.001) < 1e-3);
    // without a fit range the polynomial keeps rising much faster
    const Lens plain(LensConfig::defaults(ASPECT));
    CHECK(double{plain.tangent_at(plain.eye(0), 1, 1.5f, 0.0f)[0]} > at(1.5f) + 1.0);
}

TEST(extra_polynomial_terms_enter_as_higher_powers) {
    auto cfg = LensConfig::defaults(ASPECT);
    cfg.eye[0].k[1] = Poly{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    cfg.eye[0].k[1][3] = 1.0f; // k4 r^8
    const Lens l(cfg);
    const EyeConfig& e = l.eye(0);
    const float t = l.tangent_at(e, 1, 0.5f, 0.0f)[0];
    const float want = e.base[1] * 0.5f * (1.0f + fmt::powi(0.5f, 8));
    CHECK(std::fabs(t - want) < 1e-6f);
}

TEST(visibility_mask_hides_outside_and_pushes_uv_out_of_range) {
    auto cfg = LensConfig::defaults(ASPECT);
    cfg.eye[0].mask = Mask{MaskShape::Ellipse, 1.0f, 1.0f, 2.0f};
    const Lens l(cfg);
    // panel centre-ish: visible, unchanged by the mask
    CHECK(l.hide_amount(0, 0.5f, 0.5f) == 0.0f);
    const Lens plain(LensConfig::defaults(ASPECT));
    CHECK(l.compute(0, 0.5f, 0.5f) == plain.compute(0, 0.5f, 0.5f));
    // far right edge of the panel: outside the unit circle by more than the feather -> hidden, UV pushed out
    CHECK(l.hide_amount(0, 1.0f, 0.5f) == 1.0f);
    for (const auto& uv : l.compute(0, 1.0f, 0.5f)) {
        CHECK(uv[0] > 1.0f || uv[0] < 0.0f || uv[1] > 1.0f || uv[1] < 0.0f);
    }
    // inside the transition band the amount is strictly between 0 and 1
    auto cfg2 = LensConfig::defaults(ASPECT);
    cfg2.eye[0].mask = Mask{MaskShape::Rect, 1.0f, 1.0f, 2.0f};
    cfg2.eye[0].cx = 0.0f;
    cfg2.eye[0].cy = 0.0f;
    cfg2.eye[0].mask_feather = 0.1f;
    const Lens l2(cfg2);
    // x = 1.05 half-heights is 0.05 outside the rect -> half way through the band
    const float u = 0.5f + 1.05f / (2.0f * ASPECT);
    const float h = l2.hide_amount(0, u, 0.5f);
    CHECK(std::fabs(h - 0.5f) < 0.02f);
}

TEST(blackout_margins_cut_the_panel_edges) {
    const Lens l = lens_from("blackout = 0.2 0 0 0\nmask_feather = 0\n");
    // left margin of 0.2 half-heights = 0.2/(2*aspect) of the width
    const float edge = 0.2f / (2.0f * ASPECT);
    CHECK(l.hide_amount(0, edge * 0.5f, 0.5f) == 1.0f);
    CHECK(l.hide_amount(0, edge * 1.5f, 0.5f) == 0.0f);
    // the other sides are untouched
    CHECK(l.hide_amount(0, 0.999f, 0.5f) == 0.0f);
    CHECK(l.hide_amount(0, 0.5f, 0.001f) == 0.0f);
    CHECK(l.hide_amount(0, 0.5f, 0.999f) == 0.0f);
    const Lens all = lens_from("blackout = 0.1\nmask_feather = 0\n");
    for (const auto& [u, v] : {std::pair{0.001f, 0.5f}, std::pair{0.999f, 0.5f}, std::pair{0.5f, 0.001f},
                              std::pair{0.5f, 0.999f}}) {
        CHECK(all.hide_amount(0, u, v) == 1.0f);
    }
    CHECK(all.hide_amount(0, 0.5f, 0.5f) == 0.0f);
}

TEST(uv_tangent_roundtrip) {
    const Lens l(LensConfig::defaults(ASPECT));
    for (const Uv uv : {Uv{0.1f, 0.9f}, Uv{0.5f, 0.5f}, Uv{1.0f, 0.0f}, Uv{0.33f, 0.77f}}) {
        const Uv t = l.uv_to_tangent(1, uv);
        const Uv b = l.tangent_to_uv(1, t);
        CHECK(std::fabs(b[0] - uv[0]) < 1e-5f && std::fabs(b[1] - uv[1]) < 1e-5f);
    }
}

TEST(radius_for_angle_inverts_the_forward_model) {
    const Lens l(LensConfig::defaults(ASPECT));
    for (const float deg : {5.0f, 25.0f, 45.0f}) {
        for (std::size_t c = 0; c < 3; c++) {
            const float r = *l.radius_for_angle(0, c, deg);
            // forward: a point at panel radius r along +x of the optical axis
            const float t = l.tangent_at(l.eye(0), c, r, 0.0f)[0];
            CHECK(std::fabs(deg_of(t) - deg) < 0.02f);
        }
    }
    CHECK(l.radius_for_angle(0, 1, 89.99f).has_value());
}

PIBRIDGE_TEST_MAIN()
