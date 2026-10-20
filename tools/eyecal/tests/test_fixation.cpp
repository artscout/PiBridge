// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/fixation.rs tests (MIT OR Apache-2.0).
#include "fixation.h"

#include "pibridge/eyemath/rng.h"

#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <numbers>

using namespace pibridge::eyecal;

namespace {

double rad(double d) { return d * std::numbers::pi / 180.0; }
double deg(double r) { return r * 180.0 / std::numbers::pi; }

/// 1.2 s at 120 Hz: a 10 degree saccade in the first 40 ms, fixation with 0.7 deg noise per axis (what this tracker
/// gives), a blink at 0.6..0.7 s.
std::pair<std::vector<double>, std::vector<OptTan>> window(std::uint64_t seed) {
    pibridge::eyemath::Rng r(seed);
    const int n = 144;
    std::vector<double> t;
    for (int i = 0; i < n; i++) {
        t.push_back(i / 120.0);
    }
    const Tangent tgt{std::tan(rad(10.0)), 0.0};
    std::vector<OptTan> tan;
    for (const double ti : t) {
        if (ti >= 0.6 && ti < 0.7) {
            tan.emplace_back(std::nullopt);
            continue;
        }
        const double s = rad(0.7);
        const Tangent base = ti < 0.04 ? Tangent{tgt.u * ti / 0.04, 0.0} : tgt;
        const double a = s * r.gauss();
        const double b = s * r.gauss();
        tan.emplace_back(Tangent{base.u + a, base.v + b});
    }
    return {t, tan};
}

std::vector<bool> usable_of(const std::vector<Label>& l) {
    std::vector<bool> u;
    for (const auto x : l) {
        u.push_back(x == Label::Fixation);
    }
    return u;
}

std::size_t first_at(const std::vector<double>& t, double x) {
    return static_cast<std::size_t>(std::ranges::find_if(t, [&](double v) { return v >= x; }) - t.begin());
}

} // namespace

TEST(labels_find_the_saccade_and_the_blink_edges_not_the_noise) {
    for (std::uint64_t seed = 1; seed < 6; seed++) {
        const auto [t, tan] = window(seed);
        const auto l = label(t, tan, FixParams{});
        CHECK(std::find(l.begin(), l.begin() + 4, Label::Saccade) != l.begin() + 4);
        const std::size_t i_blink = first_at(t, 0.6);
        CHECK(l[i_blink] == Label::Invalid);
        CHECK(l[i_blink - 1] == Label::BlinkEdge);
        const std::size_t after = first_at(t, 0.7);
        CHECK(l[after] == Label::BlinkEdge);
        // fixation samples outside the saccade and the blink are kept, the noise is not a saccade
        const auto kept = std::count(l.begin() + 20, l.begin() + static_cast<std::ptrdiff_t>(i_blink - 8), Label::Fixation);
        const auto len = static_cast<double>(i_blink - 8 - 20);
        CHECK(static_cast<double>(kept) > 0.9 * len);
    }
}

TEST(longest_fixation_is_the_steady_stretch) {
    const auto [t, tan] = window(3);
    const auto l = label(t, tan, FixParams{});
    const auto usable = usable_of(l);
    const auto r = longest_fixation(t, tan, usable, FixParams{});
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    CHECK(t[r->start] > 0.03 && t[r->start] < 0.15);
    CHECK(t[r->end - 1] - t[r->start] > 0.4);
    std::vector<Tangent> vals;
    for (std::size_t i = r->start; i < r->end; i++) {
        if (usable[i] && tan[i]) {
            vals.push_back(*tan[i]);
        }
    }
    const auto m = median_tangent(vals);
    CHECK(m.has_value());
    CHECK(std::fabs(deg(std::atan(m->first.u)) - 10.0) < 0.3 && std::fabs(deg(std::atan(m->first.v))) < 0.3);
    CHECK(deg(std::atan(m->second)) < 1.2);
    const double p = rms_s2s_deg(vals);
    CHECK(p > 0.6 && p < 1.6);
    CHECK(sd_deg(vals) > 0.6);
    FixParams shorter;
    shorter.min_duration_s = 5.0;
    CHECK(!longest_fixation(t, tan, usable, shorter).has_value());
}

TEST(two_fixations_pick_the_longer_one) {
    // 0.3 s at the old place, saccade, 0.8 s at the new one
    pibridge::eyemath::Rng r(11);
    std::vector<double> t;
    for (int i = 0; i < 132; i++) {
        t.push_back(i / 120.0);
    }
    std::vector<OptTan> tan;
    for (const double ti : t) {
        const Tangent base = ti < 0.3 ? Tangent{0.0, 0.0} : Tangent{0.0, 0.15};
        const double a = 0.005 * r.gauss();
        const double b = 0.005 * r.gauss();
        tan.emplace_back(Tangent{base.u + a, base.v + b});
    }
    const auto l = label(t, tan, FixParams{});
    const auto f = longest_fixation(t, tan, usable_of(l), FixParams{});
    CHECK(f.has_value() && t[f->start] > 0.3);
}

PIBRIDGE_TEST_MAIN()
