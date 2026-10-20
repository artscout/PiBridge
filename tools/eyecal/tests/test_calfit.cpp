// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/calfit.rs tests (MIT OR Apache-2.0).
#include "calfit.h"

#include "pibridge/eyemath/rng.h"

#include "pibridge_test.h"

#include <cmath>
#include <numbers>

using namespace pibridge::eyecal::calfit;
namespace em = pibridge::eyemath;

namespace {

double rad(double d) { return d * std::numbers::pi / 180.0; }

/// Chip model: raw = gain * truth + offset, plus 0.1 tangent per mm of origin error on u and 0.06 on v.
std::vector<TargetSamples> world(std::uint64_t seed, std::size_t nt) {
    em::Rng r(seed);
    std::vector<TargetSamples> out;
    for (std::size_t i = 0; i < nt; i++) {
        const double az = -15.0 + 30.0 * static_cast<double>(i % 5) / 4.0;
        const double el = -12.0 + 24.0 * static_cast<double>(i / 5) / static_cast<double>(std::max<std::size_t>(nt / 5, 2) - 1);
        const Tangent truth{std::tan(rad(az)), std::tan(rad(el))};
        TargetSamples t;
        t.truth = truth;
        t.weight = 1.0;
        for (int k = 0; k < 60; k++) {
            const double o0 = 1.2 * r.gauss();
            const double o1 = 0.6 * r.gauss();
            const double o2 = 0.8 * r.gauss();
            const double n = 0.004;
            const double nu = n * r.gauss();
            const Tangent raw0{0.62 * truth.u - 0.05 - 0.10 * o0 + nu, 0.0};
            const double nv = n * r.gauss();
            SamplePt s;
            s.raw = Tangent{raw0.u, 0.62 * truth.v - 0.2 - 0.06 * o1 + nv};
            s.org_mm = {o0, o1, o2};
            s.pupil_mm = 0.0;
            t.samples.push_back(s);
        }
        out.push_back(std::move(t));
    }
    return out;
}

std::vector<const TargetSamples*> ptrs(const std::vector<TargetSamples>& v) {
    std::vector<const TargetSamples*> p;
    for (const auto& t : v) {
        p.push_back(&t);
    }
    return p;
}

} // namespace

TEST(origin_terms_are_found_and_win_the_selection) {
    const auto t = world(5, 15);
    const auto sel = select(t, 0.3, false);
    const auto [m, f] = sel.best();
    CHECK(m != nullptr && f != nullptr);
    if (m == nullptr || f == nullptr) {
        return;
    }
    CHECK(m->org.has_value());
    CHECK(m->loo.mean < 0.6);
    CHECK(std::fabs(f->org_u[0] - 0.10 / 0.62) < 0.03);
    CHECK(std::fabs(f->org_v[1] - 0.06 / 0.62) < 0.03);
    const em::ModelFit* plain = nullptr;
    for (const auto& c : sel.selection.candidates) {
        if (!c.org && c.kind == em::ModelKind::Affine) {
            plain = &c;
            break;
        }
    }
    CHECK(plain != nullptr && plain->loo.mean > 2.0 * m->loo.mean);
    // new targets
    const auto v = world(6, 10);
    std::vector<double> errs;
    for (const auto& x : v) {
        if (auto e = target_error_deg(*f, x)) {
            errs.push_back(*e);
        }
    }
    CHECK(em::stats(errs).mean < 0.7);
}

TEST(no_origin_variation_means_no_origin_candidates) {
    auto t = world(7, 15);
    for (auto& x : t) {
        for (auto& s : x.samples) {
            s.org_mm = {0.0, 0.0, 0.0};
        }
    }
    const auto sel = select(t, 0.3, false);
    for (const auto& c : sel.selection.candidates) {
        CHECK(!c.org.has_value());
    }
    CHECK(sel.best().first != nullptr);
}

TEST(a_wild_sample_cluster_does_not_drag_the_fit) {
    auto t = world(8, 15);
    for (std::size_t k = 0; k < 15; k++) {
        t[3].samples[k].raw.u += 0.3;
    }
    const Spec spec{em::ModelKind::Affine, true, em::Ridge{}, 0.005, false};
    const auto all = ptrs(t);
    const auto f = fit(all, spec);
    CHECK(f.has_value());
    const auto clean = world(9, 10);
    std::vector<double> errs;
    for (const auto& x : clean) {
        if (auto e = target_error_deg(*f, x)) {
            errs.push_back(*e);
        }
    }
    CHECK(em::stats(errs).mean < 1.0);
}

PIBRIDGE_TEST_MAIN()
