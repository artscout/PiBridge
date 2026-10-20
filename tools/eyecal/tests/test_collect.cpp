// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/collect.rs tests (MIT OR Apache-2.0).
#include "collect.h"

#include "pibridge/eyemath/correction.h"
#include "pibridge/eyemath/rng.h"

#include "pibridge_test.h"

#include <cmath>
#include <numbers>

using namespace pibridge::eyecal;
namespace em = pibridge::eyemath;

namespace {

std::vector<RawSample> window(const Target& target, double noise_deg, std::uint64_t seed, bool blink) {
    em::Rng r(seed);
    const std::array<Vec3, 2> pos = {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}};
    std::vector<RawSample> w;
    for (int i = 0; i < 120; i++) {
        RawSample s;
        s.t = i / 120.0;
        for (std::size_t e = 0; e < 2; e++) {
            if (blink && s.t >= 0.5 && s.t < 0.6) {
                continue;
            }
            const auto uv = *truth_tangent(target, pos[e]);
            const double sd = noise_deg * std::numbers::pi / 180.0;
            const double a = sd * r.gauss();
            const double b = sd * r.gauss();
            s.eyes[e] = RawEye{em::dir_of(uv.u + a, uv.v + b), pos[e], 3.5};
        }
        w.push_back(s);
    }
    return w;
}

} // namespace

TEST(perfect_tracker_validates_near_zero) {
    const auto cal = em::Calibration::identity(em::FrameConv{}, 0.064);
    const Target t = Target::at(10.0, -5.0);
    const auto w = window(t, 0.5, 4, true);
    const auto ev = evaluate_window(cal, w, t, FixParams{});
    for (const auto& e : ev.eye_err_deg) {
        CHECK(e.has_value() && *e < 0.25);
    }
    CHECK(ev.comb_err_deg.has_value() && *ev.comb_err_deg < 0.2);
    CHECK(ev.loss > 0.05 && ev.loss < 0.15); // the blink counts as loss
    CHECK(ev.comb_rms_s2s_deg > 0.2 && ev.comb_rms_s2s_deg < 0.9);
    const auto ew = analyze_eye(w, 0, FixParams{});
    CHECK(ew.has_value() && ew->used > 70 && ew->valid < ew->total);
    // round trip of the sample conversion
    const auto s = w[0].to_sample(7);
    const auto back = RawSample::from_sample(s);
    CHECK(back.eyes[0].has_value() && back.eyes[0]->pupil_mm == 3.5);
}

PIBRIDGE_TEST_MAIN()
