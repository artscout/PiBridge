// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/solve.rs (MIT OR Apache-2.0).
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/solve.h"
#include "pibridge_test.h"

#include <cmath>
#include <numbers>

using namespace pibridge::ctrl::track;
namespace g = pibridge::cam::geom;
using pibridge::cam::Kb4;
using pibridge::cam::rfmt::to_radians;

namespace {

LedModel ring_model() {
    // 15 LEDs on a tilted ring, like the Pimax controller (radius ~4.5 cm), normals outward.
    LedModel m;
    for (int i = 0; i < 15; i++) {
        const double a = static_cast<double>(i) / 15.0 * (2.0 * std::numbers::pi) * 0.9;
        const g::Vec3 n = g::normalize({std::cos(a), std::sin(a), -1.0 + 0.3 * std::sin(2.0 * a)});
        m.leds.push_back(Led{{0.045 * std::cos(a), 0.045 * std::sin(a), 0.015 * std::sin(3.0 * a)}, n});
    }
    m.hand = "T";
    m.serial = "t";
    return m;
}

Cam test_cam() {
    return Cam{Kb4::make(640, 480, 232.0, 232.0, 320.0, 240.0, {0.06, -0.06, 0.03, -0.008}, {}), g::Pose{}};
}

} // namespace

TEST(acquires_synthetic_constellation) {
    const LedModel model = ring_model();
    const Cam cam = test_cam();
    const g::Pose truth{g::rodrigues({0.4, 0.3, -0.2}), {0.06, 0.05, 0.40}};
    std::vector<std::pair<double, double>> px;
    const double cos_face = std::cos(to_radians(80.0));
    for (const Led& led : model.leds) {
        if (facing(truth, led, cos_face)) {
            const auto uv = cam.intr.project(truth.apply(led.p));
            px.emplace_back(uv->first + 0.2, uv->second - 0.15);
        }
    }
    px.emplace_back(100.0, 100.0); // a lamp
    const CamObs obs = CamObs::make(cam, px);
    const std::vector<bool> used(obs.px.size(), false);
    const auto tri = model.neighbour_triples(0.05, 100.0);
    const auto s = acquire(model, tri, cam, 0, obs, used, Params{});
    CHECK(s.has_value());
    if (!s) {
        return;
    }
    CHECK(s->matches.size() >= 6);
    CHECK(s->rms_px < 0.5);
    CHECK(g::norm(g::sub(s->cam_from_model.t, truth.t)) < 0.01);
}

/// Three LEDs and a rotation prior 3 deg off: the constrained refine recovers the position to under 1 cm (7 mm) from
/// a start 3 cm away (the few-LED track).
TEST(three_led_refine_with_rotation_prior) {
    const LedModel model = ring_model();
    const Cam cam = test_cam();
    const g::Pose truth{g::rodrigues({0.4, 0.3, -0.2}), {0.06, 0.05, 0.40}};
    const double cos_face = std::cos(to_radians(80.0));
    std::vector<std::size_t> vis;
    for (std::size_t l = 0; l < model.leds.size() && vis.size() < 3; l++) {
        if (facing(truth, model.leds[l], cos_face)) {
            vis.push_back(l);
        }
    }
    CHECK(vis.size() == 3);
    std::vector<Obs> obs;
    for (const std::size_t l : vis) {
        obs.push_back(Obs{&cam, model.leds[l].p, *cam.intr.project(truth.apply(model.leds[l].p))});
    }
    const g::Mat3 prior_r = g::mat_mul(truth.r, g::rodrigues({0.0, to_radians(3.0), 0.0}));
    const g::Pose start{prior_r, g::add(truth.t, {0.02, -0.015, 0.015})};
    const auto [p, rms] = refine_with(start, obs, true, 30, std::pair{prior_r, 300.0});
    const double err = g::norm(g::sub(p.t, truth.t));
    CHECK(err < 0.01);
    CHECK(rms < 1.5);
}

TEST(quat_roundtrip) {
    const g::Mat3 r = g::rodrigues({0.3, -1.2, 0.7});
    const g::Mat3 r2 = quat_to_mat(g::quat_from_mat(r));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            CHECK(std::fabs(r[i][j] - r2[i][j]) < 1e-12);
        }
    }
}

PIBRIDGE_TEST_MAIN()
