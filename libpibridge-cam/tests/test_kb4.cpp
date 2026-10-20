// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/kb4.rs (MIT OR Apache-2.0).
#include "pibridge/cam/kb4.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::cam;

static Kb4 cam0() {
    return Kb4::make(640, 480, 234.7133343909, 234.7063990438, 325.3779138659, 248.2829309197,
                     {0.0589502951, -0.0603263194, 0.0320281449, -0.0082048556}, {0.0011599741, -0.0001122173});
}

TEST(roundtrip) {
    const Kb4 c = cam0();
    for (const auto& [u, v] : {std::pair{325.0, 248.0}, {0.0, 248.0}, {639.0, 248.0}, {325.0, 0.0}, {100.0, 400.0},
                              {600.0, 60.0}}) {
        const auto r = c.unproject(u, v);
        CHECK(r.has_value());
        const auto uv = c.project(*r);
        CHECK(uv.has_value());
        CHECK(std::fabs(u - uv->first) < 1e-6 && std::fabs(v - uv->second) < 1e-6);
    }
    const Kb4 ce = c.with_extra_radial();
    const auto r = *ce.unproject(10.0, 248.0);
    const auto uv = *ce.project(r);
    CHECK(std::fabs(uv.first - 10.0) < 1e-6 && std::fabs(uv.second - 248.0) < 1e-6);
}

TEST(image_corners_are_outside_the_model) {
    // max theta_d ~1.505 rad -> ~353 px from the principal point; the corners are ~390-410 px away.
    const Kb4 c = cam0();
    CHECK(!c.unproject(0.0, 0.0));
    CHECK(!c.unproject(639.0, 479.0));
    const double lim = rfmt::to_degrees(c.theta_limit);
    CHECK(lim >= 90.0 && lim < 95.0);
}

TEST(fov_matches_camgeom) {
    // camgeom.py: trackingA HFOV ~155 deg over 640 px.
    const Kb4 c = cam0();
    const double l = *c.angle_of_pixel_deg(0.0, c.cy);
    const double r = *c.angle_of_pixel_deg(639.0, c.cy);
    CHECK(std::fabs(l + r - 155.0) < 3.0);
}

TEST(behind_camera_is_rejected) {
    const Kb4 c = cam0();
    CHECK(!c.project({0.0, 0.0, -1.0}));
    CHECK(!c.project({1.0, 0.0, -1.0}));
    const auto p = c.project({0.0, 0.0, 1.0});
    CHECK(p && p->first == c.cx && p->second == c.cy);
}

PIBRIDGE_TEST_MAIN()
