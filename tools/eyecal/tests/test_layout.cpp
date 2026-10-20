// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/layout.rs tests (MIT OR Apache-2.0); the chip target tests are new.
#include "layout.h"

#include "pibridge/eye/calibration.h"

#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::eyecal;

TEST(everything_is_inside_the_comfortable_field) {
    std::vector<Target> all = calibration_targets(13);
    for (const auto& v : {validation_targets(), recenter_targets(), service_targets()}) {
        all.insert(all.end(), v.begin(), v.end());
    }
    for (const auto& t : all) {
        CHECK(std::fabs(t.az) <= 15.0 && std::fabs(t.el) <= 12.0);
        CHECK(t.eccentricity() < 20.0);
    }
    for (int i = 0; i < 100; i++) {
        const auto p = pursuit_at(i * 0.12);
        CHECK(std::fabs(p.az) <= 10.0 + 1e-9 && std::fabs(p.el) <= 8.0 + 1e-9);
    }
}

TEST(validation_points_are_disjoint_from_calibration_and_cover_all_regions) {
    const auto cal = calibration_targets(13);
    const auto val = validation_targets();
    for (const auto& v : val) {
        for (const auto& c : cal) {
            const double d = std::sqrt((v.az - c.az) * (v.az - c.az) + (v.el - c.el) * (v.el - c.el));
            CHECK(d > 2.5);
        }
    }
    for (const auto r : ALL_REGIONS) {
        std::size_t n = 0;
        for (const auto& t : val) {
            n += region_of(t) == r ? 1 : 0;
        }
        CHECK(n >= 2);
    }
    CHECK(calibration_targets(9).size() == 9);
    CHECK(region_of(Target::at(0.0, 0.0)) == Region::Centre);
    CHECK(region_of(Target::at(15.0, 12.0)) == Region::Outer);
}

TEST(directions_and_points) {
    const Target t = Target::at(15.0, -12.0);
    const auto d = t.dir();
    CHECK(d[0] > 0.0 && d[1] < 0.0 && d[2] < -0.9);
    const auto p = t.point();
    const double l = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    CHECK(std::fabs(l - TARGET_DIST_M) < 1e-12);
    CHECK(t.name() == "+15,-12");
}

TEST(the_chip_targets_point_where_the_captured_requests_say) {
    namespace cal = pibridge::eye::calibration;
    pibridge::eyemath::FrameConv conv; // flip z only: device x right, y up, +z forward
    const auto t = chip_targets(ChipLayout{conv, false, false});
    CHECK(t.size() == 9);
    for (std::size_t i = 0; i < t.size(); i++) {
        const auto& c = cal::targets()[i];
        CHECK(t[i].chip == static_cast<int>(i));
        CHECK(t[i].bright == (i >= 5)); // pass 2 on white
        // the point is the chip's own: (x, y, 2000) mm, z forward
        const auto p = t[i].point();
        CHECK(std::fabs(p[0] - c.x_mm * 1e-3) < 1e-9);
        CHECK(std::fabs(p[1] - c.y_mm * 1e-3) < 1e-9);
        CHECK(std::fabs(p[2] + c.z_mm * 1e-3) < 1e-9);
    }
    CHECK(std::fabs(t[1].az + 15.0) < 0.1 && t[1].el > 14.0); // (-536, 536): left, up
    CHECK(std::fabs(t[5].az + 25.0) < 0.1 && std::fabs(t[5].el) < 1e-9); // (-932, 0): 25 degrees left
    CHECK(std::fabs(t[7].el - 22.8) < 0.1); // (0, 840): 23 degrees up
    // the Crystal Super's flip x mirrors them like its gaze, and the mirrors of the first live test flip them back
    conv.flip_x = true;
    const auto f = chip_targets(ChipLayout{conv, false, false});
    CHECK(std::fabs(f[1].az - 15.0) < 0.1 && f[1].el > 14.0);
    const auto m = chip_targets(ChipLayout{conv, true, true});
    CHECK(std::fabs(m[1].az + 15.0) < 0.1 && m[1].el < -14.0);
    CHECK(f[0].name() == "+0,+0" && f[0].eccentricity() < 1e-9);
}

PIBRIDGE_TEST_MAIN()
