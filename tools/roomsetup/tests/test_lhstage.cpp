// SPDX-License-Identifier: MIT
#include "../src/lhstage.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::roomsetup;

namespace {

// a pose (stage_from_raw) applied to a raw point
std::array<double, 3> apply(const StagePose& s, std::array<double, 3> p) {
    const double x = s[3], y = s[4], z = s[5], w = s[6];
    // rotate by the quaternion (v + 2w(u x v) + 2u x (u x v))
    const std::array<double, 3> u{x, y, z};
    const auto cross = [](std::array<double, 3> a, std::array<double, 3> b) {
        return std::array<double, 3>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    const auto c1 = cross(u, p);
    const auto c2 = cross(u, c1);
    return {p[0] + 2 * (w * c1[0] + c2[0]) + s[0], p[1] + 2 * (w * c1[1] + c2[1]) + s[1], p[2] + 2 * (w * c1[2] + c2[2]) + s[2]};
}

bool near(double a, double b, double e = 1e-6) {
    return std::fabs(a - b) < e;
}

RecenterResult headAt(double x, double y, double z, double yawDeg, std::optional<double> floor) {
    RecenterResult r;
    const double h = yawDeg * M_PI / 360.0;
    r.head.pose = {x, y, z, 0, std::sin(h), 0, std::cos(h)};
    r.floorStage = floor;
    return r;
}

} // namespace

TEST(file_round_trip) {
    const StagePose p{0.5, -1.25, 2, 0, 0.3826834, 0, 0.9238795};
    const auto back = parseStage(stageText(p));
    CHECK(back && near((*back)[0], 0.5) && near((*back)[1], -1.25) && near((*back)[4], 0.3826834, 1e-6));
    CHECK(!parseStage("1 2 3") && !parseStage("") && !parseStage("1 2 3 nan 0 0 1"));
}

TEST(the_head_becomes_centre_forward_and_the_touch_the_floor) {
    // from the raw space (no stage yet): head at (1, 1.7, -2) looking +X (yaw -90: forward = (-sin, 0, -cos) = (1, 0, 0))
    const auto s = lhStageFromRecenter(kStageIdentity, headAt(1, 1.7, -2, -90, 0.05), std::nullopt);
    const auto c = apply(s, {1, 0.05, -2}); // the floor under the head -> the stage origin
    CHECK(near(c[0], 0) && near(c[1], 0) && near(c[2], 0));
    const auto f = apply(s, {2, 0.05, -2}); // one metre forward of the head -> -Z
    CHECK(near(f[0], 0) && near(f[2], -1));
    const auto head = apply(s, {1, 1.7, -2});
    CHECK(near(head[1], 1.65));
}

TEST(eye_height_or_keep_when_no_touch) {
    const auto e = lhStageFromRecenter(kStageIdentity, headAt(0, 1.8, 0, 0, std::nullopt), 1.65);
    CHECK(near(apply(e, {0, 1.8, 0})[1], 1.65));
    const auto k = lhStageFromRecenter(kStageIdentity, headAt(0, 1.8, 0, 0, std::nullopt), std::nullopt);
    CHECK(near(apply(k, {0, 1.8, 0})[1], 1.8)); // the old floor stays
}

TEST(a_new_stage_composes_onto_the_old_one) {
    // the head is read in the old stage; the same raw point must land at the new origin
    const StagePose old{0.3, -0.1, 0.7, 0, std::sin(0.4), 0, std::cos(0.4)};
    const std::array<double, 3> rawHead{1.2, 1.75, -0.4};
    const auto inOld = apply(old, rawHead);
    const auto s = lhStageFromRecenter(old, headAt(inOld[0], inOld[1], inOld[2], 30, inOld[1] - 1.6), std::nullopt);
    const auto h = apply(s, rawHead);
    CHECK(near(h[0], 0) && near(h[1], 1.6) && near(h[2], 0));
}

TEST(base_stations_file_and_drawing) {
    const auto st = parseBaseStations("LHB-1 1 2.5 -3 0 0 0 1\nbroken\nLHB-2 0 0 0 0 0 0 0\nLHB-3 0 2 0 0 1 0 0");
    CHECK(st.size() == 2 && near(st[0].pos().y, 2.5) && near(st[1].axis(2).z, -1)); // LHB-2: no rotation, left out
    pibridge::room::Batch b;
    drawBaseStations(b, st);
    CHECK(b.lines.size() == 2 * 2 * 14 && b.tris.empty()); // 12 edges + the front cross, per station
}

PIBRIDGE_TEST_MAIN()
