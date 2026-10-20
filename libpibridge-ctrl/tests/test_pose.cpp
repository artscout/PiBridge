// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/pose.rs (MIT OR Apache-2.0).
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/pose.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl;

namespace {
V3d sub(V3d a, V3d b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
bool close(V3d a, V3d b, double tol) {
    const V3d d = sub(a, b);
    return std::fabs(d[0]) <= tol && std::fabs(d[1]) <= tol && std::fabs(d[2]) <= tol;
}
V3d mirror_x(V3d v) { return {-v[0], v[1], v[2]}; }
double qn(const Pose& p) {
    double n = 0.0;
    for (double c : p.q) {
        n += c * c;
    }
    return n;
}
} // namespace

TEST(quaternions_are_unit_and_the_inverse_is_exact) {
    for (Hand h : {Hand::Left, Hand::Right}) {
        const Pose p = grip_to_wrist(h);
        CHECK(std::fabs(qn(p) - 1.0) < 1e-12);
        const Pose id = p.compose(p.inverse());
        for (double c : id.p) {
            CHECK(std::fabs(c) < 1e-12);
        }
        CHECK(id.angle_to(Pose::identity()) < 1e-9);
        const Pose id2 = grip_in_wrist(h).compose(grip_to_wrist(h));
        for (double c : id2.p) {
            CHECK(std::fabs(c) < 1e-12);
        }
    }
}

TEST(left_hand_matches_the_hand_derived_numbers) {
    CHECK(close(grip_to_wrist(Hand::Left).p, {-0.0289, 0.0629, 0.0146}, 0.0005));
}

TEST(right_hand_is_the_mirror_image) {
    const Pose l = grip_to_wrist(Hand::Left);
    const Pose r = grip_to_wrist(Hand::Right);
    CHECK(close(r.p, mirror_x(l.p), 1e-9));
    const Pose m{{l.q[0], -l.q[1], -l.q[2], l.q[3]}, {0.0, 0.0, 0.0}};
    CHECK(m.angle_to(r) < 1e-6);
}

TEST(grip_axes_follow_the_openxr_definition) {
    for (auto [hand, thumb_x] : {std::pair{Hand::Left, 1.0}, std::pair{Hand::Right, -1.0}}) {
        const Pose r = grip_to_wrist(hand);
        const V3d into_palm = r.rotate({0.0, -1.0, 0.0});
        const V3d thumb = r.rotate({thumb_x, 0.0, 0.0});
        const V3d fingers = r.rotate({0.0, 0.0, -1.0});
        const double want_x = hand == Hand::Left ? 1.0 : -1.0;
        CHECK(into_palm[0] * want_x > 0.99);
        CHECK(thumb[2] < -0.97);
        CHECK(fingers[1] < -0.97);
        CHECK(r.p[1] > 0.05);
    }
}

TEST(hand_hangs_naturally_with_identity_grip) {
    const Pose l = grip_to_wrist(Hand::Left);
    CHECK(l.rotate({0.0, 0.0, -1.0})[1] < -0.97);
    CHECK(l.rotate({0.0, -1.0, 0.0})[0] > 0.99);
    CHECK(grip_to_wrist(Hand::Right).rotate({0.0, -1.0, 0.0})[0] < -0.99);
}

TEST(composing_with_a_world_pose_round_trips) {
    const Pose world = Pose::from_axes({0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 3.0});
    for (Hand h : {Hand::Left, Hand::Right}) {
        const Pose wrist = world.compose(grip_to_wrist(h));
        const Pose again = wrist.compose(grip_in_wrist(h));
        CHECK(close(again.p, world.p, 1e-9));
        CHECK(again.angle_to(world) < 1e-9);
    }
}

TEST(grip_from_joints_is_frame_independent) {
    const Pose m = Pose::from_axes({0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.4, -0.2, 0.9});
    for (Hand h : {Hand::Left, Hand::Right}) {
        const double sgn = h == Hand::Right ? -1.0 : 1.0;
        const auto at = [&](V3d v) { return m.compose(Pose::from_position({sgn * v[0], v[1], v[2]})).p; };
        const Pose palm = m.compose(Pose::from_position({sgn * PALM_LEFT[0], PALM_LEFT[1], PALM_LEFT[2]}));
        const Pose moved = grip_from_joints(palm, at(INDEX_PROXIMAL_LEFT), at(RING_PROXIMAL_LEFT), h);
        const Pose expect = m.compose(grip_in_wrist(h));
        CHECK(close(moved.p, expect.p, 1e-9));
        CHECK(moved.angle_to(expect) < 1e-9);
    }
}

TEST(from_axes_covers_all_branches) {
    const V3d cases[4][3] = {
        {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
        {{1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}},
        {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}},
        {{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}},
    };
    for (const auto& c : cases) {
        const Pose p = Pose::from_axes(c[0], c[1], c[2], {0.0, 0.0, 0.0});
        CHECK(std::fabs(qn(p) - 1.0) < 1e-12);
        CHECK(close(p.rotate({1.0, 0.0, 0.0}), c[0], 1e-12));
        CHECK(close(p.rotate({0.0, 1.0, 0.0}), c[1], 1e-12));
        CHECK(close(p.rotate({0.0, 0.0, 1.0}), c[2], 1e-12));
    }
}

PIBRIDGE_TEST_MAIN()
