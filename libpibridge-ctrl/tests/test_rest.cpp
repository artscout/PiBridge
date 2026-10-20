// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/rest.rs (MIT OR Apache-2.0).
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/rest.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl;
using pibridge::cam::rfmt::to_degrees;
using pibridge::cam::rfmt::to_radians;

namespace {

constexpr std::array<double, 4> ID = {0.0, 0.0, 0.0, 1.0};

V3d add(V3d a, V3d b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3d sub(V3d a, V3d b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3d scale(V3d a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double len(V3d a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
bool close(V3d a, V3d b, double tol) { return len(sub(a, b)) < tol; }

std::array<double, 4> quat_axis(V3d axis, double deg) {
    const double h = to_radians(deg) * 0.5;
    const double l = len(axis);
    return {axis[0] / l * std::sin(h), axis[1] / l * std::sin(h), axis[2] / l * std::sin(h), std::cos(h)};
}

V3d finger_dir(const Pose& grip, Hand hand) {
    return forward(grip.compose(grip_to_wrist(hand)));
}

} // namespace

TEST(parse_names_numbers_and_errors) {
    const auto r0 = Rest::parse("");
    CHECK(r0.has_value() && r0.value() == Rest::desk());
    const auto r1 = Rest::parse("legacy");
    CHECK(r1.has_value() && r1.value() == Rest::legacy());
    const Rest r = Rest::parse("0.1, 0.2,0.3").value();
    CHECK(r.side == 0.1 && r.down == 0.2 && r.fwd == 0.3);
    CHECK(r.grip_pitch_deg == Rest::desk().grip_pitch_deg);
    CHECK(Rest::parse("0.1,0.2,0.3,-10")->grip_pitch_deg == -10.0);
    for (const char* bad : {"x", "0.1,0.2", "1,2,3,4,5", "0.1,0.2,9", "0.1,nan,0.3", "0,0,0,400"}) {
        CHECK(!Rest::parse(bad));
    }
    CHECK(*AimMode::parse("") == AimMode::level());
    CHECK(*AimMode::parse("head") == AimMode::head_at(AimMode::DEFAULT_HEAD_DISTANCE));
    CHECK(*AimMode::parse("head:2.5") == AimMode::head_at(2.5));
    for (const char* bad : {"up", "head:", "head:0.01", "head:x"}) {
        CHECK(!AimMode::parse(bad));
    }
    // the error texts quote the value the Rust way
    CHECK(Rest::parse(" a\"b ").error() == "PIMAX_CTRL_REST=\"a\\\"b\": want desk, legacy or side,down,fwd[,pitch_deg]");
    CHECK(Rest::parse("1,2").error() == "PIMAX_CTRL_REST=\"1,2\": want 3 or 4 numbers");
    CHECK(AimMode::parse("up").error() == "PIMAX_CTRL_AIM=\"up\": want level, head or head:<0.2..20 m>");
}

TEST(legacy_reproduces_the_old_fixed_offset) {
    const auto [aim, grip] = controller_poses(Hand::Right, Rest::legacy(), AimMode::level(), ID);
    CHECK(close(aim.p, {0.20, -0.30, -0.35}, 1e-12));
    CHECK(close(grip.p, aim.p, 1e-12));
    CHECK(aim.angle_to(Pose::identity()) < 1e-9 && grip.angle_to(Pose::identity()) < 1e-9);
    const auto l = controller_poses(Hand::Left, Rest::legacy(), AimMode::level(), ID).first;
    CHECK(close(l.p, {-0.20, -0.30, -0.35}, 1e-12));
}

TEST(legacy_fingers_hang_down_and_desk_fingers_point_forward_down) {
    for (Hand hand : {Hand::Left, Hand::Right}) {
        const Pose g = controller_poses(hand, Rest::legacy(), AimMode::level(), ID).second;
        CHECK(finger_dir(g, hand)[1] < -0.95);
        const Pose g2 = controller_poses(hand, Rest::desk(), AimMode::level(), ID).second;
        const V3d d = finger_dir(g2, hand);
        const double below = to_degrees(std::atan2(-d[1], -d[2]));
        CHECK(d[2] < -0.75 && below >= 20.0 && below < 50.0);
    }
}

TEST(desk_hand_is_well_inside_the_view_and_legacy_is_at_its_edge) {
    const auto span = [](const Rest& rest, Hand hand) {
        const Pose g = controller_poses(hand, rest, AimMode::level(), ID).second;
        const Pose w = g.compose(grip_to_wrist(hand));
        const V3d tip = add(w.p, scale(forward(w), 0.19));
        return std::array<std::pair<double, double>, 2>{view_angles(ID, w.p), view_angles(ID, tip)};
    };
    for (Hand hand : {Hand::Left, Hand::Right}) {
        for (const auto& [side, below] : span(Rest::desk(), hand)) {
            CHECK(std::fabs(side) < 33.0 && below >= 10.0 && below < 32.0);
        }
        const auto legacy = span(Rest::legacy(), hand);
        CHECK(legacy[1].second > 44.55);
        CHECK(legacy[0].second > 33.0 && std::fabs(legacy[0].first) > 33.0);
    }
}

TEST(placement_follows_head_yaw_but_not_pitch) {
    const auto yaw90 = quat_axis({0.0, 1.0, 0.0}, 90.0);
    const Pose aim = controller_poses(Hand::Right, Rest::desk(), AimMode::level(), yaw90).first;
    const V3d p = Rest::desk().position(Hand::Right);
    CHECK(close(aim.p, {p[2], p[1], -p[0]}, 1e-9));
    CHECK(close(forward(aim), {-1.0, 0.0, 0.0}, 1e-9));
    const auto pitch = quat_axis({1.0, 0.0, 0.0}, -40.0);
    const auto [aim2, grip2] = controller_poses(Hand::Right, Rest::desk(), AimMode::level(), pitch);
    const auto [aim0, grip0] = controller_poses(Hand::Right, Rest::desk(), AimMode::level(), ID);
    CHECK(close(aim2.p, aim0.p, 1e-9) && aim2.angle_to(aim0) < 1e-9 && grip2.angle_to(grip0) < 1e-9);
}

TEST(head_aim_rays_meet_at_the_look_point) {
    const AimMode mode = AimMode::head_at(1.2);
    for (const auto& head : {ID, quat_axis({1.0, 0.0, 0.0}, -25.0), quat_axis({0.3, 1.0, 0.0}, 35.0)}) {
        const V3d look = scale(Pose{head, {0.0, 0.0, 0.0}}.rotate({0.0, 0.0, -1.0}), 1.2);
        for (Hand hand : {Hand::Left, Hand::Right}) {
            const auto [aim, grip] = controller_poses(hand, Rest::desk(), mode, head);
            const V3d to = sub(look, aim.p);
            const V3d hit = add(aim.p, scale(forward(aim), len(to)));
            CHECK(close(hit, look, 1e-9));
            CHECK(std::fabs(aim.rotate({1.0, 0.0, 0.0})[1]) < 1e-9);
            const Pose g_level = controller_poses(hand, Rest::desk(), AimMode::level(), head).second;
            CHECK(grip.angle_to(g_level) < 1e-12);
        }
    }
}

TEST(aim_to_grip_pitches_the_handle_forward_up) {
    const Pose g = Rest::desk().aim_to_grip();
    CHECK(close(forward(g), {0.0, std::sin(to_radians(45.0)), -std::cos(to_radians(45.0))}, 1e-12));
    CHECK(Rest::legacy().aim_to_grip().angle_to(Pose::identity()) < 1e-12);
}

TEST(yaw_of_drops_pitch_and_roll) {
    const auto q = Pose{quat_axis({0.0, 1.0, 0.0}, 30.0), {0.0, 0.0, 0.0}}
                       .compose(Pose{quat_axis({1.0, 0.0, 0.0}, 50.0), {0.0, 0.0, 0.0}})
                       .q;
    CHECK(yaw_of(q).angle_to(Pose{quat_axis({0.0, 1.0, 0.0}, 30.0), {0.0, 0.0, 0.0}}) < 1e-9);
    CHECK(yaw_of(quat_axis({1.0, 0.0, 0.0}, 90.0)) == Pose::identity());
}

TEST(view_angles_signs) {
    const auto [s, b] = view_angles(ID, {0.1, -0.1, -0.1});
    CHECK(std::fabs(s - 45.0) < 1e-9 && std::fabs(b - 45.0) < 1e-9);
}

PIBRIDGE_TEST_MAIN()
