// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/slam.rs (MIT OR Apache-2.0). The Rust lab-data tests read the
// project headset's own calibration (analysis/headset-calibration.xml); the shipped reference is that file with the
// serial scrubbed, so the twins run on it always.
#include "pibridge/cam/json.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"
#include "pibridge_test.h"

#include <cmath>
#include <functional>

using namespace pibridge::cam;
using namespace pibridge::cam::slam;
namespace rf = pibridge::cam::rfmt;

namespace {

DeviceCalib cal() {
    return *DeviceCalib::parse(reference_xml());
}

bool close3(const Vec3& a, const Vec3& b, double eps) {
    return geom::norm(geom::sub(a, b)) < eps;
}

/// Simulated head walking a circle; the tracker world is yawed and offset from ours.
void sim(double t, double slam_yaw, const Vec3& slam_off, Vec3& pos, Quat& q, Quat& q_world) {
    const double head_yaw = 0.5 * t;
    q_world = qmul(qyaw(head_yaw), {std::sin(0.1), 0.0, 0.0, std::cos(0.1)});
    const Vec3 pos_world{0.5 * std::sin(0.3 * t), 0.02 * std::sin(t), 0.5 * std::cos(0.3 * t) - 0.5};
    const Quat wq = qyaw(slam_yaw);
    q = qmul(wq, q_world);
    pos = geom::add(qrot(wq, pos_world), slam_off);
}

std::uint64_t ts_of(double t) {
    return rf::f64_to_u64(1e9 + t * 1e9);
}

/// Feed n tracker poses 40 ms apart, position pos(i), identity rotation, IMU still flag still.
std::pair<std::uint32_t, std::uint64_t> feed(Supervisor& s, std::uint64_t t0, std::uint64_t n, bool still,
                                             const std::function<Vec3(std::uint64_t)>& pos) {
    std::uint32_t ev = 0;
    std::uint64_t ts = t0;
    for (std::uint64_t i = 0; i < n; i++) {
        ts = t0 + i * 40'000'000;
        s.set_still(still);
        ev |= s.update(ts, TrackSample{ts, pos(i), QUAT_ID, QUAT_ID}).events;
    }
    return {ev, ts};
}

SupervisorParams with_acquire(std::uint32_t n) {
    SupervisorParams p;
    p.acquire_poses = n;
    return p;
}

} // namespace

TEST(extrinsics_are_plausible) {
    const auto c = cal();
    const auto a = *t_imu_cam(c, 0);
    const auto b = *t_imu_cam(c, 1);
    // Lower tracking cameras: about 10 cm apart along body x, a few cm from the IMU, looking forward (-z).
    const Vec3 base = geom::sub(b.t, a.t);
    CHECK(std::fabs(geom::norm(base) - 0.112) < 0.02);
    CHECK(std::fabs(base[0]) > 0.08);
    for (const auto& p : {a, b}) {
        CHECK(geom::norm(p.t) < 0.12);
        const Vec3 fwd = geom::col(p.r, 2);
        CHECK(fwd[2] < -0.5);
        CHECK(geom::orthonormality_error(p.r) < 1e-9);
    }
}

TEST(basalt_json_parses_and_roundtrips) {
    const auto c = cal();
    SlamSetup setup;
    setup.cams = {0, 1, 2, 3};
    const auto s = *basalt_calib_json(c, setup, ImuNoise{});
    const auto v = *json::parse(s);
    const auto* v0 = v.get("value0");
    const auto& t = *v0->get("T_imu_cam")->as_array();
    CHECK(t.size() == 4);
    const auto& intr = *v0->get("intrinsics")->as_array();
    CHECK(*intr[1].get("camera_type")->as_str() == "kb4");
    const double fx = *intr[1].get("intrinsics")->get("fx")->as_f64();
    CHECK(std::fabs(fx - c.cameras[1].intr.fx) < 1e-12);
    // T_imu_cam of cam 1 equals t_imu_cam().
    const Quat q{*t[1].get("qx")->as_f64(), *t[1].get("qy")->as_f64(), *t[1].get("qz")->as_f64(),
                 *t[1].get("qw")->as_f64()};
    const auto p = *t_imu_cam(c, 1);
    const Mat3 m = mat_of(q);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            CHECK(std::fabs(m[i][j] - p.r[i][j]) < 1e-9);
        }
    }
    CHECK(v0->get("imu_update_rate")->as_f64() == 1000.0);
}

TEST(two_noise_regimes_scale_the_densities_and_switch_with_hysteresis) {
    const ImuNoise n;
    CHECK(n.for_regime(NoiseRegime::Still) == n);
    const ImuNoise m = n.for_regime(NoiseRegime::Moving);
    CHECK(std::fabs(m.gyro_noise_std[0] - 0.0225) < 1e-9);
    CHECK(std::fabs(m.accel_noise_std[2] - 1.296) < 1e-9);
    CHECK(m.gyro_bias_std == n.gyro_bias_std);
    NoiseSelector s;
    CHECK(s.update(0.03) == NoiseRegime::Still);
    CHECK(s.update(0.06) == NoiseRegime::Moving);
    CHECK(s.update(0.03) == NoiseRegime::Moving); // between the limits it stays
    CHECK(s.update(0.01) == NoiseRegime::Still);
}

TEST(imu_noise_file) {
    ImuNoise n;
    n.gyro_noise_std = {1e-4, 2e-4, 3e-4};
    const auto back = ImuNoise::parse(n.to_json("test"));
    CHECK(back && back->gyro_noise_std == n.gyro_noise_std);
    const auto scalar = ImuNoise::parse("{\"accel_noise_std\": 0.01}");
    CHECK(scalar && scalar->accel_noise_std == (Vec3{0.01, 0.01, 0.01}));
    CHECK(!ImuNoise::parse("{\"gyro_bias_std\": 0}"));
}

TEST(cams_list) {
    CHECK(parse_cams("0,1", 4) == (std::vector<std::size_t>{0, 1}));
    CHECK(parse_cams(" 0, 1,2,3 ", 4) == (std::vector<std::size_t>{0, 1, 2, 3}));
    CHECK(!parse_cams("0,0", 4));
    CHECK(!parse_cams("4", 4));
    CHECK(!parse_cams("", 4));
}

TEST(yaw_extraction) {
    for (const double deg : {-170.0, -45.0, 0.0, 30.0, 135.0}) {
        const double y = rf::to_radians(deg);
        // yaw then some pitch: yaw_of must still return the yaw.
        const Quat q = qmul(qyaw(y), {std::sin(0.2), 0.0, 0.0, std::cos(0.2)});
        CHECK(std::fabs(yaw_of(q) - y) < 1e-9);
    }
}

TEST(supervisor_acquires_aligns_and_survives_a_jump) {
    const Vec3 head{0.0, -0.02, 0.07};
    Supervisor s(SupervisorParams{}, head);
    const double yaw = rf::to_radians(70.0);
    const Vec3 off{3.0, -1.0, 2.0};
    double t = 0.0;
    std::optional<int> acquired_at;
    for (int i = 0; i < 40; i++) {
        Vec3 pos;
        Quat q, q3;
        sim(t, yaw, off, pos, q, q3);
        const std::uint64_t ts = ts_of(t);
        const auto o = s.update(ts + 15'000'000, TrackSample{ts, pos, q, q3});
        if ((o.events & events::ACQUIRED) != 0) {
            acquired_at = i;
            // Head lands at the origin (continuity with the 3DoF output at the origin).
            CHECK(close3(s.head_world(pos, q), {0.0, 0.0, 0.0}, 1e-9));
        }
        t += 0.04;
    }
    CHECK(acquired_at == std::optional<int>(11));
    CHECK(s.state() == TrackState::Tracking);
    // After acquisition the world orientation of the IMU equals the 3DoF one (yaw aligned).
    Vec3 pos;
    Quat q, q3;
    sim(t, yaw, off, pos, q, q3);
    auto o = s.update(ts_of(t), TrackSample{ts_of(t), pos, q, q3});
    const Quat wq = qmul(o.world_from_slam.second, q);
    CHECK(qangle_deg(qmul(qconj(wq), q3)) < 1e-6);
    const Vec3 before = s.head_world(pos, q);
    // A 1 m jump in one frame: lost, held at the last good head position.
    t += 0.04;
    Vec3 pos2;
    Quat q2, q32;
    sim(t, yaw, off, pos2, q2, q32);
    pos2[0] += 1.0;
    const std::uint64_t ts2 = ts_of(t);
    o = s.update(ts2, TrackSample{ts2, pos2, q2, q32});
    CHECK(o.state == TrackState::Lost);
    CHECK((o.events & events::LOST) != 0);
    CHECK(close3(o.held_pos, before, 1e-9));
    // Heading continuity: the corrected 3DoF orientation equals the last tracked world orientation.
    CHECK(qangle_deg(qmul(qconj(qmul(o.fix3dof, q3)), wq)) < 1e-6);
    // The tracker comes back (new, shifted world): re-acquire at the held position.
    const Vec3 off2{10.0, 0.0, 0.0};
    bool reacq = false;
    for (int k = 0; k < 30; k++) {
        t += 0.04;
        Vec3 p3;
        Quat qq, qq3;
        sim(t, yaw + 0.5, off2, p3, qq, qq3);
        const std::uint64_t ts = ts_of(t);
        const auto o2 = s.update(ts, TrackSample{ts, p3, qq, qq3});
        if ((o2.events & events::ACQUIRED) != 0) {
            reacq = true;
            CHECK(close3(s.head_world(p3, qq), o2.held_pos, 1e-9));
        }
    }
    CHECK(reacq);
    CHECK(s.losses == 1);
    CHECK(s.acquisitions == 2);
}

TEST(gyro_bias_cache_is_raw_dps) {
    // The driver writes raw IMU axes in deg/s; the body frame negates x and z.
    const auto b = *parse_gyro_bias_cache("0.2315 0.7274 0.4080\n");
    const Vec3 want{rf::to_radians(-0.2315), rf::to_radians(0.7274), rf::to_radians(-0.4080)};
    CHECK(close3(b, want, 1e-12));
    CHECK(!parse_gyro_bias_cache("1 2"));
    CHECK(!parse_gyro_bias_cache("9 0 0"));
    const auto c = cal();
    SlamSetup setup;
    setup.gyro_bias = b;
    const auto v = *json::parse(*basalt_calib_json(c, setup, ImuNoise{}));
    const auto& g = *v.get("value0")->get("calib_gyro_bias")->as_array();
    CHECK(g.size() == 12);
    CHECK(std::fabs(*g[1].as_f64() - b[1]) < 1e-12);
    CHECK(g[3].as_f64() == 0.0);
}

TEST(factory_accel_bias_goes_into_calib_accel_bias) {
    const auto c = cal();
    const auto ab = factory_accel_bias(c);
    CHECK(ab.has_value());
    bool all_small = true, any_nonzero = false;
    for (const double a : *ab) {
        all_small = all_small && std::fabs(a) < 1.0;
        any_nonzero = any_nonzero || a != 0.0;
    }
    CHECK(all_small && any_nonzero);
    SlamSetup setup;
    setup.accel_bias = *ab;
    const auto v = *json::parse(*basalt_calib_json(c, setup, ImuNoise{}));
    const auto& a = *v.get("value0")->get("calib_accel_bias")->as_array();
    CHECK(a.size() == 9);
    for (int k = 0; k < 3; k++) {
        CHECK(std::fabs(*a[static_cast<std::size_t>(k)].as_f64() - (*ab)[k]) < 1e-12);
    }
    CHECK(a[3].as_f64() == 0.0);
}

TEST(still_headset_holds_and_rezeroes_a_drifting_tracker_without_a_loss) {
    Supervisor s(with_acquire(3), {0.0, 0.0, 0.0});
    // A tracker with no landmarks: constant acceleration 0.07 m/s^2 while the headset lies still, with a 2 deg tilt
    // between the tracker's and the 3DoF filter's attitude (as live: 1-3 deg).
    const Quat tilt = qnorm({std::sin(rf::to_radians(2.0)) * 0.5, 0.0, 0.0, 1.0});
    auto drift = [](double t) { return Vec3{0.5 * 0.07 * t * t, 0.0, 0.0}; };
    std::uint32_t ev = 0;
    std::uint64_t ts = 1'000'000'000;
    for (std::uint64_t i = 0; i < 50; i++) {
        ts = 1'000'000'000 + i * 40'000'000;
        s.set_still(true);
        ev |= s.update(ts, TrackSample{ts, drift(static_cast<double>(i) * 0.04), tilt, QUAT_ID}).events;
        if ((ev & events::RESET_WANTED) != 0) {
            break; // the tracker restarts: its old world ends here
        }
    }
    CHECK((ev & events::RESET_WANTED) != 0);
    CHECK((ev & events::REZEROED) != 0);
    CHECK((ev & events::LOST) == 0);
    CHECK(s.still_resets == 1 && s.losses == 0);
    // Output held at the acquisition position (origin), and the 3DoF fix carries the tracker's full attitude (tilt
    // included): fix * q3dof == world orientation of the tracker.
    const auto o = s.output(0);
    CHECK(close3(o.held_pos, {0.0, 0.0, 0.0}, 1e-9));
    CHECK(qangle_deg(qmul(qconj(qmul(o.fix3dof, QUAT_ID)), tilt)) < 1e-6);
    // New tracker world after the reset (another origin and yaw): silently re-aligned at the held position.
    auto [ev2, ts2] = feed(s, ts + 40'000'000, 10, true, [](std::uint64_t i) {
        return geom::add(Vec3{5.0, 0.0, 0.0}, Vec3{0.0001 * static_cast<double>(i), 0.0, 0.0});
    });
    CHECK((ev2 & (events::ACQUIRED | events::LOST)) == 0);
    CHECK(s.state() == TrackState::Tracking);
    CHECK(s.acquisitions == 1 && s.losses == 0);
    auto l = *s.last();
    CHECK(geom::norm(s.head_world(l.pos, l.q)) < 0.002);
    // Moving (not still): no hold, the output follows the tracker.
    auto [ev3, ts3] = feed(s, ts2 + 40'000'000, 10, false,
                           [](std::uint64_t i) { return Vec3{5.0 + 0.05 * static_cast<double>(i), 0.0, 0.0}; });
    (void)ts3;
    CHECK((ev3 & events::RESET_WANTED) == 0);
    l = *s.last();
    CHECK(s.head_world(l.pos, l.q)[0] > 0.3);
}

TEST(still_rezero_off_is_the_old_dropout) {
    Supervisor s(with_acquire(3), {0.0, 0.0, 0.0});
    StillParams sp;
    sp.rezero = false;
    s.set_still_params(sp);
    auto drift = [](double t) { return Vec3{0.5 * 0.07 * t * t, 0.0, 0.0}; };
    // 25 poses: up to the reset (3 cm from the hold start), the tracker restarts then.
    auto [ev, ts] =
        feed(s, 1'000'000'000, 25, true, [&](std::uint64_t i) { return drift(static_cast<double>(i) * 0.04); });
    CHECK((ev & events::LOST) != 0 && (ev & events::RESET_WANTED) != 0);
    CHECK(s.still_resets == 1 && s.losses == 1);
    auto [ev2, ts2] = feed(s, ts + 40'000'000, 10, true,
                           [](std::uint64_t i) { return Vec3{5.0 + 0.0001 * static_cast<double>(i), 0.0, 0.0}; });
    (void)ts2;
    CHECK((ev2 & events::ACQUIRED) != 0);
    CHECK(s.acquisitions == 2);
}

TEST(still_headset_with_a_good_tracker_is_left_alone) {
    Supervisor s(with_acquire(3), {0.0, 0.0, 0.0});
    // Landmarks lock the pose: millimetre jitter only.
    auto [ev, ts] = feed(s, 1'000'000'000, 200, true, [](std::uint64_t i) {
        return Vec3{0.001 * (static_cast<double>(i % 3) - 1.0), 0.0, 0.0};
    });
    (void)ts;
    CHECK((ev & events::RESET_WANTED) == 0);
    CHECK(s.state() == TrackState::Tracking);
    const auto l = *s.last();
    CHECK(geom::norm(s.head_world(l.pos, l.q)) < 1e-9); // held exactly
}

TEST(supervisor_stale_and_reset) {
    Supervisor s(with_acquire(2), {0.0, 0.0, 0.0});
    for (std::uint64_t i = 0; i < 3; i++) {
        const std::uint64_t ts = 1'000'000'000 + i * 40'000'000;
        s.update(ts, TrackSample{ts, {0.0, 0.0, 0.0}, QUAT_ID, QUAT_ID});
    }
    CHECK(s.state() == TrackState::Tracking);
    const std::uint64_t base = 1'080'000'000;
    // Same pose repeated (no new timestamp) for longer than stale_ns: lost.
    auto o = s.update(base + 400'000'000, std::nullopt);
    CHECK(o.state == TrackState::Lost);
    CHECK(std::string(o.reason) == "no tracker pose (stale)");
    // Lost for > 3 s: one reset request, not repeated within 6 s.
    o = s.update(base + 3'500'000'000, std::nullopt);
    CHECK((o.events & events::RESET_WANTED) != 0);
    o = s.update(base + 5'000'000'000, std::nullopt);
    CHECK((o.events & events::RESET_WANTED) == 0);
    o = s.update(base + 9'600'000'000, std::nullopt);
    CHECK((o.events & events::RESET_WANTED) != 0);
    CHECK(s.resets == 2);
}

TEST(supervisor_rejects_rotation_disagreement) {
    Supervisor s(with_acquire(3), {0.0, 0.0, 0.0});
    // The tracker spins 20 deg per frame while the gyro says still: never acquires.
    for (std::uint64_t i = 0; i < 20; i++) {
        const std::uint64_t ts = 1'000'000'000 + i * 40'000'000;
        const Quat q = qyaw(rf::to_radians(20.0 * static_cast<double>(i)));
        const auto o = s.update(ts, TrackSample{ts, {0.0, 0.0, 0.0}, q, QUAT_ID});
        CHECK(o.state != TrackState::Tracking);
    }
    CHECK(s.rejects >= 18);
}

TEST(recenter_moves_head_to_origin) {
    Supervisor s(with_acquire(1), {0.0, 0.0, 0.0});
    auto mk = [](std::uint64_t i, double x) {
        return TrackSample{1'000'000'000 + i * 40'000'000, {x, 0.0, 0.0}, QUAT_ID, QUAT_ID};
    };
    s.update(1'000'000'000, mk(0, 0.0));
    s.update(1'040'000'000, mk(1, 0.05));
    const Vec3 p1 = s.head_world({0.05, 0.0, 0.0}, QUAT_ID);
    CHECK(geom::norm(p1) > 0.04);
    s.request_recenter();
    const auto o = s.update(1'080'000'000, mk(2, 0.05));
    CHECK((o.events & events::RECENTERED) != 0);
    CHECK(close3(s.head_world({0.05, 0.0, 0.0}, QUAT_ID), {0.0, 0.0, 0.0}, 1e-9));
}

PIBRIDGE_TEST_MAIN()
