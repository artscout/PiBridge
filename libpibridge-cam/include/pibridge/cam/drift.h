// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/drift.rs (MIT OR Apache-2.0).
//
// Drift governor (PIMAX_SLAM_DRIFT_HOLD): tracker translation that the accelerometer cannot explain is put into the
// room alignment (like the desk still hold), so the head output does not slide with a drifting tracker while it is
// worn. Rotation is never touched.
//
// Session 2026-10-18 11:58-12:06 (seated, worn): Basalt's position slid 4.1 m right and 1.6 m up in 8 minutes,
// 1.5-3.3 cm/s in a quiet 20 s and 1.2 m in 70 s of fast head turns, while Monado reported "0 lost" and the output
// followed the tracker (omarchyvr ai/LESSONS.md row 77).
//
// The governor looks at the NECK, the pivot a seated head turns about (neck_in_head from the head origin): turning
// moves the head on an arc around it, which the rotation explains, while the neck itself only moves when the body
// moves (a lean, standing up, walking), and that shakes the accelerometer.
//
// * Motion evidence L: the neck's linear acceleration (specific force minus the rotation terms
//   alpha x r + w x (w x r), turned into the world with the tracker attitude carried on by the gyro, gravity
//   removed), as the root of the summed per-axis variances over the last act_window_ns. A variance, not an
//   integral: the accelerometer bias and gravity leaking through a slightly wrong attitude are nearly constant over a
//   second and drop out. A worn head at rest shows 0.02-0.05 m/s^2, a lean 0.1-0.4, steps 0.5-2.
// * Allowed neck speed = floor + gain_s x max(0, L - noise).
// * The tracker's neck speed over the last window_ns; what exceeds the allowed speed is held (moved into the room
//   alignment), fully up to band_lo and fading out to nothing at band_hi (turn_band_* while turning): Basalt's slide
//   is slow (centimetres per second), so anything faster is left to the tracker even when the IMU is unsure.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/cam/slam.h"

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

namespace pibridge::cam::headpose {
struct ImuSample;
struct BodyState;
} // namespace pibridge::cam::headpose

namespace pibridge::cam::drift {

using geom::Vec3;
using slam::Quat;

/// Governor parameters.
struct DriftParams {
    /// Hold unexplained tracker translation (PIMAX_SLAM_DRIFT_HOLD=0 / the app switch off = the old behaviour).
    bool on = true;
    /// Neck pivot relative to the head origin in the IMU frame (OpenXR axes: x right, y up, z back), m.
    Vec3 neck_in_head{0.0, -0.10, 0.08};
    /// Window (ns) of the tracker neck speed.
    std::int64_t window_ns = 250'000'000;
    /// Window (ns) of the motion evidence.
    std::int64_t act_window_ns = 1'000'000'000;
    /// Evidence level (m/s^2) of a worn head at rest (and of the IMU's own noise) ...
    double noise = 0.10;
    /// ... and the speed (m/s) allowed per m/s^2 above it.
    double gain_s = 1.0;
    /// Always allowed (m/s).
    double floor = 0.0;
    /// The head counts as turning while the bias-corrected gyro passed this (rad/s) within turn_hold_ns: the
    /// world-frame evidence is unreliable then, so only the rotation-free magnitude counts: the standard deviation of
    /// |neck specific force| over act_window_ns above turn_noise allows gain_s per m/s^2.
    double turn_rate;
    /// See turn_rate.
    std::int64_t turn_hold_ns = 500'000'000;
    /// See turn_rate.
    double turn_noise = 0.30;
    /// Unexplained speed held completely up to this (m/s) ...
    double band_lo = 0.05;
    /// ... and not at all from this one (m/s).
    double band_hi = 0.15;
    /// The same band while turning (m/s): Basalt slid in bursts of 0.1-0.2 m/s during the fast turns.
    double turn_band_lo = 0.10;
    /// See turn_band_lo.
    double turn_band_hi = 0.25;
    /// A log report at most this often (ns) ...
    std::int64_t report_every_ns = 10'000'000'000;
    /// ... and only when at least this much (m) was held since the last one.
    double report_min_m = 0.01;
    /// PiBridge (not in the Rust code; PIMAX_SLAM_DRIFT_HOLD=2): hold only while the head is quiet, i.e. every axis of
    /// the gyro below quiet_gyro_std (rad/s) and of the accelerometer below quiet_accel_std (m/s^2) over the last
    /// second (a worn head standing or sitting still: about 1 deg/s and 0.07 m/s^2; a step or a lean is louder).
    /// 0 = always (Rust).
    double quiet_gyro_std = 0.0;
    /// See quiet_gyro_std.
    double quiet_accel_std = 0.0;

    DriftParams();
    bool operator==(const DriftParams&) const = default;
};

/// What the governor held (for the log, the app and tests).
struct DriftStats {
    /// Tracker poses looked at.
    std::uint64_t poses = 0;
    /// Poses where something was held.
    std::uint64_t held_poses = 0;
    /// Path length held (m): how far the output would have slid.
    double held_m = 0;
    /// Net displacement held (world, m).
    Vec3 held_vec{};
    bool operator==(const DriftStats&) const = default;
};

/// One log report: held m metres in the last secs seconds (total_m this session).
struct DriftReport {
    /// Held since the last report (m).
    double m = 0;
    /// Seconds covered.
    double secs = 0;
    /// Held this session (m).
    double total_m = 0;
    bool operator==(const DriftReport&) const = default;
};

/// Running per-axis variance of a window of vectors.
struct VarWin {
    std::deque<std::pair<std::int64_t, Vec3>> q;
    Vec3 s{};
    Vec3 s2{};
    std::uint64_t pushes = 0;

    void push(std::int64_t ts, const Vec3& v, std::int64_t window_ns);
    /// Root of the summed per-axis variances; 0 until 100 samples.
    double spread() const;
    void clear() { *this = VarWin{}; }
};

/// The governor state (owned by headpose::HeadTracker).
class DriftGov {
public:
    /// New governor; head_in_imu = the head origin in the IMU frame.
    DriftGov(DriftParams p, const Vec3& head_in_imu);

    /// Motion evidence L (m/s^2, see the file comment); 0 until the attitude is known and the window is filled.
    double evidence() const { return acc_.spread(); }
    /// Standard deviation of |neck specific force| over the window (m/s^2): rotation does not change it.
    double shake() const { return mag_.spread(); }
    /// The head turned within turn_hold_ns of the newest IMU sample.
    bool turning() const;
    /// Neck speed (m/s) the evidence allows now.
    double allowed() const;
    /// One IMU sample (raw); bias = the gyro bias in use (rad/s).
    void push_imu(const headpose::ImuSample& s, const Vec3& bias);
    /// Forget the motion state (tracker reset, loss, desk still hold): the next pose starts a new comparison.
    void reset() { hist_.clear(); }
    /// Speed (m/s) of tracker neck motion u the governor removes, given the allowed speed.
    double removed_speed(double u, double allowed) const;
    /// A tracker pose while tracking (fused = the predictor's newest state, q_now = its attitude carried to the newest
    /// IMU sample, w_q = the room yaw). Returns the world shift to apply to the room alignment, if any: the fraction
    /// removed / u of this pose's neck step, with u the neck speed over the last window_ns.
    std::optional<Vec3> on_pose(const headpose::BodyState& fused, const Quat& q_now, const Quat& w_q);
    /// A report for the log when enough was held since the last one (at most every report_every_ns).
    std::optional<DriftReport> take_report(std::int64_t now_ns);

    /// Parameters.
    DriftParams p;
    /// Counters.
    DriftStats stats;
    /// Last pose: (tracker neck speed, allowed speed, removed speed), m/s.
    std::tuple<double, double, double> last{0.0, 0.0, 0.0};
    /// Last pose's neck step (m) and the part held (m).
    std::pair<double, double> last_step{0.0, 0.0};
    /// Held since the last report (tests set it).
    double report_m = 0.0;

private:
    Vec3 n_imu_;
    std::optional<Quat> att_;
    std::optional<std::int64_t> last_ts_;
    std::deque<std::pair<std::int64_t, Vec3>> gyro_hist_;
    VarWin acc_;
    VarWin mag_;
    std::optional<std::int64_t> last_turn_ns_;
    // Recent poses: (time, tracker neck), oldest first.
    std::deque<std::pair<std::int64_t, Vec3>> hist_;
    std::optional<std::int64_t> report_from_;
};

/// The log line of a report (parsed by the app, omarchyvr-gui-core monlog: keep the words).
std::string report_line(const DriftReport& r);

} // namespace pibridge::cam::drift
