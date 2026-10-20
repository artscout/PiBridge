// SPDX-License-Identifier: MIT
//
// pibridge-slam headreplay: the head pose output layer (libpibridge-cam headpose, what Monado calls through
// pimax_cam_track_*) replayed on a recorded IMU stream and a recorded tracker trajectory, with the numbers that tell
// whether a change to the output layer makes head tracking better (docs/research/head-tracking-improvements.md).
//
// The replay is the one of monado-abi's difftest (`slam_difftest replay`): every IMU sample is pushed, each tracker
// pose is handed over `latency` after its frame time with a causal velocity, a Mahony 3DoF filter gives the 3DoF
// orientation, tracker resets the supervisor asks for are emulated by rebasing the rest of the trajectory, and the
// head pose is queried at 90 Hz for the display time `now + predict`. The output layer reads its switches from the
// environment exactly as in Monado (headpose::HeadParams::from_env), so every PIMAX_SLAM_* / PIBRIDGE_HEAD_* switch
// applies.
//
// No ground truth: the reference is the tracker itself, looked at afterwards (non-causal: positions by cubic Hermite
// between tracker poses, orientations by the gyro between them with the end mismatch spread linearly), mapped into
// the output world with the alignment the output layer used at that moment.
#pragma once

#include "pibridge/cam/geom.h"

#include <array>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::slamtool {

struct HeadReplayOpts {
    /// EuRoC root (holding mav0) or a recording folder (its newest euroc_* child).
    std::string dataset;
    /// Tracker trajectory (Basalt tracking.csv: ts, p, qw qx qy qz; z-up tracker world).
    std::string traj;
    /// The recorder's steps.json (protocol steps with times on the dataset clock); empty = none.
    std::string steps;
    /// Basalt calibration JSON (calib_accel_bias, calib_gyro_bias): the tilt reference and the 3DoF gyro bias.
    std::string calib;
    /// Tracker latency: a pose is handed over this long after its frame time (ms).
    double latency_ms = 25.0;
    /// Display prediction: each query asks for the pose at now + this (ms).
    double predict_ms = 20.0;
    /// Query rate (Hz).
    double query_hz = 90.0;
    /// Tracker velocity: false = causal second-order backward difference (as the difftest), true = central
    /// difference (closer to a VIO state velocity, which is filtered).
    bool central_vel = false;
    /// Head origin in the IMU frame; nullopt = slam::default_head_in_imu of the factory calibration found by the
    /// default search.
    std::optional<cam::geom::Vec3> head;
    /// Write one line per query here (CSV); empty = no file.
    std::string csv;
};

/// One tracker pose as a tracker hands it over (Basalt world: z up; IMU frame).
struct TrackerPose {
    std::int64_t ts = 0;
    cam::geom::Vec3 p{};
    /// x y z w.
    std::array<double, 4> q{0, 0, 0, 1};
    /// The tracker's own velocity (VIT pose data), when has_vel.
    cam::geom::Vec3 v{};
    bool has_vel = false;
};

/// Where the tracker poses come from when not from the recorded trajectory: an analysis harness can run a live tracker
/// (Basalt through VIT) in a closed loop, so the supervisor's resets reach a real tracker.
class PoseSource {
public:
    virtual ~PoseSource() = default;
    /// Every IMU sample, in time order, before the output layer sees it.
    virtual void on_imu(std::int64_t ts, const cam::geom::Vec3& accel, const cam::geom::Vec3& gyro) = 0;
    /// Poses whose hand-over time (frame time + latency) has come by now, oldest first.
    virtual std::vector<TrackerPose> poll(std::int64_t now) = 0;
    /// The supervisor asked for a tracker reset.
    virtual void reset(std::int64_t now) = 0;
    /// One line for the report (what the source is).
    virtual std::string describe() const = 0;
};

/// Run the replay and return the report (plain text, one "key value" block per section). src = nullptr: the recorded
/// trajectory o.traj (resets emulated by rebasing it); else the poses come from src (o.traj is not read).
std::expected<std::string, std::string> head_replay(const HeadReplayOpts& o, PoseSource* src = nullptr);

} // namespace pibridge::slamtool
