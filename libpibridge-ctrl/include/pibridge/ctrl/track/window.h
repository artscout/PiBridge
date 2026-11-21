// SPDX-License-Identifier: MIT
//
// PiBridge: the sliding-window pose estimator of one controller, after Pimax's LED engine (LedObjectPoseEstimator
// 2.11.2, reverse-engineered 2026-11-20; "bei_dou" proximity optimizer: a VINS-Mono-like window).
//
// * One state per LED frame: the controller's pose in the world (R: world_from_device, p), its velocity, and its gyro
//   and accelerometer biases (device frame). The window holds the newest 10 frames; the oldest is marginalized into a
//   linear prior.
// * Factors: the IMU preintegrated between consecutive frames (midpoint integration, 9x9 covariance, VINS form; the
//   bias Jacobians by re-integration), a bias random walk, every matched LED of every frame as a reprojection residual
//   (sigma 0.1 px, Huber at 3, i.e. 0.3 px), and, while the controller is still, keep-static / zero-velocity factors.
// * Solver: Gauss-Newton / Levenberg-Marquardt on the dense normal equations (at most 150 parameters), 5 iterations
//   per frame, as Pimax (DOGLEG, DENSE_SCHUR, 5 iterations). Eigen does the linear algebra.
//
// The world is the raw head world of the hint (y up, gravity 9.80665). The window knows nothing of blobs or matching:
// the tracker (wintrack.h) feeds it matched LEDs and IMU steps.
#pragma once

#include "pibridge/ctrl/track/solve.h"

#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <vector>

namespace pibridge::ctrl::track {

/// One IMU step in the device frame: the gyro's turn over dt (rotation vector, rad) and the mean specific force.
struct ImuStep {
    double dt = 0.0;
    cam::geom::Vec3 dtheta{0.0, 0.0, 0.0};
    cam::geom::Vec3 f{0.0, 0.0, 0.0};
};

/// One matched LED: its model position and the blob it landed on, in `cam`.
struct VisObs {
    const Cam* cam = nullptr;
    cam::geom::Vec3 p_led{};
    double u = 0.0;
    double v = 0.0;
};

/// The controller's state at a frame.
struct WinState {
    std::uint64_t t = 0;
    cam::geom::Mat3 r = cam::geom::IDENTITY; // world_from_device
    cam::geom::Vec3 p{0.0, 0.0, 0.0};
    cam::geom::Vec3 v{0.0, 0.0, 0.0};
    cam::geom::Vec3 bg{0.0, 0.0, 0.0};
    cam::geom::Vec3 ba{0.0, 0.0, 0.0};
};

struct WindowCfg {
    std::size_t max_frames = 10;
    /// IMU noise (Pimax: sigma_a 0.25, sigma_g 0.05; VINS form, per step), bias random walks per sqrt(s).
    double sigma_a = 0.25;
    double sigma_g = 0.05;
    double walk_ba = 3e-4;
    double walk_bg = 9.5e-5;
    /// Visual residual weight (1 / sigma_px) and Huber threshold on the weighted residual (3; 2 when fast).
    double vis_w = 10.0;
    double huber = 3.0;
    double huber_fast = 2.0;
    /// Bias prior weights (1/sigma) while there is no marginal prior. Pimax: 5e4 on both; PiBridge: the accelerometer
    /// bias is left free enough to absorb the hint's accelerometer scale / bias error (WT_BA_W, WT_BG_W to try).
    double bias_prior_w_g = 5e4;
    double bias_prior_w_a = 10.0;
    /// Keep-static (m, rad) and zero-velocity (m/s; strict) sigmas.
    double static_p = 0.002;
    double static_r = 2.618e-3;
    double zero_v = 0.025;
    double zero_v_strict = 0.01;
    /// The first frame's prior after a (re)start: position (m), orientation (rad), velocity (m/s).
    double init_p = 0.005;
    double init_r = 0.02;
    double init_v = 0.3;
    std::size_t iterations = 5;
};

class Window {
public:
    explicit Window(WindowCfg cfg = {});

    struct Preint;
    /// The linear prior left by marginalization, on the oldest frame: r(x) = r0 + J (x (-) x0) (J row-major, k x 15).
    struct Prior {
        std::size_t rows = 0;
        std::vector<double> J;
        std::vector<double> r0;
        WinState x0;
    };

    WindowCfg cfg;

    /// Start over from an acquisition: one frame with this state.
    void reset(const WinState& s);
    bool empty() const { return frames_.empty(); }
    std::size_t frames() const { return frames_.size(); }
    const WinState& newest() const { return frames_.back().x; }
    const WinState& oldest() const { return frames_.front().x; }

    /// The state propagated from the newest frame by `steps` (its biases removed), no covariance.
    WinState predict(std::span<const ImuStep> steps, std::uint64_t t) const;

    /// A new frame at t: `steps` cover (newest().t, t]; `obs` are its matched LEDs (any number, 0 allowed);
    /// `body_from_world` the head pose at t inverted. Optimizes the window and marginalizes beyond max_frames.
    void push(std::uint64_t t, const cam::geom::Pose& body_from_world, std::vector<VisObs> obs,
              std::vector<ImuStep> steps, bool still_strict, bool still_loose, bool fast);

    /// Diagnostics of the last push.
    double last_cost = 0.0;
    double last_vis_rms_px = 0.0; // over the newest frame's observations after the solve
    std::size_t last_iterations = 0;
    std::size_t last_accepted = 0;

    struct Frame {
        WinState x;
        cam::geom::Pose body_from_world;
        std::vector<VisObs> obs;
        std::vector<ImuStep> steps; // from the previous frame to this one
        bool still_strict = false;
        bool still_loose = false;
    };
    const std::deque<Frame>& frames_debug() const { return frames_; }
    /// Test hook: the IMU factor's raw residual [theta, v, p] between two states over `steps`, and the preintegrated
    /// covariance's diagonal (same order).
    std::pair<std::array<double, 9>, std::array<double, 9>> imu_residual_debug(const WinState& a, const WinState& b,
                                                                              std::span<const ImuStep> steps) const;

private:
    std::deque<Frame> frames_;
    std::optional<Prior> prior_; // on the oldest frame
    WinState init_;              // the acquisition's state: the first frame's prior until a marginal prior exists
    bool fast_ = false;

    void marginalize();
    friend struct WindowAccess;
};

} // namespace pibridge::ctrl::track
