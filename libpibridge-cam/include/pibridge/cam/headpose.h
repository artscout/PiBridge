// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/headpose.rs (MIT OR Apache-2.0).
//
// Head pose output for 6DoF tracking (phase 2): IMU-rate prediction from the newest tracker (VIO) pose, blending of
// corrections so the room never jumps, and the supervisor's fallback states, all in one place.
//
//   IMU 1 kHz ---------------------------> Predictor (ring buffer, bias-corrected gyro, ZOH integration)
//   VIO 25 Hz (pos, q, vel; y-up) -------> Predictor base states  +  Supervisor (acquire / lose / reset / re-snap)
//   query(t, 3DoF relation) -> eval(current configuration, t)  + correction offset (decays, rate limited)
//
// * Rotation stays IMU-fast: between tracker poses the orientation is the last VIO orientation integrated with the
//   gyro up to the newest IMU sample (then constant angular velocity to the query time).
// * Position = last VIO position + VIO velocity + IMU acceleration (gravity removed) up to the newest IMU sample,
//   then constant velocity.
// * No jumps: whenever what produces the output changes (a new VIO pose corrects the dead-reckoned one, the
//   supervisor acquires / loses / re-snaps / recentres) the difference at the query time goes into a correction
//   offset (position in world metres, rotation as a world-frame rotation vector about the head) so the output is
//   continuous. The offset then decays to zero with a time constant (tau_*), never faster than a rate limit that
//   grows with head speed (corrections hide in motion, HoloLens style). Huge differences (snap_*) are applied at
//   once: sliding a room for metres is worse than one cut.
// * The supervisor (slam::Supervisor) keeps deciding when the tracker is trusted; with re-snap on it restores the old
//   room alignment when tracking comes back in the same tracker world.
#pragma once

#include "pibridge/cam/drift.h"
#include "pibridge/cam/geom.h"
#include "pibridge/cam/slam.h"

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::cam::headpose {

using geom::Vec3;
using slam::Quat;

/// PIBRIDGE_HEAD_PARITY=1: the output layer behaves exactly as omarchyvr's Rust at the pinned commit (the Rust twins
/// run this way); unset or 0: PiBridge's measured improvements are on by default
/// (docs/research/head-tracking-improvements.md). Read once per call (cheap; HeadParams::from_env and the C ABI's
/// create read it once per session).
bool parity_mode();

/// Standard gravity (m/s^2).
inline constexpr double GRAVITY = 9.80665;
/// Gravity in the y-up world.
inline constexpr Vec3 G_WORLD{0.0, -GRAVITY, 0.0};

/// Camera image brightness handed from the auto-exposure loop to the head tracker (both live in this library inside
/// the same Monado process; no driver change needed). A leaf: a stale or missing value means "unknown". The
/// auto-exposure C ABI (monado-abi ae_capi.cpp, pimax_cam_ae_update) publishes the darker pair's mean of every
/// analysed frame; with AE off nothing does and latest() is nullopt.
namespace scene {
/// Publish the darker pair's trimmed mean (0..255).
void publish(double mean);
/// The newest mean if it is at most max_age_ms old.
std::optional<double> latest(std::uint64_t max_age_ms);
} // namespace scene

/// Auto-exposure mean (0..255, target 64) below which the image counts as getting dark ...
inline constexpr double SCENE_DARK_FROM = 32.0;
/// ... and at which as fully dark (session 2026-10-16 18:18 ended at means 9-15 with exposure and gain capped).
inline constexpr double SCENE_DARK_FULL = 12.0;

/// Relation flag bits (same values as Monado's xrt_space_relation_flags).
namespace flags {
inline constexpr std::uint32_t ORIENTATION_VALID = 1;
inline constexpr std::uint32_t POSITION_VALID = 2;
inline constexpr std::uint32_t LINEAR_VELOCITY_VALID = 4;
inline constexpr std::uint32_t ANGULAR_VELOCITY_VALID = 8;
inline constexpr std::uint32_t ORIENTATION_TRACKED = 16;
inline constexpr std::uint32_t POSITION_TRACKED = 32;
inline constexpr std::uint32_t ALL = 63;
} // namespace flags

/// exp map: rotation vector (rad) -> unit quaternion.
Quat qexp(const Vec3& w);
/// log map: unit quaternion -> rotation vector (rad), shortest way.
Vec3 qlog(const Quat& q);

/// One IMU sample in the tracker's IMU frame (body = OpenXR head axes), raw (not bias corrected).
struct ImuSample {
    /// Monotonic ns (same clock as the tracker).
    std::int64_t ts_ns = 0;
    /// Specific force, m/s^2.
    Vec3 accel{};
    /// Angular rate, rad/s.
    Vec3 gyro{};
    bool operator==(const ImuSample&) const = default;
};

/// IMU (body) state in a gravity-aligned y-up frame.
struct BodyState {
    /// Monotonic ns.
    std::int64_t ts_ns = 0;
    /// Position.
    Vec3 p{};
    /// Orientation (body -> frame).
    Quat q{};
    /// Velocity.
    Vec3 v{};
    bool operator==(const BodyState&) const = default;
};

/// Predictor parameters.
struct PredictorParams {
    /// IMU samples kept (1 kHz: 4 s).
    std::size_t imu_cap = 4000;
    /// Tracker poses kept.
    std::size_t vio_cap = 64;
    /// Gyro bias low-pass time constant (s); 0 = no bias estimation.
    double bias_tau_s = 10.0;
    /// Largest accepted bias sample (rad/s).
    double bias_max = 0.15;
    /// Use the accelerometer between tracker poses (else constant velocity).
    bool use_accel = true;
    /// Fused position (0 = off: every tracker pose is the new base as it is). Each tracker pose then only pulls the
    /// IMU-propagated position by this fraction of the difference (fuse_alpha_y vertically) and the velocity by
    /// fuse_beta x difference / interval plus fuse_vel_w x (tracker velocity - propagated): an alpha-beta filter
    /// driven by the accelerometer.
    double fuse_alpha = 0.0;
    /// See fuse_alpha.
    double fuse_alpha_y = 0.0;
    /// See fuse_alpha.
    double fuse_beta = 0.0;
    /// See fuse_alpha.
    double fuse_vel_w = 0.0;
    /// Differences above this (m) reset the fused state to the tracker pose.
    double fuse_reset_m = 0.3;
    /// Speed-adaptive: the tracker velocity weight rises from fuse_vel_w at rest to 1 at this head speed (m/s; 0 =
    /// fixed).
    double fuse_fast_speed = 0.0;
    /// Automatic fusion weights (PIMAX_FUSE_AUTO): the tracker's velocity noise is estimated online (running RMS of
    /// tracker velocity - IMU-propagated velocity) and, with a dark camera image (Predictor::scene_dark), turns a
    /// noisy tracker down where its noise shows, at rest and in slow motion: velocity weight fuse_vel_w -> 0.2 x it,
    /// alphas and beta -> half. Fast motion keeps following the tracker. A clean tracker keeps the configured values.
    bool fuse_auto = false;
    /// Velocity residual RMS (m/s) at and below which the tracker counts as clean ...
    double noise_lo = 0.05;
    /// ... and at and above which as fully noisy.
    double noise_hi = 0.15;
    /// Time constant of the running residual estimate (s).
    double noise_tau_s = 2.0;
    /// PiBridge (not in the Rust code; PIMAX_SLAM_PREDICT_ACCEL): past the newest IMU sample the rotation is
    /// extrapolated with the angular velocity of the last 4 ms plus this fraction of the angular acceleration
    /// measured over the last 24 ms (0 = the last sample's rate held constant, as in Rust). Cuts the display-time
    /// rotation error in fast turns and shakes.
    double ang_accel_k = 0.0;
    /// The angular acceleration term counts for at most this long past the newest IMU sample (s).
    double ang_accel_max_s = 0.05;
    bool operator==(const PredictorParams&) const = default;
};

/// Window statistics of the buffered IMU (per-axis means and standard deviations).
struct ImuWindow {
    /// Samples.
    std::size_t n = 0;
    /// Mean gyro (rad/s, raw).
    Vec3 gyro_mean{};
    /// Gyro standard deviation per axis (rad/s).
    Vec3 gyro_std{};
    /// Mean accelerometer (m/s^2).
    Vec3 accel_mean{};
    /// Accelerometer standard deviation per axis (m/s^2).
    Vec3 accel_std{};
    /// Largest bias-corrected angular rate in the window (rad/s).
    double gyro_max = 0;
    bool operator==(const ImuWindow&) const = default;
};

/// IMU dead reckoning from the tracker's poses.
class Predictor {
public:
    explicit Predictor(PredictorParams p);

    /// Tracker noise level 0 (clean: the configured weights) .. 1 (noisy: lowest weights), from the velocity residual
    /// and the image darkness; 0 with fuse_auto off.
    double noise_level() const;
    /// Effective fusion weights [alpha, alpha_y, beta, vel_w, fast-motion vel_w] at the current noise level and head
    /// speed fraction fast (0 = at rest .. 1 = at fuse_fast_speed).
    std::array<double, 5> fuse_weights(double fast) const;
    /// Statistics of the newest window_ns of IMU samples; nullopt until the window is covered.
    std::optional<ImuWindow> imu_window(std::int64_t window_ns) const;
    /// Add an IMU sample (older-or-equal timestamps are dropped).
    void push_imu(const ImuSample& s);
    /// True when the newest window_ns of IMU samples look like a headset lying still: per-axis standard deviation
    /// below gyro_std (rad/s) and accel_std (m/s^2).
    bool imu_still(std::int64_t window_ns, double gyro_std, double accel_std) const;
    /// Newest IMU timestamp.
    std::optional<std::int64_t> last_imu_ns() const;
    /// Drop the tracker poses (tracker reset: new world).
    void clear_vio();
    /// Add a tracker pose; false if not newer than the last one. Updates the gyro bias estimate from the rotation
    /// between this pose and the previous one (when learn_bias).
    bool push_vio(BodyState st, bool learn_bias);
    /// Newest tracker pose.
    std::optional<BodyState> latest() const;
    /// Newest tracker pose at or before t (the oldest one for earlier times).
    std::optional<BodyState> base_at(std::int64_t t) const;
    /// Bias-corrected gyro rotation from t0 to t1.
    std::optional<Quat> gyro_delta(std::int64_t t0, std::int64_t t1) const;
    /// Propagate base to t (>= base time; earlier times return base) with the IMU: zero-order hold (sample i covers
    /// the interval ending at its timestamp), bias-corrected gyro, gravity-removed acceleration. Past the newest IMU
    /// sample: constant angular velocity and constant velocity. Returns the state and the body angular velocity used
    /// last.
    std::pair<BodyState, Vec3> propagate(const BodyState& base, std::int64_t t) const;
    /// State at t from the newest tracker pose at or before t.
    std::optional<std::pair<BodyState, Vec3>> state_at(std::int64_t t) const;

    /// Parameters.
    PredictorParams p;
    /// Gyro bias estimate (rad/s, body frame), from tracker vs gyro rotation.
    Vec3 gyro_bias{};
    /// Bias samples taken.
    std::uint64_t bias_samples = 0;
    /// Take the next tracker pose as it is even with fusion on (headset lying still: the hold owns it).
    bool fuse_hold = false;
    /// Headset lying still: no accelerometer dead reckoning (its bias would only add drift between poses).
    bool lying = false;
    /// Running mean square of (tracker velocity - IMU-propagated velocity), m^2/s^2 (PredictorParams::fuse_auto).
    double vel_noise_ms = 0.0;
    /// Residual samples taken.
    std::uint64_t vel_noise_n = 0;
    /// Camera image darkness 0 (fine) .. 1 (too dark to track well), set by the caller (auto-exposure mean).
    double scene_dark = 0.0;

private:
    void learn_vel_noise(const BodyState& raw);
    std::optional<Quat> gyro_delta_raw(std::int64_t t0, std::int64_t t1) const;

    std::deque<ImuSample> imu_;
    std::deque<BodyState> vio_;
    // The tracker's own newest state (before fusion), for the velocity noise estimate.
    std::optional<BodyState> last_raw_;
};

/// A relation (pose + velocities), y-up world, flags as Monado.
struct Rel {
    Vec3 pos{};
    Quat q = slam::QUAT_ID;
    Vec3 lin_vel{};
    /// Angular velocity (world frame).
    Vec3 ang_vel{};
    /// flags bits.
    std::uint32_t flags = 0;
    bool operator==(const Rel&) const = default;
};

/// Output smoothing and re-snap parameters (defaults: the automatic configuration; from_env() applies the switches).
struct HeadParams {
    /// Blend corrections (false = raw prediction, phase-1 behaviour).
    bool smooth = true;
    /// Position correction time constant (s).
    double tau_pos_s = 0.15;
    /// Rotation correction time constant (s).
    double tau_rot_s = 0.10;
    /// Position correction rate limit at rest (m/s) ...
    double v_min = 0.10;
    /// ... plus this fraction of the head speed.
    double k_v = 0.5;
    /// Rotation correction rate limit at rest (rad/s) ...
    double w_min;
    /// ... plus this fraction of the head angular speed.
    double k_w = 0.3;
    /// Larger position differences are applied at once (m).
    double snap_pos = 1.5;
    /// Larger rotation differences are applied at once (rad).
    double snap_rot;
    /// Restore the old room alignment when tracking returns in the same tracker world.
    bool resnap = true;
    /// Re-snap only if the room yaw agrees with the 3DoF heading within this (deg).
    double resnap_yaw_deg = 3.0;
    /// ... and the head is within max(resnap_min_m, resnap_speed * lost time) of where it was held.
    double resnap_speed = 1.5;
    /// See resnap_speed.
    double resnap_min_m = 0.5;
    /// Learn the gyro bias from the tracker while tracking.
    bool learn_bias = true;
    /// Headset lying still: hold the position, reset a drifting tracker (slam::StillParams).
    slam::StillParams still{};
    /// Window (ns) over which the still state is entered: the headset has to be desk-quiet this long. What tells a
    /// worn head from a desk is the level (worn rest >= 0.3 deg/s gyro std in every 0.5 s window; on the desk
    /// 0.05-0.06 deg/s), not the length: a landmark-free tracker ran away at > 1 m/s within 0.5 s of a put-down.
    std::int64_t still_enter_window_ns = 500'000'000;
    /// Window (ns) the exit test looks at (short: a pickup ends the still state at once) ...
    std::int64_t still_window_ns = 500'000'000;
    /// ... per-axis standard deviation limits (rad/s, m/s^2) to enter the still state (over still_enter_window_ns) ...
    double still_gyro_std = 0.0035;
    /// See still_gyro_std.
    double still_accel_std = 0.03;
    /// ... and the larger ones that end it (hysteresis). Worn walks never go below 0.027 rad/s.
    double still_exit_gyro_std = 0.012;
    /// See still_exit_gyro_std.
    double still_exit_accel_std = 0.15;
    /// ... and the mean gyro must be within this (rad/s) of the learned gyro bias (a slow steady head turn is flat but
    /// not still; LESSONS row 36). Used once the bias has been learned for a while; 0 = off.
    double still_bias_tol = 0.005;
    /// Exact pose freeze (PIMAX_SLAM_FREEZE, 0 = off): while the headset lies still and the bias-corrected gyro stays
    /// under freeze_gyro_on, the output pose (position AND orientation) is one fixed value with zero velocities. It
    /// ends when the gyro passes freeze_gyro_off (hysteresis) or the still state ends, and the pose then slides on
    /// through the normal correction (no jump).
    bool freeze = true;
    /// Freeze engages when the largest bias-corrected gyro rate in freeze_window_ns is under this (rad/s) ...
    double freeze_gyro_on = 0.006;
    /// ... and ends when it is over this one (rad/s).
    double freeze_gyro_off = 0.02;
    /// Window (ns) of the freeze test.
    std::int64_t freeze_window_ns = 200'000'000;
    /// The frozen pose re-latches to the live one when it differs by more than this angle (rad) or distance (m).
    double freeze_deadband_rad = 0.04 * 3.14159265358979323846264338327950288 / 180.0;
    /// See freeze_deadband_rad.
    double freeze_deadband_m = 0.001;
    /// Gravity tilt correction (PIMAX_HEAD_TILT): pitch and roll of the output are pulled slowly towards the
    /// accelerometer's gravity (yaw and position stay the tracker's), with this time constant (s; 0 = off) ...
    double tilt_tau_s = 1.5;
    /// ... never faster than this (rad/s) ...
    double tilt_rate;
    /// ... and only while |accel| is within this of g (m/s^2) ...
    double tilt_g_tol = 0.3;
    /// ... and the head turns slower than this (rad/s).
    double tilt_gyro_max = 0.15;
    /// Largest tilt correction (rad); a larger measured difference is not gravity (skipped).
    double tilt_max;
    /// Corrections whose step reaches this (m) are counted and queued for the log (HeadTracker::take_big).
    double big_corr_m = 0.02;
    /// Fused position: [alpha, alpha vertical, beta, tracker velocity weight, fast speed] (PredictorParams::
    /// fuse_alpha); alpha 0 = off. These are the values for a clean tracker ...
    std::array<double, 5> fuse{0.15, 0.075, 0.06, 0.5, 0.5};
    /// ... lowered automatically for a noisy or blind one (PredictorParams::fuse_auto).
    bool fuse_auto = true;
    /// Position corrections decay with this time constant (s) while the head is at rest, going over to tau_pos_s at
    /// rest_speed (m/s): a correction slides slowly where it would show and is hidden in motion (0 = always
    /// tau_pos_s).
    double tau_rest_s = 0.4;
    /// See tau_rest_s.
    double rest_speed = 0.3;
    /// Drift governor: tracker translation the accelerometer cannot explain is held (drift.h).
    drift::DriftParams drift{};
    /// PiBridge: angular acceleration in the rotation prediction (PredictorParams::ang_accel_k; 0 = Rust).
    double predict_accel = 0.0;

    // PiBridge additions (not in the Rust code). HeadParams() keeps the Rust defaults (the twins of the Rust tests use
    // it); from_env() turns them on unless PIBRIDGE_HEAD_PARITY=1 (docs/research/head-tracking-improvements.md).
    /// Start the learned gyro bias here instead of at zero (rad/s, body frame): the driver's cached rest bias, the same
    /// value its 3DoF filter and Basalt start from (PIMAX_SLAM_SEED_BIAS). nullopt = start at zero (Rust).
    std::optional<Vec3> seed_gyro_bias;
    /// Accelerometer bias removed from every IMU sample before the output layer uses it (m/s^2, body frame): the
    /// calibration Basalt subtracts (calib_accel_bias of the SLAM_CONFIG calibration, else the factory aBias;
    /// PIMAX_SLAM_ACCEL_BIAS). Zero = raw samples (Rust): the Crystal's 0.34 m/s^2 then integrates into a steady
    /// ~10 mm lag of the output behind the tracker and a 2 mm sawtooth at every tracker pose.
    Vec3 accel_bias{};
    /// Where the two values above came from (the describe line).
    std::string bias_source;

    HeadParams();
    /// Defaults with PIMAX_SLAM_SMOOTH (0/1), PIMAX_SLAM_TAU_MS (position; rotation = 2/3 of it), PIMAX_SLAM_RESNAP
    /// (0/1), PIMAX_SLAM_STILL (0 = no still hold/reset, else the reset distance in cm), PIMAX_SLAM_STILL_REZERO (0 =
    /// the old drop to 3DoF on a still reset, for A/B only), PIMAX_SLAM_FUSE (0 = off, or
    /// alpha,alpha_y,beta,vel_w,fast_speed), PIMAX_SLAM_TAU_REST_MS (0 = off), PIMAX_SLAM_BIG_CORR_MM (log
    /// threshold), PIMAX_FUSE_AUTO (0 = fixed fusion weights), PIMAX_HEAD_TILT (0 = no gravity tilt correction, else
    /// its time constant in ms), PIMAX_SLAM_FREEZE (0 = no exact freeze), PIMAX_SLAM_DRIFT_HOLD (0 = the tracker's
    /// unexplained translation reaches the output, the app switch "Hold the head when it does not move" off)
    /// overrides; the defaults are automatic.
    static HeadParams from_env();
    /// One log line naming the automatic output stages (the build-marker strings PIMAX_HEAD_TILT and PIMAX_FUSE_AUTO
    /// are in it).
    std::string describe() const;
    /// The PiBridge part of describe() ("; pibridge: ..."; empty in parity mode with no PiBridge switch set).
    std::string pibridge_describe() const;
    bool operator==(const HeadParams&) const = default;
};

/// One tracker pose as the driver hands it over (y-up tracker world, IMU frame).
struct VioSample {
    /// Pose time (monotonic ns).
    std::int64_t ts_ns = 0;
    /// IMU position.
    Vec3 pos{};
    /// IMU orientation.
    Quat q{};
    /// IMU velocity.
    Vec3 vel{};
    /// 3DoF orientation of the IMU at ts_ns (for the supervisor).
    Quat q3dof{};
    bool operator==(const VioSample&) const = default;
};

/// Correction statistics (for logs and tests).
struct CorrStats {
    /// Corrections blended in.
    std::uint64_t blended = 0;
    /// Corrections applied at once (too large).
    std::uint64_t snapped = 0;
    /// Largest blended position correction (m).
    double max_pos = 0;
    /// Largest blended rotation correction (rad).
    double max_rot = 0;
    /// Corrections whose offset reached HeadParams::big_corr_m, and those of them mostly vertical.
    std::uint64_t big = 0;
    /// See big.
    std::uint64_t big_vertical = 0;
    bool operator==(const CorrStats&) const = default;
};

/// One correction (a new tracker pose or a supervisor change moved what the output would be).
struct CorrEvent {
    /// Host time (ns).
    std::int64_t ts_ns = 0;
    /// This change alone: old output - new uncorrected output, minus the offset that was still pending (m, world).
    Vec3 step{};
    /// The whole offset after it (what the output has to slide off, m, world; y = vertical).
    Vec3 offset{};
    /// Rotation offset after it (rad).
    double rot = 0;
    /// Head speed (m/s) and angular speed (rad/s) at that moment.
    double speed = 0;
    /// See speed.
    double ang_speed = 0;
    /// Only the tracker pose changed (no acquire / loss / re-snap / recenter / still hold).
    bool pose_only = false;
    bool operator==(const CorrEvent&) const = default;
};

/// Supervisor + predictor + correction blending (see the file comment).
class HeadTracker {
public:
    /// New tracker; head_in_imu = head origin in the IMU frame (m).
    HeadTracker(slam::SupervisorParams sp, HeadParams p, Vec3 head_in_imu);

    /// Latest supervisor output.
    const slam::TrackOutput& output() const { return out_; }
    /// Current correction offset (position m, rotation vector rad).
    std::pair<Vec3, Vec3> correction() const { return {corr_p_, corr_r_}; }
    /// Feed one IMU sample.
    void push_imu(const ImuSample& s);
    /// Supervisor update with the tracker's newest pose (or nullopt); repeated timestamps are ignored by both.
    slam::TrackOutput update(std::int64_t now_ns, std::optional<VioSample> s);
    /// The headset counted as lying still at the last update (desk-quiet; never a worn head).
    bool is_still() const { return still_; }
    /// Current gravity tilt correction (world-frame rotation applied to the tracked output).
    Quat tilt() const { return tilt_; }
    /// Camera image brightness (auto-exposure trimmed mean 0..255 of the darker pair; nullopt = unknown): a dark image
    /// lowers the fusion weights like a noisy tracker velocity does.
    void set_scene_mean(std::optional<double> mean);
    /// Head pose at at_ns. rel3dof = the 3DoF relation for the same time (used while not tracking); now_ns = host time
    /// (drives the correction decay).
    Rel head(std::int64_t now_ns, std::int64_t at_ns, const Rel& rel3dof);
    /// IMU noise regime now (still or moving, with hysteresis).
    slam::NoiseRegime noise_regime() const { return noise_sel_.regime(); }
    /// True while the exact pose freeze holds the output.
    bool is_frozen() const { return freeze_on_; }
    /// Oldest not yet taken correction step of at least HeadParams::big_corr_m (the last 64 are kept).
    std::optional<CorrEvent> take_big();
    /// Head position in the world for a tracker pose with the current alignment (no correction).
    Vec3 head_world(const Vec3& pos, const Quat& q) const { return sup.head_world(pos, q); }
    /// Yaw (deg) of the current room alignment.
    double room_yaw_deg() const;

    /// The supervisor (state machine and room alignment).
    slam::Supervisor sup;
    /// The predictor.
    Predictor pred;
    /// Parameters.
    HeadParams p;
    /// Head queries answered with the frozen pose.
    std::uint64_t freeze_frames = 0;
    /// Tilt correction updates applied / skipped by the gates.
    std::uint64_t tilt_updates = 0;
    /// See tilt_updates.
    std::uint64_t tilt_skipped = 0;
    /// Correction counters.
    CorrStats corr{};
    /// Every correction (offline analysis; nullopt = off).
    std::optional<std::vector<CorrEvent>> corr_log;
    /// Drift governor (tracker translation without accelerometer evidence is held).
    drift::DriftGov drift;

private:
    // What produces the output: the supervisor's decision plus the tracker pose the prediction starts from.
    struct Config {
        slam::TrackState state;
        Vec3 w_t;
        Quat w_q;
        Quat fix3dof;
        Vec3 held;
        std::optional<BodyState> base;
        bool operator==(const Config&) const = default;
    };

    bool still_test() const;
    void update_tilt(const VioSample& v);
    Config config() const;
    Rel eval(const Config& c, std::int64_t t, const Rel& rel3dof) const;
    void decay(std::int64_t now_ns);
    void freeze_step(Rel& raw);

    Vec3 head_in_imu_;
    slam::TrackOutput out_;
    std::optional<Config> emitted_;
    Vec3 corr_p_{};
    Vec3 corr_r_{};
    std::optional<std::int64_t> corr_ns_;
    double speed_ = 0.0;
    double ang_speed_ = 0.0;
    bool still_ = false;
    // Exact freeze engaged, and the pose it holds.
    bool freeze_on_ = false;
    std::optional<Rel> frozen_;
    slam::NoiseSelector noise_sel_{};
    // Gravity tilt correction (world frame, horizontal axis: pitch/roll only), applied to the tracked output.
    Quat tilt_ = slam::QUAT_ID;
    std::optional<std::int64_t> tilt_ns_;
    bool tilt_reset_ = false;
    // Recent raw tracker head positions (pose time, tracker world) for the still anchor.
    std::deque<std::pair<std::int64_t, Vec3>> head_hist_;
    std::deque<CorrEvent> big_q_;

    friend struct HeadTrackerTestAccess;
};

} // namespace pibridge::cam::headpose
