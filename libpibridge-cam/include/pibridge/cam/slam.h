// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/slam.rs (MIT OR Apache-2.0).
//
// 6DoF head tracking support (visual-inertial SLAM, Monado + Basalt through VIT):
//
// * t_imu_cam: camera extrinsic in the IMU frame the driver feeds to the tracker. That frame is the factory
//   calibration's "body" frame (OpenXR head axes, x right, y up, z back), i.e. the raw IMU axes after the driver's
//   PIMAX_AXIS_MODE=1 mapping (x and z negated). Camera axes are OpenCV (x right, y down, z forward).
// * ImuNoise: IMU noise model. The factory file has IMUNoise all 0.0, so we use conservative defaults or a file
//   measured from a still recording (`omarchyvr-slam imu-noise`, ~/.config/omarchyvr/slam-imu.json).
// * basalt_calib_json: the calibration in Basalt's cereal JSON format (camera_type kb4), for offline runs
//   (basalt_vio, `monado-cli slambatch` with a SLAM_CONFIG) and for every recording.
// * Supervisor: decides when the tracker's poses are trustworthy (acquire after N consistent frames, lose on jumps /
//   rotation disagreement with the gyro / staleness), keeps the output continuous across losses (held position +
//   yaw-corrected 3DoF), and asks for a tracker reset after a long loss.
//
// Units: metres, nanoseconds, quaternions [x, y, z, w]. Every expression keeps the Rust evaluation order (the build
// uses -ffp-contract=off), so the results are bit-identical to pimax-cam's.
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/geom.h"

#include <array>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::cam::slam {

using geom::Mat3;
using geom::Pose;
using geom::Vec3;

/// Tracking-exposure frame rate of each camera (the stream is 50 fps, alternating tracking and LED exposure).
inline constexpr double TRACKING_HZ = 25.0;
/// IMU sample rate (500 reports/s with two samples each).
inline constexpr double IMU_HZ = 1000.0;

/// Quaternion [x, y, z, w].
using Quat = std::array<double, 4>;

/// Identity quaternion.
inline constexpr Quat QUAT_ID{0.0, 0.0, 0.0, 1.0};

/// Hamilton product a * b (apply b first).
Quat qmul(const Quat& a, const Quat& b);
/// Conjugate (inverse of a unit quaternion).
Quat qconj(const Quat& q);
/// Normalise (identity for a zero quaternion).
Quat qnorm(const Quat& q);
/// Rotate a vector.
Vec3 qrot(const Quat& q, const Vec3& v);
/// Rotation angle in degrees.
double qangle_deg(const Quat& q);
/// Rotation about +Y (up) by rad.
Quat qyaw(double rad);
/// Yaw (rotation about +Y) part of q: the angle that takes forward (-Z) to q's forward projected on the floor.
/// Gravity-aligned frames only differ by such a yaw.
double yaw_of(const Quat& q);
/// Matrix to quaternion.
Quat quat_of(const Mat3& m);
/// Quaternion to a row-major matrix.
Mat3 mat_of(const Quat& q);

/// imu_from_cam_i (= body_from_cam_i): maps camera-i OpenCV coordinates into the IMU frame the driver feeds.
std::optional<Pose> t_imu_cam(const DeviceCalib& cal, std::size_t i);

/// Pimax's two IMU noise regimes: the headset at rest or moving.
enum class NoiseRegime {
    /// At rest (the densities in ImuNoise's defaults).
    Still,
    /// In motion: the same densities scaled by MOVING_GYRO_RATIO / MOVING_ACCEL_RATIO.
    Moving,
};

/// Moving / stationary gyro noise ratio (Pimax Windows IMUNoise: 0.0225 / 0.001974).
inline constexpr double MOVING_GYRO_RATIO = 0.0225 / 0.001974;
/// Moving / stationary accelerometer noise ratio (1.296 / 0.081686).
inline constexpr double MOVING_ACCEL_RATIO = 1.296 / 0.081686;

/// IMU noise model in Basalt's terms (continuous-time densities: *_noise_std in unit/s/sqrt(Hz)-style units as Basalt
/// expects them, *_bias_std bias random walk).
///
/// Defaults: Pimax's own stationary densities (older Windows device_calibration.xml IMUNoise: gyro 0.001974, accel
/// 0.081686; the active file has zeros). The earlier datasheet-based 4e-4 / 4e-3 made Basalt trust a biased IMU over
/// vision (omarchyvr analysis/slam-drift.md). Bias random walks stay (larger ones made it worse).
struct ImuNoise {
    /// Accelerometer white noise, m/s^2/sqrt(Hz).
    Vec3 accel_noise_std{0.081686, 0.081686, 0.081686};
    /// Gyroscope white noise, rad/s/sqrt(Hz).
    Vec3 gyro_noise_std{0.001974, 0.001974, 0.001974};
    /// Accelerometer bias random walk, m/s^3/sqrt(Hz).
    Vec3 accel_bias_std{1.0e-3, 1.0e-3, 1.0e-3};
    /// Gyroscope bias random walk, rad/s^2/sqrt(Hz).
    Vec3 gyro_bias_std{1.0e-4, 1.0e-4, 1.0e-4};
    /// Sample rate, Hz.
    double rate_hz = IMU_HZ;
    /// Where the numbers came from ("defaults" or a file path).
    std::string source = "defaults";

    /// Default file search: $PIMAX_SLAM_IMU, then ~/.config/omarchyvr/slam-imu.json.
    static std::optional<std::string> default_path();
    /// Parse the JSON written by to_json (missing keys keep the defaults; a scalar applies to all three axes).
    static std::expected<ImuNoise, std::string> parse(std::string_view text);
    /// Load path, or the default file when path is nullopt. A value of nullopt = no file (use defaults).
    static std::expected<std::optional<ImuNoise>, std::string> load(const std::optional<std::string>& path);
    /// JSON (Basalt key names).
    std::string to_json(std::string_view comment) const;
    /// The noise model for a regime. Still is *this; Moving scales the white-noise densities by the Pimax ratios
    /// (so a measured still model from slam-imu.json keeps its own scale); bias walks are unchanged.
    ImuNoise for_regime(NoiseRegime r) const;

    bool operator==(const ImuNoise&) const = default;
};

/// Picks the NoiseRegime from the largest bias-corrected gyro rate of a short window, with hysteresis (Moving above
/// on, back to Still under off).
struct NoiseSelector {
    /// Enter Moving above this rate (rad/s).
    double on = 0.05;
    /// Return to Still under this rate (rad/s).
    double off = 0.02;

    /// Feed the newest window's largest bias-corrected gyro rate (rad/s); returns the regime.
    NoiseRegime update(double gyro_max);
    /// Current regime.
    NoiseRegime regime() const { return regime_; }

    bool operator==(const NoiseSelector&) const = default;

private:
    NoiseRegime regime_ = NoiseRegime::Still;
};

/// Which cameras feed the tracker and how.
struct SlamSetup {
    /// Factory camera ids in tracker order (cam0, cam1, ...).
    std::vector<std::size_t> cams{0, 1};
    /// Frame rate per camera.
    double cam_hz = TRACKING_HZ;
    /// Basalt cam_time_offset_ns (t_imu = t_cam + offset). The driver already shifts frame times by its own
    /// PIMAX_SLAM_CAM_OFFSET_US, so recordings carry 0 here.
    std::int64_t cam_time_offset_ns = 0;
    /// Gyro bias (rad/s, body frame) Basalt subtracts from every sample (calib_gyro_bias). Basalt starts its own
    /// bias estimate at zero; with the real ~0.85 deg/s and no landmarks yet the attitude tilts before vision can fix
    /// it and gravity leaks into position (analysis/slam-drift.md). Zero = none.
    Vec3 gyro_bias{};
    /// Accelerometer bias (m/s^2, body frame) Basalt subtracts from every sample (calib_accel_bias), normally the
    /// factory Stateinit aBias (factory_accel_bias). Without it the Crystal Super's 0.34 m/s^2 on the gravity axis
    /// reads as 3 % too much gravity. Zero = none.
    Vec3 accel_bias{};
    /// PiBridge (not in the Rust code): every camera position in T_imu_cam is multiplied by this (the stereo baseline and
    /// the camera-IMU lever arm together). Basalt's metric scale follows the baseline; on the Crystal Super Basalt's
    /// trajectory is about 0.88 of the IMU's motion (head-tracking-improvements.md, "Scale"), and 1.15 brings it to
    /// about 1. 1 = the factory geometry (default; the cause is not known yet).
    double cam_translation_scale = 1.0;

    bool operator==(const SlamSetup&) const = default;
};

/// The factory accelerometer bias (<SFConfig><Stateinit aBias>, m/s^2). The calibration's body frame is the frame the
/// driver feeds (the same frame as ombc/tbc), so it is used as is. nullopt when absent or implausible (any axis above
/// 1 m/s^2).
std::optional<Vec3> factory_accel_bias(const DeviceCalib& cal);

/// The driver's gyro bias cache ($XDG_CACHE_HOME/omarchyvr/gyro-bias, else ~/.cache/omarchyvr/gyro-bias).
std::optional<std::string> gyro_bias_cache_path();

/// Parse the driver's cache text: three numbers, raw IMU axes, deg/s (pimax_hmd.c bias_cache_save). Returns the bias
/// in the body frame (raw x and z negated, t_imu_cam's frame) in rad/s; nullopt when malformed or implausible (any
/// axis above 5 deg/s).
std::optional<Vec3> parse_gyro_bias_cache(std::string_view text);

/// The cached gyro bias (body frame, rad/s), if the driver has written one.
std::optional<Vec3> gyro_bias_cache();

/// PiBridge (not in the Rust code): the Basalt calibration file the live tracker really uses, i.e. the `cam-calib` line of
/// the toml Monado's $SLAM_CONFIG names (t_tracker_slam.cpp reads the same file; "Using sensor calibration provided by
/// the SLAM_CONFIG file"). nullopt when SLAM_CONFIG is unset, unreadable or has no cam-calib line. A relative path is
/// taken relative to the toml's folder.
std::optional<std::string> slam_config_calib_path();
/// Biases in a Basalt calibration JSON ("value0".calib_accel_bias[0..3], calib_gyro_bias[0..3]; body frame, m/s^2 and
/// rad/s). Each is nullopt when missing, short, not finite or implausible (accel above 1 m/s^2, gyro above 5 deg/s on
/// any axis).
struct CalibBiases {
    std::optional<Vec3> accel;
    std::optional<Vec3> gyro;
};
CalibBiases calib_json_biases(std::string_view json_text);

/// Parse "0,1" / "0,1,2,3" (distinct ids < count).
std::expected<std::vector<std::size_t>, std::string> parse_cams(std::string_view s, std::size_t count);

/// Basalt calibration ("value0" cereal document) for the selected cameras.
std::expected<std::string, std::string> basalt_calib_json(const DeviceCalib& cal, const SlamSetup& setup,
                                                          const ImuNoise& imu);

/// A short human/machine-readable description of a recording's sensor setup (pimax-slam-info.json).
std::string info_json(const DeviceCalib& cal, const SlamSetup& setup, const ImuNoise& imu,
                      const std::vector<std::pair<std::string, std::string>>& extra);

/// Write pimax-basalt-calib.json and pimax-slam-info.json into dir (created if needed). Returns the calib path.
std::expected<std::string, std::string> write_basalt_files(
    const std::string& dir, const DeviceCalib& cal, const SlamSetup& setup, const ImuNoise& imu,
    const std::vector<std::pair<std::string, std::string>>& extra);

/// Default head origin in the IMU frame: midway between the two lower tracking cameras, 5 cm behind them (same
/// estimate as the Mercury hand attachment, PIMAX_HT_HEAD_OFFSET).
Vec3 default_head_in_imu(const DeviceCalib& cal);

/*
 * Supervisor
 */

/// Supervisor thresholds (the C struct pimax_cam_track_params, same layout).
struct SupervisorParams {
    /// Faster than this between two tracker poses = jump (m/s).
    double max_speed_m_s = 4.0;
    /// Tracker relative rotation vs gyro (3DoF) relative rotation between two poses, degrees.
    double max_rot_mismatch_deg = 8.0;
    /// Consecutive consistent poses needed to (re)acquire.
    std::uint32_t acquire_poses = 12;
    /// Reserved, zero.
    std::uint32_t reserved = 0;
    /// No new tracker pose for this long = lost (ns).
    std::uint64_t stale_ns = 350'000'000;
    /// Lost for this long = ask for a tracker reset (ns, 0 = never).
    std::uint64_t reset_after_ns = 3'000'000'000;
    /// At most one reset request per this interval (ns).
    std::uint64_t reset_every_ns = 6'000'000'000;

    bool operator==(const SupervisorParams&) const = default;
};

/// What the supervisor does while the IMU says the headset is perfectly still (lying on a desk or a cushion). With
/// nothing in the stereo overlap beyond Basalt's 0.33 m triangulation limit the tracker has no landmarks and
/// integrates the IMU alone, so its position runs away quadratically (analysis/slam-drift.md).
struct StillParams {
    /// Hold the head position while still: the tracker's motion goes into the room alignment, not the output.
    bool hold = true;
    /// The tracker moved this far (m) while still: reset it (a fresh start while still has the right zero velocity
    /// and attitude), so the walk that follows does not inherit a runaway velocity.
    double reset_m = 0.03;
    /// At most one such reset per this interval (ns).
    std::uint64_t reset_every_ns = 1'000'000'000;
    /// Re-zero instead of dropping out: the tracker is reset but the output stays exactly where it is (position held,
    /// orientation continuous through the gyro with the tracker's full attitude), no 6DoF loss is counted, and the
    /// fresh tracker world is aligned to the held pose silently. false = the old behaviour (a loss to 3DoF with a
    /// yaw-only fix, re-aligned as a new acquisition; PIMAX_SLAM_STILL_REZERO=0).
    bool rezero = true;
    /// Consistent poses of the fresh tracker needed to take over after a re-zero.
    std::uint32_t rezero_poses = 3;
    /// PiBridge (not in the Rust code; PIMAX_SLAM_STILL_BACKOFF): while the headset stays still, each further re-zero
    /// waits twice as long as the one before, up to reset_every_ns << backoff_max (0 = a re-zero every
    /// reset_every_ns as in Rust). A fresh tracker that sees too little drifts again at once; resetting it every
    /// second keeps it in its weakest phase (study 2: 32 re-zeros in 36 s on the desk).
    std::uint32_t backoff_max = 0;

    bool operator==(const StillParams&) const = default;
};

/// Tracking state.
enum class TrackState : std::uint32_t {
    /// No usable 6DoF yet (3DoF output at the held position).
    Init = 0,
    /// Tracker poses are used.
    Tracking = 1,
    /// Lost after tracking (3DoF at the held position, yaw kept continuous).
    Lost = 2,
};

/// Event bits of one update.
namespace events {
/// Entered Tracking.
inline constexpr std::uint32_t ACQUIRED = 1;
/// Left Tracking.
inline constexpr std::uint32_t LOST = 2;
/// The caller should reset the tracker (t_slam_reset).
inline constexpr std::uint32_t RESET_WANTED = 4;
/// A recenter was applied.
inline constexpr std::uint32_t RECENTERED = 8;
/// Tracking came back in the same tracker world and the old room alignment was restored.
inline constexpr std::uint32_t RESNAPPED = 16;
/// The tracker drifted while the headset lay still and is re-zeroed (reset, output held; no loss).
inline constexpr std::uint32_t REZEROED = 32;
} // namespace events

/// Re-snap rule: when tracking returns, keep the room alignment from before the loss if the tracker world is still
/// the same one (no reset since, yaw consistent with the gyro, head position plausible).
struct Resnap {
    /// Largest yaw difference between the old alignment and the 3DoF-heading one (deg).
    double yaw_deg = 0;
    /// The head may have moved at most max(min_m, speed_m_s * lost time) from the held position.
    double speed_m_s = 0;
    /// See speed_m_s.
    double min_m = 0;

    bool operator==(const Resnap&) const = default;
};

/// One tracker pose: IMU frame in the tracker's world (already rotated to y-up), and the 3DoF filter's IMU orientation
/// at the same time.
struct TrackSample {
    /// Tracker timestamp (monotonic ns).
    std::uint64_t ts_ns = 0;
    /// Position of the IMU in the tracker world.
    Vec3 pos{};
    /// Orientation of the IMU in the tracker world.
    Quat q{};
    /// 3DoF orientation of the IMU at ts_ns.
    Quat q3dof{};

    bool operator==(const TrackSample&) const = default;
};

/// The output the driver applies.
struct TrackOutput {
    /// State.
    TrackState state = TrackState::Init;
    /// events bits raised by this update.
    std::uint32_t events = 0;
    /// world_from_slam (yaw + translation) for tracker poses.
    std::pair<Vec3, Quat> world_from_slam{};
    /// Correction applied to the 3DoF orientation while not tracking (yaw only after a loss; the full attitude of the
    /// tracker during a still re-zero, so the tilt does not change either).
    Quat fix3dof{};
    /// Head position while not tracking.
    Vec3 held_pos{};
    /// Why the last loss/reject happened (a static string).
    const char* reason = "";

    bool operator==(const TrackOutput&) const = default;
};

/// The state machine (see the file comment).
class Supervisor {
public:
    /// New supervisor; head_in_imu = head origin in the IMU frame (m).
    Supervisor(SupervisorParams p, Vec3 head_in_imu);

    /// Enable (or disable with nullopt) restoring the old room alignment after a loss in the same tracker world.
    void set_resnap(std::optional<Resnap> r) { resnap_ = r; }
    /// Still-headset behaviour (see StillParams).
    void set_still_params(StillParams p) { still_p_ = p; }
    /// The output's gravity tilt correction (world frame, applied on top of world_from_slam * tracker q): a re-zero or
    /// a loss takes the attitude the output really shows, so it does not move at either end.
    void set_tilt(const Quat& tilt) { tilt_ = qnorm(tilt); }
    /// The still period that is about to be entered really began with the head at this tracker-world position (the
    /// stillness test needs a window of quiet). Ignored unless the still hold starts next.
    void set_still_anchor(const Vec3& head_slam) { still_anchor_ = head_slam; }
    /// The IMU says the headset is perfectly still (call before update).
    void set_still(bool still);
    /// Current state.
    TrackState state() const { return state_; }
    /// Current world_from_slam (translation, yaw).
    std::pair<Vec3, Quat> world_from_slam() const { return {w_t_, w_q_}; }
    /// Move the room alignment by d (world metres) while tracking: the drift governor's hold puts tracker translation
    /// that no accelerometer evidence explains here, so the head output does not follow it. Ignored unless tracking.
    void shift_world(const Vec3& d);
    /// Head position in the world for a tracker pose (with the current alignment).
    Vec3 head_world(const Vec3& pos, const Quat& q) const;
    /// Put the head at the world origin (x, y, z = 0) without changing its heading; applied at the next update.
    void request_recenter() { recenter_ = true; }
    /// One update. sample = the tracker's newest pose (or nullopt); repeated timestamps are ignored.
    TrackOutput update(std::uint64_t now_ns, std::optional<TrackSample> sample);
    /// The output for the current state with these events.
    TrackOutput output(std::uint32_t events) const;
    /// The newest tracker pose taken (tests).
    std::optional<TrackSample> last() const { return last_; }

    /// Tracker resets raised because it drifted while the headset was still (also counted in resets).
    std::uint64_t still_resets = 0;
    /// Re-zeros in a row while the headset stayed still (PiBridge back-off; 0 when it moves).
    std::uint32_t rezero_streak() const { return rezero_streak_; }
    /// Times Tracking was entered.
    std::uint64_t acquisitions = 0;
    /// Times Tracking was left.
    std::uint64_t losses = 0;
    /// Reset requests raised.
    std::uint64_t resets = 0;
    /// Tracker poses rejected as inconsistent.
    std::uint64_t rejects = 0;

private:
    const char* consistent(const TrackSample& prev, const TrackSample& s) const;
    bool acquire(const TrackSample& s, std::uint64_t now_ns);
    void lose(std::uint64_t now_ns, const char* reason);
    std::uint32_t request_reset();
    std::uint32_t rezero(std::uint64_t now_ns);
    std::uint32_t hold_still(std::uint64_t now_ns);

    SupervisorParams p_;
    Vec3 head_in_imu_;
    TrackState state_ = TrackState::Init;
    std::uint32_t good_run_ = 0;
    std::optional<TrackSample> last_;
    std::uint64_t last_new_at_ns_ = 0;
    Vec3 w_t_{};
    Quat w_q_ = QUAT_ID;
    Quat fix3dof_ = QUAT_ID;
    Vec3 held_pos_{};
    std::uint64_t lost_since_ns_ = 0;
    std::uint64_t last_reset_ns_ = 0;
    bool recenter_ = false;
    const char* reason_ = "no tracker pose yet";
    std::optional<Resnap> resnap_;
    std::optional<std::pair<Vec3, Quat>> lost_w_;
    bool reset_since_loss_ = false;
    StillParams still_p_;
    bool still_ = false;
    // While still and tracking: (tracker-world head position, world head position) at the start of the still period
    // (or of the tracker world, after a reset).
    std::optional<std::pair<Vec3, Vec3>> still_ref_;
    std::uint64_t last_still_reset_ns_ = 0;
    // A still re-zero is in progress: the next acquisition is a silent re-alignment, not a new acquisition.
    bool rezeroing_ = false;
    // World-frame tilt (pitch/roll) correction the output applies on top of the tracker attitude (headpose gravity
    // tilt correction): a re-zero or a loss holds the corrected attitude.
    Quat tilt_ = QUAT_ID;
    // Tracker-world head position where the still period really began.
    std::optional<Vec3> still_anchor_;
    // Re-zeros since the headset last moved (StillParams::backoff_max).
    std::uint32_t rezero_streak_ = 0;
};

} // namespace pibridge::cam::slam
