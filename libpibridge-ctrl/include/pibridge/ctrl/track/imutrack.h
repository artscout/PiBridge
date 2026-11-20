// SPDX-License-Identifier: MIT
//
// The IMU-led controller tracker (PiBridge; docs/research/ctrl-tracking-improvements.md section 8). The design follows
// the constellation trackers of OpenHMD (Rift CV1) and Monado's tracking/constellation (Jan Schmidt, Beyley
// Cardellio; BSL-1.0, read for the design, no code copied):
//
// * each controller has its own state, led by its IMU: an error-state Kalman filter (orientation, position, velocity,
//   gyro bias, accelerometer bias) propagated with the controller's gyro and accelerometer (from the fuser's hint
//   file, at the fuser's write rate) in the world frame of the head pose; an optical fix is a measurement update,
//   gated by its Mahalanobis distance, never the pose itself;
// * the LED <-> blob correspondence comes from the prediction: the model's facing LEDs are projected from the
//   predicted pose and matched to blobs within a window from the state's covariance; the pose is refined with the
//   predicted orientation as a prior. A blob keeps its LED from one LED frame to the next through the prediction;
// * the brute-force P3P search runs only for a controller that is lost (first acquisition, or 10 LED frames without
//   an update); every hypothesis must agree with that controller's IMU: gravity tilt from its accelerometer and, once
//   it has had a fix, the orientation its gyro carried since. A first acquisition is tentative until three updates in
//   a row pass the filter's gate (a mirror fit turns against the gyro and fails);
// * so the hand identity is the IMU's: a fix belongs to the controller whose prediction it matches; a controller whose
//   IMU is parked (quiet for imu_parked_s) or silent is not searched for.
//
// Without IMU samples for both controllers (no hint file: another Monado build), or without a head pose (the hint's
// head pose stays the identity: no SLAM, so the head frame is not an inertial world), every LED frame goes to the
// per-frame Tracker (tracker.h) with the IMU hand identity (TrackCfg::imu_id).
#pragma once

#include "pibridge/ctrl/track/rig.h"
#include "pibridge/ctrl/track/tracker.h"

#include <array>
#include <string>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace pibridge::ctrl::track {

/// One IMU sample of a controller (hint::Entry v3 +64..+148).
struct ImuIn {
    std::uint64_t t = 0;
    /// Gyro-integrated orientation (aim frame, arbitrary origin).
    cam::geom::Mat3 qg = cam::geom::IDENTITY;
    /// Mean specific force since the previous sample (aim frame, m/s^2).
    cam::geom::Vec3 f_aim{0.0, 9.80665, 0.0};
    /// Low-passed gravity up in the aim frame; up_ok when the accelerometer reads about 1 g.
    cam::geom::Vec3 up_aim{0.0, 1.0, 0.0};
    bool up_ok = false;
    double still_s = 0.0;
    cam::geom::Pose world_from_head;
};

struct ImuTrackCfg {
    /// Correspondence window (px) from the prediction's covariance, clamped.
    double gate_min_px = 6.0;
    double gate_max_px = 40.0;
    /// Fewest matched LEDs for an update (3; 3-LED fits must have rms <= 1.2 px).
    std::size_t min_leds = 3;
    /// Acquisition: largest tilt error against the accelerometer (deg), and orientation error against the gyro-carried
    /// orientation (deg; once the controller has had a fix).
    double acq_tilt_deg = 20.0;
    /// PiBridge: the tilt test of the quad acquisition before the controller's up is learnt, against the world's up
    /// (ctrl-imu-take2: 60-90 deg alike; 45 refuses true fits, 120+ lets the half-turn twin of the right hand through).
    double acq_tilt_cold_deg = 75.0;
    /// PiBridge: a search fit closer than this to the other controller's fit of the same frame is refused.
    double acq_apart_m = 0.08;
    double acq_rot_deg = 30.0;
    /// LED frames without an update before a tracked controller is searched for again.
    std::uint32_t miss_lost = 10;
    /// Updates in a row that make a first acquisition a track.
    std::uint32_t confirm = 3;
    /// Velocity damping without updates: after free_s, with tau_v.
    double free_s = 0.25;
    double tau_v = 0.5;
    /// A controller whose gyro has been quiet this long is parked (lying somewhere): not searched for.
    double imu_parked_s = 5.0;
    /// Chi-square gate of an update (6 dof).
    double gate_chi2 = 22.5;
    /// Process noise of the acceleration (m/s^2 per sqrt(s)): the accelerometer's errors and those of the gyro
    /// orientation that rotates it into the world.
    double sigma_a = 1.5;
    /// Measurement noise of a fix's position with 8+ / 5-7 / 3-4 LEDs (m).
    std::array<double, 3> sigma_p{0.008, 0.016, 0.03};
    /// Integrate the accelerometer into the position. Needs the head pose (a level, inertial world): without it (the
    /// head frame as the world) the position runs on constant velocity and the gyro carries only the orientation.
    bool use_accel = true;
};

class ImuTracker {
public:
    ImuTracker(ImuTrackCfg cfg, TrackCfg fallback);

    ImuTrackCfg cfg;
    /// The aim frame (IMU) in the LED model frame: rot_x(-aim pitch).
    cam::geom::Mat3 device_from_aim = cam::geom::IDENTITY;
    std::array<bool, 2> wanted{true, true};
    struct Counts {
        std::uint64_t updates = 0;
        std::uint64_t misses = 0;
        std::uint64_t gate_rejects = 0;
        std::uint64_t acquisitions = 0;
        std::uint64_t acq_refused = 0;
        /// acq_refused by reason: tilt against the learnt up, turn from the filter, too far from the filter.
        std::uint64_t acq_tilt = 0;
        std::uint64_t acq_rot = 0;
        std::uint64_t acq_far = 0;
        std::uint64_t acq_overlap = 0;
        /// PiBridge: the twin check (the LED ring's near-symmetry: a fit turned ~180 deg matches the same LEDs): windows
        /// compared, votes for the twin, and switches to it.
        std::uint64_t twin_checks = 0;
        std::uint64_t twin_votes = 0;
        std::uint64_t twin_flips = 0;
        /// PiBridge: tracks dropped because neither the orientation nor its twin explained the accelerometer for 0.5 s,
        /// and the mean of the current orientation's error (m/s over 0.4 s; ~0.25 when right).
        std::uint64_t orient_resets = 0;
        /// PiBridge: frames kept tracked by single-LED filter updates (no pose fit possible), and the LEDs used.
        std::uint64_t led_updates = 0;
        std::uint64_t led_points = 0;
        /// PiBridge: predictions moved by a Pimax-style P1P hypothesis (gyro rotation, one LED-blob pair).
        std::uint64_t p1p_moves = 0;
        /// PiBridge (Pimax's ControllerDetect): cold acquisitions as hypotheses checked by the IMU over 3+ frames:
        /// hypotheses started (twins included), dropped by the gyro, accepted, refused by the motion test, and
        /// acceptances delayed while the twin explained the IMU as well.
        std::uint64_t hyp_started = 0;
        std::uint64_t hyp_gyro_drops = 0;
        std::uint64_t hyp_accepted = 0;
        std::uint64_t hyp_motion_fail = 0;
        std::uint64_t hyp_ambiguous = 0;
        /// Hypotheses refused because the other controller's gyro explained their turns better.
        std::uint64_t hyp_other_hand = 0;
        /// Few-LED runs cut at their limit, and search fits refused for lying on the other controller's state.
        std::uint64_t led_run_cut = 0;
        std::uint64_t acq_on_other = 0;
        /// Blobs near both predicted controllers given to the nearer one, and strong fits the gate refused that
        /// restarted the position and velocity instead.
        std::uint64_t contested = 0;
        std::uint64_t reanchors = 0;
        /// Fixes not published for lying over 1.2 m from the head (the controller lost instead).
        std::uint64_t far_drops = 0;
        double accel_err_sum = 0;
        std::uint64_t accel_err_n = 0;
        /// Search candidates per hand (ImuTracker::step 2), and lost controllers not searched for being parked.
        std::array<std::uint64_t, 2> acq_cands{0, 0};
        std::array<std::uint64_t, 2> parked{0, 0};
        std::uint64_t tentative_dropped = 0;
        std::uint64_t fallback_frames = 0;
        /// Why a predicted controller was missed: too few LEDs in the window, too few after the refine, the fit's rms.
        std::uint64_t miss_window = 0;
        std::uint64_t miss_refined = 0;
        std::uint64_t miss_rms = 0;
    };
    Counts counts;
    /// The per-frame tracker's statistics (stats.led_frames, hand, few, searches are kept for this tracker too).
    Stats& stats() { return fallback_.stats; }
    /// One line per hand: tracking status, learnt-up samples (`up_n`), misses in a row (the live log).
    std::string describe_slots() const;

    void imu_push(std::size_t slot, const ImuIn& s);
    /// One LED frame. Fix.found.joint is the optical fit (the measurement); Fix.published the filter's pose after the
    /// update, in the body (head) frame: what goes to the pose file.
    std::vector<Fix> step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns);

private:
    enum class Status { Lost, Tentative, Tracking };
    struct Ctl {
        Status status = Status::Lost;
        bool has_state = false; // an orientation and position (after the first acquisition)
        cam::geom::Mat3 r = cam::geom::IDENTITY;
        cam::geom::Vec3 p{0.0, 0.0, 0.0};
        cam::geom::Vec3 v{0.0, 0.0, 0.0};
        cam::geom::Vec3 bg{0.0, 0.0, 0.0};
        cam::geom::Vec3 ba{0.0, 0.0, 0.0};
        std::array<double, 225> P{};
        std::uint64_t t = 0;              // state time
        bool has_ref = false;             // gyro reference at t
        cam::geom::Mat3 qg_ref = cam::geom::IDENTITY;
        std::uint64_t last_update = 0;
        std::uint32_t misses = 0;
        std::uint32_t streak = 0;
        /// Frames in a row kept only by few-LED updates (Pimax: 15 such frames at most before the controller is lost).
        std::uint32_t led_run = 0;
        /// Gravity up in the tracking world as this controller's accelerometer sees it (learned from its fixes: absorbs
        /// a head frame that is not level when there is no head pose, and the accelerometer's axis error).
        cam::geom::Vec3 up_w{0.0, 1.0, 0.0};
        std::uint32_t up_n = 0;
    };
    void propagate(std::size_t slot, std::uint64_t t);
    /// A controller whose gyro-carried orientation still constrains its search: updated within CT_KNOWN_S (10 s) and,
    /// when lost, within 1 m of the head.
    bool known(std::size_t s, std::uint64_t t) const;
    /// The twin check of a tracked controller: the change of its velocity over 0.4 s from the optical fits against the
    /// accelerometer's under the current orientation and under the twin's; switches to the twin when it explains the
    /// motion clearly better three times in a row.
    void twin_check(std::size_t slot, std::uint64_t frame_ns);
    /// A tracked controller's state after an acquisition: the twin check starts over.
    void twin_reset(std::size_t slot);
    /// A tracked controller whose pose fit failed: each LED matched near the prediction corrects the filter by its
    /// bearing (an EKF update per LED, chi2-gated); the filter's pose then goes out as a few-LED fix. False with none.
    /// Pimax's P1P tracking hypotheses: the prediction moved (rotation kept) so that one near (LED, blob) pair agrees,
    /// the one matching the most LEDs in a 4 px window, if it beats the prediction itself.
    std::optional<cam::geom::Pose> p1p_hypothesis(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                                                  const std::vector<std::vector<bool>>& used, double cos_face,
                                                  const cam::geom::Pose& pred, double radius_px) const;
    bool led_updates(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs, std::size_t slot,
                     std::uint64_t frame_ns, const std::vector<Matches>& m, std::vector<std::vector<bool>>& used,
                     std::vector<Fix>& out);
    /// One frame of a hypothesis: the fit in the world, and the gyro's orientation at it.
    struct HypFrame {
        std::uint64_t t = 0;
        cam::geom::Mat3 r = cam::geom::IDENTITY;
        cam::geom::Vec3 p{0.0, 0.0, 0.0};
        cam::geom::Mat3 qg = cam::geom::IDENTITY;
        JointSolve j;
    };
    struct Hyp {
        std::vector<HypFrame> f;
        std::uint32_t misses = 0;
    };
    /// Pimax's ControllerDetect for a controller with no orientation to go by: every candidate (and its twin) is a
    /// hypothesis carried by the controller's gyro; a new candidate within 0.3 rad of the carried orientation extends it
    /// (two misses drop it). After 3 frames the fits' motion must agree with the accelerometer (the change of velocity
    /// against the specific force under the hypothesis' orientation; gravity alone when still). The hypothesis accepted,
    /// with its velocity, when it agrees and its twin does not explain the IMU as well.
    std::optional<std::pair<HypFrame, cam::geom::Vec3>> hyp_step(std::size_t slot, std::uint64_t frame_ns,
                                                                 const std::vector<HypFrame>& cands);
    /// The IMU disagreement of a hypothesis over its last frames (m/s per frame pair; < 0 without the IMU samples), and
    /// whether the controller was still (gravity tested instead).
    double hyp_err(std::size_t slot, const Hyp& h, bool& still) const;
    std::array<std::vector<Hyp>, 2> hyps_{};
    void step_state(Ctl& c, double dt, const cam::geom::Mat3& dm, const cam::geom::Vec3& f_model, double still_s);
    bool update(Ctl& c, const cam::geom::Pose& world_meas, double sig_p, double sig_r_rad, bool use_rot);
    const ImuIn* imu_at(std::size_t slot, std::uint64_t t) const;
    bool imu_live(std::size_t slot, std::uint64_t t) const;

    std::array<Ctl, 2> c_{};
    /// Per model: the LED ring's near-symmetry (model -> model, x -> r x + t), if it has one (found on first use).
    struct Twin {
        cam::geom::Pose t;
        double rms_m = 0;
    };
    std::array<std::optional<Twin>, 2> twin_{};
    bool twin_found_ = false;
    /// Per hand: the specific force in the world at each IMU step under the current orientation and the twin's
    /// (t, f_cur, f_twin), and the optical fits' positions (t, p); about 1.5 s.
    struct ForceRow {
        std::uint64_t t;
        cam::geom::Vec3 cur, twin;
    };
    std::array<std::deque<ForceRow>, 2> force_{};
    std::array<std::deque<std::pair<std::uint64_t, cam::geom::Vec3>>, 2> opt_{};
    std::array<std::uint64_t, 2> twin_last_{0, 0};
    std::array<std::uint32_t, 2> twin_run_{0, 0};
    std::array<std::uint32_t, 2> orient_bad_{0, 0};
    std::array<std::deque<ImuIn>, 2> imu_{};
    /// Last time a sample carried a head pose that is not the identity.
    std::uint64_t head_seen_ = 0;
    Tracker fallback_;
    std::uint64_t frame_no_ = 0;
    std::uint64_t last_fix_ns_ = 0;
};

} // namespace pibridge::ctrl::track
