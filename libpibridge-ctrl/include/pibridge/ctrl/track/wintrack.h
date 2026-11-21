// SPDX-License-Identifier: MIT
//
// PiBridge: the controller tracker after Pimax's LED engine (LedObjectPoseEstimator 2.11.2, reverse-engineered
// 2026-11-20), built on the sliding-window estimator (window.h). Per LED frame, per controller:
//
// * the state predicted to the frame with the controller's IMU (RK-like midpoint integration of the hint samples);
// * the model's LEDs projected from it (visible within 100 deg, inside the image); candidate (LED, blob) pairs within
//   999 / 300 / 150 px (first 5 frames / fewer than 4 LEDs last time / else); pose hypotheses from them: the prediction,
//   P3P on rough matches, P2P and P1P with the predicted rotation (at most 30);
// * every hypothesis scored in all cameras by greedy one-to-one matches within a gate set by the nearest LED's distance
//   to the camera (60 / 10 / 5 / 3 / 1.5 px); the choice: most inliers, then the error, then nearness to the
//   prediction (with Pimax's alternative rule when the best is far from it);
// * 2 inliers suffice: they go into the window as reprojection residuals; no gate after the solve;
// * lost: a budget of 30 frames or 0.5 s (15 with few LEDs or the controllers close, 0 with an LED under 5 cm from a
//   camera, 5 right after an acquisition) during which the IMU prediction is matched every frame (a relock needs 4
//   inliers; after 6 lost frames also Constellation's box rule: the match explains the blobs where the controller would
//   be) and the search with the gyro-carried orientation retried; beyond it, or over 1.25 m from the head, the matcher
//   stops but the search goes on;
// * warm relock (PiBridge, as imutrack's known()): for 10 s after the last update a 5+ LED fit that agrees with the
//   gyro-carried orientation (or holds it when refit from it) re-anchors the window at once: near the IMU prediction,
//   or anywhere clear of the other controller with 7+ LEDs; the restart carries the velocity only while fresh;
// * acquisition (cold, nothing known for 10 s): PiBridge's quad index and hypotheses confirmed over frames by the
//   controller's IMU (imutrack's ControllerDetect port), the window restarted from the confirmed pose;
// * output: the window's newest state (pose, velocity) per frame; the fuser carries it to the display time.
//
// Kept from PiBridge beyond Pimax: the quad index (faster than Pimax's 4-blob enumeration), the gyro hand test, the
// controllers' contested blobs given to the nearer one.
#pragma once

#include "pibridge/ctrl/track/imutrack.h"
#include "pibridge/ctrl/track/window.h"

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::ctrl::track {

struct WinTrackCfg {
    WindowCfg window;
    /// Candidate radii (px) and the distance-gated inlier tolerance (Pimax).
    double radius_first = 999.0;
    double radius_few = 300.0;
    double radius = 150.0;
    std::size_t first_frames = 5;
    /// Pose hypotheses at most.
    std::size_t max_hyp = 30;
    /// P1P: a hypothesis' translation within this of the prediction (m).
    double p1p_max_m = 0.05;
    /// PiBridge: P3P / P2P hypotheses within these of the prediction (m, deg). The gyro's orientation over one frame
    /// is good to a degree, so a hypothesis turned far from it is a wrong LED assignment (take2: 55 deg P3P picks
    /// with 6 "inliers" poisoned the window); Pimax's roughly_match is not known in this detail.
    double p3p_max_m = 0.25;
    double p3p_max_deg = 15.0;
    double p2p_max_m = 0.15;
    /// Fewest inliers for an update, and for the first update after a loss (PiBridge: Pimax's detection minimum; a
    /// 2-3 blob relock at a drifted prediction placed the controller 10-60 cm off on the mask test).
    std::size_t min_leds = 2;
    std::size_t min_leds_relock = 4;
    /// PiBridge: the matcher may relock a lost controller within this many frames of its last update; beyond it only
    /// the search (a full fit of 5+ LEDs near the prediction) may. Pimax matches the prediction on every lost frame;
    /// 2 frames (the mask test: 10-60 cm relocks after 0.5 s of IMU-only prediction, with 2-3 inliers) cost the live
    /// dump win1 a cold acquisition for every loss the search did not catch (lost time 63 / 93 s); the whole budget
    /// gave 35 / 51 s but, without the revert, 2 wrong relocks and 9 jumps on the masks; 6 frames (0.24 s) 43 / 61 s
    /// with the masks as before (WT_RELOCK_FRAMES).
    std::uint32_t relock_frames = 6;
    /// PiBridge (after Constellation's with-prior rule, docs/constellation-reference.md): beyond relock_frames and for
    /// the rest of the lost budget, the matcher may still relock when its match explains the blobs where the
    /// controller would be: at most this fraction of the free blobs inside the hypothesis' projected LED boxes is left
    /// unmatched (0 = off, the plain 6-frame cut). A 2-3 blob match at a drifted prediction leaves most of the
    /// controller's blobs unexplained; a correct one leaves none (WT_RELOCK_BOX).
    double relock_box_frac = 0.25;
    /// PiBridge option: the 1 m rule measured from the last update's position rather than the IMU-only prediction
    /// (imutrack does). Off: on win1 it only turned far resets into budget resets (WT_FAR_LAST).
    bool far_from_last = false;
    /// PiBridge: an IMU sample whose f_aim is exactly 0 (no accelerometer reading in the hint) holds the previous
    /// sample's force instead of counting as a free fall.
    bool acc_hold = true;
    /// PiBridge: Pimax's inlier gates (60 / 10 / 5 / 3 / 1.5 px by distance) scaled: our blob centroids and camera
    /// model leave ~2 px rms where Pimax's leave less (WT_GATE_SCALE to measure).
    double gate_scale = 1.5;
    /// A lost controller's gyro-carried orientation is known for this long after its last update (s), as imutrack's
    /// CT_KNOWN_S, whether the lost budget has run out or not: the search's fits must agree with it, and one that does
    /// (a 5+ LED fit near the IMU prediction, or a `relock_strong` fit anywhere clear of the other controller)
    /// re-anchors the window at once, instead of the cold acquisition's three IMU-confirmed frames. Before 2026-11-21
    /// a budget or far reset made the controller cold, and on the live dumps the cold acquisition took 0.9 s median
    /// and up to 5 s after the search had found the controller. (Re-anchoring by the orientation alone with no LED
    /// count or other-controller guard was tried earlier: take2 fell to 80 / 62 % with 17 jumps.)
    double warm_s = 10.0;
    /// PiBridge option (WT_REVERT, px; 0 = off, Pimax's way): an update whose window rms comes out above this with 4+
    /// inliers is undone and counts as a miss. It was 8 px against wrong assignments poisoning the window; live
    /// (win1) it undid 476 of 604 updates that started a loss, during fast motion (|v| 2.7 m/s) where the IMU
    /// prediction had 0-1 inliers and the P2P hypothesis 7: a window inconsistency, not a wrong assignment. Off, the
    /// window follows the LEDs: tracked 88 / 72 % -> 92 / 88 %, the error against Pimax unchanged.
    double revert_rms_px = 0.0;
    /// LED visibility (deg from the direction to the camera).
    double visible_deg = 100.0;
    /// Lost budget (frames, s) and the default / weak / acquired thresholds.
    std::uint32_t lost_default = 30;
    std::uint32_t lost_weak = 15;
    std::uint32_t lost_acquired = 5;
    double lost_max_s = 0.5;
    /// The far rule (m from the head). PiBridge: 1.25 instead of Pimax's 1 m: on the live dumps a Beat Saber swing or a
    /// hanging arm reached 0.95-1.0 m from our head origin and the rule threw a tracked controller into a cold
    /// acquisition (WT_FAR_M).
    double far_m = 1.25;
    /// PiBridge: a warm relock (the gyro-carried orientation known, see warm_s) from a fit with this many LEDs is
    /// accepted anywhere, not only near the IMU prediction, when it is clear of the other controller: after a loss the
    /// prediction's position is often off by far more than the gate (the window's velocity after 2-3 LED updates), as
    /// imutrack's CT_FAR_LOOSE (WT_STRONG).
    std::size_t relock_strong = 7;
    /// PiBridge: a warm fit whose orientation disagrees with the gyro's by over 30 deg (the LED assignment on the
    /// ring's twin, or a few-LED fit of a far controller: they scatter 20-40 deg) is refit from the gyro's orientation
    /// at its place (12 / 5 / 2.5 px windows) and accepted if 5+ LEDs hold that within 30 deg (WT_REFIT=0: refuse).
    bool refit_turned = true;
    /// Cold hypotheses: a new fit extends one when its orientation is within this of the gyro-carried one (rad; Pimax
    /// 0.3); a fit of fewer than 8 LEDs gets the looser gate, since on the live dumps such fits of a far controller
    /// scattered 20-40 deg frame to frame and the chains never reached 3 frames (WT_HYP_FEW_RAD).
    double hyp_gyro_rad = 0.3;
    double hyp_gyro_rad_few = 0.6;
    /// The controllers "close" when their projections are within this (px).
    double close_px = 75.0;
    /// Fast flag (Huber 2) above this mean |a| (m/s^2).
    double fast_a = 13.0;
};

class WinTracker {
public:
    WinTracker(WinTrackCfg cfg, TrackCfg fallback);

    WinTrackCfg cfg;
    cam::geom::Mat3 device_from_aim = cam::geom::IDENTITY;
    std::array<bool, 2> wanted{true, true};

    struct Counts {
        std::uint64_t updates = 0;        // frames with a window update
        std::uint64_t misses = 0;         // predicted controllers without one
        std::uint64_t lost_frames = 0;    // frames spent in Lost
        std::uint64_t initing_frames = 0; // frames spent in Initing
        std::uint64_t acquisitions = 0;
        std::uint64_t relocks = 0;        // searches with the gyro prior that re-anchored the window
        std::uint64_t relocks_initing = 0; // of them, after the lost budget / far rule had run out (warm Initing)
        std::uint64_t relocks_strong = 0;  // of them, accepted away from the prediction by their LED count
        std::uint64_t relocks_refit = 0;   // of them, refit from the gyro's orientation (the fit's disagreed)
        std::uint64_t relocks_twin = 0;    // of those, off the ring's twin
        std::uint64_t relock_refused = 0;  // warm fits refused (orientation, position, tilt, the other controller)
        std::uint64_t far_resets = 0;
        std::uint64_t budget_resets = 0;
        std::uint64_t hyp_total = 0;      // pose hypotheses scored
        std::uint64_t hyp_p3p = 0, hyp_p2p = 0, hyp_p1p = 0;
        std::uint64_t pick_pred = 0, pick_p3p = 0, pick_p2p = 0, pick_p1p = 0, pick_alt = 0;
        std::uint64_t contested = 0;
        std::uint64_t reverted = 0;    // updates undone for a bad window rms
        std::uint64_t miss_no_pairs = 0; // no candidate pair within the radius
        std::uint64_t miss_one = 0;      // the best hypothesis matched a single LED
        std::uint64_t relock_short = 0; // relocks refused for too few inliers
        std::uint64_t relock_late = 0;  // matcher relocks beyond relock_frames, passed by the box rule
        std::uint64_t still_strict = 0, still_loose = 0, fast_stop_frames = 0;
        std::uint64_t imu_gaps = 0; // frames whose IMU did not cover the frame time (extrapolated)
        std::uint64_t imu_no_acc = 0; // IMU samples without an accelerometer reading (f_aim 0; the last one held)
        double iter_sum = 0.0;
        double match_ms = 0.0, push_ms = 0.0, search_ms = 0.0, hyp_ms = 0.0, step_ms = 0.0, step_max_ms = 0.0; // profile
        double vis_rms_sum = 0.0;
        std::uint64_t vis_rms_n = 0;
        std::uint64_t inliers_sum = 0;
        std::uint64_t fallback_frames = 0;
        // acquisition hypotheses (as imutrack)
        std::uint64_t hyp_started = 0, hyp_gyro_drops = 0, hyp_accepted = 0, hyp_motion_fail = 0, hyp_other_hand = 0;
    };
    Counts counts;
    Stats& stats() { return fallback_.stats; }
    std::string describe_slots() const;

    void imu_push(std::size_t slot, const ImuIn& s);
    std::vector<Fix> step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns);

private:
    enum class Mode { None, Initing, Tracking, Lost };
    struct Hand {
        Mode mode = Mode::None;
        Window win;
        std::uint64_t last_update = 0;
        /// The place of the last update with 5+ LEDs (world): the far rule of the warm relock measures from here, since
        /// a 2-3 LED update can leave the window's position metres off.
        std::optional<cam::geom::Vec3> good_p;
        std::uint64_t lost_t = 0;
        std::uint32_t lost_count = 0;
        std::uint32_t lost_threshold = 30;
        std::uint32_t frames_tracked = 0;
        std::size_t last_valid = 0;
        // fast-stop
        bool fast_stop = false;
        std::uint32_t quiet_frames = 0;
        std::uint32_t slow_frames = 0;
        int motion_cooldown = 0;
        bool fast = false;
        std::uint32_t slow_a_frames = 0;
        // the controller's up (gravity) learnt for the acquisition tilt test, as imutrack
        cam::geom::Vec3 up_w{0.0, 1.0, 0.0};
        std::uint32_t up_n = 0;
    };
    std::array<Hand, 2> h_{};
    std::array<std::deque<ImuIn>, 2> imu_{};
    std::uint64_t head_seen_ = 0;
    Tracker fallback_;
    std::uint64_t frame_no_ = 0;
    std::uint64_t last_fix_ns_ = 0;
    // the LED ring's twin per model (acquisition hypotheses), found on first use
    std::array<std::optional<std::pair<cam::geom::Pose, double>>, 2> twin_{};
    bool twin_found_ = false;

    // acquisition hypotheses carried by the gyro (imutrack's ControllerDetect port)
    struct HypFrame {
        std::uint64_t t = 0;
        cam::geom::Mat3 r = cam::geom::IDENTITY; // world_from_model
        cam::geom::Vec3 p{0.0, 0.0, 0.0};
        cam::geom::Mat3 qg = cam::geom::IDENTITY;
        JointSolve j;
    };
    struct Hyp {
        std::vector<HypFrame> f;
        std::uint32_t misses = 0;
    };
    std::array<std::vector<Hyp>, 2> hyps_{};
    std::optional<std::pair<HypFrame, cam::geom::Vec3>> hyp_step(std::size_t slot, std::uint64_t frame_ns,
                                                                 const std::vector<HypFrame>& cands);
    double hyp_err(std::size_t slot, const Hyp& h, bool& still) const;

    const ImuIn* imu_at(std::size_t slot, std::uint64_t t) const;
    bool imu_live(std::size_t slot, std::uint64_t t) const;
    /// IMU steps (device frame) over (t0, t1], partial samples scaled, the tail past the last sample extrapolated.
    std::vector<ImuStep> steps(std::size_t slot, std::uint64_t t0, std::uint64_t t1, bool* gap) const;
    struct ImuStats {
        bool ok = false;
        double mean_g = 0, std_g = 0, max_g = 0, mean_a = 0, std_a = 0, max_dev = 0;
    };
    ImuStats imu_stats(std::size_t slot, std::uint64_t t, const cam::geom::Vec3& bg, const cam::geom::Vec3& ba) const;

    struct Match {
        std::vector<Matches> per_cam; // (led, blob) per camera
        std::size_t count = 0;
        double err = 0.0; // summed px error
        double dt = 0.0;  // from the prediction
        double dr = 0.0;
        int kind = 0;     // 0 prediction, 1 p3p, 2 p2p, 3 p1p
        std::size_t pred_count = 0; // the prediction's own inliers (debug)
        double pred_err = 0.0;
        cam::geom::Pose body_pose; // body_from_model of the hypothesis
        std::size_t box_blobs = 0;  // free blobs inside the hypothesis' projected LED boxes (all cameras)
        std::size_t expected = 0;   // LEDs projected visible under the hypothesis (all cameras)
    };
    /// Pimax's matcher for one controller: the best hypothesis' matches (nullopt: fewer than min_leds).
    std::optional<Match> match(const Rig& rig, std::size_t slot, const WinState& pred, const cam::geom::Pose& bfw,
                               const std::vector<CamObs>& obs, const std::vector<std::vector<bool>>& used);
};

} // namespace pibridge::ctrl::track
