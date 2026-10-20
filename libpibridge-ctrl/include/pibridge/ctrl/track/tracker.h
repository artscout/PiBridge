// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/tracker.rs (MIT OR Apache-2.0).
//
// The per-LED-frame controller tracker shared by `live` (broker) and `lockrate` (offline replay of a `.pxcam`).
//
// Per LED frame: every hand slot with a recent fix is tracked from its prior (match within a pixel gate, joint refine
// over all cameras); a slot that is not tracked triggers the brute-force search. On top of the original tracker
// (TrackCfg::baseline), the default config adds (omarchyvr analysis/ctrl-lock-rate.md):
// * a constant-velocity prediction of the prior (position and rotation from the last two fixes);
// * a wider pixel gate when the controller moves fast or the prior is older than one frame;
// * a constrained 3-4 LED track: with the orientation tied to the prediction (rotation prior in the LM), three or
//   four matched LEDs give a position fix where the original needed five. Few-LED fixes are limited to a streak of
//   `few_streak_max` frames without a 5+ LED fix, so a wrong lock cannot persist.
#pragma once

#include "pibridge/ctrl/track/assign.h"
#include "pibridge/ctrl/track/rig.h"

#include <array>
#include <deque>
#include <cstdint>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

namespace pibridge::ctrl::track {

struct TrackCfg {
    /// Constant-velocity prediction of the prior.
    bool predict = true;
    /// Second, wider prior gate (px) for fast motion or an older prior; 0 = off.
    double wide_gate_px = 24.0;
    /// Fewest LEDs for a constrained (rotation-prior) track from a prior; 5 or more = off.
    std::size_t few_leds = 3;
    /// Most consecutive few-LED fixes per slot before its prior is dropped.
    std::uint32_t few_streak_max = 12;
    /// Rotation prior weight (px per rad) for few-LED tracks.
    double rot_w = 300.0;
    /// Body model (reach, side, continuity, collapse); off = `--no-body`.
    bool body = true;
    /// Loss state machine: a slot whose prior fails this many LED frames in a row goes back to init (prior and
    /// continuity memory dropped, cold start next); 0 = off.
    std::uint32_t fail_limit = 30;
    /// A cold start (init slot) farther than this from the head (m) needs `far_leds` LEDs; infinite = off.
    double far_m = 1.8;
    /// See far_m.
    std::size_t far_leds = 4;
    /// PiBridge: each hand slot is solved with its own LED model (slot 0 = the left model, 1 = the right). A search fix
    /// labelled with the other model is re-fitted with the slot's model (or dropped): the two models are nearly
    /// mirror images, and a hand fitted with the wrong one sits about 10 cm off (ctrl2.pxcam: 11 % of the fixes,
    /// the jumps that alternate with the model label). Off = the Rust tracker.
    bool slot_model = false;
    /// PiBridge: use the fuser's hint (Tracker::hint, from the hint file pibridge/ctrl/hint.h: the IMU orientation and the
    /// inertial position at the frame time). With a hint, a slot's few-LED fix takes the hint's orientation as its
    /// rotation prior (weight hint_rot_w) and needs only hint_few_leds LEDs; a slot without a prior relocks from the
    /// hint (fix at most hint_max_age_s old); a search fix whose orientation is more than hint_lr_deg from the slot's
    /// hint is not that hand. Off = no hint used.
    bool use_hint = false;
    double hint_rot_w = 600.0;
    std::size_t hint_few_leds = 2;
    double hint_max_age_s = 1.0;
    double hint_lr_deg = 35.0;
    /// The hint replaces a slot's own prediction only when its last fix is older than this (s).
    double hint_stale_s = 0.05;
    /// PiBridge: hand identity from each controller's IMU (Tracker::imu_push, from the hint file). A fix is that hand's
    /// only when its turn since the hand's last fix agrees with that hand's gyro over the same interval (within
    /// imu_rot_deg), or, without a recent fix, when its tilt agrees with the hand's accelerometer (imu_tilt_deg); a
    /// hand whose gyro is quiet does not take a fix that moved from its last one; a hand quiet for imu_rest_s or not
    /// reporting gets no new fix. Off = no IMU used.
    bool imu_id = false;
    double imu_rot_deg = 25.0;
    double imu_tilt_deg = 30.0;
    double imu_rest_s = 5.0;
    /// Refit a refused fix with the gyro's orientation as the rotation prior (worse on the take: off).
    bool imu_refit = false;

    /// The tracker before 2026-10-15 (prior = last fix, 12 px gate, 5+ LEDs).
    static TrackCfg baseline();
    /// The Rust defaults plus PiBridge's improvements (docs/research/ctrl-tracking-improvements.md); what `live` uses
    /// unless PIBRIDGE_CTRL_PARITY=1.
    static TrackCfg improved();
};

/// The cold-start rule: from init, a controller farther than `far_m` from the head must show `far_leds` LEDs (few
/// LEDs far away are mostly reflections and give wild poses).
bool cold_start_ok(const TrackCfg& cfg, double dist_m, std::size_t leds);

/// Consecutive failed frames per slot (TrackCfg::fail_limit).
struct FailCount {
    std::array<std::uint32_t, 2> n{0, 0};
    /// Record one frame's result for a slot with a prior; true = the slot must go back to init now.
    bool frame(std::size_t slot, bool ok, std::uint32_t limit);
    /// Forget a slot (it has no prior any more).
    void clear(std::size_t slot) { n[slot] = 0; }
    bool operator==(const FailCount&) const = default;
};

struct Stats {
    std::uint64_t led_frames = 0;
    std::array<std::uint64_t, 2> hand{0, 0};
    std::array<std::uint64_t, 2> few{0, 0};
    std::uint64_t tries = 0;
    std::uint64_t searches = 0;
    std::uint64_t unassigned = 0;
    std::uint64_t collapsed = 0;
    /// Slots sent back to init after `fail_limit` failed frames.
    std::uint64_t init_resets = 0;
    /// Far cold starts refused for having fewer than `far_leds` LEDs.
    std::uint64_t far_refused = 0;
    /// PiBridge (slot_model): search fixes re-fitted with the slot's own model, and those dropped for not fitting it.
    std::uint64_t model_refit = 0;
    std::uint64_t model_dropped = 0;
    /// PiBridge (use_hint): slots relocked from the hint, search fixes refused by the hint's orientation.
    std::uint64_t hint_relocks = 0;
    std::uint64_t hint_refused = 0;
    /// PiBridge (imu_id): fixes refused by the hand-identity test (turn, tilt, at rest, not reporting).
    std::uint64_t imu_refused = 0;
};

/// One IMU sample of a controller for the hand-identity test (from the hint file, hint::Entry +64..+108).
struct ImuSample {
    std::uint64_t t = 0;
    /// Gyro-integrated orientation, aim frame, arbitrary origin.
    cam::geom::Mat3 qg = cam::geom::IDENTITY;
    cam::geom::Vec3 up_aim{0.0, 1.0, 0.0};
    bool up_ok = false;
    cam::geom::Vec3 up_body{0.0, 1.0, 0.0};
    double still_s = 0.0;
};

/// The fuser's view of one controller at an LED frame's time (TrackCfg::use_hint).
struct Hint {
    cam::geom::Pose body_from_model;
    /// Seconds since the fuser's last accepted fix.
    double fix_age_s = 0.0;
    /// hint::Entry state: 1 optical, 2 held.
    std::uint32_t state = 0;
};

/// One tracked hand this frame.
struct Fix {
    std::size_t slot = 0;
    Found found;
    /// From the constrained few-LED path.
    bool few = false;
    /// The pose to publish when it is not the optical fit itself (ImuTracker: the filter's pose after the update).
    std::optional<cam::geom::Pose> published;
};

class Tracker {
public:
    explicit Tracker(TrackCfg cfg);

    TrackCfg cfg;
    Stats stats;
    /// Slots to look for (left, right). A controller put down (Monado's PIMAX_CTRL_INHAND status) is neither tracked
    /// nor searched, and fixes assigned to it are dropped: less CPU, and its LEDs on the desk are never published.
    std::array<bool, 2> wanted{true, true};
    /// The fuser's hint per slot for the next step (set by the caller before step; TrackCfg::use_hint).
    std::array<std::optional<Hint>, 2> hint{};
    /// The aim frame of the IMU in the LED model frame (live: rot_x(-aim pitch)).
    cam::geom::Mat3 device_from_aim = cam::geom::IDENTITY;
    /// A controller's IMU sample (in time order; kept 3 s). TrackCfg::imu_id.
    void imu_push(std::size_t slot, const ImuSample& s);
    /// The identity error of a pose for a slot at t: (kind, deg) with kind 'r' (turn vs gyro) or 't' (tilt), or
    /// nullopt when the IMU cannot tell.
    std::optional<std::pair<char, double>> imu_error(std::size_t slot, const cam::geom::Mat3& r, std::uint64_t t) const;
    /// Why the IMU refuses a pose for a slot at t (nullptr = accepted).
    const char* imu_refuse(std::size_t slot, const cam::geom::Pose& p, std::uint64_t t) const;

    /// The rotation of the slot's last fix and its frame time, kept while the slot is lost or back in init: the
    /// orientation the gyro carries on from until a new fix replaces it.
    std::optional<std::pair<cam::geom::Mat3, std::uint64_t>> carried_rotation(std::size_t slot) const {
        return last_rot_[slot];
    }
    /// Last fix of a slot (model, body_from_model, frame ns).
    std::optional<std::tuple<std::size_t, cam::geom::Pose, std::uint64_t>> last(std::size_t slot) const;
    /// Process one LED frame (blobs per camera in lf, host time frame_ns).
    std::vector<Fix> step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns);

private:
    struct SlotState {
        std::size_t model = 0;
        cam::geom::Pose pose;
        std::uint64_t ns = 0;
        /// (linear m/s, angular rad/s as a rotation vector in the body frame)
        std::optional<std::pair<cam::geom::Vec3, cam::geom::Vec3>> vel;
        std::uint32_t few_streak = 0;
    };
    cam::geom::Pose predicted(const SlotState& st, std::uint64_t frame_ns) const;
    std::optional<std::pair<Found, bool>> track_slot(const Rig& rig, const std::vector<CamObs>& obs,
                                                     const std::vector<std::vector<bool>>& used,
                                                     const SlotState& st, std::uint64_t frame_ns,
                                                     const Hint* h = nullptr) const;
    std::optional<JointSolve> few_led(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                                      const std::vector<std::vector<bool>>& used, const cam::geom::Pose& pred,
                                      const std::vector<Matches>& per_cam, double dt, double cos_face,
                                      const Hint* h = nullptr) const;

    std::array<std::optional<SlotState>, 2> prev_{};
    std::array<std::optional<std::pair<std::array<double, 3>, std::uint64_t>>, 2> last_seen_{};
    std::uint64_t last_fix_ns_ = 0;
    std::uint64_t frame_no_ = 0;
    Body body_{};
    FailCount fails_{};
    std::array<std::optional<std::pair<cam::geom::Mat3, std::uint64_t>>, 2> last_rot_{};
    std::array<std::deque<ImuSample>, 2> imu_{};
    std::array<std::optional<std::pair<cam::geom::Vec3, std::uint64_t>>, 2> last_pos_{};
    /// The last fix of each slot not fitted with the gyro's orientation: the identity test's reference.
    std::array<std::optional<std::pair<cam::geom::Mat3, std::uint64_t>>, 2> anchor_{};
    const ImuSample* imu_near(std::size_t slot, std::uint64_t t) const;
    /// The slot's orientation at t: its last fix turned by its gyro since (TrackCfg::imu_id).
    std::optional<cam::geom::Mat3> imu_predict(std::size_t slot, std::uint64_t t) const;
    /// Fit the slot's model at `at` with the gyro's orientation as the rotation prior (a refused mirror fit).
    std::optional<JointSolve> imu_refit(const Rig& rig, const std::vector<CamObs>& obs,
                                        const std::vector<std::vector<bool>>& used, std::size_t slot,
                                        const cam::geom::Vec3& at, std::uint64_t t) const;
};

/// Pixel matches of `model` at `pose` in every camera.
std::vector<Matches> match_all(const cam::geom::Pose& pose, const Rig& rig, const LedModel& model,
                               const std::vector<CamObs>& obs, const std::vector<std::vector<bool>>& used,
                               double cos_face, double tol_px);

/// Facing LEDs of `model` at `pose` that project inside each camera's image, and the blobs within `near_px` of one
/// of them (loss classification).
std::pair<std::size_t, std::size_t> visibility(const Rig& rig, std::size_t model_i, const cam::geom::Pose& pose,
                                               const std::vector<std::vector<std::pair<double, double>>>& px,
                                               double near_px);

} // namespace pibridge::ctrl::track
