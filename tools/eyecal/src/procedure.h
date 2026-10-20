// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/procedure.rs (MIT OR Apache-2.0); the on-chip mode (Mode::Chip) is new.
//
// The calibration / validation procedure as a time-driven state machine. No OpenXR, no USB: the caller feeds the raw
// tracker samples, the head orientation and the wearer's commands, draws the Scene it returns, and carries out its
// Effects (print, save, send a chip request, quit). The chip's answers come back through chip_answer().
//
// What the wearer does (host fit, Mode::Full, as the Rust tool):
//  1. Intro: keep the head still, look at the dot. The screen says whether both eyes are seen.
//  2. Calibration: 9 (or 13, or the 11 of the Pimax path) targets one after the other. Each target appears with a
//     shrinking ring (the Pimax path: the green dot travels there), the procedure waits until the gaze is steady,
//     records, and turns the dot green. If the head turns more than Settings::head_limit_deg meanwhile, the target
//     says so and starts over.
//  3. The fit (per eye, the sample-level fit of refit.h); a mirrored axis is detected and the tracker restarted.
//  4. Validation: 10 *other* targets measured the same way; their errors are the reported accuracy.
//  5. Optional smooth pursuit; results, live cursor, save, the end-to-end check through the runtime.
//
// The on-chip calibration (Mode::Chip, what Pimax Play does; docs/eyecal.md):
//  1. The check grid (Phase::Kind::Grid): xr_grid's head-locked grid and one dot, the combined gaze as pibridge-eyed would
//     publish it (the tool's raw gaze through the host map pibridge-eyed loads, Settings::view, and its filter).
//     A calibrate (waits until the eye tracker streams), B exit.
//  2. The chip's two passes in the captured order (pibridge::eye::calibration::sequence()): the green dot travels to
//     each of the nine captured targets (pass 1 on black, pass 2 on white); when the gaze is steady on it the target
//     request 0x407 goes out, the procedure waits for the chip's answer, then the dot moves on. Start, compute,
//     download and finish go out between the targets. B stops (the open pass is finished with 0x3fc) and goes back
//     to the grid; nothing is kept.
//  3. The last download is the new profile (Effect::SaveProfile, not kept yet).
//  4. Validation with the new chip calibration: the 5 recentre targets, raw gaze against the targets, per eye.
//  5. End screen: A again, B save = keep the new profile (Effect::KeepProfile) and save the host map (identity per
//     eye, plus the constant offset the validation found as drift, as the manual --recenter of 2026-10-18 did), then
//     back to the grid (Phase::Kind::GridSaved), whose dot now uses the new host map (the chip uses its new
//     calibration from its 0x42e on). B there exits.
// --recenter (Pimax path) has the same frame: grid, A the 5 targets, end screen A again / B save, back to the grid.
// Anything that ends the tool without B on an end screen leaves the profile of the last save (or the one before the
// tool) in place: the caller puts it back (ProfileStore::rollback).
#pragma once

#include "collect.h"
#include "fixation.h"
#include "layout.h"

#include "pibridge/eye/calibration.h"
#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/filter.h"
#include "pibridge/eyemath/track.h"

#include <array>
#include <cstdint>
#include <deque>
#include <expected>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::eyecal {

namespace em = pibridge::eyemath;

/// The calibration path.
enum class Pattern {
    /// The 3x3 (or 13-point) grid on a dark background, a shrinking ring at each target.
    Grid,
    /// Pimax Play's path (pimax_targets): the dot travels between the targets, the second half on white; the fit
    /// also tries the pupil-size terms. The end screen has two buttons: A again, B save and exit.
    Pimax,
};

/// What the run does.
enum class Mode {
    /// Host fit: calibrate, validate, save.
    Full,
    /// Five targets: update the drift offsets of the existing calibration.
    Recenter,
    /// The chip's own calibration (two passes, nine captured targets), then a short validation.
    Chip,
};

/// Settings of a run.
struct Settings {
    /// 9 or 13 calibration targets (the grid pattern).
    std::size_t cal_points = 9;
    /// Which calibration path.
    Pattern pattern = Pattern::Grid;
    /// Run the smooth-pursuit test after the validation.
    bool pursuit = false;
    /// Device-to-head conversion of the raw tracker (the Crystal Super: flip x).
    em::FrameConv conv = [] {
        em::FrameConv c;
        c.flip_x = true;
        return c;
    }();
    /// Largest head rotation during one target, degrees.
    double head_limit_deg = 2.5;
    /// Largest head angular speed during one target, deg/s.
    double head_speed_dps = 25.0;
    /// Attract animation, seconds.
    double attract_s = 0.7;
    /// Earliest start of the recording after the target appeared, seconds.
    double min_hold_s = 0.45;
    /// Latest start (recording starts even without a steady gaze), seconds.
    double max_hold_s = 2.0;
    /// Recording per target, seconds.
    double collect_s = 1.0;
    /// Host fit, recentre, or the chip's own calibration.
    Mode mode = Mode::Full;
    /// The calibration in use (recentre / comparison).
    std::optional<em::Calibration> existing;
    /// Output filter written into the calibration.
    em::FilterParams filter;
    /// Fixation detection.
    FixParams fix;
    /// Mirrors of the chip's target table (first live test only; ChipLayout).
    bool chip_mirror_x = false;
    bool chip_mirror_y = false;
    /// The host map pibridge-eyed applies (the check grid's gaze dot); empty: identity per eye (raw gaze). Replaced by
    /// the saved one at every save.
    std::optional<em::Calibration> view;
    /// pibridge-eyed's filter override (PIBRIDGE_EYE_FILTER); empty: the host map's `[filter]`, off without a host map.
    std::optional<em::FilterParams> view_filter;

    /// The settings of the Pimax path: longer recording per target (Pimax holds each 3-5 s), a longer travel.
    Settings pimax() const;
    /// The on-chip calibration (the Pimax look: the dot travels for 1 s).
    Settings chip() const;
};

/// Which list a target belongs to.
enum class Set {
    Calibration,
    Validation,
    Recenter,
    /// Check through the runtime (combined gaze from OpenXR).
    Service,
    /// The chip's own targets (index = the captured target).
    Chip,
};
const char* set_name(Set s);

/// Sub-stage of one target.
enum class Stage {
    /// Ring shrinking onto the target (the dot travelling to it).
    Attract,
    /// Waiting for a steady gaze.
    Hold,
    /// Recording (chip targets: the request is out, waiting for the chip's answer).
    Collect,
    /// Recorded, green flash.
    Accepted,
    /// The head moved: message, then start over.
    HeadMoved,
};
const char* stage_name(Stage s);

/// Where the procedure is.
struct Phase {
    enum class Kind {
        /// Instructions, eye status.
        Intro,
        /// A target.
        Point,
        /// Smooth pursuit.
        Pursuit,
        /// Results.
        Results,
        /// Live cursor.
        Cursor,
        /// Waiting for the runtime's gaze (end-to-end check starting).
        ServiceWait,
        /// Cursor through the runtime.
        ServiceCursor,
        /// A chip request other than a target is out (start, compute, download, finish).
        ChipBusy,
        /// The check grid with the gaze dot, before anything was saved (the chip mode and --recenter start here).
        Grid,
        /// The check grid after a save: the gaze dot uses the new calibration.
        GridSaved,
    };
    Kind kind = Kind::Intro;
    /// Point: list, index, sub-stage, attempts.
    Set set = Set::Calibration;
    std::size_t i = 0;
    Stage stage = Stage::Attract;
    /// Time the stage (or phase) started.
    double t0 = 0.0;
    std::uint32_t attempt = 0;

    static Phase intro() { return {}; }
    static Phase point(Set set, std::size_t i, Stage stage, double t0, std::uint32_t attempt) {
        return Phase{Kind::Point, set, i, stage, t0, attempt};
    }
    static Phase of(Kind k, double t0 = 0.0) { return Phase{k, Set::Calibration, 0, Stage::Attract, t0, 0}; }
    bool is(Kind k) const { return kind == k; }
    bool operator==(const Phase&) const = default;
};
const char* phase_name(Phase::Kind k);

/// What the wearer asked for.
enum class Cmd {
    /// Start / continue (space, A).
    Next,
    /// Start over (R, B); in the Pimax path and the chip mode: exit / save and exit / stop.
    Redo,
    /// Save (Y, menu).
    Accept,
    /// Smooth pursuit test (P).
    Pursuit,
    /// Toggle the live cursor (C).
    Cursor,
    /// End-to-end check through the runtime (S).
    Service,
    /// The trigger: starts from the intro (in the Pimax path nothing else, so a squeeze never restarts a run).
    Start,
};

/// Things for the caller to do.
struct Effect {
    enum class Kind {
        /// A line for the terminal (`text`).
        Print,
        /// Write the results JSON (`text`).
        SaveResults,
        /// Write the calibration file (TOML `text`).
        SaveCalibration,
        /// Restart the raw tracker with `conv` (a mirrored axis was found).
        RestartTracker,
        /// Release the chip and read the gaze through the runtime.
        StartService,
        /// Back to the own tracker.
        StopService,
        /// Close the app.
        Quit,
        /// Send `step` to the chip and hand its answer to chip_answer().
        ChipRequest,
        /// The chip's new profile (`profile`): save it (with a backup of the old one), not kept yet.
        SaveProfile,
        /// Keep the saved profile (otherwise the caller puts the old one back when the run ends).
        KeepProfile,
    };
    Kind kind = Kind::Print;
    std::string text;
    em::FrameConv conv;
    pibridge::eye::calibration::Step step;
    pibridge::eye::Bytes profile;

    static Effect print(std::string s) { return Effect{Kind::Print, std::move(s), {}, {}, {}}; }
    static Effect of(Kind k) { return Effect{k, {}, {}, {}, {}}; }
    static Effect with_text(Kind k, std::string s) { return Effect{k, std::move(s), {}, {}, {}}; }
    static Effect restart(em::FrameConv c) { return Effect{Kind::RestartTracker, {}, c, {}, {}}; }
    static Effect chip(pibridge::eye::calibration::Step s) { return Effect{Kind::ChipRequest, {}, {}, s, {}}; }
    static Effect save_profile(pibridge::eye::Bytes b) { return Effect{Kind::SaveProfile, {}, {}, {}, std::move(b)}; }
};

/// Shape of a mark.
enum class MarkKind {
    Disc,
    Ring,
    /// Plus sign (two bars).
    Cross,
};

using Rgb = std::array<float, 3>;

/// One thing to draw, head frame.
struct Mark {
    /// Position, metres (head frame).
    Vec3 pos{0.0, 0.0, -2.0};
    MarkKind kind = MarkKind::Disc;
    /// Radius (disc, ring) or half length (cross), degrees.
    double radius_deg = 0.0;
    /// Ring line width / bar width, degrees.
    double width_deg = 0.0;
    /// Colour (sRGB, 0..1).
    Rgb color{1.0F, 1.0F, 1.0F};
    bool operator==(const Mark&) const = default;
};

/// What to show this frame.
struct Scene {
    /// Marks, drawn in order.
    std::vector<Mark> marks;
    /// Text lines (head-locked panel below the centre).
    std::vector<std::string> hud;
    /// Background colour (sRGB).
    Rgb background{0.10F, 0.10F, 0.11F};
    /// Draw the check grid (head-locked, under the marks; marks.h).
    bool grid = false;
    /// The gaze dot (head-frame point), drawn over everything.
    std::optional<Vec3> gaze;
};

/// Dark grey: keeps the pupils mid-sized and the white targets high in contrast.
inline constexpr Rgb BACKGROUND = {0.10F, 0.10F, 0.11F};
/// Background of the white half of the Pimax path.
inline constexpr Rgb PIMAX_WHITE = {0.92F, 0.92F, 0.92F};

/// One recorded target.
struct PointRes {
    Target target;
    /// Per eye.
    std::array<std::optional<EyeWindow>, 2> eyes{};
    /// Samples recorded.
    std::vector<RawSample> samples;
    /// Largest head rotation during the target, degrees.
    double head_deg = 0.0;
    /// Attempts used.
    std::uint32_t attempts = 0;
    /// Usable (at least one eye with a fixation).
    bool ok = false;
};

/// One validated target.
struct ValRes {
    PointRes point;
    /// Errors of the new calibration.
    WindowEval eval;
    /// Errors of the raw gaze (same window, identity per-eye maps).
    WindowEval raw_eval;
};

/// Fit of one eye.
struct EyeFit {
    /// Model comparison.
    em::Selection selection;
    /// Points used.
    std::size_t points = 0;
};

/// Smooth-pursuit outcome.
struct PursuitRes {
    /// Lag giving the lowest error, seconds.
    double lag_s = 0.0;
    /// Error at that lag over the pursuit samples (degrees).
    em::Stats err;
    /// Fraction of samples used (no saccade, gaze valid).
    double used = 0.0;
};

/// One eye of the validation after the on-chip calibration.
struct ChipEyeCheck {
    /// Targets with a fixation.
    std::size_t n = 0;
    /// Mean error of the raw gaze (identity map), degrees.
    double before_deg = 0.0;
    /// Mean error once the constant offset is removed, degrees.
    double after_deg = 0.0;
    /// The constant offset (truth minus gaze, median over the targets), tangent units.
    Tangent offset;
    /// Its size, degrees.
    double offset_deg = 0.0;
    /// The offset is constant (removing it helps) and large enough to be worth a drift entry.
    bool apply = false;
    /// The gaze moves against the targets on that axis (the chip's targets were shown mirrored).
    bool mirrored_x = false;
    bool mirrored_y = false;
};

/// The validation of the on-chip calibration, per eye.
struct ChipCheck {
    std::array<std::optional<ChipEyeCheck>, 2> eyes{};
};

/// The constant-offset analysis of the validation windows (recentre targets, raw gaze): per eye with at least 3
/// targets. An offset is applied when it is at least 0.5 degrees and removing it leaves at most half the error (or
/// at most 1 degree).
ChipCheck check_chip_validation(const std::vector<PointRes>& windows, const std::array<Vec3, 2>& eye_pos);

/// The procedure.
class Procedure {
public:
    explicit Procedure(Settings s);

    /// Settings.
    Settings s;
    /// Current phase.
    Phase phase;
    /// Recorded calibration targets.
    std::vector<PointRes> cal_res;
    /// Recorded validation targets.
    std::vector<ValRes> val_res;
    /// Recentre recordings (also the validation of the on-chip calibration).
    std::vector<PointRes> rec_res;
    /// Recordings at the chip's targets (the window while each request was out).
    std::vector<PointRes> chip_res;
    /// Per-eye fits.
    std::array<std::optional<EyeFit>, 2> fits{};
    /// The new calibration (host map).
    std::optional<em::Calibration> calibration;
    /// Smooth pursuit.
    std::optional<PursuitRes> pursuit_res;
    /// End-to-end errors through the runtime, per service target (degrees).
    std::vector<std::optional<double>> service_err;
    /// Head-motion restarts.
    std::uint32_t head_restarts = 0;
    /// Saved to disk.
    bool saved = false;
    /// The chip's new profile (the last download), once the on-chip calibration completed.
    std::optional<pibridge::eye::Bytes> chip_profile;
    /// The validation of the on-chip calibration.
    std::optional<ChipCheck> chip_check;
    /// The on-chip calibration failed (status says why).
    bool chip_failed = false;

    /// Per-eye tracking ratio over the last second.
    std::array<double, 2> eye_ratio() const;
    /// A short name of the current target state, for the raw log.
    std::string tag() const;
    /// The command handler.
    std::vector<Effect> command(Cmd c);
    /// Advance to time `t` (seconds, the clock of the samples) with the new raw samples, the head orientation (LOCAL
    /// space quaternion x, y, z, w) and the eye rendering positions (VIEW space); `service` is the runtime's combined
    /// gaze direction (head frame) during the end-to-end check.
    std::vector<Effect> frame(double t, std::optional<std::array<double, 4>> head, const std::array<Vec3, 2>& eye_pos,
                              const std::vector<RawSample>& samples, std::optional<Vec3> service);
    /// The answer to the last Effect::ChipRequest (sent with `seq`), or why there is none.
    std::vector<Effect> chip_answer(std::uint32_t seq, const std::expected<pibridge::eye::calibration::Answer, std::string>& r);
    /// Whether the chip can take requests (the tracker streams and the calibration session is open). The caller keeps
    /// it up to date; A at the intro / grid waits for it.
    void set_chip_ready(bool ready);
    bool chip_ready() const { return chip_ready_; }
    /// A chip request is out and not answered yet.
    bool chip_waiting() const { return chip_inflight_.has_value(); }
    /// The app is closing: the request that ends an open chip pass (0x3fc), if one is open.
    std::vector<Effect> shutdown();
    /// The calibrated gaze cursor (head frame direction), when there is one.
    std::optional<Vec3> cursor() const { return cursor_; }
    /// The check grid's gaze (head frame direction): the raw samples through Settings::view and its filter, as
    /// pibridge-eyed publishes them; empty without a recent sample.
    std::optional<Vec3> gaze() const { return gaze_; }
    /// This run starts and ends in the check grid (the chip mode, and --recenter on the Pimax path).
    bool grid_flow() const;
    /// Chip requests sent and not answered yet (also those of a run that was stopped).
    std::uint32_t chip_pending() const { return chip_pending_; }
    /// What to draw now.
    Scene scene() const;
    /// Measuring now (the caller hides distractions).
    bool measuring() const;
    /// The eye rendering positions in use.
    const std::array<Vec3, 2>& eye_positions() const { return eye_pos_; }
    /// Calibration targets.
    const std::vector<Target>& calibration_targets() const { return cal_t_; }
    /// Validation targets.
    const std::vector<Target>& validation_targets() const { return val_t_; }
    /// The chip's targets as shown.
    const std::vector<Target>& chip_targets() const { return chip_t_; }
    /// The status line (empty: none).
    const std::string& status() const { return status_; }
    /// The plain-words summary of the on-chip calibration (terminal and end screen), empty before it ran.
    const std::vector<std::string>& chip_lines() const { return chip_lines_; }

private:
    Target target(Set set, std::size_t i) const;
    std::size_t set_len(Set set) const;
    std::vector<Effect> accept();
    std::size_t target_index(std::size_t k) const;
    void start_point(Set set, std::size_t i, std::uint32_t attempt);
    bool steady(double t) const;
    std::vector<Effect> finish_point(Set set, std::size_t i, std::uint32_t attempt);
    std::vector<Effect> next_point(Set set, std::size_t i);
    std::vector<std::pair<std::size_t, em::EyePoint>> eye_points(std::size_t e) const;
    std::vector<Effect> finish_calibration();
    em::Calibration make_calibration(em::EyeCal l, em::EyeCal r) const;
    std::vector<Effect> finish_validation();
    std::vector<Effect> finish_recenter();
    std::vector<Effect> finish_service();
    std::vector<Effect> finish_pursuit(double t0);
    void update_cursor(const std::vector<RawSample>& samples);
    std::vector<Effect> chip_command(Cmd c);
    std::vector<Effect> chip_advance();
    std::vector<Effect> chip_fail(const std::string& why);
    std::vector<Effect> finish_chip_validation();
    void reset_keep_session();
    std::vector<Effect> grid_command(Cmd c);
    std::vector<Effect> recenter_command(Cmd c);
    std::vector<Effect> save_and_grid();
    void to_grid(std::string status);
    Effect send(const pibridge::eye::calibration::Step& st);

    std::vector<Target> cal_t_;
    std::vector<Target> val_t_;
    std::vector<Target> chip_t_;
    std::array<Vec3, 2> eye_pos_{Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}};
    std::vector<RawSample> window_;
    std::deque<RawSample> recent_;
    std::optional<std::array<double, 4>> head_ref_;
    std::optional<std::pair<double, std::array<double, 4>>> head_prev_;
    double head_max_ = 0.0;
    double head_speed_ = 0.0;
    std::vector<RawSample> pursuit_samples_;
    std::vector<std::pair<double, Vec3>> service_samples_;
    std::string status_;
    em::CombineState cursor_combine_;
    em::OneEuro cursor_filter_;
    std::optional<Vec3> cursor_;
    std::optional<Vec3> raw_cursor_;
    std::optional<Vec3> service_cursor_;
    std::vector<std::size_t> redo_queue_;
    std::uint32_t redo_rounds_ = 0;
    bool in_redo_ = false;
    std::uint32_t axis_restarts_ = 0;
    double now_ = 0.0;
    // on-chip calibration
    pibridge::eye::calibration::Run chip_run_;
    std::optional<pibridge::eye::calibration::Step> chip_inflight_;
    bool chip_aborting_ = false;
    bool chip_ready_ = false;
    double chip_sent_at_ = 0.0;
    std::optional<std::size_t> chip_last_target_;
    std::vector<std::string> chip_lines_;
    std::uint32_t chip_pending_ = 0;
    // the check grid
    bool grid_saved_ = false;
    em::CombineState gaze_combine_;
    em::OneEuro gaze_filter_;
    std::optional<Vec3> gaze_;
};

/// `left` / `right`.
const char* eye_name(std::size_t e);

/// Regions of the validation targets with their combined errors.
std::vector<std::pair<Region, std::vector<double>>> region_errors(const Procedure& p);

} // namespace pibridge::eyecal
