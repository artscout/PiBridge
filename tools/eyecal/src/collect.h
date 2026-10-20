// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/collect.rs (MIT OR Apache-2.0).
//
// One target window: raw per-eye samples in, per-eye fixation statistics out, and the evaluation of a fitted
// calibration on a window (per eye and combined, the honest numbers of the validation).
#pragma once

#include "fixation.h"
#include "layout.h"

#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/track.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace pibridge::eyecal {

/// One eye of a raw sample (head frame after the device conversion, uncalibrated).
struct RawEye {
    /// Unit direction.
    Vec3 dir{0.0, 0.0, -1.0};
    /// Origin reported by the tracker, metres.
    Vec3 origin{0.0, 0.0, 0.0};
    /// Pupil diameter, mm (negative: unknown).
    double pupil_mm = 0.0;
    bool operator==(const RawEye&) const = default;
};

/// One raw sample.
struct RawSample {
    /// Host time, seconds (`CLOCK_MONOTONIC`).
    double t = 0.0;
    /// `[left, right]`, empty = not tracked.
    std::array<std::optional<RawEye>, 2> eyes{};

    /// From a tracker sample (raw tracker: no calibration).
    static RawSample from_sample(const pibridge::eyemath::Sample& s);
    /// Back to a tracker sample (for the calibration code); `t` becomes `host_ts_ns`.
    pibridge::eyemath::Sample to_sample(std::uint64_t seq) const;
    bool operator==(const RawSample&) const = default;
};

/// Fixation statistics of one eye in one window.
struct EyeWindow {
    /// Samples in the window.
    std::size_t total = 0;
    /// Samples with this eye tracked.
    std::size_t valid = 0;
    /// Samples used (inside the fixation, not blink edge / saccade).
    std::size_t used = 0;
    /// Duration of the fixation stretch, seconds.
    double fix_s = 0.0;
    /// Median raw tangent of the used samples.
    Tangent center;
    /// Median distance from the centre, degrees.
    double mad_deg = 0.0;
    /// RMS sample-to-sample, degrees.
    double rms_s2s_deg = 0.0;
    /// Standard deviation, degrees.
    double sd_deg = 0.0;
    /// The used raw tangents.
    std::vector<Tangent> used_tan;
    /// The reported origin of each used sample (metres), aligned with `used_tan`.
    std::vector<Vec3> used_origin;
    /// The time of each used sample (seconds), aligned with `used_tan`.
    std::vector<double> used_t;
    /// The reported pupil diameter of each used sample (mm), aligned with `used_tan`.
    std::vector<double> used_pupil;
    bool operator==(const EyeWindow&) const = default;
};

/// Analyse one eye of a window. Empty when no fixation of `min_duration_s` was found.
std::optional<EyeWindow> analyze_eye(std::span<const RawSample> w, std::size_t eye, const FixParams& p);

/// Tangent of the direction from `eye_pos` to the target point.
std::optional<Tangent> truth_tangent(const Target& target, const Vec3& eye_pos);

/// Errors of a calibration on one validation window.
struct WindowEval {
    /// Per eye: error of the calibrated fixation centre (degrees), empty when the eye had no fixation.
    std::array<std::optional<double>, 2> eye_err_deg{};
    /// Combined: error of the median calibrated combined gaze over the fixation (degrees).
    std::optional<double> comb_err_deg;
    /// Combined: mean over the fixation samples of the per-sample error (degrees; includes the noise).
    std::optional<double> comb_sample_err_deg;
    /// Combined precision, RMS sample-to-sample (degrees).
    double comb_rms_s2s_deg = 0.0;
    /// Combined precision, SD (degrees).
    double comb_sd_deg = 0.0;
    /// Fraction of samples without any combined gaze.
    double loss = 0.0;
    /// Per-sample combined errors (for pooled statistics).
    std::vector<double> sample_errs;
    bool operator==(const WindowEval&) const = default;
};

/// Calibrated centre of one eye on its fixation: the median of the per-sample calibrated tangents (each with its own
/// reported origin and pupil, samples beyond the origin gate dropped); empty with fewer than 5 samples left.
std::optional<Tangent> eye_center(const pibridge::eyemath::EyeCal& cal, const EyeWindow& ew);

/// Error of one calibrated eye on its fixation against `truth`, degrees (see eye_center).
std::optional<double> eye_error_deg(const pibridge::eyemath::EyeCal& cal, const EyeWindow& ew, Tangent truth);

/// Evaluate `cal` on a window for `target`.
WindowEval evaluate_window(const pibridge::eyemath::Calibration& cal, std::span<const RawSample> w, const Target& target,
                           const FixParams& p);

} // namespace pibridge::eyecal
