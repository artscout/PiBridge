// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/refit.rs (MIT OR Apache-2.0): the sample-level fit the live procedure uses
// (eye_data, fit_eye). The offline `--refit RUN.json` of the Rust tool is not ported (docs/eyecal.md).
//
// The old fit used one median point per target and the gaze direction only. The tracker's direction error follows
// its reported eye-origin error (about 6 degrees per millimetre, see pibridge/eyemath/calib.h), so the fit here works on
// the fixation samples with the origin as a feature (calfit.h) and the calibration drops or down-weights an eye whose
// origin is far from where this wearer's eye normally is.
#pragma once

#include "calfit.h"
#include "collect.h"
#include "layout.h"

#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::eyecal {

/// How samples are taken out of a target window.
struct ExtractOpts {
    /// Seconds at the start of the window that are dropped (the gaze settles after the saccade).
    double skip_s = 0.25;
    /// Keep only the longest fixation stretch (velocity threshold) instead of every non-blink sample.
    bool ivt = false;
    /// Samples whose origin is farther than this from the eye's reference origin are dropped (millimetres); also
    /// the gate written into the calibration. 0: none.
    double gate_mm = 5.0;
    /// Fewest samples for a target to count.
    std::size_t min_samples = 20;
    /// Offer the pupil-diameter candidates (the pupil centre moves with the pupil size; only taken when the
    /// leave-one-out error is clearly better).
    bool pupil = false;
};

/// One eye's cleaned samples.
struct EyeData {
    /// Targets with enough samples.
    std::vector<calfit::TargetSamples> targets;
    /// Window index of each entry of `targets`.
    std::vector<std::size_t> window_index;
    /// Reference origin (median of the kept samples), metres.
    Vec3 reference_m{0.0, 0.0, 0.0};
    /// Reference pupil diameter (median of the kept samples), millimetres.
    double pupil_ref_mm = 0.0;
    /// Sample-to-sample noise (RMS), degrees, median over the windows with a fixation.
    double precision_deg = 1.0;
};

/// A calibration target and its recorded window.
using Window = std::pair<Target, std::span<const RawSample>>;

/// Cleaned samples of `eye` in the windows of the calibration targets.
EyeData eye_data(std::span<const Window> wins, std::size_t eye, const Vec3& eye_pos, const ExtractOpts& opts,
                 const FixParams& fix);

/// The result of fitting one eye.
struct EyeFitResult {
    /// The calibration of the eye.
    pibridge::eyemath::EyeCal cal;
    /// Candidates and the chosen one (same record as the point fit).
    pibridge::eyemath::Selection selection;
    /// Targets used.
    std::size_t points = 0;
    /// Window index of every target of `selection`'s fits, with the leave-one-out error of the chosen model.
    std::vector<std::pair<std::size_t, double>> loo_by_window;
    /// Whether the chosen model has origin terms.
    bool uses_origin = false;
};

/// Select and fit one eye from its cleaned samples; empty with fewer than 4 targets.
std::optional<EyeFitResult> fit_eye(const EyeData& data, const Vec3& eye_pos, const ExtractOpts& opts);

/// A one-line description of the chosen model, e.g. `affine + origin`.
std::string model_name(const EyeFitResult& r);

} // namespace pibridge::eyecal
