// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/calfit.rs (MIT OR Apache-2.0).
//
// Sample-level fit of the per-eye calibration (with the origin terms of pibridge::eyemath::OriginTerms). libpibridge-eyemath
// leaves calfit.rs out: only the calibration tool fits.
//
// pibridge::eyemath::select_model fits one median point per target (13 points per eye). That cannot see what limits the
// tracker: the reported direction moves about 6 degrees per millimetre of error in the reported eye origin (see
// pibridge/eyemath/calib.h), and the origin wanders by 1-3 mm within one session. Here every cleaned fixation sample is a
// data point `(raw tangent, origin deviation) -> target tangent`, so the origin coefficients are identified by the
// sample-to-sample and target-to-target origin variation, and the polynomial stays small.
//
// * features: `1, u, v` (+ `u*u, u*v, v*v` for the quadratic model) and, with origin terms, the three origin
//   deviations in millimetres,
// * each target counts the same (weights sum to the target's weight, whatever its sample count),
// * Huber IRLS on the sample residuals (HUBER_SAMPLE), ridge relative to each feature's own information,
// * the model is chosen by leave-one-target-out error of the per-target median, simpler candidates preferred unless a
//   richer one is clearly better (same rule as select_model).
#pragma once

#include "pibridge/eyemath/calib.h"

#include <array>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::eyecal::calfit {

using pibridge::eyemath::ModelFit;
using pibridge::eyemath::ModelKind;
using pibridge::eyemath::Ridge;
using pibridge::eyemath::Selection;
using pibridge::eyemath::Tangent;
using pibridge::eyemath::Vec3;

/// Huber threshold on a sample residual, tangent units (about 1.7 degrees).
inline constexpr double HUBER_SAMPLE = 0.03;

/// One fixation sample of one eye.
struct SamplePt {
    /// Raw tangent (u, v).
    Tangent raw;
    /// Reported origin minus the reference origin, millimetres.
    Vec3 org_mm{0.0, 0.0, 0.0};
    /// Reported pupil diameter minus the reference, millimetres (clamped to +-PUPIL_CLAMP_MM).
    double pupil_mm = 0.0;
    bool operator==(const SamplePt&) const = default;
};

/// All the cleaned samples of one eye at one target.
struct TargetSamples {
    /// Tangent of the direction from the eye's rendering position to the target.
    Tangent truth;
    /// The samples.
    std::vector<SamplePt> samples;
    /// Relative weight of the target (1 is normal).
    double weight = 1.0;
    bool operator==(const TargetSamples&) const = default;
};

/// One candidate.
struct Spec {
    /// Polynomial size.
    ModelKind kind = ModelKind::Affine;
    /// Whether the origin deviation is a feature.
    bool org = false;
    /// Ridge on the polynomial terms.
    Ridge ridge;
    /// Relative ridge on the origin coefficients.
    double org_ridge = 0.005;
    /// Whether the pupil diameter deviation is a feature (only with `org`).
    bool pupil = false;
};

/// A fitted candidate: polynomial map and origin coefficients (per millimetre).
struct Fitted {
    /// The polynomial part.
    pibridge::eyemath::Correction map;
    /// Tangent per millimetre of origin deviation, `u`.
    Vec3 org_u{0.0, 0.0, 0.0};
    /// Tangent per millimetre of origin deviation, `v`.
    Vec3 org_v{0.0, 0.0, 0.0};
    /// Tangent per millimetre of pupil deviation, `(u, v)`.
    Tangent pupil_uv;

    /// Corrected tangent of one sample.
    Tangent apply(const SamplePt& p) const;
    bool operator==(const Fitted&) const = default;
};

/// Median corrected tangent of a target's samples.
std::optional<Tangent> target_center(const Fitted& f, const TargetSamples& t);
/// Error in degrees of the corrected median of a target.
std::optional<double> target_error_deg(const Fitted& f, const TargetSamples& t);
/// Weighted ridge least squares plus Huber IRLS on all samples of `targets`.
std::optional<Fitted> fit(std::span<const TargetSamples* const> targets, const Spec& spec);
/// Fit on all targets and score by leave-one-target-out.
std::optional<std::pair<ModelFit, Fitted>> evaluate(std::span<const TargetSamples> targets, const Spec& spec);
/// The candidate list, simplest first.
std::vector<Spec> candidates(bool with_pupil);

/// Result of select().
struct SampleSelection {
    /// The usual selection record (raw error, candidates, chosen index).
    Selection selection;
    /// The fitted maps, aligned with `selection.candidates`.
    std::vector<Fitted> fitted;
    /// The chosen map (nullptrs for none).
    std::pair<const ModelFit*, const Fitted*> best() const;
};

/// Fit all candidates and keep the one with the lowest leave-one-out mean error, unless a simpler one is within 7 %
/// (or 0.05 degrees) of it. Candidates with origin terms are only allowed when the origin actually varies
/// (`min_org_sd_mm`, over all samples).
SampleSelection select(std::span<const TargetSamples> targets, double min_org_sd_mm, bool with_pupil);

/// Largest standard deviation over the three origin axes of all samples, millimetres.
double org_sd_mm(std::span<const TargetSamples> targets);

} // namespace pibridge::eyecal::calfit
