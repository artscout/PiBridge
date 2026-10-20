// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/fixation.rs (MIT OR Apache-2.0).
//
// Picking the fixation samples out of a target window: blinks and their edges, saccades, and the steadiest stretch of
// fixation. (libpibridge-eyemath leaves fixation.rs out: only the calibration tool uses it.)
//
// The raw 120 Hz gaze of this tracker has about 0.7 degrees of noise per sample and axis, which makes a plain
// sample-to-sample velocity useless (noise alone gives ~100 deg/s). The velocity is therefore taken on a running
// median (7 samples, +-25 ms: it keeps the step of a saccade sharp and removes the noise) over a +-2 sample baseline,
// with a 50 deg/s threshold: target jumps of 5 degrees and more (peak velocity > 150 deg/s) are found, the noise is
// not. A fixation is a run of non-saccade, non-blink samples whose smoothed track stays within `max_dispersion_deg`.
//
// Works offline on a whole window (it may look both ways in time): the samples just before a blink are corrupted by
// the closing lid as much as those just after it.
#pragma once

#include "pibridge/eyemath/types.h"

#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::eyecal {

using pibridge::eyemath::Tangent;
using OptTan = std::optional<Tangent>;

/// Settings.
struct FixParams {
    /// Faster than this (on the smoothed track) is a saccade, degrees per second.
    double velocity_dps = 50.0;
    /// Samples this close in time to a blink are dropped (seconds).
    double blink_edge_s = 0.06;
    /// A gap of invalid samples this long (from the last valid sample to the next) is a blink, shorter ones are
    /// dropped frames (seconds).
    double min_blink_s = 0.05;
    /// Unusable stretches up to this long do not end a fixation (seconds).
    double max_gap_s = 0.3;
    /// Samples next to a saccade sample are dropped too (count on each side).
    std::size_t saccade_pad = 2;
    /// Largest spread of the smoothed track during a fixation (degrees, max over u and v of the range).
    double max_dispersion_deg = 2.0;
    /// A fixation must last this long (seconds).
    double min_duration_s = 0.25;

    bool operator==(const FixParams&) const = default;
};

/// What a sample is.
enum class Label {
    /// No valid direction.
    Invalid,
    /// Valid but next to a blink / lost frame.
    BlinkEdge,
    /// Part of (or next to) a saccade.
    Saccade,
    /// Usable.
    Fixation,
};

/// A half-open index range `[start, end)` (the Rust `Range<usize>`).
struct Range {
    std::size_t start = 0;
    std::size_t end = 0;
    bool contains(std::size_t i) const { return i >= start && i < end; }
    bool operator==(const Range&) const = default;
};

/// Running median (half width `h` samples) over the valid samples; empty where the sample itself is invalid.
std::vector<OptTan> smooth(std::span<const OptTan> tan, std::size_t h);

/// Label every sample of one eye: `t` seconds, `tan` the gaze tangent (empty = invalid).
std::vector<Label> label(std::span<const double> t, std::span<const OptTan> tan, const FixParams& p);

/// The longest (in time) run of `usable` samples whose smoothed track stays within `max_dispersion_deg`; empty when
/// nothing lasts `min_duration_s`. Indices into the inputs.
std::optional<Range> longest_fixation(std::span<const double> t, std::span<const OptTan> tan,
                                      const std::vector<bool>& usable, const FixParams& p);

/// Component-wise median of tangents and the median absolute deviation (radius, tangent units).
std::optional<std::pair<Tangent, double>> median_tangent(std::span<const Tangent> v);

/// RMS of the angle between consecutive samples (degrees): the usual precision figure (RMS-S2S).
double rms_s2s_deg(std::span<const Tangent> v);

/// Standard deviation of the angle from the mean direction (degrees): precision as SD.
double sd_deg(std::span<const Tangent> v);

/// Median (NaN for an empty list), as fixation.rs, calfit.rs and refit.rs compute it.
double median_nan(std::vector<double> x);

} // namespace pibridge::eyecal
