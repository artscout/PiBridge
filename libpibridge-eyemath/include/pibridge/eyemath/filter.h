// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/filter.rs (MIT OR Apache-2.0).
//
// Smoothing of the combined gaze for UI use: a One-Euro filter (Casiez et al. 2012) on the gaze tangent, with a
// saccade snap.
//
// One-Euro is a first-order low-pass whose cutoff rises with the speed: `fc = min_cutoff + beta * |speed|` (speed in
// degrees per second, itself low-passed at `d_cutoff`). At fixation it smooths the ~1 degree sample noise strongly,
// during a saccade it opens up. Because the speed estimate needs a few samples to rise, a saccade would still be
// followed with some lag; the snap removes it: when the raw gaze is more than `snap_deg` from the output for two
// consecutive samples, the filter jumps to the raw value (the eye has landed somewhere else).
//
// Defaults (`min_cutoff` 1 Hz, `beta` 0.03 per deg/s, `d_cutoff` 2 Hz, snap 3 degrees): about 0.2 degrees of residual
// jitter at fixation for 1 degree RMS input noise at 120 Hz, and no visible lag after saccades larger than 3 degrees.
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "pibridge/eyemath/track.h"
#include "pibridge/eyemath/types.h"

namespace pibridge::eyemath {

/// Filter settings (the `[filter]` section of the calibration file).
struct FilterParams {
    /// Off: the samples pass unchanged.
    bool enabled = true;
    /// Cutoff at rest, Hz.
    double min_cutoff_hz = 1.0;
    /// Cutoff increase per degree per second of speed.
    double beta = 0.03;
    /// Cutoff of the speed estimate, Hz.
    double d_cutoff_hz = 2.0;
    /// Jump straight to the raw gaze when it stays this far away (degrees, 0 = never).
    double snap_deg = 3.0;

    friend bool operator==(const FilterParams&, const FilterParams&) = default;
};

/// Samples further apart than this restart the filter.
inline constexpr double MAX_GAP_S = 0.1;

/// The filter state for one 2D signal in tangent units.
class OneEuro {
public:
    /// Filter `x` (tangent) at time `t` seconds.
    Vec2 filter(const FilterParams& p, double t, const Vec2& x);
    /// Forget the state (next sample passes unchanged).
    void reset() { *this = OneEuro{}; }

private:
    std::optional<Vec2> x_;
    Vec2 dx_{0.0, 0.0};
    double t_ = 0.0;
    std::uint32_t far_ = 0;
};

/// Filter the combined gaze direction of a sample in place (no gaze: the filter restarts). The Rust function is
/// `filter::apply_sample`.
void filter_sample(const FilterParams& p, OneEuro& f, Sample& s);

} // namespace pibridge::eyemath
