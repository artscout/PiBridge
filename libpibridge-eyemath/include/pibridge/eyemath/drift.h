// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/drift.rs (MIT OR Apache-2.0).
//
// Continuous drift correction: a slowly updated per-eye offset (tangent units) on top of the calibration, fed by
// "the wearer was looking at this" events (a recentre dot, a confirmed click on a UI element).
//
// Each observation moves the offset a fraction `gain` towards the observed residual (truth minus calibrated gaze);
// residuals larger than `max_residual_deg` are ignored (the wearer was not looking there), and the total offset is
// bounded by `max_offset_deg` (beyond that the headset has moved too much: calibrate again).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "pibridge/eyemath/types.h"

namespace pibridge::eyemath {

/// Settings.
struct DriftParams {
    /// Fraction of a residual applied per observation.
    double gain = 0.3;
    /// Larger residuals are not believed (degrees).
    double max_residual_deg = 5.0;
    /// Bound of the accumulated offset (degrees).
    double max_offset_deg = 4.0;

    friend bool operator==(const DriftParams&, const DriftParams&) = default;
};

/// The running offsets of both eyes.
struct Drift {
    /// Per eye `[left, right]`, tangent units, added after the calibration map.
    std::array<Tangent, 2> offset{};
    /// Observations accepted per eye.
    std::array<std::uint32_t, 2> accepted{0, 0};
    /// Observations rejected per eye.
    std::array<std::uint32_t, 2> rejected{0, 0};

    /// One observation for `eye`: calibrated gaze tangent (without this offset) and the true tangent. Returns
    /// whether it was used.
    bool observe(const DriftParams& p, std::size_t eye, Tangent gaze, Tangent truth);
    /// Apply the offset of `eye` to a calibrated tangent.
    Tangent apply(std::size_t eye, Tangent t) const;
    /// Unit direction of apply().
    Vec3 apply_dir(std::size_t eye, Tangent t) const;

    friend bool operator==(const Drift&, const Drift&) = default;
};

} // namespace pibridge::eyemath
