// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/filter.rs (MIT OR Apache-2.0).
// Smoothing primitives: One Euro filter (analog inputs) and a critically damped smoother (thumb target). Both work on
// f32 with an explicit dt in seconds, never read a clock and never fail: non-finite inputs and non-positive dt leave
// the state untouched.
#pragma once

#include <optional>

namespace pibridge::ctrl {

/// One Euro filter (Casiez, Roussel, Vogel, CHI 2012): a low-pass whose cutoff rises with the filtered speed.
/// `cutoff = min_cutoff + beta * |filtered speed|`, speed in units/s, cutoffs in Hz.
struct OneEuro {
    float min_cutoff;
    float beta;
    float d_cutoff;
    float x_hat = 0.0f;
    float dx_hat = 0.0f;
    bool primed = false;

    OneEuro(float min_cutoff_, float beta_, float d_cutoff_) : min_cutoff(min_cutoff_), beta(beta_), d_cutoff(d_cutoff_) {}
    /// Forget the history; the next sample passes through unchanged.
    void reset();
    /// Start from `x` right away.
    void prime(float x);
    /// Latest output, nullopt before the first sample.
    std::optional<float> value() const { return primed ? std::optional<float>(x_hat) : std::nullopt; }
    /// Filtered speed estimate in units/s.
    float speed() const { return dx_hat; }
    /// Feed one sample taken `dt` seconds after the previous one; returns the filtered value.
    float update(float x, float dt);
};

/// Critically damped smoother toward a moving target (Unity's SmoothDamp): no overshoot, speed limited.
struct SmoothDamp {
    float smooth_time;
    float max_speed;
    float value_ = 0.0f;
    float velocity = 0.0f;

    SmoothDamp(float initial, float smooth_time_, float max_speed_);
    float value() const { return value_; }
    /// Jump to `v` with zero velocity.
    void snap(float v);
    float update(float target, float dt);
};

} // namespace pibridge::ctrl
