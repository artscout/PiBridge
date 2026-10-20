// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/filter.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/filter.h"

#include "pibridge/ctrl/rust.h"

#include <cmath>

namespace pibridge::ctrl {

namespace {

constexpr float k_pi = 3.14159265358979323846264338327950288f;

/// Smoothing factor of a first-order low-pass with the given cutoff, for a step of `dt` seconds.
float alpha(float cutoff, float dt) {
    const float tau = 1.0f / (2.0f * k_pi * rs::fmaxf(cutoff, 1e-3f));
    return dt / (dt + tau);
}

} // namespace

void OneEuro::reset() {
    primed = false;
    x_hat = 0.0f;
    dx_hat = 0.0f;
}

void OneEuro::prime(float x) {
    if (std::isfinite(x)) {
        x_hat = x;
        dx_hat = 0.0f;
        primed = true;
    }
}

float OneEuro::update(float x, float dt) {
    if (!std::isfinite(x)) {
        return x_hat;
    }
    if (!primed) {
        prime(x);
        return x_hat;
    }
    if (!(std::isfinite(dt) && dt > 0.0f)) {
        return x_hat;
    }
    const float dx = (x - x_hat) / dt;
    dx_hat += alpha(d_cutoff, dt) * (dx - dx_hat);
    const float cutoff = min_cutoff + beta * std::fabs(dx_hat);
    x_hat += alpha(cutoff, dt) * (x - x_hat);
    return x_hat;
}

SmoothDamp::SmoothDamp(float initial, float smooth_time_, float max_speed_)
    : smooth_time(smooth_time_), max_speed(max_speed_), value_(std::isfinite(initial) ? initial : 0.0f) {}

void SmoothDamp::snap(float v) {
    if (std::isfinite(v)) {
        value_ = v;
        velocity = 0.0f;
    }
}

float SmoothDamp::update(float target, float dt) {
    if (!(std::isfinite(target) && std::isfinite(dt) && dt > 0.0f)) {
        return value_;
    }
    const float smooth = rs::fmaxf(smooth_time, 1e-4f);
    const float omega = 2.0f / smooth;
    const float x = omega * dt;
    const float ex = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
    const float max_change = rs::fmaxf(max_speed, 0.0f) * smooth;
    const float change = rs::clampf(value_ - target, -max_change, max_change);
    const float clamped_target = value_ - change;
    const float temp = (velocity + omega * change) * dt;
    velocity = (velocity - omega * temp) * ex;
    float out = clamped_target + (change + temp) * ex;
    // Never overshoot the (unclamped) target.
    if ((target - value_ > 0.0f) == (out > target)) {
        out = target;
        velocity = 0.0f;
    }
    value_ = out;
    return out;
}

} // namespace pibridge::ctrl
