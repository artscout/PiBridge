// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/filter.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/filter.h"

#include <cmath>
#include <numbers>

#include "pibridge/eyemath/correction.h"
#include "num.h"

namespace pibridge::eyemath {

namespace {

double alpha(double cutoff_hz, double dt) {
    const double tau = 1.0 / (2.0 * std::numbers::pi * detail::rmax(cutoff_hz, 1e-6));
    return 1.0 / (1.0 + tau / dt);
}

} // namespace

Vec2 OneEuro::filter(const FilterParams& p, double t, const Vec2& x) {
    if (!x_) {
        x_ = x;
        dx_ = {0.0, 0.0};
        t_ = t;
        return x;
    }
    const Vec2 prev = *x_;
    const double dt = t - t_;
    if (std::isnan(dt) || dt <= 1e-5 || dt > MAX_GAP_S) {
        x_ = x;
        dx_ = {0.0, 0.0};
        t_ = t;
        far_ = 0;
        return x;
    }
    t_ = t;
    // the raw value far from the output twice in a row: a saccade landed, snap
    const double ex = x[0] - prev[0];
    const double ey = x[1] - prev[1];
    const double off_deg = detail::to_degrees(std::atan(std::sqrt(ex * ex + ey * ey)));
    if (p.snap_deg > 0.0 && off_deg > p.snap_deg) {
        far_++;
        if (far_ >= 2) {
            far_ = 0;
            x_ = x;
            dx_ = {0.0, 0.0};
            return x;
        }
    } else {
        far_ = 0;
    }
    const double ad = alpha(p.d_cutoff_hz, dt);
    const Vec2 raw_dx{(x[0] - prev[0]) / dt, (x[1] - prev[1]) / dt};
    dx_ = {dx_[0] + ad * (raw_dx[0] - dx_[0]), dx_[1] + ad * (raw_dx[1] - dx_[1])};
    const double speed_dps = detail::to_degrees(std::sqrt(dx_[0] * dx_[0] + dx_[1] * dx_[1]));
    const double a = alpha(p.min_cutoff_hz + p.beta * speed_dps, dt);
    const Vec2 y{prev[0] + a * (x[0] - prev[0]), prev[1] + a * (x[1] - prev[1])};
    x_ = y;
    return y;
}

void filter_sample(const FilterParams& p, OneEuro& f, Sample& s) {
    if (!p.enabled) {
        return;
    }
    if (!s.gaze) {
        f.reset();
        return;
    }
    auto& g = *s.gaze;
    const Vec3 d{double{g.direction[0]}, double{g.direction[1]}, double{g.direction[2]}};
    const auto t = tangent_of(d);
    if (!t) {
        f.reset();
        return;
    }
    const double ts = static_cast<double>(s.host_ts_ns) * 1e-9;
    const auto y = f.filter(p, ts, {t->u, t->v});
    const auto o = dir_of(y[0], y[1]);
    g.direction = {static_cast<float>(o[0]), static_cast<float>(o[1]), static_cast<float>(o[2])};
}

} // namespace pibridge::eyemath
