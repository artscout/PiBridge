// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/drift.rs (MIT OR Apache-2.0).
#include "pibridge/cam/drift.h"

#include "pibridge/cam/headpose.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace pibridge::cam::drift {

namespace rf = pibridge::cam::rfmt;
using headpose::G_WORLD;
using headpose::qexp;
using slam::qmul;
using slam::qnorm;
using slam::qrot;

DriftParams::DriftParams() : turn_rate(rf::to_radians(15.0)) {}

void VarWin::push(std::int64_t ts, const Vec3& v, std::int64_t window_ns) {
    q.emplace_back(ts, v);
    for (int k = 0; k < 3; k++) {
        s[k] += v[k];
        s2[k] += v[k] * v[k];
    }
    while (!q.empty() && q.front().first < ts - window_ns) {
        const Vec3 x = q.front().second;
        q.pop_front();
        for (int k = 0; k < 3; k++) {
            s[k] -= x[k];
            s2[k] -= x[k] * x[k];
        }
    }
    pushes += 1;
    if (pushes % 4096 == 0) {
        // Re-sum now and then: no rounding builds up over a session.
        s = {0.0, 0.0, 0.0};
        s2 = {0.0, 0.0, 0.0};
        for (const auto& e : q) {
            const Vec3& x = e.second;
            for (int k = 0; k < 3; k++) {
                s[k] += x[k];
                s2[k] += x[k] * x[k];
            }
        }
    }
}

double VarWin::spread() const {
    const double n = static_cast<double>(q.size());
    if (n < 100.0) {
        return 0.0;
    }
    // Iterator::sum of f64 starts at -0.0 (Rust >= 1.81); the terms are never -0.0, so the result is the same.
    double sum = -0.0;
    for (int k = 0; k < 3; k++) {
        const double m = s[k] / n;
        sum += rf::fmax(s2[k] / n - m * m, 0.0);
    }
    return std::sqrt(sum);
}

DriftGov::DriftGov(DriftParams pp, const Vec3& head_in_imu)
    : p(std::move(pp)), n_imu_(geom::add(head_in_imu, p.neck_in_head)) {}

bool DriftGov::turning() const {
    return last_turn_ns_ && last_ts_ && *last_ts_ - *last_turn_ns_ <= p.turn_hold_ns;
}

double DriftGov::allowed() const {
    const double ev = turning() ? rf::fmax(shake() - p.turn_noise, 0.0) : rf::fmax(evidence() - p.noise, 0.0);
    return p.floor + p.gain_s * ev;
}

void DriftGov::push_imu(const headpose::ImuSample& s, const Vec3& bias) {
    if (last_ts_ && s.ts_ns <= *last_ts_) {
        return;
    }
    const Vec3 w = geom::sub(s.gyro, bias);
    // Angular acceleration from a lightly smoothed rate (5 ms) over 20 ms: differentiating the raw 1 kHz rate would
    // turn the gyro's noise into ~0.2 m/s^2 of made-up acceleration at the neck.
    Vec3 ws = w;
    if (!gyro_hist_.empty() && last_ts_) {
        const Vec3 prev = gyro_hist_.back().second;
        const double k = rf::fmax(rf::fmin(static_cast<double>(s.ts_ns - *last_ts_) / 5e6, 1.0), 0.0);
        ws = geom::add(prev, geom::scale(geom::sub(w, prev), k));
    }
    while (gyro_hist_.size() > 1 && gyro_hist_[1].first <= s.ts_ns - 20'000'000) {
        gyro_hist_.pop_front();
    }
    Vec3 alpha{0.0, 0.0, 0.0};
    if (!gyro_hist_.empty()) {
        const auto [t0, w0] = gyro_hist_.front();
        if (s.ts_ns - t0 >= 10'000'000 && s.ts_ns - t0 <= 40'000'000) {
            alpha = geom::scale(geom::sub(ws, w0), 1e9 / static_cast<double>(s.ts_ns - t0));
        }
    }
    gyro_hist_.emplace_back(s.ts_ns, ws);
    if (geom::norm(w) > p.turn_rate) {
        last_turn_ns_ = s.ts_ns;
    }
    const Vec3 r = geom::scale(n_imu_, -1.0);
    const Vec3 a_rot = geom::add(geom::cross(alpha, r), geom::cross(w, geom::cross(w, r)));
    const Vec3 f = geom::sub(s.accel, a_rot);
    mag_.push(s.ts_ns, {geom::norm(f), 0.0, 0.0}, p.act_window_ns);
    if (att_ && last_ts_) {
        const double dt = rf::clamp(static_cast<double>(s.ts_ns - *last_ts_) / 1e9, 0.0, 0.01);
        const Quat q = qnorm(qmul(*att_, qexp(geom::scale(w, dt))));
        att_ = q;
        // f: the specific force at the neck (the IMU's minus its acceleration from the rotation alone).
        const Vec3 a_w = geom::add(qrot(q, f), G_WORLD);
        acc_.push(s.ts_ns, a_w, p.act_window_ns);
    }
    last_ts_ = s.ts_ns;
}

double DriftGov::removed_speed(double u, double allowed_speed) const {
    if (u <= allowed_speed) {
        return 0.0;
    }
    const bool t = turning();
    const double lo = t ? p.turn_band_lo : p.band_lo;
    const double hi = t ? p.turn_band_hi : p.band_hi;
    double w = 0.0;
    if (u <= lo) {
        w = 1.0;
    } else if (u >= hi || hi <= lo) {
        w = 0.0;
    } else {
        w = (hi - u) / (hi - lo);
    }
    return (u - allowed_speed) * w;
}

std::optional<Vec3> DriftGov::on_pose(const headpose::BodyState& fused, const Quat& q_now, const Quat& w_q) {
    if (!att_) {
        acc_.clear();
    }
    att_ = qnorm(q_now);
    const Vec3 neck = geom::add(fused.p, qrot(qnorm(fused.q), n_imu_));
    std::optional<Vec3> shift;
    if (!hist_.empty() && (fused.ts_ns <= hist_.back().first || fused.ts_ns - hist_.back().first > 200'000'000)) {
        hist_.clear();
    }
    if (!hist_.empty() && p.on) {
        const Vec3 n_prev = hist_.back().second;
        stats.poses += 1;
        std::size_t i = 0;
        for (std::size_t j = hist_.size(); j-- > 0;) {
            if (fused.ts_ns - hist_[j].first >= p.window_ns) {
                i = j;
                break;
            }
        }
        const auto [t0, n0] = hist_[i];
        const double span = static_cast<double>(fused.ts_ns - t0) / 1e9;
        const double u = geom::norm(geom::sub(neck, n0)) / rf::fmax(span, 1e-3);
        const double allowed_speed = allowed();
        const bool full = span >= 0.5 * static_cast<double>(p.window_ns) / 1e9 && evidence() > 0.0;
        const double rem = full ? removed_speed(u, allowed_speed) : 0.0;
        last = {u, allowed_speed, rem};
        const Vec3 step = geom::sub(neck, n_prev);
        last_step = {geom::norm(step), 0.0};
        if (rem > 0.0 && u > 1e-9) {
            const Vec3 ex = geom::scale(step, rf::fmin(rem / u, 1.0));
            last_step.second = geom::norm(ex);
            const Vec3 d = geom::scale(qrot(w_q, ex), -1.0);
            stats.held_poses += 1;
            stats.held_m += geom::norm(ex);
            stats.held_vec = geom::sub(stats.held_vec, d);
            report_m += geom::norm(ex);
            shift = d;
            // What is held is no longer the tracker's motion to compare.
            for (auto& h : hist_) {
                h.second = geom::add(h.second, ex);
            }
        }
    }
    hist_.emplace_back(fused.ts_ns, neck);
    while (hist_.size() > 2 && fused.ts_ns - hist_[1].first >= p.window_ns) {
        hist_.pop_front();
    }
    return shift;
}

std::optional<DriftReport> DriftGov::take_report(std::int64_t now_ns) {
    if (!report_from_) {
        report_from_ = now_ns;
    }
    const std::int64_t from = *report_from_;
    if (now_ns - from < p.report_every_ns) {
        return std::nullopt;
    }
    const double m = report_m;
    report_from_ = now_ns;
    report_m = 0.0;
    if (!(m >= p.report_min_m)) {
        return std::nullopt;
    }
    return DriftReport{m, static_cast<double>(now_ns - from) / 1e9, stats.held_m};
}

std::string report_line(const DriftReport& r) {
    return std::format("6DoF drift: head tracking slid {} cm without movement in {} s; held (session total {} cm)",
                       rf::fixed(r.m * 100.0, 0), rf::fixed(r.secs, 0), rf::fixed(r.total_m * 100.0, 0));
}

} // namespace pibridge::cam::drift
