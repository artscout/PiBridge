// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/stats.rs (MIT OR Apache-2.0).
#include "pibridge/cam/stats.h"

#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

namespace pibridge::cam {

namespace {

// Rust's `Sum for f64` starts at -0.0 (an empty sum is -0.0).
double rsum(const std::vector<double>& v) {
    double s = -0.0;
    for (const double x : v) {
        s += x;
    }
    return s;
}

} // namespace

double Series::mean() const {
    return rsum(v_) / static_cast<double>(std::max<std::size_t>(v_.size(), 1));
}

double Series::std() const {
    const double m = mean();
    double s = -0.0;
    for (const double x : v_) {
        const double d = x - m;
        s += d * d;
    }
    return std::sqrt(s / static_cast<double>(std::max<std::size_t>(v_.size(), 1)));
}

double Series::min() const {
    double m = std::numeric_limits<double>::infinity();
    for (const double x : v_) {
        m = rfmt::fmin(m, x);
    }
    return m;
}

double Series::max() const {
    double m = -std::numeric_limits<double>::infinity();
    for (const double x : v_) {
        m = rfmt::fmax(m, x);
    }
    return m;
}

double Series::quantile(double q) const {
    if (v_.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::vector<double> s = v_;
    std::stable_sort(s.begin(), s.end());
    const double idx = std::round(static_cast<double>(s.size() - 1) * q);
    return s[static_cast<std::size_t>(rfmt::f64_to_u64(idx))];
}

std::string Series::fmt(const char* unit) const {
    return std::format("{} ± {} [{} .. {}] {} (n={})", rfmt::fixed(mean(), 3), rfmt::fixed(std(), 3),
                       rfmt::fixed(min(), 3), rfmt::fixed(max(), 3), unit, len());
}

void CaptureStats::add(const FrameInfo& i) {
    frames++;
    dropped += i.dropped;
    if (!first_host_) {
        first_host_ = i.host_ts_ns;
    }
    last_host_ = i.host_ts_ns;
    if (last_) {
        const FrameInfo& p = *last_;
        host_dt_ms.push((static_cast<double>(i.host_ts_ns) - static_cast<double>(p.host_ts_ns)) / 1e6);
        if (i.dev_ts_us > 0 && p.dev_ts_us > 0) {
            dev_dt_ms.push((static_cast<double>(i.dev_ts_us) - static_cast<double>(p.dev_ts_us)) / 1e3);
        }
    }
    if (i.kind == static_cast<std::uint32_t>(FrameKind::Tracking)) {
        if (last_tracking_dev_) {
            tracking_dt_ms.push((static_cast<double>(i.dev_ts_us) - static_cast<double>(*last_tracking_dev_)) / 1e3);
        }
        last_tracking_dev_ = i.dev_ts_us;
    }
    if (i.dev_ts_us > 0) {
        raw_lat_.push_back((static_cast<double>(i.host_ts_ns) - static_cast<double>(i.dev_ts_us) * 1e3) / 1e6);
        mono_minus_host_ms.push((static_cast<double>(i.mono_ts_ns) - static_cast<double>(i.host_ts_ns)) / 1e6);
    }
    settings[SettingsKey{i.kind, i.exposure, i.gain, i.y_offset}]++;
    last_ = i;
}

void CaptureStats::finish() {
    double mn = std::numeric_limits<double>::infinity();
    for (const double l : raw_lat_) {
        mn = rfmt::fmin(mn, l);
    }
    latency_ms = Series();
    for (const double l : raw_lat_) {
        latency_ms.push(l - mn);
    }
}

double CaptureStats::duration_s() const {
    if (!first_host_) {
        return 0.0;
    }
    return static_cast<double>(last_host_ - *first_host_) / 1e9;
}

double CaptureStats::fps() const {
    if (frames < 2) {
        return 0.0;
    }
    return static_cast<double>(frames - 1) / rfmt::fmax(duration_s(), 1e-9);
}

std::string CaptureStats::report() const {
    using rfmt::fixed;
    std::string s = std::format(
        "frames {} in {} s = {} fps, lost {} ({}%)\n", frames, fixed(duration_s(), 2), fixed(fps(), 3), dropped,
        fixed(100.0 * static_cast<double>(dropped) / static_cast<double>(std::max<std::uint64_t>(frames + dropped, 1)),
              3));
    s += std::format("  host interval   {}\n", host_dt_ms.fmt("ms"));
    s += std::format("  device interval {}\n", dev_dt_ms.fmt("ms"));
    s += std::format("  tracking-frame interval (device) {}\n", tracking_dt_ms.fmt("ms"));
    s += std::format("  arrival latency above minimum: p50 {} p90 {} p99 {} max {} ms\n",
                     fixed(latency_ms.quantile(0.5), 2), fixed(latency_ms.quantile(0.9), 2),
                     fixed(latency_ms.quantile(0.99), 2), fixed(latency_ms.max(), 2));
    s += std::format("  fallback clock: mapped time - arrival {}\n", mono_minus_host_ms.fmt("ms"));
    for (const auto& [key, n] : settings) {
        const auto& [k, e, g, y] = key;
        const char* kind = k == 1 ? "tracking" : (k == 2 ? "led" : "unknown");
        s += std::format("  {:6} x {:8} exposure {} gain {} rows {}\n", n, kind, rfmt::debug_u16s(e),
                         rfmt::debug_u16s(g), rfmt::debug_u16s(y));
    }
    return s;
}

} // namespace pibridge::cam
