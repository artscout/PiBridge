// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/fixation.rs (MIT OR Apache-2.0).
#include "fixation.h"

#include "pibridge/eyemath/calib.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <numbers>

namespace pibridge::eyecal {

namespace {

double ang(Tangent a, Tangent b) { return pibridge::eyemath::tangent_angle_deg(a, b); }

double tan_deg(double d) { return std::tan(d * std::numbers::pi / 180.0); }

} // namespace

double median_nan(std::vector<double> x) {
    std::ranges::sort(x, [](double a, double b) { return std::strong_order(a, b) < 0; });
    const std::size_t n = x.size();
    if (n == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (n % 2 == 1) {
        return x[n / 2];
    }
    return 0.5 * (x[n / 2 - 1] + x[n / 2]);
}

std::vector<OptTan> smooth(std::span<const OptTan> tan, std::size_t h) {
    const std::size_t n = tan.size();
    std::vector<OptTan> out(n);
    for (std::size_t i = 0; i < n; i++) {
        if (!tan[i]) {
            continue;
        }
        const std::size_t lo = i >= h ? i - h : 0;
        const std::size_t hi = std::min(i + h + 1, n);
        std::vector<double> us;
        std::vector<double> vs;
        for (std::size_t k = lo; k < hi; k++) {
            if (tan[k]) {
                us.push_back(tan[k]->u);
                vs.push_back(tan[k]->v);
            }
        }
        out[i] = Tangent{median_nan(std::move(us)), median_nan(std::move(vs))};
    }
    return out;
}

std::vector<Label> label(std::span<const double> t_all, std::span<const OptTan> tan_all, const FixParams& p) {
    const std::size_t n = std::min(t_all.size(), tan_all.size());
    const auto t = t_all.first(n);
    const auto tan = tan_all.first(n);
    std::vector<Label> out(n);
    for (std::size_t i = 0; i < n; i++) {
        out[i] = tan[i] ? Label::Fixation : Label::Invalid;
    }
    // blinks: runs of invalid samples lasting at least `min_blink_s`; their neighbours within `blink_edge_s` are
    // corrupted by the lid. A single dropped frame is just a missing sample.
    std::vector<std::pair<double, double>> blinks;
    std::size_t i = 0;
    while (i < n) {
        if (tan[i]) {
            i++;
            continue;
        }
        const std::size_t a = i;
        while (i < n && !tan[i]) {
            i++;
        }
        const double ta = t[a];
        const double tb = t[i - 1];
        const double before = a > 0 ? t[a - 1] : ta;
        const double after = i < n ? t[i] : tb;
        if (after - before >= p.min_blink_s) {
            blinks.emplace_back(ta, tb);
        }
    }
    for (std::size_t j = 0; j < n; j++) {
        if (out[j] == Label::Fixation &&
            std::ranges::any_of(blinks, [&](const auto& b) {
                return t[j] >= b.first - p.blink_edge_s && t[j] <= b.second + p.blink_edge_s;
            })) {
            out[j] = Label::BlinkEdge;
        }
    }
    const auto sm = smooth(tan, 3);
    std::vector<bool> fast(n, false);
    for (std::size_t k = 0; k < n; k++) {
        const std::size_t a = k >= 2 ? k - 2 : 0;
        const std::size_t b = std::min(k + 2, n == 0 ? 0 : n - 1);
        if (sm[a] && sm[b]) {
            const double dt = t[b] - t[a];
            fast[k] = dt > 1e-6 && ang(*sm[a], *sm[b]) / dt > p.velocity_dps;
        }
    }
    for (std::size_t k = 0; k < n; k++) {
        if (!fast[k]) {
            continue;
        }
        const std::size_t lo = k >= p.saccade_pad ? k - p.saccade_pad : 0;
        const std::size_t hi = std::min(k + p.saccade_pad, n == 0 ? 0 : n - 1);
        for (std::size_t q = lo; q <= hi && q < n; q++) {
            if (out[q] == Label::Fixation) {
                out[q] = Label::Saccade;
            }
        }
    }
    return out;
}

std::optional<Range> longest_fixation(std::span<const double> t, std::span<const OptTan> tan,
                                      const std::vector<bool>& usable, const FixParams& p) {
    const std::size_t n = std::min({t.size(), tan.size(), usable.size()});
    const auto sm = smooth(tan.first(n), 3);
    const double disp = tan_deg(p.max_dispersion_deg);
    std::optional<std::pair<Range, double>> best;
    std::size_t start = 0;
    while (start < n) {
        if (!usable[start] || !sm[start]) {
            start++;
            continue;
        }
        double lo_u = DBL_MAX;
        double hi_u = -DBL_MAX;
        double lo_v = DBL_MAX;
        double hi_v = -DBL_MAX;
        std::size_t end = start;
        // invalid or unusable samples inside are skipped (not counted) as long as the gap is short
        std::size_t last_good = start;
        while (end < n) {
            if (!usable[end] || !sm[end]) {
                if (t[end] - t[last_good] > p.max_gap_s) {
                    break;
                }
                end++;
                continue;
            }
            const double u = sm[end]->u;
            const double v = sm[end]->v;
            const double nlu = std::min(lo_u, u);
            const double nhu = std::max(hi_u, u);
            const double nlv = std::min(lo_v, v);
            const double nhv = std::max(hi_v, v);
            if (nhu - nlu > disp || nhv - nlv > disp) {
                break;
            }
            lo_u = nlu;
            hi_u = nhu;
            lo_v = nlv;
            hi_v = nhv;
            last_good = end;
            end++;
        }
        const double dur = t[last_good] - t[start];
        if (dur >= p.min_duration_s && (!best || dur > best->second)) {
            best = std::pair{Range{start, last_good + 1}, dur};
        }
        start = last_good + 1;
    }
    if (!best) {
        return std::nullopt;
    }
    return best->first;
}

std::optional<std::pair<Tangent, double>> median_tangent(std::span<const Tangent> v) {
    if (v.empty()) {
        return std::nullopt;
    }
    std::vector<double> us;
    std::vector<double> vs;
    for (const auto& p : v) {
        us.push_back(p.u);
        vs.push_back(p.v);
    }
    const Tangent m{median_nan(std::move(us)), median_nan(std::move(vs))};
    std::vector<double> d;
    for (const auto& p : v) {
        d.push_back(std::sqrt((p.u - m.u) * (p.u - m.u) + (p.v - m.v) * (p.v - m.v)));
    }
    return std::pair{m, median_nan(std::move(d))};
}

double rms_s2s_deg(std::span<const Tangent> v) {
    if (v.size() < 2) {
        return 0.0;
    }
    double ss = 0.0;
    for (std::size_t i = 1; i < v.size(); i++) {
        const double a = ang(v[i - 1], v[i]);
        ss += a * a;
    }
    return std::sqrt(ss / static_cast<double>(v.size() - 1));
}

double sd_deg(std::span<const Tangent> v) {
    if (v.size() < 2) {
        return 0.0;
    }
    Tangent m{0.0, 0.0};
    for (const auto& p : v) {
        m.u += p.u;
        m.v += p.v;
    }
    m.u /= static_cast<double>(v.size());
    m.v /= static_cast<double>(v.size());
    double ss = 0.0;
    for (const auto& p : v) {
        const double a = ang(p, m);
        ss += a * a;
    }
    return std::sqrt(ss / static_cast<double>(v.size() - 1));
}

} // namespace pibridge::eyecal
