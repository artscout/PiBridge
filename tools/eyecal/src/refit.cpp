// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/refit.rs (MIT OR Apache-2.0).
#include "refit.h"

#include "pibridge/eyemath/correction.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pibridge::eyecal {

namespace em = pibridge::eyemath;

namespace {

struct RawPt {
    Tangent tan;
    Vec3 origin;
    double pupil;
};

Vec3 median3(const std::vector<Vec3>& o) {
    Vec3 m{};
    for (std::size_t a = 0; a < 3; a++) {
        std::vector<double> v;
        for (const auto& p : o) {
            v.push_back(p[a]);
        }
        m[a] = median_nan(std::move(v));
    }
    return m;
}

double dist_mm(const Vec3& a, const Vec3& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < 3; i++) {
        const double d = (a[i] - b[i]) * 1e3;
        s += d * d;
    }
    return std::sqrt(s);
}

} // namespace

EyeData eye_data(std::span<const Window> wins, std::size_t eye, const Vec3& eye_pos, const ExtractOpts& opts,
                 const FixParams& fix) {
    // pass 1: samples that are tracked, not at a blink edge, after the settling time
    std::vector<std::pair<std::size_t, std::vector<RawPt>>> per;
    for (std::size_t wi = 0; wi < wins.size(); wi++) {
        const auto w = wins[wi].second;
        if (w.empty()) {
            continue;
        }
        std::vector<double> t;
        std::vector<OptTan> tan;
        for (const auto& s : w) {
            t.push_back(s.t);
            tan.push_back(s.eyes[eye] ? em::tangent_of(s.eyes[eye]->dir) : std::nullopt);
        }
        const auto labels = label(t, tan, fix);
        std::vector<bool> keep(w.size(), false);
        if (opts.ivt) {
            std::vector<bool> usable;
            for (const auto l : labels) {
                usable.push_back(l == Label::Fixation);
            }
            if (const auto r = longest_fixation(t, tan, usable, fix)) {
                for (std::size_t i = 0; i < w.size(); i++) {
                    keep[i] = r->contains(i) && usable[i];
                }
            }
        } else {
            for (std::size_t i = 0; i < w.size(); i++) {
                keep[i] = labels[i] == Label::Fixation || labels[i] == Label::Saccade;
            }
        }
        const double t0 = t[0];
        std::vector<RawPt> v;
        for (std::size_t i = 0; i < w.size(); i++) {
            if (!keep[i] || t[i] - t0 < opts.skip_s) {
                continue;
            }
            const auto& e = w[i].eyes[eye];
            if (!e || !tan[i]) {
                continue;
            }
            v.push_back(RawPt{*tan[i], e->origin, e->pupil_mm});
        }
        per.emplace_back(wi, std::move(v));
    }
    std::vector<Vec3> all;
    for (const auto& [_, v] : per) {
        for (const auto& s : v) {
            all.push_back(s.origin);
        }
    }
    Vec3 reference = all.empty() ? eye_pos : median3(all);
    if (opts.gate_mm > 0.0 && !all.empty()) {
        // second pass: the reference of the samples that pass the first gate
        std::vector<Vec3> inside;
        for (const auto& o : all) {
            if (dist_mm(o, reference) <= opts.gate_mm) {
                inside.push_back(o);
            }
        }
        if (inside.size() >= 10) {
            reference = median3(inside);
        }
    }
    const auto passes = [&](const RawPt& s) { return opts.gate_mm <= 0.0 || dist_mm(s.origin, reference) <= opts.gate_mm; };
    std::vector<double> pupils;
    for (const auto& [_, v] : per) {
        for (const auto& s : v) {
            if (passes(s) && std::isfinite(s.pupil) && s.pupil > 0.0) {
                pupils.push_back(s.pupil);
            }
        }
    }
    EyeData out;
    out.pupil_ref_mm = pupils.empty() ? 0.0 : median_nan(std::move(pupils));
    out.reference_m = reference;
    for (const auto& [wi, v] : per) {
        std::vector<const RawPt*> kept;
        for (const auto& s : v) {
            if (passes(s)) {
                kept.push_back(&s);
            }
        }
        if (kept.size() < opts.min_samples) {
            continue;
        }
        const auto truth = truth_tangent(wins[wi].first, eye_pos);
        if (!truth) {
            continue;
        }
        calfit::TargetSamples ts;
        ts.truth = *truth;
        for (const auto* s : kept) {
            calfit::SamplePt p;
            p.raw = s->tan;
            for (std::size_t a = 0; a < 3; a++) {
                p.org_mm[a] = (s->origin[a] - reference[a]) * 1e3;
            }
            p.pupil_mm = std::isfinite(s->pupil) && s->pupil > 0.0 ? s->pupil - out.pupil_ref_mm : 0.0;
            ts.samples.push_back(p);
        }
        ts.weight = std::clamp(static_cast<double>(ts.samples.size()) / 60.0, 0.3, 1.0);
        out.targets.push_back(std::move(ts));
        out.window_index.push_back(wi);
    }
    std::vector<double> prec;
    for (const auto& [_, w] : wins) {
        if (const auto e = analyze_eye(w, eye, fix)) {
            prec.push_back(e->rms_s2s_deg);
        }
    }
    out.precision_deg = prec.empty() ? 1.0 : median_nan(std::move(prec)) / std::numbers::sqrt2;
    return out;
}

std::optional<EyeFitResult> fit_eye(const EyeData& data, const Vec3& eye_pos, const ExtractOpts& opts) {
    if (data.targets.size() < 4) {
        return std::nullopt;
    }
    auto sel = calfit::select(data.targets, 0.3, opts.pupil);
    if (!sel.selection.chosen) {
        return std::nullopt;
    }
    const std::size_t i = *sel.selection.chosen;
    const auto& best = sel.selection.candidates[i];
    const auto& f = sel.fitted[i];
    em::OriginTerms org;
    org.reference_m = data.reference_m;
    org.gate_mm = opts.gate_mm;
    if (best.org) {
        org.u = f.org_u;
        org.v = f.org_v;
        if (f.pupil_uv.u != 0.0 || f.pupil_uv.v != 0.0) {
            org.pupil_ref_mm = data.pupil_ref_mm;
            org.pupil_u = f.pupil_uv.u;
            org.pupil_v = f.pupil_uv.v;
        }
    }
    EyeFitResult r;
    r.cal.map = f.map;
    r.cal.origin_m = eye_pos;
    r.cal.accuracy_deg = best.loo.mean;
    r.cal.precision_deg = data.precision_deg;
    r.cal.drift = Tangent{0.0, 0.0};
    r.cal.org = org;
    for (std::size_t k = 0; k < data.window_index.size() && k < best.loo_each.size(); k++) {
        r.loo_by_window.emplace_back(data.window_index[k], best.loo_each[k]);
    }
    r.uses_origin = best.org.has_value();
    r.points = data.targets.size();
    r.selection = std::move(sel.selection);
    return r;
}

std::string model_name(const EyeFitResult& r) {
    const std::string n = r.cal.map.model;
    if (r.cal.org.pupil_u != 0.0 || r.cal.org.pupil_v != 0.0) {
        return n + " + origin + pupil";
    }
    if (r.uses_origin) {
        return n + " + origin";
    }
    return n;
}

} // namespace pibridge::eyecal
