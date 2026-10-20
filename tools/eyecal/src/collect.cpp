// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/collect.rs (MIT OR Apache-2.0).
#include "collect.h"

#include "pibridge/eyemath/correction.h"

#include <cmath>
#include <numbers>

namespace pibridge::eyecal {

namespace em = pibridge::eyemath;

RawSample RawSample::from_sample(const em::Sample& s) {
    RawSample r;
    r.t = static_cast<double>(s.host_ts_ns) * 1e-9;
    for (std::size_t i = 0; i < 2; i++) {
        const auto& e = s.eyes[i];
        if (e.origin_valid && e.direction_valid) {
            r.eyes[i] = RawEye{{e.direction[0], e.direction[1], e.direction[2]},
                               {e.origin[0], e.origin[1], e.origin[2]},
                               static_cast<double>(e.pupil_mm)};
        }
    }
    return r;
}

em::Sample RawSample::to_sample(std::uint64_t seq) const {
    em::Sample s;
    s.seq = seq;
    s.host_ts_ns = static_cast<std::int64_t>(t * 1e9);
    for (std::size_t i = 0; i < 2; i++) {
        if (const auto& e = eyes[i]) {
            em::EyeRay r;
            r.origin = {static_cast<float>(e->origin[0]), static_cast<float>(e->origin[1]),
                        static_cast<float>(e->origin[2])};
            r.direction = {static_cast<float>(e->dir[0]), static_cast<float>(e->dir[1]), static_cast<float>(e->dir[2])};
            r.origin_valid = true;
            r.direction_valid = true;
            r.pupil_mm = static_cast<float>(e->pupil_mm);
            s.eyes[i] = r;
        }
    }
    return s;
}

std::optional<EyeWindow> analyze_eye(std::span<const RawSample> w, std::size_t eye, const FixParams& p) {
    std::vector<double> t;
    std::vector<OptTan> tan;
    for (const auto& s : w) {
        t.push_back(s.t);
        tan.push_back(s.eyes[eye] ? em::tangent_of(s.eyes[eye]->dir) : std::nullopt);
    }
    const auto labels = label(t, tan, p);
    std::vector<bool> usable;
    for (const auto l : labels) {
        usable.push_back(l == Label::Fixation);
    }
    const auto r = longest_fixation(t, tan, usable, p);
    if (!r) {
        return std::nullopt;
    }
    EyeWindow ew;
    for (std::size_t i = r->start; i < r->end; i++) {
        if (!usable[i] || !tan[i]) {
            continue;
        }
        ew.used_tan.push_back(*tan[i]);
        if (w[i].eyes[eye]) {
            ew.used_origin.push_back(w[i].eyes[eye]->origin);
            ew.used_pupil.push_back(w[i].eyes[eye]->pupil_mm);
        }
        ew.used_t.push_back(t[i]);
    }
    const auto m = median_tangent(ew.used_tan);
    if (!m) {
        return std::nullopt;
    }
    ew.total = w.size();
    for (const auto& x : tan) {
        ew.valid += x.has_value() ? 1 : 0;
    }
    ew.used = ew.used_tan.size();
    ew.fix_s = t[r->end - 1] - t[r->start];
    ew.center = m->first;
    ew.mad_deg = std::atan(m->second) * 180.0 / std::numbers::pi;
    ew.rms_s2s_deg = rms_s2s_deg(ew.used_tan);
    ew.sd_deg = sd_deg(ew.used_tan);
    return ew;
}

std::optional<Tangent> truth_tangent(const Target& target, const Vec3& eye_pos) {
    const Vec3 p = target.point();
    return em::tangent_of({p[0] - eye_pos[0], p[1] - eye_pos[1], p[2] - eye_pos[2]});
}

std::optional<Tangent> eye_center(const em::EyeCal& cal, const EyeWindow& ew) {
    std::vector<double> us;
    std::vector<double> vs;
    const std::size_t n = std::min(ew.used_tan.size(), ew.used_origin.size());
    for (std::size_t i = 0; i < n; i++) {
        const auto& tan = ew.used_tan[i];
        const std::optional<double> pupil = i < ew.used_pupil.size() ? std::optional<double>(ew.used_pupil[i]) : std::nullopt;
        const auto d = cal.apply_full(em::dir_of(tan.u, tan.v), ew.used_origin[i], pupil, Tangent{0.0, 0.0});
        if (!d) {
            continue;
        }
        if (const auto t = em::tangent_of(*d)) {
            us.push_back(t->u);
            vs.push_back(t->v);
        }
    }
    if (us.size() < 5) {
        return std::nullopt;
    }
    return Tangent{median_nan(std::move(us)), median_nan(std::move(vs))};
}

std::optional<double> eye_error_deg(const em::EyeCal& cal, const EyeWindow& ew, Tangent truth) {
    const auto c = eye_center(cal, ew);
    if (!c) {
        return std::nullopt;
    }
    return em::tangent_angle_deg(*c, truth);
}

WindowEval evaluate_window(const em::Calibration& cal, std::span<const RawSample> w, const Target& target,
                           const FixParams& p) {
    WindowEval ev;
    for (std::size_t e = 0; e < 2; e++) {
        const auto ew = analyze_eye(w, e, p);
        const auto truth = truth_tangent(target, cal.eyes[e].origin_m);
        if (ew && truth) {
            ev.eye_err_deg[e] = eye_error_deg(cal.eyes[e], *ew, *truth);
        }
    }
    // combined, sample by sample, through the same code the service runs
    em::CombineState st;
    std::vector<double> t;
    std::vector<OptTan> tan;
    for (std::size_t i = 0; i < w.size(); i++) {
        auto s = w[i].to_sample(i);
        cal.apply_sample(s, st);
        t.push_back(w[i].t);
        tan.push_back(s.gaze ? em::tangent_of({s.gaze->direction[0], s.gaze->direction[1], s.gaze->direction[2]})
                             : std::nullopt);
    }
    if (w.empty()) {
        ev.loss = 1.0;
    } else {
        std::size_t none = 0;
        for (const auto& x : tan) {
            none += x ? 0 : 1;
        }
        ev.loss = static_cast<double>(none) / static_cast<double>(w.size());
    }
    const Vec3 mid = {0.5 * (cal.eyes[0].origin_m[0] + cal.eyes[1].origin_m[0]),
                      0.5 * (cal.eyes[0].origin_m[1] + cal.eyes[1].origin_m[1]),
                      0.5 * (cal.eyes[0].origin_m[2] + cal.eyes[1].origin_m[2])};
    const auto truth = truth_tangent(target, mid);
    const auto labels = label(t, tan, p);
    std::vector<bool> usable;
    for (const auto l : labels) {
        usable.push_back(l == Label::Fixation);
    }
    const auto r = longest_fixation(t, tan, usable, p);
    if (r && truth) {
        std::vector<Tangent> used;
        for (std::size_t i = r->start; i < r->end; i++) {
            if (usable[i] && tan[i]) {
                used.push_back(*tan[i]);
            }
        }
        if (const auto m = median_tangent(used)) {
            ev.comb_err_deg = em::tangent_angle_deg(m->first, *truth);
            double sum = 0.0;
            for (const auto& x : used) {
                ev.sample_errs.push_back(em::tangent_angle_deg(x, *truth));
                sum += ev.sample_errs.back();
            }
            ev.comb_sample_err_deg = sum / static_cast<double>(ev.sample_errs.size());
            ev.comb_rms_s2s_deg = rms_s2s_deg(used);
            ev.comb_sd_deg = sd_deg(used);
        }
    }
    return ev;
}

} // namespace pibridge::eyecal
