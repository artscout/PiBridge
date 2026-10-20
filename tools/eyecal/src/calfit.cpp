// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/calfit.rs (MIT OR Apache-2.0).
#include "calfit.h"

#include "fixation.h"

#include <algorithm>
#include <cmath>

namespace pibridge::eyecal::calfit {

namespace em = pibridge::eyemath;

namespace {

std::size_t poly_len(ModelKind k) { return em::model_features(k); }

std::vector<double> features(const SamplePt& p, const Spec& spec) {
    const double u = p.raw.u;
    const double v = p.raw.v;
    std::vector<double> f = {1.0, u, v, u * u, u * v, v * v};
    f.resize(poly_len(spec.kind));
    if (spec.org) {
        f.insert(f.end(), p.org_mm.begin(), p.org_mm.end());
        if (spec.pupil) {
            f.push_back(std::clamp(p.pupil_mm, -em::PUPIL_CLAMP_MM, em::PUPIL_CLAMP_MM));
        }
    }
    return f;
}

} // namespace

Tangent Fitted::apply(const SamplePt& p) const {
    const Tangent t = map.map_tangent(p.raw.u, p.raw.v);
    const Vec3& d = p.org_mm;
    const double q = std::clamp(p.pupil_mm, -em::PUPIL_CLAMP_MM, em::PUPIL_CLAMP_MM);
    return {t.u + org_u[0] * d[0] + org_u[1] * d[1] + org_u[2] * d[2] + pupil_uv.u * q,
            t.v + org_v[0] * d[0] + org_v[1] * d[1] + org_v[2] * d[2] + pupil_uv.v * q};
}

std::optional<Tangent> target_center(const Fitted& f, const TargetSamples& t) {
    if (t.samples.empty()) {
        return std::nullopt;
    }
    std::vector<double> us;
    std::vector<double> vs;
    for (const auto& s : t.samples) {
        const Tangent c = f.apply(s);
        us.push_back(c.u);
        vs.push_back(c.v);
    }
    return Tangent{median_nan(std::move(us)), median_nan(std::move(vs))};
}

std::optional<double> target_error_deg(const Fitted& f, const TargetSamples& t) {
    const auto c = target_center(f, t);
    if (!c) {
        return std::nullopt;
    }
    return em::tangent_angle_deg(*c, t.truth);
}

std::optional<Fitted> fit(std::span<const TargetSamples* const> targets, const Spec& spec) {
    const SamplePt* first = nullptr;
    for (const auto* t : targets) {
        if (!t->samples.empty()) {
            first = &t->samples.front();
            break;
        }
    }
    if (first == nullptr) {
        return std::nullopt;
    }
    const std::size_t k = features(*first, spec).size();
    const std::size_t np = poly_len(spec.kind);
    struct Row {
        std::vector<double> f;
        Tangent r;
        double w;
    };
    std::vector<Row> rows;
    std::size_t ntargets = 0;
    for (const auto* t : targets) {
        if (t->samples.empty() || !(t->weight > 0.0)) {
            continue;
        }
        ntargets++;
        const double w = t->weight / static_cast<double>(t->samples.size());
        for (const auto& s : t->samples) {
            rows.push_back(Row{features(s, spec), Tangent{t->truth.u - s.raw.u, t->truth.v - s.raw.v}, w});
        }
    }
    if (rows.empty() || ntargets < std::min(em::model_min_points(spec.kind), poly_len(spec.kind))) {
        return std::nullopt;
    }
    std::vector<double> base;
    for (const auto& r : rows) {
        base.push_back(r.w);
    }
    std::vector<double> w = base;
    std::optional<std::pair<std::vector<double>, std::vector<double>>> sol;
    for (int it = 0; it < 5; it++) {
        double total = 0.0;
        for (const double x : w) {
            total += x;
        }
        if (total <= 0.0) {
            return std::nullopt;
        }
        std::vector<std::vector<double>> ata(k, std::vector<double>(k, 0.0));
        std::vector<double> bu(k, 0.0);
        std::vector<double> bv(k, 0.0);
        for (std::size_t n = 0; n < rows.size(); n++) {
            const auto& r = rows[n];
            const double wi = w[n];
            for (std::size_t i = 0; i < k; i++) {
                for (std::size_t j = 0; j < k; j++) {
                    ata[i][j] += wi * r.f[i] * r.f[j];
                }
                bu[i] += wi * r.f[i] * r.r.u;
                bv[i] += wi * r.f[i] * r.r.v;
            }
        }
        for (std::size_t i = 0; i < k; i++) {
            double pen = spec.org_ridge;
            if (i == 0) {
                pen = 1e-12;
            } else if (i == 1 || i == 2) {
                pen = spec.ridge.linear;
            } else if (i < np) {
                pen = spec.ridge.quadratic;
            }
            ata[i][i] += pen * std::max(ata[i][i], 1e-9 * total);
        }
        auto su = em::detail::solve(ata, bu);
        if (!su) {
            return std::nullopt;
        }
        auto sv = em::detail::solve(ata, bv);
        if (!sv) {
            return std::nullopt;
        }
        for (std::size_t n = 0; n < rows.size(); n++) {
            const auto& r = rows[n];
            double pu = 0.0;
            double pv = 0.0;
            for (std::size_t i = 0; i < k; i++) {
                pu += (*su)[i] * r.f[i];
                pv += (*sv)[i] * r.f[i];
            }
            const double res = std::sqrt((pu - r.r.u) * (pu - r.r.u) + (pv - r.r.v) * (pv - r.r.v));
            w[n] = base[n] * (res <= HUBER_SAMPLE ? 1.0 : HUBER_SAMPLE / res);
        }
        sol = std::pair{std::move(*su), std::move(*sv)};
    }
    if (!sol) {
        return std::nullopt;
    }
    const auto& [su, sv] = *sol;
    Fitted f;
    f.map = em::Correction::identity();
    for (std::size_t i = 0; i < np; i++) {
        f.map.cx[i] += su[i];
        f.map.cy[i] += sv[i];
    }
    f.map.model = em::model_name(spec.kind);
    if (spec.org) {
        for (std::size_t a = 0; a < 3; a++) {
            f.org_u[a] = su[np + a];
            f.org_v[a] = sv[np + a];
        }
        if (spec.pupil) {
            f.pupil_uv = Tangent{su[np + 3], sv[np + 3]};
        }
    }
    return f;
}

std::optional<std::pair<ModelFit, Fitted>> evaluate(std::span<const TargetSamples> targets, const Spec& spec) {
    if (targets.size() < em::model_min_points(spec.kind) + 1) {
        return std::nullopt;
    }
    std::vector<const TargetSamples*> all;
    for (const auto& t : targets) {
        all.push_back(&t);
    }
    auto f = fit(all, spec);
    if (!f) {
        return std::nullopt;
    }
    std::vector<double> train;
    for (const auto& t : targets) {
        if (auto e = target_error_deg(*f, t)) {
            train.push_back(*e);
        }
    }
    std::vector<double> loo;
    for (std::size_t i = 0; i < targets.size(); i++) {
        std::vector<const TargetSamples*> rest;
        for (std::size_t j = 0; j < targets.size(); j++) {
            if (j != i) {
                rest.push_back(&targets[j]);
            }
        }
        auto m = fit(rest, spec);
        if (!m) {
            return std::nullopt;
        }
        auto e = target_error_deg(*m, targets[i]);
        if (!e) {
            return std::nullopt;
        }
        loo.push_back(*e);
    }
    ModelFit mf;
    mf.kind = spec.kind;
    mf.ridge = spec.ridge;
    mf.map = f->map;
    mf.loo = em::stats(loo);
    mf.train = em::stats(train);
    if (spec.org) {
        mf.org = std::pair{f->org_u, f->org_v};
    }
    mf.loo_each = loo;
    return std::pair{std::move(mf), std::move(*f)};
}

std::vector<Spec> candidates(bool with_pupil) {
    const auto r = [](double q) {
        Ridge x;
        x.quadratic = q;
        return x;
    };
    const double org_ridge = 0.005;
    std::vector<Spec> v;
    for (const bool org : {false, true}) {
        v.push_back(Spec{ModelKind::Offset, org, Ridge{}, org_ridge, false});
        v.push_back(Spec{ModelKind::Affine, org, Ridge{}, org_ridge, false});
        for (const double q : {1.0, 0.2, 0.02}) {
            v.push_back(Spec{ModelKind::Quadratic, org, r(q), org_ridge, false});
        }
    }
    if (with_pupil) {
        // after the others: only chosen when clearly better
        v.push_back(Spec{ModelKind::Affine, true, Ridge{}, org_ridge, true});
        v.push_back(Spec{ModelKind::Quadratic, true, r(0.2), org_ridge, true});
    }
    return v;
}

std::pair<const ModelFit*, const Fitted*> SampleSelection::best() const {
    if (!selection.chosen) {
        return {nullptr, nullptr};
    }
    return {&selection.candidates[*selection.chosen], &fitted[*selection.chosen]};
}

SampleSelection select(std::span<const TargetSamples> targets, double min_org_sd_mm, bool with_pupil) {
    std::vector<double> raw_err;
    for (const auto& t : targets) {
        if (t.samples.empty()) {
            continue;
        }
        std::vector<double> us;
        std::vector<double> vs;
        for (const auto& s : t.samples) {
            us.push_back(s.raw.u);
            vs.push_back(s.raw.v);
        }
        raw_err.push_back(em::tangent_angle_deg(Tangent{median_nan(std::move(us)), median_nan(std::move(vs))}, t.truth));
    }
    SampleSelection out;
    out.selection.raw = em::stats(raw_err);
    const double sd = org_sd_mm(targets);
    for (const auto& spec : candidates(with_pupil)) {
        if (spec.org && sd < min_org_sd_mm) {
            continue;
        }
        if (auto e = evaluate(targets, spec)) {
            out.selection.candidates.push_back(std::move(e->first));
            out.fitted.push_back(std::move(e->second));
        }
    }
    const auto& c = out.selection.candidates;
    if (!c.empty()) {
        std::size_t bi = 0;
        for (std::size_t i = 1; i < c.size(); i++) {
            // Rust min_by keeps the first of equal minima
            if (std::strong_order(c[i].loo.mean, c[bi].loo.mean) < 0) {
                bi = i;
            }
        }
        const double bm = c[bi].loo.mean;
        std::size_t chosen = bi;
        for (std::size_t i = 0; i < c.size(); i++) {
            if (c[i].loo.mean <= bm * 1.07 || c[i].loo.mean - bm <= 0.05) {
                chosen = i;
                break;
            }
        }
        out.selection.chosen = chosen;
    }
    return out;
}

double org_sd_mm(std::span<const TargetSamples> targets) {
    std::vector<const SamplePt*> all;
    for (const auto& t : targets) {
        for (const auto& s : t.samples) {
            all.push_back(&s);
        }
    }
    if (all.size() < 5) {
        return 0.0;
    }
    double best = 0.0;
    for (std::size_t a = 0; a < 3; a++) {
        double m = 0.0;
        for (const auto* s : all) {
            m += s->org_mm[a];
        }
        m /= static_cast<double>(all.size());
        double v = 0.0;
        for (const auto* s : all) {
            v += (s->org_mm[a] - m) * (s->org_mm[a] - m);
        }
        best = std::max(best, std::sqrt(v / static_cast<double>(all.size())));
    }
    return best;
}

} // namespace pibridge::eyecal::calfit
