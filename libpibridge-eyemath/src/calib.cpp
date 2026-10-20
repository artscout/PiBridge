// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/calib.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/calib.h"
#include "pibridge_env.h"

#include <algorithm>
#include <cmath>
#include <compare>
#include <cstdlib>
#include <format>

#include "num.h"

namespace pibridge::eyemath {

namespace fs = std::filesystem;
using detail::fmt_fixed;
using detail::rclamp;
using detail::rmax;

std::size_t model_features(ModelKind k) {
    switch (k) {
    case ModelKind::Offset: return 1;
    case ModelKind::Affine: return 3;
    case ModelKind::Quadratic: return 6;
    }
    return 3;
}

const char* model_name(ModelKind k) {
    switch (k) {
    case ModelKind::Offset: return "offset";
    case ModelKind::Affine: return "affine";
    case ModelKind::Quadratic: return "quadratic";
    }
    return "affine";
}

std::optional<ModelKind> parse_model(std::string_view s) {
    if (s == "offset") return ModelKind::Offset;
    if (s == "affine") return ModelKind::Affine;
    if (s == "quadratic") return ModelKind::Quadratic;
    return std::nullopt;
}

std::size_t model_min_points(ModelKind k) {
    switch (k) {
    case ModelKind::Offset: return 1;
    case ModelKind::Affine: return 4;
    case ModelKind::Quadratic: return 7;
    }
    return 4;
}

double tangent_angle_deg(Tangent a, Tangent b) {
    const auto da = dir_of(a.u, a.v);
    const auto db = dir_of(b.u, b.v);
    const Vec3 c{da[1] * db[2] - da[2] * db[1], da[2] * db[0] - da[0] * db[2], da[0] * db[1] - da[1] * db[0]};
    const double s = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    const double d = da[0] * db[0] + da[1] * db[1] + da[2] * db[2];
    return detail::to_degrees(std::atan2(s, d));
}

Stats stats(std::span<const double> v) {
    std::vector<double> s;
    for (double x : v) {
        if (std::isfinite(x)) {
            s.push_back(x);
        }
    }
    if (s.empty()) {
        return {};
    }
    // Rust total_cmp: IEEE total order
    std::ranges::sort(s, [](double a, double b) { return std::strong_order(a, b) < 0; });
    const auto pct = [&](double p) {
        const double x = p * static_cast<double>(s.size() - 1);
        const auto i = static_cast<std::size_t>(std::floor(x));
        const double f = x - static_cast<double>(i);
        return s[i] + (s[std::min(i + 1, s.size() - 1)] - s[i]) * f;
    };
    double sum = 0.0;
    for (double x : s) {
        sum += x;
    }
    Stats r;
    r.n = s.size();
    r.mean = sum / static_cast<double>(s.size());
    r.median = pct(0.5);
    r.p95 = pct(0.95);
    r.max = s.back();
    return r;
}

namespace detail {

std::optional<std::vector<double>> solve(std::vector<std::vector<double>> a, std::vector<double> b) {
    const std::size_t n = b.size();
    for (std::size_t c = 0; c < n; c++) {
        // Rust `max_by` keeps the last of equal maxima
        std::size_t p = c;
        for (std::size_t i = c + 1; i < n; i++) {
            if (std::strong_order(std::fabs(a[i][c]), std::fabs(a[p][c])) >= 0) {
                p = i;
            }
        }
        if (std::fabs(a[p][c]) < 1e-14) {
            return std::nullopt;
        }
        std::swap(a[c], a[p]);
        std::swap(b[c], b[p]);
        for (std::size_t r = c + 1; r < n; r++) {
            const double f = a[r][c] / a[c][c];
            if (f == 0.0) {
                continue;
            }
            for (std::size_t k = c; k < n; k++) {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    std::vector<double> x(n, 0.0);
    for (std::size_t r = n; r-- > 0;) {
        double s = 0.0;
        for (std::size_t k = r + 1; k < n; k++) {
            s += a[r][k] * x[k];
        }
        x[r] = (b[r] - s) / a[r][r];
    }
    return x;
}

std::optional<Correction> fit_weighted(std::span<const EyePoint> points, std::span<const double> w, ModelKind kind,
                                       Ridge ridge) {
    const std::size_t k = model_features(kind);
    double total = 0.0;
    for (double x : w) {
        total += x;
    }
    if (points.size() < std::min(model_min_points(kind), k) || total <= 0.0) {
        return std::nullopt;
    }
    std::vector<std::vector<double>> ata(k, std::vector<double>(k, 0.0));
    std::vector<double> bu(k, 0.0);
    std::vector<double> bv(k, 0.0);
    for (std::size_t pi = 0; pi < points.size() && pi < w.size(); pi++) {
        const auto& p = points[pi];
        const double wi = w[pi];
        const auto f = features(p.raw.u, p.raw.v);
        const double du = p.truth.u - p.raw.u;
        const double dv = p.truth.v - p.raw.v;
        for (std::size_t i = 0; i < k; i++) {
            for (std::size_t j = 0; j < k; j++) {
                ata[i][j] += wi * f[i] * f[j];
            }
            bu[i] += wi * f[i] * du;
            bv[i] += wi * f[i] * dv;
        }
    }
    // relative ridge: each penalty is a fraction of that feature's own information, so the strengths do not depend
    // on the size of the tangents
    for (std::size_t i = 0; i < k; i++) {
        const double pen = i == 0 ? 1e-12 : (i <= 2 ? ridge.linear : ridge.quadratic);
        ata[i][i] += pen * rmax(ata[i][i], 1e-9 * total);
    }
    const auto du = solve(ata, bu);
    if (!du) {
        return std::nullopt;
    }
    const auto dv = solve(std::move(ata), std::move(bv));
    if (!dv) {
        return std::nullopt;
    }
    Correction c = Correction::identity();
    for (std::size_t i = 0; i < k; i++) {
        c.cx[i] += (*du)[i];
        c.cy[i] += (*dv)[i];
    }
    c.model = model_name(kind);
    return c;
}

} // namespace detail

std::optional<Correction> fit_eye(std::span<const EyePoint> points, ModelKind kind, Ridge ridge) {
    std::vector<double> base;
    base.reserve(points.size());
    for (const auto& p : points) {
        base.push_back(rmax(p.weight, 0.0));
    }
    auto w = base;
    auto c = detail::fit_weighted(points, w, kind, ridge);
    if (!c) {
        return std::nullopt;
    }
    for (int it = 0; it < 5; it++) {
        for (std::size_t i = 0; i < points.size(); i++) {
            const auto& p = points[i];
            const auto m = c->map_tangent(p.raw.u, p.raw.v);
            const double eu = m.u - p.truth.u;
            const double ev = m.v - p.truth.v;
            const double r = std::sqrt(eu * eu + ev * ev);
            w[i] = base[i] * (r <= HUBER_K ? 1.0 : HUBER_K / r);
        }
        c = detail::fit_weighted(points, w, kind, ridge);
        if (!c) {
            return std::nullopt;
        }
    }
    return c;
}

double point_error_deg(const Correction& c, const EyePoint& p) {
    return tangent_angle_deg(c.map_tangent(p.raw.u, p.raw.v), p.truth);
}

std::optional<ModelFit> evaluate(std::span<const EyePoint> points, ModelKind kind, Ridge ridge) {
    if (points.size() < model_min_points(kind) + 1) {
        return std::nullopt;
    }
    auto map = fit_eye(points, kind, ridge);
    if (!map) {
        return std::nullopt;
    }
    std::vector<double> train;
    for (const auto& p : points) {
        train.push_back(point_error_deg(*map, p));
    }
    std::vector<double> loo;
    for (std::size_t i = 0; i < points.size(); i++) {
        std::vector<EyePoint> rest;
        for (std::size_t j = 0; j < points.size(); j++) {
            if (j != i) {
                rest.push_back(points[j]);
            }
        }
        const auto m = fit_eye(rest, kind, ridge);
        if (!m) {
            return std::nullopt;
        }
        loo.push_back(point_error_deg(*m, points[i]));
    }
    ModelFit f;
    f.kind = kind;
    f.ridge = ridge;
    f.map = std::move(*map);
    f.loo = stats(loo);
    f.train = stats(train);
    f.loo_each = std::move(loo);
    return f;
}

Selection select_model(std::span<const EyePoint> points) {
    std::vector<double> raw;
    for (const auto& p : points) {
        raw.push_back(tangent_angle_deg(p.raw, p.truth));
    }
    Selection sel;
    sel.raw = stats(raw);
    const auto add = [&](std::optional<ModelFit> m) {
        if (m) {
            sel.candidates.push_back(std::move(*m));
        }
    };
    add(evaluate(points, ModelKind::Offset, Ridge{}));
    add(evaluate(points, ModelKind::Affine, Ridge{}));
    for (double q : {1.0, 0.2, 0.02}) {
        Ridge r;
        r.quadratic = q;
        add(evaluate(points, ModelKind::Quadratic, r));
    }
    if (!sel.candidates.empty()) {
        // Rust `min_by` keeps the first of equal minima
        std::size_t bi = 0;
        for (std::size_t i = 1; i < sel.candidates.size(); i++) {
            if (std::strong_order(sel.candidates[i].loo.mean, sel.candidates[bi].loo.mean) < 0) {
                bi = i;
            }
        }
        const double bm = sel.candidates[bi].loo.mean;
        // the simplest candidate that is close enough to the best (candidates are in increasing complexity)
        sel.chosen = bi;
        for (std::size_t i = 0; i < sel.candidates.size(); i++) {
            const double m = sel.candidates[i].loo.mean;
            if (m <= bm * 1.07 || m - bm <= 0.05) {
                sel.chosen = i;
                break;
            }
        }
    }
    return sel;
}

bool OriginTerms::is_active() const {
    if (gate_mm > 0.0) {
        return true;
    }
    for (std::size_t i = 0; i < 3; i++) {
        if (u[i] != 0.0 || v[i] != 0.0) {
            return true;
        }
    }
    return pupil_u != 0.0 || pupil_v != 0.0;
}

Tangent OriginTerms::pupil_shift(std::optional<double> pupil_mm) const {
    if (pupil_mm && std::isfinite(*pupil_mm) && *pupil_mm > 0.0 && (pupil_u != 0.0 || pupil_v != 0.0)) {
        const double d = rclamp(*pupil_mm - pupil_ref_mm, -PUPIL_CLAMP_MM, PUPIL_CLAMP_MM);
        return {pupil_u * d, pupil_v * d};
    }
    return {};
}

Vec3 OriginTerms::deviation_mm(const Vec3& origin) const {
    return {(origin[0] - reference_m[0]) * 1e3, (origin[1] - reference_m[1]) * 1e3,
            (origin[2] - reference_m[2]) * 1e3};
}

double OriginTerms::distance_mm(const Vec3& origin) const {
    const auto d = deviation_mm(origin);
    return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

Tangent OriginTerms::shift(const Vec3& origin) const {
    if (!is_active()) {
        return {};
    }
    const auto d = deviation_mm(origin);
    return {u[0] * d[0] + u[1] * d[1] + u[2] * d[2], v[0] * d[0] + v[1] * d[1] + v[2] * d[2]};
}

bool OriginTerms::usable(const Vec3& origin) const {
    const bool finite = std::isfinite(origin[0]) && std::isfinite(origin[1]) && std::isfinite(origin[2]);
    return finite && (gate_mm <= 0.0 || distance_mm(origin) <= gate_mm);
}

double OriginTerms::reliability(const Vec3& origin) const {
    if (gate_mm <= 0.0) {
        return 1.0;
    }
    const double z = distance_mm(origin) / (0.5 * gate_mm);
    return std::exp(-0.5 * z * z);
}

std::optional<Vec3> EyeCal::apply(const Vec3& raw) const { return apply_with(raw, {}); }

std::optional<Vec3> EyeCal::apply_with(const Vec3& raw, Tangent extra) const {
    return apply_full(raw, std::nullopt, std::nullopt, extra);
}

std::optional<Vec3> EyeCal::apply_full(const Vec3& raw, std::optional<Vec3> origin, std::optional<double> pupil_mm,
                                       Tangent extra) const {
    const auto t = tangent_of(raw);
    if (!t) {
        return std::nullopt;
    }
    const auto m = map.map_tangent(t->u, t->v);
    Tangent o{};
    if (origin) {
        if (!org.usable(*origin)) {
            return std::nullopt;
        }
        o = org.shift(*origin);
    }
    const auto p = org.pupil_shift(pupil_mm);
    return dir_of(m.u + drift.u + extra.u + o.u + p.u, m.v + drift.v + extra.v + o.v + p.v);
}

double EyeCal::weight() const {
    const double a = rmax(accuracy_deg, 0.2);
    const double p = rmax(precision_deg, 0.1);
    return 1.0 / (a * a + p * p);
}

Calibration Calibration::identity(FrameConv conv, double ipd_m) {
    const auto eye = [](double x) {
        EyeCal e;
        e.map = Correction::identity();
        e.origin_m = {x, 0.0, 0.0};
        e.accuracy_deg = 1.0;
        e.precision_deg = 0.5;
        return e;
    };
    Calibration c;
    c.conv = conv;
    c.eyes = {eye(-ipd_m / 2.0), eye(ipd_m / 2.0)};
    return c;
}

namespace {

std::string fmt_arr(std::span<const double> c) {
    std::string s = "[";
    for (std::size_t i = 0; i < c.size(); i++) {
        if (i > 0) {
            s += ", ";
        }
        s += fmt_fixed(c[i], 9);
    }
    return s + "]";
}

const char* b01(bool b) { return b ? "1" : "0"; }

// PiBridge's folder, the folder of PiBridge's earlier name (pimax-linux), then omarchyvr's: preferred first.
std::array<fs::path, 3> candidates(const fs::path& home) {
    return {home / ".config/pibridge/eye-calibration.toml", home / ".config/pimax-linux/eye-calibration.toml",
            home / ".config/omarchyvr/eye-calibration.toml"};
}

} // namespace

std::string Calibration::to_toml() const {
    std::string s = "# pimax eye calibration: per-eye host-side maps (written by pimax-eyecal; the eye chip's own "
                    "calibration is not touched)\n";
    for (const auto& n : notes) {
        s += std::format("# {}\n", n);
    }
    s += "# per eye: u' = u . (1, u, v, u*u, u*v, v*v), v' = v . (...) on the tangent (x / -z, y / -z) of the raw "
         "head-frame direction; then drift and (version 2) origin_u/v . (origin - origin_ref) in mm are added\n";
    const bool v2 = eyes[0].org.is_active() || eyes[1].org.is_active();
    s += std::format("version = {}\n", v2 ? 2 : 1);
    s += std::format("created_unix = {}\n", created_unix);
    s += std::format("flip = \"{},{},{}\"\n", b01(conv.flip_x), b01(conv.flip_y), b01(conv.flip_z));
    s += std::format("swap = {}\n", b01(conv.swap_eyes));
    s += std::format("offset_mm = {}\n", fmt_arr(conv.offset_mm));
    s += std::format("depth_m = {}\n", fmt_fixed(combine.depth_m, 4));
    s += std::format("max_disagree_deg = {}\n", fmt_fixed(combine.max_disagree_deg, 3));
    s += std::format("holdoff = {}\n", combine.holdoff);
    const char* names[2] = {"left", "right"};
    for (std::size_t i = 0; i < 2; i++) {
        const auto& e = eyes[i];
        s += std::format("\n[{}]\n", names[i]);
        s += std::format("model = \"{}\"\n", e.map.model);
        s += std::format("u = {}\n", fmt_arr(e.map.cx));
        s += std::format("v = {}\n", fmt_arr(e.map.cy));
        s += std::format("origin_m = {}\n", fmt_arr(e.origin_m));
        s += std::format("accuracy_deg = {}\n", fmt_fixed(e.accuracy_deg, 4));
        s += std::format("precision_deg = {}\n", fmt_fixed(e.precision_deg, 4));
        const std::array<double, 2> drift{e.drift.u, e.drift.v};
        s += std::format("drift = {}\n", fmt_arr(drift));
        if (e.org.is_active()) {
            s += std::format("origin_ref_m = {}\n", fmt_arr(e.org.reference_m));
            s += std::format("origin_u = {}\n", fmt_arr(e.org.u));
            s += std::format("origin_v = {}\n", fmt_arr(e.org.v));
            s += std::format("origin_gate_mm = {}\n", fmt_fixed(e.org.gate_mm, 3));
            if (e.org.pupil_u != 0.0 || e.org.pupil_v != 0.0) {
                s += std::format("pupil_ref_mm = {}\n", fmt_fixed(e.org.pupil_ref_mm, 4));
                const std::array<double, 2> puv{e.org.pupil_u, e.org.pupil_v};
                s += std::format("pupil_uv = {}\n", fmt_arr(puv));
            }
        }
    }
    s += "\n[filter]\n";
    s += std::format("kind = \"{}\"\n", filter.enabled ? "one-euro" : "none");
    s += std::format("min_cutoff_hz = {}\n", fmt_fixed(filter.min_cutoff_hz, 4));
    s += std::format("beta = {}\n", fmt_fixed(filter.beta, 5));
    s += std::format("d_cutoff_hz = {}\n", fmt_fixed(filter.d_cutoff_hz, 4));
    s += std::format("snap_deg = {}\n", fmt_fixed(filter.snap_deg, 3));
    return s;
}

namespace {

bool all_finite(const std::vector<double>& a) {
    return std::ranges::all_of(a, [](double x) { return std::isfinite(x); });
}

} // namespace

std::expected<Calibration, Error> Calibration::parse(std::string_view text) {
    Calibration cal = identity(FrameConv{}, 0.064);
    std::string section;
    std::array<bool, 2> seen{false, false};
    const auto all = detail::lines(text);
    for (std::size_t n = 0; n < all.size(); n++) {
        const auto raw = all[n];
        const auto line = detail::trim(raw.substr(0, raw.find('#')));
        if (line.empty()) {
            continue;
        }
        const auto err = [n](const std::string& m) { return std::unexpected(Error{std::format("line {}: {}", n + 1, m)}); };
        if (line.size() >= 2 && line.front() == '[' && line.back() == ']') {
            section = std::string(detail::trim(line.substr(1, line.size() - 2)));
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string_view::npos) {
            return err("expected key = value");
        }
        const auto k = detail::trim(line.substr(0, eq));
        const auto v = detail::trim(line.substr(eq + 1));
        const auto unq = [&] { return detail::unquote(v); };
        const auto num = [&]() -> std::expected<double, Error> {
            auto x = detail::parse_f64(v);
            if (!x) {
                return err(std::format("{}: {}", k, x.error()));
            }
            return *x;
        };
        const auto arr = [&]() -> std::expected<std::vector<double>, Error> {
            if (v.size() < 2 || v.front() != '[' || v.back() != ']') {
                return err(std::format("{} must be [a, b, ...]", k));
            }
            auto inner = v.substr(1, v.size() - 2);
            std::vector<double> out;
            while (true) {
                const auto comma = inner.find(',');
                const auto p = detail::trim(inner.substr(0, comma));
                if (!p.empty()) {
                    auto x = detail::parse_f64(p);
                    if (!x) {
                        return err(std::format("{}: {}", k, x.error()));
                    }
                    out.push_back(*x);
                }
                if (comma == std::string_view::npos) {
                    break;
                }
                inner.remove_prefix(comma + 1);
            }
            return out;
        };
// evaluate an expected and leave parse() with its error
#define EM_TRY(var, expr)                                                                                    \
    auto var##_r = (expr);                                                                                   \
    if (!var##_r) {                                                                                          \
        return std::unexpected(var##_r.error());                                                             \
    }                                                                                                        \
    const auto var = *var##_r
        if (section.empty()) {
            if (k == "version") {
                EM_TRY(x, num());
                const auto ver = detail::sat_cast<std::int64_t>(x);
                if (ver != 1 && ver != 2) {
                    return err("unsupported version");
                }
            } else if (k == "created_unix") {
                EM_TRY(x, num());
                cal.created_unix = detail::sat_cast<std::uint64_t>(rmax(x, 0.0));
            } else if (k == "flip") {
                const auto u = unq();
                std::vector<bool> p;
                std::string_view rest = u;
                while (true) {
                    const auto comma = rest.find(',');
                    p.push_back(detail::trim(rest.substr(0, comma)) == "1");
                    if (comma == std::string_view::npos) {
                        break;
                    }
                    rest.remove_prefix(comma + 1);
                }
                if (p.size() != 3) {
                    return err("flip needs 3 values");
                }
                cal.conv.flip_x = p[0];
                cal.conv.flip_y = p[1];
                cal.conv.flip_z = p[2];
            } else if (k == "swap") {
                EM_TRY(x, num());
                cal.conv.swap_eyes = x != 0.0;
            } else if (k == "offset_mm") {
                EM_TRY(a, arr());
                if (a.size() != 3) {
                    return err("offset_mm needs 3 numbers");
                }
                cal.conv.offset_mm = {a[0], a[1], a[2]};
            } else if (k == "depth_m") {
                EM_TRY(x, num());
                cal.combine.depth_m = rclamp(x, 0.2, 100.0);
            } else if (k == "max_disagree_deg") {
                EM_TRY(x, num());
                cal.combine.max_disagree_deg = rmax(x, 0.5);
            } else if (k == "holdoff") {
                EM_TRY(x, num());
                cal.combine.holdoff = detail::sat_cast<std::uint32_t>(rclamp(x, 0.0, 30.0));
            }
        } else if (section == "left" || section == "right") {
            const std::size_t i = section == "right" ? 1 : 0;
            seen[i] = true;
            auto& e = cal.eyes[i];
            if (k == "model") {
                const auto m = unq();
                if (!parse_model(m)) {
                    return err(std::format("unknown model '{}'", m));
                }
                e.map.model = m;
            } else if (k == "u" || k == "v") {
                EM_TRY(a, arr());
                if (!(a.size() == 3 || a.size() == 6) || !all_finite(a)) {
                    return err(std::format("{} needs 3 or 6 finite numbers", k));
                }
                std::array<double, 6> c{};
                for (std::size_t j = 0; j < a.size(); j++) {
                    c[j] = a[j];
                }
                (k == "u" ? e.map.cx : e.map.cy) = c;
            } else if (k == "origin_m") {
                EM_TRY(a, arr());
                if (a.size() != 3) {
                    return err("origin_m needs 3 numbers");
                }
                e.origin_m = {a[0], a[1], a[2]};
            } else if (k == "accuracy_deg") {
                EM_TRY(x, num());
                e.accuracy_deg = rmax(x, 0.0);
            } else if (k == "precision_deg") {
                EM_TRY(x, num());
                e.precision_deg = rmax(x, 0.0);
            } else if (k == "drift") {
                EM_TRY(a, arr());
                if (a.size() != 2) {
                    return err("drift needs 2 numbers");
                }
                e.drift = {a[0], a[1]};
            } else if (k == "origin_ref_m" || k == "origin_u" || k == "origin_v") {
                EM_TRY(a, arr());
                if (a.size() != 3 || !all_finite(a)) {
                    return err(std::format("{} needs 3 finite numbers", k));
                }
                const Vec3 t{a[0], a[1], a[2]};
                if (k == "origin_ref_m") {
                    e.org.reference_m = t;
                } else if (k == "origin_u") {
                    e.org.u = t;
                } else {
                    e.org.v = t;
                }
            } else if (k == "origin_gate_mm") {
                EM_TRY(x, num());
                e.org.gate_mm = rclamp(x, 0.0, 100.0);
            } else if (k == "pupil_ref_mm") {
                EM_TRY(x, num());
                e.org.pupil_ref_mm = rclamp(x, 0.0, 20.0);
            } else if (k == "pupil_uv") {
                EM_TRY(a, arr());
                if (a.size() != 2 || !all_finite(a)) {
                    return err("pupil_uv needs 2 finite numbers");
                }
                e.org.pupil_u = a[0];
                e.org.pupil_v = a[1];
            }
        } else if (section == "filter") {
            auto& f = cal.filter;
            if (k == "kind") {
                f.enabled = unq() == "one-euro";
            } else if (k == "min_cutoff_hz") {
                EM_TRY(x, num());
                f.min_cutoff_hz = rclamp(x, 0.01, 100.0);
            } else if (k == "beta") {
                EM_TRY(x, num());
                f.beta = rclamp(x, 0.0, 10.0);
            } else if (k == "d_cutoff_hz") {
                EM_TRY(x, num());
                f.d_cutoff_hz = rclamp(x, 0.01, 100.0);
            } else if (k == "snap_deg") {
                EM_TRY(x, num());
                f.snap_deg = rclamp(x, 0.0, 90.0);
            }
        }
#undef EM_TRY
    }
    if (!(seen[0] && seen[1])) {
        return std::unexpected(Error{"both [left] and [right] sections are required"});
    }
    return cal;
}

std::expected<Calibration, Error> Calibration::load(const fs::path& path) {
    const auto t = detail::read_text(path.c_str());
    if (!t) {
        return std::unexpected(Error{std::format("{}: {}", path.string(), t.error())});
    }
    auto c = parse(*t);
    if (!c) {
        return std::unexpected(Error{std::format("{}: {}", path.string(), c.error().message)});
    }
    return c;
}

std::expected<void, Error> Calibration::save(const fs::path& path) const {
    const auto r = detail::write_text_atomic(path.c_str(), to_toml());
    if (!r) {
        return std::unexpected(Error{std::format("{}: {}", path.string(), r.error())});
    }
    return {};
}

std::optional<fs::path> Calibration::default_path() {
    const auto h = home_dir();
    if (!h) {
        return std::nullopt;
    }
    return candidates(*h)[0];
}

std::optional<fs::path> Calibration::find_default(const std::optional<fs::path>& home) {
    if (!home) {
        return std::nullopt;
    }
    for (const auto& p : candidates(*home)) {
        std::error_code ec;
        if (fs::exists(p, ec)) {
            return p;
        }
    }
    return std::nullopt;
}

std::expected<std::optional<LoadedCalibration>, Error> Calibration::select(std::optional<std::string_view> var,
                                                                           const std::optional<fs::path>& home) {
    const auto v = var ? detail::trim(*var) : std::string_view{};
    if (var && (v == "none" || v == "off" || v == "0")) {
        return std::optional<LoadedCalibration>{};
    }
    std::optional<fs::path> p;
    if (var && !v.empty()) {
        p = fs::path(v);
    } else {
        p = find_default(home);
    }
    if (!p) {
        return std::optional<LoadedCalibration>{};
    }
    auto c = load(*p);
    if (!c) {
        return std::unexpected(c.error());
    }
    return std::optional<LoadedCalibration>{LoadedCalibration{std::move(*c), *p}};
}

std::expected<std::optional<LoadedCalibration>, Error> Calibration::from_env() {
    const char* v = pibridge::env("EYE_CALIBRATION");
    return select(v ? std::optional<std::string_view>(v) : std::nullopt, home_dir());
}

namespace {

Vec3 add(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }

Vec3 scale(const Vec3& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }

std::optional<Vec3> unit(const Vec3& a) {
    const double l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (l > 1e-12 && std::isfinite(l)) {
        return scale(a, 1.0 / l);
    }
    return std::nullopt;
}

double angle_deg(const Vec3& a, const Vec3& b) {
    const Vec3 c{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    const double s = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    return detail::to_degrees(std::atan2(s, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
}

Vec3 f64v(const Vec3f& v) { return {double{v[0]}, double{v[1]}, double{v[2]}}; }

Vec3f f32v(const Vec3& v) { return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])}; }

struct Hit {
    Vec3 point;
    double weight;
    std::size_t eye;
};

} // namespace

void Calibration::apply_sample(Sample& s, CombineState& st) const { apply_sample_with(s, st, {}); }

void Calibration::apply_sample_with(Sample& s, CombineState& st, const std::array<Tangent, 2>& extra) const {
    std::vector<Hit> hits;
    hits.reserve(2);
    for (std::size_t i = 0; i < 2; i++) {
        auto& e = s.eyes[i];
        const bool ok = e.origin_valid && e.direction_valid;
        if (!ok) {
            st.since_valid[i] = 0;
            st.invalid_run_end[i] = combine.holdoff;
            continue;
        }
        const auto raw_origin = f64v(e.origin);
        if (!eyes[i].org.usable(raw_origin)) {
            // the chip's bad state (origin far from where this wearer's eye is): drop the eye like a blink
            e.direction_valid = false;
            st.since_valid[i] = 0;
            st.invalid_run_end[i] = combine.holdoff;
            continue;
        }
        if (st.since_valid[i] != UINT32_MAX) {
            st.since_valid[i]++;
        }
        const auto d = eyes[i].apply_full(f64v(e.direction), raw_origin, double{e.pupil_mm}, extra[i]);
        if (!d) {
            e.direction_valid = false;
            continue;
        }
        e.direction = f32v(*d);
        e.origin = f32v(eyes[i].origin_m);
        if (st.invalid_run_end[i] > 0) {
            // just back from a blink: corrected, published, but not used for the combined ray yet
            st.invalid_run_end[i]--;
            continue;
        }
        const auto& o = eyes[i].origin_m;
        // soft reliability: the farther the reported origin from this wearer's usual one, the less the eye counts
        const double w = eyes[i].weight() * eyes[i].org.reliability(raw_origin);
        hits.push_back({add(o, scale(*d, combine.depth_m)), w, i});
    }
    const auto mid = scale(add(eyes[0].origin_m, eyes[1].origin_m), 0.5);
    if (hits.size() == 2) {
        const auto da = unit(add(hits[0].point, scale(mid, -1.0)));
        const auto db = unit(add(hits[1].point, scale(mid, -1.0)));
        if (da && db && angle_deg(*da, *db) > combine.max_disagree_deg) {
            // keep the more trustworthy eye
            const std::size_t keep = hits[0].weight >= hits[1].weight ? 0 : 1;
            hits = {hits[keep]};
        }
    }
    double wsum = 0.0;
    for (const auto& h : hits) {
        wsum += h.weight;
    }
    s.gaze.reset();
    if (!hits.empty() && wsum > 0.0) {
        Vec3 p{0.0, 0.0, 0.0};
        std::uint8_t mask = 0;
        for (const auto& h : hits) {
            p = add(p, scale(h.point, h.weight / wsum));
            mask = static_cast<std::uint8_t>(mask | (1U << h.eye));
        }
        if (const auto d = unit(add(p, scale(mid, -1.0)))) {
            s.gaze = Gaze{f32v(mid), f32v(*d), mask};
        }
    }
}

std::optional<std::pair<Vec3, Vec3>> raw_ray(const EyeRay& e) {
    if (e.origin_valid && e.direction_valid) {
        return std::pair{f64v(e.origin), f64v(e.direction)};
    }
    return std::nullopt;
}

} // namespace pibridge::eyemath
