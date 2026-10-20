// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/model.rs (MIT OR Apache-2.0).
#include "pibridge/lens/model.h"

#include "pibridge/lens/fmt.h"

#include <cmath>
#include <limits>

namespace pibridge::lens {

namespace {

// How far (in client-image UV units) a fully hidden point is pushed out of the image (clamp-to-border-black).
constexpr float PUSH = 4.0f;

float horner(const Poly& k, float r2) {
    return k[0] + r2 * (k[1] + r2 * (k[2] + r2 * (k[3] + r2 * k[4])));
}

float smoothstep(float t) {
    t = fmt::clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

std::pair<double, double> radial_tangent(double base, double scale, const Poly& k, double r) {
    const double r2 = r * r;
    double poly = 1.0;
    double dpoly = 1.0;
    double pw = 1.0;
    for (std::size_t i = 0; i < K_TERMS; i++) {
        pw *= r2;
        poly += double{k[i]} * pw;
        dpoly += static_cast<double>(2 * (i + 1) + 1) * double{k[i]} * pw;
    }
    return {base * scale * r * poly, base * scale * dpoly};
}

Lens::Lens(LensConfig cfg) : cfg_(std::move(cfg)) {
    for (std::size_t e = 0; e < 2; e++) {
        for (std::size_t i = 0; i < 4; i++) {
            tans_[e][i] = std::tan(deg2rad(cfg_.eye[e].fov[i]));
        }
    }
}

Lens Lens::with_luts(std::array<std::shared_ptr<const Lut>, 2> luts) && {
    luts_ = std::move(luts);
    return std::move(*this);
}

bool Lens::has_mask(std::size_t view) const {
    return cfg_.eye[view].has_mask() || luts_[view] != nullptr;
}

std::pair<float, float> Lens::centred(const EyeConfig& e, float u, float v) const {
    const float px = (u - 0.5f) * 2.0f * cfg_.aspect - e.cx;
    const float py = (0.5f - v) * 2.0f - e.cy;
    return {px, py};
}

Uv Lens::tangent(std::size_t view, std::size_t c, float u, float v) const {
    const EyeConfig& e = cfg_.eye[view];
    if (luts_[view]) {
        return luts_[view]->sample(view, c, u, v, e.lut_interp);
    }
    const auto [px, py] = centred(e, u, v);
    return tangent_at(e, c, px, py);
}

Uv Lens::tangent_at(const EyeConfig& e, std::size_t c, float px, float py) const {
    const float x = (px - e.shift[c][0]) / e.ca[c];
    const float y = (py - e.shift[c][1]) / e.ca[c];
    const float r2 = x * x + y * y;
    if (e.fit_range > 0.0f && r2 > e.fit_range * e.fit_range) {
        const float r = std::sqrt(r2);
        const auto [t_end, slope] = radial_tangent(double{e.base[c]}, double{e.scale}, e.k[c], double{e.fit_range});
        const auto t = static_cast<float>(t_end + slope * (double{r} - double{e.fit_range}));
        return {x / r * t, y / r * t};
    }
    const float d = e.base[c] * e.scale * (1.0f + r2 * horner(e.k[c], r2));
    return {x * d, y * d};
}

Uv Lens::tangent_to_uv(std::size_t view, Uv t) const {
    const auto [tl, tr, tu, td] = tans_[view];
    return {(t[0] + tl) / (tl + tr), (tu - t[1]) / (tu + td)};
}

Uv Lens::uv_to_tangent(std::size_t view, Uv uv) const {
    const auto [tl, tr, tu, td] = tans_[view];
    return {uv[0] * (tl + tr) - tl, tu - uv[1] * (tu + td)};
}

Uv3 Lens::compute(std::size_t view, float u, float v) const {
    if (!cfg_.enabled) {
        return {Uv{u, v}, Uv{u, v}, Uv{u, v}};
    }
    const EyeConfig& e = cfg_.eye[view];
    const auto [px, py] = centred(e, u, v);
    Uv3 out{};
    for (std::size_t c = 0; c < CHANNELS; c++) {
        const Uv t = luts_[view] ? luts_[view]->sample(view, c, u, v, e.lut_interp) : tangent_at(e, c, px, py);
        out[c] = tangent_to_uv(view, t);
    }
    if (e.has_mask()) {
        const float hide = hide_amount(view, u, v);
        if (hide > 0.0f) {
            for (Uv& o : out) {
                const float dx = o[0] - 0.5f;
                const float dy = o[1] - 0.5f;
                const float len = std::sqrt(dx * dx + dy * dy);
                float ux = 1.0f;
                float uy = 0.0f;
                if (len > 1e-6f) {
                    ux = dx / len;
                    uy = dy / len;
                }
                o[0] += ux * hide * PUSH;
                o[1] += uy * hide * PUSH;
            }
        }
    }
    return out;
}

float Lens::mask_distance(std::size_t view, float u, float v) const {
    const EyeConfig& e = cfg_.eye[view];
    const float aspect = cfg_.aspect;
    const float xp = (u - 0.5f) * 2.0f * aspect;
    const float yp = (0.5f - v) * 2.0f;
    float s = std::numeric_limits<float>::infinity();
    bool any = false;
    for (const float b : e.blackout) {
        any = any || b > 0.0f;
    }
    if (any) {
        s = std::fmin(s, (xp + aspect) - e.blackout[0]);
        s = std::fmin(s, (aspect - xp) - e.blackout[1]);
        s = std::fmin(s, (1.0f - yp) - e.blackout[2]);
        s = std::fmin(s, (yp + 1.0f) - e.blackout[3]);
    }
    const float px = xp - e.cx;
    const float py = yp - e.cy;
    const Mask& m = e.mask;
    switch (m.shape) {
    case MaskShape::Off:
        break;
    case MaskShape::Ellipse: {
        const float f = std::sqrt(fmt::powi(px / m.a, 2) + fmt::powi(py / m.b, 2));
        s = std::fmin(s, (1.0f - f) * std::fmin(m.a, m.b));
        break;
    }
    case MaskShape::Rect:
        s = std::fmin(s, std::fmin(m.a - std::fabs(px), m.b - std::fabs(py)));
        break;
    case MaskShape::Superellipse: {
        const float f =
            std::pow(std::pow(std::fabs(px / m.a), m.n) + std::pow(std::fabs(py / m.b), m.n), 1.0f / m.n);
        s = std::fmin(s, (1.0f - f) * std::fmin(m.a, m.b));
        break;
    }
    }
    return s;
}

float Lens::hide_amount(std::size_t view, float u, float v) const {
    const EyeConfig& e = cfg_.eye[view];
    if (!e.has_mask()) {
        return 0.0f;
    }
    const float s = mask_distance(view, u, v);
    if (s >= 0.0f) {
        return 0.0f;
    }
    if (e.mask_feather <= 0.0f) {
        return 1.0f;
    }
    return smoothstep(-s / e.mask_feather);
}

std::optional<float> Lens::radius_for_angle(std::size_t view, std::size_t c, float angle_deg) const {
    const EyeConfig& e = cfg_.eye[view];
    const double target = std::tan(double{fmt::to_radians(angle_deg)});
    const auto t_of = [&](double r) -> double {
        const double rr = double{e.fit_range};
        if (e.fit_range > 0.0f && r > rr) {
            const auto [t, s] = radial_tangent(double{e.base[c]}, double{e.scale}, e.k[c], rr);
            return t + s * (r - rr);
        }
        return radial_tangent(double{e.base[c]}, double{e.scale}, e.k[c], r).first;
    };
    double lo = 0.0;
    double hi = 4.0;
    if (t_of(hi) < target) {
        return std::nullopt;
    }
    for (int i = 0; i < 80; i++) {
        const double mid = 0.5 * (lo + hi);
        if (t_of(mid) < target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return static_cast<float>(0.5 * (lo + hi) * double{e.ca[c]});
}

std::optional<float> Lens::first_fold(std::size_t view, std::size_t c, float rmax) const {
    const EyeConfig& e = cfg_.eye[view];
    const int n = 3000;
    for (int i = 1; i <= n; i++) {
        const double r = double{rmax} * static_cast<double>(i) / static_cast<double>(n);
        const double rr = double{e.fit_range};
        const double slope = e.fit_range > 0.0f && r > rr
                                 ? radial_tangent(double{e.base[c]}, double{e.scale}, e.k[c], rr).second
                                 : radial_tangent(double{e.base[c]}, double{e.scale}, e.k[c], r).second;
        if (slope <= 0.0) {
            return static_cast<float>(r);
        }
    }
    return std::nullopt;
}

} // namespace pibridge::lens
