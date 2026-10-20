// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/p3p.rs (MIT OR Apache-2.0).
// Every expression keeps the Rust evaluation order (built with -ffp-contract=off): the roots and poses are
// bit-identical to the Rust ones.
#include "pibridge/ctrl/track/p3p.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
using cam::rfmt::fmax;

namespace {

struct C {
    double re;
    double im;
    C add(C o) const { return C{re + o.re, im + o.im}; }
    C sub(C o) const { return C{re - o.re, im - o.im}; }
    C mul(C o) const { return C{re * o.re - im * o.im, re * o.im + im * o.re}; }
    C div(C o) const {
        const double d = o.re * o.re + o.im * o.im;
        return C{(re * o.re + im * o.im) / d, (im * o.re - re * o.im) / d};
    }
};

} // namespace

std::vector<double> quartic_real_roots(const std::array<double, 5>& c) {
    double cmax = 0.0;
    for (const double x : c) {
        cmax = fmax(cmax, std::fabs(x));
    }
    if (std::fabs(c[0]) < 1e-14 * cmax) {
        return {};
    }
    std::array<double, 5> a{};
    for (int i = 0; i < 5; i++) {
        a[i] = c[i] / c[0];
    }
    const auto eval = [&](C z) {
        C r{1.0, 0.0};
        for (int k = 1; k < 5; k++) {
            r = r.mul(z).add(C{a[k], 0.0});
        }
        return r;
    };
    double amax = 0.0;
    for (int k = 1; k < 5; k++) {
        amax = fmax(amax, std::fabs(a[k]));
    }
    const double bound = 1.0 + amax;
    const C seed{0.4, 0.9};
    std::array<C, 4> z{C{1.0, 0.0}, C{1.0, 0.0}, C{1.0, 0.0}, C{1.0, 0.0}};
    for (int i = 1; i < 4; i++) {
        z[i] = z[i - 1].mul(seed);
    }
    for (C& zi : z) {
        zi = C{zi.re * bound * 0.5, zi.im * bound * 0.5};
    }
    for (int it = 0; it < 80; it++) {
        double delta = 0.0;
        for (int i = 0; i < 4; i++) {
            C den{1.0, 0.0};
            for (int j = 0; j < 4; j++) {
                if (i != j) {
                    den = den.mul(z[i].sub(z[j]));
                }
            }
            if (den.re == 0.0 && den.im == 0.0) {
                den = C{1e-12, 0.0};
            }
            const C step = eval(z[i]).div(den);
            z[i] = z[i].sub(step);
            delta = fmax(delta, std::fabs(step.re) + std::fabs(step.im));
        }
        if (delta < 1e-9 * bound) {
            break;
        }
    }
    const auto p = [&](double x) {
        double r = 1.0;
        for (int k = 1; k < 5; k++) {
            r = r * x + a[k];
        }
        return r;
    };
    const auto dp = [&](double x) { return 4.0 * x * x * x + 3.0 * a[1] * x * x + 2.0 * a[2] * x + a[3]; };
    std::vector<double> out;
    for (const C zi : z) {
        if (std::fabs(zi.im) <= 1e-6 * (1.0 + std::fabs(zi.re))) {
            double x = zi.re;
            for (int k = 0; k < 4; k++) {
                const double d = dp(x);
                if (std::fabs(d) < 1e-300) {
                    break;
                }
                x -= p(x) / d;
            }
            out.push_back(x);
        }
    }
    return out;
}

std::optional<g::Pose> triad_align(const std::array<g::Vec3, 3>& m, const std::array<g::Vec3, 3>& c) {
    const auto frame = [](const std::array<g::Vec3, 3>& p) -> std::optional<g::Mat3> {
        const g::Vec3 e1 = g::normalize(g::sub(p[1], p[0]));
        g::Vec3 e3 = g::cross(e1, g::sub(p[2], p[0]));
        if (g::norm(e3) < 1e-12) {
            return std::nullopt;
        }
        e3 = g::normalize(e3);
        const g::Vec3 e2 = g::cross(e3, e1);
        return g::from_cols(e1, e2, e3);
    };
    const auto fm = frame(m);
    if (!fm) {
        return std::nullopt;
    }
    const auto fc = frame(c);
    if (!fc) {
        return std::nullopt;
    }
    const g::Mat3 r = g::mat_mul(*fc, g::transpose(*fm));
    const g::Vec3 cm = g::scale(g::add(g::add(m[0], m[1]), m[2]), 1.0 / 3.0);
    const g::Vec3 cc = g::scale(g::add(g::add(c[0], c[1]), c[2]), 1.0 / 3.0);
    return g::Pose{r, g::sub(cc, g::mat_vec(r, cm))};
}

std::vector<g::Pose> p3p(const std::array<g::Vec3, 3>& y_in, const std::array<g::Vec3, 3>& x) {
    const std::array<g::Vec3, 3> y{g::normalize(y_in[0]), g::normalize(y_in[1]), g::normalize(y_in[2])};
    // side lengths: a opposite x0 (x1-x2), b opposite x1 (x0-x2), c opposite x2 (x0-x1)
    const g::Vec3 da = g::sub(x[1], x[2]);
    const double a2 = g::dot(da, da);
    const g::Vec3 db = g::sub(x[0], x[2]);
    const double b2 = g::dot(db, db);
    const g::Vec3 dc = g::sub(x[0], x[1]);
    const double c2 = g::dot(dc, dc);
    if (a2 < 1e-12 || b2 < 1e-12 || c2 < 1e-12) {
        return {};
    }
    const double ca = g::dot(y[1], y[2]); // cos alpha
    const double cb = g::dot(y[0], y[2]); // cos beta
    const double cg = g::dot(y[0], y[1]); // cos gamma
    const double amc = (a2 - c2) / b2;
    const double apc = (a2 + c2) / b2;
    const double bmc = (b2 - c2) / b2;
    const double bma = (b2 - a2) / b2;
    const double amc1 = amc - 1.0;
    const double c4 = amc1 * amc1 - 4.0 * c2 / b2 * ca * ca;
    const double c3 = 4.0 * (amc * (1.0 - amc) * cb - (1.0 - apc) * ca * cg + 2.0 * c2 / b2 * ca * ca * cb);
    const double c2q = 2.0 * (amc * amc - 1.0 + 2.0 * amc * amc * cb * cb + 2.0 * bmc * ca * ca -
                              4.0 * apc * ca * cb * cg + 2.0 * bma * cg * cg);
    const double c1 = 4.0 * (-amc * (1.0 + amc) * cb + 2.0 * a2 / b2 * cg * cg * cb - (1.0 - apc) * ca * cg);
    const double apc1 = 1.0 + amc;
    const double c0 = apc1 * apc1 - 4.0 * a2 / b2 * cg * cg;
    std::vector<g::Pose> out;
    for (const double v : quartic_real_roots({c4, c3, c2q, c1, c0})) {
        if (v <= 0.0) {
            continue;
        }
        const double den = 2.0 * (cg - v * ca);
        if (std::fabs(den) < 1e-12) {
            continue;
        }
        const double u = ((-1.0 + amc) * v * v - 2.0 * amc * cb * v + 1.0 + amc) / den;
        if (u <= 0.0) {
            continue;
        }
        const double s1sq = c2 / (1.0 + u * u - 2.0 * u * cg);
        if (s1sq <= 0.0) {
            continue;
        }
        const double s1 = std::sqrt(s1sq);
        const std::array<g::Vec3, 3> pc{g::scale(y[0], s1), g::scale(y[1], u * s1), g::scale(y[2], v * s1)};
        if (const auto p = triad_align(x, pc)) {
            out.push_back(*p);
        }
    }
    return out;
}

} // namespace pibridge::ctrl::track
