// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/kb4.rs (MIT OR Apache-2.0).
#include "pibridge/cam/kb4.h"

#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>

namespace pibridge::cam {

Kb4 Kb4::make(std::uint32_t width, std::uint32_t height, double fx, double fy, double cx, double cy,
              std::array<double, 4> k, std::vector<double> extra) {
    Kb4 m;
    m.width = width;
    m.height = height;
    m.fx = fx;
    m.fy = fy;
    m.cx = cx;
    m.cy = cy;
    m.k = k;
    m.extra = std::move(extra);
    m.use_extra = false;
    m.theta_limit = m.compute_theta_max();
    return m;
}

Kb4 Kb4::with_extra_radial() const {
    Kb4 m = *this;
    m.use_extra = true;
    m.theta_limit = m.compute_theta_max();
    return m;
}

Kb4 Kb4::with_tangential() const {
    Kb4 m = *this;
    m.use_tangential = extra.size() >= 2;
    return m;
}

std::size_t Kb4::n_coeffs() const {
    // k.iter().chain(extra.iter().take(n_extra))
    return 4 + (use_extra ? std::min<std::size_t>(2, extra.size()) : 0);
}

double Kb4::theta_d(double theta) const {
    const double t2 = theta * theta;
    double poly = 0.0;
    double p = t2;
    for (std::size_t i = 0; i < n_coeffs(); i++) {
        poly += coeff(i) * p;
        p *= t2;
    }
    return theta * (1.0 + poly);
}

double Kb4::dtheta_d(double theta) const {
    const double t2 = theta * theta;
    double d = 1.0;
    double p = t2;
    double n = 3.0;
    for (std::size_t i = 0; i < n_coeffs(); i++) {
        d += n * coeff(i) * p;
        p *= t2;
        n += 2.0;
    }
    return d;
}

double Kb4::compute_theta_max() const {
    const double step = 0.0005;
    const double cap = rfmt::to_radians(100.0);
    double t = 0.0;
    while (t < cap && dtheta_d(t + step) > 0.0) {
        t += step;
    }
    return t;
}

std::optional<std::pair<double, double>> Kb4::project(const geom::Vec3& p) const {
    const double r = std::sqrt(p[0] * p[0] + p[1] * p[1]);
    if (r < 1e-12) {
        if (p[2] > 0.0) {
            return std::pair{cx, cy};
        }
        return std::nullopt;
    }
    const double theta = std::atan2(r, p[2]);
    if (theta > theta_limit) {
        return std::nullopt;
    }
    const double td = theta_d(theta);
    double xd = td * p[0] / r;
    double yd = td * p[1] / r;
    if (use_tangential) {
        const double pa = extra[0], pb = extra[1], rd2 = td * td;
        const double xt = xd + pa * (rd2 + 2.0 * xd * xd) + 2.0 * pb * xd * yd;
        const double yt = yd + pb * (rd2 + 2.0 * yd * yd) + 2.0 * pa * xd * yd;
        xd = xt;
        yd = yt;
    }
    return std::pair{fx * xd + cx, fy * yd + cy};
}

std::optional<geom::Vec3> Kb4::unproject(double u, double v) const {
    double mx = (u - cx) / fx;
    double my = (v - cy) / fy;
    if (use_tangential) {
        // invert the tangential step: (xd, yd) such that T(xd, yd) = (mx, my); Gauss-Newton from the measured point
        const double pa = extra[0], pb = extra[1];
        double xd = mx, yd = my;
        for (int i = 0; i < 10; i++) {
            const double rd2 = xd * xd + yd * yd;
            const double fx_ = xd + pa * (rd2 + 2.0 * xd * xd) + 2.0 * pb * xd * yd - mx;
            const double fy_ = yd + pb * (rd2 + 2.0 * yd * yd) + 2.0 * pa * xd * yd - my;
            // Jacobian of T
            const double j00 = 1.0 + 6.0 * pa * xd + 2.0 * pb * yd;
            const double j01 = 2.0 * pa * yd + 2.0 * pb * xd;
            const double j10 = 2.0 * pb * xd + 2.0 * pa * yd;
            const double j11 = 1.0 + 6.0 * pb * yd + 2.0 * pa * xd;
            const double det = j00 * j11 - j01 * j10;
            if (std::fabs(det) < 1e-12) {
                break;
            }
            const double dx = (j11 * fx_ - j01 * fy_) / det;
            const double dy = (-j10 * fx_ + j00 * fy_) / det;
            xd -= dx;
            yd -= dy;
            if (std::fabs(dx) + std::fabs(dy) < 1e-12) {
                break;
            }
        }
        mx = xd;
        my = yd;
    }
    const double td = std::sqrt(mx * mx + my * my);
    if (td < 1e-12) {
        return geom::Vec3{0.0, 0.0, 1.0};
    }
    const double tmax = theta_limit;
    if (td > theta_d(tmax)) {
        return std::nullopt;
    }
    double th = rfmt::fmin(td, tmax);
    for (int i = 0; i < 50; i++) {
        const double f = theta_d(th) - td;
        const double d = dtheta_d(th);
        if (std::fabs(d) < 1e-12) {
            break;
        }
        const double step = f / d;
        th = rfmt::clamp(th - step, 0.0, tmax);
        if (std::fabs(step) < 1e-13) {
            break;
        }
    }
    if (std::fabs(theta_d(th) - td) > 1e-7) {
        return std::nullopt;
    }
    const double s = std::sin(th) / td;
    return geom::Vec3{mx * s, my * s, std::cos(th)};
}

std::optional<double> Kb4::angle_of_pixel_deg(double u, double v) const {
    const auto r = unproject(u, v);
    if (!r) {
        return std::nullopt;
    }
    return rfmt::to_degrees(std::acos(rfmt::clamp((*r)[2], -1.0, 1.0)));
}

} // namespace pibridge::cam
