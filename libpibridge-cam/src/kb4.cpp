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
    return std::pair{fx * td * p[0] / r + cx, fy * td * p[1] / r + cy};
}

std::optional<geom::Vec3> Kb4::unproject(double u, double v) const {
    const double mx = (u - cx) / fx;
    const double my = (v - cy) / fy;
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
