// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/layout.rs (MIT OR Apache-2.0).
#include "layout.h"

#include "pibridge/eye/calibration.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace pibridge::eyecal {

namespace {
constexpr double rad(double d) { return d * std::numbers::pi / 180.0; }
constexpr double deg(double r) { return r * 180.0 / std::numbers::pi; }
} // namespace

Vec3 dir_az_el(double az, double el) {
    const double a = rad(az);
    const double e = rad(el);
    return {std::sin(a) * std::cos(e), std::sin(e), -std::cos(a) * std::cos(e)};
}

Vec3 Target::dir() const { return dir_az_el(az, el); }

Vec3 Target::point() const {
    const Vec3 d = dir();
    return {d[0] * dist_m, d[1] * dist_m, d[2] * dist_m};
}

double Target::eccentricity() const { return deg(std::acos(std::clamp(-dir()[2], -1.0, 1.0))); }

std::string Target::name() const {
    // Rust `{:+.0}` rounds half to even like std::format; a negative zero prints as "-0" in both
    return std::format("{:+.0f},{:+.0f}", az, el);
}

std::vector<Target> calibration_targets(std::size_t points) {
    std::vector<Target> v = {
        Target::at(0.0, 0.0),        Target::at(-CAL_AZ, CAL_EL), Target::at(0.0, CAL_EL),
        Target::at(CAL_AZ, CAL_EL),  Target::at(CAL_AZ, 0.0),     Target::at(CAL_AZ, -CAL_EL),
        Target::at(0.0, -CAL_EL),    Target::at(-CAL_AZ, -CAL_EL), Target::at(-CAL_AZ, 0.0),
    };
    if (points >= 13) {
        v.push_back(Target::at(-7.5, 6.0));
        v.push_back(Target::at(7.5, 6.0));
        v.push_back(Target::at(7.5, -6.0));
        v.push_back(Target::at(-7.5, -6.0));
    }
    return v;
}

std::vector<Target> pimax_targets() {
    return {
        Target::at(0.0, 0.0),
        Target::at(-CAL_AZ, -CAL_EL),
        Target::at(CAL_AZ, CAL_EL),
        Target::at(CAL_AZ, -CAL_EL),
        Target::at(-CAL_AZ, CAL_EL),
        Target::at(0.0, 0.0),
        Target::on_white(-CAL_AZ, 0.0),
        Target::on_white(CAL_AZ, 0.0),
        Target::on_white(0.0, 0.0),
        Target::on_white(0.0, CAL_EL),
        Target::on_white(0.0, -CAL_EL),
    };
}

std::vector<Target> validation_targets() {
    return {
        Target::at(4.0, 3.0),   Target::at(10.0, 0.0),  Target::at(12.0, 9.0),   Target::at(0.0, 8.0),
        Target::at(-12.0, 9.0), Target::at(-4.0, -3.0), Target::at(-10.0, 0.0),  Target::at(-12.0, -9.0),
        Target::at(0.0, -8.0),  Target::at(12.0, -9.0),
    };
}

std::vector<Target> recenter_targets() {
    return {Target::at(0.0, 0.0), Target::at(8.0, 0.0), Target::at(0.0, 6.0), Target::at(-8.0, 0.0),
            Target::at(0.0, -6.0)};
}

std::vector<Target> service_targets() {
    return {Target::at(0.0, 0.0), Target::at(10.0, 0.0), Target::at(0.0, 8.0), Target::at(-10.0, 0.0),
            Target::at(0.0, -8.0)};
}

std::vector<Target> chip_targets(const ChipLayout& l) {
    namespace cal = pibridge::eye::calibration;
    std::vector<Target> out;
    const auto& table = cal::targets();
    for (std::size_t i = 0; i < table.size(); i++) {
        const auto& t = table[i];
        const double x = l.mirror_x ? -t.x_mm : t.x_mm;
        const double y = l.mirror_y ? -t.y_mm : t.y_mm;
        // the chip's point (mm, its frame) in the head frame, like a gaze origin (FrameConv::origin)
        const Vec3 p = l.conv.origin({x, y, static_cast<double>(t.z_mm)});
        const double n = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        Target g;
        g.az = deg(std::atan2(p[0], -p[2]));
        g.el = deg(std::atan2(p[1], std::hypot(p[0], p[2])));
        g.bright = std::ranges::find(cal::PASS2_TARGETS, i) != cal::PASS2_TARGETS.end();
        g.chip = static_cast<int>(i);
        g.dist_m = n * 1e-3;
        out.push_back(g);
    }
    return out;
}

const char* region_name(Region r) {
    switch (r) {
    case Region::Centre: return "centre";
    case Region::Mid: return "mid";
    case Region::Outer: return "outer";
    }
    return "?";
}

Region region_of(const Target& t) {
    const double e = t.eccentricity();
    if (e <= 7.0 + 1e-9) {
        return Region::Centre;
    }
    if (e <= 12.5) {
        return Region::Mid;
    }
    return Region::Outer;
}

Target pursuit_at(double t) {
    const double a = 2.0 * std::numbers::pi * t / PURSUIT_PERIOD_S;
    return Target::at(PURSUIT_RADIUS_DEG * std::cos(a), PURSUIT_RADIUS_DEG * 0.8 * std::sin(a));
}

} // namespace pibridge::eyecal
