// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/track.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/track.h"
#include "pibridge_env.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string_view>
#include <vector>

#include "num.h"

namespace pibridge::eyemath {

namespace {

bool env_flag(const char* name, bool def) {
    const char* v = pibridge::env(name);
    if (v == nullptr) {
        return def;
    }
    const auto t = detail::trim(v);
    return t == "1" || t == "true" || t == "yes" || t == "on";
}

std::optional<Vec3> norm(const Vec3& v) {
    const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l > 1e-9 && std::isfinite(l)) {
        return Vec3{v[0] / l, v[1] / l, v[2] / l};
    }
    return std::nullopt;
}

Vec3f f32v(const Vec3& v) {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

} // namespace

FrameConv FrameConv::from_env() {
    const FrameConv d;
    Vec3 offset = d.offset_mm;
    if (const char* v = pibridge::env("EYE_OFFSET_MM")) {
        // like the Rust code: the parts that parse are kept, and exactly three are needed
        std::vector<double> p;
        std::string_view s = v;
        while (true) {
            const auto comma = s.find(',');
            const auto part = detail::trim(s.substr(0, comma));
            if (auto x = detail::parse_f64(part)) {
                p.push_back(*x);
            }
            if (comma == std::string_view::npos) {
                break;
            }
            s.remove_prefix(comma + 1);
        }
        if (p.size() == 3) {
            offset = {p[0], p[1], p[2]};
        }
    }
    FrameConv c;
    c.flip_x = env_flag("EYE_FLIP_X", d.flip_x);
    c.flip_y = env_flag("EYE_FLIP_Y", d.flip_y);
    c.flip_z = env_flag("EYE_FLIP_Z", d.flip_z);
    c.offset_mm = offset;
    c.swap_eyes = env_flag("EYE_SWAP", d.swap_eyes);
    return c;
}

Vec3 FrameConv::axes(const Vec3& v) const {
    return {flip_x ? -v[0] : v[0], flip_y ? -v[1] : v[1], flip_z ? -v[2] : v[2]};
}

Vec3 FrameConv::origin(const Vec3& v) const {
    const auto o = axes(v);
    return {o[0] + offset_mm[0], o[1] + offset_mm[1], o[2] + offset_mm[2]};
}

Vec3 FrameConv::direction(const Vec3& v) const { return axes(v); }

std::size_t FrameConv::slot(bool a) const {
    // eye A has positive device x: right when x is not flipped
    const bool a_is_right = !flip_x;
    return static_cast<std::size_t>(a == a_is_right) ^ static_cast<std::size_t>(swap_eyes);
}

Sample convert(const GazeSample& g, const FrameConv& conv, std::uint64_t seq, std::int64_t host_ts_ns,
               std::int64_t arrival_ns) {
    std::array<EyeRay, 2> eyes{};
    Vec3 sum_o{0.0, 0.0, 0.0};
    Vec3 sum_d{0.0, 0.0, 0.0};
    std::uint32_t n = 0;
    std::uint8_t mask = 0;
    for (std::size_t i = 0; i < g.eyes.size(); i++) {
        const auto& e = g.eyes[i];
        const auto slot = conv.slot(i == 0);
        const auto o_mm = conv.origin(e.origin_mm);
        const auto dir = norm(conv.direction(e.direction));
        const bool dir_valid = e.direction_valid && dir.has_value();
        EyeRay r;
        r.origin = f32v({o_mm[0] / 1000.0, o_mm[1] / 1000.0, o_mm[2] / 1000.0});
        r.direction = dir ? f32v(*dir) : Vec3f{0.0F, 0.0F, 0.0F};
        r.origin_valid = e.origin_valid;
        r.direction_valid = dir_valid;
        r.pupil_mm = static_cast<float>(e.pupil_diameter_mm);
        r.pupil_pos = {static_cast<float>(e.pupil_pos[0]), static_cast<float>(e.pupil_pos[1])};
        r.pupil_pos_valid = e.pupil_pos_valid;
        r.openness = static_cast<float>(e.openness_raw);
        eyes[slot] = r;
        if (e.origin_valid && dir_valid) {
            const auto d = *dir;
            for (std::size_t k = 0; k < 3; k++) {
                sum_o[k] += o_mm[k] / 1000.0;
                sum_d[k] += d[k];
            }
            n++;
            mask = static_cast<std::uint8_t>(mask | (1U << slot));
        }
    }
    std::optional<Gaze> gaze;
    if (n > 0) {
        const double inv = 1.0 / static_cast<double>(n);
        // opposite rays cancel: treat a zero-length mean as "no combined gaze"
        if (const auto d = norm(sum_d)) {
            gaze = Gaze{f32v({sum_o[0] * inv, sum_o[1] * inv, sum_o[2] * inv}), f32v(*d), mask};
        }
    }
    Sample s;
    s.seq = seq;
    s.device_ts_us = g.timestamp_us;
    s.host_ts_ns = host_ts_ns;
    s.arrival_ns = arrival_ns;
    s.eyes = eyes;
    s.gaze = gaze;
    s.ipd_mm = static_cast<float>(g.frame.field_34);
    return s;
}

std::optional<Sample> sample_at(std::span<const Sample> history, std::int64_t t_ns) {
    if (history.empty()) {
        return std::nullopt;
    }
    const auto& first = history.front();
    const auto& last = history.back();
    if (t_ns <= first.host_ts_ns) {
        return first;
    }
    if (t_ns >= last.host_ts_ns) {
        return last;
    }
    const auto it = std::partition_point(history.begin(), history.end(),
                                         [&](const Sample& s) { return s.host_ts_ns <= t_ns; });
    const auto i = static_cast<std::size_t>(it - history.begin());
    const auto& a = history[i - 1];
    const auto& b = history[i];
    const float f = static_cast<float>(t_ns - a.host_ts_ns) /
                    static_cast<float>(std::max<std::int64_t>(b.host_ts_ns - a.host_ts_ns, 1));
    Sample out = f < 0.5F ? a : b;
    out.host_ts_ns = t_ns;
    if (a.gaze && b.gaze) {
        const auto& ga = *a.gaze;
        const auto& gb = *b.gaze;
        const auto mix = [f](const Vec3f& x, const Vec3f& y) {
            return Vec3f{x[0] + (y[0] - x[0]) * f, x[1] + (y[1] - x[1]) * f, x[2] + (y[2] - x[2]) * f};
        };
        const auto d = mix(ga.direction, gb.direction);
        if (const auto dn = norm({double{d[0]}, double{d[1]}, double{d[2]}})) {
            out.gaze = Gaze{mix(ga.origin, gb.origin), f32v(*dn), static_cast<std::uint8_t>(ga.eyes & gb.eyes)};
        }
    }
    return out;
}

} // namespace pibridge::eyemath
