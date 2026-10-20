// SPDX-License-Identifier: MIT
#include "sky.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>

namespace pibridge::home::sky {

namespace {

Pano halve(const Pano& p) {
    Pano o;
    o.w = p.w / 2;
    o.h = p.h / 2;
    o.rgb.resize(static_cast<std::size_t>(o.w) * o.h * 3);
    for (int y = 0; y < o.h; y++) {
        for (int x = 0; x < o.w; x++) {
            for (int c = 0; c < 3; c++) {
                const auto at = [&](int xx, int yy) {
                    return static_cast<int>(p.rgb[(static_cast<std::size_t>(yy) * p.w + xx) * 3 + c]);
                };
                const int v = at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) + at(2 * x + 1, 2 * y + 1);
                o.rgb[(static_cast<std::size_t>(y) * o.w + x) * 3 + c] = static_cast<std::uint8_t>((v + 2) / 4);
            }
        }
    }
    return o;
}

// a small deterministic hash for the stars
std::uint32_t hash(std::uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

} // namespace

std::expected<Pano, std::string> loadPpm(const std::string& path, int maxW) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::unexpected("cannot open " + path);
    }
    std::string magic;
    int w = 0, h = 0, mx = 0;
    in >> magic;
    // comments between the fields
    const auto num = [&](int& v) {
        while (in >> std::ws && in.peek() == '#') {
            std::string line;
            std::getline(in, line);
        }
        in >> v;
    };
    num(w);
    num(h);
    num(mx);
    in.get();
    if (magic != "P6" || w <= 0 || h <= 0 || mx != 255 || w > 16384 || h > 8192) {
        return std::unexpected(path + " is not an 8-bit binary PPM");
    }
    Pano p;
    p.w = w;
    p.h = h;
    p.rgb.resize(static_cast<std::size_t>(w) * h * 3);
    in.read(reinterpret_cast<char*>(p.rgb.data()), static_cast<std::streamsize>(p.rgb.size()));
    if (in.gcount() != static_cast<std::streamsize>(p.rgb.size())) {
        return std::unexpected(path + " is cut short");
    }
    while (p.w > maxW && p.w % 2 == 0 && p.h % 2 == 0) {
        p = halve(p);
    }
    return p;
}

Pano dusk(int w, int h) {
    Pano p;
    p.w = w;
    p.h = h;
    p.rgb.resize(static_cast<std::size_t>(w) * h * 3);
    for (int y = 0; y < h; y++) {
        const double lat = (0.5 - (y + 0.5) / h) * std::numbers::pi; // + up
        for (int x = 0; x < w; x++) {
            const double lon = ((x + 0.5) / w - 0.5) * 2 * std::numbers::pi; // 0 ahead
            std::array<double, 3> c{};
            if (lat >= 0) {
                // zenith deep blue, towards the horizon violet and a warm glow ahead-left where the sun went down
                const double t = std::pow(1.0 - lat / (std::numbers::pi / 2), 3.0);
                const double glow = std::exp(-std::pow((lon + 0.6) / 0.9, 2)) * std::exp(-lat * 9.0);
                c = {0.04 + 0.30 * t + 0.65 * glow, 0.05 + 0.16 * t + 0.30 * glow, 0.16 + 0.28 * t + 0.08 * glow};
                const std::uint32_t r = hash(static_cast<std::uint32_t>(y * w + x));
                if (lat > 0.12 && (r & 0x3ff) == 0) {
                    const double s = 0.4 + 0.6 * ((r >> 10) & 0xff) / 255.0;
                    c = {c[0] + s * (1 - t), c[1] + s * (1 - t), c[2] + s * (1 - t)};
                }
            } else {
                // a dark plain, a little lighter at the horizon
                const double t = std::exp(lat * 14.0);
                c = {0.025 + 0.10 * t, 0.03 + 0.07 * t, 0.045 + 0.08 * t};
            }
            for (int k = 0; k < 3; k++) {
                const double v = std::pow(std::clamp(c[static_cast<std::size_t>(k)], 0.0, 1.0), 1.0 / 2.2);
                p.rgb[(static_cast<std::size_t>(y) * w + x) * 3 + k] = static_cast<std::uint8_t>(std::lround(v * 255));
            }
        }
    }
    return p;
}

std::vector<std::uint8_t> toRgba(const Pano& p) {
    std::vector<std::uint8_t> o(static_cast<std::size_t>(p.w) * p.h * 4);
    for (std::size_t i = 0, n = static_cast<std::size_t>(p.w) * p.h; i < n; i++) {
        o[i * 4] = p.rgb[i * 3];
        o[i * 4 + 1] = p.rgb[i * 3 + 1];
        o[i * 4 + 2] = p.rgb[i * 3 + 2];
        o[i * 4 + 3] = 255;
    }
    return o;
}

void dirToSt(room::Vec3 d, double yaw, double& s, double& t) {
    d = d.norm();
    const double lon = std::atan2(d.x, -d.z) + yaw;
    const double lat = std::asin(std::clamp(d.y, -1.0, 1.0));
    s = lon / (2 * std::numbers::pi) + 0.5;
    s -= std::floor(s);
    t = 0.5 - lat / std::numbers::pi;
}

std::array<std::uint8_t, 3> sample(const Pano& p, room::Vec3 d, double yaw) {
    if (p.w == 0) {
        return {0, 0, 0};
    }
    double s = 0, t = 0;
    dirToSt(d, yaw, s, t);
    const double px = s * p.w - 0.5, py = std::clamp(t * p.h - 0.5, 0.0, p.h - 1.0);
    const int x0 = static_cast<int>(std::floor(px)), y0 = static_cast<int>(std::floor(py));
    const double ax = px - x0, ay = py - y0;
    std::array<std::uint8_t, 3> o{};
    for (int c = 0; c < 3; c++) {
        const auto at = [&](int x, int y) {
            x = ((x % p.w) + p.w) % p.w;
            y = std::clamp(y, 0, p.h - 1);
            return static_cast<double>(p.rgb[(static_cast<std::size_t>(y) * p.w + x) * 3 + c]);
        };
        const double v = (at(x0, y0) * (1 - ax) + at(x0 + 1, y0) * ax) * (1 - ay) + (at(x0, y0 + 1) * (1 - ax) + at(x0 + 1, y0 + 1) * ax) * ay;
        o[static_cast<std::size_t>(c)] = static_cast<std::uint8_t>(std::lround(v));
    }
    return o;
}

room::Image render(const Pano& p, const room::Mat34& e, double tanL, double tanR, double tanU, double tanD, int w, int h,
                   double yaw) {
    room::Image img{w, h, std::vector<std::uint8_t>(static_cast<std::size_t>(w) * h * 3)};
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const double tx = tanL + (tanR - tanL) * (x + 0.5) / w, ty = tanU + (tanD - tanU) * (y + 0.5) / h;
            const room::Vec3 d = e.axis(0) * tx + e.axis(1) * ty + e.axis(2) * -1.0;
            const auto c = sample(p, d, yaw);
            std::copy(c.begin(), c.end(), img.rgb.begin() + (static_cast<std::ptrdiff_t>(y) * w + x) * 3);
        }
    }
    return img;
}

} // namespace pibridge::home::sky
