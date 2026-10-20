// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/synth.rs (MIT OR Apache-2.0). See synth.h.
#include "pibridge/roommap/synth.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"

#include <cmath>

namespace pibridge::roommap::synth {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;

RayTable RayTable::make(const cam::Kb4& intr) {
    RayTable t;
    t.w = static_cast<std::size_t>(intr.width);
    t.h = static_cast<std::size_t>(intr.height);
    t.rays.reserve(t.w * t.h);
    const double offs[4][2] = {{-0.25, -0.25}, {0.25, -0.25}, {-0.25, 0.25}, {0.25, 0.25}};
    for (std::size_t y = 0; y < t.h; y++) {
        for (std::size_t x = 0; x < t.w; x++) {
            std::array<std::array<float, 3>, 4> r{};
            for (std::size_t k = 0; k < 4; k++) {
                if (auto v = intr.unproject(static_cast<double>(x) + offs[k][0], static_cast<double>(y) + offs[k][1])) {
                    r[k] = {static_cast<float>((*v)[0]), static_cast<float>((*v)[1]), static_cast<float>((*v)[2])};
                }
            }
            t.rays.push_back(r);
        }
    }
    return t;
}

namespace {

std::uint64_t hash(std::uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdull;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ull;
    return x ^ (x >> 33);
}

double lattice(std::uint64_t seed, std::int64_t i, std::int64_t j) {
    const std::uint64_t h = hash(seed ^ (static_cast<std::uint64_t>(i) * 0x9E3779B97F4A7C15ull) ^
                                 (static_cast<std::uint64_t>(j) * 0xC2B2AE3D27D4EB4Full));
    return static_cast<double>(h >> 11) / static_cast<double>(1ull << 53);
}

double smooth(double t) {
    return t * t * (3.0 - 2.0 * t);
}

double value_noise(std::uint64_t seed, double u, double v) {
    const std::int64_t i = rs::to_i64(std::floor(u));
    const std::int64_t j = rs::to_i64(std::floor(v));
    const double fu = u - std::floor(u);
    const double fv = v - std::floor(v);
    const double a = lattice(seed, i, j), b = lattice(seed, i + 1, j);
    const double c = lattice(seed, i, j + 1), d = lattice(seed, i + 1, j + 1);
    const double x0 = a + (b - a) * smooth(fu);
    const double x1 = c + (d - c) * smooth(fu);
    return x0 + (x1 - x0) * smooth(fv);
}

double texture(std::uint64_t seed, std::uint64_t face, double u, double v) {
    const std::uint64_t s = seed ^ (face * 0x1000000001b3ull);
    double t = 0.0;
    double amp = 0.5;
    double f = 1.5;
    for (std::uint64_t o = 0; o < 4; o++) {
        t += amp * value_noise(s + o, u * f, v * f);
        amp *= 0.5;
        f *= 2.3;
    }
    double val = 40.0 + 150.0 * t;
    const std::int64_t ci = rs::to_i64(std::floor(u / 0.6));
    const std::int64_t cj = rs::to_i64(std::floor(v / 0.6));
    if (lattice(s ^ 0xABCD, ci, cj) < 0.45) {
        const double fu = u / 0.6 - static_cast<double>(ci);
        const double fv = v / 0.6 - static_cast<double>(cj);
        const double w = 0.2 + 0.25 * lattice(s ^ 0x1111, ci, cj);
        const double h = 0.2 + 0.25 * lattice(s ^ 0x2222, ci, cj);
        const double ou = 0.1 + 0.3 * lattice(s ^ 0x3333, ci, cj);
        const double ov = 0.1 + 0.3 * lattice(s ^ 0x4444, ci, cj);
        if (fu > ou && fu < ou + w && fv > ov && fv < ov + h) {
            const bool dark = lattice(s ^ 0x5555, ci, cj) < 0.5;
            val = dark ? 25.0 : 225.0;
            if (lattice(s ^ 0x6666, ci, cj) < 0.4 &&
                (rs::to_i64(std::floor(fu * 20.0)) + rs::to_i64(std::floor(fv * 20.0))) % 2 == 0) {
                val = 255.0 - val;
            }
        }
    }
    return val;
}

std::optional<std::pair<double, std::size_t>> hit_box(const Vec3& o, const Vec3& d, const Aabb& b) {
    std::optional<std::pair<double, std::size_t>> best;
    for (std::size_t axis = 0; axis < 3; axis++) {
        if (std::fabs(d[axis]) < 1e-12) {
            continue;
        }
        const double planes[2] = {b.lo[axis], b.hi[axis]};
        for (std::size_t k = 0; k < 2; k++) {
            const double t = (planes[k] - o[axis]) / d[axis];
            if (t <= 1e-6) {
                continue;
            }
            const Vec3 p = g::add(o, g::scale(d, t));
            bool ok = true;
            for (std::size_t a = 0; a < 3; a++) {
                if (a != axis && !(p[a] >= b.lo[a] - 1e-9 && p[a] <= b.hi[a] + 1e-9)) {
                    ok = false;
                    break;
                }
            }
            if (ok && (!best || t < best->first)) {
                best = std::pair{t, axis * 2 + k};
            }
        }
    }
    return best;
}

} // namespace

std::optional<std::pair<double, double>> Room::trace(const Vec3& o, const Vec3& d) const {
    struct Hit {
        double t;
        std::uint64_t key;
        Vec3 p;
        std::size_t f;
    };
    std::optional<Hit> best;
    if (auto h = hit_box(o, d, shell)) {
        best = Hit{h->first, h->second, g::add(o, g::scale(d, h->first)), h->second};
    }
    for (std::size_t bi = 0; bi < boxes.size(); bi++) {
        if (auto h = hit_box(o, d, boxes[bi])) {
            if (!best || h->first < best->t) {
                best = Hit{h->first, 100 + bi * 6 + h->second, g::add(o, g::scale(d, h->first)), h->second};
            }
        }
    }
    if (!best) {
        return std::nullopt;
    }
    const std::size_t axis = best->f / 2;
    double u, v;
    if (axis == 0) {
        u = best->p[2];
        v = best->p[1];
    } else if (axis == 1) {
        u = best->p[0];
        v = best->p[2];
    } else {
        u = best->p[0];
        v = best->p[1];
    }
    return std::pair{best->t, rf::clamp(texture(seed, best->key, u, v) * gain, 0.0, 255.0)};
}

std::vector<std::uint8_t> Room::render(const cam::Kb4& intr, const Pose& room_from_cam) const {
    return render_rays(RayTable::make(intr), room_from_cam);
}

std::vector<std::uint8_t> Room::render_rays(const RayTable& rays, const Pose& room_from_cam) const {
    std::vector<std::uint8_t> img(rays.w * rays.h, 0);
    for (std::size_t i = 0; i < rays.rays.size(); i++) {
        double sum = 0.0;
        int n = 0;
        for (const auto& r : rays.rays[i]) {
            if (r[2] == 0.0f && r[0] == 0.0f && r[1] == 0.0f) {
                continue;
            }
            const Vec3 d = g::normalize(g::mat_vec(
                room_from_cam.r, {static_cast<double>(r[0]), static_cast<double>(r[1]), static_cast<double>(r[2])}));
            if (auto h = trace(room_from_cam.t, d)) {
                sum += h->second;
                n++;
            }
        }
        if (n > 0) {
            img[i] = rs::to_u8(rs::round_fast(sum / static_cast<double>(n)));
        }
    }
    return img;
}

std::optional<double> Room::depth(const Pose& room_from_cam, const Vec3& ray_cam) const {
    const Vec3 d = g::normalize(g::mat_vec(room_from_cam.r, ray_cam));
    if (auto h = trace(room_from_cam.t, d)) {
        return h->first;
    }
    return std::nullopt;
}

} // namespace pibridge::roommap::synth
