// SPDX-License-Identifier: MIT
#include "pibridge/ctrl/track/quads.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <set>
#include <tuple>

namespace pibridge::ctrl::track {

namespace g = cam::geom;

namespace {

struct C2 {
    double x = 0, y = 0;
};

C2 csub(C2 a, C2 b) { return {a.x - b.x, a.y - b.y}; }
C2 cdiv(C2 a, C2 b) {
    const double d = b.x * b.x + b.y * b.y;
    return {(a.x * b.x + a.y * b.y) / d, (a.y * b.x - a.x * b.y) / d};
}

/// Gnomonic coordinates of four directions about their mean direction.
std::array<C2, 4> plane(const std::array<g::Vec3, 4>& dirs) {
    g::Vec3 c{0, 0, 0};
    for (const auto& v : dirs) {
        c = g::add(c, g::normalize(v));
    }
    c = g::normalize(c);
    const g::Vec3 h = std::abs(c[0]) < 0.9 ? g::Vec3{1, 0, 0} : g::Vec3{0, 1, 0};
    const g::Vec3 u = g::normalize(g::cross(h, c));
    const g::Vec3 v = g::cross(c, u);
    std::array<C2, 4> z{};
    for (std::size_t i = 0; i < 4; i++) {
        const g::Vec3 d = g::normalize(dirs[i]);
        const double w = std::max(g::dot(d, c), 1e-6);
        z[i] = {g::dot(d, u) / w, g::dot(d, v) / w};
    }
    return z;
}

std::array<double, 4> shape_of(const std::array<C2, 4>& z, std::size_t a, std::size_t b, std::size_t c, std::size_t d) {
    const C2 base = csub(z[b], z[a]);
    const C2 wc = cdiv(csub(z[c], z[a]), base);
    const C2 wd = cdiv(csub(z[d], z[a]), base);
    return {wc.x, wc.y, wd.x, wd.y};
}

double dist2(C2 a, C2 b) { return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y); }

/// The other two of {0..3} besides a and b.
std::pair<std::size_t, std::size_t> others(std::size_t a, std::size_t b) {
    std::size_t o[2], n = 0;
    for (std::size_t i = 0; i < 4; i++) {
        if (i != a && i != b) {
            o[n++] = i;
        }
    }
    return {o[0], o[1]};
}

} // namespace

std::array<double, 4> quad_shape(const std::array<g::Vec3, 4>& dirs, std::size_t a, std::size_t b, std::size_t c,
                                 std::size_t d) {
    return shape_of(plane(dirs), a, b, c, d);
}

std::uint64_t QuadIndex::key(const std::array<int, 4>& q) const {
    std::uint64_t k = 0;
    for (const int v : q) {
        k = (k << 16) | static_cast<std::uint16_t>(static_cast<std::int16_t>(v));
    }
    return k;
}

std::shared_ptr<const QuadIndex> QuadIndex::build(const LedModel& model, const QuadParams& p) {
    const auto t0 = std::chrono::steady_clock::now();
    auto ix = std::make_shared<QuadIndex>();
    ix->p_ = p;
    const std::size_t n = model.leds.size();
    if (n < 4 || n > 255) {
        return ix;
    }
    g::Vec3 centre{0, 0, 0};
    for (const Led& l : model.leds) {
        centre = g::add(centre, l.p);
    }
    centre = g::scale(centre, 1.0 / static_cast<double>(n));
    const double cos_face = std::cos(p.facing_deg * M_PI / 180.0);
    // (key, leds) already stored
    std::set<std::pair<std::uint64_t, std::array<std::uint8_t, 4>>> seen;
    const double golden = M_PI * (3.0 - std::sqrt(5.0));
    for (std::size_t di = 0; di < p.directions; di++) {
        const double y = 1.0 - 2.0 * (static_cast<double>(di) + 0.5) / static_cast<double>(p.directions);
        const double r = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double th = golden * static_cast<double>(di);
        const g::Vec3 dir{r * std::cos(th), y, r * std::sin(th)};
        for (const double dist : p.distances) {
            const g::Vec3 eye = g::add(centre, g::scale(dir, dist));
            // the LEDs facing the camera and their directions from it
            std::vector<std::size_t> vis;
            std::vector<g::Vec3> b;
            for (std::size_t i = 0; i < n; i++) {
                const g::Vec3 to_eye = g::sub(eye, model.leds[i].p);
                if (g::dot(g::normalize(to_eye), model.leds[i].n) < cos_face) {
                    continue;
                }
                vis.push_back(i);
                b.push_back(g::normalize(g::scale(to_eye, -1.0)));
            }
            const std::size_t m = vis.size();
            if (m < 4) {
                continue;
            }
            for (std::size_t a = 0; a < m; a++) {
                // the anchor's nearest visible LEDs (angular distance, as pixel distance in the image)
                std::vector<std::pair<double, std::size_t>> nb;
                for (std::size_t j = 0; j < m; j++) {
                    if (j != a) {
                        nb.emplace_back(-g::dot(b[a], b[j]), j);
                    }
                }
                std::sort(nb.begin(), nb.end());
                if (nb.size() > p.knn) {
                    nb.resize(p.knn);
                }
                for (std::size_t i1 = 0; i1 < nb.size(); i1++) {
                    for (std::size_t i2 = i1 + 1; i2 < nb.size(); i2++) {
                        for (std::size_t i3 = i2 + 1; i3 < nb.size(); i3++) {
                            const std::array<std::size_t, 4> q{a, nb[i1].second, nb[i2].second, nb[i3].second};
                            const std::array<g::Vec3, 4> d4{b[q[0]], b[q[1]], b[q[2]], b[q[3]]};
                            const auto z = plane(d4);
                            double dmax = 0;
                            for (std::size_t s = 0; s < 4; s++) {
                                for (std::size_t t = s + 1; t < 4; t++) {
                                    dmax = std::max(dmax, dist2(z[s], z[t]));
                                }
                            }
                            for (std::size_t s = 0; s < 4; s++) {
                                for (std::size_t t = 0; t < 4; t++) {
                                    if (s == t || dist2(z[s], z[t]) < p.base_frac * p.base_frac * dmax) {
                                        continue;
                                    }
                                    const auto [c0, c1] = others(s, t);
                                    for (int ord = 0; ord < 2; ord++) {
                                        const std::size_t c = ord == 0 ? c0 : c1, d = ord == 0 ? c1 : c0;
                                        const auto sh = shape_of(z, s, t, c, d);
                                        // the lookup orders c, d by x: store the other order too when they are close
                                        if (sh[0] > sh[2] + 0.2) {
                                            continue;
                                        }
                                        std::array<int, 4> qk{};
                                        for (int k = 0; k < 4; k++) {
                                            qk[k] = static_cast<int>(std::floor(sh[k] / p.cell));
                                        }
                                        Entry e;
                                        e.led = {static_cast<std::uint8_t>(vis[q[s]]), static_cast<std::uint8_t>(vis[q[t]]),
                                                 static_cast<std::uint8_t>(vis[q[c]]), static_cast<std::uint8_t>(vis[q[d]])};
                                        const std::uint64_t k = ix->key(qk);
                                        if (!seen.insert({k, e.led}).second) {
                                            continue;
                                        }
                                        for (int j = 0; j < 4; j++) {
                                            e.shape[j] = static_cast<float>(sh[j]);
                                        }
                                        ix->map_[k].push_back(e);
                                        ix->entries_++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    ix->build_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return ix;
}

void QuadIndex::lookup(const std::array<g::Vec3, 4>& dirs, double tol, std::vector<Hit>& out) const {
    const std::size_t first = out.size();
    auto z = plane(dirs);
    static const bool mirror = std::getenv("CT_QUAD_MIRROR") != nullptr;
    if (mirror) {
        for (auto& p : z) {
            p.y = -p.y;
        }
    }
    std::size_t a = 0, b = 1;
    double dmax = -1;
    for (std::size_t s = 0; s < 4; s++) {
        for (std::size_t t = s + 1; t < 4; t++) {
            if (const double d = dist2(z[s], z[t]); d > dmax) {
                dmax = d;
                a = s;
                b = t;
            }
        }
    }
    auto [c, d] = others(a, b);
    auto sh = shape_of(z, a, b, c, d);
    if (sh[0] > sh[2]) {
        std::swap(c, d);
        sh = {sh[2], sh[3], sh[0], sh[1]};
    }
    const int r = std::min(2, static_cast<int>(std::ceil(tol / p_.cell)));
    std::array<int, 4> q0{};
    for (int k = 0; k < 4; k++) {
        q0[k] = static_cast<int>(std::floor(sh[k] / p_.cell));
    }
    // every hit appears once per stored base direction: both directions of the base are stored, the lookup finds
    // the one matching its own (a, b); the reverse has a different shape
    std::array<int, 4> q{};
    for (q[0] = q0[0] - r; q[0] <= q0[0] + r; q[0]++) {
        for (q[1] = q0[1] - r; q[1] <= q0[1] + r; q[1]++) {
            for (q[2] = q0[2] - r; q[2] <= q0[2] + r; q[2]++) {
                for (q[3] = q0[3] - r; q[3] <= q0[3] + r; q[3]++) {
                    const auto it = map_.find(key(q));
                    if (it == map_.end()) {
                        continue;
                    }
                    for (const Entry& e : it->second) {
                        bool ok = true;
                        for (int k = 0; k < 4 && ok; k++) {
                            ok = std::abs(static_cast<double>(e.shape[k]) - sh[k]) <= tol;
                        }
                        if (ok) {
                            out.push_back(Hit{{static_cast<std::uint8_t>(a), static_cast<std::uint8_t>(b),
                                               static_cast<std::uint8_t>(c), static_cast<std::uint8_t>(d)},
                                              e.led});
                        }
                    }
                }
            }
        }
    }    // the same LED quad is stored from many neighbouring views: once is enough
    std::sort(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(),
              [](const Hit& x, const Hit& y) { return x.led < y.led; });
    out.erase(std::unique(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(),
                          [](const Hit& x, const Hit& y) { return x.led == y.led; }),
              out.end());
}

} // namespace pibridge::ctrl::track
