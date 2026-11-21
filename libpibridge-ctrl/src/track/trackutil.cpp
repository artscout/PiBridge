// SPDX-License-Identifier: MIT
#include "pibridge/ctrl/track/trackutil.h"

#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

std::vector<Obs> obs_list(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                          const std::vector<Matches>& per_cam) {
    std::vector<Obs> o;
    for (std::size_t ci = 0; ci < per_cam.size(); ci++) {
        for (const auto& [l, b] : per_cam[ci]) {
            o.push_back(Obs{&rig.cams[ci], model.leds[l].p, obs[ci].px[b]});
        }
    }
    return o;
}

std::size_t count(const std::vector<Matches>& m) {
    std::size_t t = 0;
    for (const auto& x : m) {
        t += x.size();
    }
    return t;
}

std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) { return a > b ? a - b : 0; }

double angle_deg(const g::Mat3& a, const g::Mat3& b) {
    return rf::to_degrees(g::norm(rotvec(g::mat_mul(a, g::transpose(b)))));
}

/// The rigid map taking a onto b (least squares): the rotation by the polar decomposition (Newton iterations).
std::optional<g::Pose> rigid_fit(const std::vector<g::Vec3>& a, const std::vector<g::Vec3>& b) {
    const std::size_t n = a.size();
    if (n < 3 || b.size() != n) {
        return std::nullopt;
    }
    g::Vec3 ma{0, 0, 0}, mb{0, 0, 0};
    for (std::size_t q = 0; q < n; q++) {
        ma = g::add(ma, g::scale(a[q], 1.0 / static_cast<double>(n)));
        mb = g::add(mb, g::scale(b[q], 1.0 / static_cast<double>(n)));
    }
    g::Mat3 r{};
    for (std::size_t q = 0; q < n; q++) {
        const g::Vec3 da = g::sub(a[q], ma), db = g::sub(b[q], mb);
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                r[x][y] += db[x] * da[y];
            }
        }
    }
    for (int it = 0; it < 40; it++) {
        const double d = g::det(r);
        if (std::abs(d) < 1e-18) {
            return std::nullopt;
        }
        g::Mat3 inv{};
        inv[0] = g::scale(g::cross(r[1], r[2]), 1.0 / d);
        inv[1] = g::scale(g::cross(r[2], r[0]), 1.0 / d);
        inv[2] = g::scale(g::cross(r[0], r[1]), 1.0 / d);
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                r[x][y] = 0.5 * (r[x][y] + inv[x][y]);
            }
        }
    }
    if (g::det(r) < 0.5) {
        return std::nullopt;
    }
    return g::Pose{r, g::sub(mb, g::mat_vec(r, ma))};
}

/// The LED model's near-symmetry: a rigid map, turned at least 120 deg, that takes the LEDs onto LEDs (and the normals
/// onto normals) within 10 mm rms; nullopt without one. Three LEDs onto any three at the same distances (Kabsch).
std::optional<std::pair<g::Pose, double>> model_twin(const LedModel& m) {
    const std::size_t n = m.leds.size();
    if (n < 6) {
        return std::nullopt;
    }
    std::optional<std::pair<g::Pose, double>> best;
    const auto dist = [&](std::size_t a, std::size_t b) { return g::norm(g::sub(m.leds[a].p, m.leds[b].p)); };
    for (std::size_t a0 = 0; a0 < n / 3; a0++) {
    const std::size_t b0 = a0 + n / 3, c0 = a0 + 2 * n / 3;
    for (std::size_t i = 0; i < n; i++) {
        for (std::size_t j = 0; j < n; j++) {
            for (std::size_t k = 0; k < n; k++) {
                if (i == j || j == k || i == k || std::abs(dist(i, j) - dist(a0, b0)) > 0.006 ||
                    std::abs(dist(j, k) - dist(b0, c0)) > 0.006 || std::abs(dist(i, k) - dist(a0, c0)) > 0.006) {
                    continue;
                }
                const std::array<g::Vec3, 3> A{m.leds[a0].p, m.leds[b0].p, m.leds[c0].p};
                const std::array<g::Vec3, 3> B{m.leds[i].p, m.leds[j].p, m.leds[k].p};
                g::Vec3 ma{0, 0, 0}, mb{0, 0, 0};
                for (int q = 0; q < 3; q++) {
                    ma = g::add(ma, g::scale(A[q], 1.0 / 3.0));
                    mb = g::add(mb, g::scale(B[q], 1.0 / 3.0));
                }
                // Kabsch through the polar decomposition of H = sum (b - mb)(a - ma)^T, by Newton iterations
                g::Mat3 h{};
                for (int q = 0; q < 3; q++) {
                    const g::Vec3 da = g::sub(A[q], ma), db = g::sub(B[q], mb);
                    for (int r = 0; r < 3; r++) {
                        for (int cc = 0; cc < 3; cc++) {
                            h[r][cc] += db[r] * da[cc];
                        }
                    }
                }
                // three points only fix the rotation through their plane: add the cross product (the normal) pair
                const g::Vec3 na = g::cross(g::sub(A[1], A[0]), g::sub(A[2], A[0]));
                const g::Vec3 nb = g::cross(g::sub(B[1], B[0]), g::sub(B[2], B[0]));
                for (int r = 0; r < 3; r++) {
                    for (int cc = 0; cc < 3; cc++) {
                        h[r][cc] += nb[r] * na[cc];
                    }
                }
                g::Mat3 r = h;
                for (int it = 0; it < 30; it++) {
                    const double d = g::det(r);
                    if (std::abs(d) < 1e-15) {
                        break;
                    }
                    // r <- (r + r^-T) / 2; the rows of r^-T are the cross products of r's rows over det
                    g::Mat3 inv{};
                    inv[0] = g::scale(g::cross(r[1], r[2]), 1.0 / d);
                    inv[1] = g::scale(g::cross(r[2], r[0]), 1.0 / d);
                    inv[2] = g::scale(g::cross(r[0], r[1]), 1.0 / d);
                    for (int x = 0; x < 3; x++) {
                        for (int y = 0; y < 3; y++) {
                            r[x][y] = 0.5 * (r[x][y] + inv[x][y]);
                        }
                    }
                }
                if (g::det(r) < 0.5) {
                    continue;
                }
                const double turn = rf::to_degrees(std::acos(std::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0, -1.0, 1.0)));
                if (turn < 120.0) {
                    continue;
                }
                g::Pose t{r, g::sub(mb, g::mat_vec(r, ma))};
                // refine on all LEDs: nearest LED as the partner, the rigid fit again (a few rounds)
                for (int round = 0; round < 4; round++) {
                    std::vector<g::Vec3> pa, pb;
                    for (std::size_t q = 0; q < n; q++) {
                        const g::Vec3 y = t.apply(m.leds[q].p);
                        std::size_t qi = 0;
                        double dmin = 1e9;
                        for (std::size_t w = 0; w < n; w++) {
                            const double dd = g::norm(g::sub(y, m.leds[w].p));
                            if (dd < dmin) {
                                dmin = dd;
                                qi = w;
                            }
                        }
                        pa.push_back(m.leds[q].p);
                        pb.push_back(m.leds[qi].p);
                    }
                    if (const auto f = rigid_fit(pa, pb)) {
                        t = *f;
                    }
                }
                r = t.r;
                const double turn2 =
                    rf::to_degrees(std::acos(std::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0, -1.0, 1.0)));
                double sum = 0;
                std::size_t agree = 0;
                for (std::size_t q = 0; q < n; q++) {
                    const g::Vec3 y = t.apply(m.leds[q].p);
                    double dmin = 1e9;
                    std::size_t qi = 0;
                    for (std::size_t w = 0; w < n; w++) {
                        const double dd = g::norm(g::sub(y, m.leds[w].p));
                        if (dd < dmin) {
                            dmin = dd;
                            qi = w;
                        }
                    }
                    sum += dmin * dmin;
                    agree += g::dot(g::mat_vec(r, m.leds[q].n), m.leds[qi].n) > 0.5 ? 1 : 0;
                }
                const double rms = std::sqrt(sum / static_cast<double>(n));
                if (std::getenv("CT_TWIN_DEBUG") != nullptr) {
                    std::fprintf(stderr, "twin cand %zu %zu %zu: turn %.0f rms %.1f mm agree %zu det %.3f\n", i, j, k, turn2,
                                 rms * 1000.0, agree, g::det(r));
                }
                if (agree * 5 >= n * 4 && rms < 0.010 && turn2 >= 120.0 && (!best || rms < best->second)) {
                    best = std::pair{t, rms};
                }
            }
        }
    }
    }
    return best;
}


} // namespace pibridge::ctrl::track
