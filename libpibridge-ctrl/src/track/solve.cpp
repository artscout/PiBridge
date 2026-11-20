// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/solve.rs (MIT OR Apache-2.0).
// Evaluation order, sort stability (Rust sort_by = std::stable_sort) and the tie rule of Rust's `max_by` (the last
// maximum wins) are kept, so the poses are bit-identical to the Rust ones.
#include "pibridge/ctrl/track/solve.h"

#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/p3p.h"
#include "pibridge/ctrl/track/quads.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

CamObs CamObs::make(const Cam& cam, std::vector<std::pair<double, double>> px) {
    CamObs o;
    o.bearing.reserve(px.size());
    for (const auto& [u, v] : px) {
        o.bearing.push_back(cam.intr.unproject(u, v));
    }
    o.px = std::move(px);
    return o;
}

std::optional<g::Vec3> facing(const g::Pose& cam_from_model, const Led& led, double cos_lim) {
    const g::Vec3 pc = cam_from_model.apply(led.p);
    if (pc[2] <= 0.0) {
        return std::nullopt;
    }
    const g::Vec3 d = g::normalize(pc);
    const g::Vec3 n = g::mat_vec(cam_from_model.r, led.n);
    if (g::dot(n, g::scale(d, -1.0)) >= cos_lim) {
        return d;
    }
    return std::nullopt;
}

namespace {

struct Cand3 {
    double v;
    std::size_t li;
    std::size_t bi;
};

/// Greedy one-to-one angular matching of facing LEDs to unused blobs.
std::pair<Matches, double> match_angular(const g::Pose& pose, const LedModel& model, const CamObs& obs,
                                         const std::vector<bool>& used, double cos_face, double cos_tol) {
    std::vector<Cand3> cand;
    for (std::size_t li = 0; li < model.leds.size(); li++) {
        const auto d = facing(pose, model.leds[li], cos_face);
        if (!d) {
            continue;
        }
        for (std::size_t bi = 0; bi < obs.bearing.size(); bi++) {
            if (used[bi]) {
                continue;
            }
            if (const auto& b = obs.bearing[bi]) {
                const double c = g::dot(*d, *b);
                if (c >= cos_tol) {
                    cand.push_back({c, li, bi});
                }
            }
        }
    }
    std::stable_sort(cand.begin(), cand.end(), [](const Cand3& a, const Cand3& b) { return a.v > b.v; });
    bool lu[64] = {};
    std::vector<bool> bu(obs.px.size(), false);
    Matches m;
    double err = 0.0;
    for (const Cand3& c : cand) {
        if (!lu[c.li] && !bu[c.bi]) {
            lu[c.li] = true;
            bu[c.bi] = true;
            m.emplace_back(c.li, c.bi);
            err += 1.0 - c.v;
        }
    }
    return {std::move(m), err};
}

g::Pose perturb(const g::Pose& p, const std::array<double, 6>& d) {
    return g::Pose{g::mat_mul(g::rodrigues({d[0], d[1], d[2]}), p.r), g::add(p.t, {d[3], d[4], d[5]})};
}

using M6 = std::array<std::array<double, 6>, 6>;
using V6 = std::array<double, 6>;

std::optional<V6> solve6(M6& a, V6& b) {
    for (std::size_t c = 0; c < 6; c++) {
        // Rust `(c..6).max_by(..)`: the last of equal maxima
        std::size_t piv = c;
        for (std::size_t i = c + 1; i < 6; i++) {
            if (!(std::fabs(a[i][c]) < std::fabs(a[piv][c]))) {
                piv = i;
            }
        }
        if (std::fabs(a[piv][c]) < 1e-18) {
            return std::nullopt;
        }
        std::swap(a[c], a[piv]);
        std::swap(b[c], b[piv]);
        for (std::size_t r = c + 1; r < 6; r++) {
            const double f = a[r][c] / a[c][c];
            for (std::size_t k = c; k < 6; k++) {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    V6 x{};
    for (std::size_t c = 6; c-- > 0;) {
        double s = b[c];
        for (std::size_t k = c + 1; k < 6; k++) {
            s -= a[c][k] * x[k];
        }
        x[c] = s / a[c][c];
    }
    return x;
}

/// Rust `iter().map(|x| x * x).sum::<f64>()` (the sum starts at -0.0, the float neutral element).
double sum_sq(const double* r, std::size_t n) {
    double s = -0.0;
    for (std::size_t i = 0; i < n; i++) {
        s += r[i] * r[i];
    }
    return s;
}

} // namespace

Matches match_pixels(const g::Pose& pose, const LedModel& model, const Cam& cam, const CamObs& obs,
                     const std::vector<bool>& used, double cos_face, double tol_px) {
    std::vector<Cand3> cand;
    for (std::size_t li = 0; li < model.leds.size(); li++) {
        const Led& led = model.leds[li];
        if (!facing(pose, led, cos_face)) {
            continue;
        }
        const auto uv = cam.intr.project(pose.apply(led.p));
        if (!uv) {
            continue;
        }
        const auto [u, v] = *uv;
        for (std::size_t bi = 0; bi < obs.px.size(); bi++) {
            if (used[bi]) {
                continue;
            }
            const auto [bu, bv] = obs.px[bi];
            const double du = u - bu;
            const double dv = v - bv;
            const double d = std::sqrt(du * du + dv * dv);
            if (d <= tol_px) {
                cand.push_back({d, li, bi});
            }
        }
    }
    std::stable_sort(cand.begin(), cand.end(), [](const Cand3& a, const Cand3& b) { return a.v < b.v; });
    bool lu[64] = {};
    std::vector<bool> bu(obs.px.size(), false);
    Matches m;
    for (const Cand3& c : cand) {
        if (!lu[c.li] && !bu[c.bi]) {
            lu[c.li] = true;
            bu[c.bi] = true;
            m.emplace_back(c.li, c.bi);
        }
    }
    return m;
}

std::pair<g::Pose, double> refine(const g::Pose& pose, std::span<const Obs> obs, bool joint, std::size_t iters) {
    return refine_with(pose, obs, joint, iters, std::nullopt);
}

g::Vec3 rotvec(const g::Mat3& r) {
    const double c = rf::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0, -1.0, 1.0);
    const double ang = std::acos(c);
    if (ang < 1e-12) {
        return {0.0, 0.0, 0.0};
    }
    const double k = ang / (2.0 * std::sin(ang));
    return {k * (r[2][1] - r[1][2]), k * (r[0][2] - r[2][0]), k * (r[1][0] - r[0][1])};
}

std::pair<g::Pose, double> refine_with(const g::Pose& pose, std::span<const Obs> obs, bool joint, std::size_t iters,
                                       const std::optional<std::pair<g::Mat3, double>>& rot_prior) {
    std::optional<std::pair<g::Mat3, double>> rp_t;
    if (rot_prior) {
        rp_t = std::pair{g::transpose(rot_prior->first), rot_prior->second};
    }
    const auto res = [&](const g::Pose& p, std::vector<double>& out) -> bool {
        out.clear();
        if (rp_t) {
            const g::Vec3 d = rotvec(g::mat_mul(p.r, rp_t->first));
            for (const double x : d) {
                out.push_back(x * rp_t->second);
            }
        }
        for (const Obs& o : obs) {
            const g::Vec3 x = p.apply(o.p);
            const g::Vec3 xc = joint ? o.cam->cam_from_body.apply(x) : x;
            const auto uv = o.cam->intr.project(xc);
            if (!uv) {
                return false;
            }
            out.push_back(uv->first - o.px.first);
            out.push_back(uv->second - o.px.second);
        }
        return true;
    };
    g::Pose p = pose;
    std::vector<double> r0;
    if (!res(p, r0)) {
        return {p, std::numeric_limits<double>::infinity()};
    }
    double c0 = sum_sq(r0.data(), r0.size());
    double lambda = 1e-3;
    std::vector<double> r1;
    for (std::size_t it = 0; it < iters; it++) {
        // numeric Jacobian
        std::vector<V6> jac(r0.size(), V6{});
        bool ok = true;
        for (std::size_t k = 0; k < 6; k++) {
            V6 d{};
            const double h = k < 3 ? 1e-6 : 1e-7;
            d[k] = h;
            if (!res(perturb(p, d), r1)) {
                ok = false;
                break;
            }
            for (std::size_t i = 0; i < jac.size(); i++) {
                jac[i][k] = (r1[i] - r0[i]) / h;
            }
        }
        if (!ok) {
            break;
        }
        M6 jtj{};
        V6 jtr{};
        for (std::size_t i = 0; i < jac.size(); i++) {
            const V6& row = jac[i];
            for (std::size_t a = 0; a < 6; a++) {
                jtr[a] -= row[a] * r0[i];
                for (std::size_t b = 0; b < 6; b++) {
                    jtj[a][b] += row[a] * row[b];
                }
            }
        }
        bool improved = false;
        for (int t = 0; t < 8; t++) {
            M6 a = jtj;
            for (std::size_t d = 0; d < 6; d++) {
                a[d][d] += lambda * (jtj[d][d] + 1e-9);
            }
            V6 b = jtr;
            const auto dx = solve6(a, b);
            if (!dx) {
                break;
            }
            const g::Pose pn = perturb(p, *dx);
            if (res(pn, r1)) {
                const double c1 = sum_sq(r1.data(), r1.size());
                if (c1 < c0) {
                    p = pn;
                    std::swap(r0, r1);
                    const double rel = (c0 - c1) / rf::fmax(c0, 1e-30);
                    c0 = c1;
                    lambda = rf::fmax(lambda * 0.3, 1e-9);
                    improved = rel > 1e-10;
                    break;
                }
            }
            lambda *= 10.0;
        }
        if (!improved) {
            break;
        }
    }
    const std::size_t npx = rp_t ? (r0.size() >= 3 ? r0.size() - 3 : 0) : r0.size();
    const double prior_cost = rp_t ? sum_sq(r0.data(), std::min<std::size_t>(3, r0.size())) : 0.0;
    const double rms =
        std::sqrt(rf::fmax(c0 - prior_cost, 0.0) / static_cast<double>(std::max<std::size_t>(npx / 2, 1)));
    // re-orthonormalise
    p.r = quat_to_mat(g::quat_from_mat(p.r));
    return {p, rms};
}

g::Mat3 quat_to_mat(const std::array<double, 4>& q) {
    // q = [x, y, z, w] (as geom::quat_from_mat)
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const double x = q[0] / n;
    const double y = q[1] / n;
    const double z = q[2] / n;
    const double w = q[3] / n;
    return {{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)},
             {2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)},
             {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)}}};
}

std::optional<CamSolve> polish(const g::Pose& pose, const LedModel& model, const Cam& cam, std::size_t cam_idx,
                               const CamObs& obs, const std::vector<bool>& used, const Params& prm,
                               const Matches& matches) {
    const double cos_face = std::cos(rf::to_radians(prm.facing_deg));
    const auto mk = [&](const Matches& m) {
        std::vector<Obs> o;
        o.reserve(m.size());
        for (const auto& [l, b] : m) {
            o.push_back(Obs{&cam, model.leds[l].p, obs.px[b]});
        }
        return o;
    };
    const auto [p1, r1] = refine(pose, mk(matches), false, 15);
    (void)r1;
    const Matches m2 = match_pixels(p1, model, cam, obs, used, cos_face, prm.tol_px * 2.0);
    if (m2.size() < 4) {
        return std::nullopt;
    }
    const auto [p2, r2] = refine(p1, mk(m2), false, 20);
    (void)r2;
    Matches m3 = match_pixels(p2, model, cam, obs, used, cos_face, prm.tol_px);
    if (m3.size() < 4) {
        return std::nullopt;
    }
    const auto [p3, rms] = refine(p2, mk(m3), false, 20);
    return CamSolve{cam_idx, p3, std::move(m3), rms};
}

std::optional<CamSolve> acquire(const LedModel& model, std::span<const std::array<std::size_t, 3>> triples,
                                const Cam& cam, std::size_t cam_idx, const CamObs& obs, const std::vector<bool>& used,
                                const Params& prm) {
    const std::size_t n = obs.px.size();
    std::vector<std::size_t> free;
    for (std::size_t i = 0; i < n; i++) {
        if (!used[i] && obs.bearing[i]) {
            free.push_back(i);
        }
    }
    if (free.size() < std::max<std::size_t>(prm.min_inliers, 3)) {
        return std::nullopt;
    }
    const double cos_face = std::cos(rf::to_radians(prm.facing_deg));
    const double cos_tol = std::cos(prm.tol_rad);
    // blob triples: anchor + 2 of its k nearest free neighbours
    std::vector<std::array<std::size_t, 3>> btri;
    for (const std::size_t a : free) {
        std::vector<std::pair<double, std::size_t>> nb;
        for (const std::size_t b : free) {
            if (b == a) {
                continue;
            }
            const double dx = obs.px[a].first - obs.px[b].first;
            const double dy = obs.px[a].second - obs.px[b].second;
            nb.emplace_back(dx * dx + dy * dy, b);
        }
        std::stable_sort(nb.begin(), nb.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
        if (nb.size() > prm.knn) {
            nb.resize(prm.knn);
        }
        for (std::size_t i = 0; i < nb.size(); i++) {
            for (std::size_t j = i + 1; j < nb.size(); j++) {
                std::array<std::size_t, 3> t{a, nb[i].second, nb[j].second};
                std::sort(t.begin(), t.end());
                btri.push_back(t);
            }
        }
    }
    std::sort(btri.begin(), btri.end());
    btri.erase(std::unique(btri.begin(), btri.end()), btri.end());
    struct Best {
        std::size_t n;
        double err;
        g::Pose pose;
        Matches m;
    };
    std::optional<Best> best;
    for (const auto& bt : btri) {
        const std::array<g::Vec3, 3> y{*obs.bearing[bt[0]], *obs.bearing[bt[1]], *obs.bearing[bt[2]]};
        bool stop = false;
        for (const auto& lt : triples) {
            const std::array<g::Vec3, 3> x{model.leds[lt[0]].p, model.leds[lt[1]].p, model.leds[lt[2]].p};
            for (const g::Pose& pose : p3p(y, x)) {
                const double dist = g::norm(pose.t);
                if (dist < prm.min_dist || dist > prm.max_dist) {
                    continue;
                }
                bool hidden = false;
                for (const std::size_t l : lt) {
                    if (!facing(pose, model.leds[l], cos_face)) {
                        hidden = true;
                        break;
                    }
                }
                if (hidden) {
                    continue;
                }
                auto [m, err] = match_angular(pose, model, obs, used, cos_face, cos_tol);
                const bool better = !best || m.size() > best->n || (m.size() == best->n && err < best->err);
                if (better && m.size() >= 3) {
                    const std::size_t cnt = m.size();
                    best = Best{cnt, err, pose, std::move(m)};
                    if (cnt >= prm.good_enough) {
                        stop = true;
                        break;
                    }
                }
            }
            if (stop) {
                break;
            }
        }
        if (stop) {
            break;
        }
    }
    if (!best || best->n < prm.min_inliers) {
        return std::nullopt;
    }
    auto s = polish(best->pose, model, cam, cam_idx, obs, used, prm, best->m);
    if (!s || !(s->matches.size() >= prm.min_inliers && s->rms_px <= prm.tol_px)) {
        return std::nullopt;
    }
    return s;
}

std::optional<CamSolve> acquire_quads(const LedModel& model, const QuadIndex& index, const Cam& cam,
                                      std::size_t cam_idx, const CamObs& obs, const std::vector<bool>& used,
                                      const Params& prm, const RigView* all, const OrientPrior* orient) {
    const std::size_t n = obs.px.size();
    std::vector<std::size_t> free;
    for (std::size_t i = 0; i < n; i++) {
        if (!used[i] && obs.bearing[i]) {
            free.push_back(i);
        }
    }
    if (free.size() < std::max<std::size_t>(prm.min_inliers, 4)) {
        return std::nullopt;
    }
    const double cos_face = std::cos(rf::to_radians(prm.facing_deg));
    const double cos_tol = std::cos(prm.tol_rad);
    const double cos_fourth = std::cos(3.0 * prm.tol_rad);
    // blob quads: anchor + 3 of its k nearest free neighbours, each once
    std::vector<std::array<std::size_t, 4>> quads;
    for (const std::size_t a : free) {
        std::vector<std::pair<double, std::size_t>> nb;
        for (const std::size_t b : free) {
            if (b == a) {
                continue;
            }
            const double dx = obs.px[a].first - obs.px[b].first;
            const double dy = obs.px[a].second - obs.px[b].second;
            nb.emplace_back(dx * dx + dy * dy, b);
        }
        std::sort(nb.begin(), nb.end());
        static const std::size_t knn_q = [] {
            const char* e = std::getenv("CT_QUAD_KNN");
            return e != nullptr ? static_cast<std::size_t>(std::atoi(e)) : std::size_t{5};
        }();
        if (nb.size() > knn_q) {
            nb.resize(knn_q);
        }
        for (std::size_t i = 0; i < nb.size(); i++) {
            for (std::size_t j = i + 1; j < nb.size(); j++) {
                for (std::size_t k = j + 1; k < nb.size(); k++) {
                    std::array<std::size_t, 4> q{a, nb[i].second, nb[j].second, nb[k].second};
                    std::sort(q.begin(), q.end());
                    quads.push_back(q);
                }
            }
        }
    }
    std::sort(quads.begin(), quads.end());
    quads.erase(std::unique(quads.begin(), quads.end()), quads.end());
    struct Best {
        std::size_t n;
        std::size_t total;
        double err;
        g::Pose pose;
        Matches m;
    };
    std::optional<Best> best;
    // the IMU's orientation in this camera: up vectors and the rotation to compare a hypothesis with
    std::optional<std::pair<g::Vec3, g::Vec3>> up_cam;
    std::optional<g::Mat3> r_cam, r_cam_twin;
    double cos_tilt = -1.0;
    if (orient != nullptr) {
        if (orient->up) {
            up_cam = std::pair{orient->up->first, g::mat_vec(cam.cam_from_body.r, orient->up->second)};
            cos_tilt = std::cos(rf::to_radians(orient->tilt_deg));
        }
        if (orient->r) {
            r_cam = g::mat_mul(cam.cam_from_body.r, *orient->r);
        }
        if (orient->r_twin) {
            r_cam_twin = g::mat_mul(cam.cam_from_body.r, *orient->r_twin);
        }
    }
    std::vector<QuadIndex::Hit> hits;
    std::size_t dbg_q = 0, dbg_hits = 0, dbg_poses = 0, dbg_scored = 0, dbg_dist = 0, dbg_fourth = 0, dbg_tilt = 0, dbg_rot = 0, dbg_face = 0, dbg_few = 0;
    for (const auto& q : quads) {
        dbg_q++;
        const std::array<g::Vec3, 4> d{*obs.bearing[q[0]], *obs.bearing[q[1]], *obs.bearing[q[2]], *obs.bearing[q[3]]};
        // shape tolerance: the angular noise over the base length (the longest pair), at least one cell
        double base = 0;
        for (std::size_t s = 0; s < 4; s++) {
            for (std::size_t t = s + 1; t < 4; t++) {
                base = std::max(base, std::acos(std::clamp(g::dot(d[s], d[t]), -1.0, 1.0)));
            }
        }
        // (tol_rad is ~3 px: 1.5 px of noise on each end of the base moves a shape coordinate by ~tol_rad / base)
        static const double tol_k = [] {
            const char* e = std::getenv("CT_QUAD_TOL");
            return e != nullptr ? std::atof(e) : 1.25;
        }();
        static const double tol_min = [] {
            const char* e = std::getenv("CT_QUAD_TOL_MIN");
            return e != nullptr ? std::atof(e) : 0.03;
        }();
        static const double tol_max = [] {
            const char* e = std::getenv("CT_QUAD_TOL_MAX");
            return e != nullptr ? std::atof(e) : 0.15;
        }();
        const double tol = std::clamp(tol_k * prm.tol_rad / std::max(base, 1e-6), tol_min, tol_max);
        hits.clear();
        index.lookup(d, tol, hits);
        dbg_hits += hits.size();
        bool stop = false;
        for (const auto& h : hits) {
            const std::array<g::Vec3, 3> y{d[h.slot[0]], d[h.slot[1]], d[h.slot[2]]};
            const std::array<g::Vec3, 3> x{model.leds[h.led[0]].p, model.leds[h.led[1]].p, model.leds[h.led[2]].p};
            for (const g::Pose& pose : p3p(y, x)) {
                dbg_poses++;
                const double dist = g::norm(pose.t);
                if (dist < prm.min_dist || dist > prm.max_dist) {
                    dbg_dist++;
                    continue;
                }
                // the fourth pair must agree, and all four LEDs face the camera
                const g::Vec3 p4 = g::normalize(pose.apply(model.leds[h.led[3]].p));
                if (g::dot(p4, d[h.slot[3]]) < cos_fourth) {
                    dbg_fourth++;
                    continue;
                }
                if (up_cam && g::dot(g::mat_vec(pose.r, up_cam->first), up_cam->second) < cos_tilt) {
                    dbg_tilt++;
                    continue; // tilted against the accelerometer
                }
                const auto off_deg = [&](const g::Mat3& rc) {
                    return rf::to_degrees(g::norm(rotvec(g::mat_mul(pose.r, g::transpose(rc)))));
                };
                if (r_cam && off_deg(*r_cam) > orient->rot_deg && !(r_cam_twin && off_deg(*r_cam_twin) <= orient->rot_deg)) {
                    dbg_rot++;
                    continue; // turned against the gyro
                }
                bool hidden = false;
                for (const std::uint8_t l : h.led) {
                    if (!facing(pose, model.leds[l], cos_face)) {
                        hidden = true;
                        break;
                    }
                }
                if (hidden) {
                    dbg_face++;
                    continue;
                }
                dbg_scored++;
                auto [m, err] = match_angular(pose, model, obs, used, cos_face, cos_tol);
                if (m.size() < 4) {
                    dbg_few++;
                    continue;
                }
                std::size_t total = m.size();
                if (all != nullptr) {
                    const g::Pose body_from_model = cam.cam_from_body.inverse().compose(pose);
                    for (std::size_t cj = 0; cj < all->cams.size(); cj++) {
                        if (cj != cam_idx) {
                            total += match_angular(all->cams[cj].cam_from_body.compose(body_from_model), model,
                                                   all->obs[cj], all->used[cj], cos_face, cos_tol)
                                         .first.size();
                        }
                    }
                }
                const bool better = !best || total > best->total || (total == best->total && err < best->err);
                if (better) {
                    const std::size_t cnt = total;
                    best = Best{m.size(), total, err, pose, std::move(m)};
                    static const std::size_t enough = [] {
                        const char* e = std::getenv("CT_QUAD_ENOUGH");
                        return e != nullptr ? static_cast<std::size_t>(std::atoi(e)) : std::size_t{12};
                    }();
                    if (cnt >= enough) {
                        stop = true;
                        break;
                    }
                }
            }
            if (stop) {
                break;
            }
        }
        if (stop) {
            break;
        }
    }
    if (std::getenv("CT_QUAD_DEBUG") != nullptr) {
        std::fprintf(stderr,
                     "quads %s cam %zu: blobs %zu quads %zu hits %zu poses %zu | dist %zu fourth %zu tilt %zu rot %zu "
                     "face %zu few %zu scored %zu | prior up %d r %d | best %zu total %zu\n",
                     model.hand.c_str(), cam_idx, free.size(), quads.size(), dbg_hits, dbg_poses, dbg_dist, dbg_fourth,
                     dbg_tilt, dbg_rot, dbg_face, dbg_few, dbg_scored, up_cam ? 1 : 0, r_cam ? 1 : 0, best ? best->n : 0,
                     best ? best->total : 0);
    }
    if (!best || best->n < prm.min_inliers) {
        return std::nullopt;
    }
    auto s = polish(best->pose, model, cam, cam_idx, obs, used, prm, best->m);
    if (!s || !(s->matches.size() >= prm.min_inliers && s->rms_px <= prm.tol_px)) {
        return std::nullopt;
    }
    return s;
}

std::size_t JointSolve::total() const {
    std::size_t t = 0;
    for (const auto& m : matches) {
        t += m.size();
    }
    return t;
}

std::size_t JointSolve::cams() const {
    std::size_t c = 0;
    for (const auto& m : matches) {
        c += m.empty() ? 0 : 1;
    }
    return c;
}

std::optional<JointSolve> joint_from_prior(const g::Pose& body_from_model, const LedModel& model,
                                           std::span<const Cam> cams, std::span<const CamObs> obs,
                                           std::span<const std::vector<bool>> used, double tol_px,
                                           double facing_deg) {
    const double cos_face = std::cos(rf::to_radians(facing_deg));
    std::vector<Matches> per_cam;
    per_cam.reserve(cams.size());
    for (std::size_t ci = 0; ci < cams.size(); ci++) {
        const g::Pose cfm = cams[ci].cam_from_body.compose(body_from_model);
        per_cam.push_back(match_pixels(cfm, model, cams[ci], obs[ci], used[ci], cos_face, tol_px));
    }
    std::size_t total = 0;
    for (const auto& m : per_cam) {
        total += m.size();
    }
    if (total < 4) {
        return std::nullopt;
    }
    std::vector<Obs> o;
    for (std::size_t ci = 0; ci < per_cam.size(); ci++) {
        for (const auto& [l, b] : per_cam[ci]) {
            o.push_back(Obs{&cams[ci], model.leds[l].p, obs[ci].px[b]});
        }
    }
    const auto [pose, rms] = refine(body_from_model, o, true, 25);
    return JointSolve{pose, std::move(per_cam), rms};
}

} // namespace pibridge::ctrl::track
