// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/align.rs (MIT OR Apache-2.0). See align.h.
#include "pibridge/roommap/align.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace pibridge::roommap::align {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;
namespace sl = cam::slam;
using g::Mat3;

namespace {

constexpr double PI = std::numbers::pi;

double median(std::vector<double> v) {
    std::stable_sort(v.begin(), v.end(), rs::total_less);
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t n = v.size();
    if (n % 2 == 1) {
        return v[n / 2];
    }
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

struct Ray {
    Pose cam_from_slam;
    Vec3 b;
    Vec3 p;
};

using X4 = std::array<double, 4>;
using M4 = std::array<std::array<double, 4>, 4>;

/// transpose(rot_y(yaw)): the same values as computing it per ray (sin/cos of the same argument), once per pose.
Mat3 ryt_of(double yaw) {
    return g::transpose(pnp::rot_y(yaw));
}

std::optional<Vec3> residual_r(const Ray& r, const Mat3& ryt, const X4& x) {
    const Vec3 p_slam = g::mat_vec(ryt, g::sub(r.p, {x[1], x[2], x[3]}));
    const Vec3 xc = r.cam_from_slam.apply(p_slam);
    const double n = g::norm(xc);
    if (std::isnan(n) || n <= 1e-9 || g::dot(xc, r.b) <= 0.0) {
        return std::nullopt;
    }
    return g::cross(r.b, g::scale(xc, 1.0 / n));
}

double angle_deg(const Vec3& r) {
    return rf::to_degrees(std::asin(rf::clamp(g::norm(r), 0.0, 1.0)));
}

std::optional<X4> solve4(const M4& a, const X4& b) {
    std::array<std::array<double, 5>, 4> m{};
    for (std::size_t i = 0; i < 4; i++) {
        for (std::size_t j = 0; j < 4; j++) {
            m[i][j] = a[i][j];
        }
        m[i][4] = b[i];
    }
    for (std::size_t c = 0; c < 4; c++) {
        // (c..4).max_by(total_cmp of |m[i][c]|): the last of equal maxima
        std::size_t p = c;
        for (std::size_t i = c + 1; i < 4; i++) {
            if (rs::total_cmp(std::fabs(m[i][c]), std::fabs(m[p][c])) >= 0) {
                p = i;
            }
        }
        if (std::fabs(m[p][c]) < 1e-300) {
            return std::nullopt;
        }
        std::swap(m[c], m[p]);
        for (std::size_t r = 0; r < 4; r++) {
            if (r != c) {
                const double f = m[r][c] / m[c][c];
                for (std::size_t k = c; k < 5; k++) {
                    m[r][k] -= f * m[c][k];
                }
            }
        }
    }
    return X4{m[0][4] / m[0][0], m[1][4] / m[1][1], m[2][4] / m[2][2], m[3][4] / m[3][3]};
}

std::optional<M4> inv4(const M4& a) {
    M4 out{};
    for (std::size_t j = 0; j < 4; j++) {
        X4 e{0.0, 0.0, 0.0, 0.0};
        e[j] = 1.0;
        const auto c = solve4(a, e);
        if (!c) {
            return std::nullopt;
        }
        for (std::size_t i = 0; i < 4; i++) {
            out[i][j] = (*c)[i];
        }
    }
    return out;
}

struct Lin {
    double cost;
    M4 h;
    X4 g;
    std::size_t n;
};

Lin linearize(const std::vector<Ray>& rays, const X4& x, double k) {
    Lin l{0.0, {}, {0.0, 0.0, 0.0, 0.0}, 0};
    const double steps[4] = {1e-6, 1e-6, 1e-6, 1e-6};
    X4 xps[4], xms[4];
    Mat3 rp[4], rm[4];
    for (std::size_t p = 0; p < 4; p++) {
        xps[p] = x;
        xms[p] = x;
        xps[p][p] += steps[p];
        xms[p][p] -= steps[p];
        rp[p] = ryt_of(xps[p][0]);
        rm[p] = ryt_of(xms[p][0]);
    }
    const Mat3 r_x = ryt_of(x[0]);
    for (const Ray& r : rays) {
        const auto r0 = residual_r(r, r_x, x);
        if (!r0) {
            continue;
        }
        std::array<std::array<double, 4>, 3> jac{};
        bool ok = true;
        for (std::size_t p = 0; p < 4; p++) {
            const auto a = residual_r(r, rp[p], xps[p]);
            const auto b = residual_r(r, rm[p], xms[p]);
            if (a && b) {
                for (std::size_t i = 0; i < 3; i++) {
                    jac[i][p] = ((*a)[i] - (*b)[i]) / (2.0 * steps[p]);
                }
            } else {
                ok = false;
            }
        }
        if (!ok) {
            continue;
        }
        const double e = g::norm(*r0);
        const double w = e <= k ? 1.0 : k / e;
        l.cost += e <= k ? e * e : 2.0 * k * e - k * k;
        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t a = 0; a < 4; a++) {
                l.g[a] += w * jac[i][a] * (*r0)[i];
                for (std::size_t b = 0; b < 4; b++) {
                    l.h[a][b] += w * jac[i][a] * jac[i][b];
                }
            }
        }
        l.n++;
    }
    return l;
}

double huber_cost(const std::vector<Ray>& rays, const X4& x, double k) {
    double s = -0.0;
    const Mat3 ryt = ryt_of(x[0]);
    for (const Ray& r : rays) {
        if (const auto e3 = residual_r(r, ryt, x)) {
            const double e = g::norm(*e3);
            s += e <= k ? e * e : 2.0 * k * e - k * k;
        }
    }
    return s;
}

std::pair<X4, M4> lm(const std::vector<Ray>& rays, const X4& x0, double k) {
    X4 x = x0;
    double lambda = 1e-3;
    Lin lin = linearize(rays, x, k);
    double cost = lin.cost;
    M4 h = lin.h;
    X4 gv = lin.g;
    for (int it = 0; it < 40; it++) {
        bool improved = false;
        for (int t = 0; t < 12; t++) {
            M4 a = h;
            for (std::size_t i = 0; i < 4; i++) {
                a[i][i] += lambda * rf::fmax(h[i][i], 1e-12);
            }
            const auto d = solve4(a, {-gv[0], -gv[1], -gv[2], -gv[3]});
            if (!d) {
                break;
            }
            const X4 xn{x[0] + (*d)[0], x[1] + (*d)[1], x[2] + (*d)[2], x[3] + (*d)[3]};
            const double cn = huber_cost(rays, xn, k);
            if (cn < cost) {
                x = xn;
                lambda = rf::fmax(lambda * 0.3, 1e-9);
                improved = true;
                double ss = -0.0;
                for (double v : *d) {
                    ss += v * v;
                }
                const double step = std::sqrt(ss);
                const Lin l2 = linearize(rays, x, k);
                cost = l2.cost;
                h = l2.h;
                gv = l2.g;
                if (step < 1e-9) {
                    return {x, h};
                }
                break;
            }
            lambda *= 10.0;
        }
        if (!improved) {
            break;
        }
    }
    return {x, h};
}

std::vector<Ray> build_rays(std::span<const FrameObs* const> obs, std::span<const Pose> bfc) {
    std::vector<Ray> v;
    for (const FrameObs* o : obs) {
        for (const auto& c : o->corrs) {
            if (c.cam >= bfc.size()) {
                continue;
            }
            const Pose& b = bfc[c.cam];
            const double n = g::norm(c.bearing);
            if (std::isnan(n) || n <= 0.0) {
                continue;
            }
            v.push_back(Ray{o->slam_from_body.compose(b).inverse(), g::scale(c.bearing, 1.0 / n), c.point});
        }
    }
    return v;
}

} // namespace

double wrap(double a) {
    while (a > PI) {
        a -= 2.0 * PI;
    }
    while (a <= -PI) {
        a += 2.0 * PI;
    }
    return a;
}

std::optional<Alignment> median_alignment(std::span<const Alignment> v) {
    if (v.empty()) {
        return std::nullopt;
    }
    const Alignment& a0 = v.front();
    std::vector<double> ys;
    for (const auto& a : v) {
        ys.push_back(wrap(a.yaw - a0.yaw));
    }
    const double yaw = a0.yaw + median(ys);
    Vec3 t{};
    for (std::size_t i = 0; i < 3; i++) {
        std::vector<double> c;
        for (const auto& a : v) {
            c.push_back(a.t[i]);
        }
        t[i] = median(c);
    }
    return Alignment{wrap(yaw), t};
}

std::pair<double, double> diff(const Alignment& a, const Alignment& b) {
    return {g::norm(g::sub(a.t, b.t)), rf::to_degrees(std::fabs(wrap(a.yaw - b.yaw)))};
}

std::optional<FrameObs> observe(const map::RoomMap& map, const stereo::Rig& rig, std::span<const stereo::Features> feats,
                                std::int64_t ts_ns, const Pose& slam_from_body, const ObserveParams& p) {
    std::vector<std::size_t> all(map.keyframes.size());
    for (std::size_t i = 0; i < all.size(); i++) {
        all[i] = i;
    }
    return observe_kfs(map, all, rig, feats, ts_ns, slam_from_body, p);
}

std::optional<FrameObs> observe_kfs(const map::RoomMap& map, std::span<const std::size_t> kfs, const stereo::Rig& rig,
                                    std::span<const stereo::Features> feats, std::int64_t ts_ns,
                                    const Pose& slam_from_body, const ObserveParams& p) {
    if (kfs.empty()) {
        return std::nullopt;
    }
    std::vector<const orb::Descriptor*> all_q;
    for (const auto& f : feats) {
        for (const auto& d : f.desc) {
            all_q.push_back(&d);
        }
    }
    if (all_q.empty()) {
        return std::nullopt;
    }
    const std::size_t stride = std::max<std::size_t>(all_q.size() / std::max<std::size_t>(p.rank_sample, 1), 1);
    std::vector<const orb::Descriptor*> sample;
    for (std::size_t i = 0; i < all_q.size(); i += stride) {
        sample.push_back(all_q[i]);
    }
    std::vector<std::pair<std::size_t, std::size_t>> scored;
    scored.reserve(kfs.size());
    for (std::size_t i : kfs) {
        const auto& k = map.keyframes[i];
        std::size_t hits = 0;
        for (const orb::Descriptor* q : sample) {
            for (const auto& m : k.points) {
                if (orb::hamming(*q, m.desc) <= p.rank_dist) {
                    hits++;
                    break;
                }
            }
        }
        scored.emplace_back(hits, i);
    }
    std::stable_sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<std::size_t> top;
    for (std::size_t i = 0; i < scored.size() && i < std::max<std::size_t>(p.top_k, 1); i++) {
        if (scored[i].first > 0) {
            top.push_back(scored[i].second);
        }
    }
    if (top.empty()) {
        return std::nullopt;
    }
    std::vector<Vec3> pts;
    std::vector<orb::Descriptor> pdesc;
    std::vector<std::uint32_t> pkf;
    for (std::size_t k : top) {
        for (const auto& m : map.keyframes[k].points) {
            pts.push_back(m.p);
            pdesc.push_back(m.desc);
            pkf.push_back(static_cast<std::uint32_t>(k));
        }
    }
    FrameObs o;
    for (std::size_t c = 0; c < feats.size(); c++) {
        for (const auto& [i, j, d] : orb::match_bf(feats[c].desc, pdesc, p.max_dist, p.ratio)) {
            (void)d;
            o.corrs.push_back(pnp::Corr{c, feats[c].bearings[i], pts[j]});
            o.corr_kf.push_back(pkf[j]);
        }
    }
    if (o.corrs.size() < p.min_inliers) {
        return std::nullopt;
    }
    const auto bfc = rig.body_from_cams();
    const Mat3 tilt = map::tilt_of(sl::quat_of(slam_from_body.r));
    auto res = pnp::ransac(o.corrs, bfc, tilt, p.pnp);
    if (!res) {
        return std::nullopt;
    }
    if (res->inliers.size() < p.min_inliers ||
        static_cast<double>(res->inliers.size()) < p.min_inlier_frac * static_cast<double>(o.corrs.size())) {
        return std::nullopt;
    }
    o.room_from_body = res->pose.to_pose(tilt);
    map::Reloc reloc;
    reloc.room_from_body = o.room_from_body;
    reloc.inliers = res->inliers.size();
    reloc.corrs = o.corrs.size();
    reloc.rms_deg = res->rms_deg;
    const auto [yaw, t] = map::room_from_slam(reloc, slam_from_body);
    o.ts_ns = ts_ns;
    o.slam_from_body = slam_from_body;
    o.inliers = std::move(res->inliers);
    o.hyp = Alignment{yaw, t};
    o.rms_deg = res->rms_deg;
    return o;
}

std::optional<JointFit> joint_fit(std::span<const FrameObs* const> obs, std::span<const Pose> bfc, const JointParams& p) {
    std::vector<Alignment> hyps;
    for (const FrameObs* o : obs) {
        hyps.push_back(o->hyp);
    }
    const auto start = median_alignment(hyps);
    if (!start) {
        return std::nullopt;
    }
    std::vector<const FrameObs*> keep;
    for (const FrameObs* o : obs) {
        const auto [dm, dd] = diff(o->hyp, *start);
        if (dm <= p.hyp_m && dd <= p.hyp_deg) {
            keep.push_back(o);
        }
    }
    if (keep.empty()) {
        return std::nullopt;
    }
    const std::vector<Ray> all = build_rays(keep, bfc);
    X4 x{start->yaw, start->t[0], start->t[1], start->t[2]};
    const double gate = std::sin(rf::to_radians(p.gate_deg));
    const double k = std::sin(rf::to_radians(p.huber_deg));
    std::vector<const Ray*> used;
    M4 h{};
    for (std::size_t round = 0; round < std::max<std::size_t>(p.rounds, 1); round++) {
        used.clear();
        const Mat3 ryt = ryt_of(x[0]);
        for (const Ray& r : all) {
            const auto e = residual_r(r, ryt, x);
            if (e && g::norm(*e) <= gate) {
                used.push_back(&r);
            }
        }
        if (used.size() < 8) {
            return std::nullopt;
        }
        std::vector<Ray> sub;
        sub.reserve(used.size());
        for (const Ray* r : used) {
            sub.push_back(*r);
        }
        const auto [xn, hn] = lm(sub, x, k);
        x = xn;
        h = hn;
    }
    std::vector<double> res;
    const Mat3 ryt_final = ryt_of(x[0]);
    for (const Ray* r : used) {
        if (const auto e = residual_r(*r, ryt_final, x)) {
            res.push_back(angle_deg(*e));
        }
    }
    const std::size_t n = res.size();
    double ss = -0.0;
    for (double a : res) {
        ss += a * a;
    }
    const double rms = std::sqrt(ss / static_cast<double>(std::max<std::size_t>(n, 1)));
    const double rr = rf::to_radians(rms);
    const double s2 = rr * rr / 2.0;
    double sigma_deg = std::numeric_limits<double>::quiet_NaN();
    double sigma_m = std::numeric_limits<double>::quiet_NaN();
    if (const auto c = inv4(h)) {
        sigma_deg = rf::to_degrees(std::sqrt((*c)[0][0] * s2));
        sigma_m = std::sqrt(((*c)[1][1] + (*c)[2][2] + (*c)[3][3]) * s2);
    }
    JointFit f;
    f.align = Alignment{wrap(x[0]), {x[1], x[2], x[3]}};
    f.frames = keep.size();
    f.inliers = n;
    f.rms_deg = rms;
    f.sigma_deg = sigma_deg;
    f.sigma_m = sigma_m;
    const double nk = static_cast<double>(keep.size());
    double sm = -0.0, sd = -0.0;
    for (const FrameObs* o : keep) {
        const double dm = diff(o->hyp, f.align).first;
        sm += dm * dm;
        const double dd = rf::to_degrees(wrap(o->hyp.yaw - f.align.yaw));
        sd += dd * dd;
    }
    f.hyp_rms_m = std::sqrt(sm / nk);
    f.hyp_rms_deg = std::sqrt(sd / nk);
    std::int64_t lo = keep[0]->ts_ns, hi = keep[0]->ts_ns;
    for (const FrameObs* o : keep) {
        lo = std::min(lo, o->ts_ns);
        hi = std::max(hi, o->ts_ns);
    }
    f.span_ns = {lo, hi};
    return f;
}

std::optional<std::pair<std::size_t, JointFit>> first_fix(std::span<const FrameObs> obs, std::span<const Pose> bfc,
                                                          const JointParams& jp, const AcceptParams& ap) {
    for (std::size_t i = 0; i < obs.size(); i++) {
        const std::int64_t t1 = obs[i].ts_ns;
        std::vector<const FrameObs*> win;
        for (std::size_t j = 0; j <= i; j++) {
            if (t1 - obs[j].ts_ns <= ap.window_ns) {
                win.push_back(&obs[j]);
            }
        }
        if (win.size() < ap.min_frames) {
            continue;
        }
        if (auto f = joint_fit(win, bfc, jp)) {
            if (f->inliers >= ap.min_inliers &&
                static_cast<double>(f->frames) >= ap.min_agree * static_cast<double>(win.size())) {
                return std::pair{i, *f};
            }
        }
    }
    return std::nullopt;
}

std::vector<JointFit> window_fits(std::span<const FrameObs> obs, std::span<const Pose> bfc, const JointParams& jp,
                                  std::int64_t window_ns, std::size_t min_frames) {
    std::vector<JointFit> out;
    if (obs.empty()) {
        return out;
    }
    const std::int64_t t0 = obs.front().ts_ns;
    const std::int64_t t_end = obs.back().ts_ns;
    std::int64_t a = t0;
    while (a <= t_end) {
        std::vector<const FrameObs*> win;
        for (const auto& o : obs) {
            if (o.ts_ns >= a && o.ts_ns < a + window_ns) {
                win.push_back(&o);
            }
        }
        if (win.size() >= min_frames) {
            if (auto f = joint_fit(win, bfc, jp)) {
                out.push_back(*f);
            }
        }
        a += window_ns;
    }
    return out;
}

} // namespace pibridge::roommap::align
