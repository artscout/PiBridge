// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/loopclose.rs (MIT OR Apache-2.0). See loopclose.h.
#include "pibridge/roommap/loopclose.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <thread>

namespace pibridge::roommap::loopclose {

namespace g = cam::geom;
namespace rf = cam::rfmt;
using align::wrap;
using g::Mat3;

namespace {

Mat3 ryt(double y) {
    return g::transpose(pnp::rot_y(y));
}

Mat3 dryt(double y) {
    const double s = std::sin(y), c = std::cos(y);
    return {{{-s, 0.0, -c}, {0.0, 0.0, 0.0}, {c, 0.0, -s}}};
}

double chi(const Edge& e, std::span<const Node> x) {
    const auto [ry, rt] = edge_residual(e, x);
    return std::sqrt(ry * ry * e.wy + g::dot(rt, rt) * e.wt);
}

std::optional<std::vector<double>> cholesky_solve(std::vector<double> a, std::size_t n, std::vector<double> b) {
    for (std::size_t j = 0; j < n; j++) {
        double d = a[j * n + j];
        for (std::size_t k = 0; k < j; k++) {
            d -= a[j * n + k] * a[j * n + k];
        }
        if (d <= 0.0) {
            return std::nullopt;
        }
        d = std::sqrt(d);
        a[j * n + j] = d;
        for (std::size_t i = j + 1; i < n; i++) {
            double s = a[i * n + j];
            for (std::size_t k = 0; k < j; k++) {
                s -= a[i * n + k] * a[j * n + k];
            }
            a[i * n + j] = s / d;
        }
    }
    for (std::size_t i = 0; i < n; i++) {
        double s = b[i];
        for (std::size_t k = 0; k < i; k++) {
            s -= a[i * n + k] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    for (std::size_t i = n; i-- > 0;) {
        double s = b[i];
        for (std::size_t k = i + 1; k < n; k++) {
            s -= a[k * n + i] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    return b;
}

Edge odo_edge(std::size_t a, std::size_t b, const Pose& pa, const Pose& pb, const LoopParams& p) {
    const double ya = map::yaw_of_mat(pa.r), yb = map::yaw_of_mat(pb.r);
    const Vec3 dt = g::mat_vec(ryt(ya), g::sub(pb.t, pa.t));
    const double dist = g::norm(dt);
    const double st = p.odo_min_m + p.odo_frac * dist;
    const double sy = rf::to_radians(p.odo_min_deg + p.odo_deg_per_m * dist);
    return Edge{a, b, wrap(yb - ya), dt, 1.0 / (st * st), 1.0 / (sy * sy), false};
}

} // namespace

std::pair<double, Vec3> edge_residual(const Edge& e, std::span<const Node> x) {
    const auto& [ya, ta] = x[e.a];
    const auto& [yb, tb] = x[e.b];
    const double ry = wrap(yb - ya - e.dyaw);
    const Vec3 rt = g::sub(g::mat_vec(ryt(ya), g::sub(tb, ta)), e.dt);
    return {ry, rt};
}

void solve_graph(std::span<Node> x, std::span<const Edge> edges, std::size_t iters, double huber) {
    const std::size_t n = x.size();
    if (n < 2) {
        return;
    }
    const std::size_t dim = 4 * (n - 1);
    struct Row {
        double r;
        double wr;
        std::vector<std::pair<std::size_t, double>> cols;
    };
    for (std::size_t it = 0; it < iters; it++) {
        std::vector<double> h(dim * dim, 0.0);
        std::vector<double> gv(dim, 0.0);
        for (const Edge& e : edges) {
            const auto [ry, rt] = edge_residual(e, x);
            double w = 1.0;
            if (e.is_loop) {
                const double c = chi(e, x);
                w = c > huber ? huber / c : 1.0;
            }
            const auto& [ya, ta] = x[e.a];
            const Vec3 tb = x[e.b].second;
            const Mat3 rta = ryt(ya);
            const Vec3 d_rt_dya = g::mat_vec(dryt(ya), g::sub(tb, ta));
            std::vector<Row> rows;
            rows.reserve(4);
            rows.push_back(Row{ry, e.wy * w, {{e.b * 4, 1.0}, {e.a * 4, -1.0}}});
            for (std::size_t i = 0; i < 3; i++) {
                std::vector<std::pair<std::size_t, double>> cols = {{e.a * 4, d_rt_dya[i]}};
                for (std::size_t k = 0; k < 3; k++) {
                    cols.emplace_back(e.b * 4 + 1 + k, rta[i][k]);
                    cols.emplace_back(e.a * 4 + 1 + k, -rta[i][k]);
                }
                rows.push_back(Row{rt[i], e.wt * w, std::move(cols)});
            }
            for (const Row& row : rows) {
                for (const auto& [ci, vi] : row.cols) {
                    if (ci < 4) {
                        continue;
                    }
                    const std::size_t i = ci - 4;
                    gv[i] += row.wr * vi * row.r;
                    for (const auto& [cj, vj] : row.cols) {
                        if (cj < 4) {
                            continue;
                        }
                        h[i * dim + cj - 4] += row.wr * vi * vj;
                    }
                }
            }
        }
        for (std::size_t i = 0; i < dim; i++) {
            h[i * dim + i] += 1e-9;
        }
        std::vector<double> neg(dim);
        for (std::size_t i = 0; i < dim; i++) {
            neg[i] = -gv[i];
        }
        const auto d = cholesky_solve(std::move(h), dim, std::move(neg));
        if (!d) {
            return;
        }
        double step = 0.0;
        for (std::size_t k = 1; k < n; k++) {
            const std::size_t o = 4 * (k - 1);
            x[k].first = wrap(x[k].first + (*d)[o]);
            for (std::size_t i = 0; i < 3; i++) {
                x[k].second[i] += (*d)[o + 1 + i];
            }
            double m = 0.0;
            for (std::size_t i = 0; i < 4; i++) {
                m = rf::fmax(m, std::fabs((*d)[o + i]));
            }
            step = rf::fmax(step, m);
        }
        if (step < 1e-8) {
            break;
        }
    }
}

Alignment correction_at(std::span<const Correction> c, std::int64_t ts) {
    if (c.empty()) {
        return Alignment{};
    }
    const std::size_t i = static_cast<std::size_t>(
        std::partition_point(c.begin(), c.end(), [&](const Correction& x) { return x.ts_ns < ts; }) - c.begin());
    if (i == 0) {
        return c[0].c;
    }
    if (i >= c.size()) {
        return c.back().c;
    }
    const Correction& a = c[i - 1];
    const Correction& b = c[i];
    const double f = static_cast<double>(ts - a.ts_ns) / static_cast<double>(std::max<std::int64_t>(b.ts_ns - a.ts_ns, 1));
    Alignment out;
    out.yaw = wrap(a.c.yaw + f * wrap(b.c.yaw - a.c.yaw));
    for (std::size_t k = 0; k < 3; k++) {
        out.t[k] = a.c.t[k] * (1.0 - f) + b.c.t[k] * f;
    }
    return out;
}

std::pair<LoopStats, std::vector<Correction>> close_loops(map::RoomMap& map, const stereo::Rig& rig,
                                                          std::vector<refine::MapFrame>& frames,
                                                          const align::ObserveParams& op, const LoopParams& p,
                                                          std::size_t threads) {
    LoopStats st;
    std::vector<std::size_t> order(map.keyframes.size());
    for (std::size_t i = 0; i < order.size(); i++) {
        order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t a, std::size_t b) { return map.keyframes[a].ts_ns < map.keyframes[b].ts_ns; });
    const std::size_t n = order.size();
    if (n < 3) {
        return {st, {}};
    }
    std::map<std::size_t, std::size_t> node_of;
    for (std::size_t nd = 0; nd < n; nd++) {
        node_of[order[nd]] = nd;
    }
    std::vector<Pose> old;
    for (std::size_t k : order) {
        old.push_back(map.keyframes[k].pose);
    }
    std::vector<Edge> edges;
    for (std::size_t i = 1; i < n; i++) {
        edges.push_back(odo_edge(i - 1, i, old[i - 1], old[i], p));
    }
    // loop detection (parallel over keyframes, results in node order)
    const std::size_t nt = std::clamp<std::size_t>(threads, 1, 64);
    const std::size_t chunk = std::max<std::size_t>((n + nt - 1) / nt, 1);
    std::vector<std::vector<Edge>> found_parts((n + chunk - 1) / chunk);
    {
        const map::RoomMap& map_ref = map;
        const std::vector<refine::MapFrame>& frames_ref = frames;
        std::vector<std::jthread> hs;
        for (std::size_t part = 0; part < found_parts.size(); part++) {
            hs.emplace_back([&, part] {
                std::vector<Edge>& out = found_parts[part];
                const std::size_t lo = part * chunk, hi = std::min(n, lo + chunk);
                for (std::size_t ni = lo; ni < hi; ni++) {
                    const std::size_t ki = order[ni];
                    const std::int64_t ti = map_ref.keyframes[ki].ts_ns;
                    const std::size_t fi = static_cast<std::size_t>(
                        std::partition_point(frames_ref.begin(), frames_ref.end(),
                                             [&](const refine::MapFrame& f) { return f.ts_ns < ti; }) -
                        frames_ref.begin());
                    const refine::MapFrame* f = nullptr;
                    std::int64_t fd = 0;
                    for (std::size_t j : {fi == 0 ? std::size_t{0} : fi - 1, fi}) {
                        if (j >= frames_ref.size()) {
                            continue;
                        }
                        const std::int64_t d = frames_ref[j].ts_ns > ti ? frames_ref[j].ts_ns - ti : ti - frames_ref[j].ts_ns;
                        if (d > 60000000) {
                            continue;
                        }
                        if (f == nullptr || d < fd) {
                            f = &frames_ref[j];
                            fd = d;
                        }
                    }
                    if (f == nullptr) {
                        continue;
                    }
                    std::vector<std::size_t> far;
                    for (std::size_t j = 0; j < map_ref.keyframes.size(); j++) {
                        const std::int64_t d = map_ref.keyframes[j].ts_ns - ti;
                        if ((d < 0 ? -d : d) > p.min_gap_ns) {
                            far.push_back(j);
                        }
                    }
                    const auto o1 = align::observe_kfs(map_ref, far, rig, f->feats, f->ts_ns, f->room_from_body, op);
                    if (!o1) {
                        continue;
                    }
                    // dominant keyframe among the inliers (most inliers; ties to the lower index)
                    std::map<std::uint32_t, std::size_t> cnt;
                    for (std::size_t i : o1->inliers) {
                        cnt[o1->corr_kf[i]]++;
                    }
                    if (cnt.empty()) {
                        continue;
                    }
                    std::uint32_t kk = 0;
                    std::size_t best = 0;
                    bool have = false;
                    for (const auto& [k, c] : cnt) {
                        if (!have || c > best) {
                            kk = k;
                            best = c;
                            have = true;
                        }
                    }
                    const std::int64_t tk = map_ref.keyframes[kk].ts_ns;
                    std::vector<std::size_t> local;
                    for (std::size_t j : far) {
                        const std::int64_t d = map_ref.keyframes[j].ts_ns - tk;
                        if ((d < 0 ? -d : d) <= p.local_ns) {
                            local.push_back(j);
                        }
                    }
                    const auto o2 = align::observe_kfs(map_ref, local, rig, f->feats, f->ts_ns, f->room_from_body, op);
                    if (!o2) {
                        continue;
                    }
                    if (o2->inliers.size() < p.min_inliers ||
                        static_cast<double>(o2->inliers.size()) < p.min_frac * static_cast<double>(o2->corrs.size()) ||
                        o2->rms_deg > p.max_rms_deg) {
                        continue;
                    }
                    const Pose& ti_old = old[ni];
                    const Pose meas = o2->room_from_body.compose(f->room_from_body.inverse().compose(ti_old));
                    const double jump_m = g::norm(g::sub(meas.t, ti_old.t));
                    const double jump_d = rf::to_degrees(std::fabs(wrap(map::yaw_of_mat(meas.r) - map::yaw_of_mat(ti_old.r))));
                    if (jump_m > p.max_jump_m || jump_d > p.max_jump_deg) {
                        continue;
                    }
                    const std::size_t nk = node_of.at(kk);
                    Edge e = odo_edge(nk, ni, old[nk], meas, p);
                    const double sy = rf::to_radians(p.sigma_loop_deg);
                    e.wt = 1.0 / (p.sigma_loop_m * p.sigma_loop_m);
                    e.wy = 1.0 / (sy * sy);
                    e.is_loop = true;
                    out.push_back(e);
                }
            });
        }
    }
    std::vector<Edge> found;
    for (auto& v : found_parts) {
        found.insert(found.end(), v.begin(), v.end());
    }
    st.tried = n;
    st.loops = found.size();
    edges.insert(edges.end(), found.begin(), found.end());
    std::vector<Node> x0;
    for (const Pose& q : old) {
        x0.emplace_back(map::yaw_of_mat(q.r), q.t);
    }
    std::vector<Node> x = x0;
    solve_graph(x, edges, 20, 2.0);
    for (int k = 0; k < 3; k++) {
        const std::size_t before = edges.size();
        std::erase_if(edges, [&](const Edge& e) { return e.is_loop && !(chi(e, x) <= p.reject_chi); });
        st.rejected += before - edges.size();
        if (edges.size() == before) {
            break;
        }
        x = x0;
        solve_graph(x, edges, 20, 2.0);
    }
    std::vector<Correction> corr;
    corr.reserve(n);
    for (std::size_t nd = 0; nd < n; nd++) {
        const std::size_t k = order[nd];
        const auto& [yo, to] = x0[nd];
        const auto& [yn, tn] = x[nd];
        const double yc = wrap(yn - yo);
        const Vec3 tc = g::sub(tn, g::mat_vec(pnp::rot_y(yc), to));
        const Alignment c{yc, tc};
        st.max_corr_m = rf::fmax(st.max_corr_m, g::norm(g::sub(tn, to)));
        st.max_corr_deg = rf::fmax(st.max_corr_deg, rf::to_degrees(std::fabs(yc)));
        const Pose cp = c.pose();
        auto& kf = map.keyframes[k];
        kf.pose = cp.compose(kf.pose);
        for (auto& m : kf.points) {
            m.p = cp.apply(m.p);
        }
        corr.push_back(Correction{kf.ts_ns, c});
    }
    for (auto& f : frames) {
        const Pose c = correction_at(corr, f.ts_ns).pose();
        f.room_from_body = c.compose(f.room_from_body);
    }
    return {st, corr};
}

} // namespace pibridge::roommap::loopclose
