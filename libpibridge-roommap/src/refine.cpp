// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/refine.rs (MIT OR Apache-2.0). See refine.h.
#include "pibridge/roommap/refine.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <thread>

namespace pibridge::roommap::refine {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;
using g::Vec3;

Grid Grid::make(const stereo::Features& f, std::size_t w, std::size_t h, float cell) {
    Grid gr;
    gr.cell = cell;
    gr.cols = rs::to_usize(std::ceil(static_cast<float>(w) / cell)) + 1;
    gr.rows = rs::to_usize(std::ceil(static_cast<float>(h) / cell)) + 1;
    gr.cells.assign(gr.cols * gr.rows, {});
    for (std::size_t i = 0; i < f.kps.size(); i++) {
        const std::size_t cx = rs::to_usize(f.kps[i].x / cell);
        const std::size_t cy = rs::to_usize(f.kps[i].y / cell);
        if (cx < gr.cols && cy < gr.rows) {
            gr.cells[cy * gr.cols + cx].push_back(static_cast<std::uint32_t>(i));
        }
    }
    return gr;
}

void Grid::near(float x, float y, float r, std::vector<std::uint32_t>& out) const {
    out.clear();
    const std::size_t x0 = rs::to_usize(std::fmax(std::floor((x - r) / cell), 0.0f));
    const std::size_t x1 = rs::to_usize(std::floor((x + r) / cell));
    const std::size_t y0 = rs::to_usize(std::fmax(std::floor((y - r) / cell), 0.0f));
    const std::size_t y1 = rs::to_usize(std::floor((y + r) / cell));
    const std::size_t ye = std::min(y1, rows - 1);
    const std::size_t xe = std::min(x1, cols - 1);
    for (std::size_t cy = y0; cy <= ye; cy++) {
        for (std::size_t cx = x0; cx <= xe; cx++) {
            const auto& c = cells[cy * cols + cx];
            out.insert(out.end(), c.begin(), c.end());
        }
    }
}

MapFrame MapFrame::make(std::int64_t ts_ns, const Pose& room_from_body, std::vector<stereo::Features> feats,
                        const stereo::Rig& rig) {
    MapFrame m;
    m.ts_ns = ts_ns;
    m.room_from_body = room_from_body;
    for (std::size_t i = 0; i < feats.size() && i < rig.cams.size(); i++) {
        m.grids.push_back(Grid::make(feats[i], rig.cams[i].pinhole.w, rig.cams[i].pinhole.h, 16.0f));
    }
    for (auto& f : feats) {
        f.img = std::vector<std::uint8_t>();
    }
    m.feats = std::move(feats);
    return m;
}

namespace {

struct Obs {
    Pose cam_from_room;
    Vec3 centre;
    Vec3 b;
};

std::optional<std::pair<float, float>> project(const stereo::Rig& rig, std::size_t cam, const Vec3& x_cam) {
    const auto& ph = rig.cams[cam].pinhole;
    const Vec3 v = g::mat_vec(g::transpose(ph.cam_from_virt), x_cam);
    if (v[2] <= 1e-6) {
        return std::nullopt;
    }
    const double x = ph.f * v[0] / v[2] + ph.cx;
    const double y = ph.f * v[1] / v[2] + ph.cy;
    const float xf = static_cast<float>(x), yf = static_cast<float>(y);
    if (!ph.covered(xf, yf)) {
        return std::nullopt;
    }
    return std::pair{xf, yf};
}

std::optional<Vec3> residual(const Obs& o, const Vec3& p) {
    const Vec3 x = o.cam_from_room.apply(p);
    const double n = g::norm(x);
    if (std::isnan(n) || n <= 1e-9 || g::dot(x, o.b) <= 0.0) {
        return std::nullopt;
    }
    return g::cross(o.b, g::scale(x, 1.0 / n));
}

double res_deg(const Obs& o, const Vec3& p) {
    if (auto r = residual(o, p)) {
        return rf::to_degrees(std::asin(rf::fmin(g::norm(*r), 1.0)));
    }
    return 180.0;
}

std::optional<Vec3> solve3(const g::Mat3& a, const Vec3& b) {
    const double d = g::det(a);
    if (std::fabs(d) < 1e-18) {
        return std::nullopt;
    }
    auto col = [&](int k) {
        g::Mat3 m = a;
        for (int i = 0; i < 3; i++) {
            m[i][k] = b[i];
        }
        return g::det(m) / d;
    };
    return Vec3{col(0), col(1), col(2)};
}

Vec3 triangulate(const std::vector<const Obs*>& obs, const Vec3& init) {
    Vec3 p = init;
    for (int it = 0; it < 10; it++) {
        g::Mat3 h{};
        Vec3 gv{0.0, 0.0, 0.0};
        for (const Obs* o : obs) {
            const auto r0 = residual(*o, p);
            if (!r0) {
                continue;
            }
            g::Mat3 j{};
            for (int k = 0; k < 3; k++) {
                Vec3 q = p;
                const double s = 1e-6 * (1.0 + g::norm(g::sub(p, o->centre)));
                q[k] += s;
                const auto r1 = residual(*o, q);
                if (!r1) {
                    continue;
                }
                for (int i = 0; i < 3; i++) {
                    j[i][k] = ((*r1)[i] - (*r0)[i]) / s;
                }
            }
            for (int i = 0; i < 3; i++) {
                for (int a = 0; a < 3; a++) {
                    gv[a] += j[i][a] * (*r0)[i];
                    for (int b = 0; b < 3; b++) {
                        h[a][b] += j[i][a] * j[i][b];
                    }
                }
            }
        }
        const auto d = solve3(h, {-gv[0], -gv[1], -gv[2]});
        if (!d) {
            break;
        }
        p = g::add(p, *d);
        if (g::norm(*d) < 1e-7) {
            break;
        }
    }
    return p;
}

double parallax_deg(const std::vector<const Obs*>& obs, const Vec3& p) {
    std::vector<Vec3> dirs;
    for (const Obs* o : obs) {
        dirs.push_back(g::normalize(g::sub(p, o->centre)));
    }
    double best = 0.0;
    for (std::size_t i = 0; i < dirs.size(); i++) {
        for (std::size_t j = i + 1; j < dirs.size(); j++) {
            best = rf::fmax(best, g::angle_deg(dirs[i], dirs[j]));
        }
    }
    return best;
}

struct Part {
    RefineStats st;
    std::vector<double> moves;
    std::vector<double> nobs;
};

Part refine_part(std::span<map::Keyframe> kfs, const stereo::Rig& rig, const std::vector<MapFrame>& frames,
                 const RefineParams& p, const std::vector<Pose>& bfc) {
    Part out;
    std::vector<std::uint32_t> cand;
    for (auto& kf : kfs) {
        std::vector<const MapFrame*> near;
        for (const auto& f : frames) {
            const std::int64_t d = f.ts_ns > kf.ts_ns ? f.ts_ns - kf.ts_ns : kf.ts_ns - f.ts_ns;
            if (d <= p.window_ns) {
                near.push_back(&f);
            }
        }
        std::vector<std::vector<std::pair<Pose, Vec3>>> cams;
        cams.reserve(near.size());
        for (const MapFrame* f : near) {
            std::vector<std::pair<Pose, Vec3>> v;
            for (const Pose& b : bfc) {
                const Pose rc = f->room_from_body.compose(b);
                v.emplace_back(rc.inverse(), rc.t);
            }
            cams.push_back(std::move(v));
        }
        const Pose kf_body = kf.pose;
        std::vector<map::MapPoint> keep;
        keep.reserve(kf.points.size());
        for (const auto& mp : kf.points) {
            out.st.points++;
            std::vector<Obs> obs;
            for (std::size_t fi = 0; fi < near.size(); fi++) {
                const MapFrame* f = near[fi];
                for (std::size_t c = 0; c < std::min(f->feats.size(), rig.cams.size()); c++) {
                    const auto& [cfr, centre] = cams[fi][c];
                    const Vec3 xc = cfr.apply(mp.p);
                    const auto uv = project(rig, c, xc);
                    if (!uv) {
                        continue;
                    }
                    const auto [u, v] = *uv;
                    f->grids[c].near(u, v, p.radius_px, cand);
                    std::uint32_t best = std::numeric_limits<std::uint32_t>::max();
                    std::uint32_t second = std::numeric_limits<std::uint32_t>::max();
                    std::size_t bi = std::numeric_limits<std::size_t>::max();
                    for (std::uint32_t i : cand) {
                        const auto& k = f->feats[c].kps[i];
                        const float dx = k.x - u, dy = k.y - v;
                        if (dx * dx + dy * dy > p.radius_px * p.radius_px) {
                            continue;
                        }
                        const std::uint32_t d = orb::hamming(mp.desc, f->feats[c].desc[i]);
                        if (d < best) {
                            second = best;
                            best = d;
                            bi = i;
                        } else if (d < second) {
                            second = d;
                        }
                    }
                    if (best <= p.max_dist && (second == std::numeric_limits<std::uint32_t>::max() ||
                                               static_cast<float>(best) < 0.9f * static_cast<float>(second))) {
                        obs.push_back(Obs{cfr, centre, f->feats[c].bearings[bi]});
                    }
                }
            }
            const double depth = g::norm(g::sub(mp.p, kf_body.t));
            std::optional<std::tuple<Vec3, double, std::size_t>> refined;
            if (obs.size() >= p.min_obs) {
                std::vector<const Obs*> all;
                for (const auto& o : obs) {
                    all.push_back(&o);
                }
                Vec3 x = triangulate(all, mp.p);
                std::vector<const Obs*> inl;
                for (const Obs* o : all) {
                    if (res_deg(*o, x) <= p.gate_deg * 2.0) {
                        inl.push_back(o);
                    }
                }
                if (inl.size() >= p.min_obs) {
                    x = triangulate(inl, x);
                    inl.clear();
                    for (const Obs* o : all) {
                        if (res_deg(*o, x) <= p.gate_deg) {
                            inl.push_back(o);
                        }
                    }
                    if (inl.size() >= p.min_obs) {
                        x = triangulate(inl, x);
                        double ss = -0.0;
                        for (const Obs* o : inl) {
                            const double r = res_deg(*o, x);
                            ss += r * r;
                        }
                        const double rms = std::sqrt(ss / static_cast<double>(inl.size()));
                        const double mv = g::norm(g::sub(x, mp.p));
                        bool front = true;
                        for (const Obs* o : inl) {
                            if (!(g::dot(o->cam_from_room.apply(x), o->b) > 0.0)) {
                                front = false;
                                break;
                            }
                        }
                        if (front && rms <= p.max_rms_deg && parallax_deg(inl, x) >= p.min_parallax_deg &&
                            mv <= rf::fmin(p.max_move_m, p.max_move_frac * rf::fmax(depth, 0.3))) {
                            refined = std::tuple{x, mv, inl.size()};
                        }
                    }
                }
            }
            if (refined) {
                out.st.refined++;
                out.moves.push_back(std::get<1>(*refined));
                out.nobs.push_back(static_cast<double>(std::get<2>(*refined)));
                map::MapPoint q = mp;
                q.p = std::get<0>(*refined);
                keep.push_back(q);
            } else if (p.drop_unrefined) {
                out.st.dropped++;
            } else {
                keep.push_back(mp);
            }
        }
        kf.points = std::move(keep);
    }
    return out;
}

double med(std::vector<double> v) {
    std::stable_sort(v.begin(), v.end(), rs::total_less);
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return v[v.size() / 2];
}

} // namespace

RefineStats refine_map(map::RoomMap& map, const stereo::Rig& rig, const std::vector<MapFrame>& frames,
                       const RefineParams& p, std::size_t threads) {
    const auto bfc = rig.body_from_cams();
    RefineStats st;
    const std::size_t n_kf = map.keyframes.size();
    const std::size_t nt = std::clamp<std::size_t>(threads, 1, 64);
    const std::size_t chunk = std::max<std::size_t>((n_kf + nt - 1) / nt, 1);
    std::vector<Part> parts((n_kf + chunk - 1) / chunk);
    {
        std::vector<std::jthread> hs;
        for (std::size_t c = 0; c < parts.size(); c++) {
            const std::size_t a = c * chunk, b = std::min(n_kf, a + chunk);
            hs.emplace_back([&, a, b, c] {
                parts[c] = refine_part(std::span(map.keyframes).subspan(a, b - a), rig, frames, p, bfc);
            });
        }
    }
    std::vector<double> moves, nobs;
    for (const auto& pt : parts) {
        st.points += pt.st.points;
        st.refined += pt.st.refined;
        st.dropped += pt.st.dropped;
        moves.insert(moves.end(), pt.moves.begin(), pt.moves.end());
        nobs.insert(nobs.end(), pt.nobs.begin(), pt.nobs.end());
    }
    st.median_move_m = med(std::move(moves));
    st.median_obs = med(std::move(nobs));
    return st;
}

} // namespace pibridge::roommap::refine
