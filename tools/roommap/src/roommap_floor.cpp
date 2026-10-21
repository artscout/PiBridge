// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/floorcmd.rs (MIT OR Apache-2.0): `pibridge-roommap floor ...` (detect,
// drift, touch, set, show, follow, preview) and the floor of a new map.
#include "roommap_cli.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/shm.h"
#include "pibridge/roommap/floor.h"
#include "pibridge/roommap/live.h"
#include "pibridge/roommap/loopclose.h"
#include "pibridge/roommap/pipeline.h"
#include "pibridge/roommap/rec.h"

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <deque>
#include <thread>

namespace pibridge::roommap::cli {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;
using session::Alignment;

namespace {

const char* FLOOR_HELP =
    "pibridge-roommap floor detect  [--name NAME | --map DIR] [--rec DIR] [--max-range M] [--save]\n"
    "    floor from the map points (saved as NAME/floor.toml + preview.txt with --save); with --rec alone from a\n"
    "    recording's stereo points in its tracker world; with --rec and a map, from the recording the map was built\n"
    "    from (dense points through the map's loop-closure correction; what build does)\n"
    "pibridge-roommap floor drift   --rec DIR [--window S] [--every N] [--csv FILE]\n"
    "    floor height per window over a recording: the tracker's vertical drift, and the residual with the continuous\n"
    "    floor lock (Follower) simulated causally\n"
    "pibridge-roommap floor touch   [--name NAME] [--hand left|right] [--bottom M] [--seconds S] [--save]\n"
    "    live, read-only: a controller lying still on the floor for 1 s (optical pose + Monado head ring)\n"
    "pibridge-roommap floor set     [--name NAME] (--y Y | --standing EYE_HEIGHT_M)\n"
    "    manual floor (room frame), or from the current head height while standing upright (live head ring)\n"
    "pibridge-roommap floor show    [--name NAME]\n"
    "    the stored floor, and live (read-only): head height above it, the offset's floor shift\n"
    "pibridge-roommap floor follow  [--seconds S] [--rate M_PER_S] [--window S] [--no-snap] [--dry-run]\n"
    "    live floor lock: re-measure the floor from the broker's stereo points every few seconds and move the\n"
    "    offset's y smoothly (at most RATE m/s, default 0.01) so the floor stays at Monado's STAGE y = 0; the first\n"
    "    measurement places it at once unless --no-snap\n"
    "pibridge-roommap floor preview [--name NAME | --map DIR]\n"
    "    (re)write NAME/preview.txt (top-down points for the app)";

void print_floor(const std::string& label, const floor::Floor& f) {
    std::printf("%s: floor y %s m (+-%s mm, spread %s mm, %zu points, %s m^2, tilt %s deg) | head %s m above | confidence %s\n",
                label.c_str(), rf::fixed(f.layer.y, 4, true).c_str(), rf::fixed(f.layer.sigma_m * 1e3, 1).c_str(),
                rf::fixed(f.layer.spread_m * 1e3, 1).c_str(), f.layer.inliers, rf::fixed(f.layer.area_m2, 1).c_str(),
                rf::fixed(f.layer.tilt_deg, 2).c_str(), rf::fixed(f.head_above_m, 3).c_str(),
                rf::fixed(f.confidence, 2).c_str());
}

void print_layers(const floor::Floor& f) {
    for (const auto& l : f.layers) {
        std::printf("    layer %s m  %s points  %s m^2  tilt %s deg  spread %s mm%s\n", rf::fixed(l.y, 3, true).c_str(),
                    std::format("{:>5}", l.inliers).c_str(), rf::fixed_w(l.area_m2, 5, 1).c_str(),
                    rf::fixed(l.tilt_deg, 2).c_str(), rf::fixed(l.spread_m * 1e3, 1).c_str(),
                    std::fabs(l.y - f.layer.y) < 1e-9 ? "  <- floor" : "");
    }
    for (const auto& r : f.rejected) {
        std::printf("    not the floor: %s\n", r.c_str());
    }
}

floor::StoredFloor stored(const floor::Floor& f) {
    floor::StoredFloor s;
    s.y = f.layer.y;
    s.source = floor::FloorSource::Points;
    s.confidence = f.confidence;
    s.sigma_m = f.layer.sigma_m;
    s.inliers = f.layer.inliers;
    s.head_above_m = f.head_above_m;
    s.when = unix_now();
    return s;
}

R<void> write_preview(const std::string& dir, const map::RoomMap& map, std::optional<double> floor_y) {
    const auto [pts, heads] = floor::map_points(map, 4.0);
    if (auto w = cam::write_file(cam::path_join(dir, "preview.txt"), floor::preview_text(pts, heads, floor_y, 6000)); !w) {
        return std::unexpected(w.error().text());
    }
    return {};
}

void floor_at_build(const std::string& dir, const map::RoomMap& map, const std::vector<pipeline::PointFrame>& frames,
                    const std::vector<loopclose::Correction>& corr) {
    std::vector<g::Vec3> pts, heads;
    for (const auto& f : frames) {
        const g::Pose c = loopclose::correction_at(corr, f.ts_ns).pose();
        heads.push_back(c.apply(f.pose.t));
        for (const auto& p : f.pts) {
            pts.push_back(c.apply(p));
        }
    }
    auto f = floor::detect(pts, heads, floor::FloorParams{});
    if (f) {
        print_floor("floor", *f);
        if (auto s = stored(*f).save(dir); !s) {
            std::fprintf(stderr, "floor.toml: %s\n", s.error().c_str());
        }
        (void)write_preview(dir, map, f->layer.y);
    } else {
        std::printf("floor not found in the mapping walk (%s): look at the floor around you while mapping, or set it "
                    "(omarchyvr-roommap floor touch|set)\n",
                    f.error().c_str());
        (void)write_preview(dir, map, std::nullopt);
    }
}

#define TRYV(var, expr)                                                                                                \
    auto var##_r = (expr);                                                                                             \
    if (!var##_r) {                                                                                                    \
        return std::unexpected(var##_r.error());                                                                       \
    }                                                                                                                  \
    auto var = *var##_r

R<void> cmd_detect(Args v) {
    TRYV(recp, opt(v, "--rec"));
    TRYV(max_range, num_f64(v, "--max-range", 3.5));
    const bool save = flag(v, "--save");
    TRYV(threads, num_usize(v, "--threads", 4));
    TRYV(every, num_usize(v, "--every", 2));
    // PiBridge: --min-below M: a layer must be at least M below the head (the standing fast setup passes 1.0)
    TRYV(min_below, num_f64(v, "--min-below", floor::FloorParams{}.head_min_m));
    floor::FloorParams fp;
    fp.head_min_m = min_below;
    if (recp) {
        auto rr = rig(v);
        if (!rr) {
            return std::unexpected(rr.error());
        }
        const auto& rg = rr->second;
        const bool has_map = std::find(v.begin(), v.end(), "--map") != v.end() || std::find(v.begin(), v.end(), "--name") != v.end();
        TRYV(dir, map_dir(v));
        if (auto d = done(v); !d) {
            return d;
        }
        auto rec = rec::Recording::open(*recp, std::nullopt);
        if (!rec) {
            return std::unexpected(rec.error());
        }
        auto fr = pipeline::recording_points(*rec, rg, every, max_range, threads);
        if (!fr) {
            return std::unexpected(fr.error());
        }
        if (has_map) {
            auto m = map::RoomMap::load(dir);
            if (!m) {
                return std::unexpected(m.error());
            }
            std::printf("%s: %zu frames into map %s\n", rec->name().c_str(), fr->size(), dir.c_str());
            if (save) {
                floor_at_build(dir, *m, *fr, store::load_corrections(dir));
            } else {
                const auto corr = store::load_corrections(dir);
                std::vector<g::Vec3> pts, heads;
                for (const auto& f : *fr) {
                    const g::Pose c = loopclose::correction_at(corr, f.ts_ns).pose();
                    heads.push_back(c.apply(f.pose.t));
                    for (const auto& p : f.pts) {
                        pts.push_back(c.apply(p));
                    }
                }
                auto f = floor::detect(pts, heads, fp);
                if (!f) {
                    return std::unexpected(f.error());
                }
                print_floor("mapping walk floor (room frame)", *f);
                print_layers(*f);
            }
            return {};
        }
        std::vector<g::Vec3> pts, heads;
        for (const auto& f : *fr) {
            pts.insert(pts.end(), f.pts.begin(), f.pts.end());
        }
        for (const auto& f : *fr) {
            heads.push_back(f.pose.t);
        }
        std::printf("%s: %zu frames, %zu stereo points\n", rec->name().c_str(), fr->size(), pts.size());
        auto f = floor::detect(pts, heads, fp);
        if (!f) {
            return std::unexpected(f.error());
        }
        print_floor("whole session (tracker world)", *f);
        print_layers(*f);
        return {};
    }
    TRYV(dir, map_dir(v));
    if (auto d = done(v); !d) {
        return d;
    }
    auto m = map::RoomMap::load(dir);
    if (!m) {
        return std::unexpected(m.error());
    }
    const auto [pts, heads] = floor::map_points(*m, max_range);
    std::printf("map %s: %zu keyframes, %zu points within %s m\n", dir.c_str(), heads.size(), pts.size(),
                rf::display_f64(max_range).c_str());
    auto f = floor::detect(pts, heads, fp);
    if (!f) {
        return std::unexpected(f.error());
    }
    print_floor("map floor (room frame)", *f);
    print_layers(*f);
    if (save) {
        const auto s = stored(*f);
        if (auto old = floor::StoredFloor::load(dir)) {
            if (old->source != floor::FloorSource::Points && std::fabs(old->y - s.y) > 0.03) {
                std::printf("kept the %s floor %s m (points say %s m)\n", floor::source_name(old->source),
                            rf::fixed(old->y, 4, true).c_str(), rf::fixed(s.y, 4, true).c_str());
                return write_preview(dir, *m, old->y);
            }
        }
        if (auto w = s.save(dir); !w) {
            return w;
        }
        if (auto w = write_preview(dir, *m, s.y); !w) {
            return w;
        }
        std::printf("saved %s and preview.txt\n", cam::path_join(dir, "floor.toml").c_str());
    }
    return {};
}

} // namespace

/// Window measurements of the floor (tracker world): (window end s, floor y, points, head above).
std::vector<std::tuple<double, double, std::size_t, double>> window_floors(const std::vector<pipeline::PointFrame>& fr,
                                                                           double win_s, double step_s,
                                                                           std::optional<double> prior) {
    std::vector<std::tuple<double, double, std::size_t, double>> out;
    if (fr.empty()) {
        return out;
    }
    const std::int64_t t0 = fr.front().ts_ns;
    const double t_end = static_cast<double>(fr.back().ts_ns - t0) / 1e9;
    floor::FloorParams p;
    p.min_support = 40;
    p.min_area_m2 = 0.25;
    double t = win_s;
    while (t <= t_end + 1e-9) {
        std::vector<g::Vec3> pts, heads;
        for (const auto& f : fr) {
            const double s = static_cast<double>(f.ts_ns - t0) / 1e9;
            if (s > t - win_s && s <= t) {
                pts.insert(pts.end(), f.pts.begin(), f.pts.end());
            }
        }
        for (const auto& f : fr) {
            const double s = static_cast<double>(f.ts_ns - t0) / 1e9;
            if (s > t - win_s && s <= t) {
                heads.push_back(f.pose.t);
            }
        }
        std::optional<floor::Floor> f;
        if (prior) {
            f = floor::track(pts, heads, *prior, 0.15, p);
        } else if (auto d = floor::detect(pts, heads, p)) {
            f = *d;
        }
        if (f) {
            out.emplace_back(t, f->layer.y, f->layer.inliers, f->head_above_m);
        }
        t += step_s;
    }
    return out;
}

namespace {

std::tuple<double, double, double> stats(const std::vector<double>& v) {
    std::vector<double> a;
    for (double x : v) {
        a.push_back(std::fabs(x));
    }
    std::stable_sort(a.begin(), a.end(), rs::total_less);
    if (a.empty()) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan};
    }
    auto q = [&](double f) { return a[rs::to_usize(std::round(static_cast<double>(a.size() - 1) * f))]; };
    return {q(0.5), q(0.9), q(1.0)};
}

double slope_cm_per_min(const std::vector<double>& t, const std::vector<double>& y) {
    const double n = static_cast<double>(t.size());
    if (t.size() < 3) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double st = -0.0, sy = -0.0;
    for (double x : t) {
        st += x;
    }
    for (double x : y) {
        sy += x;
    }
    const double mt = st / n, my = sy / n;
    double num = -0.0, den = -0.0;
    for (std::size_t i = 0; i < t.size(); i++) {
        num += (t[i] - mt) * (y[i] - my);
    }
    for (double a : t) {
        den += (a - mt) * (a - mt);
    }
    return num / den * 100.0 * 60.0;
}

R<void> cmd_drift(Args v) {
    auto rr = rig(v);
    if (!rr) {
        return std::unexpected(rr.error());
    }
    const auto& rg = rr->second;
    TRYV(recp, opt(v, "--rec"));
    if (!recp) {
        return std::unexpected("--rec DIR");
    }
    TRYV(win, num_f64(v, "--window", 6.0));
    TRYV(step, num_f64(v, "--step", 2.0));
    TRYV(every, num_usize(v, "--every", 1));
    TRYV(threads, num_usize(v, "--threads", 4));
    TRYV(rate, num_f64(v, "--rate", 0.01));
    TRYV(csv, opt(v, "--csv"));
    if (auto d = done(v); !d) {
        return d;
    }
    auto rec = rec::Recording::open(*recp, std::nullopt);
    if (!rec) {
        return std::unexpected(rec.error());
    }
    auto fr = pipeline::recording_points(*rec, rg, every, 3.5, threads);
    if (!fr) {
        return std::unexpected(fr.error());
    }
    std::vector<g::Vec3> pts, heads;
    for (const auto& f : *fr) {
        pts.insert(pts.end(), f.pts.begin(), f.pts.end());
    }
    for (const auto& f : *fr) {
        heads.push_back(f.pose.t);
    }
    std::printf("%s: %zu frames, %zu stereo points\n", rec->name().c_str(), fr->size(), pts.size());
    std::optional<double> prior;
    if (auto f = floor::detect(pts, heads, floor::FloorParams{})) {
        print_floor("whole session", *f);
        print_layers(*f);
        prior = f->layer.y;
    } else {
        std::printf("whole session: no floor (%s)\n", f.error().c_str());
    }
    const auto w = window_floors(*fr, win, step, prior);
    const std::int64_t last = fr->empty() ? 0 : fr->back().ts_ns;
    std::printf("%zu of the %s s windows found the floor\n", w.size(),
                rf::fixed(static_cast<double>(last - (*fr)[0].ts_ns) / 1e9 / step, 0).c_str());
    if (w.size() < 2) {
        return {};
    }
    const double y0 = std::get<1>(w[0]);
    floor::Follower fol;
    fol.max_rate = rate;
    std::vector<double> before, after, ts, ys;
    std::string lines = "t_s,floor_y,points,head_above,shift_applied,residual_uncorrected,residual_corrected\n";
    double t_prev = std::get<0>(w[0]);
    for (std::size_t k = 0; k < w.size(); k++) {
        const auto& [t, y, n, head] = w[k];
        double tt = t_prev;
        while (tt < t) {
            fol.step(1.0 / 90.0);
            tt += 1.0 / 90.0;
        }
        t_prev = t;
        const double shift = fol.applied.value_or(-y0);
        const double r_before = y - y0;
        const double r_after = y + shift;
        if (k > 0) {
            before.push_back(r_before);
            after.push_back(r_after);
        }
        ts.push_back(t);
        ys.push_back(y);
        lines += rf::fixed(t, 2) + "," + rf::fixed(y, 5) + "," + std::to_string(n) + "," + rf::fixed(head, 4) + "," +
                 rf::fixed(shift, 5) + "," + rf::fixed(r_before, 5) + "," + rf::fixed(r_after, 5) + "\n";
        fol.measure(-y, k == 0 ? 0.0 : step);
        if (k == 0) {
            fol.step(0.0);
        }
    }
    const auto [bm, b9, bx] = stats(before);
    const auto [am, a9, ax] = stats(after);
    std::printf("vertical drift (floor height vs the first window): median %s mm, p90 %s mm, max %s mm, trend %s cm/min\n",
                rf::fixed(bm * 1e3, 1).c_str(), rf::fixed(b9 * 1e3, 1).c_str(), rf::fixed(bx * 1e3, 1).c_str(),
                rf::fixed(slope_cm_per_min(ts, ys), 2, true).c_str());
    std::printf("with the floor lock (%s mm/s, tau %s s): median %s mm, p90 %s mm, max %s mm\n",
                rf::fixed(rate * 1e3, 0).c_str(), rf::fixed(fol.tau_s, 0).c_str(), rf::fixed(am * 1e3, 1).c_str(),
                rf::fixed(a9 * 1e3, 1).c_str(), rf::fixed(ax * 1e3, 1).c_str());
    std::vector<double> hd;
    for (const auto& x : w) {
        hd.push_back(std::get<3>(x));
    }
    std::stable_sort(hd.begin(), hd.end(), rs::total_less);
    std::printf("head above the floor: min %s m, median %s m, max %s m\n", rf::fixed(hd[0], 2).c_str(),
                rf::fixed(hd[hd.size() / 2], 2).c_str(), rf::fixed(hd.back(), 2).c_str());
    if (csv) {
        if (auto wr = cam::write_file(*csv, lines); !wr) {
            return std::unexpected(wr.error().text());
        }
        std::printf("wrote %s\n", csv->c_str());
    }
    return {};
}

std::optional<Alignment> session_fix(const std::string& dir, std::uint32_t pid) {
    auto r = store::RelocResult::load(cam::path_join(dir, "last-reloc.toml"));
    if (!r || !(r->pid == pid && pid != 0)) {
        return std::nullopt;
    }
    return r->align;
}

R<anchor::HeadRing> ring() {
    if (auto r = anchor::HeadRing::read(anchor::ring_path())) {
        return *r;
    }
    return std::unexpected("no Monado head ring at " + anchor::ring_path() + " (desk mode with --anchor, PIMAX_ANCHOR=1)");
}

R<void> cmd_touch(Args v) {
    TRYV(dir, map_dir(v));
    TRYV(hand_o, opt(v, "--hand"));
    const std::string hand = hand_o.value_or("right");
    TRYV(bottom, num_f64(v, "--bottom", 0.035));
    TRYV(seconds, num_f64(v, "--seconds", 20.0));
    const bool save = flag(v, "--save");
    if (auto d = done(v); !d) {
        return d;
    }
    std::size_t hi;
    if (hand == "left") {
        hi = 0;
    } else if (hand == "right") {
        hi = 1;
    } else {
        return std::unexpected("--hand left|right");
    }
    const std::string sp = ctrl::track::shm::default_path();
    auto reader = ctrl::track::shm::PoseReader::open(sp);
    if (!reader) {
        return std::unexpected("controller poses (" + sp + "): " + reader.error().text() + " (desk mode with --ctrl-track)");
    }
    std::printf("Lay the %s controller flat on the floor in view of the headset and let go; keep looking at it.\n",
                hand.c_str());
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::pair<std::uint64_t, g::Vec3>> buf;
    std::uint32_t pid = 0;
    std::uint64_t last_frame = 0;
    std::optional<std::pair<std::pair<double, double>, g::Vec3>> result;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const auto hs = (*reader)->read();
        if (!hs) {
            continue;
        }
        const auto h = (*hs)[hi];
        if (h.frame_ns == 0 || h.frame_ns == last_frame) {
            continue;
        }
        last_frame = h.frame_ns;
        const auto r = anchor::HeadRing::read(anchor::ring_path());
        if (!r) {
            continue;
        }
        pid = r->pid;
        const auto head = r->at(h.frame_ns, 100000000);
        if (!head) {
            continue;
        }
        const g::Vec3 p = head->apply({static_cast<double>(h.pos[0]), static_cast<double>(h.pos[1]), static_cast<double>(h.pos[2])});
        buf.emplace_back(h.frame_ns, p);
        std::erase_if(buf, [&](const std::pair<std::uint64_t, g::Vec3>& x) { return h.frame_ns - x.first > 1000000000ull; });
        if (buf.size() >= 10 && h.frame_ns - buf[0].first >= 900000000ull && head->t[1] - p[1] > 0.5) {
            std::vector<g::Vec3> o;
            for (const auto& x : buf) {
                o.push_back(x.second);
            }
            if (auto x = floor::touch_floor(o, bottom, 0.006)) {
                result = std::pair{*x, head->t};
                break;
            }
        }
    }
    if (!result) {
        return std::unexpected("no still controller on the floor within " + rf::fixed(seconds, 0) + " s");
    }
    const auto [ys, head] = *result;
    const auto [y_slam, spread] = ys;
    std::printf("controller on the floor: floor y %s m in the tracker world (still within %s mm, bottom offset %s mm)\n",
                rf::fixed(y_slam, 4, true).c_str(), rf::fixed(spread * 1e3, 1).c_str(), rf::fixed(bottom * 1e3, 0).c_str());
    std::printf("head is %s m above it now\n", rf::fixed(head[1] - y_slam, 3).c_str());
    const auto fix = session_fix(dir, pid);
    if (!fix) {
        std::printf("no room fix for this Monado session (relocalize first): floor not stored in the map\n");
        return {};
    }
    const double y_room = fix->pose().apply({head[0], y_slam, head[2]})[1];
    std::optional<floor::StoredFloor> points;
    if (auto f = floor::StoredFloor::load(dir); f && f->source == floor::FloorSource::Points) {
        points = *f;
    }
    std::optional<std::pair<double, double>> pp;
    if (points) {
        pp = std::pair{points->y, rf::fmax(points->sigma_m, 0.003)};
    }
    const auto [y, sig] = *floor::combine(pp, std::pair{y_room, 0.008}, 0.04);
    if (points) {
        std::printf("map points said %s m, touch %s m (room frame): using %s m\n", rf::fixed(points->y, 4, true).c_str(),
                    rf::fixed(y_room, 4, true).c_str(), rf::fixed(y, 4, true).c_str());
    }
    if (save) {
        floor::StoredFloor s;
        s.y = y;
        s.source = floor::FloorSource::Touch;
        s.confidence = 0.95;
        s.sigma_m = sig;
        s.inliers = 0;
        s.head_above_m = head[1] - y_slam;
        s.when = unix_now();
        if (auto w = s.save(dir); !w) {
            return w;
        }
        std::printf("saved %s\n", cam::path_join(dir, "floor.toml").c_str());
    }
    return {};
}

R<void> cmd_set(Args v) {
    TRYV(dir, map_dir(v));
    TRYV(y_o, opt(v, "--y"));
    TRYV(st_o, opt(v, "--standing"));
    if (auto d = done(v); !d) {
        return d;
    }
    floor::StoredFloor s;
    if (y_o && !st_o) {
        auto y = rs::parse_f64(*y_o);
        if (!y) {
            return std::unexpected("--y: bad number");
        }
        s.y = *y;
        s.source = floor::FloorSource::Manual;
        s.confidence = 0.6;
        s.sigma_m = 0.02;
        s.inliers = 0;
        s.head_above_m = std::numeric_limits<double>::quiet_NaN();
        s.when = unix_now();
    } else if (!y_o && st_o) {
        auto h = rs::parse_f64(*st_o);
        if (!h) {
            return std::unexpected("--standing: bad number");
        }
        if (!(*h >= 1.0 && *h <= 2.3)) {
            return std::unexpected("--standing is your eye height standing upright, 1.0-2.3 m");
        }
        auto r = ring();
        if (!r) {
            return std::unexpected(r.error());
        }
        const anchor::RingSample* last = nullptr;
        for (auto it = r->samples.rbegin(); it != r->samples.rend(); ++it) {
            if (it->valid) {
                last = &*it;
                break;
            }
        }
        if (last == nullptr) {
            return std::unexpected("no valid head pose in the ring");
        }
        const auto fix = session_fix(dir, r->pid);
        if (!fix) {
            return std::unexpected("no room fix for this Monado session: relocalize first");
        }
        const g::Vec3 head_room = fix->pose().apply(last->pose.t);
        s.y = head_room[1] - *h;
        s.source = floor::FloorSource::Standing;
        s.confidence = 0.7;
        s.sigma_m = 0.03;
        s.inliers = 0;
        s.head_above_m = *h;
        s.when = unix_now();
    } else {
        return std::unexpected("floor set needs --y Y or --standing EYE_HEIGHT_M");
    }
    if (auto w = s.save(dir); !w) {
        return w;
    }
    std::printf("saved %s: floor y %s m (%s)\n", cam::path_join(dir, "floor.toml").c_str(), rf::fixed(s.y, 4, true).c_str(),
                floor::source_name(s.source));
    if (auto m = map::RoomMap::load(dir)) {
        return write_preview(dir, *m, s.y);
    }
    return {};
}

R<void> cmd_show(Args v) {
    TRYV(dir, map_dir(v));
    if (auto d = done(v); !d) {
        return d;
    }
    if (auto f = floor::StoredFloor::load(dir)) {
        std::printf("%s: floor y %s m in the room frame (%s, confidence %s, +-%s mm, head %s m above while mapping)\n",
                    dir.c_str(), rf::fixed(f->y, 4, true).c_str(), floor::source_name(f->source),
                    rf::fixed(f->confidence, 2).c_str(), rf::fixed(f->sigma_m * 1e3, 1).c_str(),
                    rf::fixed(f->head_above_m, 2).c_str());
    } else {
        std::printf("no floor stored: %s\n", f.error().c_str());
    }
    auto r = ring();
    if (!r) {
        std::printf("Monado not running with the anchor hook: nothing live to show\n");
        return {};
    }
    const double oh = floor::origin_height(r->pid);
    std::optional<g::Pose> off;
    if (auto o = anchor::read_offset(anchor::offset_path()); o && o->first == r->pid) {
        off = o->second;
    }
    if (off) {
        std::printf("offset of Monado pid %u: t (%s %s %s)\n", r->pid, rf::fixed(off->t[0], 3, true).c_str(),
                    rf::fixed(off->t[1], 3, true).c_str(), rf::fixed(off->t[2], 3, true).c_str());
    } else {
        std::printf("no offset for Monado pid %u\n", r->pid);
    }
    for (auto it = r->samples.rbegin(); it != r->samples.rend(); ++it) {
        if (!it->valid) {
            continue;
        }
        const g::Pose head = it->pose;
        const g::Pose app = off.value_or(g::Pose{g::rot_x(0.0), {0.0, 0.0, 0.0}}).compose(head);
        std::printf("head now: %s m above Monado's STAGE floor (origin height %s m)\n", rf::fixed(app.t[1] + oh, 3).c_str(),
                    rf::fixed(oh, 2).c_str());
        auto f = floor::StoredFloor::load(dir);
        const auto fix = session_fix(dir, r->pid);
        if (f && fix) {
            const g::Vec3 room = fix->pose().apply(head.t);
            std::printf("head now: %s m above the room's floor\n", rf::fixed(room[1] - f->y, 3).c_str());
        }
        break;
    }
    return {};
}

} // namespace

/// The offset with its y moved so the floor (at floor_slam under the head, tracker world) lands on STAGE y = 0.
std::pair<g::Pose, double> with_floor(const g::Pose& off, const g::Vec3& floor_slam, double origin_height) {
    const g::Vec3 app = off.apply(floor_slam);
    const double d = floor::floor_shift(app[1], origin_height);
    return {g::Pose{off.r, {off.t[0], off.t[1] + d, off.t[2]}}, d};
}

R<std::pair<Alignment, std::optional<double>>> reloc_offset_hook(const std::string& dir, const Alignment& a,
                                                                 std::uint32_t pid) {
    // Under SteamVR the floor belongs to SteamVR's chaperone (its standing pose, written head-relative by the room
    // setup). The anchor offset must not move the world vertically from floor.toml's (raw room-frame) floor_y: that
    // shift (tens of metres) is double-counted with SteamVR's standing frame and throws the floor into the ceiling, or
    // blows the offset past the 20 m guard. Keep only the relocalisation's own alignment here; write_offset zeroes the
    // vertical for vrserver.
    if (floor::is_vrserver(pid)) {
        return std::pair{a, std::optional<double>{}};
    }
    if (auto f = floor::StoredFloor::load(dir)) {
        const double d = floor::floor_shift(f->y, floor::origin_height(pid));
        return std::pair{Alignment{a.yaw, {a.t[0], a.t[1] + d, a.t[2]}}, std::optional<double>{d}};
    }
    return std::pair{a, std::optional<double>{}};
}

R<void> floor_at_build_hook(const std::string& dir, const map::RoomMap& map, const std::vector<pipeline::PointFrame>& frames,
                            const std::vector<loopclose::Correction>& corr) {
    floor_at_build(dir, map, frames, corr);
    return {};
}

namespace {

R<void> cmd_follow(Args v) {
    auto rr = rig(v);
    if (!rr) {
        return std::unexpected(rr.error());
    }
    const auto& rg = rr->second;
    TRYV(seconds, num_f64(v, "--seconds", 1e9));
    TRYV(rate, num_f64(v, "--rate", 0.01));
    TRYV(win, num_f64(v, "--window", 6.0));
    const bool dry = flag(v, "--dry-run");
    const bool no_snap = flag(v, "--no-snap");
    TRYV(src_o, opt(v, "--source"));
    const std::string source = src_o.value_or("broker");
    if (auto d = done(v); !d) {
        return d;
    }
    const orb::OrbParams orb;
    std::deque<pipeline::PointFrame> frames;
    floor::Follower fol;
    fol.max_rate = rate;
    auto last_measure = std::chrono::steady_clock::now();
    auto last_write = std::chrono::steady_clock::now();
    std::size_t k = 0;
    std::optional<double> known;
    const std::string path = anchor::offset_path();
    std::printf("floor lock: measuring every 2 s over %s s windows, at most %s mm/s%s\n", rf::fixed(win, 0).c_str(),
                rf::fixed(rate * 1e3, 0).c_str(), dry ? " (dry run)" : "");
    auto secs = [](auto a) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count(); };
    auto st = live::capture(source, seconds, [&](live::LiveFrame& f) {
        k++;
        if (k % 3 == 0) {
            const auto feats = pipeline::frame_features(rg, f.imgs[0], f.imgs[1], orb);
            frames.push_back(pipeline::PointFrame{f.ts_ns, f.slam_from_body,
                                                  pipeline::frame_points(rg, feats, f.slam_from_body, 3.5)});
            while (!frames.empty() && f.ts_ns - frames.front().ts_ns > rs::to_i64(win * 1e9)) {
                frames.pop_front();
            }
        }
        const double oh = floor::origin_height(f.pid);
        g::Pose off{g::rot_x(0.0), {0.0, 0.0, 0.0}};
        if (auto o = anchor::read_offset(path); o && o->first == f.pid) {
            off = o->second;
        }
        if (secs(last_measure) >= 2.0 && frames.size() >= 10) {
            const double dt = secs(last_measure);
            last_measure = std::chrono::steady_clock::now();
            const std::vector<pipeline::PointFrame> fv(frames.begin(), frames.end());
            const auto wf = window_floors(fv, win, win, known);
            if (!wf.empty()) {
                const auto& [t, y, n, above] = wf.back();
                (void)t;
                known = y;
                const g::Vec3 h = f.slam_from_body.t;
                const auto [o2, d] = with_floor(off, {h[0], y, h[2]}, oh);
                (void)o2;
                fol.measure(off.t[1] + d, dt);
                std::printf("floor %s m (tracker), %zu points, head %s m above; offset y %s -> target %s\n",
                            rf::fixed(y, 4, true).c_str(), n, rf::fixed(above, 2).c_str(),
                            rf::fixed(off.t[1], 4, true).c_str(),
                            rf::fixed(fol.target.value_or(std::numeric_limits<double>::quiet_NaN()), 4, true).c_str());
            }
        }
        const double dtw = secs(last_write);
        if (dtw >= 0.25) {
            last_write = std::chrono::steady_clock::now();
            if (!fol.applied && no_snap) {
                fol.applied = off.t[1];
            }
            if (const auto y = fol.step(rf::fmin(dtw, 0.5))) {
                if (std::fabs(*y - off.t[1]) >= 0.0002) {
                    const g::Pose o{off.r, {off.t[0], *y, off.t[2]}};
                    if (!dry) {
                        if (auto l = take_lease(f.pid, anchor::Role::Follow); !l) {
                            std::fprintf(stderr, "floor follow stops: %s\n", l.error().c_str());
                            return false;
                        }
                        if (auto w = anchor::write_offset(path, f.pid, o); !w) {
                            std::fprintf(stderr, "cannot write %s: %s\n", path.c_str(), w.error().text().c_str());
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    });
    if (!st) {
        return std::unexpected(st.error());
    }
    return {};
}

R<void> cmd_preview(Args v) {
    TRYV(dir, map_dir(v));
    if (auto d = done(v); !d) {
        return d;
    }
    auto m = map::RoomMap::load(dir);
    if (!m) {
        return std::unexpected(m.error());
    }
    std::optional<double> fy;
    if (auto f = floor::StoredFloor::load(dir)) {
        fy = f->y;
    }
    if (auto w = write_preview(dir, *m, fy); !w) {
        return w;
    }
    std::printf("wrote %s\n", cam::path_join(dir, "preview.txt").c_str());
    return {};
}

} // namespace

R<void> cmd_floor(Args v) {
    if (v.empty()) {
        return std::unexpected(std::string("floor needs a subcommand:\n") + FLOOR_HELP);
    }
    const std::string sub = v[0];
    v.erase(v.begin());
    if (sub == "detect") {
        return cmd_detect(std::move(v));
    }
    if (sub == "drift") {
        return cmd_drift(std::move(v));
    }
    if (sub == "touch") {
        return cmd_touch(std::move(v));
    }
    if (sub == "set") {
        return cmd_set(std::move(v));
    }
    if (sub == "show") {
        return cmd_show(std::move(v));
    }
    if (sub == "follow") {
        return cmd_follow(std::move(v));
    }
    if (sub == "preview") {
        return cmd_preview(std::move(v));
    }
    return std::unexpected("unknown floor command " + rf::debug_str(sub) + "\n" + FLOOR_HELP);
}

} // namespace pibridge::roommap::cli
