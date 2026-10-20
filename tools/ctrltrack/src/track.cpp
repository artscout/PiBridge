// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/track.rs (MIT OR Apache-2.0).
//
// `track FILE.pxcam`: offline controller poses from a recording. Pass 1 detects the blobs of every LED frame, pass 2
// runs the brute-force acquisition per frame on all CPUs (the result does not depend on the thread count), pass 3
// tracks with priors; then the optional extrinsics refit, the CSV, the per-camera bias, the overlay pictures and the
// summary.
#include "cli.h"

#include "pibridge/cam/frame.h"
#include "pibridge/cam/record.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/blob.h"
#include "pibridge/ctrl/track/extrinsics.h"
#include "pibridge/ctrl/track/rig.h"
#include "pibridge/ctrl/track/rust.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <format>
#include <limits>
#include <map>
#include <memory>
#include <thread>

namespace pibridge::ctrltool {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace tr = ctrl::track;

namespace {

using Clock = std::chrono::steady_clock;

double secs_since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

bool has(std::span<const std::string> a, std::string_view k) { return std::find(a.begin(), a.end(), k) != a.end(); }

/// Rig::load: the rig, or the plain reason and exit 2 (the launcher then runs without optical tracking).
tr::Rig load_rig(std::span<const std::string> a) {
    auto r = tr::Rig::try_load(a);
    if (!r) {
        eprintln("pimax-ctrltrack: " + r.error());
        eprintln("pimax-ctrltrack: optical controller tracking is off; the controllers still work (buttons, gyro).");
        std::fflush(stdout);
        std::exit(2);
    }
    return std::move(*r);
}

/// Rust `sort_by(partial_cmp)` of a Vec<f64> (stable: -0.0 and 0.0 keep their order).
void sort_f64(std::vector<double>& v) {
    std::stable_sort(v.begin(), v.end(), [](double x, double y) { return x < y; });
}

/// The `pct` closure: the q quantile of the sorted values (NaN when empty).
double pct(std::vector<double>& v, double q) {
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    sort_f64(v);
    return v[tr::rs::f64_to_usize(std::round(static_cast<double>(v.size() - 1) * q))];
}

/// Open the recording or panic like `.expect("open")`.
std::unique_ptr<cam::Reader> open_or_panic(const std::string& path) {
    auto r = cam::Reader::open(path);
    if (!r) {
        panic("open: " + io_debug(r.error()));
    }
    return std::move(*r);
}

bool next_or_panic(cam::Reader& r, cam::Record& rec) {
    auto got = r.next_record(rec);
    if (!got) {
        panic("read: " + io_debug(got.error()));
    }
    return *got;
}

/// Mosaics of every `oe`-th solved LED frame: blobs red boxes, projected facing LEDs green (matched) or blue
/// (unmatched), the left controller's LEDs as filled dots, the right's as rings.
void overlay(const std::string& path, const std::string& dir, std::size_t oe, const std::vector<tr::LedFrame>& frames,
             const std::vector<std::vector<tr::Found>>& out, const std::vector<tr::Cam>& cams,
             const std::array<tr::LedModel, 2>& models, const tr::Params& prm) {
    if (auto c = cam::create_dir_all(dir); !c) {
        panic("overlay dir: " + io_debug(c.error()));
    }
    if (oe == 0) {
        panic("attempt to calculate the remainder with a divisor of zero");
    }
    // rec -> frame index (a lookup only: no iteration order reaches the output)
    std::map<std::size_t, std::size_t> want;
    for (std::size_t k = 0; k < frames.size(); k++) {
        if (k % oe == 0) {
            want[frames[k].rec] = k;
        }
    }
    auto r = open_or_panic(path);
    std::size_t i = 0;
    const std::size_t w = cam::TILE_W * cam::TILE_COUNT;
    const double cos_face = std::cos(rf::to_radians(prm.facing_deg));
    cam::Record rec;
    while (next_or_panic(*r, rec)) {
        i += 1;
        const auto it = want.find(i);
        if (it == want.end()) {
            continue;
        }
        const std::size_t k = it->second;
        const auto m = cam::Meta::parse(std::span(rec.data).first(cam::META_LEN));
        if (!m) {
            panic("called `Option::unwrap()` on a `None` value");
        }
        const cam::TileRows rows = cam::tile_rows(&*m, rec.data, rec.stride);
        std::vector<std::uint8_t> rgb(w * cam::TILE_H * 3, 0);
        std::vector<std::uint8_t> tile(cam::TILE_W * cam::TILE_H, 0);
        const auto put = [&](std::size_t t, std::int64_t x, std::int64_t y, std::array<std::uint8_t, 3> c) {
            if (x >= 0 && y >= 0 && static_cast<std::size_t>(x) < cam::TILE_W &&
                static_cast<std::size_t>(y) < cam::TILE_H) {
                const std::size_t o =
                    (static_cast<std::size_t>(y) * w + t * cam::TILE_W + static_cast<std::size_t>(x)) * 3;
                rgb[o] = c[0];
                rgb[o + 1] = c[1];
                rgb[o + 2] = c[2];
            }
        };
        for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
            cam::copy_tile(rec.data, rec.stride, t, rows.y[t], tile, cam::TILE_W);
            for (std::size_t y = 0; y < cam::TILE_H; y++) {
                for (std::size_t x = 0; x < cam::TILE_W; x++) {
                    const auto v = static_cast<std::uint8_t>(tile[y * cam::TILE_W + x] / 2);
                    put(t, static_cast<std::int64_t>(x), static_cast<std::int64_t>(y), {v, v, v});
                }
            }
            for (const auto& [bx, by] : frames[k].px[t]) {
                const std::int64_t cx = tr::rs::f64_to_i64(std::round(bx));
                const std::int64_t cy = tr::rs::f64_to_i64(std::round(by));
                for (std::int64_t d = -6; d <= 6; d++) {
                    put(t, cx + d, cy - 6, {255, 40, 40});
                    put(t, cx + d, cy + 6, {255, 40, 40});
                    put(t, cx - 6, cy + d, {255, 40, 40});
                    put(t, cx + 6, cy + d, {255, 40, 40});
                }
            }
            for (const tr::Found& fo : out[k]) {
                const tr::LedModel& model = models[fo.model];
                const g::Pose cfm = cams[t].cam_from_body.compose(fo.joint.body_from_model);
                for (std::size_t li = 0; li < model.leds.size(); li++) {
                    const tr::Led& led = model.leds[li];
                    const g::Vec3 pc = cfm.apply(led.p);
                    const g::Vec3 n = g::mat_vec(cfm.r, led.n);
                    if (pc[2] <= 0.0 || g::dot(n, g::scale(g::normalize(pc), -1.0)) < cos_face) {
                        continue;
                    }
                    const auto uv = cams[t].intr.project(pc);
                    if (!uv) {
                        continue;
                    }
                    bool matched = false;
                    for (const auto& [l, b] : fo.joint.matches[t]) {
                        matched = matched || l == li;
                    }
                    const std::array<std::uint8_t, 3> c =
                        matched ? std::array<std::uint8_t, 3>{40, 255, 40} : std::array<std::uint8_t, 3>{60, 140, 255};
                    const std::int64_t cx = tr::rs::f64_to_i64(std::round(uv->first));
                    const std::int64_t cy = tr::rs::f64_to_i64(std::round(uv->second));
                    for (std::int64_t dy = -3; dy <= 3; dy++) {
                        for (std::int64_t dx = -3; dx <= 3; dx++) {
                            const std::int64_t r2 = dx * dx + dy * dy;
                            const bool ring = r2 >= 5 && r2 <= 9;
                            if ((fo.model == 0 && r2 <= 4) || (fo.model == 1 && ring)) {
                                put(t, cx + dx, cy + dy, c);
                            }
                        }
                    }
                }
            }
        }
        const std::string p = cam::path_join(dir, std::format("rec{:05}.png", i));
        if (auto wr = write_png_rgb(p, w, cam::TILE_H, rgb); !wr) {
            panic("png: " + io_debug(wr.error()));
        }
    }
}

} // namespace

int cmd_track(std::span<const std::string> a) {
    const std::string path = at(a, 0);
    const std::size_t start = tr::arg(a, "--start") ? parse_usize(*tr::arg(a, "--start")) : 0;
    const std::size_t count =
        tr::arg(a, "--count") ? parse_usize(*tr::arg(a, "--count")) : std::numeric_limits<std::size_t>::max();
    const std::size_t every = tr::arg(a, "--every") ? parse_usize(*tr::arg(a, "--every")) : 1;
    const bool use_prior = !has(a, "--no-prior");
    tr::Rig rig = load_rig(a);
    std::vector<tr::Cam> cams = rig.cams;
    const auto& models = rig.models;
    const auto& triples = rig.triples;
    const tr::Params prm = rig.prm;
    const cam::DeviceCalib& calib = rig.calib;

    // pass 1: blobs of every LED frame
    const auto t0 = Clock::now();
    auto r = open_or_panic(path);
    const tr::BlobParams bp;
    std::vector<std::uint8_t> tile(cam::TILE_W * cam::TILE_H);
    std::vector<tr::LedFrame> frames;
    std::size_t i = 0;
    std::size_t nled = 0;
    cam::Record rec;
    while (next_or_panic(*r, rec)) {
        i += 1;
        const auto m = cam::Meta::parse(std::span(rec.data).first(cam::META_LEN));
        if (!m) {
            continue;
        }
        if (m->kind() != cam::FrameKind::Led) {
            continue;
        }
        nled += 1;
        if (nled <= start) {
            continue;
        }
        if (every == 0) {
            panic("attempt to calculate the remainder with a divisor of zero");
        }
        if ((nled - start - 1) % every != 0) {
            continue;
        }
        if (frames.size() >= count) {
            break;
        }
        const cam::TileRows rows = cam::tile_rows(&*m, rec.data, rec.stride);
        tr::LedFrame lf{i, m->ts_us, {}};
        for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
            cam::copy_tile(rec.data, rec.stride, t, rows.y[t], tile, cam::TILE_W);
            std::vector<std::pair<double, double>> px;
            for (const tr::Blob& b : tr::detect(tile, cam::TILE_W, cam::TILE_H, cam::TILE_W, bp)) {
                px.emplace_back(b.x, b.y);
            }
            lf.px.push_back(std::move(px));
        }
        frames.push_back(std::move(lf));
    }
    eprintln(std::format("pass 1: {} LED frames, blobs in {} s", frames.size(), rf::fixed(secs_since(t0), 1)));

    // pass 2: brute-force acquisition per frame, in parallel
    const auto t1 = Clock::now();
    const std::size_t nthreads = available_parallelism();
    std::atomic<std::size_t> next{0};
    std::vector<std::vector<tr::Found>> acq(frames.size());
    {
        std::vector<std::thread> workers;
        for (std::size_t w = 0; w < nthreads; w++) {
            workers.emplace_back([&] {
                for (;;) {
                    const std::size_t k = next.fetch_add(1, std::memory_order_relaxed);
                    if (k >= frames.size()) {
                        break;
                    }
                    acq[k] = tr::solve_frame(frames[k], cams, models, triples, prm, nullptr, {}, false);
                }
            });
        }
        for (std::thread& t : workers) {
            t.join();
        }
    }
    eprintln(std::format("pass 2: acquisition in {} s on {} threads", rf::fixed(secs_since(t1), 1), nthreads));

    // pass 3: temporal pass with priors (tracking): keep the acquisition result unless the prior gives more matches
    std::vector<std::vector<tr::Found>> out = acq;
    if (use_prior) {
        std::vector<tr::Prior> prev(2);
        for (std::size_t k = 0; k < frames.size(); k++) {
            const tr::LedFrame& f = frames[k];
            std::vector<tr::Found> tracked = tr::solve_frame(f, cams, models, triples, prm, &prev, {}, false);
            // fill hands that the prior lost from acquisition
            for (const tr::Found& fa : acq[k]) {
                bool same = false;
                for (const tr::Found& t : tracked) {
                    same = same || tr::same_object(t, fa);
                }
                if (!same) {
                    tracked.push_back(fa);
                }
            }
            if (tracked.size() > 2) {
                tracked.resize(2);
            }
            prev.assign(2, std::nullopt);
            for (std::size_t slot = 0; slot < tracked.size(); slot++) {
                prev[slot] = std::tuple{tracked[slot].model, tracked[slot].joint.body_from_model, f.ts_us};
            }
            out[k] = std::move(tracked);
        }
    }

    // optional: refit the extrinsics of cameras 1..3 (camera 0 fixed) on the controller poses, then re-solve
    if (has(a, "--fit-extrinsics")) {
        std::vector<g::Pose> orig;
        for (const tr::Cam& c : cams) {
            orig.push_back(c.cam_from_body);
        }
        for (int round = 0; round < 4; round++) {
            for (std::size_t c = 1; c < cam::TILE_COUNT; c++) {
                std::vector<std::pair<g::Vec3, std::pair<double, double>>> pts;
                for (std::size_t k = 0; k < frames.size(); k++) {
                    for (const tr::Found& fo : out[k]) {
                        if (fo.joint.cams() < 2) {
                            continue;
                        }
                        const g::Pose cfm = cams[c].cam_from_body.compose(fo.joint.body_from_model);
                        const tr::CamObs ob = tr::CamObs::make(cams[c], frames[k].px[c]);
                        const std::vector<bool> none(ob.px.size(), false);
                        for (const auto& [l, b] : tr::match_pixels(cfm, models[fo.model], cams[c], ob, none,
                                                                   std::cos(rf::to_radians(prm.facing_deg)), 8.0)) {
                            pts.emplace_back(fo.joint.body_from_model.apply(models[fo.model].leds[l].p), ob.px[b]);
                        }
                    }
                }
                if (pts.size() < 30) {
                    eprintln(std::format("extrinsics: cam{}: only {} points, not refitted", c, pts.size()));
                    continue;
                }
                const tr::Cam raw{cams[c].intr, g::Pose{}};
                std::vector<tr::Obs> o;
                for (const auto& [p, px] : pts) {
                    o.push_back(tr::Obs{&raw, p, px});
                }
                const auto [np, rms] = tr::refine(cams[c].cam_from_body, o, false, 30);
                eprintln(std::format("extrinsics round {}: cam{}: {} points, rms {} px", round, c, pts.size(),
                                     rf::fixed(rms, 2)));
                cams[c].cam_from_body = np;
            }
            for (std::size_t k = 0; k < frames.size(); k++) {
                std::vector<tr::CamObs> obs;
                for (std::size_t ci = 0; ci < cams.size() && ci < frames[k].px.size(); ci++) {
                    obs.push_back(tr::CamObs::make(cams[ci], frames[k].px[ci]));
                }
                std::vector<std::vector<bool>> used;
                for (const tr::CamObs& ob : obs) {
                    used.emplace_back(ob.px.size(), false);
                }
                for (tr::Found& fo : out[k]) {
                    auto j = tr::joint_from_prior(fo.joint.body_from_model, models[fo.model], cams, obs, used,
                                                  prm.tol_px * 2.0, prm.facing_deg);
                    if (j) {
                        j = tr::joint_from_prior(j->body_from_model, models[fo.model], cams, obs, used, prm.tol_px,
                                                 prm.facing_deg);
                    }
                    if (j) {
                        tr::claim(used, *j);
                        fo.joint = std::move(*j);
                    }
                }
            }
        }
        if (const auto sp = tr::arg(a, "--save-extrinsics")) {
            tr::extrinsics::CamPoses v;
            for (std::size_t c = 1; c < cam::TILE_COUNT; c++) {
                v.emplace_back(c, cams[c].cam_from_body);
            }
            const std::string note =
                std::format("fitted on controller poses of {} ({} LED frames)", path, frames.size());
            if (auto s = tr::extrinsics::save(*sp, calib.uid, v, note)) {
                println("extrinsics saved to " + *sp);
            } else {
                eprintln(std::format("could not save {}: {}", *sp, s.error()));
            }
        }
        for (std::size_t c = 1; c < cam::TILE_COUNT; c++) {
            // correction about the camera's own centre: cam_from_body' = D * cam_from_body
            const g::Pose d = cams[c].cam_from_body.compose(orig[c].inverse());
            const double ang = g::angle_deg({0.0, 0.0, 1.0}, g::mat_vec(d.r, {0.0, 0.0, 1.0}));
            const g::Vec3 centre_shift = g::sub(cams[c].cam_from_body.inverse().t, orig[c].inverse().t);
            println(std::format("extrinsic correction cam{}: optical axis moved {} deg, rotation {} deg, centre moved "
                                "({} {} {}) mm in the body frame",
                                c, rf::fixed(ang, 2), rf::fixed(rf::to_degrees(g::norm(tr::rotvec(d.r))), 2),
                                rf::fixed(centre_shift[0] * 1e3, 1, true), rf::fixed(centre_shift[1] * 1e3, 1, true),
                                rf::fixed(centre_shift[2] * 1e3, 1, true)));
        }
    }

    // report
    std::FILE* csv = nullptr;
    if (const auto p = tr::arg(a, "--csv")) {
        csv = std::fopen(p->c_str(), "w");
        if (csv == nullptr) {
            panic("csv: " + io_debug(cam::IoError::last_os()));
        }
    }
    std::string csv_buf;
    const auto csv_line = [&](const std::string& s) {
        if (csv != nullptr) {
            csv_buf += s;
            csv_buf += '\n';
        }
    };
    csv_line("record,ts_s,hand,lr_by_position,cams,leds,rms_px,alt_rms_px,alt_leds,cam_spread_mm,x_m,y_m,z_m,qx,qy,qz,qw");
    std::vector<double> rms_all;
    std::vector<double> leds_all;
    std::vector<double> spread;
    std::array<std::size_t, 3> nposes{};
    std::array<std::size_t, 3> acq_n{};
    std::pair<std::size_t, std::size_t> lr_agree{0, 0};
    std::vector<double> steps;
    std::array<std::optional<std::pair<g::Vec3, std::uint64_t>>, 2> last{};
    for (std::size_t k = 0; k < frames.size(); k++) {
        const tr::LedFrame& f = frames[k];
        nposes[std::min<std::size_t>(out[k].size(), 2)] += 1;
        acq_n[std::min<std::size_t>(acq[k].size(), 2)] += 1;
        std::vector<tr::Found> found = out[k];
        std::stable_sort(found.begin(), found.end(), [](const tr::Found& x, const tr::Found& y) {
            return x.joint.body_from_model.t[0] < y.joint.body_from_model.t[0];
        });
        for (std::size_t pi = 0; pi < found.size(); pi++) {
            const tr::Found& fo = found[pi];
            const std::string by_pos = found.size() == 2 ? (pi == 0 ? "L" : "R") : "-";
            const std::string& hand_name = models[fo.model].hand;
            const std::string hand = hand_name.substr(0, 1);
            if (found.size() == 2) {
                lr_agree.second += 1;
                if (hand == by_pos) {
                    lr_agree.first += 1;
                }
            }
            const g::Pose& p = fo.joint.body_from_model;
            const auto q = g::quat_from_mat(p.r);
            rms_all.push_back(fo.joint.rms_px);
            leds_all.push_back(static_cast<double>(fo.joint.total()));
            if (fo.single >= 2) {
                spread.push_back(fo.cam_spread_m * 1000.0);
            }
            const std::size_t slot = by_pos == "R" ? 1 : 0;
            if (found.size() == 2) {
                if (last[slot]) {
                    const auto& [lp, lt] = *last[slot];
                    if (f.ts_us - lt < 50'000) {
                        steps.push_back(g::norm(g::sub(p.t, lp)) * 1000.0);
                    }
                }
                last[slot] = std::pair{p.t, f.ts_us};
            }
            csv_line(std::format(
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", f.rec,
                rf::fixed(static_cast<double>(f.ts_us) * 1e-6, 6), hand_name, by_pos, fo.joint.cams(),
                fo.joint.total(), rf::fixed(fo.joint.rms_px, 3), fo.alt ? rf::fixed(fo.alt->rms_px, 3) : "",
                fo.alt ? std::to_string(fo.alt->total()) : "", rf::fixed(fo.cam_spread_m * 1000.0, 1),
                rf::fixed(p.t[0], 4), rf::fixed(p.t[1], 4), rf::fixed(p.t[2], 4), rf::fixed(q[0], 5),
                rf::fixed(q[1], 5), rf::fixed(q[2], 5), rf::fixed(q[3], 5)));
            if (k % 25 == 0) {
                println(std::format(
                    "rec {:5} t {}  {}({}) cams {} LEDs {:2} rms {} px  pos ({} {} {}) m  alt {}", f.rec,
                    rf::fixed_w(static_cast<double>(f.ts_us) * 1e-6, 8, 3), hand_name, by_pos, fo.joint.cams(),
                    fo.joint.total(), rf::fixed(fo.joint.rms_px, 2), rf::fixed(p.t[0], 3, true),
                    rf::fixed(p.t[1], 3, true), rf::fixed(p.t[2], 3, true),
                    fo.alt ? std::format("{} LEDs {} px", fo.alt->total(), rf::fixed(fo.alt->rms_px, 2))
                           : std::string("-")));
            }
        }
    }
    // per-camera bias: blobs within 15 px of the projected facing LEDs of each final pose
    struct Bias {
        std::vector<double> du;
        std::vector<double> dv;
        std::size_t n = 0;
        std::size_t used = 0;
    };
    std::vector<Bias> bias(cam::TILE_COUNT);
    for (std::size_t k = 0; k < frames.size(); k++) {
        for (const tr::Found& fo : out[k]) {
            for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
                const g::Pose cfm = cams[t].cam_from_body.compose(fo.joint.body_from_model);
                const tr::CamObs ob = tr::CamObs::make(cams[t], frames[k].px[t]);
                const std::vector<bool> none(ob.px.size(), false);
                const tr::Matches m = tr::match_pixels(cfm, models[fo.model], cams[t], ob, none,
                                                       std::cos(rf::to_radians(prm.facing_deg)), 15.0);
                bias[t].used += fo.joint.matches[t].size();
                for (const auto& [l, b] : m) {
                    const auto uv = cams[t].intr.project(cfm.apply(models[fo.model].leds[l].p));
                    bias[t].du.push_back(ob.px[b].first - uv->first);
                    bias[t].dv.push_back(ob.px[b].second - uv->second);
                    bias[t].n += 1;
                }
            }
        }
    }
    for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
        Bias& b = bias[t];
        sort_f64(b.du);
        sort_f64(b.dv);
        const auto med = [](const std::vector<double>& v) {
            return v.empty() ? std::numeric_limits<double>::quiet_NaN() : v[v.size() / 2];
        };
        println(std::format("cam{}: blobs within 15 px of a projected LED {}, used in the joint fit {}, median offset "
                            "blob-proj ({}, {}) px",
                            t, b.n, b.used, rf::fixed(med(b.du), 2, true), rf::fixed(med(b.dv), 2, true)));
    }
    if (const auto dir = tr::arg(a, "--overlay")) {
        const std::size_t oe = tr::arg(a, "--overlay-every") ? parse_usize(*tr::arg(a, "--overlay-every")) : 50;
        overlay(path, *dir, oe, frames, out, cams, models, prm);
    }
    const double n = static_cast<double>(std::max<std::size_t>(frames.size(), 1));
    println(std::format("== summary over {} LED frames", frames.size()));
    println(std::format("controllers found per frame (tracking): 0: {} %  1: {} %  2: {} %",
                        rf::fixed(100.0 * static_cast<double>(nposes[0]) / n, 1),
                        rf::fixed(100.0 * static_cast<double>(nposes[1]) / n, 1),
                        rf::fixed(100.0 * static_cast<double>(nposes[2]) / n, 1)));
    println(std::format("controllers found per frame (acquisition only, no prior): 0: {} %  1: {} %  2: {} %",
                        rf::fixed(100.0 * static_cast<double>(acq_n[0]) / n, 1),
                        rf::fixed(100.0 * static_cast<double>(acq_n[1]) / n, 1),
                        rf::fixed(100.0 * static_cast<double>(acq_n[2]) / n, 1)));
    const double r50 = pct(rms_all, 0.5);
    const double r90 = pct(rms_all, 0.9);
    const double r99 = pct(rms_all, 0.99);
    println(std::format("joint reprojection RMS px: median {}  p90 {}  p99 {}", rf::fixed(r50, 2), rf::fixed(r90, 2),
                        rf::fixed(r99, 2)));
    const double l50 = pct(leds_all, 0.5);
    const double l10 = pct(leds_all, 0.1);
    println(std::format("matched LEDs per pose (all cameras): median {}  p10 {}", rf::fixed(l50, 0),
                        rf::fixed(l10, 0)));
    std::vector<double> spread_c = spread;
    const double s50 = pct(spread_c, 0.5);
    const double s90 = pct(spread, 0.9);
    println(std::format("single-camera solve vs joint pose (seen by >=2 cams): median {} mm  p90 {} mm  (n={})",
                        rf::fixed(s50, 1), rf::fixed(s90, 1), spread.size()));
    std::vector<double> st1 = steps;
    std::vector<double> st2 = steps;
    const double t50 = pct(st1, 0.5);
    const double t90 = pct(st2, 0.9);
    const double t99 = pct(steps, 0.99);
    println(std::format("frame-to-frame step (40 ms): median {} mm  p90 {} mm  p99 {} mm", rf::fixed(t50, 1),
                        rf::fixed(t90, 1), rf::fixed(t99, 1)));
    println(std::format("left/right: shape-fit label agrees with the left/right position in {}/{} poses",
                        lr_agree.first, lr_agree.second));
    if (csv != nullptr) {
        std::fwrite(csv_buf.data(), 1, csv_buf.size(), csv);
        std::fclose(csv);
    }
    return 0;
}

} // namespace pibridge::ctrltool
