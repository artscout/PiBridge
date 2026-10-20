// SPDX-License-Identifier: MIT
//
// pibridge-ctrlbench: the offline measurement harness of the optical controller tracker (docs/research/
// ctrl-tracking-improvements.md). PiBridge's own tool (no Rust twin): it replays the LED frames of a `.pxcam` recording
// through the same per-frame code as `pibridge-ctrltrack live` (blob detection + the shared Tracker), as fast as
// possible, and prints the numbers the improvements are judged by:
//
//   tracked-frame %, gaps, frame-to-frame jumps, pose jitter (second differences of consecutive fixes at low speed,
//   mm / deg RMS), reprojection error and LED count, left/right swaps, CPU ms per LED frame (blobs, tracker);
//   with --mask-every: re-acquisition after a synthetic occlusion of one hand (its blobs removed for D seconds),
//   frames to relock and the error of the first fix against the unmasked run;
//   with --sim K: every LED-frame pixel scaled by K (saturating at 255): a brighter LED exposure (40 us / gain 62
//   is about K = 2.8 against the recordings' 25 / 35);
//   with --fuse: the controller fuser (libpibridge-ctrl fuse.h) on a synthetic controller IMU made from the tracked
//   trajectory, with the optical fixes cut for 0.5 / 1 / 2 / 5 s (error of the shown pose vs time, jump at relock).
//
//   pibridge-ctrlbench FILE.pxcam [--calib XML] [--factory] [--parity] [--blob parity|core] [--thr T] [--sim K]
//                   [--mask-every S --mask-s D] [--fixes OUT.csv] [--leds OUT.csv] [--fuse [--fuse-parity]]
//                   [--json]
//
// `--parity` runs blobs and tracker exactly as the Rust tool (PIBRIDGE_CTRL_PARITY=1 does the same).
#include "pibridge/cam/frame.h"
#include "pibridge/cam/record.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/parity.h"
#include "pibridge/ctrl/track/blob.h"
#include "pibridge/ctrl/track/imutrack.h"
#include "pibridge/ctrl/track/rig.h"
#include "pibridge/ctrl/track/tracker.h"

#include "bench_fuse.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace g = pibridge::cam::geom;
namespace tr = pibridge::ctrl::track;
namespace cam = pibridge::cam;

namespace {

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

bool has(const std::vector<std::string>& a, std::string_view k) { return std::find(a.begin(), a.end(), k) != a.end(); }
std::optional<std::string> argv_of(const std::vector<std::string>& a, std::string_view k) {
    for (std::size_t i = 0; i + 1 < a.size(); i++) {
        if (a[i] == k) {
            return a[i + 1];
        }
    }
    return std::nullopt;
}

struct Frame {
    std::uint64_t ns = 0;
    std::array<std::vector<tr::Blob>, cam::TILE_COUNT> blobs;
    std::array<std::uint16_t, 4> exp{};
    std::array<std::uint16_t, 4> gain{};
};

struct Loaded {
    std::vector<Frame> frames;
    double blob_ms = 0;
};

Loaded load(const std::string& path, const tr::BlobParams& bp, double sim) {
    auto r = cam::Reader::open(path);
    if (!r) {
        std::fprintf(stderr, "open %s: %s\n", path.c_str(), r.error().text().c_str());
        std::exit(1);
    }
    Loaded out;
    std::vector<std::uint8_t> tile(cam::TILE_W * cam::TILE_H);
    std::array<std::uint8_t, 256> lut{};
    for (int v = 0; v < 256; v++) {
        lut[static_cast<std::size_t>(v)] =
            static_cast<std::uint8_t>(std::min(255.0, std::round(static_cast<double>(v) * sim)));
    }
    cam::Record rec;
    for (;;) {
        auto got = (*r)->next_record(rec);
        if (!got || !*got) {
            break;
        }
        const auto m = cam::Meta::parse(std::span(rec.data).first(cam::META_LEN));
        if (!m || m->kind() != cam::FrameKind::Led) {
            continue;
        }
        Frame f;
        f.ns = rec.host_ts_ns;
        f.exp = m->exposure;
        f.gain = m->gain;
        const cam::TileRows rows = cam::tile_rows(&*m, rec.data, rec.stride);
        for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
            cam::copy_tile(rec.data, rec.stride, t, rows.y[t], tile, cam::TILE_W);
            if (sim != 1.0) {
                for (auto& v : tile) {
                    v = lut[v];
                }
            }
            const auto t0 = Clock::now();
            f.blobs[t] = tr::detect(tile, cam::TILE_W, cam::TILE_H, cam::TILE_W, bp);
            out.blob_ms += ms_since(t0);
        }
        out.frames.push_back(std::move(f));
    }
    return out;
}

struct FixOut {
    tr::Fix fix;
};
struct FrameOut {
    std::array<std::optional<tr::Fix>, 2> fix;
    double ms = 0;
};

/// Per-hand masks: (slot, from ns, to ns).
struct Mask {
    std::size_t slot;
    std::uint64_t t0, t1;
};

std::vector<FrameOut> run(const tr::Rig& rig, const std::vector<Frame>& frames, const tr::TrackCfg& cfg,
                          const std::vector<Mask>& masks, const std::vector<FrameOut>* ref,
                          pibridge::ctrlbench::HintSim* hs = nullptr, pibridge::ctrlbench::ImuFeed* imu = nullptr,
                          bool imutrack = false) {
    tr::Tracker tk(cfg);
    tk.device_from_aim = g::rot_x(-52.0);
    tr::ImuTrackCfg icfg;
    if (const char* e = std::getenv("BENCH_SA")) {
        icfg.sigma_a = std::atof(e);
    }
    if (const char* e = std::getenv("BENCH_SP")) {
        const double k = std::atof(e);
        icfg.sigma_p = {icfg.sigma_p[0] * k, icfg.sigma_p[1] * k, icfg.sigma_p[2] * k};
    }
    if (const char* e = std::getenv("BENCH_ACCEL")) {
        icfg.use_accel = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("BENCH_CHI2")) {
        icfg.gate_chi2 = std::atof(e);
    }
    tr::ImuTracker itk(icfg, cfg);
    itk.device_from_aim = g::rot_x(-52.0);
    std::vector<FrameOut> out(frames.size());
    const double cos_face = std::cos(cam::rfmt::to_radians(rig.prm.facing_deg));
    for (std::size_t i = 0; i < frames.size(); i++) {
        const Frame& f = frames[i];
        tr::LedFrame lf{i, 0, {}};
        for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
            std::vector<std::pair<double, double>> px;
            for (const tr::Blob& b : f.blobs[t]) {
                px.emplace_back(b.x, b.y);
            }
            lf.px.push_back(std::move(px));
        }
        // synthetic occlusion: remove the masked hand's blobs (those the unmasked run matched, and any blob near
        // that pose's projected facing LEDs)
        for (const Mask& m : masks) {
            if (ref == nullptr || f.ns < m.t0 || f.ns >= m.t1 || !(*ref)[i].fix[m.slot]) {
                continue;
            }
            const tr::Fix& rf = *(*ref)[i].fix[m.slot];
            const tr::LedModel& model = rig.models[rf.found.model];
            for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
                const g::Pose cfm = rig.cams[t].cam_from_body.compose(rf.found.joint.body_from_model);
                std::vector<std::pair<double, double>> proj;
                for (const tr::Led& led : model.leds) {
                    if (const auto d = tr::facing(cfm, led, cos_face)) {
                        if (const auto uv = rig.cams[t].intr.project(cfm.apply(led.p))) {
                            proj.push_back(*uv);
                        }
                    }
                }
                std::vector<bool> drop(lf.px[t].size(), false);
                for (const auto& [l, b] : rf.found.joint.matches[t]) {
                    drop[b] = true;
                }
                for (std::size_t b = 0; b < lf.px[t].size(); b++) {
                    for (const auto& q : proj) {
                        const double du = q.first - lf.px[t][b].first;
                        const double dv = q.second - lf.px[t][b].second;
                        if (du * du + dv * dv < 25.0 * 25.0) {
                            drop[b] = true;
                        }
                    }
                }
                std::vector<std::pair<double, double>> keep;
                for (std::size_t b = 0; b < lf.px[t].size(); b++) {
                    if (!drop[b]) {
                        keep.push_back(lf.px[t][b]);
                    }
                }
                lf.px[t] = std::move(keep);
            }
        }
        if (imu != nullptr) {
            // what the tracker would read from the hint file by now (the frame is processed about 20 ms later)
            const auto got = imu->advance(f.ns + 20'000'000);
            for (std::size_t s = 0; s < 2; s++) {
                for (const auto& x : got[s]) {
                    tr::ImuSample is;
                    is.t = x.t;
                    is.qg = tr::quat_to_mat(x.qg);
                    is.up_aim = x.up;
                    is.up_ok = x.up_ok;
                    is.still_s = x.still_s;
                    const g::Mat3 rwh = tr::quat_to_mat(x.head_q);
                    is.up_body = g::mat_vec(g::transpose(rwh), {0.0, 1.0, 0.0});
                    tk.imu_push(s, is);
                    tr::ImuIn ii;
                    ii.t = x.t;
                    ii.qg = is.qg;
                    ii.f_aim = x.f_aim;
                    ii.up_aim = x.up;
                    ii.up_ok = x.up_ok;
                    ii.still_s = x.still_s;
                    ii.world_from_head = g::Pose{rwh, x.head_p};
                    itk.imu_push(s, ii);
                }
            }
        }
        if (hs != nullptr) {
            const auto h = hs->at(f.ns);
            for (std::size_t s = 0; s < 2; s++) {
                tk.hint[s] = std::nullopt;
                if (h[s].valid) {
                    tk.hint[s] = tr::Hint{h[s].body_from_model, h[s].fix_age_s, h[s].state};
                }
            }
        }
        const auto t0 = Clock::now();
        auto fixes = imutrack ? itk.step(rig, lf, f.ns) : tk.step(rig, lf, f.ns);
        out[i].ms = ms_since(t0);
        for (auto& fx : fixes) {
            if (hs != nullptr) {
                if (std::getenv("BENCH_HDBG") != nullptr && tk.hint[fx.slot]) {
                    const auto& hp = tk.hint[fx.slot]->body_from_model;
                    const auto& fp = fx.found.joint.body_from_model;
                    std::fprintf(stderr, "hdbg frame %zu slot %zu leds %zu few %d: hint-fix angle %.1f deg, dist %.1f mm, fix age %.3f state %u\n",
                                 i, fx.slot, fx.found.joint.total(), fx.few ? 1 : 0,
                                 g::norm(tr::rotvec(g::mat_mul(fp.r, g::transpose(hp.r)))) * 180.0 / M_PI,
                                 g::norm(g::sub(fp.t, hp.t)) * 1000.0, tk.hint[fx.slot]->fix_age_s, tk.hint[fx.slot]->state);
                }
                hs->fix(fx.slot, f.ns, fx.found.joint.body_from_model, fx.found.joint.total(), fx.found.joint.rms_px);
            }
            out[i].fix[fx.slot] = std::move(fx);
        }
    }
    if (std::getenv("BENCH_STATS") != nullptr && imutrack) {
        const auto& c = itk.counts;
        std::fprintf(stderr, "imutrack: updates %llu misses %llu gate rejects %llu acquisitions %llu refused %llu "
                             "tentative dropped %llu fallback frames %llu searches %llu | misses: window %llu refined %llu rms %llu\n",
                     (unsigned long long)c.updates, (unsigned long long)c.misses, (unsigned long long)c.gate_rejects,
                     (unsigned long long)c.acquisitions, (unsigned long long)c.acq_refused,
                     (unsigned long long)c.tentative_dropped, (unsigned long long)c.fallback_frames,
                     (unsigned long long)itk.stats().searches, (unsigned long long)c.miss_window,
                     (unsigned long long)c.miss_refined, (unsigned long long)c.miss_rms);
    }
    if (std::getenv("BENCH_STATS") != nullptr) {
        const tr::Stats& s = tk.stats;
        std::fprintf(stderr, "stats: tries %llu searches %llu unassigned %llu collapsed %llu resets %llu far %llu refit %llu "
                             "dropped %llu\n",
                     (unsigned long long)s.tries, (unsigned long long)s.searches, (unsigned long long)s.unassigned,
                     (unsigned long long)s.collapsed, (unsigned long long)s.init_resets,
                     (unsigned long long)s.far_refused, (unsigned long long)s.model_refit,
                     (unsigned long long)s.model_dropped);
        std::fprintf(stderr, "stats: hint relocks %llu hint refused %llu imu refused %llu\n",
                     (unsigned long long)s.hint_relocks, (unsigned long long)s.hint_refused,
                     (unsigned long long)s.imu_refused);
    }
    return out;
}

double pctl(std::vector<double> v, double q) {
    if (v.empty()) {
        return NAN;
    }
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(std::round(static_cast<double>(v.size() - 1) * q))];
}
double mean(const std::vector<double>& v) {
    double s = 0;
    for (double x : v) {
        s += x;
    }
    return v.empty() ? NAN : s / static_cast<double>(v.size());
}
double rms(const std::vector<double>& v) {
    double s = 0;
    for (double x : v) {
        s += x * x;
    }
    return v.empty() ? NAN : std::sqrt(s / static_cast<double>(v.size()));
}

g::Vec3 rotvec_between(const g::Mat3& a, const g::Mat3& b) { return tr::rotvec(g::mat_mul(g::transpose(a), b)); }

struct Metrics {
    std::array<double, 2> tracked_pct{};
    std::array<double, 2> few_pct{};
    std::array<int, 2> gaps_short{}, gaps_long{};
    int jumps = 0;
    double step_p95_mm = 0;
    std::array<double, 2> jit_mm{}, jit_deg{};
    std::array<int, 2> jit_n{};
    double rms_px_mean = 0, leds_mean = 0;
    int swaps = 0, crossed = 0;
    double ms_mean = 0, ms_p95 = 0;
    /// Fixes with at most 7 LEDs against the interpolation of the slot's 8+ LED fixes within 100 ms on both sides.
    double interp_p50_mm = 0, interp_p90_mm = 0;
    int interp_n = 0, interp_bad = 0;
};

Metrics metrics(const std::vector<Frame>& frames, const std::vector<FrameOut>& out) {
    Metrics m;
    const double n = static_cast<double>(std::max<std::size_t>(frames.size(), 1));
    std::vector<double> steps, rmsv, ledv, msv;
    for (std::size_t s = 0; s < 2; s++) {
        int tracked = 0, few = 0, gap = 0;
        bool seen = false;
        std::vector<double> jp, jr;
        for (std::size_t i = 0; i < frames.size(); i++) {
            const auto& fx = out[i].fix[s];
            if (fx) {
                tracked++;
                few += fx->few ? 1 : 0;
                rmsv.push_back(fx->found.joint.rms_px);
                ledv.push_back(static_cast<double>(fx->found.joint.total()));
                if (seen && gap > 0) {
                    (gap <= 2 ? m.gaps_short : m.gaps_long)[s]++;
                }
                gap = 0;
                seen = true;
                if (i > 0 && out[i - 1].fix[s]) {
                    const double d = g::norm(g::sub(fx->found.joint.body_from_model.t,
                                                    out[i - 1].fix[s]->found.joint.body_from_model.t));
                    steps.push_back(d * 1000.0);
                    if (d > 0.08) {
                        m.jumps++;
                    }
                }
                // jitter: second difference of three consecutive fixes, at low speed (the noise; smooth motion
                // cancels to first order). sigma = rms(d2) / sqrt(1.5) for white noise.
                if (i >= 2 && out[i - 1].fix[s] && out[i - 2].fix[s]) {
                    const g::Pose& a = out[i - 2].fix[s]->found.joint.body_from_model;
                    const g::Pose& b = out[i - 1].fix[s]->found.joint.body_from_model;
                    const g::Pose& c = fx->found.joint.body_from_model;
                    const double dt = static_cast<double>(frames[i].ns - frames[i - 2].ns) * 1e-9;
                    const double speed = g::norm(g::sub(c.t, a.t)) / std::max(dt, 1e-3);
                    if (speed < 0.15 && dt < 0.1) {
                        const g::Vec3 d2 = g::sub(g::add(a.t, c.t), g::scale(b.t, 2.0));
                        jp.push_back(g::norm(d2) / std::sqrt(1.5) * 1000.0 / std::sqrt(3.0));
                        const g::Vec3 r1 = rotvec_between(a.r, b.r);
                        const g::Vec3 r2 = rotvec_between(b.r, c.r);
                        jr.push_back(g::norm(g::sub(r2, r1)) / std::sqrt(1.5) * 180.0 / M_PI / std::sqrt(3.0));
                    }
                }
                // a swap: this slot's fix lands where the other hand just was, far from its own last fix
                const std::size_t o = 1 - s;
                if (i > 0 && out[i - 1].fix[o] && out[i - 1].fix[s]) {
                    const g::Vec3 p = fx->found.joint.body_from_model.t;
                    if (g::norm(g::sub(p, out[i - 1].fix[o]->found.joint.body_from_model.t)) < 0.06 &&
                        g::norm(g::sub(p, out[i - 1].fix[s]->found.joint.body_from_model.t)) > 0.15) {
                        m.swaps++;
                    }
                }
                if (s == 0 && out[i].fix[1] &&
                    fx->found.joint.body_from_model.t[0] > out[i].fix[1]->found.joint.body_from_model.t[0] + 0.10) {
                    m.crossed++;
                }
            } else if (seen) {
                gap++;
            }
        }
        m.tracked_pct[s] = 100.0 * tracked / n;
        m.few_pct[s] = tracked ? 100.0 * few / tracked : 0;
        m.jit_mm[s] = rms(jp);
        m.jit_deg[s] = rms(jr);
        m.jit_n[s] = static_cast<int>(jp.size());
    }
    for (const auto& o : out) {
        msv.push_back(o.ms);
    }
    std::vector<double> ie;
    for (std::size_t s = 0; s < 2; s++) {
        for (std::size_t i = 0; i < frames.size(); i++) {
            const auto& fx = out[i].fix[s];
            if (!fx || fx->found.joint.total() >= 8) {
                continue;
            }
            std::optional<std::size_t> a, b;
            for (std::size_t k = i; k-- > 0 && frames[i].ns - frames[k].ns < 100'000'000;) {
                if (out[k].fix[s] && out[k].fix[s]->found.joint.total() >= 8) {
                    a = k;
                    break;
                }
            }
            for (std::size_t k = i + 1; k < frames.size() && frames[k].ns - frames[i].ns < 100'000'000; k++) {
                if (out[k].fix[s] && out[k].fix[s]->found.joint.total() >= 8) {
                    b = k;
                    break;
                }
            }
            if (!a || !b) {
                continue;
            }
            const double f = static_cast<double>(frames[i].ns - frames[*a].ns) /
                             static_cast<double>(frames[*b].ns - frames[*a].ns);
            const g::Vec3 pa = out[*a].fix[s]->found.joint.body_from_model.t;
            const g::Vec3 pb = out[*b].fix[s]->found.joint.body_from_model.t;
            const g::Vec3 pi = g::add(pa, g::scale(g::sub(pb, pa), f));
            const double e = g::norm(g::sub(fx->found.joint.body_from_model.t, pi)) * 1000.0;
            ie.push_back(e);
            m.interp_bad += e > 50.0;
        }
    }
    m.interp_n = static_cast<int>(ie.size());
    m.interp_p50_mm = pctl(ie, 0.5);
    m.interp_p90_mm = pctl(ie, 0.9);
    m.step_p95_mm = pctl(steps, 0.95);
    m.rms_px_mean = mean(rmsv);
    m.leds_mean = mean(ledv);
    m.ms_mean = mean(msv);
    m.ms_p95 = pctl(msv, 0.95);
    return m;
}

void print_metrics(const char* label, const Metrics& m, double blob_ms) {
    std::printf("%s: tracked L %.1f %% R %.1f %% (few-LED %.0f / %.0f %%) | gaps 1-2: %d/%d, 3+: %d/%d | jumps>8cm %d, "
                "step p95 %.1f mm | jitter L %.2f mm %.3f deg (n %d), R %.2f mm %.3f deg (n %d) | rms %.2f px, LEDs "
                "%.1f | swaps %d, crossed %d | 5-7 LED fixes vs 8+ LED interpolation p50 %.1f p90 %.1f mm, >5 cm %d of %d | "
                "ms/frame blobs %.2f tracker %.2f (p95 %.2f)\n",
                label, m.tracked_pct[0], m.tracked_pct[1], m.few_pct[0], m.few_pct[1], m.gaps_short[0],
                m.gaps_short[1], m.gaps_long[0], m.gaps_long[1], m.jumps, m.step_p95_mm, m.jit_mm[0], m.jit_deg[0],
                m.jit_n[0], m.jit_mm[1], m.jit_deg[1], m.jit_n[1], m.rms_px_mean, m.leds_mean, m.swaps, m.crossed,
                m.interp_p50_mm, m.interp_p90_mm, m.interp_bad, m.interp_n, blob_ms, m.ms_mean, m.ms_p95);
}

/// Hand identity and stability against the controllers' own IMU (pibridge-ctrlbench --imu): per pair of consecutive
/// fixes of a hand (at most 100 ms apart, turning at least 5 deg by the gyro or the optics), the optical turn against
/// that hand's gyro and against the other hand's: "own" (within 15 deg of its own gyro), "wrong hand" (more than 25
/// from its own, within 15 of the other's), "neither" (more than 25 from both: a mirror or garbage fit). Also: two
/// hands on one controller (fixes closer than 6 cm in a frame), jumps over 5 cm between consecutive fixes, position
/// spread while the hand's gyro is quiet, and the tracked share of each 10 s third of the take.
void eval_imu(const std::vector<Frame>& frames, const std::vector<FrameOut>& out,
              const std::array<std::vector<pibridge::ctrlbench::ImuFeed::Sample>, 2>& imu, const char* label) {
    const g::Mat3 D = g::rot_x(-52.0);
    const auto near = [&](std::size_t s, std::uint64_t t) -> const pibridge::ctrlbench::ImuFeed::Sample* {
        const auto& v = imu[s];
        const auto it = std::lower_bound(v.begin(), v.end(), t, [](const auto& e, std::uint64_t x) { return e.t < x; });
        if (it == v.end() || it->t - t > 15'000'000) {
            return nullptr;
        }
        return &*it;
    };
    int own = 0, wrong = 0, neither = 0, flat = 0, same = 0, jumps = 0;
    std::array<std::vector<double>, 2> rest_spread;
    for (std::size_t s = 0; s < 2; s++) {
        std::vector<g::Vec3> still_run;
        for (std::size_t i = 1; i < frames.size(); i++) {
            const auto& a = out[i - 1].fix[s];
            const auto& b = out[i].fix[s];
            if (b && out[i].fix[1 - s] && s == 0 &&
                g::norm(g::sub(b->found.joint.body_from_model.t, out[i].fix[1]->found.joint.body_from_model.t)) < 0.06) {
                same++;
            }
            // position spread while the gyro is quiet (0.5 s or more): the jitter at rest
            const auto* sn = near(s, frames[i].ns);
            if (b && sn != nullptr && sn->still_s > 0.5) {
                still_run.push_back(b->found.joint.body_from_model.t);
            } else if (still_run.size() >= 8) {
                g::Vec3 m{0, 0, 0};
                for (const auto& p : still_run) {
                    m = g::add(m, p);
                }
                m = g::scale(m, 1.0 / static_cast<double>(still_run.size()));
                double ss = 0;
                for (const auto& p : still_run) {
                    ss += std::pow(g::norm(g::sub(p, m)), 2);
                }
                rest_spread[s].push_back(std::sqrt(ss / static_cast<double>(still_run.size())) * 1000.0);
                still_run.clear();
            } else {
                still_run.clear();
            }
            if (!a || !b || frames[i].ns - frames[i - 1].ns > 100'000'000) {
                continue;
            }
            if (g::norm(g::sub(b->found.joint.body_from_model.t, a->found.joint.body_from_model.t)) > 0.05) {
                jumps++;
            }
            const auto* g0 = near(s, frames[i - 1].ns);
            const auto* g1 = near(s, frames[i].ns);
            const auto* o0 = near(1 - s, frames[i - 1].ns);
            const auto* o1 = near(1 - s, frames[i].ns);
            if (g0 == nullptr || g1 == nullptr) {
                continue;
            }
            const g::Mat3 opt = g::mat_mul(g::transpose(a->found.joint.body_from_model.r), b->found.joint.body_from_model.r);
            const auto err = [&](const auto* x0, const auto* x1) {
                const g::Mat3 dg = g::mat_mul(g::transpose(tr::quat_to_mat(x0->qg)), tr::quat_to_mat(x1->qg));
                const g::Mat3 pred = g::mat_mul(g::mat_mul(D, dg), g::transpose(D));
                return std::pair{g::norm(tr::rotvec(g::mat_mul(g::transpose(pred), opt))) * 180.0 / M_PI,
                                 g::norm(tr::rotvec(pred)) * 180.0 / M_PI};
            };
            const auto [e_own, turn_own] = err(g0, g1);
            if (std::getenv("BENCH_EOWN") != nullptr && a->found.joint.total() >= 9 && b->found.joint.total() >= 9) {
                std::fprintf(stderr, "eown %zu %.1f turn %.1f opt %.1f dt %.0f\n", s, e_own, turn_own,
                             g::norm(tr::rotvec(opt)) * 180.0 / M_PI,
                             static_cast<double>(frames[i].ns - frames[i - 1].ns) * 1e-6);
            }
            const double turn_opt = g::norm(tr::rotvec(opt)) * 180.0 / M_PI;
            if (turn_own < 5.0 && turn_opt < 5.0) {
                flat++;
                continue;
            }
            const double e_other = (o0 != nullptr && o1 != nullptr) ? err(o0, o1).first : 1e9;
            if (e_own <= 15.0) {
                own++;
            } else if (e_own > 25.0 && e_other <= 15.0) {
                wrong++;
            } else if (e_own > 25.0) {
                neither++;
            }
        }
    }
    std::array<std::array<double, 3>, 2> thirds{};
    const std::uint64_t t0 = frames.front().ns;
    std::array<int, 3> nfr{};
    for (std::size_t i = 0; i < frames.size(); i++) {
        const std::size_t k = std::min<std::size_t>(2, (frames[i].ns - t0) / 10'000'000'000ull);
        nfr[k]++;
        for (std::size_t s = 0; s < 2; s++) {
            thirds[s][k] += out[i].fix[s] ? 1 : 0;
        }
    }
    const auto med = [](std::vector<double> v) { return pctl(std::move(v), 0.5); };
    std::printf("%s IMU check: turning pairs own %d, WRONG HAND %d, neither (mirror/garbage) %d, not turning %d | both "
                "hands on one controller %d frames | jumps>5cm %d | spread at rest (median of quiet runs) L %.1f R %.1f "
                "mm | tracked L/R 0-10 s %.0f/%.0f, 10-20 s %.0f/%.0f, 20-30 s %.0f/%.0f %%\n",
                label, own, wrong, neither, flat, same, jumps, med(rest_spread[0]), med(rest_spread[1]),
                100.0 * thirds[0][0] / std::max(nfr[0], 1), 100.0 * thirds[1][0] / std::max(nfr[0], 1),
                100.0 * thirds[0][1] / std::max(nfr[1], 1), 100.0 * thirds[1][1] / std::max(nfr[1], 1),
                100.0 * thirds[0][2] / std::max(nfr[2], 1), 100.0 * thirds[1][2] / std::max(nfr[2], 1));
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> a(argv + 1, argv + argc);
    if (!a.empty() && a[0] == "--imu-log") {
        // pibridge-ctrlbench --imu-log LOG.csv [--calib-json ctrl-imu.json] [--fuse-parity]: no recording needed
        if (a.size() < 2) {
            std::fprintf(stderr, "usage: pibridge-ctrlbench --imu-log LOG.csv [--calib-json JSON] [--fuse-parity]\n");
            return 2;
        }
        return pibridge::ctrlbench::imu_log_bench(a[1], argv_of(a, "--calib-json").value_or(""), has(a, "--fuse-parity"));
    }
    if (!a.empty() && a[0] == "--selftest-imu-log") {
        return pibridge::ctrlbench::imu_log_selftest(a.size() > 1 ? a[1] : std::string("/tmp"));
    }
    if (a.empty() || a[0].starts_with("-")) {
        std::fprintf(stderr, "usage: pibridge-ctrlbench FILE.pxcam [--calib XML] [--factory] [--parity] [--blob parity|core] "
                             "[--thr T] [--sim K] [--mask-every S --mask-s D] [--fixes OUT] [--leds OUT] [--fuse]\n");
        return 2;
    }
    const std::string path = a[0];
    std::vector<std::string> rig_args(a.begin() + 1, a.end());
    auto loaded_rig = tr::Rig::try_load(rig_args);
    if (!loaded_rig) {
        std::fprintf(stderr, "rig: %s\n", loaded_rig.error().c_str());
        return 2;
    }
    const tr::Rig rig = std::move(*loaded_rig);
    const bool parity = has(a, "--parity") || pibridge::ctrl::parity_mode();
    tr::BlobParams bp = tr::BlobParams::live();
    if (!parity) {
        bp = tr::BlobParams::improved();
    }
    if (const auto v = argv_of(a, "--blob")) {
        bp.core = *v == "core";
    }
    if (const auto v = argv_of(a, "--thr")) {
        bp.threshold = static_cast<std::uint8_t>(std::stoi(*v));
    }
    if (const auto v = argv_of(a, "--core-frac")) {
        bp.core_frac = std::stod(*v);
    }
    const double sim = argv_of(a, "--sim") ? std::stod(*argv_of(a, "--sim")) : 1.0;
    tr::TrackCfg cfg = parity ? tr::TrackCfg{} : tr::TrackCfg::improved();
    if (const auto v = argv_of(a, "--track")) {
        cfg = *v == "parity" ? tr::TrackCfg{} : tr::TrackCfg::improved();
    }
    const auto t_load = Clock::now();
    Loaded L = load(path, bp, sim);
    const double nfr = static_cast<double>(std::max<std::size_t>(L.frames.size(), 1));
    std::fprintf(stderr, "%zu LED frames (exposure %u gain %u in the first) loaded in %.1f s\n", L.frames.size(),
                 L.frames.empty() ? 0u : L.frames[0].exp[0], L.frames.empty() ? 0u : L.frames[0].gain[0],
                 ms_since(t_load) / 1000.0);
    {
        std::vector<double> nb, peak, area;
        int sat = 0, tot = 0;
        for (const Frame& f : L.frames) {
            std::size_t k = 0;
            for (const auto& bs : f.blobs) {
                k += bs.size();
                for (const auto& b : bs) {
                    peak.push_back(b.peak);
                    area.push_back(b.area);
                    sat += b.peak == 255;
                    tot++;
                }
            }
            nb.push_back(static_cast<double>(k));
        }
        std::printf("blobs: per frame mean %.1f p95 %.0f | area p50 %.0f p95 %.0f | peak p50 %.0f | saturated %.0f %%\n",
                    mean(nb), pctl(nb, 0.95), pctl(area, 0.5), pctl(area, 0.95), pctl(peak, 0.5),
                    tot ? 100.0 * sat / tot : 0.0);
    }
    std::unique_ptr<pibridge::ctrlbench::ImuFeed> feed;
    if (const auto v = argv_of(a, "--imu")) {
        const auto comma = v->find(',');
        feed = std::make_unique<pibridge::ctrlbench::ImuFeed>(v->substr(0, comma),
                                                           comma == std::string::npos ? "" : v->substr(comma + 1),
                                                           argv_of(a, "--calib-json").value_or(""));
        if (!feed->ok()) {
            return 1;
        }
        if (const auto e = argv_of(a, "--imu-id")) {
            cfg.imu_id = *e == "1";
        }
        if (const auto e = argv_of(a, "--imu-rot")) {
            cfg.imu_rot_deg = std::stod(*e);
        }
        if (const auto e = argv_of(a, "--imu-tilt")) {
            cfg.imu_tilt_deg = std::stod(*e);
        }
        if (const auto e = argv_of(a, "--imu-rest")) {
            cfg.imu_rest_s = std::stod(*e);
        }
    }
    const bool imutrack = has(a, "--imutrack");
    auto ref = run(rig, L.frames, cfg, {}, nullptr, nullptr, feed.get(), imutrack);
    if (imutrack && !has(a, "--measured")) {
        // the metrics on what is published (the filter's pose); --measured: on the optical fits
        for (auto& fo : ref) {
            for (auto& fx : fo.fix) {
                if (fx && fx->published) {
                    fx->found.joint.body_from_model = *fx->published;
                }
            }
        }
    }
    const Metrics m = metrics(L.frames, ref);
    print_metrics(parity ? "parity" : "improved", m, L.blob_ms / nfr);
    if (const auto v = argv_of(a, "--imu")) {
        const auto comma = v->find(',');
        pibridge::ctrlbench::ImuFeed all(v->substr(0, comma), comma == std::string::npos ? "" : v->substr(comma + 1),
                                      argv_of(a, "--calib-json").value_or(""));
        eval_imu(L.frames, ref, all.advance(~0ull),
                 imutrack ? "imutrack" : parity ? "parity" : cfg.imu_id ? "imu-id" : "improved");
    }

    if (has(a, "--hint")) {
        // closed loop with the fuser on a synthetic IMU (bench_fuse.cpp): the same tracker with the hint used
        std::vector<pibridge::ctrlbench::FixIn> refix;
        for (std::size_t i = 0; i < ref.size(); i++) {
            for (const auto& fx : ref[i].fix) {
                if (fx) {
                    refix.push_back({fx->slot, L.frames[i].ns, fx->found.joint.body_from_model, fx->found.joint.total(),
                                     fx->found.joint.rms_px});
                }
            }
        }
        const unsigned long seed = argv_of(a, "--seed") ? std::stoul(*argv_of(a, "--seed")) : 1;
        const double berr = argv_of(a, "--bias-err") ? std::stod(*argv_of(a, "--bias-err")) : 0.3;
        tr::TrackCfg hc = cfg;
        hc.use_hint = true;
        if (const auto v = argv_of(a, "--hint-few")) {
            hc.hint_few_leds = std::stoul(*v);
        }
        if (const auto v = argv_of(a, "--hint-rot-w")) {
            hc.hint_rot_w = std::stod(*v);
        }
        if (const auto v = argv_of(a, "--hint-lr")) {
            hc.hint_lr_deg = std::stod(*v);
        }
        if (const auto v = argv_of(a, "--hint-stale")) {
            hc.hint_stale_s = std::stod(*v);
        }
        if (const auto v = argv_of(a, "--hint-age")) {
            hc.hint_max_age_s = std::stod(*v);
        }
        tr::TrackCfg nc = cfg;
        nc.use_hint = false;
        pibridge::ctrlbench::HintSim truth(refix, seed, berr);
        const auto few_err = [&](const std::vector<FrameOut>& o) {
            std::vector<double> e;
            std::vector<double> e_all;
            for (std::size_t i = 0; i < o.size(); i++) {
                for (const auto& fx : o[i].fix) {
                    if (!fx) {
                        continue;
                    }
                    if (const auto tp = truth.truth(fx->slot, L.frames[i].ns)) {
                        const double d = g::norm(g::sub(fx->found.joint.body_from_model.t, *tp)) * 1000.0;
                        e_all.push_back(d);
                        if (fx->found.joint.total() <= 4) {
                            e.push_back(d);
                        }
                    }
                }
            }
            return std::tuple{e.size(), pctl(e, 0.5), pctl(e, 0.9), pctl(e_all, 0.5), pctl(e_all, 0.9)};
        };
        for (int pass = 0; pass < 2; pass++) {
            pibridge::ctrlbench::HintSim hs(refix, seed, berr);
            const auto got = run(rig, L.frames, pass == 0 ? nc : hc, {}, nullptr, &hs);
            const Metrics mh = metrics(L.frames, got);
            print_metrics(pass == 0 ? "closed loop, hint off" : "closed loop, hint on", mh, L.blob_ms / nfr);
            const auto [n, p50, p90, a50, a90] = few_err(got);
            std::printf("  vs truth: fixes with <= 4 LEDs %zu, error p50 %.1f p90 %.1f mm; all fixes p50 %.1f p90 %.1f mm\n",
                        n, p50, p90, a50, a90);
            if (const auto every = argv_of(a, "--mask-every")) {
                const double S = std::stod(*every);
                const double D = argv_of(a, "--mask-s") ? std::stod(*argv_of(a, "--mask-s")) : 0.5;
                std::vector<Mask> masks;
                const std::uint64_t t_first = L.frames.front().ns;
                std::size_t k = 0;
                for (double t = 1.0; t_first + static_cast<std::uint64_t>((t + D + 0.5) * 1e9) < L.frames.back().ns;
                     t += S, k++) {
                    masks.push_back(Mask{k % 2, t_first + static_cast<std::uint64_t>(t * 1e9),
                                         t_first + static_cast<std::uint64_t>((t + D) * 1e9)});
                }
                pibridge::ctrlbench::HintSim hs2(refix, seed, berr);
                const auto gm = run(rig, L.frames, pass == 0 ? nc : hc, masks, &ref, &hs2);
                std::vector<double> lat;
                int never = 0, eligible = 0, wrong = 0;
                for (const Mask& mk : masks) {
                    std::size_t i0 = 0;
                    while (i0 < L.frames.size() && L.frames[i0].ns < mk.t1) {
                        i0++;
                    }
                    if (i0 >= L.frames.size() || !ref[i0].fix[mk.slot]) {
                        continue;
                    }
                    eligible++;
                    bool done = false;
                    for (std::size_t i = i0; i < L.frames.size() && i < i0 + 60; i++) {
                        if (gm[i].fix[mk.slot]) {
                            lat.push_back(static_cast<double>(L.frames[i].ns - L.frames[i0].ns) * 1e-6);
                            if (const auto tp = truth.truth(mk.slot, L.frames[i].ns)) {
                                wrong += g::norm(g::sub(gm[i].fix[mk.slot]->found.joint.body_from_model.t, *tp)) > 0.1;
                            }
                            done = true;
                            break;
                        }
                    }
                    never += done ? 0 : 1;
                }
                const Metrics mm = metrics(L.frames, gm);
                std::printf("  masks %zu of %.1f s: eligible %d, relock ms p50 %.0f p95 %.0f max %.0f, not relocked %d, "
                            "wrong first fix %d | tracked L %.1f R %.1f %%, swaps %d, jumps %d, few-LED %.0f/%.0f %%\n",
                            masks.size(), D, eligible, pctl(lat, 0.5), pctl(lat, 0.95), pctl(lat, 1.0), never, wrong,
                            mm.tracked_pct[0], mm.tracked_pct[1], mm.swaps, mm.jumps, mm.few_pct[0], mm.few_pct[1]);
            }
        }
    }
    if (const auto p = argv_of(a, "--fixes")) {
        std::FILE* f = std::fopen(p->c_str(), "w");
        std::fprintf(f, "frame,ns,slot,x,y,z,qx,qy,qz,qw,leds,cams,rms,few,model\n");
        for (std::size_t i = 0; i < ref.size(); i++) {
            for (const auto& fx : ref[i].fix) {
                if (!fx) {
                    continue;
                }
                const auto& j = fx->found.joint;
                const auto q = g::quat_from_mat(j.body_from_model.r);
                std::fprintf(f, "%zu,%llu,%zu,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%zu,%zu,%.3f,%d,%zu\n", i,
                             static_cast<unsigned long long>(L.frames[i].ns), fx->slot, j.body_from_model.t[0],
                             j.body_from_model.t[1], j.body_from_model.t[2], q[0], q[1], q[2], q[3], j.total(),
                             j.cams(), j.rms_px, fx->few ? 1 : 0, fx->found.model);
            }
        }
        std::fclose(f);
    }
    if (const auto p = argv_of(a, "--leds")) {
        // per matched LED: its blob's brightness and size (LED modulation / identity study)
        std::FILE* f = std::fopen(p->c_str(), "w");
        std::fprintf(f, "frame,ns,dev_frame,slot,model,led,cam,peak,area,w,h,u,v,dist_m\n");
        for (std::size_t i = 0; i < ref.size(); i++) {
            for (const auto& fx : ref[i].fix) {
                if (!fx) {
                    continue;
                }
                const auto& j = fx->found.joint;
                for (std::size_t c = 0; c < j.matches.size(); c++) {
                    for (const auto& [l, b] : j.matches[c]) {
                        const tr::Blob& bl = L.frames[i].blobs[c][b];
                        const g::Vec3 pc = rig.cams[c].cam_from_body.compose(j.body_from_model).apply(
                            rig.models[fx->found.model].leds[l].p);
                        std::fprintf(f, "%zu,%llu,%zu,%zu,%zu,%zu,%zu,%u,%u,%u,%u,%.2f,%.2f,%.3f\n", i,
                                     static_cast<unsigned long long>(L.frames[i].ns), i, fx->slot, fx->found.model, l,
                                     c, bl.peak, bl.area, bl.w, bl.h, bl.x, bl.y, g::norm(pc));
                    }
                }
            }
        }
        std::fclose(f);
    }
    if (const auto every = argv_of(a, "--mask-every")) {
        // synthetic occlusions of one hand at a time, alternating hands, D seconds every S seconds
        const double S = std::stod(*every);
        const double D = argv_of(a, "--mask-s") ? std::stod(*argv_of(a, "--mask-s")) : 0.5;
        std::vector<Mask> masks;
        const std::uint64_t t_first = L.frames.front().ns;
        const std::uint64_t t_last = L.frames.back().ns;
        std::size_t k = 0;
        for (double t = 1.0; t_first + static_cast<std::uint64_t>((t + D + 0.5) * 1e9) < t_last; t += S, k++) {
            masks.push_back(Mask{k % 2, t_first + static_cast<std::uint64_t>(t * 1e9),
                                 t_first + static_cast<std::uint64_t>((t + D) * 1e9)});
        }
        const auto got = run(rig, L.frames, cfg, masks, &ref);
        std::vector<double> lat_ms, lat_fr, err_mm;
        int relocked = 0, never = 0, wrong = 0, eligible = 0;
        for (const Mask& mk : masks) {
            // the first frame at or after the mask end where the unmasked run tracks the hand
            std::size_t i0 = 0;
            while (i0 < L.frames.size() && L.frames[i0].ns < mk.t1) {
                i0++;
            }
            if (i0 >= L.frames.size() || !ref[i0].fix[mk.slot]) {
                continue;
            }
            eligible++;
            bool done = false;
            for (std::size_t i = i0; i < L.frames.size() && i < i0 + 60; i++) {
                if (got[i].fix[mk.slot]) {
                    relocked++;
                    lat_fr.push_back(static_cast<double>(i - i0));
                    lat_ms.push_back(static_cast<double>(L.frames[i].ns - L.frames[i0].ns) * 1e-6);
                    if (ref[i].fix[mk.slot]) {
                        const double e = g::norm(g::sub(got[i].fix[mk.slot]->found.joint.body_from_model.t,
                                                        ref[i].fix[mk.slot]->found.joint.body_from_model.t));
                        err_mm.push_back(e * 1000.0);
                        if (e > 0.10) {
                            wrong++;
                        }
                    }
                    done = true;
                    break;
                }
            }
            never += done ? 0 : 1;
        }
        const Metrics mm = metrics(L.frames, got);
        std::printf("masks %zu of %.1f s (one hand each): eligible %d, relocked %d, not within 60 frames %d | frames to "
                    "relock p50 %.0f p95 %.0f max %.0f (ms p50 %.0f p95 %.0f) | first-fix error p50 %.1f p95 %.1f mm, "
                    "wrong (>10 cm) %d | tracked L %.1f R %.1f %%, swaps %d, jumps %d\n",
                    masks.size(), D, eligible, relocked, never, pctl(lat_fr, 0.5), pctl(lat_fr, 0.95),
                    pctl(lat_fr, 1.0), pctl(lat_ms, 0.5), pctl(lat_ms, 0.95), pctl(err_mm, 0.5), pctl(err_mm, 0.95),
                    wrong, mm.tracked_pct[0], mm.tracked_pct[1], mm.swaps, mm.jumps);
    }
    if (has(a, "--fuse")) {
        pibridge::ctrlbench::FuseBenchIn in;
        for (std::size_t i = 0; i < ref.size(); i++) {
            for (const auto& fx : ref[i].fix) {
                if (!fx) {
                    continue;
                }
                const auto& j = fx->found.joint;
                in.fixes.push_back({fx->slot, L.frames[i].ns, j.body_from_model, j.total(), j.rms_px});
            }
        }
        in.parity = has(a, "--fuse-parity");
        in.seed = argv_of(a, "--seed") ? std::stoul(*argv_of(a, "--seed")) : 1;
        if (const auto v = argv_of(a, "--bias-err")) {
            in.bias_err_dps = std::stod(*v);
        }
        if (const auto v = argv_of(a, "--swap-p")) {
            in.swap_p = std::stod(*v);
        }
        pibridge::ctrlbench::fuse_bench(in);
    }
    return 0;
}
