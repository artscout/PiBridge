// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/main.rs (MIT OR Apache-2.0): `pibridge-roommap`, the persistent room map
// (build a map from a recording or live, recognise the room, apply the Monado world offset, info, export / view,
// cycle, eval, floor, setup, auto, light). Options, output and files are the Rust tool's (docs/pibridge-roommap.md);
// errors start with "pibridge-roommap <cmd>:".
#include "roommap_cli.h"

#include "pibridge/anchor/backend.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/roommap/align.h"
#include "pibridge/roommap/exportmap.h"
#include "pibridge/roommap/floor.h"
#include "pibridge/roommap/live.h"
#include "pibridge/roommap/loopclose.h"
#include "pibridge/roommap/pipeline.h"
#include "pibridge/roommap/rec.h"
#include "pibridge/roommap/refine.h"
#include "pibridge/roommap/setup.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <cstdio>
#include <numbers>
#include <cstring>
#include <fcntl.h>
#include <format>
#include <limits>
#include <mutex>
#include <optional>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace pibridge::roommap::cli {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;
using session::Alignment;

R<std::optional<std::string>> opt(Args& v, const std::string& name) {
    const auto it = std::find(v.begin(), v.end(), name);
    if (it == v.end()) {
        return std::optional<std::string>{};
    }
    const std::size_t i = static_cast<std::size_t>(it - v.begin());
    if (i + 1 < v.size()) {
        std::string x = v[i + 1];
        v.erase(v.begin() + static_cast<std::ptrdiff_t>(i), v.begin() + static_cast<std::ptrdiff_t>(i) + 2);
        return std::optional<std::string>{x};
    }
    return std::unexpected(name + " needs a value");
}

bool flag(Args& v, const std::string& name) {
    const auto it = std::find(v.begin(), v.end(), name);
    if (it == v.end()) {
        return false;
    }
    v.erase(it);
    return true;
}

R<void> done(const Args& v) {
    if (!v.empty()) {
        return std::unexpected("unexpected argument " + rf::debug_str(v[0]));
    }
    return {};
}

R<double> num_f64(Args& v, const std::string& name, double d) {
    auto x = opt(v, name);
    if (!x) {
        return std::unexpected(x.error());
    }
    if (!*x) {
        return d;
    }
    if (auto r = rs::parse_f64(**x)) {
        return *r;
    }
    return std::unexpected(name + ": bad number " + rf::debug_str(**x));
}

R<float> num_f32(Args& v, const std::string& name, float d) {
    auto x = opt(v, name);
    if (!x) {
        return std::unexpected(x.error());
    }
    if (!*x) {
        return d;
    }
    // Rust f32 parse rounds the decimal once to f32 (strtof does too); the grammar check is the f64 one
    if (rs::parse_f64(**x)) {
        float f = 0;
        const std::string& s = **x;
        if (s == "inf" || s == "+inf" || s == "infinity" || s == "+infinity" || s == "Infinity") {
            return std::numeric_limits<float>::infinity();
        }
        f = std::strtof(s.c_str(), nullptr);
        return f;
    }
    return std::unexpected(name + ": bad number " + rf::debug_str(**x));
}

R<std::size_t> num_usize(Args& v, const std::string& name, std::size_t d) {
    auto x = opt(v, name);
    if (!x) {
        return std::unexpected(x.error());
    }
    if (!*x) {
        return d;
    }
    if (auto r = rs::parse_usize(**x)) {
        return *r;
    }
    return std::unexpected(name + ": bad number " + rf::debug_str(**x));
}

R<std::pair<cam::DeviceCalib, stereo::Rig>> rig(Args& v) {
    auto p = opt(v, "--calib");
    if (!p) {
        return std::unexpected(p.error());
    }
    auto c = cam::DeviceCalib::load_default(*p);
    if (!c) {
        return std::unexpected(c.error().text());
    }
    std::fprintf(stderr, "calibration %s (%s)\n", c->second.c_str(), c->first.uid.c_str());
    const std::size_t ids[2] = {0, 1};
    auto r = stereo::Rig::from_calib(c->first, ids);
    if (!r) {
        return std::unexpected(r.error());
    }
    return std::pair{std::move(c->first), std::move(*r)};
}

R<std::string> map_dir(Args& v) {
    auto m = opt(v, "--map");
    if (!m) {
        return std::unexpected(m.error());
    }
    auto n = opt(v, "--name");
    if (!n) {
        return std::unexpected(n.error());
    }
    if (*m) {
        return **m;
    }
    return store::map_dir(n->value_or("home"));
}

std::uint64_t unix_now() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string fmt_align(const Alignment& a) {
    return "yaw " + rf::fixed_w(rf::to_degrees(a.yaw), 8, 3, true) + " deg  t (" + rf::fixed(a.t[0], 4, true) + " " +
           rf::fixed(a.t[1], 4, true) + " " + rf::fixed(a.t[2], 4, true) + ") m";
}

std::vector<std::string> s(std::initializer_list<const char*> v) {
    return std::vector<std::string>(v.begin(), v.end());
}

namespace {

std::once_flag g_role_once;
std::optional<anchor::Role> g_role;

} // namespace

anchor::Role writer_role(anchor::Role def) {
    return g_role.value_or(def);
}

R<void> take_lease(std::uint32_t pid, anchor::Role role) {
    std::call_once(g_role_once, [&] { g_role = role; });
    const anchor::Role r = *g_role;
    auto t = anchor::take_lease(anchor::offset_path(), pid, r);
    if (!t) {
        return std::unexpected(t.error() + "; offset unchanged");
    }
    if (*t) {
        std::printf("%s\n", (*t)->c_str());
    }
    return {};
}

std::string no_poses_text(const std::string& what) {
    const auto f = anchor::feed_now();
    if (f.kind == anchor::Feed::Kind::Live) {
        return "no head poses matched the camera frames (Monado's feed is live; camera and head clocks apart?); " + what;
    }
    return "no head poses: " + f.plain() + "; " + what;
}

namespace {

void print_refine(const refine::RefineStats& rs_) {
    std::printf("refined %zu of %zu points from multi-view observations (median %s views, median move %s cm), dropped %zu\n",
                rs_.refined, rs_.points, rf::fixed(rs_.median_obs, 0).c_str(),
                rf::fixed(rs_.median_move_m * 100.0, 1).c_str(), rs_.dropped);
}

void print_loops(const loopclose::LoopStats& ls) {
    std::printf("loop closure: %zu keyframes, %zu loop edges (%zu rejected), largest keyframe correction %s cm / %s deg\n",
                ls.tried, ls.loops, ls.rejected, rf::fixed(ls.max_corr_m * 100.0, 1).c_str(),
                rf::fixed(ls.max_corr_deg, 2).c_str());
}

orb::Image image(const pngread::Grey& gr) {
    return orb::Image{gr.w, gr.h, gr.w, gr.data};
}

} // namespace

R<void> floor_at_build_hook(const std::string& dir, const map::RoomMap& map, const std::vector<pipeline::PointFrame>& frames,
                            const std::vector<loopclose::Correction>& corr);

R<void> cmd_build_with(Args v, const std::function<void(const light::MapProgress&)>* progress,
                       const std::function<bool()>* stop_early,
                       const std::function<void(const std::vector<guide::P3f>&)>* scan) {
    auto rr = rig(v);
    if (!rr) {
        return std::unexpected(rr.error());
    }
    auto& [cal, rg] = *rr;
#define TRYV(var, expr)                                                                                                \
    auto var##_r = (expr);                                                                                             \
    if (!var##_r) {                                                                                                    \
        return std::unexpected(var##_r.error());                                                                       \
    }                                                                                                                  \
    auto var = *var##_r
    TRYV(dir, map_dir(v));
    TRYV(every, num_usize(v, "--every", 1));
    TRYV(kf_dist, num_f64(v, "--kf-dist", 0.20));
    TRYV(kf_angle, num_f64(v, "--kf-angle", 15.0));
    TRYV(recp, opt(v, "--rec"));
    TRYV(tracking, opt(v, "--tracking"));
    TRYV(source, opt(v, "--source"));
    TRYV(seconds, num_f64(v, "--seconds", 60.0));
    const bool no_refine = !flag(v, "--refine");
    bool no_loops = flag(v, "--no-loops");
    const bool keep_unrefined = flag(v, "--keep-unrefined");
    TRYV(threads, num_usize(v, "--threads", 8));
    TRYV(min_kf, num_usize(v, "--min-keyframes", 0));
    TRYV(extend, num_f64(v, "--extend", 0.0));
    // The mapping walk continues until the room is actually covered, not a fixed timer (room-setup rework Phase 1,
    // premature-exit fix): --cover-target is the fraction of the 12 head-yaw sectors (30 deg each) that must be
    // faced before the walk may stop early. 0 (the default) keeps the old timer-only behaviour; the headset setup
    // injects 0.8 (look around in at least 10 of the 12 directions). The hard cap stays `--seconds + --extend`.
    TRYV(cover_target, num_f64(v, "--cover-target", 0.0));
    const bool append = flag(v, "--append");
    if (auto d = done(v); !d) {
        return d;
    }
    refine::RefineParams rp;
    rp.drop_unrefined = !keep_unrefined;
    auto sp = pipeline::build_params(kf_dist, kf_angle);
    if (append && recp) {
        return std::unexpected("--append works with --source broker only");
    }
    std::optional<std::pair<map::RoomMap, store::RelocResult>> appending;
    if (append) {
        auto m = map::RoomMap::load(dir);
        if (!m) {
            return std::unexpected(m.error());
        }
        auto r = store::RelocResult::load(cam::path_join(dir, "last-reloc.toml"));
        if (!r) {
            return std::unexpected(r.error());
        }
        sp.kf_per_session = true;
        appending = std::pair{std::move(*m), *r};
    }
    no_loops = no_loops || appending.has_value();
    const std::size_t ids[2] = {0, 1};
    map::RoomMap map0 = map::RoomMap::make(cal.uid, ids);
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<loopclose::Correction> corrections;
    std::vector<pipeline::PointFrame> floor_frames;
    map::RoomMap m;
    std::string what;
    if (recp) {
        auto rec = rec::Recording::open(*recp, tracking);
        if (!rec) {
            return std::unexpected(rec.error());
        }
        std::fprintf(stderr, "%s: %zu frame sets, %zu poses (%s)\n", rec->root.c_str(), rec->frames.size(),
                     rec->traj.size(), rec->tracking.c_str());
        const Alignment id{};
        auto b = pipeline::build_from_recording(*rec, rg, std::move(map0), sp, every, id);
        if (!b) {
            return std::unexpected(b.error());
        }
        m = std::move(b->first);
        const auto st = b->second;
        auto ff = pipeline::recording_points(*rec, rg, 3, 3.5, threads);
        if (!ff) {
            return std::unexpected(ff.error());
        }
        floor_frames = std::move(*ff);
        std::fprintf(stderr, "%zu frames used, %zu without a tracker pose\n", st.frames, st.no_pose);
        if (!(no_refine && no_loops)) {
            auto frames = pipeline::map_frames(*rec, rg, 2, orb::OrbParams{}, id, threads);
            if (!frames) {
                return std::unexpected(frames.error());
            }
            if (!no_loops) {
                auto [ls, corr] = loopclose::close_loops(m, rg, *frames, align::ObserveParams{}, loopclose::LoopParams{},
                                                         threads);
                print_loops(ls);
                corrections = std::move(corr);
            }
            if (!no_refine) {
                print_refine(refine::refine_map(m, rg, *frames, rp, threads));
            }
        }
        what = rec->root;
    } else if (source) {
        const std::string spec = *source;
        std::printf("Mapping for %s s: walk slowly through the room, look at every wall, the desk and the floor near you.\n",
                    rf::fixed(seconds, 0).c_str());
        const std::size_t need = sp.kf_min_points;
        std::optional<Alignment> room_from_slam;
        if (appending) {
            room_from_slam = appending->second.align;
        }
        std::optional<session::RoomSession> sess;
        if (appending) {
            sess.emplace(session::RoomSession::with_alignment(std::move(appending->first), rg, sp, appending->second.align));
        } else {
            sess.emplace(std::move(map0), rg, sp);
        }
        std::vector<refine::MapFrame> frames;
        std::size_t k = 0, kfs = 0, since_kf = 0;
        std::optional<std::int64_t> t_first;
        std::vector<light::FrameQuality> qs;
        const orb::OrbParams orb;
        // head-yaw coverage of the look-around (12 sectors of 30 deg); the walk may stop early once enough are seen
        std::array<bool, 12> cov_seen{};
        const auto cov_fraction = [&cov_seen] {
            return static_cast<double>(std::ranges::count(cov_seen, true)) / static_cast<double>(cov_seen.size());
        };
        // the scan cloud for the headset's "scanning" overlay: room-frame points voxel-downsampled (8 cm) and capped,
        // grown across the walk (first point per voxel wins; once full it stops growing). Sent to `scan` each tick.
        std::unordered_map<std::int64_t, guide::P3f> scan_vox;
        const auto scan_add = [&scan, &scan_vox](const auto& p) {
            constexpr double voxel = 0.08;
            constexpr std::size_t cap = 4000;
            if (scan == nullptr || scan_vox.size() >= cap) {
                return;
            }
            const auto q = [](double c) { return static_cast<std::int64_t>(std::llround(c / voxel)); };
            const std::int64_t key = (q(p[0]) * 73856093LL) ^ (q(p[1]) * 19349663LL) ^ (q(p[2]) * 83492791LL);
            scan_vox.try_emplace(key,
                                 guide::P3f{static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
        };
        auto st = live::capture(spec, seconds + rf::fmax(extend, 0.0), [&](live::LiveFrame& f) {
            if (!t_first) {
                t_first = f.ts_ns;
            }
            const double el = static_cast<double>(f.ts_ns - *t_first) / 1e9;
            if (!room_from_slam) {
                room_from_slam = live::current_offset(f.pid);
            }
            const Alignment a = *room_from_slam;
            const orb::Image views[2] = {image(f.imgs[0]), image(f.imgs[1])};
            const auto ev = sess->on_frame(f.ts_ns, views, f.slam_from_body, a, true);
            std::size_t new_kf = 0;
            for (const auto& e : ev) {
                new_kf += e.kind == session::Event::Kind::Keyframe;
            }
            kfs += new_kf;
            since_kf = new_kf > 0 ? 0 : since_kf + 1;
            if (k % 3 == 0) {
                auto feats = pipeline::frame_features(rg, f.imgs[0], f.imgs[1], orb);
                const g::Pose rb = a.pose().compose(f.slam_from_body);
                if (scan != nullptr) {
                    // the scan overlay points in the RAW SLAM frame: the headset places them with its StageLink during
                    // the walk (the room frame / conv is not published until after mapping). Drop points within 0.5 m
                    // of the head: those are the hands, the controllers and the body (they triangulate too and would
                    // float "in the air" right in front of the user), not the room's surfaces.
                    const g::Vec3 hp = f.slam_from_body.apply(g::Vec3{0.0, 0.0, 0.0});
                    for (const auto& p : pipeline::frame_points(rg, feats, f.slam_from_body, 3.5)) {
                        const double dx = p[0] - hp[0], dy = p[1] - hp[1], dz = p[2] - hp[2];
                        if (dx * dx + dy * dy + dz * dz < 0.5 * 0.5) {
                            continue;
                        }
                        scan_add(p);
                    }
                }
                floor_frames.push_back(pipeline::PointFrame{f.ts_ns, rb, pipeline::frame_points(rg, feats, rb, 3.5)});
                // which yaw sector the head faces now (forward = the body's -Z in the room frame)
                const g::Vec3 fwd = g::mat_vec(rb.r, {0.0, 0.0, -1.0});
                double hdg = std::atan2(fwd[0], fwd[2]);
                if (hdg < 0.0) {
                    hdg += 2.0 * std::numbers::pi;
                }
                const int sector = std::clamp(static_cast<int>(hdg / (2.0 * std::numbers::pi / 12.0)), 0, 11);
                cov_seen[static_cast<std::size_t>(sector)] = true;
                if (k % 15 == 0) {
                    const auto q = light::quality_of(rg, feats, f.imgs[0], f.imgs[1]);
                    qs.push_back(q);
                    const light::MapProgress p{el, kfs, q, light::advice(q, need), since_kf > 90};
                    if (progress != nullptr) {
                        (*progress)(p);
                    }
                    if (scan != nullptr) {
                        std::vector<guide::P3f> pc;
                        pc.reserve(scan_vox.size());
                        for (const auto& [key, pt] : scan_vox) {
                            pc.push_back(pt);
                        }
                        (*scan)(pc);
                    }
                }
                if (!no_loops && k % 6 == 0) {
                    frames.push_back(refine::MapFrame::make(f.ts_ns, rb, std::move(feats), rg));
                }
            } else if (!no_loops && (k & 1) == 0) {
                auto feats = pipeline::frame_features(rg, f.imgs[0], f.imgs[1], orb);
                frames.push_back(refine::MapFrame::make(f.ts_ns, a.pose().compose(f.slam_from_body), std::move(feats), rg));
            }
            k++;
            if (stop_early && (*stop_early)()) {
                return false; // the headset pressed Finish: end the walk now
            }
            // keep mapping while below the minimum time, below the keyframe floor, or (when a target is set) the room
            // is not yet covered; the capture's own cap of `seconds + extend` stops it in any case (the sane max).
            return el < seconds || kfs < min_kf || (cover_target > 0.0 && cov_fraction() < cover_target);
        });
        if (!st) {
            return std::unexpected(st.error());
        }
        std::fprintf(stderr, "%zu tracking frames, %zu with a Monado head pose (pid %u), %zu new keyframes\n", st->frames,
                     st->posed, st->pid, kfs);
        const auto lr = light::summarise(qs, need);
        if (lr.frames > 0) {
            std::fprintf(stderr,
                         "image: brightness %s/255, ORB corners %zu / %zu, stereo points %zu (a keyframe needs %zu), %zu of "
                         "%zu sampled frame sets usable\n",
                         rf::fixed(lr.mean, 0).c_str(), lr.corners[0], lr.corners[1], lr.stereo, need, lr.good, lr.frames);
        }
        if (st->posed == 0) {
            return std::unexpected(no_poses_text("nothing saved"));
        }
        if (kfs == 0 && lr.frames > 0) {
            light::FrameQuality q;
            q.mean = lr.mean;
            q.stereo = lr.stereo;
            std::string why;
            if (light::advice(q, need) == light::Advice::TooDark) {
                why = "too dark: brightness " + rf::fixed(lr.mean, 0) + "/255, " + std::to_string(lr.stereo) +
                      " stereo points per view (a view needs " + std::to_string(need) + "); turn on the room lights";
            } else {
                why = "too little detail: " + std::to_string(lr.stereo) + " stereo points per view (a view needs " +
                      std::to_string(need) + "); look at furniture, not blank walls";
            }
            return std::unexpected("no keyframes (" + why + "); nothing saved");
        }
        m = std::move(*sess).finish();
        if (!no_loops) {
            auto [ls, corr] = loopclose::close_loops(m, rg, frames, align::ObserveParams{}, loopclose::LoopParams{}, threads);
            print_loops(ls);
            corrections = std::move(corr);
            if (!no_refine) {
                print_refine(refine::refine_map(m, rg, frames, rp, threads));
            }
        }
        what = spec + " (Monado pid " + std::to_string(st->pid) + ")";
    } else {
        return std::unexpected("build needs --rec DIR or --source broker");
    }
    if (m.keyframes.empty()) {
        return std::unexpected("no keyframes (too dark, too little texture or no stereo overlap); nothing saved");
    }
    auto p = m.save(dir);
    if (!p) {
        return std::unexpected(p.error());
    }
    if (auto w = cam::write_file(cam::path_join(dir, "roommap.toml"),
                                 store::meta_text(what, m.keyframes.size(), m.points(), unix_now()));
        !w) {
        return std::unexpected(w.error().text());
    }
    if (!append) {
        if (auto c = store::save_corrections(dir, corrections); !c) {
            return c;
        }
        ::unlink(cam::path_join(dir, "floor.toml").c_str());
        (void)floor_at_build_hook(dir, m, floor_frames, corrections);
    }
    std::printf("%s: %zu keyframes, %zu points (%s s)\n", p->c_str(), m.keyframes.size(), m.points(),
                rf::fixed(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 1).c_str());
    return {};
}

namespace {

R<void> cmd_build(Args v) {
    return cmd_build_with(std::move(v), nullptr);
}

R<void> cmd_light(Args v) {
    auto rr = rig(v);
    if (!rr) {
        return std::unexpected(rr.error());
    }
    const auto& rg = rr->second;
    TRYV(spec_o, opt(v, "--source"));
    const std::string spec = spec_o.value_or("broker");
    TRYV(seconds, num_f64(v, "--seconds", 5.0));
    TRYV(every, num_usize(v, "--every", 3));
    const bool no_stretch = flag(v, "--no-stretch");
    std::optional<std::pair<float, float>> dk;
    {
        TRYV(dx, opt(v, "--darken"));
        if (dx) {
            float f = 0;
            if (!rs::parse_f64(*dx)) {
                return std::unexpected("--darken: bad number " + rf::debug_str(*dx));
            }
            f = std::strtof(dx->c_str(), nullptr);
            TRYV(noise, num_f32(v, "--noise", 3.0f));
            dk = std::pair{f, noise};
        }
    }
    if (auto d = done(v); !d) {
        return d;
    }
    const std::size_t need = pipeline::build_params(0.2, 15.0).kf_min_points;
    light::MeasureOpts o;
    o.every = every;
    o.low = no_stretch ? std::nullopt : std::optional<stereo::LowLight>(stereo::LOW_LIGHT);
    o.darken = dk;
    auto qs = light::measure_source(spec, seconds, o, rg, orb::OrbParams{});
    if (!qs) {
        return std::unexpected(qs.error());
    }
    const auto r = light::summarise(*qs, need);
    std::printf("%zu frame sets: brightness %s/255, exposure %u gain %u, ORB corners %zu / %zu, stereo points %zu (a "
                "keyframe needs %zu); %zu of %zu frame sets usable\n",
                r.frames, rf::fixed(r.mean, 0).c_str(), r.exposure, r.gain, r.corners[0], r.corners[1], r.stereo, need,
                r.good, r.frames);
    if (!qs->empty()) {
        const auto& q = (*qs)[qs->size() / 2];
        std::printf("%s\n", light::advice_text(light::advice(q, need)));
    }
    return {};
}

void print_fit(const std::string& label, const align::JointFit& f) {
    std::printf("%s: %s | %zu frames, %zu inliers, rms %s deg, formal 1-sigma %s deg / %s mm, frame spread %s mm / %s deg\n",
                label.c_str(), fmt_align(f.align).c_str(), f.frames, f.inliers, rf::fixed(f.rms_deg, 3).c_str(),
                rf::fixed(f.sigma_deg, 3).c_str(), rf::fixed(f.sigma_m * 1e3, 1).c_str(),
                rf::fixed(f.hyp_rms_m * 1e3, 1).c_str(), rf::fixed(f.hyp_rms_deg, 2).c_str());
}

std::tuple<double, double, double, double> spread(const std::vector<align::JointFit>& fits, const Alignment& center) {
    const double n = static_cast<double>(std::max<std::size_t>(fits.size(), 1));
    std::vector<double> dm, dd;
    for (const auto& f : fits) {
        dm.push_back(align::diff(f.align, center).first);
        dd.push_back(rf::to_degrees(align::wrap(f.align.yaw - center.yaw)));
    }
    double sm = -0.0, sd = -0.0;
    for (double x : dm) {
        sm += x * x;
    }
    for (double x : dd) {
        sd += x * x;
    }
    double mm = 0.0, md = 0.0;
    for (double x : dm) {
        mm = rf::fmax(mm, x);
    }
    for (double x : dd) {
        md = rf::fmax(md, std::fabs(x));
    }
    return {std::sqrt(sm / n), std::sqrt(sd / n), mm, md};
}

/// Rust `chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' { c } else { '_' })`: one '_' per UTF-8 character.
std::string safe_name(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size();) {
        const auto c = static_cast<unsigned char>(s[i]);
        std::size_t len = 1;
        if (c >= 0xF0) {
            len = 4;
        } else if (c >= 0xE0) {
            len = 3;
        } else if (c >= 0xC0) {
            len = 2;
        }
        if (len == 1 && (std::isalnum(c) != 0 || c == '-')) {
            out += static_cast<char>(c);
        } else {
            out += '_';
        }
        i += len;
    }
    return out;
}

} // namespace

R<void> cmd_relocalize(Args v) {
    auto rr = rig(v);
    if (!rr) {
        return std::unexpected(rr.error());
    }
    const auto& rg = rr->second;
    TRYV(dir, map_dir(v));
    TRYV(every, num_usize(v, "--every", 2));
    TRYV(window, num_f64(v, "--window", 4.0));
    TRYV(threads, num_usize(v, "--threads", 8));
    TRYV(recp, opt(v, "--rec"));
    TRYV(tracking, opt(v, "--tracking"));
    TRYV(source, opt(v, "--source"));
    TRYV(seconds, num_f64(v, "--seconds", 15.0));
    TRYV(result, opt(v, "--result"));
    const bool apply = flag(v, "--apply");
    const bool no_floor = flag(v, "--no-floor");
    if (auto d = done(v); !d) {
        return d;
    }
    auto mm = map::RoomMap::load(dir);
    if (!mm) {
        return std::unexpected(mm.error());
    }
    const map::RoomMap& m = *mm;
    std::printf("map %s: %zu keyframes, %zu points\n", dir.c_str(), m.keyframes.size(), m.points());
    const auto bfc = rg.body_from_cams();
    const align::JointParams jp;
    align::AcceptParams ap;
    ap.window_ns = rs::to_i64(window * 1e9);
    const align::ObserveParams op;
    const orb::OrbParams orb;
    std::vector<align::FrameObs> obs;
    std::size_t tried = 0;
    std::int64_t t_first = 0;
    std::string src_name;
    std::uint32_t pid = 0;
    if (recp) {
        auto rec = rec::Recording::open(*recp, tracking);
        if (!rec) {
            return std::unexpected(rec.error());
        }
        const auto t0 = std::chrono::steady_clock::now();
        auto r = pipeline::observe_recording(*rec, m, rg, every, orb, op, threads);
        if (!r) {
            return std::unexpected(r.error());
        }
        obs = std::move(r->first);
        tried = r->second;
        std::fprintf(stderr, "matched %zu of %zu frame sets in %s s\n", obs.size(), tried,
                     rf::fixed(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 1).c_str());
        t_first = rec->frames.empty() ? 0 : rec->frames.front().ts_ns;
        src_name = rec->name();
    } else if (source) {
        std::printf("Look around the mapped room for up to %s s (slow head turns).\n", rf::fixed(seconds, 0).c_str());
        std::size_t n = 0;
        std::optional<std::int64_t> tf;
        auto st = live::capture(*source, seconds, [&](live::LiveFrame& f) {
            n++;
            if (!tf) {
                tf = f.ts_ns;
            }
            const auto feats = pipeline::frame_features(rg, f.imgs[0], f.imgs[1], orb);
            if (auto o = align::observe(m, rg, feats, f.ts_ns, f.slam_from_body, op)) {
                obs.push_back(std::move(*o));
                const std::size_t from = obs.size() > 200 ? obs.size() - 200 : 0;
                if (align::first_fix(std::span(obs).subspan(from), bfc, jp, ap)) {
                    return false;
                }
            }
            return true;
        });
        if (!st) {
            return std::unexpected(st.error());
        }
        if (st->frames > 0 && st->posed == 0) {
            return std::unexpected(no_poses_text("offset unchanged"));
        }
        tried = n;
        t_first = tf.value_or(0);
        src_name = *source;
        pid = st->pid;
    } else {
        return std::unexpected("relocalize needs --rec DIR or --source broker");
    }
    std::printf("%zu of %zu frame sets matched the map\n", obs.size(), tried);
    const auto ff = align::first_fix(obs, bfc, jp, ap);
    if (!ff) {
        return std::unexpected("room NOT recognised (" + std::to_string(obs.size()) + " matching frames, need " +
                               std::to_string(ap.min_frames) + " agreeing within " + rf::fixed(window, 0) +
                               " s); offset unchanged");
    }
    const auto& [i, fix] = *ff;
    const double fix_after_s = static_cast<double>(obs[i].ts_ns - t_first) / 1e9;
    print_fit("first fix after " + rf::fixed(fix_after_s, 2) + " s", fix);
    std::vector<const align::FrameObs*> refs;
    for (const auto& o : obs) {
        refs.push_back(&o);
    }
    if (refs.size() > i + 1) {
        if (auto all = align::joint_fit(refs, bfc, jp)) {
            print_fit("whole session", *all);
            const auto [dm, dd] = align::diff(all->align, fix.align);
            std::printf("  first fix vs whole session: %s mm, %s deg\n", rf::fixed(dm * 1e3, 1).c_str(),
                        rf::fixed(dd, 2).c_str());
            const auto ws = align::window_fits(obs, bfc, jp, ap.window_ns, ap.min_frames);
            if (ws.size() >= 2) {
                const auto [rm, rd, mx, md] = spread(ws, all->align);
                std::printf("  %zu windows of %s s: spread around the whole-session fit %s mm / %s deg RMS, max %s mm / %s "
                            "deg (tracker drift + relocalisation noise)\n",
                            ws.size(), rf::fixed(window, 0).c_str(), rf::fixed(rm * 1e3, 1).c_str(),
                            rf::fixed(rd, 2).c_str(), rf::fixed(mx * 1e3, 1).c_str(), rf::fixed(md, 2).c_str());
                for (const auto& w : ws) {
                    std::printf("    %s-%s s  %s  %s frames %s inl\n",
                                rf::fixed_w(static_cast<double>(w.span_ns.first - t_first) / 1e9, 6, 1).c_str(),
                                rf::fixed_w(static_cast<double>(w.span_ns.second - t_first) / 1e9, 5, 1).c_str(),
                                fmt_align(w.align).c_str(), std::format("{:>3}", w.frames).c_str(),
                                std::format("{:>5}", w.inliers).c_str());
                }
            }
        }
    }
    store::RelocResult res;
    res.map = dir;
    res.source = src_name;
    res.align = fix.align;
    res.frames = fix.frames;
    res.inliers = fix.inliers;
    res.rms_deg = fix.rms_deg;
    res.sigma_deg = fix.sigma_deg;
    res.sigma_m = fix.sigma_m;
    res.fix_after_s = fix_after_s;
    res.pid = pid;
    const std::string out = result ? *result : cam::path_join(dir, "reloc-" + safe_name(src_name) + ".toml");
    if (auto w = res.save(out); !w) {
        return w;
    }
    if (auto w = res.save(cam::path_join(dir, "last-reloc.toml")); !w) {
        return w;
    }
    std::printf("saved %s (and last-reloc.toml)\n", out.c_str());
    if (apply) {
        return write_offset(dir, res, std::nullopt, false, no_floor, anchor::Role::Reloc);
    }
    return {};
}

R<std::pair<Alignment, std::optional<double>>> reloc_offset_hook(const std::string& dir, const Alignment& a,
                                                                 std::uint32_t pid);

R<void> write_offset(const std::string& dir, const store::RelocResult& res, std::optional<std::uint32_t> pid_in, bool dry,
                     bool no_floor, anchor::Role role) {
    const auto ring = anchor::HeadRing::read(anchor::ring_path());
    std::uint32_t pid = 0;
    if (pid_in) {
        pid = *pid_in;
    } else if (res.pid != 0) {
        pid = res.pid;
    } else if (ring) {
        pid = ring->pid;
    }
    if (ring && res.pid != 0 && ring->pid != res.pid) {
        return std::unexpected("the result belongs to Monado pid " + std::to_string(res.pid) + " but pid " +
                               std::to_string(ring->pid) + " runs now: relocalise again; offset unchanged");
    }
    Alignment al = res.align;
    std::optional<double> shift;
    if (!no_floor) {
        auto r = reloc_offset_hook(dir, res.align, pid);
        if (!r) {
            return std::unexpected(r.error());
        }
        al = r->first;
        shift = r->second;
    }
    if (shift) {
        std::printf("floor: %s/floor.toml puts the room %s m vertically onto Monado's STAGE floor\n", dir.c_str(),
                    rf::fixed(*shift, 3, true).c_str());
    } else if (!no_floor) {
        std::printf("floor: none stored for this map (omarchyvr-roommap floor detect --save): height left as mapped\n");
    }
    const auto room_setup = setup::RoomSetup::load(dir);
    // The backend says how the room is committed (anchor/backend.h), never which process writes the ring: guessing it
    // from the ring's writer sent the SteamVR path down the Monado branch (the zone turned ~90 deg, 2026-10-25).
    const bool steamvr = pibridge::anchor::backend_from_env() == pibridge::anchor::Backend::SteamVr;
    if (steamvr) {
        // Under SteamVR the offset is ONLY room_from_slam (yaw + horizontal), so the tracker's output is the ROOM frame
        // in every session. The setup's centre / forward / floor live in SteamVR's chaperone (universe 30, written in
        // room coordinates at the end of the setup) and are applied by SteamVR itself. They must NOT also be folded
        // into this offset: composing setup_from_room here, and composing the chaperone's stored standing pose (which is
        // in the SETUP session's raw space, meaningless in a new session), shifted and turned the restored play area
        // by an arbitrary angle after a restart (seen as ~90 deg). The floor stays SteamVR's: offset y = 0.
        al.t[1] = 0.0;
        if (room_setup) {
            std::printf("room setup \"%s\": centre / forward / floor are SteamVR's chaperone; the offset is room_from_slam only\n",
                        room_setup->describe().c_str());
        }
    } else {
        if (room_setup) {
            std::printf("room setup: centre (%s, %s) m and forward %s deg of the map become the STAGE origin and -Z; %s\n",
                        rf::fixed(room_setup->centre[0], 2, true).c_str(),
                        rf::fixed(room_setup->centre[1], 2, true).c_str(),
                        rf::fixed(rf::to_degrees(room_setup->yaw), 1).c_str(), room_setup->describe().c_str());
            al = store::compose(room_setup->setup_from_room(), al);
        }
        // Monado backend: the offset carries everything (floor, centre, forward); SteamVR's stage is never read.
    }
    std::printf("offset (app_from_slam): %s\n", fmt_align(al).c_str());
    if (g::norm(al.t) > 20.0) {
        return std::unexpected("offset larger than 20 m: wrong map? not written");
    }
    if (dry) {
        std::printf("dry run: nothing written\n");
        return {};
    }
    if (pid == 0) {
        std::printf("note: no Monado pid known (Monado not running with PIMAX_ANCHOR=1?): Monado ignores the file until a "
                    "matching pid is written\n");
    }
    if (auto l = take_lease(pid, writer_role(role)); !l) {
        return l;
    }
    auto p = store::write_offset(al, pid);
    if (!p) {
        return std::unexpected(p.error().text());
    }
    std::printf("wrote %s for Monado pid %u\n", p->c_str(), pid);
    if (steamvr && floor::StoredFloor::load(dir)) {
        // The chaperone's floor is a raw-space y of the session that wrote it; this session's raw space is new. Put the
        // saved floor plane back through the fix just written (floor.toml y - last-reloc t[1]); centre / forward stay
        // (they are room-frame, valid in every session). pibridge-roomsetup is the OpenVR client for that.
        const int cs = std::system("pibridge-roomsetup --vr-standing --floor-only >/dev/null 2>&1");
        std::printf("SteamVR chaperone floor from the saved room: %s\n", cs == 0 ? "set" : "not set (is SteamVR running?)");
    }
    if (room_setup && pid != 0) {
        if (auto b = write_stage_bounds(pid, room_setup->play_area)) {
            std::printf("STAGE bounds %s x %s m: %s\n", rf::fixed(room_setup->play_area[0], 2).c_str(),
                        rf::fixed(room_setup->play_area[1], 2).c_str(), b->c_str());
        } else {
            std::fprintf(stderr, "cannot write the STAGE bounds: %s\n", b.error().c_str());
        }
    }
    return {};
}

namespace {

R<void> cmd_apply(Args v) {
    TRYV(dir, map_dir(v));
    TRYV(result_o, opt(v, "--result"));
    const std::string result = result_o.value_or(cam::path_join(dir, "last-reloc.toml"));
    TRYV(pid_o, opt(v, "--pid"));
    std::optional<std::uint32_t> pid;
    if (pid_o) {
        auto p = rs::parse_u32(*pid_o);
        if (!p) {
            return std::unexpected("--pid: bad number");
        }
        pid = *p;
    }
    const bool dry = flag(v, "--dry-run");
    const bool no_floor = flag(v, "--no-floor");
    if (auto d = done(v); !d) {
        return d;
    }
    auto res = store::RelocResult::load(result);
    if (!res) {
        return std::unexpected(res.error());
    }
    std::printf("%s: %s from %s (%zu frames, %zu inliers, rms %s deg)\n", result.c_str(), res->map.c_str(),
                res->source.c_str(), res->frames, res->inliers, rf::fixed(res->rms_deg, 3).c_str());
    return write_offset(dir, *res, pid, dry, no_floor, anchor::Role::Apply);
}

R<void> cmd_info(Args v) {
    TRYV(dir, map_dir(v));
    if (auto d = done(v); !d) {
        return d;
    }
    auto m = map::RoomMap::load(dir);
    if (!m) {
        return std::unexpected(m.error());
    }
    g::Vec3 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    g::Vec3 hi{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
               std::numeric_limits<double>::lowest()};
    for (const auto& k : m->keyframes) {
        for (int i = 0; i < 3; i++) {
            lo[i] = rf::fmin(lo[i], k.pose.t[i]);
            hi[i] = rf::fmax(hi[i], k.pose.t[i]);
        }
    }
    std::printf("%s: device %s | %u sessions, %zu keyframes, %zu points | keyframes x %s..%s y %s..%s z %s..%s m\n",
                dir.c_str(), m->device_uid.c_str(), m->sessions, m->keyframes.size(), m->points(),
                rf::fixed(lo[0], 2).c_str(), rf::fixed(hi[0], 2).c_str(), rf::fixed(lo[1], 2).c_str(),
                rf::fixed(hi[1], 2).c_str(), rf::fixed(lo[2], 2).c_str(), rf::fixed(hi[2], 2).c_str());
    if (auto t = cam::read_to_string(cam::path_join(dir, "roommap.toml"))) {
        std::fputs(t->c_str(), stdout);
    }
    if (auto r = store::RelocResult::load(cam::path_join(dir, "last-reloc.toml"))) {
        std::printf("last relocalisation: %s from %s (%s s to fix)\n", fmt_align(r->align).c_str(), r->source.c_str(),
                    rf::fixed(r->fix_after_s, 2).c_str());
    }
    return {};
}

std::string dir_name(const std::string& dir) {
    return rf::utf8_lossy(rs::file_name(dir));
}

R<void> cmd_export(Args v) {
    TRYV(dir, map_dir(v));
    TRYV(out_o, opt(v, "--out"));
    const std::string out = out_o.value_or(cam::path_join(dir, "map.json"));
    if (auto d = done(v); !d) {
        return d;
    }
    auto j = exportmap::export_json(dir, dir_name(dir));
    if (!j) {
        return std::unexpected(j.error());
    }
    if (auto w = cam::write_file(out, *j); !w) {
        return std::unexpected(out + ": " + w.error().text());
    }
    std::printf("%s (%zu bytes)\n", out.c_str(), j->size());
    return {};
}

R<void> cmd_view(Args v) {
    TRYV(dir, map_dir(v));
    TRYV(out_o, opt(v, "--out"));
    const std::string out = out_o.value_or(cam::path_join(dir, "map.html"));
    const bool open = flag(v, "--open");
    if (auto d = done(v); !d) {
        return d;
    }
    const std::string name = dir_name(dir);
    auto j = exportmap::export_json(dir, name);
    if (!j) {
        return std::unexpected(j.error());
    }
    const std::string page = exportmap::html_page(*j, name);
    if (auto w = cam::write_file(out, page); !w) {
        return std::unexpected(out + ": " + w.error().text());
    }
    std::printf("%s (%zu KB)\n", out.c_str(), page.size() / 1024);
    if (open) {
        posix_spawn_file_actions_t fa;
        posix_spawn_file_actions_init(&fa);
        posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
        posix_spawn_file_actions_addopen(&fa, 1, "/dev/null", O_WRONLY, 0);
        posix_spawn_file_actions_addopen(&fa, 2, "/dev/null", O_WRONLY, 0);
        char* argv[] = {const_cast<char*>("xdg-open"), const_cast<char*>(out.c_str()), nullptr};
        pid_t child = 0;
        const int e = posix_spawnp(&child, "xdg-open", &fa, nullptr, argv, environ);
        posix_spawn_file_actions_destroy(&fa);
        if (e != 0) {
            return std::unexpected("xdg-open: " + cam::IoError::from_errno(e).text());
        }
    }
    return {};
}

R<void> cmd_cycle(Args v) {
    if (v.size() < 2) {
        return std::unexpected("cycle needs at least two result files");
    }
    Alignment acc{};
    for (const auto& f : v) {
        auto r = store::RelocResult::load(f);
        if (!r) {
            return std::unexpected(r.error());
        }
        std::printf("%s: %s\n", f.c_str(), fmt_align(r->align).c_str());
        acc = store::compose(acc, r->align);
    }
    std::printf("cycle product (identity if consistent): %s -> %s mm, %s deg\n", fmt_align(acc).c_str(),
                rf::fixed(g::norm(acc.t) * 1e3, 1).c_str(), rf::fixed(std::fabs(rf::to_degrees(acc.yaw)), 3).c_str());
    return {};
}

std::vector<std::tuple<align::JointFit, g::Vec3, g::Vec3>> placed_windows(const std::vector<align::FrameObs>& obs,
                                                                         std::span<const g::Pose> bfc,
                                                                         const align::JointParams& jp,
                                                                         std::int64_t win_ns) {
    std::vector<std::tuple<align::JointFit, g::Vec3, g::Vec3>> out;
    for (auto& f : align::window_fits(obs, bfc, jp, win_ns, 6)) {
        std::vector<const align::FrameObs*> fr;
        for (const auto& o : obs) {
            if (o.ts_ns >= f.span_ns.first && o.ts_ns <= f.span_ns.second) {
                fr.push_back(&o);
            }
        }
        if (fr.empty()) {
            continue;
        }
        const double n = static_cast<double>(fr.size());
        g::Vec3 p{0.0, 0.0, 0.0}, d{0.0, 0.0, 0.0};
        for (const auto* o : fr) {
            p = g::add(p, g::scale(o->slam_from_body.t, 1.0 / n));
        }
        for (const auto* o : fr) {
            d = g::add(d, g::mat_vec(o->slam_from_body.r, {0.0, 0.0, -1.0}));
        }
        out.emplace_back(f, p, g::normalize(d));
    }
    return out;
}

R<void> cmd_eval(Args v) {
    auto rr = rig(v);
    if (!rr) {
        return std::unexpected(rr.error());
    }
    const auto& rg = rr->second;
    TRYV(ma_o, opt(v, "--map-a"));
    if (!ma_o) {
        return std::unexpected("--map-a NAME");
    }
    const std::string map_a = store::map_dir(*ma_o);
    TRYV(mb_o, opt(v, "--map-b"));
    if (!mb_o) {
        return std::unexpected("--map-b NAME");
    }
    const std::string map_b = store::map_dir(*mb_o);
    TRYV(ra_o, opt(v, "--rec-a"));
    if (!ra_o) {
        return std::unexpected("--rec-a DIR");
    }
    TRYV(rb_o, opt(v, "--rec-b"));
    if (!rb_o) {
        return std::unexpected("--rec-b DIR");
    }
    TRYV(every, num_usize(v, "--every", 2));
    TRYV(window, num_f64(v, "--window", 3.0));
    TRYV(threads, num_usize(v, "--threads", 8));
    if (auto d = done(v); !d) {
        return d;
    }
    auto ma = map::RoomMap::load(map_a);
    if (!ma) {
        return std::unexpected(ma.error());
    }
    auto mb = map::RoomMap::load(map_b);
    if (!mb) {
        return std::unexpected(mb.error());
    }
    auto ra = rec::Recording::open(*ra_o, std::nullopt);
    if (!ra) {
        return std::unexpected(ra.error());
    }
    auto rb = rec::Recording::open(*rb_o, std::nullopt);
    if (!rb) {
        return std::unexpected(rb.error());
    }
    const auto bfc = rg.body_from_cams();
    const orb::OrbParams orb;
    const align::ObserveParams op;
    const align::JointParams jp;
    auto ob = pipeline::observe_recording(*rb, *ma, rg, every, orb, op, threads);
    if (!ob) {
        return std::unexpected(ob.error());
    }
    auto oa = pipeline::observe_recording(*ra, *mb, rg, every, orb, op, threads);
    if (!oa) {
        return std::unexpected(oa.error());
    }
    const auto ca = store::load_corrections(map_a);
    const auto cb = store::load_corrections(map_b);
    const std::int64_t w = rs::to_i64(window * 1e9);
    const auto wb = placed_windows(ob->first, bfc, jp, w);
    const auto wa = placed_windows(oa->first, bfc, jp, w);
    std::printf("%zu windows of B in map A, %zu windows of A in map B (%s s)\n", wb.size(), wa.size(),
                rf::fixed(window, 0).c_str());
    struct ARoom {
        const align::JointFit* f;
        g::Vec3 p;
        g::Vec3 d;
        Alignment c;
    };
    std::vector<ARoom> wa_room;
    for (const auto& [f, p, d] : wa) {
        const Alignment c = loopclose::correction_at(ca, (f.span_ns.first + f.span_ns.second) / 2);
        const g::Pose cp = c.pose();
        wa_room.push_back(ARoom{&f, cp.apply(p), g::mat_vec(cp.r, d), c});
    }
    std::vector<std::pair<double, double>> errs;
    for (const auto& [fb, pb, db] : wb) {
        const g::Pose ab = fb.align.pose();
        const g::Vec3 pos = ab.apply(pb);
        const g::Vec3 dir = g::mat_vec(ab.r, db);
        const ARoom* best = nullptr;
        double bd = 0;
        for (const auto& x : wa_room) {
            const double dd = g::norm(g::sub(x.p, pos));
            if (dd <= 0.5 && g::angle_deg(x.d, dir) <= 30.0) {
                if (best == nullptr || rs::total_cmp(dd, bd) < 0) {
                    best = &x;
                    bd = dd;
                }
            }
        }
        if (best != nullptr) {
            const Alignment c_b = loopclose::correction_at(cb, (fb.span_ns.first + fb.span_ns.second) / 2);
            const Alignment cyc = store::compose(store::compose(fb.align, store::inverse(c_b)), best->f->align);
            const g::Vec3 pa_slam = best->c.pose().inverse().apply(pos);
            const double at = g::norm(g::sub(cyc.pose().apply(pa_slam), best->c.pose().apply(pa_slam)));
            errs.emplace_back(at, std::fabs(rf::to_degrees(align::wrap(cyc.yaw - best->c.yaw))));
        }
    }
    if (errs.empty()) {
        std::printf("no same-place window pairs (the sessions do not overlap)\n");
        return {};
    }
    std::vector<double> t, d;
    for (const auto& e : errs) {
        t.push_back(e.first);
        d.push_back(e.second);
    }
    std::stable_sort(t.begin(), t.end(), rs::total_less);
    std::stable_sort(d.begin(), d.end(), rs::total_less);
    auto q = [](const std::vector<double>& x, double f) {
        return x[rs::to_usize(std::round(static_cast<double>(x.size() - 1) * f))];
    };
    std::printf("same-place cycles: %zu pairs | position at the place: median %s mm, p90 %s mm, max %s mm | yaw: median %s "
                "deg, p90 %s deg, max %s deg\n",
                errs.size(), rf::fixed(q(t, 0.5) * 1e3, 1).c_str(), rf::fixed(q(t, 0.9) * 1e3, 1).c_str(),
                rf::fixed(q(t, 1.0) * 1e3, 1).c_str(), rf::fixed(q(d, 0.5), 2).c_str(), rf::fixed(q(d, 0.9), 2).c_str(),
                rf::fixed(q(d, 1.0), 2).c_str());
    return {};
}

const char* HELP =
    "`pibridge-roommap`: persistent room map (save the room, recognise it next time, restore the world origin);\n"
    "the C++ port of omarchyvr-roommap (docs/pibridge-roommap.md).\n"
    "\n"
    "pibridge-roommap build      (--rec DIR [--tracking CSV] | --source broker [--seconds 60]) [--name NAME | --map DIR]\n"
    "[--every N] [--kf-dist M] [--kf-angle DEG] [--no-loops] [--refine [--keep-unrefined]]\n"
    "[--min-keyframes N --extend S] [--append]\n"
    "pibridge-roommap relocalize (--rec DIR [--tracking CSV] | --source broker [--seconds 15]) [--name NAME | --map DIR]\n"
    "[--every N] [--window S] [--threads N] [--result FILE] [--apply [--no-floor]]\n"
    "pibridge-roommap apply      [--name NAME | --map DIR] [--result FILE] [--pid N] [--dry-run] [--no-floor]\n"
    "pibridge-roommap info       [--name NAME | --map DIR]\n"
    "pibridge-roommap export     [--name NAME | --map DIR] [--out FILE.json]\n"
    "pibridge-roommap view       [--name NAME | --map DIR] [--out FILE.html] [--open]\n"
    "pibridge-roommap floor      detect|drift|touch|set|show|follow|preview ...   (pibridge-roommap floor: details)\n"
    "pibridge-roommap setup      fast|full|show [--name NAME] [--label TEXT]   (pibridge-roommap setup: details)\n"
    "pibridge-roommap auto       [--name NAME] [--wait S]\n"
    "pibridge-roommap light      [--source broker|FILE.pxcam] [--seconds 5] [--every N] [--no-stretch] [--darken F [--noise G]]\n"
    "pibridge-roommap cycle      RESULT.toml RESULT.toml [RESULT.toml ...]\n"
    "pibridge-roommap eval       --map-a A --rec-a DIR --map-b B --rec-b DIR [--every N] [--window S]\n"
    "common: --calib XML (default: the headset calibration); --backend steamvr|monado (required, or PIBRIDGE_BACKEND):\n"
    "        each backend has its own rooms (~/.local/share/pibridge/rooms/<backend>) and runtime files";

} // namespace

} // namespace pibridge::roommap::cli

int main(int argc, char** argv) {
    using namespace pibridge::roommap::cli;
    std::setvbuf(stdout, nullptr, _IOLBF, 0); // line by line, as Rust's stdout
    Args v(argv + 1, argv + argc);
    // --backend steamvr|monado anywhere on the line (else PIBRIDGE_BACKEND): whose rooms, runtime files and commit
    // this run uses (anchor/backend.h). It goes into the environment, so the library and every child see the same one.
    for (std::size_t i = 0; i < v.size(); i++) {
        if (v[i] == "--backend") {
            if (i + 1 >= v.size() || !pibridge::anchor::parse_backend(v[i + 1])) {
                std::fprintf(stderr, "pibridge-roommap: --backend takes steamvr or monado\n");
                return 2;
            }
            ::setenv(pibridge::anchor::BACKEND_ENV, v[i + 1].c_str(), 1);
            v.erase(v.begin() + static_cast<std::ptrdiff_t>(i), v.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            break;
        }
    }
    if (v.empty() || v[0] == "-h" || v[0] == "--help") {
        std::fprintf(stderr, "%s\n", HELP);
        std::fprintf(stderr, "map directory: %s\n", pibridge::roommap::store::base_dir().c_str());
        return 2;
    }
    {
        std::string bad;
        if (!pibridge::anchor::backend_from_env(&bad)) {
            std::fprintf(stderr, "pibridge-roommap: %s\n",
                         bad.empty() ? "which backend? --backend steamvr|monado (or PIBRIDGE_BACKEND): each keeps its own rooms"
                                     : ("PIBRIDGE_BACKEND=" + bad + " is neither steamvr nor monado").c_str());
            return 2;
        }
    }
    const std::string cmd = v[0];
    v.erase(v.begin());
    R<void> r;
    if (cmd == "build") {
        r = cmd_build(v);
    } else if (cmd == "relocalize" || cmd == "reloc") {
        r = cmd_relocalize(v);
    } else if (cmd == "apply") {
        r = cmd_apply(v);
    } else if (cmd == "info") {
        r = cmd_info(v);
    } else if (cmd == "export") {
        r = cmd_export(v);
    } else if (cmd == "view") {
        r = cmd_view(v);
    } else if (cmd == "cycle") {
        r = cmd_cycle(v);
    } else if (cmd == "eval") {
        r = cmd_eval(v);
    } else if (cmd == "floor") {
        r = cmd_floor(v);
    } else if (cmd == "light") {
        r = cmd_light(v);
    } else if (cmd == "setup") {
        r = cmd_setup(v);
    } else if (cmd == "auto") {
        r = cmd_auto(v);
    } else {
        r = std::unexpected("unknown command " + pibridge::cam::rfmt::debug_str(cmd) +
                            " (build, relocalize, apply, info, export, view, cycle, eval, floor, setup, auto)");
    }
    if (!r) {
        std::fprintf(stderr, "pibridge-roommap %s: %s\n", cmd.c_str(), r.error().c_str());
        return 1;
    }
    return 0;
}
