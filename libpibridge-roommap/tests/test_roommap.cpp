// SPDX-License-Identifier: MIT
// Twins of omarchyvr-roommap's library tests: tests.rs (joint fit, pose graph, corrections, result and offset files,
// the synthetic room end to end, loop closure, export, the 2026-10-17 floor-jump replay: 9 tests), floor.rs (6),
// setup.rs (9) and light.rs (4). The end-to-end ones use the shipped reference calibration (the Rust ones need the lab
// copy of the Dream Air calibration).
#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge/cam/calib.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/slam.h"
#include "pibridge/roommap/align.h"
#include "pibridge/roommap/exportmap.h"
#include "pibridge/roommap/floor.h"
#include "pibridge/roommap/light.h"
#include "pibridge/roommap/loopclose.h"
#include "pibridge/roommap/pipeline.h"
#include "pibridge/roommap/refine.h"
#include "pibridge/roommap/session.h"
#include "pibridge/roommap/setup.h"
#include "pibridge/roommap/store.h"
#include "pibridge/roommap/synth.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <numbers>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>
#include <utime.h>

using namespace pibridge;
using namespace pibridge::roommap;
namespace g = cam::geom;
namespace sl = cam::slam;
using g::Pose;
using g::Vec3;
using session::Alignment;

namespace {

constexpr double PI = std::numbers::pi;

struct Rng {
    std::uint64_t s;
    double next() {
        s += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return static_cast<double>((z ^ (z >> 31)) >> 11) / static_cast<double>(1ull << 53);
    }
    double sym() { return 2.0 * next() - 1.0; }
    double gauss() {
        const double u = std::fmax(next(), 1e-12), v = next();
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * PI * v);
    }
};

sl::Quat body_q(double yaw, double pitch) {
    return sl::qmul(sl::qyaw(yaw), {std::sin(pitch / 2.0), 0.0, 0.0, std::cos(pitch / 2.0)});
}

std::pair<double, double> align_err(const Alignment& a, const Alignment& b) {
    return {g::norm(g::sub(a.t, b.t)), std::fabs(align::wrap(a.yaw - b.yaw)) * 180.0 / PI};
}

std::string tmpdir(const std::string& tag) {
    const char* t = std::getenv("TMPDIR");
    const std::string d = std::string(t != nullptr ? t : "/tmp") + "/omarchyvr-" + tag + "-" + std::to_string(::getpid());
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    return d;
}

const std::size_t IDS[2] = {0, 1};

struct Cams {
    stereo::Rig rig;
    std::vector<synth::RayTable> rays;
    Cams() : rig(*stereo::Rig::from_calib(*cam::DeviceCalib::load(PIBRIDGE_REFERENCE_XML), IDS)) {
        for (const auto& c : rig.cams) {
            rays.push_back(synth::RayTable::make(c.intr));
        }
    }
    std::array<pngread::Grey, 2> render(const synth::Room& room, const Pose& rb) const {
        std::array<pngread::Grey, 2> out;
        for (std::size_t i = 0; i < 2; i++) {
            out[i] = pngread::Grey{640, 480, room.render_rays(rays[i], rb.compose(rig.cams[i].body_from_cam))};
        }
        return out;
    }
};

orb::Image image(const pngread::Grey& gr) {
    return orb::Image{gr.w, gr.h, gr.w, gr.data};
}

std::vector<Pose> walk(std::size_t n, double r, double phase) {
    std::vector<Pose> v;
    for (std::size_t i = 0; i < n; i++) {
        const double a = phase + static_cast<double>(i) / static_cast<double>(n) * (2.0 * PI);
        v.push_back(Pose{sl::mat_of(body_q(a + 0.5, -0.1)), {r * std::sin(a), 1.55, r * std::cos(a)}});
    }
    return v;
}

} // namespace

TEST(joint_fit_recovers_transform_with_noise_and_outliers) {
    Rng rng{1};
    const Alignment truth{0.7, {1.2, -0.05, -2.3}};
    const std::vector<Pose> bfc = {Pose{sl::mat_of(sl::qyaw(0.45)), {-0.05, 0.0, 0.0}},
                                   Pose{sl::mat_of(sl::qyaw(-0.45)), {0.05, 0.0, 0.0}}};
    std::vector<align::FrameObs> obs;
    for (int f = 0; f < 20; f++) {
        const Pose sb{sl::mat_of(body_q(0.1 * f, -0.2)), {0.05 * f, 1.5, 0.02 * f}};
        const Pose room_from_body = truth.pose().compose(sb);
        std::vector<pnp::Corr> corrs;
        for (int i = 0; i < 60; i++) {
            const std::size_t c = static_cast<std::size_t>(rng.next() * 2.0);
            const Pose rc = room_from_body.compose(bfc[c]);
            const double d = 1.0 + 4.0 * rng.next();
            const double x0 = rng.sym() * d * 0.6;
            const double x1 = rng.sym() * d * 0.5;
            const Vec3 pc{x0, x1, d};
            const Vec3 p_room = rc.apply(pc);
            Vec3 b = g::normalize(pc);
            const double n0 = rng.gauss() * 0.003;
            const double n1 = rng.gauss() * 0.003;
            b = g::normalize(g::add(b, {n0, n1, 0.0}));
            if (rng.next() < 0.2) {
                const double o0 = rng.sym();
                const double o1 = rng.sym();
                b = g::normalize({o0, o1, 1.0});
            }
            corrs.push_back(pnp::Corr{c, b, p_room});
        }
        const std::size_t n = corrs.size();
        align::FrameObs o;
        o.ts_ns = f * 100000000LL;
        o.slam_from_body = sb;
        o.corr_kf.assign(n, 0);
        o.corrs = corrs;
        for (std::size_t i = 0; i < n; i++) {
            o.inliers.push_back(i);
        }
        const double hy = truth.yaw + 0.01 * rng.sym();
        const double h0 = 0.05 * rng.sym();
        const double h2 = 0.05 * rng.sym();
        o.hyp = Alignment{hy, g::add(truth.t, {h0, 0.0, h2})};
        o.room_from_body = room_from_body;
        obs.push_back(o);
    }
    std::vector<const align::FrameObs*> refs;
    for (const auto& o : obs) {
        refs.push_back(&o);
    }
    const auto f = align::joint_fit(refs, bfc, align::JointParams{});
    CHECK(f.has_value());
    const auto [dm, dd] = align_err(f->align, truth);
    CHECK(dm < 0.005 && dd < 0.05);
    CHECK(f->inliers > 800);
    align::AcceptParams ap;
    ap.min_frames = 8;
    ap.min_inliers = 200;
    const auto ff = align::first_fix(obs, bfc, align::JointParams{}, ap);
    CHECK(ff && ff->first == 7);
    CHECK(ff && align_err(ff->second.align, truth).first < 0.01);
}

TEST(pose_graph_removes_injected_drift) {
    const std::size_t n = 40;
    std::vector<loopclose::Node> truth;
    for (std::size_t i = 0; i < n; i++) {
        const double a = static_cast<double>(i) / static_cast<double>(n) * 2.0 * PI;
        truth.emplace_back(a, Vec3{2.0 * std::sin(a), 0.0, 2.0 * (1.0 - std::cos(a))});
    }
    std::vector<loopclose::Node> est = {truth[0]};
    std::vector<loopclose::Edge> edges;
    for (std::size_t i = 1; i < n; i++) {
        const auto& [ya, ta] = truth[i - 1];
        const auto& [yb, tb] = truth[i];
        Vec3 dt = g::mat_vec(g::transpose(pnp::rot_y(ya)), g::sub(tb, ta));
        const double dyaw = align::wrap(yb - ya) + 0.3 * PI / 180.0;
        dt = g::add(dt, {0.004, 0.0, 0.0});
        edges.push_back(loopclose::Edge{i - 1, i, dyaw, dt, 1.0 / (0.01 * 0.01),
                                        1.0 / ((0.2 * PI / 180.0) * (0.2 * PI / 180.0)), false});
        const auto& [yp, tp] = est[i - 1];
        est.emplace_back(align::wrap(yp + dyaw), g::add(tp, g::mat_vec(pnp::rot_y(yp), dt)));
    }
    const double drift_before = g::norm(g::sub(est[n - 1].second, truth[n - 1].second));
    CHECK(drift_before > 0.2);
    const auto& [y0, t0] = truth[0];
    const auto& [yl, tl] = truth[n - 1];
    edges.push_back(loopclose::Edge{0, n - 1, align::wrap(yl - y0), g::mat_vec(g::transpose(pnp::rot_y(y0)), g::sub(tl, t0)),
                                    1.0 / (0.01 * 0.01), 1.0 / ((0.1 * PI / 180.0) * (0.1 * PI / 180.0)), true});
    loopclose::solve_graph(est, edges, 20, 2.0);
    const double after = g::norm(g::sub(est[n - 1].second, truth[n - 1].second));
    const double mid = g::norm(g::sub(est[n / 2].second, truth[n / 2].second));
    CHECK(after < 0.02);
    CHECK(mid < drift_before / 2.0);
}

TEST(correction_interpolates_and_clamps) {
    const std::vector<loopclose::Correction> c = {{0, Alignment{0.0, {0.0, 0.0, 0.0}}},
                                                  {10, Alignment{0.2, {1.0, 0.0, -1.0}}}};
    const auto m = loopclose::correction_at(c, 5);
    CHECK(std::fabs(m.yaw - 0.1) < 1e-12 && std::fabs(m.t[0] - 0.5) < 1e-12 && std::fabs(m.t[2] + 0.5) < 1e-12);
    CHECK(loopclose::correction_at(c, -5) == c[0].c);
    CHECK(loopclose::correction_at(c, 50) == c[1].c);
}

TEST(result_roundtrip_compose_and_offset_file) {
    store::RelocResult r;
    r.map = "/x/home";
    r.source = "rec";
    r.align = Alignment{0.3, {0.1, -0.2, 0.3}};
    r.frames = 12;
    r.inliers = 345;
    r.rms_deg = 0.4;
    r.sigma_deg = 0.02;
    r.sigma_m = 0.001;
    r.fix_after_s = 3.5;
    r.pid = 4242;
    const auto back = store::RelocResult::parse(r.to_toml());
    CHECK(back && back->pid == 4242 && back->frames == 12);
    const auto [dm, dd] = align_err(back->align, r.align);
    CHECK(dm < 1e-5 && dd < 1e-5);
    const auto id = store::compose(r.align, store::inverse(r.align));
    CHECK(g::norm(id.t) < 1e-12 && std::fabs(id.yaw) < 1e-12);
    const std::string dir = tmpdir("roommap-test");
    const std::string p = dir + "/pimax-anchor.offset";
    CHECK(anchor::take_lease(p, 4242, anchor::Role::Apply).has_value());
    CHECK(anchor::write_offset(p, 4242, store::offset_pose(r.align)).has_value());
    const auto o = anchor::read_offset(p);
    CHECK(o && o->first == 4242);
    const Pose q = r.align.pose();
    CHECK(g::norm(g::sub(o->second.t, q.t)) < 1e-5);
    const Vec3 x{0.3, 1.1, -0.7};
    CHECK(g::norm(g::sub(o->second.apply(x), q.apply(x))) < 1e-5);
    CHECK(std::fabs(o->second.r[1][1] - 1.0) < 1e-6);
    std::filesystem::remove_all(dir);
}

TEST(synthetic_room_second_session_relocalises) {
    const Cams cams;
    const synth::Room room;
    const Alignment id{};
    session::RoomSession s1(map::RoomMap::make("test", IDS), cams.rig, session::SessionParams{});
    const auto w1 = walk(30, 0.9, 0.0);
    for (std::size_t k = 0; k < w1.size(); k++) {
        const auto im = cams.render(room, w1[k]);
        const orb::Image v[2] = {image(im[0]), image(im[1])};
        s1.on_frame(static_cast<std::int64_t>(k) * 100000000, v, w1[k], id, true);
    }
    const auto m = std::move(s1).finish();
    CHECK(m.keyframes.size() >= 5);
    const Alignment truth{-0.8, {1.0, 0.0, -3.0}};
    const Pose slam_from_room = truth.pose().inverse();
    synth::Room dim;
    dim.gain = 0.6;
    const orb::OrbParams orb;
    std::vector<align::FrameObs> obs;
    const auto w2 = walk(30, 0.6, 0.1);
    for (std::size_t k = 0; k < 14; k++) {
        const auto im = cams.render(dim, w2[k]);
        const auto feats = pipeline::frame_features(cams.rig, im[0], im[1], orb);
        const Pose sb = slam_from_room.compose(w2[k]);
        if (auto o = align::observe(m, cams.rig, feats, static_cast<std::int64_t>(k) * 250000000, sb, align::ObserveParams{})) {
            obs.push_back(*o);
        }
    }
    CHECK(obs.size() >= 8);
    std::vector<const align::FrameObs*> refs;
    for (const auto& o : obs) {
        refs.push_back(&o);
    }
    const auto f = align::joint_fit(refs, cams.rig.body_from_cams(), align::JointParams{});
    CHECK(f.has_value());
    if (f) {
        const auto [dm, dd] = align_err(f->align, truth);
        std::printf("alignment error %.1f mm %.3f deg\n", dm * 1e3, dd);
        CHECK(dm < 0.01 && dd < 0.2);
    }
    synth::Room other;
    other.seed = 99;
    int hits = 0;
    for (std::size_t k = 0; k < 8; k++) {
        const auto im = cams.render(other, w2[k]);
        const auto feats = pipeline::frame_features(cams.rig, im[0], im[1], orb);
        const Pose sb = slam_from_room.compose(w2[k]);
        hits += align::observe(m, cams.rig, feats, static_cast<std::int64_t>(k), sb, align::ObserveParams{}).has_value();
    }
    CHECK(hits <= 1);
}

TEST(synthetic_loop_closure_removes_mapping_drift) {
    const Cams cams;
    const synth::Room room;
    const std::size_t n = 60;
    const auto truth = walk(n, 0.9, 0.0);
    std::vector<Pose> drifted;
    for (std::size_t i = 0; i < n; i++) {
        const double f = static_cast<double>(i) / static_cast<double>(n);
        const Alignment d{3.0 * f * PI / 180.0, {0.15 * f, 0.0, -0.10 * f}};
        drifted.push_back(d.pose().compose(truth[i]));
    }
    const Alignment id{};
    session::SessionParams sp;
    sp.kf_min_dist = 0.15;
    sp.kf_min_angle_deg = 12.0;
    sp.kf_min_points = 40;
    session::RoomSession s(map::RoomMap::make("test", IDS), cams.rig, sp);
    const orb::OrbParams orb;
    std::vector<refine::MapFrame> frames;
    const std::int64_t dt = 1000000000;
    for (std::size_t k = 0; k < n; k++) {
        const auto im = cams.render(room, truth[k]);
        const orb::Image v[2] = {image(im[0]), image(im[1])};
        s.on_frame(static_cast<std::int64_t>(k) * dt, v, drifted[k], id, true);
        frames.push_back(refine::MapFrame::make(static_cast<std::int64_t>(k) * dt, drifted[k],
                                                pipeline::frame_features(cams.rig, im[0], im[1], orb), cams.rig));
    }
    auto m = std::move(s).finish();
    const auto last = *std::max_element(m.keyframes.begin(), m.keyframes.end(),
                                        [](const auto& a, const auto& b) { return a.ts_ns < b.ts_ns; });
    const std::size_t k_last = static_cast<std::size_t>(last.ts_ns / dt);
    const double err_before = g::norm(g::sub(last.pose.t, truth[k_last].t));
    loopclose::LoopParams lp;
    lp.min_gap_ns = 12 * dt;
    lp.local_ns = 6 * dt;
    const auto [st, corr] = loopclose::close_loops(m, cams.rig, frames, align::ObserveParams{}, lp, 4);
    CHECK(st.loops >= 2);
    CHECK(!corr.empty());
    const auto after = std::find_if(m.keyframes.begin(), m.keyframes.end(), [&](const auto& k) { return k.ts_ns == last.ts_ns; });
    const double err_after = g::norm(g::sub(after->pose.t, truth[k_last].t));
    std::printf("end keyframe error %.3f -> %.3f m (%zu loops)\n", err_before, err_after, st.loops);
    CHECK(err_after < 0.5 * err_before && err_after < 0.04);
}

TEST(export_json_and_viewer_page) {
    auto m = map::RoomMap::make("UID", IDS);
    for (std::uint32_t i = 0; i < 3; i++) {
        map::Keyframe k;
        k.id = i;
        k.ts_ns = i;
        k.session = 0;
        k.pose = Pose{sl::mat_of(sl::qyaw(0.3)), {static_cast<double>(i), 1.2, 0.0}};
        for (int j = 0; j < 4; j++) {
            k.points.push_back(map::MapPoint{{i * 0.001 + j, 0.5 + j * 0.5, -1.0}, {}});
        }
        m.keyframes.push_back(k);
    }
    m.anchors.push_back(map::Anchor{"desk\"</x>", {1.0, 0.7, 2.0}, {0.0, 0.0, 0.0, 1.0}});
    floor::StoredFloor fl;
    fl.y = 0.05;
    fl.source = floor::FloorSource::Manual;
    fl.confidence = 0.9;
    fl.sigma_m = 0.01;
    fl.head_above_m = 1.2;
    fl.when = 1;
    const std::string j = exportmap::map_json(m, "t", &fl, std::string("source = \"x\"\ncreated = \"unix 5\"\n"));
    CHECK(j.find("\"raw_observations\":12,\"points\":4") != std::string::npos);
    CHECK(j.find("\"floor\":{\"y\":0.05,\"source\":\"manual\"") != std::string::npos);
    CHECK(j.find("\"columns\":[\"x\",\"y\",\"z\",\"n\"],\"data\":[0.001,0.5,-1,3,") != std::string::npos);
    std::size_t ids = 0;
    for (std::size_t p = j.find("\"id\":"); p != std::string::npos; p = j.find("\"id\":", p + 1)) {
        ids++;
    }
    CHECK(ids == 3);
    CHECK(j.find("\"q\":[0,0.149,0,0.989]") != std::string::npos);
    CHECK(j.find("source") != std::string::npos && j.find("unix 5") != std::string::npos);
    CHECK(j.find("</") == std::string::npos);
    const std::string page = exportmap::html_page(j, "t");
    CHECK(page.find("const DATA = {") != std::string::npos && page.find("/*__DATA__*/") == std::string::npos &&
          page.find("__TITLE__") == std::string::npos);
    CHECK(page.find("https://") == std::string::npos && page.find("http://") == std::string::npos);
}

namespace {

const char* JUMP_SETUP_TOML = "label = \"home\"\nmode = \"fast\"\ncentre = [0.8017, -0.5628]\nyaw_deg = 19.1557\nplay_area = [0.800, 0.800]\nhead_m = 1.7573\nwhen = 1792225029\nboundary = [[0.2926, -0.8094], [1.0483, -1.0720], [1.3272, -0.2690], [0.5715, -0.0065]]\n";
constexpr double JUMP_FLOOR_Y = -0.63519;
const Alignment JUMP_CORRECTION{-0.003655764, {-0.192141, -0.067361, 0.043380}};
const Alignment JUMP_RELOC{-0.971515 * PI / 180.0, {-0.284277, -0.069415, 0.139813}};

Alignment jump_offset(const Alignment& fix) {
    const auto st = *setup::RoomSetup::parse(JUMP_SETUP_TOML);
    const double d = floor::floor_shift(JUMP_FLOOR_Y, floor::MONADO_ORIGIN_HEIGHT);
    return store::compose(st.setup_from_room(), Alignment{fix.yaw, {fix.t[0], fix.t[1] + d, fix.t[2]}});
}

} // namespace

TEST(floor_jump_replay_setup_includes_the_loop_closure) {
    const Alignment id{};
    const auto raw = jump_offset(id);
    CHECK(std::fabs(raw.yaw * 180.0 / PI + 19.156) < 0.01 && g::norm(g::sub(raw.t, {-0.9420, -0.9648, 0.2686})) < 0.001);
    const auto au = jump_offset(JUMP_RELOC);
    CHECK(std::fabs(au.yaw * 180.0 / PI + 20.127) < 0.01 && g::norm(g::sub(au.t, {-1.2564, -1.0342, 0.3074})) < 0.001);
    const auto fixed = jump_offset(store::fix_after_mapping(id, JUMP_CORRECTION));
    const double d_raw = g::norm(g::sub(raw.t, au.t)), d_fixed = g::norm(g::sub(fixed.t, au.t));
    const double v_raw = std::fabs(raw.t[1] - au.t[1]), v_fixed = std::fabs(fixed.t[1] - au.t[1]);
    std::printf("floor jump: raw %.1f cm (%.1f vertical), with the loop closure %.1f cm (%.1f vertical)\n", d_raw * 100,
                v_raw * 100, d_fixed * 100, v_fixed * 100);
    CHECK(v_fixed < 0.01);
    CHECK(v_raw > 0.05 && d_fixed < d_raw / 2.0);
    const std::string dir = tmpdir("jump-corr");
    CHECK(store::save_corrections(dir, {{1, JUMP_CORRECTION}}).has_value());
    CHECK(cam::write_file(dir + "/map.bin", "x").has_value());
    const auto c = store::last_correction(dir);
    CHECK(c && std::fabs(c->t[1] - JUMP_CORRECTION.t[1]) < 1e-6);
    struct timeval tv[2];
    gettimeofday(&tv[0], nullptr);
    tv[0].tv_sec -= 600;
    tv[1] = tv[0];
    utimes((dir + "/correction.csv").c_str(), tv);
    CHECK(!store::last_correction(dir));
    std::filesystem::remove_all(dir);
}

TEST(floor_jump_replay_one_offset_only) {
    const std::string dir = tmpdir("jump-lease");
    const std::string off = dir + "/pimax-anchor.offset";
    const std::uint32_t monado = 1092498, au = 1092600, su = 1093007;
    auto never = [](std::uint32_t) { return false; };
    auto ok = [](std::uint32_t) -> std::expected<void, std::string> { return {}; };
    CHECK(anchor::take_lease_as(off, monado, au, anchor::Role::Auto, never, ok).has_value());
    std::uint32_t ended = 0;
    CHECK(anchor::take_lease_as(off, monado, su, anchor::Role::Setup, [&](std::uint32_t p) { return p == au; },
                                [&](std::uint32_t p) -> std::expected<void, std::string> {
                                    ended = p;
                                    return {};
                                })
              .has_value());
    CHECK(ended == au);
    const Alignment id{};
    const auto placed = jump_offset(store::fix_after_mapping(id, JUMP_CORRECTION));
    int writes = 0;
    if (anchor::write_offset_as(off, monado, su, store::offset_pose(placed))) {
        writes++;
    }
    const auto setup_text = cam::read_to_string(off);
    const auto second = jump_offset(JUMP_RELOC);
    const auto e = anchor::write_offset_as(off, monado, au, store::offset_pose(second));
    CHECK(!e && e.error().text().find("room setup") != std::string::npos);
    if (anchor::write_offset_as(off, monado, au, store::offset_pose(second))) {
        writes++;
    }
    const auto e2 = anchor::take_lease_as(off, monado, au, anchor::Role::Auto, [&](std::uint32_t p) { return p == au; }, ok);
    CHECK(!e2 && e2.error().find("already placed") != std::string::npos);
    CHECK(!anchor::take_lease_as(off, monado, 1099999, anchor::Role::Auto, never, ok));
    CHECK(writes == 1);
    CHECK(*cam::read_to_string(off) == *setup_text);
    std::filesystem::remove_all(dir);
}

// ---- floor.rs ----

namespace {

double rnd(std::uint64_t& s) {
    s = s * 6364136223846793005ull + 1442695040888963407ull;
    return static_cast<double>(s >> 11) / static_cast<double>(1ull << 53);
}

double gauss6(std::uint64_t& s) {
    double sum = -0.0;
    for (int i = 0; i < 6; i++) {
        sum += rnd(s);
    }
    return sum - 3.0;
}

std::pair<std::vector<Vec3>, std::vector<Vec3>> room_pts(std::uint64_t seed) {
    std::uint64_t s = seed;
    std::vector<Vec3> p;
    for (int i = 0; i < 900; i++) {
        const double x = rnd(s) * 5.0 - 2.5;
        const double y = -1.62 + 0.012 * gauss6(s) + 0.01 * (rnd(s) - 0.5);
        const double z = rnd(s) * 4.0 - 2.0;
        p.push_back({x, y, z});
    }
    for (int i = 0; i < 1500; i++) {
        const double x = rnd(s) * 1.4 + 0.3;
        const double y = -0.88 + 0.006 * gauss6(s);
        const double z = rnd(s) * 0.7 - 1.5;
        p.push_back({x, y, z});
    }
    for (int i = 0; i < 30; i++) {
        const double x = rnd(s) * 2.0;
        const double y = -2.3 + 0.01 * gauss6(s);
        const double z = rnd(s);
        p.push_back({x, y, z});
    }
    for (int i = 0; i < 3000; i++) {
        const double y = -1.62 + rnd(s) * 2.6;
        if (rnd(s) < 0.5) {
            const double z = rnd(s) * 4.0 - 2.0;
            p.push_back({2.5, y, z});
        } else {
            const double x = rnd(s) * 5.0 - 2.5;
            p.push_back({x, y, -2.0});
        }
    }
    std::vector<Vec3> heads;
    for (int i = 0; i < 40; i++) {
        heads.push_back({i * 0.05, 0.02 * gauss6(s), 0.0});
    }
    return {p, heads};
}

} // namespace

TEST(floor_under_a_desk_and_above_reflections) {
    for (std::uint64_t seed = 1; seed < 6; seed++) {
        const auto [p, h] = room_pts(seed);
        const auto f = floor::detect(p, h, floor::FloorParams{});
        CHECK(f.has_value());
        if (!f) {
            continue;
        }
        CHECK(std::fabs(f->layer.y + 1.62) < 0.005);
        CHECK(f->layer.tilt_deg < 1.0 && f->confidence > 0.5);
        CHECK(std::any_of(f->layers.begin(), f->layers.end(), [](const auto& l) { return std::fabs(l.y + 0.88) < 0.01; }));
        const auto t = floor::track(p, h, -1.58, 0.1, floor::FloorParams{});
        CHECK(t && std::fabs(t->layer.y + 1.62) < 0.005);
        CHECK(!floor::track(p, h, -1.3, 0.1, floor::FloorParams{}));
    }
}

TEST(tilted_tracker_floor_level_median) {
    std::uint64_t s = 9;
    const double a = std::tan(0.5 * PI / 180.0);
    std::vector<Vec3> p;
    for (int i = 0; i < 800; i++) {
        const double x = rnd(s) * 6.0 - 1.0;
        const double z = rnd(s) * 4.0 - 2.0;
        p.push_back({x, -1.5 + a * (x - 1.0) + 0.01 * gauss6(s), z});
    }
    const std::vector<Vec3> heads = {{1.0, 0.0, 0.0}};
    const auto f = floor::detect(p, heads, floor::FloorParams{});
    CHECK(f && std::fabs(f->layer.y - (-1.5 + a)) < 0.004);
    CHECK(f && std::fabs(f->layer.tilt_deg - 0.5) < 0.3);
}

TEST(no_floor_when_only_a_desk_is_seen) {
    std::uint64_t s = 3;
    std::vector<Vec3> p;
    for (int i = 0; i < 500; i++) {
        const double x = rnd(s);
        const double y = -0.3 + 0.005 * gauss6(s);
        const double z = rnd(s);
        p.push_back({x, y, z});
    }
    const std::vector<Vec3> heads = {{0.0, 0.0, 0.0}};
    CHECK(!floor::detect(p, heads, floor::FloorParams{}));
}

TEST(stored_floor_round_trip_and_shift) {
    floor::StoredFloor f;
    f.y = -1.234;
    f.source = floor::FloorSource::Touch;
    f.confidence = 0.9;
    f.sigma_m = 0.004;
    f.inliers = 0;
    f.head_above_m = 1.6;
    f.when = 7;
    const auto b = floor::StoredFloor::parse(f.to_toml());
    CHECK(b && b->y == -1.234 && b->source == floor::FloorSource::Touch && b->confidence == 0.9 && b->sigma_m == 0.004 &&
          b->head_above_m == 1.6 && b->when == 7);
    CHECK(std::fabs(floor::floor_shift(-0.45, 1.6) + 1.15) < 1e-12);
}

TEST(follower_is_smooth_and_robust) {
    floor::Follower f;
    f.measure(0.30, 0.0);
    CHECK(f.step(0.0) == 0.30);
    double last = 0.30;
    for (int i = 0; i < 90 * 30; i++) {
        if (i % 45 == 0) {
            f.measure(0.32, 0.5);
        }
        const double a = *f.step(1.0 / 90.0);
        CHECK(std::fabs(a - last) <= 0.01 / 90.0 + 1e-12);
        last = a;
    }
    CHECK(std::fabs(last - 0.32) < 0.003);
    f.measure(0.80, 0.5);
    CHECK(std::fabs(*f.target - 0.32) < 0.003);
    f.measure(0.81, 0.5);
    f.measure(0.80, 0.5);
    CHECK(std::fabs(*f.target - 0.80) < 0.011);
}

TEST(touch_and_combine) {
    std::vector<Vec3> o;
    for (int i = 0; i < 30; i++) {
        o.push_back({0.5, -1.55 + 0.0005 * (i % 3), 0.2});
    }
    const auto t = floor::touch_floor(o, 0.035, 0.01);
    CHECK(t && std::fabs(t->first + 1.585) < 0.002 && t->second < 0.002);
    std::vector<Vec3> moving;
    for (int i = 0; i < 30; i++) {
        moving.push_back({0.5 + 0.01 * i, -1.55, 0.2});
    }
    CHECK(!floor::touch_floor(moving, 0.035, 0.01));
    CHECK((floor::combine(std::pair{-1.60, 0.003}, std::pair{-1.50, 0.01}, 0.05) == std::pair{-1.50, 0.01}));
    const auto m = floor::combine(std::pair{-1.60, 0.01}, std::pair{-1.58, 0.01}, 0.05);
    CHECK(m && std::fabs(m->first + 1.59) < 1e-9);
}

// ---- setup.rs ----

namespace {

struct Script : setup::Guide {
    double t = 0.0;
    std::function<setup::Sample(double)> f;
    std::vector<std::string> said;
    void say(const std::string& title, const std::string& text, std::optional<float>) override {
        said.push_back(title + ": " + text);
    }
    setup::Sample sample() override {
        t += 0.02;
        setup::Sample s = f(t);
        s.t = t;
        return s;
    }
    bool said_any(const char* what) const {
        return std::any_of(said.begin(), said.end(), [&](const std::string& s) { return s.find(what) != std::string::npos; });
    }
};

setup::RoomSetup mk_setup(setup::P2 c, double yaw, setup::SetupMode mode = setup::SetupMode::Fast) {
    setup::RoomSetup s;
    s.label = "x";
    s.mode = mode;
    s.centre = c;
    s.yaw = yaw;
    s.boundary = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    s.play_area = {1.0, 1.0};
    s.head_m = 1.6;
    s.when = 0;
    return s;
}

} // namespace

TEST(frames_round_trip_and_heading) {
    const setup::P2 c{1.0, -2.0};
    const double yaw = 0.7;
    const setup::P2 p{0.3, 0.9};
    const auto q = setup::to_room(c, yaw, setup::to_setup(c, yaw, p));
    CHECK(setup::dist(p, q) < 1e-12);
    const auto f = setup::to_room({0.0, 0.0}, yaw, {0.0, -1.0});
    CHECK(std::fabs(setup::heading(f[0], f[1]) - yaw) < 1e-12);
    const auto s = mk_setup(c, yaw);
    const Vec3 r = s.room_from_setup().pose().apply({p[0], 0.0, p[1]});
    const auto q2 = setup::to_room(c, yaw, p);
    CHECK(std::fabs(r[0] - q2[0]) < 1e-12 && std::fabs(r[2] - q2[1]) < 1e-12);
    const Vec3 back = s.setup_from_room().pose().apply(r);
    CHECK(std::fabs(back[0] - p[0]) < 1e-12 && std::fabs(back[2] - p[1]) < 1e-12);
}

TEST(final_offset_puts_centre_on_the_origin_and_floor_at_zero) {
    const Alignment a{0.3, {0.5, 0.1, -0.2}};
    const double floor_y = -1.4;
    const double dy = floor::floor_shift(floor_y, 1.6);
    const auto s = mk_setup({2.0, 1.0}, 1.1, setup::SetupMode::Full);
    const auto off = setup::final_offset(a, dy, &s);
    const Vec3 slam_pt = a.pose().inverse().apply({2.0, floor_y, 1.0});
    const Vec3 app = off.pose().apply(slam_pt);
    CHECK(std::fabs(app[0]) < 1e-9 && std::fabs(app[2]) < 1e-9 && std::fabs(app[1] + 1.6) < 1e-9);
    const auto f = setup::to_room(s.centre, s.yaw, {0.0, -1.0});
    const Vec3 app_f = off.pose().apply(a.pose().inverse().apply({f[0], floor_y, f[1]}));
    CHECK(std::fabs(app_f[0]) < 1e-9 && std::fabs(app_f[2] + 1.0) < 1e-9);
    const auto o2 = setup::final_offset(a, dy, nullptr);
    CHECK(std::fabs(o2.t[1] - (a.t[1] + dy)) < 1e-12 && o2.yaw == a.yaw);
}

TEST(toml_round_trip) {
    setup::RoomSetup s;
    s.label = "Living \"room\"";
    s.mode = setup::SetupMode::Full;
    s.centre = {0.25, -1.5};
    s.yaw = 0.5;
    s.boundary = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 3.0}, {0.0, 3.0}};
    s.play_area = {1.8, 2.6};
    s.head_m = 1.65;
    s.when = 1792000000;
    const auto r = setup::RoomSetup::parse(s.to_toml());
    CHECK(r && r->label == "Living  room " && r->mode == setup::SetupMode::Full && r->boundary == s.boundary);
    CHECK(r && std::fabs(r->yaw - 0.5) < 1e-6 && r->centre == s.centre && r->play_area == s.play_area && r->when == s.when);
    CHECK(!setup::RoomSetup::parse("centre = [0, 0]\nyaw_deg = 0\nboundary = [[0,0],[1,0]]\n"));
    CHECK(r && r->describe().find("1.8 x 2.6 m") != std::string::npos);
}

TEST(play_area_of_a_square_an_l_and_an_offset_centre) {
    const std::vector<setup::P2> sq = {{-1.5, -1.0}, {1.5, -1.0}, {1.5, 1.0}, {-1.5, 1.0}};
    const auto pa = setup::play_area(sq);
    CHECK(std::fabs(pa[0] - 3.0) < 0.12 && std::fabs(pa[1] - 2.0) < 0.12);
    const std::vector<setup::P2> l = {{-2.0, -2.0}, {2.0, -2.0}, {2.0, 0.5}, {0.5, 0.5}, {0.5, 2.0}, {-2.0, 2.0}};
    const auto pl = setup::play_area(l);
    CHECK(setup::rect_inside(l, pl[0] / 2.0 - 1e-6, pl[1] / 2.0 - 1e-6));
    CHECK(pl[0] <= 1.0 + 1e-6 || pl[1] <= 1.0 + 1e-6);
    CHECK(pl[0] * pl[1] > 1.9);
    const std::vector<setup::P2> away = {{1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}, {1.0, 2.0}};
    CHECK((setup::play_area(away) == std::array<double, 2>{0.0, 0.0}));
}

TEST(boundary_cleanup) {
    std::vector<setup::P2> tr;
    const setup::P2 corners[5] = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}};
    std::uint32_t k = 0;
    for (int w = 0; w < 4; w++) {
        const auto a = corners[w], b = corners[w + 1];
        const std::size_t n = static_cast<std::size_t>(setup::dist(a, b) / 0.02);
        for (std::size_t i = 0; i < n; i++) {
            const double f = static_cast<double>(i) / static_cast<double>(n);
            k = k * 1103515245u + 12345u;
            const double jit = static_cast<double>((k >> 16) % 100) / 100.0 * 0.02 - 0.01;
            tr.push_back({a[0] + (b[0] - a[0]) * f + jit, a[1] + (b[1] - a[1]) * f - jit});
        }
    }
    const auto b = setup::clean_boundary(tr, 0.05);
    CHECK(b && b->size() >= 4 && b->size() <= 8);
    CHECK(b && std::fabs(std::fabs(setup::area(*b)) - 6.0) < 0.3);
    CHECK(b && !setup::self_intersects(*b));
    const std::vector<setup::P2> eight = {{0.0, 0.0}, {2.0, 2.0}, {2.0, 0.0}, {0.0, 2.0}};
    CHECK(setup::self_intersects(eight));
    std::vector<setup::P2> tr8;
    const setup::P2 e5[5] = {eight[0], eight[1], eight[2], eight[3], eight[0]};
    for (int w = 0; w < 4; w++) {
        for (int i = 0; i < 50; i++) {
            const double f = i / 50.0;
            tr8.push_back({e5[w][0] + (e5[w + 1][0] - e5[w][0]) * f, e5[w][1] + (e5[w + 1][1] - e5[w][1]) * f});
        }
    }
    const auto b8 = setup::clean_boundary(tr8, 0.05);
    CHECK(b8 && !setup::self_intersects(*b8) && std::fabs(std::fabs(setup::area(*b8)) - 4.0) < 0.2);
    CHECK(setup::clean_boundary({{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}}, 0.05).error().find("too short") != std::string::npos);
    std::vector<setup::P2> small;
    for (int i = 0; i < 200; i++) {
        const double a = i / 200.0 * 2.0 * PI;
        small.push_back({0.4 * std::cos(a), 0.4 * std::sin(a)});
    }
    CHECK(setup::clean_boundary(small, 0.05).error().find("m^2") != std::string::npos);
}

TEST(free_area_stops_at_furniture_and_the_map_edge) {
    const auto r0 = setup::free_area({}, std::nullopt, 2.5);
    CHECK(std::fabs(std::fabs(setup::area(r0)) - 4.0) < 1e-9);
    std::vector<setup::P2> obs;
    for (int i = 0; i < 40; i++) {
        obs.push_back({-1.0 + i * 0.05, -1.25});
        obs.push_back({2.02, -2.0 + i * 0.1});
    }
    const auto r = setup::free_area(obs, std::array<double, 4>{-3.0, 3.0, -3.0, 1.6}, 2.5);
    const double x0 = r[0][0], x1 = r[1][0], z0 = r[0][1], z1 = r[2][1];
    CHECK(z0 > -1.26 && z0 < -1.1);
    CHECK(x1 > 1.85 && x1 < 2.02);
    CHECK(z1 <= 1.6 + 1e-9 && z1 > 1.5);
    CHECK(std::fabs(x0 + 2.5) < 1e-9);
    const std::vector<std::array<double, 3>> pts = {{0.0, -1.4, -1.0}, {0.0, -0.8, -1.0}, {0.0, 1.0, -1.0}, {9.0, -0.8, 0.0}};
    const auto [o, e] = setup::obstacles_from_points(pts, -1.4, {0.0, 0.0}, 0.0);
    CHECK((o == std::vector<setup::P2>{{0.0, -1.0}}));
    CHECK(!e);
}

TEST(walls_like_steamvr) {
    const auto w = setup::walls({{-0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}, {0.5, -0.5}}, setup::WALL_HEIGHT_M);
    CHECK(w.size() == 4);
    const std::array<std::array<double, 3>, 4> w0 = {{{-0.5, 0.0, -0.5}, {-0.5, 2.43, -0.5}, {-0.5, 2.43, 0.5}, {-0.5, 0.0, 0.5}}};
    CHECK(w[0] == w0);
    CHECK((w[3][2] == std::array<double, 3>{-0.5, 2.43, -0.5}));
}

TEST(forward_by_trigger_and_by_holding_still) {
    Script g;
    g.f = [](double t) {
        setup::Sample s;
        s.head = std::pair{std::array<double, 3>{0.4, 1.7, -0.3}, -PI / 2.0};
        s.hands[1] = setup::HandSample{std::array<double, 3>{0.0, 0.0, 0.0}, t > 1.5 ? 1.0 : 0.0, true};
        return s;
    };
    const auto r = setup::confirm_forward(g, 30.0);
    CHECK(r && (std::get<0>(*r) == setup::P2{0.4, -0.3}));
    CHECK(r && std::fabs(std::get<1>(*r) + PI / 2.0) < 1e-12 && std::fabs(std::get<2>(*r) - 1.7) < 1e-12);
    CHECK(g.t < 1.6);
    Script g2;
    g2.f = [](double) {
        setup::Sample s;
        s.head = std::pair{std::array<double, 3>{0.0, 1.6, 0.0}, 0.2};
        s.hands[0] = setup::HandSample{std::nullopt, 1.0, true};
        return s;
    };
    const auto r2 = setup::confirm_forward(g2, 30.0);
    CHECK(r2 && std::fabs(std::get<1>(*r2) - 0.2) < 1e-12);
    CHECK(g2.t >= 3.0 && g2.t < 3.2);
    CHECK(g2.said_any("Hold still"));
    Script g3;
    g3.f = [](double t) {
        setup::Sample s;
        s.head = std::pair{std::array<double, 3>{t, 1.6, 0.0}, t};
        return s;
    };
    CHECK(setup::confirm_forward(g3, 10.0).error().find("run the setup again") != std::string::npos);
}

TEST(trace_closes_by_release_and_by_itself) {
    const double per = 7.0;
    auto at = [per](double d) -> std::array<double, 3> {
        d = std::fmod(std::fmod(d, per) + per, per);
        double x, z;
        if (d < 2.0) {
            x = d;
            z = 0.0;
        } else if (d < 3.5) {
            x = 2.0;
            z = d - 2.0;
        } else if (d < 5.5) {
            x = 5.5 - d;
            z = 1.5;
        } else {
            x = 0.0;
            z = 7.0 - d;
        }
        return {x, 1.0, z};
    };
    Script g;
    g.f = [&](double t) {
        const double d = std::fmax(t - 1.0, 0.0) * 0.5;
        const bool held = t > 1.0 && d < per - 0.5;
        setup::Sample s;
        s.hands[1] = setup::HandSample{at(d), held ? 1.0 : 0.0, true};
        return s;
    };
    const auto p = setup::trace_boundary(g, 60.0);
    CHECK(p.has_value());
    const auto b = setup::clean_boundary(*p, 0.05);
    CHECK(b && std::fabs(std::fabs(setup::area(*b)) - 3.0) < 0.3);
    Script g2;
    g2.f = [&](double t) {
        const double d = std::fmax(t - 1.0, 0.0) * 0.5;
        setup::Sample s;
        s.hands[0] = setup::HandSample{at(d), t > 1.0 ? 1.0 : 0.0, true};
        return s;
    };
    const auto p2 = setup::trace_boundary(g2, 60.0);
    CHECK(p2 && g2.t < 1.0 + per / 0.5 + 0.5);
    CHECK(p2 && setup::dist(p2->front(), p2->back()) < 0.3);
    Script g3;
    g3.f = [&](double t) {
        const double d = std::fmax(t - 1.0, 0.0) * 0.5;
        const bool held = (t >= 1.0 && t < 2.0) || t > 3.0;
        setup::Sample s;
        s.hands[0] = setup::HandSample{at(d), held ? 1.0 : 0.0, true};
        return s;
    };
    CHECK(setup::trace_boundary(g3, 60.0).has_value());
    CHECK(g3.said_any("too short"));
    Script g4;
    g4.f = [](double) { return setup::Sample{}; };
    CHECK(setup::trace_boundary(g4, 5.0).error().find("no controller") != std::string::npos);
}

// ---- light.rs ----

TEST(advice_and_text) {
    auto q = [](double mean, std::size_t stereo) {
        light::FrameQuality f;
        f.mean = mean;
        f.stereo = stereo;
        return f;
    };
    CHECK(light::advice(q(8.0, 0), 40) == light::Advice::TooDark);
    CHECK(light::advice(q(60.0, 10), 40) == light::Advice::TooPlain);
    CHECK(light::advice(q(8.0, 55), 40) == light::Advice::Good);
    light::MapProgress p{3.0, 4, q(8.0, 0), light::Advice::TooDark, false};
    CHECK(light::progress_text(p) == "4 views mapped, 0 points in view. Too dark for the cameras: turn on the room lights.");
    p.advice = light::Advice::Good;
    p.move_more = true;
    CHECK(light::progress_text(p).find("Turn your head further") != std::string::npos);
    const std::vector<light::FrameQuality> qs = {q(8.0, 0), q(10.0, 50), q(12.0, 60)};
    const auto r = light::summarise(qs, 40);
    CHECK(r.frames == 3 && r.mean == 10.0 && r.stereo == 50 && r.good == 2);
}

TEST(mean_ignores_lens_corners_and_clipped_cells) {
    const std::size_t w = 640, h = 480;
    std::vector<std::uint8_t> d(w * h, 80);
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const double dx = (static_cast<double>(x) - 319.5) / 320.0, dy = (static_cast<double>(y) - 239.5) / 240.0;
            if (dx * dx + dy * dy > 1.0) {
                d[y * w + x] = 0;
            }
            if (x > 440 && y < 160) {
                d[y * w + x] = 255;
            }
        }
    }
    double plain = 0;
    for (auto v : d) {
        plain += v;
    }
    plain /= static_cast<double>(d.size());
    const pngread::Grey gr{w, h, d};
    const auto q = light::quality_of(Cams().rig, {stereo::Features{}, stereo::Features{}}, gr, gr);
    CHECK(plain < 90.0);
    CHECK(std::fabs(q.mean - 80.0) < 2.0);
    CHECK(q.sat > 0.02 && q.sat < 0.2);
}

TEST(head_motion) {
    auto at = [](double yaw, double x) { return Pose{g::rodrigues({0.0, yaw * PI / 180.0, 0.0}), {x, 1.6, 0.0}}; };
    const std::vector<Pose> a = {at(0.0, 0.0), at(0.05, 0.001)};
    const std::vector<Pose> b = {at(0.0, 0.0), at(12.0, 0.0)};
    const std::vector<Pose> c = {at(0.0, 0.0), at(0.0, 0.2)};
    CHECK(!light::moved(a, 8.0, 0.1));
    CHECK(light::moved(b, 8.0, 0.1));
    CHECK(light::moved(c, 8.0, 0.1));
    CHECK(!light::moved(std::vector<Pose>{}, 8.0, 0.1));
}

TEST(darken_is_deterministic_and_dims) {
    pngread::Grey a{4, 1, {200, 100, 0, 255}};
    pngread::Grey b = a;
    light::darken(a, 0.25f, 0.0f, 1);
    light::darken(b, 0.25f, 0.0f, 1);
    CHECK((a.data == std::vector<std::uint8_t>{50, 25, 0, 64}));
    CHECK(a.data == b.data);
}

// PiBridge: SteamVR's standing frame (floor.h SteamVrStage) and the offset chain under driver_monado
TEST(pibridge_steamvr_stage_puts_the_floor_and_centre_into_the_standing_frame) {
    // raw_from_stage: yaw 0.2 rad, origin (0.0235, -1.521, -0.0153) (the live universe 2 of 2026-10-19)
    const double a = 0.2;
    const g::Vec3 t{0.0235, -1.521, -0.0153};
    // standing_from_raw = inverse: R = Ry(-a), t' = -R t
    const double c = std::cos(-a), sn = std::sin(-a);
    const double R[3][3] = {{c, 0, sn}, {0, 1, 0}, {-sn, 0, c}};
    std::string m;
    // row major [R | -R t]
    m = "pibridge-steamvr-stage 1\nvrserver 4242\nuniverse 2\nstanding_from_raw";
    for (int r = 0; r < 3; r++) {
        double tr = 0;
        for (int k = 0; k < 3; k++) {
            tr -= R[r][k] * t[std::size_t(k)];
        }
        for (int k = 0; k < 3; k++) {
            char b[32];
            std::snprintf(b, sizeof b, " %.9f", R[r][k]);
            m += b;
        }
        char b[32];
        std::snprintf(b, sizeof b, " %.9f", tr);
        m += b;
    }
    m += "\n";
    const auto st = floor::parse_steamvr_stage(m);
    CHECK(st && st->pid == 4242 && st->universe == 2);
    CHECK(std::fabs(st->yaw - a) < 1e-6 && st->tilt_deg < 1e-3);
    for (int k = 0; k < 3; k++) {
        CHECK(std::fabs(st->t[std::size_t(k)] - t[std::size_t(k)]) < 1e-6);
    }
    CHECK(!floor::parse_steamvr_stage("pibridge-steamvr-stage 1\nvrserver 1\n"));
    CHECK(!floor::parse_steamvr_stage("garbage"));
    // the chain of write_offset: floor shift with SteamVR's standing height, the setup, then the stage's yaw and xz
    const double floor_y = -1.30; // the room frame (= this session's tracker world)
    const session::Alignment room_from_slam{0.0, {0.0, 0.0, 0.0}};
    const double oh = -st->t[1];
    session::Alignment al{room_from_slam.yaw, {0.0, floor::floor_shift(floor_y, oh), 0.0}};
    setup::RoomSetup rs;
    rs.centre = {0.4, -0.3};
    rs.yaw = 0.5;
    al = store::compose(rs.setup_from_room(), al);
    al = store::compose(session::Alignment{st->yaw, {st->t[0], 0.0, st->t[2]}}, al);
    // a floor point under the centre and the head there, through offset (app = SteamVR raw) and standing_from_raw
    const auto standing = [&](g::Vec3 p) {
        const g::Vec3 raw = al.pose().apply(p);
        g::Vec3 o{0, 0, 0};
        for (int r = 0; r < 3; r++) {
            double v = 0;
            for (int k = 0; k < 3; k++) {
                v += R[r][k] * (raw[std::size_t(k)] - t[std::size_t(k)]);
            }
            o[std::size_t(r)] = v;
        }
        return o;
    };
    const g::Vec3 f = standing({0.4, floor_y, -0.3});
    CHECK(std::fabs(f[0]) < 1e-6 && std::fabs(f[1]) < 1e-6 && std::fabs(f[2]) < 1e-6);
    // the room's forward (the setup yaw) is the standing -Z
    const g::Vec3 ahead = standing({0.4 - std::sin(0.5), floor_y, -0.3 - std::cos(0.5)});
    const g::Vec3 ahead2 = standing({0.4 + std::sin(0.5), floor_y, -0.3 - std::cos(0.5)});
    CHECK((std::fabs(ahead[0]) < 1e-6 && std::fabs(ahead[2] + 1) < 1e-6) ||
          (std::fabs(ahead2[0]) < 1e-6 && std::fabs(ahead2[2] + 1) < 1e-6));
    // not a vrserver: Monado's own 1.6 m (the Rust rule) is kept
    CHECK(!floor::is_vrserver(static_cast<std::uint32_t>(getpid())));
    CHECK(std::fabs(floor::origin_height(static_cast<std::uint32_t>(getpid())) - 1.6) < 1e-9 ||
          std::getenv("XRT_TRACKING_ORIGIN_OFFSET_Y") != nullptr);
}

PIBRIDGE_TEST_MAIN()
