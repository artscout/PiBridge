// SPDX-License-Identifier: MIT
// Twins of omarchyvr-room tests.rs (end to end on the synthetic room through a factory calibration: 6 tests; the
// #[ignore] `debug_stereo_potential` is left out) and pngread.rs's test. The Rust tests need the lab copy of the Dream
// Air calibration; the twins use the shipped reference (PIBRIDGE_REFERENCE_XML).
#include "pibridge/cam/calib.h"
#include "pibridge/cam/slam.h"
#include "pibridge/roommap/map.h"
#include "pibridge/roommap/pngread.h"
#include "pibridge/roommap/session.h"
#include "pibridge/roommap/stereo.h"
#include "pibridge/roommap/synth.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numbers>
#include <vector>
#include <unistd.h>
#include <zlib.h>

using namespace pibridge;
using namespace pibridge::roommap;
namespace g = cam::geom;
namespace sl = cam::slam;
using g::Pose;
using g::Vec3;

namespace {

cam::DeviceCalib calib() {
    return *cam::DeviceCalib::load(PIBRIDGE_REFERENCE_XML);
}

sl::Quat body_q(double yaw, double pitch) {
    return sl::qmul(sl::qyaw(yaw), {std::sin(pitch / 2.0), 0.0, 0.0, std::cos(pitch / 2.0)});
}

Pose pose(const Vec3& p, const sl::Quat& q) {
    return Pose{sl::mat_of(q), p};
}

const std::size_t IDS[2] = {0, 1};

struct Cams {
    stereo::Rig rig;
    std::vector<synth::RayTable> rays;
    Cams() : rig(*stereo::Rig::from_calib(calib(), IDS)) {
        for (const auto& c : rig.cams) {
            rays.push_back(synth::RayTable::make(c.intr));
        }
    }
    std::vector<std::vector<std::uint8_t>> render(const synth::Room& room, const Pose& rb) const {
        std::vector<std::vector<std::uint8_t>> v;
        for (std::size_t i = 0; i < rig.cams.size(); i++) {
            v.push_back(room.render_rays(rays[i], rb.compose(rig.cams[i].body_from_cam)));
        }
        return v;
    }
};

orb::Image img(const std::vector<std::uint8_t>& d) {
    return orb::Image{640, 480, 640, d};
}

std::vector<std::pair<Vec3, sl::Quat>> walk(std::size_t n, double r, double phase) {
    std::vector<std::pair<Vec3, sl::Quat>> v;
    for (std::size_t i = 0; i < n; i++) {
        const double a = phase + static_cast<double>(i) / static_cast<double>(n) * (2.0 * std::numbers::pi);
        const Vec3 p{r * std::sin(a), 1.55 + 0.05 * std::sin(3.0 * a), r * std::cos(a)};
        v.emplace_back(p, body_q(a + std::numbers::pi * 0.15, -0.2 + 0.1 * std::sin(2.0 * a)));
    }
    return v;
}

std::vector<std::uint8_t> noisy(std::vector<std::uint8_t> v, std::uint64_t seed) {
    std::uint64_t x = seed | 1;
    for (auto& p : v) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        const int n = static_cast<int>(x >> 61) - 3;
        p = static_cast<std::uint8_t>(std::clamp(static_cast<int>(p) + n, 0, 255));
    }
    return v;
}

} // namespace

TEST(stereo_depth_matches_the_room) {
    const Cams cams;
    const synth::Room room;
    const Pose rb = pose({0.3, 1.6, 0.5}, body_q(0.4, -0.25));
    const auto ims = cams.render(room, rb);
    const orb::OrbParams o;
    const auto fa = stereo::features(img(ims[0]), cams.rig.cams[0], o);
    const auto fb = stereo::features(img(ims[1]), cams.rig.cams[1], o);
    const auto pts = stereo::triangulate(fa, fb, cams.rig.cams[0], cams.rig.cams[1], stereo::StereoParams{});
    CHECK(pts.size() >= 80);
    const Pose c0 = rb.compose(cams.rig.cams[0].body_from_cam);
    std::vector<double> rel;
    for (const auto& s : pts) {
        const auto truth = room.depth(c0, fa.bearings[s.ia]);
        if (!truth) {
            continue;
        }
        const double est = g::norm(g::sub(rb.apply(s.p_body), c0.t));
        rel.push_back(std::fabs((est - *truth) / *truth));
    }
    std::sort(rel.begin(), rel.end());
    const double med = rel[rel.size() / 2], p90 = rel[rel.size() * 9 / 10];
    std::printf("stereo: %zu points, depth error median %.1f %%, p90 %.1f %%\n", pts.size(), med * 100.0, p90 * 100.0);
    CHECK(med < 0.04 && p90 < 0.15);
}

TEST(second_session_relocalizes_into_the_same_room) {
    const Cams cams;
    const synth::Room room;
    const auto cal = calib();
    session::SessionParams p;
    p.kf_min_dist = 0.25;
    p.kf_min_angle_deg = 20.0;
    session::RoomSession s1(map::RoomMap::make(cal.uid, IDS), cams.rig, p);
    const session::Alignment ident{};
    const auto w1 = walk(24, 0.9, 0.0);
    for (std::size_t k = 0; k < w1.size(); k++) {
        const Pose rb = pose(w1[k].first, w1[k].second);
        const auto ims = cams.render(room, rb);
        const orb::Image views[2] = {img(ims[0]), img(ims[1])};
        s1.on_frame(static_cast<std::int64_t>(k) * 40000000, views, rb, ident, true);
    }
    auto m = std::move(s1).finish();
    std::printf("session 1: %zu keyframes, %zu points\n", m.keyframes.size(), m.points());
    CHECK(m.keyframes.size() >= 12);
    const char* t = std::getenv("TMPDIR");
    const std::string dir = std::string(t != nullptr ? t : "/tmp") + "/omarchyvr-room-test-" + std::to_string(::getpid());
    CHECK(m.save(dir).has_value());
    auto loaded = map::RoomMap::load(dir);
    std::filesystem::remove_all(dir);
    CHECK(loaded && loaded->sessions == 1);
    synth::Room room2;
    room2.gain = 0.7;
    const Pose slam_from_room{sl::mat_of(sl::qyaw(1.1)), {2.0, 0.0, -1.0}};
    const Pose true_room_from_slam = slam_from_room.inverse();
    session::RoomSession s2(std::move(*loaded), cams.rig, p);
    std::optional<std::pair<std::size_t, session::Alignment>> fixed;
    const auto w2 = walk(24, 0.75, 0.13);
    for (std::size_t k = 0; k < 10; k++) {
        const Pose rb = pose(w2[k].first, w2[k].second);
        const Pose sb = slam_from_room.compose(rb);
        const auto ims = cams.render(room2, rb);
        const orb::Image views[2] = {img(ims[0]), img(ims[1])};
        for (const auto& e : s2.on_frame(static_cast<std::int64_t>(k) * 250000000, views, sb, ident, true)) {
            if (e.kind == session::Event::Kind::Relocalized && !fixed) {
                fixed = std::pair{k, e.align};
            }
        }
    }
    CHECK(fixed.has_value());
    if (!fixed) {
        return;
    }
    const Pose est = fixed->second.pose();
    const double dt = g::norm(g::sub(est.t, true_room_from_slam.t));
    const double dr = sl::qangle_deg(sl::qmul(sl::qconj(sl::quat_of(est.r)), sl::quat_of(true_room_from_slam.r)));
    std::printf("session 2: relocalised at frame %zu: %.1f mm, %.2f deg\n", fixed->first, dt * 1000.0, dr);
    CHECK(fixed->first <= 3);
    CHECK(dt < 0.03 && dr < 1.0);
}

TEST(relocalize_single_frame_and_reject_wrong_room) {
    const Cams cams;
    const synth::Room room;
    const auto cal = calib();
    auto m = map::RoomMap::make(cal.uid, IDS);
    const orb::OrbParams o;
    for (const auto& [ps, q] : walk(16, 0.9, 0.0)) {
        const Pose rb = pose(ps, q);
        const auto ims = cams.render(room, rb);
        const auto fa = stereo::features(img(ims[0]), cams.rig.cams[0], o);
        const auto fb = stereo::features(img(ims[1]), cams.rig.cams[1], o);
        const auto pts = stereo::triangulate(fa, fb, cams.rig.cams[0], cams.rig.cams[1], stereo::StereoParams{});
        m.add_keyframe(0, rb, pts);
    }
    const auto [ps, q] = walk(16, 0.8, 0.2)[5];
    const Pose rb = pose(ps, q);
    auto ims = cams.render(room, rb);
    std::vector<stereo::Features> qf = {stereo::features(img(ims[0]), cams.rig.cams[0], o),
                                        stereo::features(img(ims[1]), cams.rig.cams[1], o)};
    const auto r = m.relocalize(qf, cams.rig, map::tilt_of(q), std::nullopt, map::RelocParams{});
    CHECK(r.has_value());
    if (r) {
        const double dt = g::norm(g::sub(r->room_from_body.t, ps));
        const double dr = sl::qangle_deg(sl::qmul(sl::qconj(sl::quat_of(r->room_from_body.r)), q));
        std::printf("single frame: %zu inliers / %zu corrs, %.1f mm, %.2f deg\n", r->inliers, r->corrs, dt * 1000.0, dr);
        CHECK(dt < 0.03 && dr < 1.0);
    }
    synth::Room other;
    other.seed = 999;
    ims = cams.render(other, rb);
    qf = {stereo::features(img(ims[0]), cams.rig.cams[0], o), stereo::features(img(ims[1]), cams.rig.cams[1], o)};
    CHECK(!m.relocalize(qf, cams.rig, map::tilt_of(q), std::nullopt, map::RelocParams{}));
}

TEST(map_file_roundtrip) {
    const std::size_t ids[2] = {0, 1};
    auto m = map::RoomMap::make("P511X0X000000000002", ids);
    stereo::StereoPoint sp;
    sp.p_body = {0.1, 0.2, -1.5};
    sp.desc.fill(7);
    sp.depth = 1.5;
    sp.ia = 0;
    const std::vector<stereo::StereoPoint> pts = {sp};
    m.add_keyframe(123, pose({1.0, 1.6, 2.0}, body_q(0.3, -0.1)), pts);
    m.set_anchor("desk", {0.5, 0.75, -0.4}, body_q(1.0, 0.0));
    auto back = map::RoomMap::decode(m.encode());
    CHECK(back && back->keyframes.size() == 1);
    CHECK(back && back->anchors == m.anchors);
    CHECK(back && g::norm(g::sub(back->keyframes[0].points[0].p, m.keyframes[0].points[0].p)) < 1e-6);
    auto b2 = *back;
    CHECK(b2.remove_anchor("desk") && b2.anchors.empty());
    CHECK(!map::RoomMap::decode("XXXX"));
    // the error texts
    CHECK(map::RoomMap::decode("XXXX").error() == "not a pimax room map (magic)");
    std::string v2 = m.encode();
    v2[4] = 2;
    CHECK(map::RoomMap::decode(v2).error() == "room map version 2, this build reads 1");
    CHECK(map::RoomMap::decode(m.encode().substr(0, 40)).error() == "truncated room map");
}

TEST(night_frames_keep_stereo_points_and_relocalise_on_a_daylight_map) {
    const Cams cams;
    const auto cal = calib();
    const synth::Room day;
    synth::Room night;
    night.gain = 0.12;
    const orb::OrbParams o;
    const stereo::StereoParams sp;
    std::size_t with = 0, without = 0;
    const auto w0 = walk(6, 0.9, 0.3);
    for (std::size_t k = 0; k < w0.size(); k++) {
        const Pose rb = pose(w0[k].first, w0[k].second);
        auto r = cams.render(night, rb);
        std::vector<std::vector<std::uint8_t>> ims;
        for (std::size_t i = 0; i < r.size(); i++) {
            ims.push_back(noisy(r[i], static_cast<std::uint64_t>(k * 2 + i) + 7));
        }
        auto n = [&](const std::optional<stereo::LowLight>& low) {
            const auto fa = stereo::features_with(img(ims[0]), cams.rig.cams[0], o, low);
            const auto fb = stereo::features_with(img(ims[1]), cams.rig.cams[1], o, low);
            return stereo::triangulate(fa, fb, cams.rig.cams[0], cams.rig.cams[1], sp).size();
        };
        with += n(stereo::LOW_LIGHT);
        without += n(std::nullopt);
    }
    std::printf("night stereo points over 6 views: %zu with the stretch, %zu without\n", with, without);
    CHECK(with > without + without / 4);
    CHECK(with / 6 >= 40);
    session::SessionParams p;
    p.kf_min_dist = 0.25;
    p.kf_min_angle_deg = 20.0;
    session::RoomSession s1(map::RoomMap::make(cal.uid, IDS), cams.rig, p);
    const session::Alignment ident{};
    const auto w1 = walk(24, 0.9, 0.0);
    for (std::size_t k = 0; k < w1.size(); k++) {
        const Pose rb = pose(w1[k].first, w1[k].second);
        const auto ims = cams.render(day, rb);
        const orb::Image views[2] = {img(ims[0]), img(ims[1])};
        s1.on_frame(static_cast<std::int64_t>(k) * 40000000, views, rb, ident, true);
    }
    auto m = std::move(s1).finish();
    const Pose slam_from_room{sl::mat_of(sl::qyaw(-0.7)), {0.5, 0.0, 1.0}};
    session::RoomSession s2(std::move(m), cams.rig, p);
    std::optional<std::pair<std::size_t, session::Alignment>> fixed;
    const auto w2 = walk(24, 0.8, 0.1);
    for (std::size_t k = 0; k < 12; k++) {
        const Pose rb = pose(w2[k].first, w2[k].second);
        auto r = cams.render(night, rb);
        std::vector<std::vector<std::uint8_t>> ims;
        for (std::size_t i = 0; i < r.size(); i++) {
            ims.push_back(noisy(r[i], static_cast<std::uint64_t>(k * 2 + i) + 99));
        }
        const orb::Image views[2] = {img(ims[0]), img(ims[1])};
        for (const auto& e : s2.on_frame(static_cast<std::int64_t>(k) * 250000000, views, slam_from_room.compose(rb), ident, true)) {
            if (e.kind == session::Event::Kind::Relocalized && !fixed) {
                fixed = std::pair{k, e.align};
            }
        }
    }
    CHECK(fixed.has_value());
    if (fixed) {
        const double dt = g::norm(g::sub(fixed->second.pose().t, slam_from_room.inverse().t));
        std::printf("night relocalised at frame %zu: %.1f mm\n", fixed->first, dt * 1000.0);
        CHECK(dt < 0.05);
    }
}

TEST(low_light_stretch_leaves_lit_frames_alone) {
    std::vector<std::uint8_t> lit(1000);
    for (std::size_t i = 0; i < lit.size(); i++) {
        lit[i] = static_cast<std::uint8_t>(i % 250);
    }
    const auto before = lit;
    stereo::LowLight plain = stereo::LOW_LIGHT;
    plain.denoise = false;
    CHECK(stereo::low_light_stretch(lit, 100, plain) == 1.0f);
    CHECK(lit == before);
    std::vector<std::uint8_t> dark(1000);
    for (std::size_t i = 0; i < dark.size(); i++) {
        dark[i] = i < 100 ? 0 : static_cast<std::uint8_t>(i % 21);
    }
    const float gn = stereo::low_light_stretch(dark, 100, plain);
    CHECK(std::fabs(gn - 6.0f) < 1e-6f);
    CHECK(dark[0] == 0);
    CHECK(dark[110] == 30);
    CHECK(std::all_of(dark.begin(), dark.end(), [](std::uint8_t v) { return v <= 120; }));
    std::vector<std::uint8_t> black(100, 0);
    CHECK(stereo::low_light_stretch(black, 10, stereo::LOW_LIGHT) == 1.0f);
    std::vector<std::uint8_t> flat(100, 10);
    CHECK(std::fabs(stereo::low_light_stretch(flat, 10, stereo::LOW_LIGHT) - 6.0f) < 1e-6f);
    CHECK(std::all_of(flat.begin(), flat.end(), [](std::uint8_t v) { return v == 60; }));
}

TEST(per_session_keyframe_spacing) {
    const auto cal = calib();
    auto m = map::RoomMap::make(cal.uid, IDS);
    const Pose at = pose({0.0, 1.5, 0.0}, body_q(0.0, 0.0));
    m.add_keyframe(0, at, {});
    CHECK(!m.wants_keyframe(at, 0.2, 15.0));
    CHECK(!m.wants_keyframe_in(at, 0.2, 15.0, 0u));
    m.sessions = 1;
    CHECK(m.wants_keyframe_in(at, 0.2, 15.0, 1u));
}

// pngread.rs: a round trip through a PNG written here (the Rust test uses pimax_cam::png::encode; this one writes
// the same kind of file: one IHDR, zlib IDAT with filter 0 rows, IEND), plus the colour conversion and errors.
namespace {

void be32(std::vector<std::uint8_t>& o, std::uint32_t v) {
    o.push_back(static_cast<std::uint8_t>(v >> 24));
    o.push_back(static_cast<std::uint8_t>(v >> 16));
    o.push_back(static_cast<std::uint8_t>(v >> 8));
    o.push_back(static_cast<std::uint8_t>(v));
}

void chunk(std::vector<std::uint8_t>& o, const char* kind, const std::vector<std::uint8_t>& body) {
    be32(o, static_cast<std::uint32_t>(body.size()));
    const std::size_t at = o.size();
    o.insert(o.end(), kind, kind + 4);
    o.insert(o.end(), body.begin(), body.end());
    be32(o, static_cast<std::uint32_t>(crc32(0, o.data() + at, static_cast<uInt>(body.size() + 4))));
}

std::vector<std::uint8_t> encode(std::size_t w, std::size_t h, std::uint8_t ctype, const std::vector<std::uint8_t>& px,
                                 std::uint8_t filter = 0) {
    const std::size_t bpp = ctype == 0 ? 1 : (ctype == 2 ? 3 : 4);
    std::vector<std::uint8_t> raw;
    std::vector<std::uint8_t> prev(w * bpp, 0);
    for (std::size_t y = 0; y < h; y++) {
        raw.push_back(filter);
        for (std::size_t x = 0; x < w * bpp; x++) {
            const std::uint8_t v = px[y * w * bpp + x];
            raw.push_back(filter == 2 ? static_cast<std::uint8_t>(v - prev[x]) : v);
            prev[x] = v;
        }
    }
    uLongf n = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> z(n);
    compress2(z.data(), &n, raw.data(), static_cast<uLong>(raw.size()), 6);
    z.resize(n);
    std::vector<std::uint8_t> o = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    std::vector<std::uint8_t> ihdr;
    be32(ihdr, static_cast<std::uint32_t>(w));
    be32(ihdr, static_cast<std::uint32_t>(h));
    ihdr.insert(ihdr.end(), {8, ctype, 0, 0, 0});
    chunk(o, "IHDR", ihdr);
    chunk(o, "IDAT", z);
    chunk(o, "IEND", {});
    return o;
}

} // namespace

TEST(png_roundtrip_with_an_encoder) {
    const std::size_t w = 37, h = 11;
    std::vector<std::uint8_t> px(w * h);
    for (std::size_t i = 0; i < px.size(); i++) {
        px[i] = static_cast<std::uint8_t>((i * 7) % 251);
    }
    for (std::uint8_t f : {std::uint8_t{0}, std::uint8_t{2}}) {
        const auto g = pngread::decode(encode(w, h, 0, px, f));
        CHECK(g && g->w == w && g->h == h && g->data == px);
    }
    std::vector<std::uint8_t> rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255, 10, 20, 30};
    const auto c = pngread::decode(encode(4, 1, 2, rgb));
    CHECK(c && (c->data == std::vector<std::uint8_t>{76, 149, 28, 18}));
    CHECK(pngread::decode(std::vector<std::uint8_t>{1, 2, 3}).error() == "not a PNG");
    auto bad = encode(w, h, 0, px);
    bad[8 + 4 + 4 + 8] = 16; // bit depth 16
    CHECK(pngread::decode(bad).error() == "unsupported PNG (depth 16, colour type 0, interlace 0)");
}

PIBRIDGE_TEST_MAIN()
