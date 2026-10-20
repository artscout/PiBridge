// SPDX-License-Identifier: MIT
// The passthrough (tools/xrkit passthrough.h) without a headset: the lower pair of the reference calibration, the
// synthetic room rendered through it as the camera would see it, and the eye image the shader's arithmetic makes
// of it compared with the room traced straight from the eye (floor pixels: the floor plane is exact there). The
// time warp: the head moved between the capture and the display, and only the capture-time pose puts the floor back.
// The camera feed thread on a .pxcam of synthetic frames; a lab recording when there is one (skipped otherwise).
#include "pibridge_test.h"
#include "ptsynth.h"

#include "pibridge/cam/frame.h"
#include "pibridge/cam/record.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <numbers>
#include <thread>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::roomsetup;
namespace pt = pibridge::xrkit::pt;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

room::Mat34 eyeOf(const room::Mat34& head, int eye) {
    room::Mat34 o;
    o.m[0][3] = eye == 0 ? -0.032 : 0.032;
    return room::mul34(head, o);
}

struct Err {
    double mean = 0;
    std::size_t n = 0;
};

// mean |passthrough - truth| over floor pixels seen by a camera (grey levels)
Err floorError(const roommap::synth::Room& rm, const pt::Uniform& u, const pt::Frame& f, const room::Mat34& eye,
               const pt::EyeView& ev) {
    Err e;
    double sum = 0;
    for (int y = 0; y < 60; y++) {
        for (int x = 0; x < 80; x++) {
            const double uu = (x + 0.5) / 80, vv = (y + 0.5) / 60;
            const double tx = ev.tanL + (ev.tanR - ev.tanL) * uu, ty = ev.tanU + (ev.tanD - ev.tanU) * vv;
            const room::Vec3 d = (eye.axis(0) * tx + eye.axis(1) * ty + eye.axis(2) * -1.0).norm();
            const room::Vec3 o = eye.pos();
            const auto hit = rm.trace({o.x, o.y, o.z}, {d.x, d.y, d.z});
            if (!hit || std::fabs(o.y + d.y * hit->first) > 0.01) {
                continue; // not the floor
            }
            const auto g = pt::sample(u, f, uu, vv);
            if (!g) {
                continue;
            }
            sum += std::fabs(*g * 255.0 - hit->second);
            e.n++;
        }
    }
    e.mean = e.n ? sum / static_cast<double>(e.n) : 1e9;
    return e;
}

} // namespace

TEST(lower_pair_of_the_reference) {
    const auto cams = referenceRig(false);
    CHECK(cams.has_value());
    if (!cams) {
        return;
    }
    CHECK(cams->n == 2);
    // both look forward (-Z of the view) and down
    for (std::size_t s = 0; s < cams->n; s++) {
        const auto& c = cams->cam[s];
        const room::Vec3 z = c.viewFromCam.axis(2); // the optical axis in the view
        CHECK(!c.upper);
        CHECK(z.z < -0.5);
        CHECK(z.y < -0.2);
        CHECK(c.viewFromCam.pos().length() < 0.15);
    }
    // each eye tries the camera on its own side first
    CHECK(cams->cam[cams->order[0][0]].viewFromCam.pos().x < 0);
    CHECK(cams->cam[cams->order[1][0]].viewFromCam.pos().x > 0);
}

TEST(all_four_cameras_of_the_reference) {
    const auto rig = referenceRig(true);
    CHECK(rig.has_value());
    if (!rig) {
        return;
    }
    CHECK(rig->n == 4);
    int upper = 0;
    for (std::size_t s = 0; s < rig->n; s++) {
        upper += rig->cam[s].upper ? 1 : 0;
        std::printf("slot %zu: tile %zu %s, axis y %.2f, x %.3f\n", s, rig->cam[s].tile, rig->cam[s].name.c_str(),
                    rig->cam[s].viewFromCam.axis(2).y, rig->cam[s].viewFromCam.pos().x);
    }
    CHECK(upper == 2);
    for (int eye = 0; eye < 2; eye++) {
        const auto& o = rig->order[static_cast<std::size_t>(eye)];
        // lower on the eye's side, the other lower, the upper on the eye's side, the other upper
        CHECK(!rig->cam[o[0]].upper && !rig->cam[o[1]].upper && rig->cam[o[2]].upper && rig->cam[o[3]].upper);
        CHECK((rig->cam[o[0]].viewFromCam.pos().x < 0) == (eye == 0));
        CHECK((rig->cam[o[2]].viewFromCam.pos().x < 0) == (eye == 0));
    }
}

TEST(upper_cameras_fill_the_ceiling_and_seams_are_blended) {
    const auto lower = referenceRig(false);
    const auto all = referenceRig(true);
    if (!lower || !all) {
        CHECK(false);
        return;
    }
    const roommap::synth::Room rm;
    const room::Mat34 head = headPose({0.1, 1.6, 0.2}, 0, 0); // looking straight ahead
    const room::Mat34 e = eyeOf(head, 0);
    const pt::EyeView ev = pt::eyeView(e, -50 * kDeg, 45 * kDeg, 48 * kDeg, -52 * kDeg);
    const pt::Frame f2 = synthFrame(rm, *lower, head, 1);
    const pt::Frame f4 = synthFrame(rm, *all, head, 1);
    pt::Look look{};
    look.room = roomShape(rm);
    const pt::Uniform u2 = pt::makeUniform(0, ev, *lower, head, look);
    const pt::Uniform u4 = pt::makeUniform(0, ev, *all, head, look);
    int seen2 = 0, seen4 = 0, topSeen2 = 0, topSeen4 = 0, blended = 0, n = 0, top = 0;
    for (int y = 0; y < 60; y++) {
        for (int x = 0; x < 80; x++) {
            const double uu = (x + 0.5) / 80, vv = (y + 0.5) / 60;
            const auto a = pt::sampleFull(u2, f2, uu, vv);
            const auto b = pt::sampleFull(u4, f4, uu, vv);
            n++;
            seen2 += a ? 1 : 0;
            seen4 += b ? 1 : 0;
            blended += b && b->b >= 0 ? 1 : 0;
            if (y < 15) {
                top++;
                topSeen2 += a ? 1 : 0;
                topSeen4 += b ? 1 : 0;
            }
        }
    }
    std::printf("seen: 2 cameras %.1f %% (top quarter %.1f %%), 4 cameras %.1f %% (top quarter %.1f %%), blended %.1f %%\n",
                100.0 * seen2 / n, 100.0 * topSeen2 / top, 100.0 * seen4 / n, 100.0 * topSeen4 / top, 100.0 * blended / n);
    CHECK(seen4 > seen2);
    CHECK(topSeen4 > topSeen2 + top / 4);
    CHECK(seen4 > n * 95 / 100);
    CHECK(blended > n / 50);
    // continuity: along a row the grey never jumps at a camera change by more than the picture's own steps
    int jumps = 0;
    for (int x = 1; x < 400; x++) {
        const auto p = pt::sampleFull(u4, f4, (x - 0.5) / 400, 0.2);
        const auto q = pt::sampleFull(u4, f4, (x + 0.5) / 400, 0.2);
        if (p && q && p->a != q->a && std::fabs(p->grey - q->grey) > 0.25F) {
            jumps++;
        }
    }
    CHECK(jumps == 0);
}

TEST(the_room_prism_beats_the_sphere_on_walls) {
    const auto rig = referenceRig(true);
    if (!rig) {
        CHECK(false);
        return;
    }
    const roommap::synth::Room rm;
    const room::Mat34 head = headPose({0.3, 1.6, 0.4}, 30 * kDeg, 5 * kDeg);
    const pt::Frame f = synthFrame(rm, *rig, head, 1);
    const room::Mat34 e = eyeOf(head, 0);
    const pt::EyeView ev = pt::eyeView(e, -50 * kDeg, 45 * kDeg, 48 * kDeg, -52 * kDeg);
    pt::Look sphere{};
    pt::Look prism = sphere;
    prism.room = roomShape(rm);
    // every pixel against the room traced straight from the eye
    const auto err = [&](const pt::Look& look) {
        const pt::Uniform u = pt::makeUniform(0, ev, *rig, head, look);
        double sum = 0;
        int n = 0;
        for (int y = 0; y < 60; y++) {
            for (int x = 0; x < 80; x++) {
                const double uu = (x + 0.5) / 80, vv = (y + 0.5) / 60;
                const double tx = ev.tanL + (ev.tanR - ev.tanL) * uu, ty = ev.tanU + (ev.tanD - ev.tanU) * vv;
                const room::Vec3 d = (e.axis(0) * tx + e.axis(1) * ty + e.axis(2) * -1.0).norm();
                const room::Vec3 o = e.pos();
                const auto hit = rm.trace({o.x, o.y, o.z}, {d.x, d.y, d.z});
                const auto g = pt::sample(u, f, uu, vv);
                if (!hit || !g) {
                    continue;
                }
                sum += std::fabs(*g * 255.0 - hit->second);
                n++;
            }
        }
        return n ? sum / n : 1e9;
    };
    const double es = err(sphere), ep = err(prism);
    std::printf("all pixels: sphere %.1f, room prism %.1f grey levels\n", es, ep);
    CHECK(ep < es);
}

TEST(uniform_layout_and_sample_agree_with_the_room) {
    const auto cams = referenceRig(false);
    CHECK(cams.has_value());
    if (!cams) {
        return;
    }
    const roommap::synth::Room rm;
    const room::Mat34 head = headPose({0.2, 1.6, 0.3}, 20 * kDeg, -30 * kDeg);
    const pt::Frame f = synthFrame(rm, *cams, head, 1);
    CHECK(f.mean > 10);
    for (int eye = 0; eye < 2; eye++) {
        const room::Mat34 e = eyeOf(head, eye);
        const pt::EyeView ev = pt::eyeView(e, -50 * kDeg, 45 * kDeg, 48 * kDeg, -52 * kDeg);
        const pt::Uniform u = pt::makeUniform(eye, ev, *cams, head, pt::Look{});
        CHECK(sizeof u == 848 && u.info[0] == 2.0F);
        const Err err = floorError(rm, u, f, e, ev);
        std::printf("eye %d: %zu floor samples, mean error %.1f grey levels\n", eye, err.n, err.mean);
        CHECK(err.n > 400);
        CHECK(err.mean < 14.0);
    }
}

TEST(time_warp_uses_the_capture_pose) {
    const auto cams = referenceRig(false);
    if (!cams) {
        CHECK(false);
        return;
    }
    const roommap::synth::Room rm;
    // captured 60 ms ago; the head has turned 6 deg and moved 4 cm since
    const room::Mat34 atCapture = headPose({0.0, 1.62, 0.0}, 10 * kDeg, -35 * kDeg);
    const room::Mat34 atDisplay = headPose({0.04, 1.62, -0.01}, 16 * kDeg, -33 * kDeg);
    const pt::Frame f = synthFrame(rm, *cams, atCapture, 1);
    const room::Mat34 e = eyeOf(atDisplay, 0);
    const pt::EyeView ev = pt::eyeView(e, -50 * kDeg, 45 * kDeg, 48 * kDeg, -52 * kDeg);
    const pt::Look look{};
    const Err warped = floorError(rm, pt::makeUniform(0, ev, *cams, atCapture, look), f, e, ev);
    const Err stale = floorError(rm, pt::makeUniform(0, ev, *cams, atDisplay, look), f, e, ev);
    std::printf("with the capture pose %.1f, with the display pose %.1f grey levels\n", warped.mean, stale.mean);
    CHECK(warped.mean < 14.0);
    CHECK(stale.mean > warped.mean * 1.5);
}

TEST(nothing_seen_is_dark_and_gain_lifts_dark_rooms) {
    const auto cams = referenceRig(false);
    if (!cams) {
        CHECK(false);
        return;
    }
    roommap::synth::Room rm;
    const room::Mat34 head = headPose({0, 1.6, 0}, 0, 0);
    const pt::Frame f = synthFrame(rm, *cams, head, 1);
    const pt::EyeView ev = pt::eyeView(eyeOf(head, 0), -50 * kDeg, 45 * kDeg, 48 * kDeg, -52 * kDeg);
    const pt::Uniform u = pt::makeUniform(0, ev, *cams, head, pt::Look{});
    CHECK(!pt::sample(u, f, 0.5, 0.0)); // straight up: the lower cameras do not see it
    CHECK(pt::sample(u, f, 0.5, 0.9).has_value());
    CHECK(pt::autoGain(25) == 4.0 && pt::autoGain(200) == 1.0 && pt::autoGain(50) == 2.0);
    const room::Image img = pt::render(u, f, 64, 48);
    CHECK(img.w == 64 && img.rgb.size() == 64 * 48 * 3);
}

TEST(feed_reads_a_recording_and_keeps_the_newest_frame) {
    // three synthetic tracking frames (and LED frames between them) in a .pxcam
    const char* tmp = std::getenv("TMPDIR");
    const std::string path = std::string(tmp && *tmp ? tmp : "/tmp") + "/pibridge-pt-feed-" + std::to_string(::getpid()) + ".pxcam";
    {
        auto rec = cam::Recorder::create(path, cam::FRAME_W, std::nullopt);
        CHECK(rec.has_value());
        if (!rec) {
            return;
        }
        for (std::uint32_t i = 0; i < 6; i++) {
            std::vector<std::uint8_t> raw(cam::FRAME_BYTES, 0);
            std::uint8_t meta[cam::META_LEN] = {};
            meta[32] = i % 2 == 0 ? 0x86 : 0x06;
            const std::uint64_t ts = 1000000 + i * 20000;
            for (int k = 0; k < 8; k++) {
                meta[33 + k] = static_cast<std::uint8_t>(ts >> (8 * k));
            }
            for (int k = 0; k < 4; k++) {
                meta[41 + k] = static_cast<std::uint8_t>(i >> (8 * k));
                meta[61 + 2 * k] = 10; // y_offset 10
                meta[92 + k] = cam::META_MAGIC[static_cast<std::size_t>(k)];
            }
            for (std::size_t r = 0; r < cam::META_ROWS; r++) {
                std::copy(meta, meta + cam::META_LEN, raw.begin() + static_cast<std::ptrdiff_t>(r * cam::FRAME_W));
            }
            for (std::size_t y = 0; y < cam::TILE_H; y++) {
                for (std::size_t x = 0; x < cam::FRAME_W; x++) {
                    raw[(10 + y) * cam::FRAME_W + x] = static_cast<std::uint8_t>(20 + (x / cam::TILE_W) * 40 + i);
                }
            }
            cam::RawFrame rf{raw, cam::FRAME_W, 5'000'000'000ULL + i * 20'000'000ULL, i};
            CHECK((*rec)->write(rf).has_value());
        }
        CHECK((*rec)->finish().has_value());
    }
    pt::Feed feed(path, {0, 1, 2, 3}, false);
    feed.start();
    pt::Frame got;
    bool have = false;
    for (int i = 0; i < 100 && !have; i++) {
        have = feed.take(got, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    CHECK(have);
    CHECK(got.img[0].size() == 640u * 480u && got.img[1].size() == 640u * 480u);
    CHECK(got.n == 4 && got.img[3].size() == 640u * 480u);
    CHECK(got.img[1][1000] - got.img[0][1000] == 40); // tile 1 is tile 0 + 40
    CHECK(got.img[3][1000] - got.img[0][1000] == 120); // the upper pair too
    CHECK(got.monoNs != 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    feed.stop();
    CHECK(!feed.take(got, feed.seq())); // nothing newer than the newest
    std::filesystem::remove(path);
}

TEST(feed_says_why_without_a_camera) {
    pt::Feed feed("/nonexistent/pibridge-no-such.pxcam", {0, 1}, false);
    feed.start();
    std::string why;
    for (int i = 0; i < 50 && why.empty(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        why = feed.error();
    }
    feed.stop();
    CHECK(why.find("no camera picture") != std::string::npos);
}

TEST(lab_recording_passthrough) {
    // a real recording when the lab data is there (PIBRIDGE_LAB_CAM_DIR, default ../ext/cam): skipped otherwise
    const char* d = std::getenv("PIBRIDGE_LAB_CAM_DIR");
    const std::string dir = d && *d ? d : "../ext/cam";
    const std::string file = dir + "/ctrl1.pxcam";
    if (!std::filesystem::exists(file)) {
        std::printf("skipped: no lab recording %s\n", file.c_str());
        return;
    }
    pt::Feed feed(file, {0, 1, 2, 3}, false);
    feed.start();
    pt::Frame got;
    bool have = false;
    for (int i = 0; i < 200 && !have; i++) {
        have = feed.take(got, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    feed.stop();
    CHECK(have && got.mean > 1);
}

PIBRIDGE_TEST_MAIN()
