// SPDX-License-Identifier: MIT
#include "selftest.h"

#include "pibridge/room/chap.h"
#include "pibridge/room/editor.h"
#include "pibridge/room/files.h"
#include "pibridge/room/pose.h"
#include "pibridge/room/raster.h"
#include "pibridge/room/scene.h"
#include "mapflow.h"
#include "ptsynth.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <memory>
#include <numbers>

namespace fs = std::filesystem;
using namespace pibridge::room;
namespace pt = pibridge::xrkit::pt;

namespace pibridge::roomsetup {

std::string selfTestVrchap() {
    const auto quads = [](double w, double d) {
        const Json b = makeBounds(rectangle(w, d), kDefaultHeight);
        return b.dump();
    };
    return std::format(R"({{
   "jsonid" : "chaperone_info",
   "universes" : [
      {{
         "collision_bounds" : {},
         "play_area" : [ 3, 2 ],
         "seated" : {{ "translation" : [ 0, 1.2, 0 ], "yaw" : 0 }},
         "standing" : {{ "translation" : [ 0.25, 0, -0.5 ], "yaw" : 0.3 }},
         "time" : "Mon Oct 19 10:00:00 2026",
         "universeID" : "1111111111"
      }},
      {{
         "collision_bounds" : {},
         "play_area" : [ 2, 1.5 ],
         "seated" : {{ "translation" : [ 0, 1.2, 0 ], "yaw" : 0 }},
         "standing" : {{ "translation" : [ 0, 0, 0 ], "yaw" : 0 }},
         "time" : "Sun Oct 18 09:00:00 2026",
         "universeID" : "2"
      }}
   ],
   "version" : 5
}}
)",
                       quads(4, 3), quads(3, 2));
}

namespace {

Mat34 at(Vec3 p) {
    Mat34 m;
    m.m[0][3] = p.x;
    m.m[1][3] = p.y;
    m.m[2][3] = p.z;
    return m;
}

struct Script {
    Editor& ed;
    double& now;
    Vec3 head{0, 1.7, 0.5};
    void frame(Vec3 p, std::uint64_t b = 0) {
        FrameInput in;
        in.headValid = true;
        in.head = at(head);
        in.hands[0] = {true, at({-0.4, 1.0, 0.4}), true, at({-0.4, 1.0, 0.4}), 0};
        in.hands[1] = {true, at(p), true, at(p), b};
        ed.frame(in);
        now += 1.0 / 90;
    }
    void click(Vec3 p, unsigned bitNo) {
        frame(p);
        frame(p, 1ULL << bitNo);
        frame(p);
    }
    bool press(const std::string& label) {
        if (!ed.menuOpen) {
            click({0.3, 1.0, 0.3}, bit::Menu);
        }
        const auto [slots, h] = ed.layout();
        for (std::size_t i = 0; i < ed.cur.size() && i < slots.size(); i++) {
            if (ed.cur[i].label && ed.cur[i].label().starts_with(label)) {
                const Vec3 tip = ed.menuPos + ed.menuR * slots[i].cx + ed.menuU * slots[i].cy;
                click(tip + Vec3{0, 0, 0.09}, bit::Trigger);
                return true;
            }
        }
        return false;
    }
};

int failures = 0;

void step(const char* name, bool ok, const std::string& detail = {}) {
    std::printf("%s %s%s%s\n", ok ? "ok  " : "FAIL", name, detail.empty() ? "" : ": ", detail.c_str());
    if (!ok) {
        failures++;
    }
}

void snapshot(Editor& ed, double t, const std::string& dir, const std::string& name, Vec3 eye, Vec3 look) {
    if (dir.empty()) {
        return;
    }
    Batch b;
    buildScene(ed, b, t);
    const Mat4 vp = mul4(projectionVulkan(-0.85, 0.85, 0.6, -0.6, 0.05, 60), lookAt(eye, look));
    const Image img = rasterize(b, vp, 1280, 900, {0.02F, 0.03F, 0.03F, 1});
    const std::string path = dir + "/" + name + ".ppm";
    step(("snapshot " + name).c_str(), writePpm(img, path), path);
}

// --map: the headset side of the room mapping on the synthetic room, its passthrough drawn with the shader's
// arithmetic on the CPU and the scene over it (one eye)
void mapSnapshot(const MapFlow& flow, const pt::Frame& frame, const pt::Rig& cams, const Mat34& capture,
                 const Mat34& head, double t, const std::string& dir, const std::string& name) {
    if (dir.empty()) {
        return;
    }
    Mat34 eyeOff;
    eyeOff.m[0][3] = -0.032;
    const Mat34 eye = mul34(head, eyeOff);
    const double l = -0.9, r = 0.85, u = 0.75, d = -0.85;
    const pt::EyeView ev = pt::eyeView(eye, l, r, u, d);
    pt::Look look;
    look.floorY = flow.floorStage();
    look.gain = pt::autoGain(frame.mean);
    Image img = pt::render(pt::makeUniform(0, ev, cams, capture, look), frame, 1100, 950);
    Batch b;
    flow.build(b, t);
    rasterizeOnto(img, b, mul4(projectionVulkan(l, r, u, d, 0.05, 60), mat4From34(inv34(eye))));
    const std::string path = dir + "/" + name + ".ppm";
    step(("snapshot " + name).c_str(), writePpm(img, path), path);
}

void mapFlowSelfTest(const std::string& snapshotDir) {
    const auto cams = referenceRig();
    step("map: the reference cameras", cams.has_value(), cams ? "" : cams.error());
    if (!cams) {
        return;
    }
    const roommap::synth::Room rm;
    StageLink link; // the synthetic stage is Monado's raw frame here: the link is the identity
    for (int i = 0; i < 12; i++) {
        const Mat34 h = headPose({0.1 * i, 1.6, 0}, 0.4 * i, 0);
        link.add(h, h);
    }
    guide::State st;
    st.pid = 1;
    st.step = "map";
    st.title = "Room setup";
    st.text = "Stand in the middle of your play space. Look around slowly: left, right, behind you, and at the floor.";
    st.conv = guide::Conv{};
    st.keyframes = 9;
    st.points = 412;
    MapFlow flow;
    double now = 100;
    std::array<Mat34, 2> hands{headPose({-0.25, 1.05, 0.35}, 0.3, -55 * std::numbers::pi / 180),
                               headPose({0.3, 1.05, 0.35}, 0.3, -55 * std::numbers::pi / 180)};
    Mat34 head = headPose({0.3, 1.65, 0.6}, 0.0, -0.2);
    const auto frame = [&](std::uint64_t right = 0) {
        FrameInput in;
        in.headValid = true;
        in.head = head;
        for (std::size_t i = 0; i < 2; i++) {
            in.hands[i] = {true, hands[i], true, hands[i], 0};
        }
        in.hands[1].buttons = right;
        flow.frame(in, now, st, link);
        now += 1.0 / 90;
    };
    const std::uint64_t trig = 1ULL << bit::Trigger;
    st.seq = 1;
    for (int i = 0; i < 7; i++) {
        head = headPose({0.3, 1.65, 0.6}, Coverage::sectorHeading(i), -0.2);
        frame();
    }
    head = headPose({0.3, 1.65, 0.6}, 0.35, -0.25);
    frame();
    const Mat34 cap0 = headPose({0.31, 1.65, 0.6}, 0.31, -0.24); // captured a little earlier: the time warp
    const pt::Frame f0 = synthFrame(rm, *cams, cap0, 1);
    const auto lines = flow.lines();
    step("map: coverage and the mapping numbers",
         flow.coverage().fraction() > 0.3 && std::ranges::find(lines, std::string("VIEWS MAPPED 9   POINTS IN VIEW 412")) != lines.end());
    mapSnapshot(flow, f0, *cams, cap0, head, now, snapshotDir, "roommap-1-look-around");

    st.seq = 2;
    st.step = "floor-confirm";
    st.title = "Floor";
    st.text = "The grid shows the floor below your feet. Nudge it up / down with the thumbstick, then press NEXT.";
    st.floor = 0.0;
    head = headPose({0.3, 1.65, 0.6}, 0.35, -0.6);
    frame();
    frame();
    const pt::Frame f1 = synthFrame(rm, *cams, head, 2);
    mapSnapshot(flow, f1, *cams, head, head, now, snapshotDir, "roommap-2-floor");
    // the floor is a head-relative drop the user tunes with the stick, then confirms with NEXT (answer "height").
    // Here the grid sits on the published floor (room y 0): floorStage() ~ 0 and the floor is known.
    step("map: the floor grid on the published floor", flow.floorKnown() && std::fabs(flow.floorStage()) < 0.05);

    st.seq = 3;
    st.step = "boundary";
    st.title = "Play area";
    st.text = "The green line is the free floor around you. Pull the trigger to keep it, or hold the trigger and draw the "
              "edge on the floor with the controller.";
    st.boundary = {{-1.6, -2.3}, {1.0, -2.3}, {1.0, 1.8}, {-1.6, 1.8}};
    head = headPose({0.3, 1.65, 0.6}, 0.35, -0.45);
    frame();
    for (int i = 0; i < 60; i++) { // a stroke under way
        hands[1] = headPose({0.3, 1.05, 0.35}, 0.9 - i * 0.012, -40 * std::numbers::pi / 180);
        frame(trig);
    }
    const pt::Frame f2 = synthFrame(rm, *cams, head, 3);
    mapSnapshot(flow, f2, *cams, head, head, now, snapshotDir, "roommap-3-boundary");
    step("map: drawing the boundary with the ray", flow.drawn().size() > 10, std::format("{} points", flow.drawn().size()));
    frame();
    auto a = flow.takeAnswers();
    step("map: a short stroke is not a boundary", a.empty());
    // No separate "forward" step any more: in the custom flow the centre and forward are taken from the live head at
    // SAVE (mapflow.cpp), so there is nothing to confirm with the trigger here.
}

} // namespace

int runSelfTest(const std::string& snapshotDir) {
    failures = 0;
    std::string tmpl = (fs::temp_directory_path() / "pibridge-roomsetup-selftest-XXXXXX").string();
    if (::mkdtemp(tmpl.data()) == nullptr) {
        std::printf("FAIL no temporary folder\n");
        return 1;
    }
    const std::string dir = tmpl;
    const std::string cfg = dir + "/config";
    fs::create_directories(cfg);
    if (!snapshotDir.empty()) {
        std::error_code ec;
        fs::create_directories(snapshotDir, ec);
    }
    (void)writeFileAtomic(cfg + "/chaperone_info.vrchap", selfTestVrchap());
    auto store = Store::load({cfg}, "1111111111");
    step("load the chaperone file", store.has_value(), store ? std::format("{} universes", store->ids().size()) : store.error());
    if (!store) {
        fs::remove_all(dir);
        return 1;
    }
    double now = 10;
    FileZoneControl zone(cfg + "/steamvr.vrsettings", dir + "/rooms", [] { return std::time_t{1760000000}; });
    EditorHost host;
    host.now = [&now] { return now; };
    host.wallClock = [] { return std::time_t{1760000000}; };
    host.afterSave = [](const SaveResult&) { return std::string("self test: no runtime to reload"); };
    host.zone = &zone;
    Editor ed(*store, RoomLib(dir + "/rooms"), host);
    Script s{ed, now};

    // move a corner by 30 cm
    const Pt2 c = ed.pts[1];
    s.frame({c.x, 1.0, c.z});
    s.frame({c.x, 1.0, c.z}, 1ULL << bit::Trigger);
    s.frame({c.x + 0.3, 1.0, c.z}, 1ULL << bit::Trigger);
    s.frame({c.x + 0.3, 1.0, c.z});
    step("move a corner", std::fabs(ed.pts[1].x - (c.x + 0.3)) < 1e-9 && ed.dirty);
    snapshot(ed, now, snapshotDir, "roomsetup-1-room", {0.5, 2.6, 3.6}, {0, 0.4, -0.4});

    // the menu
    s.click({0.3, 1.0, 0.3}, bit::Menu);
    step("open the menu", ed.menuOpen && ed.cur.size() >= 20, std::format("{} buttons", ed.cur.size()));
    s.frame(ed.menuPos + ed.menuR * ed.layout().first[0].cx + ed.menuU * ed.layout().first[0].cy + Vec3{0, 0, 0.09});
    snapshot(ed, now, snapshotDir, "roomsetup-2-menu", ed.hmd.pos() + Vec3{0.25, 0.05, 0.45}, ed.menuPos + Vec3{0.12, 0, 0});
    s.click({0.3, 1.0, 0.3}, bit::Menu);

    // play area: unlock, fit, move the centre
    const Pt2 lock = ed.lockPos();
    s.click({lock.x, 1.0, lock.z}, bit::Trigger);
    step("unlock the play area", ed.areaMode);
    const bool fitted = s.press("FIT AREA") && ed.areaDirty;
    step("fit the play area", fitted, std::format("{:.2f} x {:.2f} m", ed.area.w, ed.area.h));
    s.click({0.3, 1.0, 0.3}, bit::Menu);
    snapshot(ed, now, snapshotDir, "roomsetup-3-play-area", {0.2, 3.2, 2.8}, {0, 0, -0.3});
    s.click({lock.x, 1.0, lock.z}, bit::Trigger);
    step("lock the play area", !ed.areaMode);

    // trace a new room by walking round it with the trigger held
    const bool started = s.press("TRACE OUTLINE") && ed.tracing;
    step("start a trace", started);
    FakePoseSource walk(rigidFromYaw(0.2, 0.2, -0.1).all(rectangle(3.6, 2.8)), 1.0, 0.2);
    for (double t = 0; t < walk.lapSeconds() + 1; t += 1.0 / 90) {
        const auto p = walk.sample(t);
        if (t > walk.lapSeconds() * 0.6 && t < walk.lapSeconds() * 0.6 + 1.0 / 90) {
            snapshot(ed, now, snapshotDir, "roomsetup-4-trace", {0.4, 3.4, 3.2}, {0, 0, -0.3});
        }
        s.frame(p->hands[1].pos, p->trigger[1] ? 1ULL << bit::Trigger : 0);
    }
    step("trace the room", !ed.tracing && ed.pts.size() == 4,
         std::format("{} corners, {:.1f} m2", ed.pts.size(), std::fabs(polyArea(ed.pts))));

    // save, name it, reload
    const bool saved = s.press("SAVE") && !ed.dirty;
    step("save", saved, ed.status());
    s.click({0.3, 1.0, 0.3}, bit::Menu);
    auto named = ed.saveAsNew("Selftest room");
    step("save as a named room", named.has_value() && ed.lib.find("selftest room").has_value());
    auto again = Store::load({cfg}, "1111111111");
    step("reload sees the traced room", again.has_value() && again->stagePts().size() == 4 &&
                                            std::fabs(std::fabs(polyArea(again->stagePts())) - 3.6 * 2.8) < 0.5);
    step("one backup of the original", backupsOf(cfg + "/chaperone_info.vrchap").size() == 1);
    ed.restoreOriginal();
    step("restore original", std::fabs(std::fabs(polyArea(ed.pts)) - 12) < 1e-6, ed.status());

    mapFlowSelfTest(snapshotDir);

    std::error_code ec;
    fs::remove_all(dir, ec);
    std::printf("%s (%d failed)\n", failures == 0 ? "self test passed" : "self test FAILED", failures);
    return failures == 0 ? 0 : 1;
}

} // namespace pibridge::roomsetup
