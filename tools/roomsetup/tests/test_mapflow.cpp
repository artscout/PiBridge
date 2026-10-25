// SPDX-License-Identifier: MIT
// pibridge-roomsetup --map's headset side (mapflow.h) with scripted frames and made-up pibridge-roommap states: the stage
// link, the coverage hints, the floor-drop shown and nudged with the stick, the boundary drawn with the ray, the stop,
// the end, and that answers need the link. No runtime, no files. (The floor "yes/touch", the trigger-kept boundary and
// the forward step are gone -- the floor is the stick + the NEXT chip, the boundary the SAVE chip, forward taken at SAVE.)
#include "pibridge_test.h"
#include "mapflow.h"
#include "ptsynth.h"

#include <cmath>
#include <numbers>

using namespace pibridge;
using namespace pibridge::roomsetup;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr std::uint64_t kTrig = 1ULL << room::bit::Trigger;
constexpr std::uint64_t kMenu = 1ULL << room::bit::Menu;

bool near(double a, double b, double e = 1e-3) { return std::fabs(a - b) <= e; }

// the panel's words in one line (long lines are wrapped)
bool says(const MapFlow& f, const std::string& what) {
    std::string all;
    for (const auto& l : f.lines()) {
        all += l + " ";
    }
    return all.find(what) != std::string::npos;
}

// stage = Ry(0.4) raw + (0.3, 1.5, -0.2): Monado's offset and SteamVR's standing frame in one
const double kLinkYaw = 0.4;
const room::Vec3 kLinkT{0.3, 1.5, -0.2};

room::Mat34 rawToStage(const room::Mat34& raw) {
    const room::Mat34 t = headPose(kLinkT, kLinkYaw, 0);
    return room::mul34(t, raw);
}

StageLink trainedLink() {
    StageLink l;
    for (int i = 0; i < 30; i++) {
        const room::Mat34 raw = headPose({0.1 * i, 0.2, -0.05 * i}, 0.3 * i, (i % 5 - 2) * 10 * kDeg);
        l.add(rawToStage(raw), raw);
    }
    return l;
}

guide::State state(std::uint64_t seq, const std::string& step) {
    guide::State s;
    s.pid = 900;
    s.seq = seq;
    s.step = step;
    s.title = "Room setup";
    s.text = "Testing";
    s.conv = guide::Conv{0.2, {0.1, -1.0, 0.3}};
    s.floor = -2.6; // room frame
    return s;
}

struct Rig {
    MapFlow flow;
    StageLink link = trainedLink();
    double now = 0;
    room::Mat34 head = headPose({0, 1.7, 0}, 0.5, 0);
    std::array<room::Mat34, 2> hand{headPose({-0.2, 1.1, -0.3}, 0.5, -60 * kDeg), headPose({0.2, 1.1, -0.3}, 0.5, -60 * kDeg)};
    std::optional<guide::State> st;
    std::array<bool, 2> tracked{true, true};
    std::array<float, 2> stick{0, 0}; // the thumbstick Y per hand (the floor step is nudged with it)

    void frame(std::uint64_t right = 0, std::uint64_t left = 0) {
        room::FrameInput in;
        in.dt = 1.0 / 90;
        in.headValid = true;
        in.head = head;
        for (std::size_t i = 0; i < 2; i++) {
            in.hands[i].valid = in.hands[i].aimValid = true;
            in.hands[i].grip = in.hands[i].aim = hand[i];
            in.hands[i].tracked = tracked[i];
            in.hands[i].stickY = stick[i];
        }
        in.hands[0].buttons = left;
        in.hands[1].buttons = right;
        flow.frame(in, now, st, link);
        now += 1.0 / 90;
    }
    void click(std::uint64_t b = kTrig) {
        frame(b);
        frame();
    }
};

} // namespace

TEST(stage_link_finds_the_yaw_and_shift_and_follows_a_jump) {
    StageLink l = trainedLink();
    CHECK(l.valid());
    CHECK(near(l.yaw(), kLinkYaw, 1e-6));
    CHECK(near(l.shift().x, kLinkT.x, 1e-6) && near(l.shift().y, kLinkT.y, 1e-6) && near(l.shift().z, kLinkT.z, 1e-6));
    const room::Vec3 p{1, 0.5, -2};
    const room::Vec3 q = l.toRaw(l.toStage(p));
    CHECK(near(q.x, p.x, 1e-9) && near(q.z, p.z, 1e-9));
    // one outlier is ignored, ten in a row are the new link (a new offset written)
    const room::Mat34 jump = headPose({0, 0.5, 0}, 0.4 + 0.5, 0);
    l.add(room::mul34(jump, headPose({0, 0.2, 0}, 0, 0)), headPose({0, 0.2, 0}, 0, 0));
    CHECK(near(l.yaw(), kLinkYaw, 1e-6));
    for (int i = 0; i < 10; i++) {
        const room::Mat34 raw = headPose({0.1 * i, 0.2, 0}, 0.2 * i, 0);
        l.add(room::mul34(jump, raw), raw);
    }
    CHECK(near(l.yaw(), 0.9, 1e-6));
    CHECK(!StageLink{}.valid());
}

TEST(coverage_ring_and_hints) {
    Coverage c;
    CHECK(c.fraction() == 0);
    c.look(headPose({0, 1.6, 0}, 0.1, 0));
    CHECK(c.seen(1, 0));
    // the nearest unseen sector to the left (heading grows to the left)
    CHECK(c.hint(15 * kDeg) == "LOOK LEFT" || c.hint(15 * kDeg) == "LOOK RIGHT");
    for (int s = 0; s < 6; s++) {
        c.look(headPose({0, 1.6, 0}, Coverage::sectorHeading(s), 0));
    }
    CHECK(c.hint(Coverage::sectorHeading(5)) == "LOOK LEFT");
    CHECK(c.hint(Coverage::sectorHeading(0)) == "LOOK RIGHT");
    for (int s = 0; s < 12; s++) {
        c.look(headPose({0, 1.6, 0}, Coverage::sectorHeading(s), 0));
    }
    CHECK(c.hint(0) == "LOOK DOWN AT THE FLOOR");
    for (int s = 0; s < 12; s += 2) {
        c.look(headPose({0, 1.6, 0}, Coverage::sectorHeading(s), -50 * kDeg));
    }
    CHECK(c.hint(0) == "LOOK UP");
    c.look(headPose({0, 1.6, 0}, 0, 50 * kDeg));
    c.look(headPose({0, 1.6, 0}, 2, 50 * kDeg));
    CHECK(c.hint(0).empty());
    CHECK(c.fraction() > 0.95);
}

TEST(mapping_lines_and_coverage) {
    Rig r;
    r.frame();
    CHECK(r.flow.lines().front() == "ROOM MAPPING");
    auto s = state(1, "map");
    s.keyframes = 7;
    s.points = 230;
    r.st = s;
    for (int i = 0; i < 12; i++) {
        r.head = headPose({0, 1.7, 0}, Coverage::sectorHeading(i), 0);
        r.frame();
    }
    const auto l = r.flow.lines();
    CHECK(std::ranges::find(l, std::string("VIEWS MAPPED 7   POINTS IN VIEW 230")) != l.end());
    CHECK(r.flow.coverage().fraction() >= 0.7);
    room::Batch b;
    r.flow.build(b, r.now);
    CHECK(b.tris.size() > 36 * 6); // 36 ring segments + the panel + the text
    CHECK(r.flow.takeAnswers().empty());
}

TEST(floor_step_shows_the_drop_and_the_stick_nudges_it) {
    // (room-setup rework) the floor step is no longer a trigger "yes" / "touch the floor": it shows the floor as a
    // drop below the face (default from the eye height) and the thumbstick fine-tunes it.
    Rig r;
    r.flow.setEyeHeight(1.68);
    r.st = state(1, "floor-height");
    r.frame();
    CHECK(r.flow.floorKnown() && says(r.flow, "FLOOR IS")); // the floor drop is shown
    const double d0 = r.flow.floorStage();
    // the stick held down lowers the floor (bigger drop -> lower stage floor); many frames to clear the nudge rate limit
    r.stick[1] = -1.0;
    for (int i = 0; i < 300; i++) {
        r.frame();
    }
    CHECK(r.flow.floorStage() < d0 - 0.02);
    // a plain trigger does not answer the floor step (that is the NEXT chip's job, driven by the pointer)
    r.stick[1] = 0;
    r.click();
    CHECK(r.flow.takeAnswers().empty());
}

TEST(height_step_measures_the_head_and_the_stick_tunes_it) {
    // full setup "рост": the step shows the standing eye height taken from the head, and the stick tunes it
    Rig r;
    r.head = headPose({0, 1.75, 0}, 0.5, 0);
    r.st = state(1, "height");
    r.frame();
    // the eye height 1.75 is shown as a STATURE (eye / 0.933 ~ 1.88 m), not the floor-to-headset distance
    CHECK(says(r.flow, "YOUR HEIGHT 1.88 M"));
    r.stick[1] = 1.0; // stick up: taller
    for (int i = 0; i < 300; i++) {
        r.frame();
    }
    CHECK(says(r.flow, "YOUR HEIGHT") && !says(r.flow, "YOUR HEIGHT 1.88 M")); // changed by the stick
}

TEST(boundary_drawn_with_the_ray) {
    // (room-setup rework) drawing the boundary with the controller ray still works, but the stroke is NOT auto-kept on
    // release and there is no trigger "yes": the user must press the SAVE chip (pointer), so here we check the drawing
    // and the hints, not a submit.
    Rig r;
    auto s = state(1, "boundary");
    s.boundary = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    r.st = s;
    r.frame();
    room::Batch b;
    r.flow.build(b, r.now);
    CHECK(b.lines.size() > 4 * 2 * 2); // the proposed boundary is drawn
    // draw: hold the trigger and sweep the ray's floor point round a circle of ~0.8 m
    s.seq = 2;
    r.st = s;
    for (int i = 0; i <= 120; i++) {
        const double ang = i * 2 * std::numbers::pi / 120;
        r.hand[1] = headPose({0.2, 1.1, -0.3}, ang, -60 * kDeg);
        r.frame(kTrig);
    }
    CHECK(r.flow.drawn().size() > 40);
    r.frame(); // release: no auto-submit; the hint asks for SAVE
    CHECK(r.flow.takeAnswers().empty());
    CHECK(says(r.flow, "PRESS SAVE"));
    // a too short stroke is refused in words
    s.seq = 3;
    r.st = s;
    r.frame();
    for (int i = 0; i < 90; i++) {
        r.hand[1] = headPose({0.2, 1.1, -0.3}, i * 0.005, -60 * kDeg);
        r.frame(kTrig);
    }
    r.frame();
    CHECK(r.flow.takeAnswers().empty());
    CHECK(says(r.flow, "TOO SHORT"));
}

TEST(stop_with_b_twice_and_the_end) {
    Rig r;
    r.st = state(1, "map");
    r.frame();
    r.click(kMenu);
    CHECK(r.flow.lines().back() == "PRESS B AGAIN TO STOP");
    r.click(kMenu);
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "cancel");
    r.st = state(2, "failed");
    r.st->title = "Room setup stopped";
    r.st->text = "stopped in the headset";
    r.frame();
    r.flow.childEnded(1, "pibridge-roommap: stopped in the headset", r.now);
    CHECK(!r.flow.finished(r.now + 1.0));
    CHECK(r.flow.finished(r.now + 2.0));
    CHECK(!r.flow.succeeded());
    // a run that ended before it said anything shows its last line
    Rig q;
    q.frame();
    q.flow.childEnded(1, "pibridge-roommap: cannot start the room setup: no head ring", q.now);
    q.frame();
    const auto l = q.flow.lines();
    CHECK(l.size() >= 2 && l[1].find("CANNOT START") != std::string::npos);
    CHECK(q.flow.finished(q.now + 6));
}

TEST(answers_need_the_link) {
    // without the stage link the raw<->stage transform is unknown: no answer can be formed and the stage floor is not
    // known (so the floor grid is not placed).
    Rig r;
    r.link = StageLink{};
    r.st = state(1, "floor-height");
    r.frame();
    r.click();
    CHECK(r.flow.takeAnswers().empty());
    CHECK(!r.flow.floorKnown());
}

PIBRIDGE_TEST_MAIN()
