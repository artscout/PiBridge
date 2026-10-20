// SPDX-License-Identifier: MIT
// pibridge-roomsetup --map's headset side (mapflow.h) with scripted frames and made-up pibridge-roommap states: the stage
// link, the coverage hints, the floor yes / touch, the boundary kept or drawn, forward by trigger and by holding
// still, the stop, the end. No runtime, no files.
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

    void frame(std::uint64_t right = 0, std::uint64_t left = 0) {
        room::FrameInput in;
        in.dt = 1.0 / 90;
        in.headValid = true;
        in.head = head;
        for (std::size_t i = 0; i < 2; i++) {
            in.hands[i].valid = in.hands[i].aimValid = true;
            in.hands[i].grip = in.hands[i].aim = hand[i];
            in.hands[i].tracked = tracked[i];
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

TEST(floor_yes_and_touch) {
    Rig r;
    r.st = state(1, "floor-confirm");
    r.frame();
    // the floor in the stage: room y -2.6 -> raw -1.6 -> stage -0.1
    CHECK(r.flow.floorKnown() && near(r.flow.floorStage(), -0.1, 1e-6));
    r.click(); // the controller at the hip (0.6 m below the head): yes
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "yes" && a[0].step == "floor-confirm" && a[0].id == 1);
    r.click(); // no second answer before pibridge-roommap reacted
    CHECK(r.flow.takeAnswers().empty());
    // pibridge-roommap asks again (new seq); the controller is on the real floor, 30 cm under the grid
    r.st = state(2, "floor-confirm");
    r.hand[1] = headPose({0.2, -0.4, -0.3}, 0.5, -60 * kDeg);
    r.click();
    a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "touch");
    // stage -0.4 -> raw -1.9 -> room -2.9
    CHECK(a.size() == 1 && near(a[0].nums[0], -2.9, 1e-6));
}

TEST(floor_touch_needs_a_tracked_controller_and_counts_near_the_grid) {
    // live 2026-10-19: the controller lay on the floor out of the cameras' view; its held pose at hip height made the
    // trigger a "yes" and the touch was lost
    Rig r;
    r.st = state(1, "floor-confirm");
    r.frame();
    r.tracked[1] = false;
    r.click();
    CHECK(r.flow.takeAnswers().empty());
    CHECK(says(r.flow, "DOES NOT SEE THAT CONTROLLER"));
    // seen again, 0.8 m below the head but within 30 cm of the grid (-0.1): a touch, not a yes
    r.tracked[1] = true;
    r.hand[1] = headPose({0.2, 0.9 - 0.75, -0.3}, 0.5, -60 * kDeg);
    r.click();
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "touch");
}

TEST(floor_from_the_eye_height_standing_still_and_level) {
    // user decision 2026-10-19: the floor = the head standing upright minus the eye height, the default method
    Rig r;
    r.flow.setEyeHeight(1.68);
    r.st = state(1, "floor-confirm");
    r.head = headPose({0, 1.7, 0}, 0.5, 2 * kDeg);
    for (int i = 0; i < 100; i++) {
        r.frame();
    }
    CHECK(r.flow.standingFloor(r.now).has_value());
    CHECK(says(r.flow, "YOUR EYE HEIGHT 1.68 M"));
    r.click(); // the controller at the hip: the floor from the height
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "height");
    // stage 1.7 - 1.68 = 0.02 -> raw -1.48 -> room -2.48
    CHECK(a.size() == 1 && near(a[0].nums[0], -2.48, 1e-6) && near(a[0].nums[1], 1.68, 1e-9));
    // A: the grid is on the floor
    r.st = state(2, "floor-confirm");
    r.frame();
    r.click(1ULL << room::bit::Grip);
    a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "yes");
    // a controller on the floor is still a touch
    r.st = state(3, "floor-confirm");
    r.hand[1] = headPose({0.2, -0.05, -0.3}, 0.5, -60 * kDeg);
    r.click();
    a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "touch");
}

TEST(floor_from_the_eye_height_refuses_a_moving_or_tilted_head) {
    Rig r;
    r.flow.setEyeHeight(1.68);
    r.st = state(1, "floor-height");
    for (int i = 0; i < 100; i++) { // nodding: 5 cm up and down
        r.head = headPose({0, 1.7 + 0.05 * std::sin(i * 0.2), 0}, 0.5, 0);
        r.frame();
    }
    r.click();
    CHECK(r.flow.takeAnswers().empty());
    CHECK(says(r.flow, "STAND STILL AND UPRIGHT"));
    r.head = headPose({0, 1.7, 0}, 0.5, -35 * kDeg); // looking down at the floor
    for (int i = 0; i < 100; i++) {
        r.frame();
    }
    r.click();
    CHECK(r.flow.takeAnswers().empty());
    // in the floor-height step a low controller is not a touch: the height only
    r.head = headPose({0, 1.7, 0}, 0.5, 0);
    r.hand[1] = headPose({0.2, -0.05, -0.3}, 0.5, -60 * kDeg);
    for (int i = 0; i < 100; i++) {
        r.frame();
    }
    r.click();
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "height");
    // without an eye height nothing of this
    Rig q;
    CHECK(!q.flow.standingFloor(q.now));
}

TEST(floor_touch_refuses_a_hand_in_the_air) {
    Rig r;
    auto s = state(1, "floor-touch");
    s.floor.reset();
    r.st = s;
    r.frame();
    CHECK(!r.flow.floorKnown() && near(r.flow.floorStage(), 1.7 - 1.6, 1e-9));
    r.hand[1] = headPose({0.2, 1.5, -0.3}, 0.5, 0);
    r.click();
    CHECK(r.flow.takeAnswers().empty());
    CHECK(says(r.flow, "NOT THE FLOOR"));
}

TEST(boundary_kept_with_a_click_or_drawn) {
    Rig r;
    auto s = state(1, "boundary");
    s.boundary = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    r.st = s;
    r.frame();
    room::Batch b;
    r.flow.build(b, r.now);
    CHECK(b.lines.size() > 4 * 2 * 2);
    r.click();
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "yes");
    // draw: hold the trigger and sweep the ray's floor point round a circle of 0.8 m
    s.seq = 2;
    r.st = s;
    for (int i = 0; i <= 120; i++) {
        const double ang = i * 2 * std::numbers::pi / 120;
        // the hand turns: its ray (60 deg down from 1.1 m) hits the floor (-0.1) about 0.69 m away
        r.hand[1] = headPose({0.2, 1.1, -0.3}, ang, -60 * kDeg);
        r.frame(kTrig);
    }
    CHECK(r.flow.drawn().size() > 40);
    r.frame();
    a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "path" && a[0].nums.size() >= 80);
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

TEST(forward_by_trigger_and_by_holding_still) {
    Rig r;
    r.st = state(1, "forward");
    r.head = headPose({0.5, 1.7, -0.4}, 0.25, -5 * kDeg);
    r.frame();
    r.click();
    auto a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "forward");
    if (a.size() == 1) {
        // heading in the room = stage heading - link yaw + room fix yaw
        CHECK(near(a[0].nums[2], 0.25 - kLinkYaw + 0.2, 1e-6));
        // the head in the room: raw = Ry(-0.4)(p - t), room = Ry(0.2) raw + (0.1, -1, 0.3)
        CHECK(near(a[0].nums[3], 1.7 - 1.5 - 1.0, 1e-6));
    }
    // still for 3 s: forward by itself
    r.st = state(2, "forward");
    for (int i = 0; i < 90 * 6; i++) {
        r.frame();
    }
    a = r.flow.takeAnswers();
    CHECK(a.size() == 1 && a[0].kind == "forward");
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
    Rig r;
    r.link = StageLink{};
    r.st = state(1, "floor-confirm");
    r.frame();
    r.click();
    CHECK(r.flow.takeAnswers().empty());
    CHECK(says(r.flow, "WAITING FOR THE HEAD TRACKING"));
}

PIBRIDGE_TEST_MAIN()
