// SPDX-License-Identifier: MIT
// pibridge-home without a runtime: the command line, the scene parts (grid, horizon, room, controllers, panel), the
// saved room in stage numbers, and the session rules that keep a game in front (begin once, hidden while another app
// is shown, end and leave on STOPPING, the exit wait). Files only in a temporary folder.
#include "loop.h"
#include "options.h"
#include "scene.h"

#include "pibridge/room/rooms.h"
#include "pibridge_test.h"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <numbers>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::home;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

struct TmpDir {
    std::filesystem::path p;
    TmpDir() {
        p = std::filesystem::temp_directory_path() / ("pibridge-home-test-" + std::to_string(::getpid()));
        std::filesystem::remove_all(p);
        std::filesystem::create_directories(p);
    }
    ~TmpDir() {
        std::error_code ec;
        std::filesystem::remove_all(p, ec);
    }
};

room::Mat34 yawHead(room::Vec3 p, double yaw) {
    return room::poseMatrix(0, std::sin(yaw / 2), 0, std::cos(yaw / 2), p);
}

} // namespace

TEST(options) {
    CHECK(parseArgs({})->resDiv == 1 && !parseArgs({})->selfTest);
    CHECK(parseArgs({"--selftest", "--snapshot", "/tmp/x"})->snapshotDir == "/tmp/x");
    CHECK(!parseArgs({"--snapshot", "/tmp/x"}).has_value()); // only with --selftest
    CHECK(parseArgs({"--res-div", "2"})->resDiv == 2);
    CHECK(!parseArgs({"--res-div", "0"}).has_value());
    CHECK(!parseArgs({"--res-div"}).has_value());
    CHECK(parseArgs({"--rooms", "/r"})->roomsDir == "/r");
    CHECK(!parseArgs({"--bogus"}).has_value());
    CHECK(parseArgs({"-h"})->help);
}

TEST(grid_horizon_on_the_floor_in_stage_and_local) {
    World w;
    room::Batch b;
    const std::size_t g = addGrid(b, w);
    CHECK(g == 25 * 24 * 2 + 32); // 1 m pieces over +-12 m, both ways, plus the play centre ring
    for (const room::Vertex& v : b.lines) {
        CHECK(near(v.y, 0.0, 1e-6));
    }
    const std::size_t h0 = b.lines.size();
    CHECK(addHorizon(b, w) == 128);
    CHECK(near(b.lines[h0].y, kEyeHeight, 1e-6));
    World l = worldFor(false, "/nonexistent-pibridge-rooms");
    CHECK(!l.stage && near(l.floorY, kLocalFloorY) && !l.room);
    room::Batch bl;
    addGrid(bl, l);
    CHECK(near(bl.lines.front().y, kLocalFloorY, 1e-6));
}

TEST(saved_room_in_stage_numbers) {
    TmpDir t;
    room::RoomLib lib(t.p.string());
    room::Room r;
    r.name = "Office";
    r.universe = "1";
    r.points = {{1, 2}, {5, 2}, {5, 5}, {1, 5}};
    r.standing = room::Standing{3, 0.1, 3.5, std::numbers::pi / 2};
    CHECK(lib.save(r, std::time(nullptr)).has_value());
    CHECK(!worldFor(true, t.p.string()).room); // not active yet
    lib.setActive("Office");
    lib.setFrameMode(room::FrameMode::A);
    const World w = worldFor(true, t.p.string());
    CHECK(w.room && w.room->size() == 4 && w.roomName == "Office");
    CHECK(near(w.room->front().x, 1.5) && near(w.room->front().z, -2.0)); // (1,2)-(3,3.5) turned by 90 deg
    const auto b = roomOutline(r, room::FrameMode::B);
    CHECK(near(b->front().x, -1.5) && near(b->front().z, 2.0));
    CHECK(roomOutline(r, room::FrameMode::Off)->front() == (room::Pt2{1, 2}));
    room::Room s = r;
    s.frame = "stage";
    CHECK(roomOutline(s, room::FrameMode::A)->front() == (room::Pt2{1, 2}));
    s.standing.reset();
    s.frame = "raw";
    CHECK(roomOutline(s, room::FrameMode::A)->front() == (room::Pt2{1, 2})); // no standing pose: as is
    s.points.resize(2);
    CHECK(!roomOutline(s, room::FrameMode::A));
    room::Batch bb;
    CHECK(addRoom(bb, w) == 12);
    World none;
    CHECK(addRoom(bb, none) == 0);
}

TEST(panel_and_controllers) {
    const Panel p = placePanel(yawHead({0, 1.7, 0}, 0), 0);
    CHECK(near(p.center.z, -1.8, 1e-9) && near(p.center.y, 1.6, 1e-9));
    CHECK(near(p.right.x, 1, 1e-9) && near(p.up.y, 1));
    CHECK(p.right.cross(p.up).dot(room::Vec3{0, 1.7, 0} - p.center) > 0); // the text faces the head
    // looking straight down: the stage's forward
    const room::Mat34 down = room::poseMatrix(-std::sin(std::numbers::pi / 4), 0, 0, std::cos(std::numbers::pi / 4), {0, 1.7, 0});
    CHECK(near(placePanel(down, 0).center.z, -1.8, 1e-6));
    CHECK(near(placePanel(yawHead({0, 3.5, 0}, 0), 0).center.y, 2.2)); // clamped
    room::Batch b;
    CHECK(addPanel(b, p) == 4 && !b.tris.empty());
    Frame f;
    room::Batch c;
    CHECK(addControllers(c, f) == 0); // no poses: nothing
    f.grip[1] = room::Mat34::identity();
    CHECK(addControllers(c, f) == 3 && c.tris.size() == 9);
    f.aim[1] = room::Mat34::identity();
    room::Batch c2;
    CHECK(addControllers(c2, f) == 4);
    room::Batch all;
    f.panel = p;
    buildScene(all, World{}, f);
    CHECK(all.lines.size() / 2 < 3000);
}

TEST(controllers_shown_only_while_tracked) {
    HandShown h;
    CHECK(!h.update(true, false, 0.0));  // a pose, never tracked: the rest pose before the headset saw it
    CHECK(h.update(true, true, 1.0));    // seen
    CHECK(h.update(true, false, 1.8));   // a short occlusion: still drawn
    CHECK(!h.update(true, false, 2.2));  // lost longer: gone
    CHECK(h.update(true, true, 3.0));    // seen again
    CHECK(!h.update(false, false, 3.1)); // switched off: no pose, gone at once
}

TEST(session_rules_keep_a_game_in_front) {
    Loop l;
    CHECK(l.status() == "starting" && !l.running());
    CHECK(l.onState(XrState::Idle) == Step::None);
    CHECK(l.onState(XrState::Ready) == Step::Begin);
    CHECK(l.running() && l.status() == "running");
    CHECK(l.onState(XrState::Synchronized) == Step::None && !l.submit(true));
    l.onState(XrState::Visible);
    CHECK(l.submit(true) && !l.submit(false));
    l.onState(XrState::Focused);
    CHECK(l.focused() && l.status() == "focused");
    l.onState(XrState::Visible); // a game began: Monado makes it the primary app
    l.onState(XrState::Synchronized);
    CHECK(!l.visible() && l.running() && !l.submit(true));
    CHECK(l.onState(XrState::Ready) == Step::None); // never a second begin
    l.onState(XrState::Visible); // the game ended: shown again, without a new session
    CHECK(l.submit(true));
    CHECK(l.onState(XrState::Stopping) == Step::End);
    CHECK(!l.running() && l.shouldLeave(0) && l.status() == "ending");
    CHECK(l.onState(XrState::Ready) == Step::None); // after STOPPING: leave, never begin again
    CHECK(l.onState(XrState::Stopping) == Step::None);
}

TEST(stop_signal_and_losses) {
    Loop l;
    l.onState(XrState::Ready);
    CHECK(l.requestStop(100.0));  // xrRequestExitSession now
    CHECK(!l.requestStop(100.1)); // once
    CHECK(!l.shouldLeave(101.0));
    CHECK(l.onState(XrState::Stopping) == Step::End && l.shouldLeave(101.0));
    Loop w; // the runtime does not answer
    w.onState(XrState::Ready);
    w.requestStop(5.0);
    CHECK(!w.shouldLeave(7.9) && w.shouldLeave(5.0 + kExitWaitS));
    Loop n; // stopped before the session began
    CHECK(!n.requestStop(1.0) && n.shouldLeave(1.0));
    CHECK(n.onState(XrState::Ready) == Step::None);
    Loop e;
    e.onState(XrState::Ready);
    CHECK(e.onState(XrState::LossPending) == Step::Leave && e.shouldLeave(0));
    Loop i;
    CHECK(i.onInstanceLoss() == Step::Leave && i.shouldLeave(0));
    CHECK(std::string(stateName(XrState::Focused)) == "FOCUSED");
}

TEST(status_fields) {
    Loop l;
    l.onState(XrState::Ready);
    l.onState(XrState::Focused);
    const std::string t = statusFields(l, 77, "Monado", false, "", "").text();
    CHECK(t.find("\"state\": \"focused\"") != std::string::npos);
    CHECK(t.find("\"space\": \"local\"") != std::string::npos);
    CHECK(t.find("\"pid\": 77") != std::string::npos);
    CHECK(t.find("\"visible\": true") != std::string::npos);
}

PIBRIDGE_TEST_MAIN()
