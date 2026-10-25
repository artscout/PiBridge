// SPDX-License-Identifier: MIT
// The in-headset room setup's two files (guide_ipc.h): round trips, refusals, and Conv against session::Alignment.
#include "pibridge/roommap/guide_ipc.h"
#include "pibridge/roommap/session.h"
#include "pibridge_test.h"

#include <cstdlib>
#include <filesystem>
#include <unistd.h>

using namespace pibridge::roommap;

namespace {

bool near(double a, double b, double e = 1e-4) { return std::fabs(a - b) <= e; }

} // namespace

TEST(state_round_trip) {
    guide::State s;
    s.pid = 42;
    s.seq = 7;
    s.step = "boundary";
    s.title = "Play area";
    s.text = "Pull the trigger\nto keep it";
    s.progress = 0.25;
    s.keyframes = 12;
    s.points = 340;
    s.conv = guide::Conv{0.3, {1.0, -1.5, 0.25}};
    s.floor = -1.52;
    s.boundary = {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}};
    const auto t = guide::format_state(s);
    auto p = guide::parse_state(t);
    CHECK(p.has_value());
    CHECK(p->text == "Pull the trigger to keep it"); // one line
    s.text = p->text;
    CHECK(*p == s);
    // the minimum: a step
    auto m = guide::parse_state("pibridge-roommap-guide 1\nstep map\nnewkey 1 2 3\n");
    CHECK(m && m->step == "map" && !m->conv && !m->floor && m->boundary.empty());
}

TEST(state_refusals) {
    CHECK(!guide::parse_state(""));
    CHECK(!guide::parse_state("something else\nstep map\n"));
    CHECK(!guide::parse_state("pibridge-roommap-guide 1\npid 3\n"));          // no step
    CHECK(!guide::parse_state("pibridge-roommap-guide 1\nstep map\nfloor x\n"));
    CHECK(!guide::parse_state("pibridge-roommap-guide 1\nstep map\nconv 1 2\n"));
    CHECK(!guide::parse_state("pibridge-roommap-guide 1\nstep map\nboundary 1 2 3\n"));
    CHECK(!guide::parse_state("pibridge-roommap-guide 1\nstep map\nseq -1\n"));
}

TEST(answer_round_trip_and_shapes) {
    guide::Answer a{77, 3, "forward", "forward", {0.1, -0.2, 1.5, 1.62}};
    auto p = guide::parse_answer(guide::format_answer(a));
    CHECK(p && *p == a);
    guide::Answer y{1, 1, "floor-confirm", "yes", {}};
    CHECK(guide::parse_answer(guide::format_answer(y)).value_or(guide::Answer{}) == y);
    guide::Answer path{1, 2, "boundary", "path", {0, 0, 1, 0, 1, 1}};
    CHECK(guide::parse_answer(guide::format_answer(path)).has_value());
    // wrong number counts are refused
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer touch\n"));
    CHECK(guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer height -1.5 1.68\n").has_value());
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer height -1.5\n"));
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer forward 1 2 3\n"));
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer path 1 2 3 4\n"));
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer yes 1\n"));
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\n"));
    CHECK(!guide::parse_answer("pibridge-roomsetup-guide 1\nid 1\nstep x\nanswer touch nan\n"));
}

TEST(conv_is_the_alignment) {
    const session::Alignment al{0.7, {0.4, -1.2, 2.0}};
    const guide::Conv c{al.yaw, {al.t[0], al.t[1], al.t[2]}};
    const std::array<double, 3> p{1.0, 1.7, -0.5};
    const auto q = al.pose().apply(p);
    const auto r = c.apply(p);
    CHECK(near(q[0], r[0]) && near(q[1], r[1]) && near(q[2], r[2]));
    const auto back = c.inverse_apply(r);
    CHECK(near(back[0], p[0]) && near(back[1], p[1]) && near(back[2], p[2]));
}

TEST(files_are_written_atomically) {
    const char* tmp = std::getenv("TMPDIR");
    const std::string dir = std::string(tmp && *tmp ? tmp : "/tmp") + "/pibridge-guide-test-" + std::to_string(::getpid());
    std::filesystem::create_directories(dir);
    CHECK(!guide::read_file(dir, guide::STATE_FILE));
    CHECK(guide::write_atomic(dir, guide::STATE_FILE, "pibridge-roommap-guide 1\nstep map\n").has_value());
    CHECK(guide::read_file(dir, guide::STATE_FILE).value_or("") == "pibridge-roommap-guide 1\nstep map\n");
    CHECK(!std::filesystem::exists(dir + "/state.tmp"));
    std::filesystem::remove_all(dir);
}

TEST(points_round_trip) {
    std::vector<guide::P3f> pts{{0.0f, 1.0f, -2.0f}, {3.5f, -4.25f, 0.125f}, {-10.0f, 0.0f, 7.0f}};
    const std::string s = guide::format_points(pts);
    const std::vector<guide::P3f> back = guide::parse_points(s);
    CHECK(back == pts);
    CHECK(guide::parse_points("").empty());             // empty buffer
    CHECK(guide::parse_points("PT01").empty());         // magic but no count
    CHECK(guide::parse_points("junk nonsense").empty()); // wrong magic
    CHECK(guide::parse_points(s.substr(0, s.size() - 1)).empty()); // truncated -> ignored
    CHECK(guide::format_points({}) == guide::format_points({})); // empty cloud round-trips
    CHECK(guide::parse_points(guide::format_points({})).empty());
}

PIBRIDGE_TEST_MAIN()
