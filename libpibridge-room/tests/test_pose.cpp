// SPDX-License-Identifier: MIT
// The fake pose source and the tracer.
#include "pibridge/room/pose.h"
#include "pibridge_test.h"

using namespace pibridge::room;

TEST(fake_walk_traces_the_room) {
    FakePoseSource walk(rectangle(4, 3), 0.8, 0.5);
    CHECK_NEAR(walk.lapSeconds(), 14 / 0.8, 1e-9);
    TraceFollow follow;
    std::optional<Poly> got;
    int samples = 0;
    for (double t = 0; t < walk.lapSeconds() + 2; t += 1.0 / 60) {
        const auto s = walk.sample(t);
        CHECK(s.has_value());
        CHECK(s->head.valid && s->head.pos.y > 1.6);
        if (auto p = follow.step(*s)) {
            got = p;
        }
        samples++;
    }
    CHECK(samples > 100);
    CHECK(got.has_value());
    CHECK(got->size() == 4);
    CHECK(polyArea(*got) > 0);
    CHECK_NEAR(polyArea(*got), 12, 0.3);
    CHECK(!follow.rec.running());
}

TEST(head_stays_inside) {
    FakePoseSource walk(rectangle(4, 3));
    for (double t = 1; t < 15; t += 0.5) {
        const auto s = walk.sample(t);
        CHECK(pointInPoly(rectangle(4, 3), s->head.pos.x, s->head.pos.z));
    }
}

TEST(recorder_thins_points) {
    TraceRecorder r;
    r.add({0, 0}); // not running: ignored
    CHECK(r.path().empty());
    r.start();
    for (int i = 0; i < 100; i++) {
        r.add({i * 0.001, 0});
    }
    CHECK(r.path().size() == 2); // one every 5 cm
    CHECK(r.finish().empty());   // a line is no room
    CHECK(!FakePoseSource({{0, 0}, {1, 0}}).sample(1).has_value());
}

PIBRIDGE_TEST_MAIN()
