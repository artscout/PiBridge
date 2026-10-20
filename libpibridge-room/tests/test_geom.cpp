// SPDX-License-Identifier: MIT
// Ported from ZoneTweak geom2_test.go and store_test.go (TestRigid's pure part) (MIT, Copyright (c) 2026 killdano).
#include "pibridge/room/geom.h"
#include "pibridge_test.h"

#include <numbers>

using namespace pibridge::room;

TEST(inset_and_fit) {
    const Poly sq{{-2, -1.5}, {2, -1.5}, {2, 1.5}, {-2, 1.5}};
    const Poly in = insetPoly(sq, 0.5);
    CHECK_NEAR(in[0].x, -1.5, 1e-9);
    CHECK_NEAR(in[0].z, -1.0, 1e-9);
    const Poly rev{sq[3], sq[2], sq[1], sq[0]};
    const Poly in2 = insetPoly(rev, 0.5);
    CHECK_NEAR(std::fabs(in2[0].x), 1.5, 1e-9);
    CHECK_NEAR(distToBoundary(sq, {1.7, 0}), 0.3, 1e-9);
    CHECK(distToBoundary(sq, {5, 0}) == 0);
    const auto f = fitArea(sq, 0);
    CHECK(f.has_value());
    CHECK_NEAR(f->w, 4, 0.01);
    CHECK_NEAR(f->h, 3, 0.01);
    const Poly L{{-2, -2}, {2, -2}, {2, 0}, {0, 0}, {0, 2}, {-2, 2}};
    const Poly Lc = rigidFromYaw(0, 0.5, -0.5).inv().all(L);
    const auto fl = fitArea(Lc, 0);
    CHECK(fl.has_value() && fl->w > 0 && fl->h > 0);
    CHECK(!rectFits(Lc, fl->w / 2 + 0.01, fl->h / 2 + 0.01)); // the fit is maximal
    CHECK(fitArea(sq, 0.1).has_value());
    CHECK(!fitArea(rigidFromYaw(0, 10, 0).all(sq), 0).has_value()); // the origin outside
}

TEST(rigid) {
    const Rigid2 r = rigidFromYaw(0.7, 1.5, -2);
    const Rigid2 q = rigidFromYaw(-1.1, 0.3, 0.4);
    const Pt2 p{0.25, -3};
    const Pt2 a = r.then(q).apply(p);
    const Pt2 b = q.apply(r.apply(p));
    const Pt2 c = r.inv().apply(r.apply(p));
    CHECK_NEAR(a.x, b.x, 1e-12);
    CHECK_NEAR(a.z, b.z, 1e-12);
    CHECK_NEAR(c.x, p.x, 1e-12);
    CHECK_NEAR(c.z, p.z, 1e-12);
    CHECK_NEAR(rigidFromYaw(0.4, 0, 0).angle(), 0.4, 1e-12);
}

TEST(area_perimeter_orientation) {
    const Poly sq = rectangle(2, 2);
    CHECK_NEAR(polyArea(sq), 4, 1e-12); // counter-clockwise
    CHECK_NEAR(polyPerimeter(sq), 8, 1e-12);
    const Poly cw(sq.rbegin(), sq.rend());
    CHECK_NEAR(polyArea(cw), -4, 1e-12);
    CHECK(polyArea(ccw(cw)) > 0);
    CHECK(pointInPoly(sq, 0, 0));
    CHECK(!pointInPoly(sq, 1.5, 0));
    const auto c = centroid(rigidFromYaw(0, 3, -1).all(sq));
    CHECK(c.has_value());
    CHECK_NEAR(c->x, 3, 1e-12);
    CHECK_NEAR(c->z, -1, 1e-12);
}

TEST(simple_polygons) {
    CHECK(isSimple(rectangle(2, 3)));
    const Poly bow{{0, 0}, {2, 2}, {2, 0}, {0, 2}}; // a bow tie crosses itself
    CHECK(!isSimple(bow));
    CHECK(!isSimple({{0, 0}, {1, 0}}));
    CHECK(!isSimple({{0, 0}, {1, 0}, {2, 0}})); // no area
    CHECK(!isSimple({{0, 0}, {0.001, 0}, {1, 1}, {0, 1}})); // two corners on top of each other
}

TEST(simplify_and_square_up) {
    // a 4 x 3 room with extra points on the walls and a little noise
    Poly p{{-2, -1.5}, {0, -1.52}, {2, -1.5}, {2.01, 0}, {2, 1.5}, {0, 1.49}, {-2, 1.5}, {-1.99, 0}};
    const Poly s = simplify(p, 0.08);
    CHECK(s.size() == 4);
    const Poly q = squareUp(simplify(p, 0.10), domAngle(p), 0.20);
    CHECK(q.size() == 4);
    CHECK_NEAR(std::fabs(polyArea(q)), 12, 0.15);
    // a turned room squares up in its own frame
    const Poly t = rigidFromYaw(0.3, 0, 0).all(p);
    CHECK_NEAR(std::remainder(domAngle(t) - 0.3, std::numbers::pi / 2), 0, 0.02);
    CHECK(squareUp({{0, 0}, {1, 0}, {0, 1}}, 0, 0.2).empty());
}

TEST(snap) {
    const Pt2 prev{0, 0};
    const Pt2 next{2, 2};
    const Pt2 s = snapToNeighbors({0.05, 1.0}, &prev, nullptr, 0); // almost straight above prev
    CHECK_NEAR(s.x, 0, 1e-12);
    CHECK_NEAR(s.z, 1.0, 1e-12);
    const Pt2 t = snapToNeighbors({1.0, 1.0}, &prev, &next, 0); // 45 degrees: nothing to snap
    CHECK_NEAR(t.x, 1.0, 1e-12);
}

TEST(douglas_peucker) {
    Poly line;
    for (int i = 0; i <= 20; i++) {
        line.push_back({i * 0.1, (i % 2) * 0.01});
    }
    const Poly d = douglasPeucker(line, 0.05);
    CHECK(d.size() == 2);
    CHECK(d.front() == line.front());
    CHECK(d.back() == line.back());
    Poly corner{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
    CHECK(douglasPeucker(corner, 0.05).size() == 3);
}

TEST(polygon_from_trace) {
    // walking a 3 x 2 room with 2 cm noise, several points per metre, back to the start
    Poly path;
    const Poly room = rectangle(3, 2);
    for (std::size_t i = 0; i < room.size(); i++) {
        const Pt2 a = room[i];
        const Pt2 b = room[(i + 1) % room.size()];
        for (int k = 0; k < 30; k++) {
            const double t = k / 30.0;
            const double n = ((k * 7919) % 5 - 2) * 0.01;
            path.push_back({a.x + (b.x - a.x) * t + n, a.z + (b.z - a.z) * t - n});
        }
    }
    path.push_back(room.front());
    const Poly p = polygonFromTrace(path, 0.08);
    CHECK(p.size() == 4);
    CHECK(polyArea(p) > 0);
    CHECK_NEAR(polyArea(p), 6, 0.3);
    // a walk that crosses itself is refused
    CHECK(polygonFromTrace({{0, 0}, {2, 2}, {2, 0}, {0, 2}, {0, 0.01}}, 0.05).empty());
    CHECK(polygonFromTrace({{0, 0}, {0.01, 0}}, 0.05).empty());
}

PIBRIDGE_TEST_MAIN()
