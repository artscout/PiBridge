// SPDX-License-Identifier: MIT
// Partly ported from ZoneTweak profiles_test.go (MIT, Copyright (c) 2026 killdano).
#include "pibridge/room/files.h"
#include "pibridge/room/rooms.h"
#include "pibridge_test.h"
#include "fixtures.h"

using namespace pibridge::room;

namespace {

Room room(const std::string& name, Poly p) {
    Room r;
    r.name = name;
    r.universe = "31";
    r.height = 3;
    r.points = std::move(p);
    return r;
}

} // namespace

TEST(spaces) {
    fixtures::TempDir t;
    RoomLib l(t.path + "/rooms");
    const Poly pts{{0, 0}, {2, 0}, {2, 3}, {0, 3}};
    CHECK(l.save(room("Door Open", pts), 1760000000).has_value());
    CHECK(l.save(room("door closed", Poly(pts.begin(), pts.begin() + 3)), 1760000000).has_value());
    Poly five = pts;
    five.push_back({-1, 1});
    CHECK(l.save(room("Door Open", five), 1760000100).has_value());
    const auto ls = l.list();
    CHECK(ls.size() == 2);
    CHECK(ls[0].name == "door closed");
    CHECK(ls[1].points.size() == 5);
    const auto p = l.find("DOOR OPEN");
    CHECK(p.has_value() && p->points.size() == 5);
    CHECK(p->created == rfc3339(1760000000)); // kept from the first save
    CHECK(p->saved == rfc3339(1760000100));
    l.setActive("Door Open");
    CHECK(l.active() == "Door Open");
    CHECK(l.remove("door open").has_value());
    CHECK(l.list().size() == 1);
    CHECK(l.active().empty()); // removing the active room clears it
    CHECK(!l.save(room("  ", pts), 1760000000).has_value());
    CHECK(!l.save(room("x", {{0, 0}}), 1760000000).has_value());
    CHECK(!l.remove("nothing").has_value());
}

TEST(file_names) {
    RoomLib l("/r");
    CHECK(l.fileFor("Door Open") == "/r/door-open.json");
    CHECK(l.fileFor("  Büro_2!  ") == "/r/bro-2.json");
    CHECK(l.fileFor("***") == "/r/space.json");
}

TEST(full_room_round_trip) {
    Room r = room("Living room", rectangle(3, 2.5));
    r.standing = Standing{0.1, 0.02, -0.3, 0.7};
    r.playArea = Size2{2.5, 2};
    r.warningDistance = 0.4;
    r.saved = "2026-10-19T10:00:00+02:00";
    const auto back = roomFromJson(roomToJson(r));
    CHECK(back.has_value());
    CHECK(back->name == r.name);
    CHECK(back->points == r.points);
    CHECK(back->standing.has_value() && *back->standing == *r.standing);
    CHECK_NEAR(back->floorOffset(), 0.02, 1e-12);
    CHECK_NEAR(back->playArea->w, 2.5, 1e-12);
    CHECK_NEAR(*back->warningDistance, 0.4, 1e-12);
    const UniverseData d = back->toUniverse();
    CHECK(d.bounds == r.points);
    CHECK_NEAR(d.height, 3, 1e-12);
    // a ZoneTweak space (no standing, no play area) loads too
    const auto zt = roomFromJson(R"({"name":"Door","universe":"31","height":3,"points":[[0,0],[2,0],[2,3]],"frame":"raw","saved":"x"})");
    CHECK(zt.has_value() && !zt->standing && !zt->playArea);
    CHECK(!roomFromJson("{\"name\":\"x\",\"points\":[]}").has_value());
}

TEST(room_rename) {
    fixtures::TempDir t;
    RoomLib l(t.path + "/rooms");
    CHECK(l.save(room("Old", rectangle(2, 2)), 1760000000).has_value());
    CHECK(l.save(room("Other", rectangle(2, 2)), 1760000000).has_value());
    l.setActive("Old");
    CHECK(l.rename("Old", "New name", 1760000100).has_value());
    CHECK(!l.find("Old").has_value());
    CHECK(l.find("new name").has_value());
    CHECK(l.active() == "New name");
    CHECK(!l.rename("New name", "Other", 1760000100).has_value()); // taken
    CHECK(!l.rename("Missing", "X", 1760000100).has_value());
    CHECK(l.rename("New name", "NEW NAME", 1760000100).has_value()); // same file, new spelling
    CHECK(l.find("new name")->name == "NEW NAME");
}

TEST(frame_mode_file) {
    fixtures::TempDir t;
    RoomLib l(t.path + "/rooms");
    CHECK(l.frameMode(FrameMode::Off) == FrameMode::Off);
    l.setFrameMode(FrameMode::A);
    CHECK(l.frameMode(FrameMode::Off) == FrameMode::A);
}

PIBRIDGE_TEST_MAIN()
