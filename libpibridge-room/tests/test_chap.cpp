// SPDX-License-Identifier: MIT
// Partly ported from ZoneTweak store_test.go and geom2_test.go (MIT, Copyright (c) 2026 killdano), with synthetic
// files instead of a real room.
#include "pibridge/room/chap.h"
#include "pibridge/room/files.h"
#include "pibridge_test.h"
#include "fixtures.h"

#include <filesystem>

using namespace pibridge::room;

namespace {

Store load(const fixtures::TempDir& t, const std::string& prefer = {}) {
    auto s = Store::load({t.path + "/config"}, prefer);
    CHECK(s.has_value());
    return std::move(*s);
}

} // namespace

TEST(load_and_pick) {
    fixtures::TempDir t;
    const std::string cfg = t.sub("config");
    fixtures::write(cfg + "/chaperone_info.vrchap", fixtures::vrchap());
    Store s = load(t);
    CHECK(s.ids().size() == 2);
    CHECK(s.universe() == "1111111111" || s.universe() == "2"); // same file: most points or the first
    Store p = load(t, "2");
    CHECK(p.universe() == "2");
    CHECK(p.stagePts().size() == 4);
    CHECK_NEAR(p.height(), 2.43, 1e-9);
    CHECK_NEAR(p.playArea().w, 2.5, 1e-9);
    CHECK(p.mode() == FrameMode::Off); // not a Pimax universe
    CHECK(p.universes().at("2").time == "Sun Oct 18 09:00:00 2026");
    CHECK(p.nextUniverse() == "1111111111");
    CHECK(!Store::load({t.sub("empty")}).has_value());
}

TEST(save_keeps_unknown_fields_and_other_universes) {
    fixtures::TempDir t;
    const std::string cfg = t.sub("config");
    const std::string file = cfg + "/chaperone_info.vrchap";
    fixtures::write(file, fixtures::vrchap());
    Store s = load(t, "1111111111");
    const Poly room{{-1, -1}, {1, -1}, {1.5, 0}, {1, 1}, {-1, 1}};
    auto r = s.save(room, 2.5, {}, 1760000000);
    CHECK(r.has_value());
    CHECK(r->written.size() == 1);
    auto doc = readJsonFile(file);
    CHECK(doc.has_value());
    CHECK(doc->get("unknown_top").get("keep").size() == 3);
    CHECK(doc->get("jsonid").str() == "chaperone_info");
    CHECK(doc->get("version").num() == 5);
    const Json* u = findUniverse(*doc, "1111111111");
    CHECK(u != nullptr);
    CHECK(u->get("x_vendor_field").str() == "keep me");
    CHECK(u->get("seated").get("yaw").num() == 0.5);
    const auto [pts, h] = boundsOf(*u);
    CHECK(pts.size() == 5);
    CHECK_NEAR(h, 2.5, 1e-9);
    CHECK_NEAR(pts[2].x, 1.5, 1e-9);
    // the other universe is exactly as it was
    auto orig = parseJson(fixtures::vrchap());
    CHECK(*findUniverse(*doc, "2") == *findUniverse(*orig, "2"));
    // one backup, of the original, made before the first write only
    CHECK(backupsOf(file).size() == 1);
    CHECK(s.save(room, 2.4, {}, 1760000100).has_value());
    CHECK(backupsOf(file).size() == 1);
    auto bak = readFile(backupsOf(file).front());
    CHECK(bak.has_value() && *bak == fixtures::vrchap());
    // a reload sees the new boundary
    Store s2 = load(t, "1111111111");
    CHECK(s2.stagePts().size() == 5);
    CHECK_NEAR(s2.height(), 2.4, 1e-9);
}

TEST(restore_original_from_oldest_backup) {
    fixtures::TempDir t;
    const std::string file = t.sub("config") + "/chaperone_info.vrchap";
    fixtures::write(file, fixtures::vrchap());
    {
        Store s = load(t, "2");
        CHECK(s.save(rectangle(1, 1), 2.43, {}, 1760000000).has_value());
    }
    {
        Store s = load(t, "2"); // a new session backs up again (the edited file)
        CHECK(s.save(rectangle(2, 2), 2.43, {}, 1760003600).has_value());
        CHECK(backupsOf(file).size() == 2);
        auto b = s.oldestBackup();
        CHECK(b.has_value());
        CHECK(b->stagePts.size() == 4);
        CHECK_NEAR(std::fabs(polyArea(b->stagePts)), 6, 1e-9); // the original 3 x 2 room
        CHECK_NEAR(b->data.playArea.w, 2.5, 1e-9);
    }
}

TEST(frame_modes_round_trip) {
    fixtures::TempDir t;
    fixtures::write(t.sub("config") + "/chaperone_info.vrchap", fixtures::vrchap());
    Store s = load(t, "1111111111");
    for (int m = 0; m < kFrameModes; m++) {
        for (const Pt2 p : s.rawPts()) {
            const Pt2 q = s.toFileMode(static_cast<FrameMode>(m), s.toStageMode(static_cast<FrameMode>(m), p));
            CHECK_NEAR(q.x, p.x, 1e-9);
            CHECK_NEAR(q.z, p.z, 1e-9);
        }
    }
    // ZoneTweak TestStandingRigid: mode A's file conversion is the standing pose
    const Pt2 p{1.2, -0.4};
    const Pt2 a = s.standingRigid().apply(p);
    const Pt2 b = s.toFileMode(FrameMode::A, p);
    CHECK_NEAR(a.x, b.x, 1e-9);
    CHECK_NEAR(a.z, b.z, 1e-9);
}

TEST(moving_the_center_keeps_the_walls) {
    // ZoneTweak TestInsetAndFit's centre part, for both readings of the file
    for (const FrameMode mode : {FrameMode::A, FrameMode::Off}) {
        fixtures::TempDir t;
        const std::string file = t.sub("config") + "/chaperone_info.vrchap";
        fixtures::write(file, fixtures::vrchap());
        Store s = load(t, "1111111111");
        (void)s.setMode(mode, s.stagePts());
        const Poly before = s.stagePts();
        const Rigid2 center = rigidFromYaw(0.4, 0.3, -0.2);
        SaveExtras ex;
        ex.standing = s.standingForCenter(center);
        ex.playArea = Size2{2, 1.5};
        CHECK(s.save(before, 2.43, ex, 1760000000).has_value());
        s.setAdj(center);
        // the walls are where they were in the old stage
        for (std::size_t i = 0; i < before.size(); i++) {
            CHECK_NEAR(s.stagePts()[i].x, before[i].x, 1e-5);
            CHECK_NEAR(s.stagePts()[i].z, before[i].z, 1e-5);
        }
        // a fresh load (new stage = new standing): moving back by the centre gives the old stage
        auto s2 = Store::load({t.path + "/config"}, "1111111111");
        CHECK(s2.has_value());
        (void)s2->setMode(mode, s2->stagePts());
        for (std::size_t i = 0; i < before.size(); i++) {
            const Pt2 back = center.apply(s2->stagePts()[i]);
            CHECK_NEAR(back.x, before[i].x, 1e-4);
            CHECK_NEAR(back.z, before[i].z, 1e-4);
        }
        CHECK_NEAR(s2->playArea().w, 2, 1e-6);
        // centerForStanding undoes standingForCenter
        const Rigid2 c2 = s.centerForStanding(s.standingForCenter(center));
        CHECK_NEAR(c2.tx, center.tx, 1e-9);
        CHECK_NEAR(c2.angle(), center.angle(), 1e-9);
    }
}

TEST(floor_delta_and_write_universe) {
    fixtures::TempDir t;
    const std::string file = t.sub("config") + "/chaperone_info.vrchap";
    fixtures::write(file, fixtures::vrchap());
    Store s = load(t, "1111111111");
    SaveExtras ex;
    ex.floorDelta = 0.05;
    CHECK(s.save(s.stagePts(), 2.43, ex, 1760000000).has_value());
    auto doc = readJsonFile(file);
    CHECK_NEAR(universeData(*findUniverse(*doc, "1111111111")).standing.y, 0.06, 1e-9);
    // applying a room in file numbers
    UniverseData d;
    d.bounds = rectangle(2, 2);
    d.height = 2.2;
    d.standing = {0.5, 0.0, 0.5, 1.0};
    d.playArea = {1.5, 1.5};
    CHECK(s.writeUniverse("2", d, true, true, 1760000100).has_value());
    doc = readJsonFile(file);
    const UniverseData w = universeData(*findUniverse(*doc, "2"));
    CHECK(w.bounds == d.bounds);
    CHECK_NEAR(w.standing.x, 0.5, 1e-9);
    CHECK_NEAR(w.standing.yaw, 1.0, 1e-9);
    CHECK_NEAR(w.playArea.w, 1.5, 1e-9);
    CHECK(!s.writeUniverse("nope", d, true, true, 1760000100).has_value());
}

TEST(atomic_write_leaves_no_tmp) {
    fixtures::TempDir t;
    const std::string p = t.path + "/x.json";
    CHECK(writeFileAtomic(p, "{}").has_value());
    CHECK(!std::filesystem::exists(p + ".tmp"));
    CHECK(readFile(p).value_or("") == "{}");
    CHECK(!writeFileAtomic(t.path + "/no/such/dir/x", "1").has_value());
}

TEST(config_dirs) {
    fixtures::TempDir t;
    Env env{t.path, t.path + "/cfg", t.path + "/data"};
    const std::string steam = t.sub("data/Steam/config");
    const std::string other = t.sub("elsewhere");
    t.sub("cfg/openvr");
    fixtures::write(t.path + "/cfg/openvr/openvrpaths.vrpath", "{\"config\" : [\"" + other + "\", \"" + steam + "\"]}");
    const auto d = steamConfigDirs(env);
    CHECK(d.size() == 2);
    CHECK(d[0] == other);
    CHECK(d[1] == steam); // listed once
    CHECK(roomsDir(env) == t.path + "/cfg/pibridge/rooms");
}

PIBRIDGE_TEST_MAIN()
