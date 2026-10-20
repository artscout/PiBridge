// SPDX-License-Identifier: MIT
// The editor driven by scripted controller frames (ZoneTweak's handleHand and the menu), on synthetic files.
#include "editor_util.h"
#include "pibridge/room/files.h"
#include "pibridge_test.h"

using namespace pibridge::room;
using edtest::bits;
using edtest::Rig;

TEST(grab_and_move_a_corner_then_undo) {
    Rig r;
    CHECK(r.ed->pts.size() == 4);
    const Pt2 c0 = r.ed->pts[1]; // (2, -1.5)
    r.frame({c0.x - 0.05, 1.0, c0.z + 0.05});
    CHECK(r.ed->hands[1].hoverV == 1);
    r.frame({c0.x - 0.05, 1.0, c0.z + 0.05}, bits({bit::Trigger}));
    CHECK(r.ed->hands[1].grab == 1);
    r.frame({c0.x + 0.45, 1.0, c0.z + 0.05}, bits({bit::Trigger}));
    CHECK_NEAR(r.ed->pts[1].x, c0.x + 0.5, 1e-9);
    r.frame({c0.x + 0.45, 1.0, c0.z + 0.05});
    CHECK(r.ed->hands[1].grab == -1);
    CHECK(r.ed->dirty);
    CHECK(r.ed->changed());
    r.click({0, 1.0, 0}, bit::Stick); // undo
    CHECK(r.ed->pts[1] == c0);
    CHECK(r.haptics > 0);
}

TEST(insert_on_a_wall_and_delete) {
    Rig r;
    r.frame({0, 1.0, -1.45});
    CHECK(r.ed->hands[1].hoverE == 0);
    r.frame({0, 1.0, -1.45}, bits({bit::Trigger}));
    CHECK(r.ed->pts.size() == 5);
    CHECK_NEAR(r.ed->pts[1].z, -1.5, 1e-9);
    r.frame({0, 1.0, -1.2}, bits({bit::Trigger}));
    CHECK_NEAR(r.ed->pts[1].z, -1.25, 1e-9); // dragged inwards
    r.frame({0, 1.0, -1.2});
    r.click({0, 1.0, -1.2}, bit::Grip); // A deletes the corner under the controller
    CHECK(r.ed->pts.size() == 4);
    // never below 3 points
    r.click({2, 1.0, -1.5}, bit::Grip);
    CHECK(r.ed->pts.size() == 3);
    r.click({2, 1.0, 1.5}, bit::Grip);
    CHECK(r.ed->pts.size() == 3);
}

TEST(menu_save_writes_the_file) {
    Rig r;
    r.frame({r.ed->pts[1].x, 1.0, r.ed->pts[1].z}, bits({bit::Trigger}));
    r.frame({2.6, 1.0, -1.5}, bits({bit::Trigger}));
    r.frame({2.6, 1.0, -1.5});
    r.click({0.3, 1.0, 0.3}, bit::Menu);
    CHECK(r.ed->menuOpen);
    CHECK(r.ed->cur.size() > 10);
    r.pressMenu("SAVE");
    CHECK(r.reloads == 1);
    CHECK(!r.ed->dirty);
    CHECK(r.ed->status().starts_with("SAVED TO 1 FILE"));
    auto doc = readJsonFile(r.chapFile());
    const auto [pts, h] = boundsOf(*findUniverse(*doc, "1111111111"));
    CHECK_NEAR(pts[1].x, 2.6, 1e-6);
    CHECK(backupsOf(r.chapFile()).size() == 1);
}

TEST(play_area_unlock_move_and_save) {
    Rig r;
    const Pt2 lock = r.ed->lockPos();
    r.click({lock.x, 1.0, lock.z}, bit::Trigger);
    CHECK(r.ed->areaMode);
    // grab the centre post and move it 0.5 m right
    const Rigid2 c = r.ed->centerPose();
    r.frame({c.tx, 1.0, c.tz});
    CHECK(r.ed->hands[1].areaHover == 1);
    r.frame({c.tx, 1.0, c.tz}, bits({bit::Trigger}));
    r.frame({c.tx + 0.5, 1.0, c.tz}, bits({bit::Trigger}));
    r.frame({c.tx + 0.5, 1.0, c.tz});
    CHECK(r.ed->centerDirty);
    CHECK_NEAR(r.ed->centerPose().tx, c.tx + 0.5, 1e-9);
    // stretch: the +x edge handle out by 0.25 m
    const double w0 = r.ed->area.w;
    const Pt2 e2 = r.ed->centerPose().apply({w0 / 2, 0});
    r.frame({e2.x, 1.0, e2.z});
    r.frame({e2.x, 1.0, e2.z}, bits({bit::Trigger}));
    r.frame({e2.x + 0.25, 1.0, e2.z}, bits({bit::Trigger}));
    r.frame({e2.x + 0.25, 1.0, e2.z});
    CHECK_NEAR(r.ed->area.w, w0 + 0.25, 1e-6);
    const Poly before = r.ed->pts;
    CHECK(r.ed->save());
    auto doc = readJsonFile(r.chapFile());
    const UniverseData d = universeData(*findUniverse(*doc, "1111111111"));
    CHECK_NEAR(d.playArea.w, w0 + 0.25, 1e-5);
    // the new standing origin is where the centre was put (Off: standing coordinates of the old standing pose)
    const Standing s0{0.25, 0.01, -0.5, 0.3};
    const Pt2 expect = rigidFromYaw(-s0.yaw, s0.x, s0.z).apply({c.tx + 0.5 + 0.125, c.tz});
    CHECK_NEAR(d.standing.x, expect.x, 1e-5);
    CHECK_NEAR(d.standing.z, expect.z, 1e-5);
    // the walls did not move in the room
    for (std::size_t i = 0; i < before.size(); i++) {
        CHECK_NEAR(r.ed->pts[i].x, before[i].x, 1e-9);
    }
}

TEST(fit_and_center) {
    Rig r;
    r.ed->fitArea();
    CHECK(r.ed->areaDirty);
    CHECK_NEAR(r.ed->area.w, 3.8, 0.02); // 4 x 3 room minus 10 cm on each side
    CHECK_NEAR(r.ed->area.h, 2.8, 0.02);
    r.ed->center = rigidFromYaw(0.2, 0.7, 0.3);
    r.ed->centerInRoom();
    CHECK_NEAR(r.ed->centerPose().tx, 0, 1e-9);
    r.ed->squareToRoom();
    CHECK_NEAR(std::remainder(r.ed->centerPose().angle(), std::numbers::pi / 2), 0, 1e-9);
    r.ed->resetArea();
    CHECK(!r.ed->centerDirty && !r.ed->areaDirty);
}

TEST(new_outline_by_corners) {
    Rig r;
    r.pressMenu("NEW OUTLINE");
    CHECK(r.ed->drawing && !r.ed->menuOpen);
    for (const Pt2 p : Poly{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
        r.click({p.x, 1.0, p.z}, bit::Trigger);
    }
    CHECK(r.ed->draft.size() == 4);
    r.click({-0.95, 1.0, -0.95}, bit::Trigger); // near the first corner: closes
    CHECK(!r.ed->drawing);
    CHECK(r.ed->pts.size() == 4);
    CHECK_NEAR(std::fabs(polyArea(r.ed->pts)), 4, 1e-9);
}

TEST(trace_outline_by_walking) {
    Rig r;
    r.pressMenu("TRACE OUTLINE");
    CHECK(r.ed->tracing);
    FakePoseSource walk(rectangle(3, 2.5), 1.0, 0.2);
    for (double t = 0; t < walk.lapSeconds() + 1.0; t += 1.0 / 90) {
        const auto s = *walk.sample(t);
        r.frame(s.hands[1].pos, s.trigger[1] ? bits({bit::Trigger}) : 0);
    }
    CHECK(!r.ed->tracing);
    CHECK(r.ed->pts.size() == 4);
    CHECK_NEAR(std::fabs(polyArea(r.ed->pts)), 7.5, 0.4);
    CHECK(r.ed->status().starts_with("TRACED OUTLINE"));
}

TEST(saved_spaces_and_restore) {
    Rig r;
    CHECK(r.ed->saveAsNew("Door open").has_value());
    CHECK(r.ed->lib.active() == "Door open");
    const auto room = r.ed->lib.find("door open");
    CHECK(room.has_value() && room->points.size() == 4 && room->standing.has_value());
    // change, save, then use the saved space again: the boundary comes back
    r.ed->setPts(rectangle(1, 1), "");
    CHECK(r.ed->save());
    r.ed->useSpace(*room);
    CHECK_NEAR(std::fabs(polyArea(r.ed->pts)), 12, 1e-6);
    auto doc = readJsonFile(r.chapFile());
    CHECK_NEAR(std::fabs(polyArea(boundsOf(*findUniverse(*doc, "1111111111")).first)), 12, 1e-6);
    // restore original: the oldest backup's boundary, not yet saved
    r.ed->setPts(rectangle(1, 1), "");
    r.ed->restoreOriginal();
    CHECK_NEAR(std::fabs(polyArea(r.ed->pts)), 12, 1e-6);
    CHECK(r.ed->dirty);
}

TEST(realign_and_stage_moves_keep_the_room) {
    Rig r;
    const Poly before = r.ed->pts;
    const Rigid2 oldFromNew = rigidFromYaw(0.5, 1.0, -0.5);
    r.ed->stageMoved(oldFromNew);
    for (std::size_t i = 0; i < before.size(); i++) {
        const Pt2 back = oldFromNew.apply(r.ed->pts[i]);
        CHECK_NEAR(back.x, before[i].x, 1e-9);
        CHECK_NEAR(back.z, before[i].z, 1e-9);
    }
    // saving after the move writes the same file numbers as before
    CHECK(r.ed->save());
    auto s2 = Store::load({r.dir.path + "/config"}, "1111111111");
    for (std::size_t i = 0; i < before.size(); i++) {
        CHECK_NEAR(s2->stagePts()[i].x, before[i].x, 1e-5);
    }
}

TEST(warning_line_drag_writes_the_setting) {
    Rig r;
    r.pressMenu("ZONE SETTINGS");
    r.pressMenu("PROXIMITY TEST");
    CHECK(r.ed->proxTest);
    r.click({0.3, 1.0, 0.3}, bit::Menu); // close the menu
    CHECK(!r.ed->menuOpen);
    // the line sits 0.7 m (SteamVR's default) inside the -z wall at z = -1.5
    r.frame({0, 1.0, -0.8});
    r.frame({0, 1.0, -0.8}, bits({bit::Trigger}));
    CHECK(r.ed->hands[1].lineGrab);
    r.frame({0, 1.0, -1.0}, bits({bit::Trigger}));
    CHECK_NEAR(r.ed->currentWarnLine(), 0.5, 1e-9);
    r.frame({0, 1.0, -1.0});
    CHECK_NEAR(r.zone->values().fade.value_or(0), 0.5, 1e-9);
    // the head inside the warning distance raises the level
    r.head = {0, 1.7, -1.3};
    r.frame({0, 1.0, 0});
    CHECK(r.ed->proximityLevel() > 0.5);
}

TEST(zone_page_nudges_and_switch) {
    Rig r;
    r.pressMenu("ZONE SETTINGS");
    r.pressMenu("FARTHER +10");
    CHECK_NEAR(r.zone->values().fade.value_or(0), 0.8, 1e-9);
    r.pressMenu("SAFETY ZONE");
    CHECK(!r.zone->values().zoneOn());
    r.pressMenu("SAFETY ZONE");
    CHECK(r.zone->values().zoneOn());
    r.pressMenu("BACK");
    CHECK(r.ed->page == &r.ed->pageMain);
}

TEST(laser_pointer_grabs_from_afar) {
    Rig r;
    r.click({0.3, 1.0, 0.3}, bit::Tool);
    CHECK(r.ed->laser);
    // aim from the middle at corner 1 (2, -1.5): the aim's -z must point at it, level
    const Vec3 o{0, 1.0, 0};
    const Vec3 target{2, 1.0, -1.5};
    const Vec3 f = (target - o).norm();
    Mat34 aim = edtest::at(o);
    const Vec3 zAxis = f * -1;
    const Vec3 xAxis = Vec3{0, 1, 0}.cross(zAxis).norm();
    const Vec3 yAxis = zAxis.cross(xAxis);
    aim.m[0] = {xAxis.x, yAxis.x, zAxis.x, o.x};
    aim.m[1] = {xAxis.y, yAxis.y, zAxis.y, o.y};
    aim.m[2] = {xAxis.z, yAxis.z, zAxis.z, o.z};
    FrameInput in;
    in.headValid = true;
    in.head = edtest::at(r.head);
    in.hands[0] = {true, edtest::at({-0.4, 1, 0.4}), true, edtest::at({-0.4, 1, 0.4}), 0};
    in.hands[1] = {true, aim, true, aim, 0};
    r.ed->frame(in);
    CHECK(r.ed->hands[1].hoverV == 1);
    in.hands[1].buttons = bits({bit::Trigger});
    r.ed->frame(in);
    CHECK(r.ed->hands[1].rayGrab);
}

TEST(tips) {
    CHECK(tipFor("SAVE").starts_with("Writes the boundary"));
    CHECK(tipFor("SAVE AS NEW").starts_with("Name and store"));
    CHECK(tipFor("DOOR (4)").starts_with("Loads this saved space"));
    CHECK(tipFor("XYZ").empty());
}

PIBRIDGE_TEST_MAIN()
