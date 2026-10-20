// SPDX-License-Identifier: MIT
// The Room page's logic (core/roomsetup.h). Every file lives in a temporary directory; nothing is started (FakeRunner).
#include <QTemporaryDir>

#include "core/roomsetup.h"
#include "pibridge/room/files.h"
#include "pibridge/room/geom.h"
#include "pibridge/room/json.h"
#include "pibridge_test.h"

using namespace pibridge::app;
namespace room = pibridge::room;

namespace {

room::Env tempEnv(const QTemporaryDir& d) {
    const std::string p = d.path().toStdString();
    return {p + "/home", p + "/config", p + "/data"};
}

} // namespace

TEST(gates) {
    CHECK(!headsetGate(false, QStringLiteral("/x/pibridge-roomsetup")).ok);
    CHECK(headsetGate(false, QStringLiteral("/x/pibridge-roomsetup")).why.contains(QStringLiteral("Start SteamVR")));
    CHECK(headsetGate(true, QStringLiteral("/x/pibridge-roomsetup")).ok);
    CHECK(!headsetGate(true, QString()).ok);
    CHECK(applyGate(false, true).ok);
    CHECK(!applyGate(true, true).ok);
    CHECK(applyGate(true, true).why.contains(QStringLiteral("writes it back")));
    CHECK(!applyGate(false, false).ok);
}

TEST(active_universe) {
    SteamVrStatus s;
    s.state = VrLinkState::Connected;
    s.devices = fakeVrDevices();
    CHECK(activeUniverse(s) == QStringLiteral("1111111111"));
    s.state = VrLinkState::NotRunning;
    CHECK(activeUniverse(s).isEmpty());
}

TEST(fake_files_and_rows) {
    QTemporaryDir d;
    const room::Env env = tempEnv(d);
    writeFakeRoomFiles(env);
    const auto dirs = room::steamConfigDirs(env);
    CHECK(dirs.size() == 1);
    auto st = room::Store::load(dirs, "1111111111");
    CHECK(st.has_value());
    CHECK(st->stagePts().size() == 5);
    const auto rows = universeRows(*st, QStringLiteral("1111111111"));
    CHECK(rows.size() == 2);
    CHECK(rows[0].id == QStringLiteral("1111111111") && rows[0].active);
    CHECK(rows[0].detail.contains(QStringLiteral("5 corners")));
    CHECK(rows[0].detail.contains(QStringLiteral("play area 3.4 x 2.6 m"))); // a point in every locale
    CHECK(!rows[1].active);
    const auto rr = roomRows(room::RoomLib(room::roomsDir(env)));
    CHECK(rr.size() == 2);
    CHECK(rr[0].name == QStringLiteral("Door closed"));
    CHECK(rr[1].active);
    CHECK(rr[1].detail.contains(QStringLiteral("universe 1111111111")));
}

TEST(fake_env_never_points_at_the_real_home) {
    qputenv("XDG_DATA_HOME", "/tmp/pibridge-fake-test/xdg_data_home");
    const room::Env env = roomEnv(true);
    CHECK(env.home == "/tmp/pibridge-fake-test/xdg_data_home/fake-home");
}

TEST(inside_out_universe_2_and_the_lighthouse_rooms) {
    // the universe: SteamVR's when known, else driver_monado's 2 in the inside-out mode
    CHECK(roomTargetUniverse(true, {}) == QStringLiteral("2"));
    CHECK(roomTargetUniverse(true, QStringLiteral("2")) == QStringLiteral("2"));
    CHECK(roomTargetUniverse(false, {}).isEmpty());
    CHECK(roomTargetUniverse(false, QStringLiteral("1111111111")) == QStringLiteral("1111111111"));
    CHECK(insideOutRoomNote().contains(QStringLiteral("map the room once")));
    QTemporaryDir d;
    const room::Env env = tempEnv(d);
    writeFakeRoomFiles(env);
    room::RoomLib lib(room::roomsDir(env));
    // a Lighthouse room is never overwritten by an inside-out one of the same name, nor the other way round
    const Gate g = saveRoomGate(lib, QStringLiteral("Door open"), QStringLiteral("2"));
    CHECK(!g.ok && g.why.contains(QStringLiteral("universe 1111111111 (Lighthouse)")));
    CHECK(g.why.contains(QStringLiteral("\"Door open inside-out\"")));
    CHECK(saveRoomGate(lib, QStringLiteral("door OPEN"), QStringLiteral("2")).ok == false); // the same file
    CHECK(saveRoomGate(lib, QStringLiteral("Door open"), QStringLiteral("1111111111")).ok); // updating it
    CHECK(saveRoomGate(lib, QStringLiteral("Living room"), QStringLiteral("2")).ok);
    room::Room io;
    io.name = "Living room";
    io.universe = "2";
    io.points = room::rectangle(2, 2);
    CHECK(lib.save(io, 1792404000).has_value());
    const Gate back = saveRoomGate(lib, QStringLiteral("Living room"), QStringLiteral("1111111111"));
    CHECK(!back.ok && back.why.contains(QStringLiteral("universe 2 (inside-out)")));
    // showing a room: in the inside-out mode never a foreign room on universe 2
    CHECK(showRoomGate(true, QStringLiteral("2"), true).ok);
    CHECK(showRoomGate(true, QStringLiteral("1111111111"), true).ok);  // shown on its own universe
    CHECK(!showRoomGate(true, QStringLiteral("3333"), false).ok);      // its universe is not in the file
    CHECK(showRoomGate(false, QStringLiteral("3333"), false).ok);      // Lighthouse mode: as before
    // universe 2 added to a file that has none (backup first, the rest untouched)
    const std::string file = room::steamConfigDirs(env).front() + "/chaperone_info.vrchap";
    auto doc = room::parseJson(*room::readFile(file));
    room::Json& unis = (*doc)["universes"];
    unis.items().erase(unis.items().begin() + 1); // drop the fake universe 2
    CHECK(room::writeFileAtomic(file, doc->dump() + "\n").has_value());
    const std::string before = *room::readFile(file);
    CHECK(addMonadoUniverse(QString::fromStdString(file), 1792404000).has_value());
    auto st = room::Store::load(room::steamConfigDirs(env), "2");
    CHECK(st.has_value() && st->universe() == "2" && st->stagePts().size() == 4);
    CHECK(st->universes().contains("1111111111") && st->universes().at("1111111111").points == 5);
    CHECK(*room::readFile(file + ".bak-" + room::backupStamp(1792404000)) == before);
    CHECK(!addMonadoUniverse(QString::fromStdString(file), 1792404001).has_value()); // there already
    const auto rows = universeRows(*st, QStringLiteral("2"));
    CHECK(rows.size() == 2 && rows[1].monado && rows[1].active && !rows[0].monado);
}

TEST(recenter_gate_and_arguments) {
    const QString tool = QStringLiteral("/x/pibridge-roomsetup");
    CHECK(!recenterGate(false, true, true, tool).ok);
    CHECK(recenterGate(true, true, false, tool).why.contains(QStringLiteral("Turn Room anchor on")));
    CHECK(recenterGate(true, false, true, tool).why.contains(QStringLiteral("Start tracking first")));
    CHECK(!recenterGate(true, true, true, QString()).ok);
    CHECK(recenterGate(true, true, true, tool).ok);
    CHECK((recenterArgs(QStringLiteral("/s/recenter.txt"), QStringLiteral("/r/pimax-anchor.head")) ==
           QStringList{QStringLiteral("--recenter"), QStringLiteral("--recenter-out"), QStringLiteral("/s/recenter.txt"),
                       QStringLiteral("--ring"), QStringLiteral("/r/pimax-anchor.head")}));
    CHECK(recenterReminder().contains(QStringLiteral("after every start")) && recenterReminder().contains(QStringLiteral("1.5 s")));
}

TEST(launch) {
    FakeRunner r;
    CHECK(launchRoomSetup(r, QStringLiteral("/fake/pibridge-roomsetup"), QStringLiteral("1111111111")));
    CHECK(r.calls.size() == 1);
    CHECK(r.calls[0].detached);
    CHECK(r.calls[0].line() == QStringLiteral("/fake/pibridge-roomsetup --universe 1111111111"));
    CHECK(roomSetupArgs(QString()).isEmpty());
}

PIBRIDGE_TEST_MAIN()
