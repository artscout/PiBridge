// SPDX-License-Identifier: MIT
// The Room page's logic (no widgets): where the room files are for this app instance, what may be done now (open the
// in-headset room setup only while SteamVR runs; write the chaperone file only while it does not), the universe and
// room lists, the launch of pibridge-roomsetup through the Runner, and the made-up files of the fake mode. The editing
// itself is libpibridge-room's Editor, the same one pibridge-roomsetup runs in the headset.
#pragma once

#include <QString>
#include <QStringList>
#include <ctime>
#include <expected>
#include <functional>
#include <vector>

#include "pibridge/room/chap.h"
#include "pibridge/room/files.h"
#include "pibridge/room/rooms.h"
#include "runner.h"
#include "steamvr.h"

namespace pibridge::app {

/// The room files' environment: the process's HOME and XDG dirs; in fake mode everything (also HOME, which
/// ~/.steam/steam/config hangs off) inside the fake mode's temporary XDG_DATA_HOME.
pibridge::room::Env roomEnv(bool fake);

/// Made-up SteamVR files and rooms for the fake mode and the screenshots (a 4.2 x 3.4 m room with a cut corner in
/// universe 1111111111, a 3 x 2 m one in universe 30, two saved rooms).
void writeFakeRoomFiles(const pibridge::room::Env& env);

struct Gate {
    bool ok = false;
    QString why; // plain words when not ok
};
/// "Open room setup in headset": SteamVR must be running (the tool is an OpenXR app inside it) and the tool found.
Gate headsetGate(bool steamvrRunning, const QString& toolPath);
/// "Apply to SteamVR" (writing the chaperone file from the desktop): only while SteamVR is not running.
Gate applyGate(bool steamvrRunning, bool haveFile);

/// The universe SteamVR tracks in (the headset's Prop_CurrentUniverseId_Uint64), "" when unknown.
QString activeUniverse(const SteamVrStatus& s);

// ---------------------------------------------------------------- inside-out (SteamVR + Monado)

/// The inside-out universe (was driver_monado's "2" before pibridge-headtrack; ovrd_driver.cpp sets
/// Prop_CurrentUniverseId_Uint64 = 2), whatever room it is in.
inline constexpr char kInsideOutUniverse[] = "30"; // CustomHeadsetOpenVR's SLAM headset (pibridge-headtrack, the original Crystal)
/// The universe the page and the headset tool work on: the one SteamVR tracks in, else universe 30 in the inside-out
/// mode, else "" (the store's own pick).
QString roomTargetUniverse(bool insideOut, const QString& active);
/// The plain words the Room page shows in the inside-out mode (no persistent world anchor yet).
QString insideOutRoomNote();
/// Saving a room under `name` for `universe`: refused when a saved room of that name (the same file) belongs to another
/// universe, so a Lighthouse room is never overwritten by an inside-out one (or the other way round).
Gate saveRoomGate(const pibridge::room::RoomLib& lib, const QString& name, const QString& universe);
/// Showing a saved room of `roomUniverse` in the inside-out mode: only on its own universe (the page then switches to
/// it); a room whose universe is not in the chaperone file is not shown on universe 30.
Gate showRoomGate(bool insideOut, const QString& roomUniverse, bool roomUniverseInFile);
/// RECENTER IN HEADSET (inside-out): pibridge-roomsetup --recenter needs SteamVR + Monado running with the room anchor
/// on (PIMAX_ANCHOR=1, Monado's offset hook and head ring) and the tool.
Gate recenterGate(bool insideOut, bool steamvrRunning, bool anchorOn, const QString& toolPath);
/// The arguments of that start: --recenter --recenter-out OUT --ring RING.
QStringList recenterArgs(const QString& out, const QString& ring);
/// The line PiBridge shows at every inside-out start (the anchor is not persistent yet).
QString recenterReminder();

/// Adds universe 30 with a 2 x 2 m boundary around the origin to `file` (SteamVR's chaperone file), for the first
/// inside-out session before SteamVR made one. The file is backed up first (<file>.bak-<stamp>) and written
/// atomically; everything else in it stays. Refused when the universe is there already. Only while SteamVR is closed
/// (the caller's applyGate).
std::expected<void, QString> addInsideOutUniverse(const QString& file, std::time_t now);

struct UniverseRow {
    QString id;
    QString detail; // "5 corners · 13.9 m² · play area 3.4 x 2.6 m · Mon Oct 19 10:00:00 2026"
    bool active = false;
    bool pimax = false;
    bool monado = false; // universe 30: the inside-out headset's
};
std::vector<UniverseRow> universeRows(const pibridge::room::Store& st, const QString& active);

struct RoomRow {
    QString name;
    QString detail; // "universe 1111111111 · 5 corners · saved 19 Oct 2026, 14:12"
    bool active = false;
};
std::vector<RoomRow> roomRows(const pibridge::room::RoomLib& lib);

/// Start pibridge-roomsetup (detached) for `universe` ("" = the one SteamVR tracks in). False when it could not start.
bool launchRoomSetup(Runner& runner, const QString& toolPath, const QString& universe);

/// The arguments launchRoomSetup passes.
QStringList roomSetupArgs(const QString& universe);

} // namespace pibridge::app
