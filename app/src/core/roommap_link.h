// SPDX-License-Identifier: MIT
// The persistent room map in the app (docs/ROOM-SETUP.md "Room map (inside-out)"): the C++ room-map tool
// pibridge-roommap (tools/roommap, a port of omarchyvr-roommap) does the work; the app only starts it and reads its files.
//
//   Map the room   Room page, inside-out: `pibridge-roommap setup fast --name <room>` against the running session (it
//                  reads Monado's head ring and the camera broker): stand in the middle, look around ~30 s, hold still
//                  looking forward. It saves the map, the floor and the room setup, and writes the world offset through
//                  the writer lease (floor + centre + forward), so the floor is at y = 0 and forward is -Z.
//   At every start both inside-out sessions (core/io_session.h) pass PIMAX_ANCHOR=1 (core/inside_out.h) and, when a
//                  saved room exists, start `pibridge-roommap auto --name <room> --wait 120` detached once the runtime is
//                  up; it waits for the head ring, relocalises in 15 s windows for up to 120 s, applies, and exits. Stop
//                  ends it by PID with the session.
//   Status         pibridge-roommap writes $XDG_RUNTIME_DIR/omarchyvr-room.status (state, room, pid = the Monado session,
//                  text, when); the Room and Tracking pages show it as "Room recognised / Searching / Room not found".
#pragma once

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <optional>

#include "conf.h"

namespace pibridge::app {

namespace roomkey {
inline constexpr char kRoom[] = "io_room"; // the saved room's name (a folder under the room-map dir); default "home"
inline constexpr char kBodyHeight[] = "room_body_height_m"; // the user's body height (m); empty = not entered
inline constexpr char kEyeHeight[] = "room_eye_height_m";   // the measured eye height (m); empty = from the body height
} // namespace roomkey

inline constexpr char kDefaultRoom[] = "home";
inline constexpr int kRoomAutoWaitS = 120; // pibridge-roommap auto --wait (omarchyvr's default)

// The room's name from pibridge.conf (io_room); "home" when unset or not a plain folder name.
QString roomName(const Conf& conf);
// Where the room maps are: PIBRIDGE_ROOMMAP_DIR, PIMAX_ROOMMAP_DIR, else ~/.local/share/pibridge/roommap, or omarchyvr's
// ~/.local/share/omarchyvr/roommap when only that exists (the same rule as libpibridge-roommap store::base_dir).
QString roommapBaseDir();
// A saved room: <base>/<name>/map.bin exists.
bool roomSaved(const QString& baseDir, const QString& name);

// SETUP PLAYSPACE (Room page, inside-out): `pibridge-roommap setup fast --name <room> --no-hud` against the running
// session (it reads Monado's head ring and the camera broker). When an eye height is given it is passed as
// --eye-height, so the floor comes from the head (the first floor method) instead of the point cloud.
QStringList roomSetupFastArgs(const QString& name, std::optional<double> eyeHeight = std::nullopt);
// MAP THE ROOM in the headset (docs/ROOM-SETUP.md "Map the room in the headset"): pibridge-roomsetup --map shows the room
// through the headset cameras with the steps over it and runs `pibridge-roommap setup fast --guide` itself.
// `chaperone` (SteamVR path): pibridge-roomsetup --map --chaperone -> pibridge-roommap saves the setup with --no-offset and
// pibridge-roomsetup writes the floor/centre/forward into SteamVR's chaperone instead of a Monado world offset.
// `quickDirect` starts straight in the quick circle setup (posture -> size -> finish), skipping the in-VR Full/Quick
// mode menu (the Dashboard's QUICK ROOM SETUP button); false keeps the mode menu (the plain ROOM SETUP button).
QStringList roomMapHeadsetArgs(const QString& name, const QString& roommapTool, const QString& ring,
                               std::optional<double> eyeHeight = std::nullopt, bool chaperone = false,
                               bool quickDirect = false);

// The floor from the user's height (user decision 2026-10-19: the default floor method; the controllers are tracked
// poorly near the floor and the map takes tables and beds for the floor). The eye height standing upright: the
// measured one (room_eye_height_m, 1.0..2.3 m) when entered, else the body height (room_body_height_m, 1.2..2.3 m)
// x 0.936 (the anthropometric average: 1.78 -> 1.67 m); nullopt when neither is entered (then the floor step asks
// for the grid or a touch as before).
inline constexpr double kEyeFromBody = 0.936;
std::optional<double> parseHeightM(const QString& text, double lo, double hi);
std::optional<double> derivedEyeHeight(const QString& bodyText);
std::optional<double> eyeHeightOf(const QString& bodyText, const QString& eyeText);
std::optional<double> eyeHeightOf(const Conf& conf);
// SET FLOOR FROM MY HEIGHT (Room page): pibridge-roomsetup --floor-height in the running session.
QStringList floorHeightArgs(const QString& name, double eyeHeight, const QString& roommapTool, const QString& ring);
QStringList roomAutoArgs(const QString& name);

// $XDG_RUNTIME_DIR/omarchyvr-room.status (the file pibridge-roommap setup / auto write).
QString roomStatusPath();
struct RoomStatus {
    QString state; // ok | searching | failed | skipped | off
    QString room;
    quint32 pid = 0; // the Monado session (the ring's writer); 0 = written before the session was found
    QString text;
    qint64 when = 0; // unix seconds
};
std::optional<RoomStatus> parseRoomStatus(const QByteArray& text);

enum class RoomLook { NoRoom, Saved, Searching, Recognised, NotFound, Other };
struct RoomWords {
    RoomLook look = RoomLook::NoRoom;
    QString headline; // "Room recognised", "Searching for the room", "Room not found", ...
    QString detail;   // the tool's own sentence, or what happens next
};
// What to show. `status`: the file (nullopt: none); it counts when it names `sessionPid` (the running session, 0 =
// none) or was written at or after `sinceUnix` (0 = never). `autoRunning`: pibridge-roommap auto of this session runs.
RoomWords roomStatusWords(const std::optional<RoomStatus>& status, quint32 sessionPid, qint64 sinceUnix, bool saved,
                          bool autoRunning, const QString& name);

// The live inside-out session as the room map sees it: Monado's head ring (core/anchor_offset.h) with a running writer
// and a fresh sample.
struct RingFeed {
    quint32 pid = 0;     // the ring's writer (0: no ring)
    bool alive = false;  // the writer runs
    double ageS = -1;    // age of the newest valid sample (-1: none)
};
// Reads AnchorFiles::defaults().ring and CLOCK_MONOTONIC (Monado's clock).
RingFeed readRingFeed();

struct RoomGate {
    bool ok = false;
    QString why;
};
// "Map the room": the inside-out mode, the tool found, a live session with the room anchor (a fresh head ring), and
// no run of it already.
// A warning when the saved room's floor (floor.toml head_above_m) is too close below the head for a standing user:
// live 2026-10-19 the fast setup took a table 0.76 m below the head for the floor. Empty when fine or unknown.
QString floorWarning(const QByteArray& floorToml);
inline constexpr double kFloorMinBelowHeadM = 1.0;

RoomGate mapRoomGate(bool insideOut, const QString& tool, const RingFeed& feed, bool running);
// The same for the run in the headset: also pibridge-roomsetup (it shows the steps in the headset).
// SET FLOOR FROM MY HEIGHT: the inside-out mode, both tools, an eye height, a live session in which the room was
// recognised (the status file of this session says ok), nothing else running.
RoomGate floorHeightGate(bool insideOut, const QString& roommapTool, const QString& roomsetupTool, const RingFeed& feed,
                         std::optional<double> eyeHeight, bool recognised, bool running);
RoomGate mapRoomHeadsetGate(bool insideOut, const QString& roommapTool, const QString& roomsetupTool, const RingFeed& feed,
                            bool running);

} // namespace pibridge::app
