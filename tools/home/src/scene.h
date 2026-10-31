// SPDX-License-Identifier: MIT
// What pibridge-home shows in the headset while the Tracking session runs with apps through Monado directly, as
// libpibridge-room scene lists (unlit coloured lines and triangles in stage space, pibridge/room/draw.h): a floor grid that
// fades out with distance, the horizon as a ring at eye height, the active saved room's boundary when there is one,
// the controllers as small markers, and a text panel in front ("PIBRIDGE - TRACKING ON / START AN APP FROM PIBRIDGE.").
// Pure: no runtime, no GPU (the self test and the tests build and rasterize it).
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/geom.h"
#include "pibridge/room/rooms.h"
#include "pibridge/room/vmath.h"

#include <array>
#include <optional>
#include <string>

namespace pibridge::home {

/// Where things are in the space the session draws in.
struct World {
    bool stage = true;                // STAGE space (floor at y = 0); false: LOCAL (no floor known)
    double floorY = 0.0;              // 0 in STAGE; kLocalFloorY in LOCAL
    std::optional<room::Poly> room;   // the active room's boundary in stage numbers (floor points x, z)
    double roomHeight = room::kDefaultHeight;
    std::string roomName;
    bool grid = true; // the floor grid and the horizon (the dark home); off over a picture or the passthrough
};
inline constexpr double kLocalFloorY = -1.6;  // LOCAL's origin is the head at the start: the floor about 1.6 m below
inline constexpr double kEyeHeight = 1.6;     // the horizon ring's height above the floor
inline constexpr double kGridRadius = 12.0;   // metres; the grid fades out towards it
inline constexpr double kHorizonRadius = 50.0;
inline constexpr double kFarZ = 120.0;        // the far plane the renderer uses (beyond the horizon ring)

/// Whether a controller is drawn (its marker and laser, and whether it points): while the runtime tracks its position,
/// and kLostGraceS after that (a short occlusion does not blink). Never tracked yet (the Pimax controllers before the
/// headset first sees them sit at a guessed rest pose), no pose (switched off), or lost longer: not drawn.
struct HandShown {
    static constexpr double kLostGraceS = 1.0;
    double lastTrackedS = -1.0; // < 0: never tracked
    bool update(bool located, bool tracked, double nowS) {
        if (located && tracked) {
            lastTrackedS = nowS;
        }
        return located && lastTrackedS >= 0.0 && nowS - lastTrackedS <= kLostGraceS;
    }
};

/// The text panel: placed once in front of the first valid head pose (yaw only), then fixed in the room.
struct Panel {
    room::Vec3 center;
    room::Vec3 right; // along the text
    room::Vec3 up;
};
Panel placePanel(const room::Mat34& head, double floorY);

struct Frame {
    std::optional<room::Mat34> head;
    std::array<std::optional<room::Mat34>, 2> grip; // left, right
    std::array<std::optional<room::Mat34>, 2> aim;
    std::optional<Panel> panel;
};

/// The parts (each appends to `b` and returns the number of lines it added).
std::size_t addGrid(room::Batch& b, const World& w);
std::size_t addHorizon(room::Batch& b, const World& w);
std::size_t addRoom(room::Batch& b, const World& w);
std::size_t addControllers(room::Batch& b, const Frame& f);
std::size_t addPanel(room::Batch& b, const Panel& p);
/// The whole scene (cleared first).
void buildScene(room::Batch& b, const World& w, const Frame& f);

/// The panel's lines of text.
inline constexpr const char* kPanelTitle = "PIBRIDGE - TRACKING ON";
inline constexpr const char* kPanelHint = "START AN APP FROM PIBRIDGE.";

/// The boundary of a saved room in stage numbers: frame "stage" as is; frame "raw" through the room's standing pose
/// in the frame mode (A: stage = rotate(yaw) of (p - standing), B: the same with -yaw, Off: as is). nullopt with fewer
/// than three points.
std::optional<room::Poly> roomOutline(const room::Room& r, room::FrameMode mode);
/// The active saved room of `roomsDir` (RoomLib active(), its frame mode), or an empty World::room.
World worldFor(bool stage, const std::string& roomsDir);

} // namespace pibridge::home
