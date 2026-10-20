// SPDX-License-Identifier: MIT
// Ported from ZoneTweak profiles.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// Saved rooms ("named spaces"): one JSON file per room in $XDG_CONFIG_HOME/pibridge/rooms/<slug>.json, the active
// room's name in active.txt and the frame mode in align.txt, as ZoneTweak's "spaces" folder. A room holds the file
// numbers of a universe (frame "raw": exactly what goes into the vrchap), so applying it writes it back unchanged:
//   {"name", "universe", "height", "points": [[x, z], ...], "frame": "raw", "saved", "created",
//    "standing": {"translation": [x, y, z], "yaw"}, "play_area": [w, d], "warning_distance"}
// The last four are PiBridge's additions; ZoneTweak's files (without them) load too. Unknown keys are ignored.
#pragma once

#include "pibridge/room/chap.h"
#include "pibridge/room/geom.h"

#include <expected>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::room {

struct Room {
    std::string name;
    std::string universe;
    double height = kDefaultHeight;
    Poly points;                 // file numbers of the universe ("raw"), or stage numbers when frame == "stage"
    std::string frame = "raw";
    std::string saved;           // RFC 3339 local time of the last save
    std::string created;
    std::optional<Standing> standing; // origin: translation (y = floor) and yaw, file numbers
    std::optional<Size2> playArea;
    std::optional<double> warningDistance;

    /// floor offset = standing translation y (0 without a standing pose)
    double floorOffset() const { return standing ? standing->y : 0.0; }
    UniverseData toUniverse() const;
    static Room fromUniverse(const std::string& name, const std::string& universe, const UniverseData& d);
};

std::string roomToJson(const Room& r);
std::expected<Room, std::string> roomFromJson(std::string_view text);
/// RFC 3339 in local time with the offset ("2026-10-19T14:12:05+02:00").
std::string rfc3339(std::time_t t);

class RoomLib {
public:
    explicit RoomLib(std::string dir) : dir_(std::move(dir)) {}
    const std::string& dir() const { return dir_; }
    /// The file for a name: lower case letters and digits, space/-/_ become '-', the rest dropped ("space" if empty).
    std::string fileFor(const std::string& name) const;
    /// Every valid room (a name and at least 3 points), sorted by name without case.
    std::vector<Room> list() const;
    std::optional<Room> find(const std::string& name) const;
    /// Save (replacing a room of the same file name). Keeps "created" of the room it replaces.
    std::expected<void, std::string> save(Room r, std::time_t now);
    std::expected<void, std::string> remove(const std::string& name);
    /// Rename: the new file is written first, then the old one removed.
    std::expected<void, std::string> rename(const std::string& from, const std::string& to, std::time_t now);
    std::string active() const;
    void setActive(const std::string& name) const;
    FrameMode frameMode(FrameMode fallback) const;
    void setFrameMode(FrameMode m) const;

private:
    std::string dir_;
};

} // namespace pibridge::room
