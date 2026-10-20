// SPDX-License-Identifier: MIT
// Ported from ZoneTweak chap.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// SteamVR's chaperone files (chaperone_info*.vrchap in the SteamVR config folders). JSON: "jsonid", "version",
// "universes": [{ "universeID" (a string), "play_area": [w, d], "collision_bounds": one quad per wall, each 4 points
// [x, y, z] (floor, top, top, floor; walls from the floor to ~2.43 m), "standing" / "seated": {"translation":
// [x, y, z], "yaw"}, "time" }]. Everything else (other universes, unknown fields, "seated") is written back as it was.
//
// The Store is ZoneTweak's: all files are read, each universe is listed with the files that hold it, one universe is
// edited, a save writes it into every file that has it, with a timestamped .bak-YYYYMMDD-HHMMSS copy of each file
// before the first write of the session, through a temporary file and a rename.
//
// Frames. The editor works in stage space (the OpenXR STAGE space = SteamVR's standing space). How the file's points
// relate to it is the frame mode, as in ZoneTweak:
//   A    the file holds raw tracking coordinates; stage = inverse(standing pose) applied (ZoneTweak's default, right
//        for Pimax's own universes),
//   B    the same with the yaw negated,
//   Off  the file holds standing coordinates already (what SteamVR's own Lighthouse universes look like on Linux:
//        the standing origin is inside the boundary only in this reading).
// PiBridge picks A for a universe Pimax marked (center_calied / floor_calied) and Off otherwise; the editor can switch.
// With Off, moving the play centre re-expresses the points in the new standing frame so the walls stay where they
// are in the room (ZoneTweak's Off kept the numbers, which moves the walls with the centre).
#pragma once

#include "pibridge/room/geom.h"
#include "pibridge/room/json.h"

#include <ctime>
#include <expected>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace pibridge::room {

inline constexpr double kDefaultHeight = 2.43;

enum class FrameMode { A = 0, B = 1, Off = 2 };
inline constexpr int kFrameModes = 3;
const char* frameModeName(FrameMode m);

/// The standing (or seated) pose: translation and yaw, as in the file.
struct Standing {
    double x = 0, y = 0, z = 0, yaw = 0;
    friend bool operator==(const Standing&, const Standing&) = default;
};

/// One universe entry, in the file's own numbers.
struct UniverseData {
    Poly bounds;   // the floor corners of the walls (the first point of each quad)
    double height = kDefaultHeight;
    Standing standing;
    Size2 playArea{1, 1};
    bool pimax = false;
    std::string time;
};

// ---------------------------------------------------------------- file level

std::vector<Json*> universesOf(Json& doc);
std::vector<const Json*> universesOf(const Json& doc);
std::string universeId(const Json& u);
bool isPimax(const Json& u);
/// The floor points and the wall height (2.43 when the quads say nothing).
std::pair<Poly, double> boundsOf(const Json& u);
UniverseData universeData(const Json& u);
const Json* findUniverse(const Json& doc, const std::string& id);
Json* findUniverse(Json& doc, const std::string& id);
/// collision_bounds for `p`: one quad per edge, floor-top-top-floor, numbers rounded to 1e-6 (as ZoneTweak).
Json makeBounds(const Poly& p, double h);
std::expected<Json, std::string> readJsonFile(const std::string& path);

// ---------------------------------------------------------------- the store

struct UniverseInfo {
    std::string id;
    bool pimax = false;
    int points = 0;
    std::vector<std::string> files;
    std::time_t newest = 0; // mtime of `source`
    std::string source;     // the file it is read from
    std::string time;       // the entry's "time" field as written by SteamVR
};

/// What to write besides the boundary.
struct SaveExtras {
    std::optional<Standing> standing;  // new standing x, z, yaw (y is changed through floorDelta only)
    std::optional<Size2> playArea;
    double floorDelta = 0;             // added to standing translation y
};

struct SaveResult {
    std::vector<std::string> written;
    std::vector<std::string> skipped;
};

class Store {
public:
    /// Read every chaperone_info*.vrchap in `dirs` and pick the universe to edit: `prefer` when given and present,
    /// else a Pimax universe (the one with most points), else the newest by its "time"/file, else most points.
    static std::expected<Store, std::string> load(const std::vector<std::string>& dirs, const std::string& prefer = {});

    std::expected<void, std::string> selectUniverse(const std::string& id);
    std::string nextUniverse() const;

    // the frames
    Pt2 toStageMode(FrameMode m, Pt2 p) const;
    Pt2 toFileMode(FrameMode m, Pt2 q) const;
    Rigid2 adjust() const { return adj_; }
    Poly toStageAll(const Poly& p) const;
    Poly toFileAll(const Poly& p) const;
    /// standing coordinates -> raw (rotation about +y by yaw, then the translation).
    Rigid2 standingRigid() const { return rigidFromYaw(-yaw_, tx_, tz_); }
    void setAdj(const Rigid2& r);
    void moveStage(const Rigid2& r);
    /// Switch the frame mode; returns `pts` (stage points of the old mode) re-expressed in the new one.
    Poly setMode(FrameMode m, const Poly& pts);

    /// The standing pose for a play centre placed at `center` (stage coordinates, as the editor holds it).
    Standing standingForCenter(const Rigid2& center) const;
    /// The play centre (stage) of a standing pose: the inverse of standingForCenter.
    Rigid2 centerForStanding(const Standing& s) const;

    /// Write `stagePts` (and the extras) into every file that has the universe. Backs each file up before its first
    /// write in this session. `now` names the backups.
    std::expected<SaveResult, std::string> save(const Poly& stagePts, double h, const SaveExtras& extras,
                                                std::time_t now);
    /// Write a universe entry given in file numbers (applying a saved room). Same backups and atomic writes.
    std::expected<SaveResult, std::string> writeUniverse(const std::string& id, const UniverseData& data,
                                                         bool withStanding, bool withPlayArea, std::time_t now);

    /// The universe as the oldest backup has it (Pimax's or SteamVR's own, from before any edit): points in stage
    /// coordinates, the backup file, and its file numbers.
    struct Backup {
        Poly stagePts;
        std::string file;
        UniverseData data;
    };
    std::expected<Backup, std::string> oldestBackup() const;

    // state (read by the editor and the app)
    const std::vector<std::string>& files() const { return files_; }
    const std::vector<std::string>& ids() const { return ids_; }
    const std::map<std::string, UniverseInfo>& universes() const { return unis_; }
    const std::string& universe() const { return uniId_; }
    const std::vector<std::string>& universeFiles() const { return uniFiles_; }
    const std::string& source() const { return source_; }
    const Poly& stagePts() const { return pts_; }
    const Poly& rawPts() const { return rawPts_; }
    double height() const { return height_; }
    double standY() const { return standY_; }
    Standing standing() const { return {tx_, standY_, tz_, yaw_}; }
    Size2 playArea() const { return playArea_; }
    FrameMode mode() const { return mode_; }
    bool backedUp(const std::string& file) const { return backedUp_.contains(file); }

private:
    std::expected<SaveResult, std::string> writeAll(const std::string& id, const std::vector<std::string>& files,
                                                    const Poly& filePts, double h, const SaveExtras& extras,
                                                    std::time_t now);

    std::vector<std::string> files_;
    std::map<std::string, UniverseInfo> unis_;
    std::vector<std::string> ids_;
    std::string uniId_;
    std::vector<std::string> uniFiles_;
    std::string source_;
    Poly pts_;
    double height_ = kDefaultHeight;
    double standY_ = 0;
    double tx_ = 0, tz_ = 0, yaw_ = 0;
    FrameMode mode_ = FrameMode::Off;
    Poly rawPts_;
    Rigid2 adj_;
    Size2 playArea_{1, 1};
    std::set<std::string> backedUp_;
};

/// The frame mode PiBridge starts with for a universe (A for a Pimax one, else Off).
FrameMode defaultFrameMode(bool pimax);

} // namespace pibridge::room
