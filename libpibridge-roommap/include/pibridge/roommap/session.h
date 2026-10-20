// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/session.rs (MIT OR Apache-2.0).
//
// One tracking session against a room map: relocalise at the start (the room comes back where it was), add keyframes
// where the map has no coverage yet, refine the alignment from time to time (drift), and hand the map back at the
// end. The driver feeds frame sets with the tracker's pose of the same frame; the session answers with events.
#pragma once

#include "pibridge/roommap/map.h"
#include "pibridge/roommap/orb.h"
#include "pibridge/roommap/stereo.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace pibridge::roommap::session {

using cam::geom::Pose;
using cam::geom::Vec3;

struct SessionParams {
    double kf_min_dist = 0.35;
    double kf_min_angle_deg = 25.0;
    std::size_t kf_min_points = 60;
    std::size_t max_keyframes = 2000;
    /// Keyframe spacing counts only this session's keyframes (a map update in another light).
    bool kf_per_session = false;
    std::int64_t search_every_ns = 200000000;
    std::int64_t refine_every_ns = 2000000000;
    std::size_t agree = 2;
    double agree_m = 0.05;
    double agree_deg = 2.0;
    double refine_min_m = 0.01;
    double refine_min_deg = 0.3;
    double refine_big_m = 0.3;
    double refine_radius = 2.0;
    orb::OrbParams orb;
    stereo::StereoParams stereo;
    map::RelocParams reloc;
};

/// Alignment room_from_slam (yaw rad, translation m).
struct Alignment {
    double yaw = 0;
    Vec3 t{0.0, 0.0, 0.0};
    Pose pose() const;
    bool close(const Alignment& o, double m, double deg) const;
    bool operator==(const Alignment&) const = default;
};

/// What a frame changed.
struct Event {
    enum class Kind { Relocalized, Refined, Keyframe };
    Kind kind;
    Alignment align;
    std::uint32_t id = 0;
};

struct SessionStats {
    std::uint64_t attempts = 0;
    std::uint64_t found = 0;
    std::uint64_t keyframes = 0;
    std::uint64_t refined = 0;
    std::optional<std::int64_t> first_fix_ns;
};

class RoomSession {
public:
    /// A session on `map` (empty = first visit: the session's own world becomes the room frame).
    RoomSession(map::RoomMap map, stereo::Rig rig, SessionParams p);
    /// A session on a saved map whose room frame is known (keyframes from the first frame, no relocalisation).
    static RoomSession with_alignment(map::RoomMap map, stereo::Rig rig, SessionParams p, const Alignment& room_from_slam);

    std::optional<Alignment> alignment() const { return aligned_; }
    /// One frame set: imgs[i] = image of rig camera i.
    std::vector<Event> on_frame(std::int64_t ts_ns, std::span<const orb::Image> imgs, const Pose& slam_from_body,
                                const Alignment& world_from_slam, bool tracking);
    /// End of the session: the grown map.
    map::RoomMap finish() &&;

    map::RoomMap map;
    SessionParams p;
    SessionStats stats;

private:
    stereo::Rig rig_;
    std::optional<Alignment> aligned_;
    std::vector<Alignment> candidates_;
    std::optional<std::int64_t> last_try_ns_;
    std::optional<std::int64_t> first_ns_;
    bool had_map_;
};

} // namespace pibridge::roommap::session
