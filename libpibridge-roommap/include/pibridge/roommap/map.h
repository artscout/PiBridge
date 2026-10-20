// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/map.rs (MIT OR Apache-2.0).
//
// The room map: stereo keyframes (pose in the room frame + their triangulated ORB points in room coordinates),
// saved per room (`map.bin`, magic "PXRM", version 1, little endian; and a `room.json` summary), and the
// relocalisation of a new frame against it (keyframes ranked by quick descriptor hits of a sample of the query,
// brute-force matching against the pooled points of the best ones, then gravity-aware 4-DoF PnP over all rig
// cameras). Anchors are named poses in the same frame. Room frame: gravity aligned, y up; a session's tracker world
// is tied to it by room_from_slam = yaw + translation.
//
// map.bin: "PXRM", u32 version, string device_uid (u32 length + bytes), u32 n_cams + u32 ids (at most 8 read),
// u32 sessions, u32 next_id, u32 n_keyframes, per keyframe {u32 id, i64 ts_ns, u32 session, pose (7 f64: t, quat
// x y z w), u32 n_points, per point {3 f32, 32-byte descriptor}}, u32 n_anchors, per anchor {string name, 7 f64}.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/cam/slam.h"
#include "pibridge/roommap/orb.h"
#include "pibridge/roommap/pnp.h"
#include "pibridge/roommap/stereo.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::roommap::map {

using cam::geom::Mat3;
using cam::geom::Pose;
using cam::geom::Vec3;
using cam::slam::Quat;

inline constexpr char MAGIC[4] = {'P', 'X', 'R', 'M'};
inline constexpr std::uint32_t VERSION = 1;

struct MapPoint {
    Vec3 p{};
    orb::Descriptor desc{};
    bool operator==(const MapPoint&) const = default;
};

struct Keyframe {
    std::uint32_t id = 0;
    std::int64_t ts_ns = 0;
    std::uint32_t session = 0;
    /// room_from_body at capture.
    Pose pose;
    std::vector<MapPoint> points;
    bool operator==(const Keyframe&) const = default;
};

struct Anchor {
    std::string name;
    Vec3 p{};
    Quat q{};
    bool operator==(const Anchor&) const = default;
};

/// Roll/pitch of a gravity-aligned orientation without its yaw: q = Ry(yaw_of(q)) * tilt.
Mat3 tilt_of(const Quat& q);
/// Yaw part of a rotation matrix.
double yaw_of_mat(const Mat3& r);

struct RelocParams {
    std::size_t top_k = 3;
    std::size_t rank_sample = 200;
    std::uint32_t rank_dist = 45;
    std::uint32_t max_dist = 60;
    float ratio = 0.8f;
    pnp::PnpParams pnp{0.8, 400, 0.999, 20, 7};
    std::size_t min_inliers = 25;
    double min_inlier_frac = 0.25;
    bool operator==(const RelocParams&) const = default;
};

struct Reloc {
    Pose room_from_body;
    std::size_t inliers = 0;
    std::size_t corrs = 0;
    double rms_deg = 0;
    std::vector<std::uint32_t> keyframes;
};

/// room_from_slam (yaw, translation) from one relocalisation and the tracker's pose of the same frame; the
/// translation is re-derived for the yaw-only rotation so the body lands exactly on the relocalised position.
std::pair<double, Vec3> room_from_slam(const Reloc& r, const Pose& slam_from_body);

struct RoomMap {
    std::string device_uid;
    std::vector<std::uint32_t> cams;
    std::uint32_t sessions = 0;
    std::vector<Keyframe> keyframes;
    std::vector<Anchor> anchors;
    std::uint32_t next_id = 0;

    static RoomMap make(const std::string& device_uid, std::span<const std::size_t> cams);
    std::size_t points() const;
    bool wants_keyframe(const Pose& room_from_body, double min_dist, double min_angle_deg) const;
    /// Counting only the keyframes of `session` (nullopt: all).
    bool wants_keyframe_in(const Pose& room_from_body, double min_dist, double min_angle_deg,
                           std::optional<std::uint32_t> session) const;
    /// Add a keyframe from stereo points (body frame) at room_from_body; returns its id.
    std::uint32_t add_keyframe(std::int64_t ts_ns, const Pose& room_from_body, std::span<const stereo::StereoPoint> pts);
    /// Relocalise a frame: query[i] = features of rig camera i; r_tilt = the body's roll/pitch; prior = (room
    /// position, radius) restricting the keyframes.
    std::optional<Reloc> relocalize(std::span<const stereo::Features> query, const stereo::Rig& rig, const Mat3& r_tilt,
                                    const std::optional<std::pair<Vec3, double>>& prior, const RelocParams& p) const;
    void set_anchor(const std::string& name, const Vec3& p, const Quat& q);
    bool remove_anchor(const std::string& name);
    std::string encode() const;
    static std::expected<RoomMap, std::string> decode(std::string_view data);
    /// Save dir/map.bin (atomic, fsync) and dir/room.json; the map path.
    std::expected<std::string, std::string> save(const std::string& dir) const;
    /// Load dir/map.bin, or a file path.
    static std::expected<RoomMap, std::string> load(const std::string& path);
    bool operator==(const RoomMap&) const = default;
};

/// omarchyvr-room's default room directory: ~/.local/share/omarchyvr/rooms/<name>.
std::string room_dir(const std::string& name);

} // namespace pibridge::roommap::map
