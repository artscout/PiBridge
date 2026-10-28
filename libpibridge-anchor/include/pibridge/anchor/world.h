// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/world.rs (MIT OR Apache-2.0).
//
// The files shared with Monado's room-anchor hook (`forks/monado/src/xrt/drivers/pimax/pimax_anchor.h`, enabled by
// PIMAX_ANCHOR=1) and the offset maths. Byte for byte the formats of the Rust code and of the Monado reader/writer:
//
// * Head ring (`$XDG_RUNTIME_DIR/pimax-anchor.head`, PIMAX_ANCHOR_HEAD_FILE; written by Monado): the raw SLAM head
//   poses (before the offset) with their timestamps. 16-byte header (u32 magic 0x434E4150, u32 version 1, u32 seq,
//   odd while writing, u32 writer pid), then 128 slots of 40 bytes: u64 ts_ns (CLOCK_MONOTONIC), f32 pos[3],
//   f32 quat[4] (x y z w), u32 flags (bit 0 = position valid), round robin, empty slots have ts 0. Little endian.
// * Offset file (`$XDG_RUNTIME_DIR/pimax-anchor.offset`, PIMAX_ANCHOR_FILE; written here, read by Monado): text,
//   `pimax-anchor-offset 1`, `pid N`, `pose px py pz qx qy qz qw`. Monado applies `app_from_slam = offset * slam` and
//   ignores the file unless the pid is its own; it deletes the file at its start. Changes after the first one glide
//   (PIMAX_ANCHOR_GLIDE).
// * Writer lease (`omarchyvr-anchor.offset.owner` next to the offset): one writer of the offset per Monado session;
//   write_offset refuses without it; the decision and the write run under one flock (`omarchyvr-anchor.offset.lock`).
//
// The environment names are the Rust (and Monado) ones, PIMAX_ANCHOR_FILE / PIMAX_ANCHOR_HEAD_FILE: Monado reads
// only these, so a PIBRIDGE_ name could point the tools at another file than the one Monado reads.
#pragma once

#include "pibridge/anchor/backend.h"
#include "pibridge/cam/geom.h"
#include "pibridge/cam/io.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pibridge::anchor {

using cam::geom::Pose;
using cam::geom::Vec3;

inline constexpr std::uint32_t RING_MAGIC = 0x434E4150;
inline constexpr std::size_t RING_SLOTS = 128;
inline constexpr std::size_t RING_HDR = 16;
inline constexpr std::size_t RING_SLOT = 40;
inline constexpr std::size_t RING_SIZE = RING_HDR + RING_SLOTS * RING_SLOT;

/// $XDG_RUNTIME_DIR (set, even when empty: Rust's `var_os`), else /tmp.
std::string runtime_dir();
/// The head ring: PIMAX_ANCHOR_HEAD_FILE (when not empty), else <runtime>/pibridge/<PIBRIDGE_BACKEND>/anchor.head
/// (backend.h), else (no backend: old tools and tests) <runtime>/pimax-anchor.head.
std::string ring_path();
/// The offset file: PIMAX_ANCHOR_FILE (when not empty), else <runtime>/pibridge/<PIBRIDGE_BACKEND>/anchor.offset,
/// else <runtime>/pimax-anchor.offset.
std::string offset_path();

/// (t, quaternion x y z w) of a pose (`geom::quat_from_mat`).
std::pair<Vec3, std::array<double, 4>> pq(const Pose& p);
/// The pose of (t, q) (q normalised first).
Pose from_pq(const Vec3& t, const std::array<double, 4>& q);
/// Interpolate between two poses (position linear, rotation slerp).
Pose interp(const Pose& a, const Pose& b, double f);

/// One ring sample: (ts_ns, world_from_head, position valid).
struct RingSample {
    std::uint64_t ts_ns = 0;
    Pose pose;
    bool valid = false;
};

struct HeadRing {
    std::uint32_t pid = 0;
    /// Sorted by time (stable).
    std::vector<RingSample> samples;

    /// Parse the file bytes; nullopt when too short or not a ring (magic, version 1).
    static std::optional<HeadRing> parse(std::string_view b);
    /// Seqlock read of the file (up to 8 tries).
    static std::optional<HeadRing> read(const std::string& path);
    /// Newest sample time (0 when empty).
    std::uint64_t newest_ns() const;
    /// Raw head pose at `ts_ns`: interpolated between the valid samples around it when they are at most `max_gap_ns`
    /// apart, else the nearest valid sample within `max_gap_ns / 2`.
    std::optional<Pose> at(std::uint64_t ts_ns, std::uint64_t max_gap_ns) const;
};

/// The ring bytes Monado writes (test helper / reference of the layout): the samples in slot order.
std::string ring_bytes(std::uint32_t pid, const std::vector<RingSample>& samples);

/// `offset` with `offset * world_from_tag_now == reference`; `yaw_only`: a rotation about +Y and a translation.
Pose compute_offset(const Pose& reference, const Pose& world_from_tag_now, bool yaw_only);

/// Not in the Rust code (PiBridge's Recenter): the offset that puts the raw head pose `slam_from_head` at x = z = 0 of
/// the app world, looking along -Z (yaw only: the tracker world is gravity aligned, roll and pitch stay), its height
/// unchanged. Write it with take_lease + write_offset (role Apply: a user action).
Pose recenter_offset(const Pose& slam_from_head);

/// The offset file text.
std::string offset_text(std::uint32_t pid, const Pose& o);
/// Parse the offset file text: (pid, offset).
std::optional<std::pair<std::uint32_t, Pose>> parse_offset(std::string_view s);
/// Read the offset file.
std::optional<std::pair<std::uint32_t, Pose>> read_offset(const std::string& path);
/// Write the offset for Monado `pid` atomically as this process; refused (PermissionDenied) unless this process
/// holds the writer lease of that session.
cam::Result<void> write_offset(const std::string& path, std::uint32_t pid, const Pose& o);
/// write_offset as writer `me` (tests pass a fake writer pid).
cam::Result<void> write_offset_as(const std::string& path, std::uint32_t pid, std::uint32_t me, const Pose& o);

/// (translation error m, rotation error deg) between two poses (omarchyvr-anchor detect.rs `pose_error`).
std::pair<double, double> pose_error(const Pose& a, const Pose& b);

/// Robust mean of poses: samples further than `pos_tol` from the per-axis median are dropped. Returns (mean, kept,
/// position scatter m, rotation scatter deg). `rot_tol_deg` is unused (as in the Rust code). A NaN position panics
/// like the Rust sort does.
std::optional<std::tuple<Pose, std::size_t, double, double>> robust_mean(const std::vector<Pose>& v, double pos_tol,
                                                                         double rot_tol_deg);

// ---------------------------------------------------------------------------------------------------------------
// One writer of the room offset per Monado session (the writer lease).
//
// Live failure 2026-10-17 16:16-16:17 (omarchyvr): Fast room setup wrote the offset, and the detached `auto` started
// at the session start recognised the brand-new map 73 s later and wrote a second offset 32 cm away; Monado snapped
// the world (the "floor jump"). The lease file's first line stays what the apps read (`roommap <monado pid>` /
// `anchor <monado pid>`); schema 2 adds `lease 2`, `writer <pid>`, `role <role>`.

inline constexpr std::uint32_t LEASE_SCHEMA = 2;

/// Who writes the offset.
enum class Role { Setup, Auto, Follow, Reloc, Apply, Tag };

const char* role_name(Role r);
std::optional<Role> role_parse(std::string_view s);
/// The lease's first word: "anchor" for the AprilTag, "roommap" otherwise.
const char* role_kind(Role r);
/// Plain words for the user.
const char* role_title(Role r);
/// A user action (setup, recognise, apply, tag): takes the lease from a running background writer.
bool role_preempts(Role r);
/// Only places a room nobody placed yet in this session (`auto`).
bool role_first_only(Role r);

struct Lease {
    std::string kind;
    std::uint32_t session = 0;
    /// 0: unknown (a schema 1 file written by an app).
    std::uint32_t writer = 0;
    std::optional<Role> role;
    std::uint32_t schema = 1;

    std::string text() const;
    /// Schema 2, and the app's schema 1 line ("roommap 42": writer unknown).
    static std::optional<Lease> parse(std::string_view s);
    std::string holder_title() const;
    bool operator==(const Lease&) const = default;
};

/// The lease next to an offset file.
std::string lease_path_for(const std::string& offset);
/// The lease of the live offset file.
std::string lease_path();
std::optional<Lease> read_lease(const std::string& path);

/// May `me` (role `role`) take the lease of session `session`? A value = yes (after ending the running writer with
/// that pid when it is set); an error = no, in plain words. `offset_written`: the offset file holds an offset of this
/// session. Pure.
std::expected<std::optional<std::uint32_t>, std::string> decide_lease(const std::optional<Lease>& cur,
                                                                      std::uint32_t session, std::uint32_t me,
                                                                      Role role, bool offset_written,
                                                                      const std::function<bool(std::uint32_t)>& alive);

/// Take the writer lease of Monado session `session` for this process as `role` (kept when held already). A user
/// action ends a running background writer first (SIGTERM, only one of our offset writers by name). Returns a line
/// for the log when another writer was ended.
std::expected<std::optional<std::string>, std::string> take_lease(const std::string& offset, std::uint32_t session,
                                                                  Role role);
/// take_lease with the writer pid, the liveness check and the eviction passed in (tests).
std::expected<std::optional<std::string>, std::string> take_lease_as(
    const std::string& offset, std::uint32_t session, std::uint32_t me, Role role,
    const std::function<bool(std::uint32_t)>& alive,
    const std::function<std::expected<void, std::string>(std::uint32_t)>& evict);

/// Does this process (or `me`) hold the lease of session `session`?
bool holds_lease(const std::string& offset, std::uint32_t session);
bool holds_lease_as(const std::string& offset, std::uint32_t session, std::uint32_t me);

/// The process names (`/proc/<pid>/comm`, 15 bytes) of the offset writers end_writer may stop: omarchyvr's
/// `omarchyvr-roommap` / `omarchyvr-anchor` and PiBridge's `pibridge-roommap` / `pibridge-anchor`.
bool is_offset_writer_comm(std::string_view comm);

// ---------------------------------------------------------------------------------------------------------------
// Is the head ring the live feed of the running Monado? (live failure 2026-10-16: SteamVR's Monado driver replaced
// the ring with vrserver's pid and exited; readers must say who wrote the ring and how fresh it is.)

/// A ring whose newest sample is older than this (s) is stale.
inline constexpr double RING_STALE_S = 5.0;

/// Pids of running processes whose `/proc/*/comm` is `name`, ascending.
std::vector<std::uint32_t> pids_named(std::string_view name);
/// The running `monado-service` processes, else SteamVR's `vrserver`.
std::vector<std::uint32_t> tracker_pids();
/// The pid runs (`/proc/<pid>` exists; 0 never).
bool pid_alive(std::uint32_t pid);

/// State of the head-pose feed.
struct Feed {
    enum class Kind { Missing, NoMonado, Foreign, Stale, Live };
    Kind kind = Kind::Missing;
    std::uint32_t pid = 0;
    /// Foreign: the running Monado.
    std::uint32_t monado = 0;
    /// Foreign: the writer still runs.
    bool alive = false;
    /// Stale: age of the newest sample (s), none without samples.
    std::optional<double> age_s;

    std::optional<std::uint32_t> live_pid() const;
    /// One sentence for the user.
    std::string plain() const;
    bool operator==(const Feed&) const = default;
};

/// Classify `ring` against the running tracker pids at `now_ns` (CLOCK_MONOTONIC). Pure.
Feed check_feed(const std::optional<HeadRing>& ring, const std::vector<std::uint32_t>& monado, std::uint64_t now_ns,
                const std::function<bool(std::uint32_t)>& alive);
/// The feed now: the ring file, the running Monado (or vrserver) and the clock.
Feed feed_now();

} // namespace pibridge::anchor
