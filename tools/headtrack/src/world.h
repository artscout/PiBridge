// SPDX-License-Identifier: MIT
// The world pibridge-headtrack hands out: the room anchor and the recenter on top of the tracker's raw head pose, as
// Monado's Pimax driver did (pimax_anchor.h, pimax_hmd.c pimax_hmd_apply_recenter). The file formats are the ones
// libpibridge-anchor reads and writes (world.h), so pibridge-roommap and the app's RECENTER IN HEADSET work as before:
//   - the head ring $XDG_RUNTIME_DIR/pimax-anchor.head: 16-byte header (u32 magic 0x434E4150, u32 version 1, u32 seq
//     odd while writing, u32 writer pid), 128 slots of 40 bytes (u64 ts_ns, f32 pos[3], f32 quat xyzw, u32 flags bit 0
//     position valid), the raw head poses, at most one every 3 ms; created under a temporary name and renamed in;
//   - the offset file $XDG_RUNTIME_DIR/pimax-anchor.offset ("pimax-anchor-offset 1", "pid N", "pose px py pz qx qy qz
//     qw"), deleted at start (the tracker world is new every run), re-read every 250 ms, honoured only for our pid,
//     applied as offset * raw. The first offset of a session is placed at once, later ones glide (0.05 m/s, 2 deg/s,
//     up to 3x while the head turns faster than 60 deg/s).
//   - the recenter (a long press of the Pimax button): the head's forward onto -Z and its position over the origin,
//     the floor kept; cleared by a new room offset.
#pragma once

#include "pibridge/cam/slam.h"

#include <atomic>
#include <cstdint>
#include <string>

namespace pibridge::headtrack {

using cam::slam::Quat;
using cam::slam::Vec3;

struct RigidPose {
    Vec3 p{};
    Quat q = cam::slam::QUAT_ID;
};

class World {
public:
    World();
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    /// The raw head pose at `ts_ns` (recorded into the ring when `record`) to the transform that takes raw poses into
    /// the world handed out: recenter * offset. `head_dps`: the head turn rate (the glide is faster while it turns).
    RigidPose update(std::int64_t ts_ns, const RigidPose& raw, bool position_valid, double head_dps, std::int64_t now_ns);
    /// Recenter at the next update (any thread).
    void request_recenter() { recenter_wanted_ = true; }

private:
    void create_ring();
    void keep_ring(std::int64_t now_ns);
    void record(std::int64_t ts_ns, const RigidPose& raw, bool position_valid);
    void refresh_offset(std::int64_t now_ns);
    void step(std::int64_t now_ns, double head_dps);

    std::string ring_path_, offset_path_;
    std::uint8_t* ring_ = nullptr;
    std::uint64_t ring_dev_ = 0, ring_ino_ = 0;
    std::uint32_t next_ = 0;
    std::int64_t last_ring_ns_ = 0, last_ring_check_ns_ = 0, last_check_ns_ = 0, last_step_ns_ = 0;
    std::int64_t mtime_s_ = 0, mtime_ns_ = 0;
    bool have_offset_ = false, have_applied_ = false, placed_ = false;
    RigidPose offset_, applied_;
    std::uint32_t generation_ = 0, seen_generation_ = 0;
    RigidPose recenter_;
    std::atomic<bool> recenter_wanted_{false};
};

/// a * b
RigidPose compose(const RigidPose& a, const RigidPose& b);

} // namespace pibridge::headtrack
