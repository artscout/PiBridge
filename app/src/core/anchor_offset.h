// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/world.rs (MIT OR Apache-2.0): the offset file and the writer lease only.
//
// Recenter in the inside-out mode (Room page, pibridge-roomsetup --recenter): Monado's PIMAX_ANCHOR hook applies
// `app_from_slam = offset * slam` to the head pose, the offset read from a text file; PiBridge writes that file. This is
// a minimal private writer behind one function (writeRecenterOffset) until libpibridge-anchor (milestone R1) is there;
// then it is swapped for that library's API. The formats are omarchyvr's, byte for byte:
//   offset   $XDG_RUNTIME_DIR/pimax-anchor.offset (PIMAX_ANCHOR_FILE overrides):
//              "pimax-anchor-offset 1\npid N\npose px py pz qx qy qz qw\n" (6 / 7 decimals); Monado ignores it when
//              the pid is not its own (the process that runs the Pimax driver: vrserver under SteamVR) and deletes it
//              at its start;
//   lease    omarchyvr-anchor.offset.owner next to it: "roommap <session>\nlease 2\nwriter <pid>\nrole setup\n", one
//              writer per Monado session; the check and the write under flock(omarchyvr-anchor.offset.lock);
//   ring     $XDG_RUNTIME_DIR/pimax-anchor.head (PIMAX_ANCHOR_HEAD_FILE): 16 byte header (u32 magic 0x434E4150,
//              u32 version 1, u32 seq, u32 writer pid), 128 slots of 40 bytes (u64 ts_ns, f32 pos[3], f32 quat[4]
//              x y z w, u32 flags bit 0 = position valid). Its writer pid names the session.
// PiBridge writes as role "setup" (a user action), but it never ends another writer: when a live one holds the lease
// (omarchyvr's roommap, the AprilTag), the recenter is refused in plain words.
//
// The recenter itself (recenterOffset): the user stands where the play area centre should be and looks forward; the
// new offset turns the raw head's forward (yaw only, gravity stays) onto -Z and puts the head's floor point at the
// origin. The floor: kept (the current offset's height), or moved by the height of a floor touch measured in the
// stage (a vertical shift is the same in every gravity-aligned frame).
#pragma once

#include <QByteArray>
#include <QString>
#include <array>
#include <expected>
#include <functional>
#include <optional>

namespace pibridge::app {

struct AnchorPose {
    std::array<double, 3> t{0, 0, 0};
    std::array<double, 4> q{0, 0, 0, 1}; // x y z w
};

inline constexpr quint32 kRingMagic = 0x434E4150;
inline constexpr int kRingHdr = 16;
inline constexpr int kRingSlots = 128;
inline constexpr int kRingSlot = 40;

struct AnchorFiles {
    QString offset; // pimax-anchor.offset
    QString ring;   // pimax-anchor.head
    static AnchorFiles defaults();
    QString lease() const; // omarchyvr-anchor.offset.owner next to the offset
    QString lock() const;  // omarchyvr-anchor.offset.lock
};

QString anchorOffsetText(quint32 pid, const AnchorPose& o);
std::optional<std::pair<quint32, AnchorPose>> parseAnchorOffset(const QByteArray& text);

struct AnchorLease {
    QString kind;     // roommap | anchor
    quint32 session = 0;
    quint32 writer = 0;
    QString role;     // setup | auto | follow | reloc | apply | tag | ""
    quint32 schema = 1;
    QString text() const;
    static std::optional<AnchorLease> parse(const QByteArray& s);
};

struct RingHead {
    quint32 pid = 0;
    quint64 tsNs = 0;
    AnchorPose pose;
};
// The newest valid head sample of a ring (nullopt: not a ring, written right now, or no valid sample).
std::optional<RingHead> parseRingNewest(const QByteArray& bytes);

// The recenter (pure). `head`: the raw SLAM head pose at the trigger; `current`: the offset in use (nullopt: none);
// `floorStage`: the floor touch's height in the stage (nullopt: keep the floor).
AnchorPose recenterOffset(const AnchorPose& head, const std::optional<AnchorPose>& current,
                          const std::optional<double>& floorStage);

struct RecenterInput {
    quint32 session = 0;          // the Monado session (the ring's writer)
    AnchorPose head;              // raw head at the trigger
    std::optional<double> floorStage;
};
struct RecenterWritten {
    AnchorPose offset;
    QString words; // plain words for the page
};
// THE one function the app calls (swapped for libpibridge-anchor later): take the lease of the session (refused while
// another live writer holds it), compute the offset from the current one and write it atomically, all under the lock.
// `me`: this process; `alive`: is that pid running.
std::expected<RecenterWritten, QString> writeRecenterOffset(const AnchorFiles& f, const RecenterInput& in, quint32 me,
                                                            const std::function<bool(quint32)>& alive);

// The result file pibridge-roomsetup --recenter writes: "pibridge-recenter 1\npid N\nhead px py pz qx qy qz qw\nfloor keep|Y\n".
std::optional<RecenterInput> parseRecenterResult(const QByteArray& text);

} // namespace pibridge::app
