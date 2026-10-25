// SPDX-License-Identifier: MIT
// pibridge-roomsetup --recenter: the inside-out recenter in the headset (no persistent world anchor yet, so PiBridge asks
// for it at every inside-out start). Pure logic here; the OpenXR side is runRecenterXr (xrapp.h).
//
//   1. "Stand where the play area centre should be. Look forward. Pull the trigger." On the trigger the newest raw
//      head pose of Monado's head ring (pimax-anchor.head, written by the Pimax driver with PIMAX_ANCHOR=1; layout in
//      app/src/core/anchor_offset.h) is taken: that is the head before Monado's offset.
//   2. "Touch the floor with a controller and pull its trigger, or press B / menu to keep the floor." The floor
//      height is the controller's height in the stage at that trigger.
//   3. The result goes to the --recenter-out file ("pibridge-recenter 1\npid N\nhead px py pz qx qy qz qw\nfloor keep|Y\n")
//      and the tool ends. The app (Room page) writes Monado's offset from it (the writer lease, one function in
//      app/src/core/anchor_offset.h). Nothing here writes Monado's files.
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/editor.h"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::roomsetup {

struct RingSample {
    std::uint32_t pid = 0;
    std::uint64_t tsNs = 0;
    std::array<double, 7> pose{0, 0, 0, 0, 0, 0, 1}; // px py pz qx qy qz qw (raw SLAM)
};
/// The newest valid sample of a head ring (nullopt: no ring, written right now, nothing valid).
std::optional<RingSample> parseRing(std::string_view bytes);
/// Read and parse the ring file.
std::optional<RingSample> readRing(const std::string& path);

struct RecenterResult {
    RingSample head;
    std::optional<double> floorStage; // nullopt: keep the floor
};
std::string recenterResultText(const RecenterResult& r);

/// The floor touch of step 2: the controller must be 0.5..2.3 m below the head (same stage frame, so a wrong stage
/// floor does not matter); without a head pose, within 1.5 m of the stage floor.
inline constexpr double kFloorMinBelowHead = 0.5;
inline constexpr double kFloorMaxBelowHead = 2.3;
bool recenterFloorPlausible(bool headValid, double headY, double controllerY);

class RecenterFlow {
public:
    enum class Step { Centre, Floor, Done };
    /// One frame at time `now` (seconds). The recenter (step Centre) is a HOLD: the trigger must be held for kHoldS
    /// while a filling ring shows the progress; the recenter fires when the ring completes (so a brief touch never
    /// recenters, and the user sees it coming).
    void frame(const room::FrameInput& in, double now, const std::function<std::optional<RingSample>()>& ring);
    /// The prompt lines (upper case, the panel font).
    std::vector<std::string> lines() const;
    /// The panel in front of the head (placed at the first valid head pose, then fixed); `now` drives the hold ring.
    void build(room::Batch& b, double now) const;
    Step step() const { return step_; }
    bool finished() const { return step_ == Step::Done; }
    const std::optional<RecenterResult>& result() const { return result_; }

    /// How long the recenter trigger must be held (seconds); the ring fills over this time.
    static constexpr double kHoldS = 0.8;

private:
    Step step_ = Step::Centre;
    std::array<std::uint64_t, 2> prev_{0, 0};
    std::optional<RecenterResult> result_;
    RingSample head_;
    std::string error_;
    bool placed_ = false;
    room::Vec3 at_{0, 1.5, -1.5}, right_{1, 0, 0};
    // the recenter hold: which hand is holding the trigger (-1 none), when it started, and 0..1 fill for build()
    int holdHand_ = -1;
    double holdStart_ = 0;
    double holdProg_ = 0;
};

} // namespace pibridge::roomsetup
