// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/assign.rs (MIT OR Apache-2.0).
//
// Which controller is which: a rule-based body model for the live tracker (head frame = OpenXR head axes, x right,
// y up, z back). Used for every pose found by the brute-force search, and to settle two tracked poses that collapsed
// onto one controller.
//
// Rules:
// * reach: a hand is at most `reach` from its shoulder (shoulders 0.18 m to the side, 0.25 m below and 0.10 m
//   behind the eyes; the yaw-only part of the head is ignored here, so looking down shortens the reach a bit);
// * side: the left hand is not more than `cross` to the right of the head centre, and the other way round;
// * continuity: a slot seen less than `memory_s` ago only takes a pose within `0.12 m + max_speed * age` of where it
//   was;
// * separation: two hands are never closer than `min_sep`;
// * among the feasible assignments the one with the lowest cost wins (distance to the slot's last pose, a penalty for
//   the wrong side of the head centre and for the other hand's LED-shape label).
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::ctrl::track {

using V3 = std::array<double, 3>;

/// A pose found this frame: head-frame position and the LED model it fitted best (0 left, 1 right).
struct Cand {
    V3 p{};
    std::size_t model = 0;
};

/// What we know about a slot: its last pose and how old it is (s), and whether it is already tracked this frame.
struct Slot {
    std::optional<std::pair<V3, double>> last;
    std::optional<V3> tracked_now;
};

struct Body {
    std::array<V3, 2> shoulder{V3{-0.18, -0.25, 0.10}, V3{0.18, -0.25, 0.10}};
    double reach = 0.80;
    double cross = 0.30;
    double min_sep = 0.08;
    double max_speed = 2.5;
    double memory_s = 1.5;

    /// Why `p` cannot be `slot`'s hand (nullptr = plausible), ignoring continuity and the other hand.
    const char* implausible(std::size_t slot, const V3& p) const;
    /// Assign candidates to the free slots: returns (candidate index, slot) pairs. Slots already tracked this frame
    /// are not free. Maximises the number of assigned candidates, then minimises the cost; two candidates closer
    /// than `min_sep` are never both assigned.
    std::vector<std::pair<std::size_t, std::size_t>> assign(std::span<const Cand> cands,
                                                            const std::array<Slot, 2>& slots) const;
    /// Two tracked hands closer than `min_sep`: which slot to drop (the one that moved more from its last pose).
    std::optional<std::size_t> collapsed(const std::array<V3, 2>& now,
                                         const std::array<std::optional<V3>, 2>& last) const;

private:
    std::optional<double> cost(std::size_t slot, const Cand& c, const Slot& s, const Slot& other) const;
};

} // namespace pibridge::ctrl::track
