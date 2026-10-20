// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/assign.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/assign.h"
#include "pibridge_test.h"

#include <algorithm>

using namespace pibridge::ctrl::track;
using Pairs = std::vector<std::pair<std::size_t, std::size_t>>;

namespace {
Slot slot(V3 last, double age, std::optional<V3> now) {
    Slot s;
    s.last = std::pair{last, age};
    s.tracked_now = now;
    return s;
}
} // namespace

TEST(a_second_solve_of_the_left_controller_is_not_given_to_the_lost_right_hand) {
    const Body b;
    // left tracked at (-0.1, -0.1, -0.35); right was last at (0.2, -0.1, -0.35) 0.2 s ago
    const std::array<Slot, 2> slots{slot({-0.1, -0.1, -0.35}, 0.0, V3{-0.1, -0.1, -0.35}),
                                    slot({0.2, -0.1, -0.35}, 0.2, std::nullopt)};
    // the search returns a pose 5 cm from the left hand: duplicate, not the right hand
    const std::vector<Cand> c1{Cand{{-0.06, -0.12, -0.36}, 1}};
    CHECK(b.assign(c1, slots).empty());
    // a pose near where the right hand was is taken
    const std::vector<Cand> c2{Cand{{0.22, -0.1, -0.33}, 1}};
    CHECK((b.assign(c2, slots) == Pairs{{0, 1}}));
}

TEST(two_new_poses_go_by_continuity_then_side) {
    const Body b;
    const std::array<Slot, 2> none{};
    // crossed hands without history: side rule (left = smaller x)
    const std::vector<Cand> c{Cand{{0.15, -0.1, -0.4}, 0}, Cand{{-0.15, -0.1, -0.4}, 1}};
    auto a = b.assign(c, none);
    std::sort(a.begin(), a.end());
    CHECK((a == Pairs{{0, 1}, {1, 0}}));
    // with history the hands keep their identity even when they cross the middle a little
    const std::array<Slot, 2> hist{slot({0.05, -0.1, -0.4}, 0.04, std::nullopt),
                                   slot({-0.05, -0.1, -0.4}, 0.04, std::nullopt)};
    const std::vector<Cand> c2{Cand{{0.07, -0.1, -0.4}, 0}, Cand{{-0.07, -0.1, -0.4}, 1}};
    a = b.assign(c2, hist);
    std::sort(a.begin(), a.end());
    CHECK((a == Pairs{{0, 0}, {1, 1}}));
}

TEST(teleports_and_impossible_poses_are_rejected) {
    const Body b;
    const std::array<Slot, 2> slots{slot({-0.2, -0.1, -0.3}, 0.04, std::nullopt), Slot{}};
    // 0.5 m in 40 ms for the left slot, and wrong side for a fresh right: left cannot take it
    const std::vector<Cand> c{Cand{{0.3, -0.1, -0.3}, 0}};
    CHECK((b.assign(c, slots) == Pairs{{0, 1}}));
    // 1.5 m away: out of reach for both
    const std::vector<Cand> far{Cand{{0.0, 0.0, -1.5}, 0}};
    CHECK(b.assign(far, std::array<Slot, 2>{}).empty());
    CHECK(b.collapsed({V3{0.0, 0.0, 0.0}, V3{0.03, 0.0, 0.0}}, {V3{0.0, 0.0, 0.0}, V3{0.3, 0.0, 0.0}}) ==
          std::optional<std::size_t>(1));
}

PIBRIDGE_TEST_MAIN()
