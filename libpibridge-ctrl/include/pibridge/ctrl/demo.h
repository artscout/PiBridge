// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/demo.rs (MIT OR Apache-2.0).
// A scripted "controller" for end-to-end tests: a pure function of the monotonic clock (PIMAX_CTRL_HANDS_DEMO=1 in
// the driver). One 10 s cycle: rest | grip 0>1>0 | trigger 0>1>0 | touch A/X | touch B/Y (+click) | stick | rest;
// the right hand runs 2.5 s later.
#pragma once

#include "pibridge/ctrl/mapping.h"

#include <cstdint>

namespace pibridge::ctrl {

inline constexpr double DEMO_PERIOD_S = 10.0;
inline constexpr double DEMO_RIGHT_OFFSET_S = 2.5;

/// Scripted input at monotonic time `now_ns` for the left (`right == false`) or right hand.
Input demo_input(std::uint64_t now_ns, bool right);

} // namespace pibridge::ctrl
