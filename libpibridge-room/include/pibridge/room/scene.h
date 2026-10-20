// SPDX-License-Identifier: MIT
// Ported from ZoneTweak main.go (buildScene, buildPanel), area.go (drawArea, drawLock, drawArcs), features.go
// (drawProximity, wallColors) and tool.go (the beams) (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// The in-headset view of the editor as plain coloured triangles and lines in stage space: the floor grid, the
// boundary walls (translucent sheets with corner posts and their lengths), the play area and centre with its handles,
// the warning line of the proximity test, the ghost of the saved outline, the new outline / trace, the controller
// beams, the menu panel with its buttons and tooltips, and the status line. ZoneTweak's prison set dressing (lasers,
// generators, robots, lights, fog) is replaced by this plain look.
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/editor.h"

namespace pibridge::room {

namespace col {
inline constexpr Rgba green{0.24F, 0.86F, 0.52F, 1};
inline constexpr Rgba amber{1.00F, 0.70F, 0.25F, 1};
inline constexpr Rgba ghost{0.42F, 0.48F, 0.45F, 1};
inline constexpr Rgba grid{0.13F, 0.18F, 0.15F, 1};
inline constexpr Rgba grid1{0.20F, 0.27F, 0.23F, 1};
inline constexpr Rgba yellow{1.00F, 0.84F, 0.28F, 1};
inline constexpr Rgba white{0.90F, 0.94F, 0.92F, 1};
inline constexpr Rgba muted{0.58F, 0.65F, 0.61F, 1};
inline constexpr Rgba panel{0.06F, 0.08F, 0.07F, 0.92F};
inline constexpr Rgba button{0.13F, 0.17F, 0.15F, 1};
inline constexpr Rgba buttonHot{0.24F, 0.86F, 0.52F, 1};
inline constexpr Rgba buttonOn{0.10F, 0.30F, 0.20F, 1};
inline constexpr Rgba blue{0.45F, 0.70F, 1.00F, 1};
inline constexpr Rgba warn{1.00F, 0.55F, 0.15F, 1};
inline constexpr Rgba danger{1.00F, 0.22F, 0.20F, 1};
inline constexpr Rgba area{1.00F, 0.30F, 0.85F, 1};
inline constexpr Rgba laser{0.75F, 0.95F, 1.00F, 1};
inline constexpr Rgba beam{0.30F, 1.00F, 0.70F, 1};
} // namespace col

/// Rebuild `b` from the editor's state. `t` (seconds) drives the small pulses.
void buildScene(Editor& e, Batch& b, double t);

} // namespace pibridge::room
