// SPDX-License-Identifier: MIT
// The quad views presets of the Performance page: each sets the two densities and the gaze area size; every other
// setting stays as it is. The page shows CUSTOM when the values match no preset.
#pragma once

#include <array>

#include "../qv/settings.h"

namespace pibridge::app {

struct QvPreset {
    const char* name;
    float peripheral; // peripheral_density (edge resolution)
    float focus;      // focus_density (center resolution)
    float gaze;       // gaze_width and gaze_height
};

inline constexpr std::array<QvPreset, 4> kQvPresets{{
    {"PERFORMANCE", 0.10f, 1.00f, 0.30f},
    {"BALANCED", 0.20f, 1.00f, 0.35f},
    {"QUALITY", 0.30f, 1.25f, 0.40f},
    {"EXTREME", 0.40f, 1.50f, 0.45f},
}};

// The index of the preset `s` matches (values within half a percent), or -1 (CUSTOM).
int matchPreset(const qv::Settings& s);
// Set the preset's values in `s`; an index out of range changes nothing.
void applyPreset(qv::Settings& s, int index);

} // namespace pibridge::app
