// SPDX-License-Identifier: MIT
#include "presets.h"

#include <cmath>

namespace pibridge::app {

namespace {
bool near(float a, float b) {
    return std::fabs(a - b) < 0.005f;
}
} // namespace

int matchPreset(const qv::Settings& s) {
    for (size_t i = 0; i < kQvPresets.size(); i++) {
        const auto& p = kQvPresets[i];
        if (near(s.peripheralDensity, p.peripheral) && near(s.focusDensity, p.focus) && near(s.gazeSize[0], p.gaze) &&
            near(s.gazeSize[1], p.gaze)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void applyPreset(qv::Settings& s, int index) {
    if (index < 0 || index >= static_cast<int>(kQvPresets.size())) {
        return;
    }
    const auto& p = kQvPresets[static_cast<size_t>(index)];
    s.peripheralDensity = p.peripheral;
    s.focusDensity = p.focus;
    s.gazeSize = {p.gaze, p.gaze};
}

} // namespace pibridge::app
