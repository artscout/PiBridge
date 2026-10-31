// SPDX-License-Identifier: MIT
// The volume indicator of pibridge-xroverlay: a speaker and a bar with the headset volume, shown over any app for a
// moment after a volume button press. pibridge-keepalived changes the volume (the headset's buttons; under Monado the
// Pimax driver hands them over) and writes the new value to $XDG_RUNTIME_DIR/pibridge/volume-hud ("<percent>").
// One sprite sheet of kVolFrames frames (0, 5 .. 100 %), uploaded once, like the ring.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::xroverlay {

inline constexpr std::uint32_t kVolW = 512; // one frame, pixels
inline constexpr std::uint32_t kVolH = 96;
inline constexpr int kVolFrames = 21;      // 0, 5 .. 100 %; the sheet is one column of frames
inline constexpr double kVolShowS = 1.5;   // shown this long after the last change

/// The sheet: kVolW x (kVolH * kVolFrames), RGBA straight alpha, sRGB values, frame f at rows f*kVolH...
std::vector<std::uint8_t> drawVolumeSheet();
/// The frame for a volume in percent (clamped, to the nearest 5 %).
int volumeFrame(int percent);
/// The file keepalived writes ($XDG_RUNTIME_DIR/pibridge/volume-hud).
std::string volumeHudPath();
/// The volume to show now: the file's percent while it is younger than kVolShowS, else nothing.
std::optional<int> volumeToShow(const std::string& path);

} // namespace pibridge::xroverlay
