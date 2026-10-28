// SPDX-License-Identifier: MIT
// The loading animation of pibridge-xroverlay: three dots that pulse one after another, shown while the headset waits
// for the home (PiBridge World starting again after a game). One sprite sheet of kDotsFrames frames, uploaded once.
// The dashboard overlay (pibridge-home --dashboard-overlay) asks for it with $XDG_RUNTIME_DIR/pibridge/loading.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::xroverlay {

inline constexpr std::uint32_t kDotsW = 384; // one frame, pixels
inline constexpr std::uint32_t kDotsH = 128;
inline constexpr int kDotsFrames = 24;       // one cycle; the sheet is one column of frames
inline constexpr double kDotsCycleS = 1.2;   // seconds per cycle
inline constexpr double kLoadingMaxAgeS = 30; // a request file older than this is ignored (its writer died)

/// The sheet: kDotsW x (kDotsH * kDotsFrames), RGBA straight alpha, sRGB values, frame f at rows f*kDotsH...
std::vector<std::uint8_t> drawDotsSheet();
/// The frame to show at time t (seconds).
int dotsFrameAt(double t);
/// How bright dot i (0..2) is in frame f, 0..1 (the pulse travels left to right).
double dotLevel(int frame, int dot);
/// The request file ($XDG_RUNTIME_DIR/pibridge/loading).
std::string loadingRequestPath();
/// The animation is wanted: the request file exists and is younger than kLoadingMaxAgeS.
bool loadingWanted(const std::string& path);

} // namespace pibridge::xroverlay
