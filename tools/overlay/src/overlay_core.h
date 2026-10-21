// SPDX-License-Identifier: MIT
// pibridge-overlay, the pure parts (no OpenVR, no I/O): a small fixed panel drawn in code. A compact in-VR page with one
// START/STOP RECORDING button and a ROOM section (ROOM SETUP, QUICK ROOM SETUP), the laser hit test, and the SteamVR
// application manifest. The recording parameters (view / fps / size) live in the app's Recording page, not here: the
// overlay only starts and stops, and the button follows the app's real recording state. overlay.cpp uploads the panel
// to a small dashboard overlay ONCE (re-drawing only when the hovered target or the recording state changes), and on a
// click writes a request file the app reads. There is no app-window embed and no per-frame upload (that crashed SteamVR).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::overlay {

// The panel is a few hundred pixels, not 2048: a small RGBA texture uploaded at most on a hover / state change.
inline constexpr std::uint32_t kPanelW = 520;
inline constexpr std::uint32_t kPanelH = 354;

// A laser target: one of the wide buttons.
struct Hit {
    enum class Kind { None, Room, QuickRoom, Record };
    Kind kind = Kind::None;
    bool operator==(const Hit&) const = default;
};

// The overlay request file in $XDG_RUNTIME_DIR/pibridge; the app reads one line and acts on it itself.
inline constexpr char kRequestFileName[] = "overlay-request";
inline constexpr char kRequestRoomSetup[] = "room-setup";
inline constexpr char kRequestQuickRoomSetup[] = "quick-room-setup";
inline constexpr char kRequestRecordStart[] = "record-start";
inline constexpr char kRequestRecordStop[] = "record-stop";

// The request line for a hit (""; for Hit::Kind::None). The record button sends start or stop by `recording`.
std::string requestFor(Hit h, bool recording);

// The target under a laser-mouse coordinate. OpenVR reports the cursor in GL space (origin bottom-left, range [0, w] x
// [0, h] because the overlay's mouse scale is the panel size); the panel's rows run top-first, so y is flipped here.
Hit hitTest(float x, float y);

// The panel as RGBA8 (top row first, what SetOverlayRaw wants): the dark card with the START/STOP RECORDING button and
// the ROOM buttons, the target in `hovered` lit; the record button reads STOP RECORDING when `recording`, else START
// RECORDING. kPanelW x kPanelH x 4.
std::vector<std::uint8_t> drawPanel(bool recording, Hit hovered);

// The SteamVR application manifest for the one dashboard overlay (is_dashboard_overlay). `binaryPath` and `iconPath`
// are absolute; `appKey` is e.g. "pibridge.dashboard"; `name` is the tab's friendly name.
std::string manifestJson(const std::string& appKey, const std::string& binaryPath, const std::string& iconPath,
                         const std::string& name);

} // namespace pibridge::overlay
