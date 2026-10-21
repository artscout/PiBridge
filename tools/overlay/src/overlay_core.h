// SPDX-License-Identifier: MIT
// pibridge-overlay, the pure parts (no OpenVR, no I/O): a small fixed launcher panel drawn in code (a dark card with two
// buttons, ROOM SETUP and QUICK ROOM SETUP), the laser hit test, and the SteamVR application manifest. overlay.cpp
// uploads the panel to a small dashboard overlay ONCE (re-drawing only when the hovered button changes), and on a click
// writes a request file the app reads. There is no app-window embed and no per-frame upload (that crashed SteamVR).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::overlay {

// The panel is a few hundred pixels, not 2048: a small RGBA texture uploaded at most on a hover change.
inline constexpr std::uint32_t kPanelW = 512;
inline constexpr std::uint32_t kPanelH = 402;

enum class Button { None = -1, RoomSetup = 0, QuickRoomSetup = 1, RecordStart = 2, RecordStop = 3 };

// The overlay request file in $XDG_RUNTIME_DIR/pibridge; the app reads one line and acts on it itself.
inline constexpr char kRequestFileName[] = "overlay-request";
inline constexpr char kRequestRoomSetup[] = "room-setup";
inline constexpr char kRequestQuickRoomSetup[] = "quick-room-setup";
inline constexpr char kRequestRecordStart[] = "record-start";
inline constexpr char kRequestRecordStop[] = "record-stop";

// The request line for a button (""; for Button::None).
std::string requestFor(Button b);

// The button under a laser-mouse coordinate. OpenVR reports the cursor in GL space (origin bottom-left, range [0, w] x
// [0, h] because the overlay's mouse scale is the panel size); the panel's rows run top-first, so y is flipped here.
Button hitTest(float x, float y);

// The panel as RGBA8 (top row first, what SetOverlayRaw wants): a dark card with the two buttons, the one in `hovered`
// lit. kPanelW x kPanelH x 4 bytes.
std::vector<std::uint8_t> drawPanel(Button hovered);

// The SteamVR application manifest for the one dashboard overlay (is_dashboard_overlay). `binaryPath` and `iconPath`
// are absolute; `appKey` is e.g. "pibridge.dashboard"; `name` is the tab's friendly name.
std::string manifestJson(const std::string& appKey, const std::string& binaryPath, const std::string& iconPath,
                         const std::string& name);

} // namespace pibridge::overlay
