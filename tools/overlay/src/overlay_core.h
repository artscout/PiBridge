// SPDX-License-Identifier: MIT
// pibridge-overlay, the pure parts (no OpenVR, no I/O): a small fixed panel drawn in code. A compact in-VR page with a
// RECORDING section (VIEW / FPS / SIZE segmented rows and one START/STOP RECORDING button) and a ROOM section (ROOM
// SETUP, QUICK ROOM SETUP), the laser hit test, and the SteamVR application manifest. overlay.cpp uploads the panel to a
// small dashboard overlay ONCE (re-drawing only when the hovered target, the selection or the recording state changes),
// and on a click writes a request file the app reads. There is no app-window embed and no per-frame upload (that
// crashed SteamVR).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::overlay {

// The panel is a few hundred pixels, not 2048: a small RGBA texture uploaded at most on a hover / state change.
inline constexpr std::uint32_t kPanelW = 520;
inline constexpr std::uint32_t kPanelH = 540;

// The three segmented recording controls (a row each; cells 0..2).
enum class Control { View = 0, Fps = 1, Size = 2 };

// A laser target: a wide button, or one cell of one segmented control.
struct Hit {
    enum class Kind { None, Room, QuickRoom, Record, Seg };
    Kind kind = Kind::None;
    int control = -1; // Seg only: 0 = View, 1 = Fps, 2 = Size
    int cell = -1;    // Seg only: 0..2
    bool operator==(const Hit&) const = default;
};

// The current recording selection, as cell indices per control (the panel lights the chosen cell).
struct Selection {
    int view = 2; // 0 = left, 1 = right, 2 = both
    int fps = 1;  // 0 = 30, 1 = 60, 2 = 72
    int size = 1; // 0 = 720, 1 = 1080, 2 = full (no cap)
    bool operator==(const Selection&) const = default;
};

// The overlay request file in $XDG_RUNTIME_DIR/pibridge; the app reads one line and acts on it itself.
inline constexpr char kRequestFileName[] = "overlay-request";
inline constexpr char kRequestRoomSetup[] = "room-setup";
inline constexpr char kRequestQuickRoomSetup[] = "quick-room-setup";
inline constexpr char kRequestRecordStart[] = "record-start";
inline constexpr char kRequestRecordStop[] = "record-stop";
// A segment click sends "record-set <key> <value>" (key = view | fps | max_height), e.g. "record-set view both".
inline constexpr char kRequestRecordSetPrefix[] = "record-set ";

// The control-file key ("view" / "fps" / "max_height") and value ("left"/"right"/"both", "30"/"60"/"72", "720"/"1080"/
// "0") for a segmented control's cell. `value` returns "" for an out-of-range cell.
std::string segKey(int control);
std::string segValue(int control, int cell);

// The request line for a hit (""; for Hit::Kind::None). The record button sends start or stop by `recording`; a segment
// cell sends "record-set <key> <value>".
std::string requestFor(Hit h, bool recording);

// A Selection from the control file's values (defaults when a value is missing or unknown).
Selection selectionFrom(const std::string& view, int fps, int maxHeight);

// The target under a laser-mouse coordinate. OpenVR reports the cursor in GL space (origin bottom-left, range [0, w] x
// [0, h] because the overlay's mouse scale is the panel size); the panel's rows run top-first, so y is flipped here.
Hit hitTest(float x, float y);

// The panel as RGBA8 (top row first, what SetOverlayRaw wants): the dark card with the RECORDING rows (the chosen cell
// of each lit) and the ROOM buttons, the target in `hovered` lit; the record button reads STOP RECORDING when
// `recording`, else START RECORDING. kPanelW x kPanelH x 4.
std::vector<std::uint8_t> drawPanel(const Selection& sel, bool recording, Hit hovered);

// The SteamVR application manifest for the one dashboard overlay (is_dashboard_overlay). `binaryPath` and `iconPath`
// are absolute; `appKey` is e.g. "pibridge.dashboard"; `name` is the tab's friendly name.
std::string manifestJson(const std::string& appKey, const std::string& binaryPath, const std::string& iconPath,
                         const std::string& name);

} // namespace pibridge::overlay
