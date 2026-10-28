// SPDX-License-Identifier: MIT
// pibridge-overlay, the pure parts (no OpenVR, no I/O). SteamVR gives no API to add buttons to its own dashboard bar or
// quick menu, but every dashboard overlay gets its own button (its thumbnail + its name) in that bar. So the overlay is
// THREE small dashboard overlays -- Room Setup, Quick Room Setup, Record Video -- each one button in the SteamVR
// dashboard bar that opens one big button (no menu). This file draws those pages and the bar icons in code, does the
// laser hit test and the request lines, parses the state the buttons follow, and the SteamVR application manifest.
// overlay.cpp uploads a page / an icon only when what it shows changes (no per-frame upload, that crashed SteamVR), and
// on a click writes a request file the app reads and acts on with its own machinery.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::overlay {

// A page is a few hundred pixels, not 2048: a small RGBA texture uploaded at most on a hover / state change.
inline constexpr std::uint32_t kPanelW = 520;
inline constexpr std::uint32_t kPanelH = 220;
// The bar icon (the dashboard thumbnail), square RGBA with a transparent background.
inline constexpr std::uint32_t kIconSize = 128;

// One button in the SteamVR dashboard bar = one dashboard overlay.
enum class Tab { Room, QuickRoom, Record };
inline constexpr std::array<Tab, 3> kTabs{Tab::Room, Tab::QuickRoom, Tab::Record};

// What the buttons show, from the real state (overlay.cpp reads it; nothing here is the overlay's own guess).
struct TabState {
    bool recording = false;        // the capture control file says active AND a pibridge-record process is alive
    bool roomSetupRunning = false; // a `pibridge-roomsetup --map` process is alive (the in-headset room setup)
    bool roomPending = false;      // a room-setup request was just sent and no setup has appeared yet (a few seconds)
    bool recordPending = false;    // a record start/stop request was just sent and the state has not flipped yet
    bool operator==(const TabState&) const = default;
};

// The dashboard overlay key of a tab (pibridge.dashboard is the Room Setup one: the key SteamVR may already remember as
// the last opened external overlay).
const char* tabKey(Tab t);
// The tab's name, which SteamVR shows under its icon in the dashboard bar ("Recording" while a recording runs).
std::string tabTitle(Tab t, const TabState& s);
// The big button can be pressed now: not while a room setup runs or was just asked for (no second instance), not while
// a record request is on its way.
bool buttonEnabled(Tab t, const TabState& s);
// The big button's label and the line under it (uppercase: the built-in font has no lower case).
std::string buttonLabel(Tab t, const TabState& s);
std::string statusLine(Tab t, const TabState& s);

// The overlay request file in $XDG_RUNTIME_DIR/pibridge; the app reads one line and acts on it itself.
inline constexpr char kRequestFileName[] = "overlay-request";
inline constexpr char kRequestRoomSetup[] = "room-setup";
inline constexpr char kRequestQuickRoomSetup[] = "quick-room-setup";
inline constexpr char kRequestRecordStart[] = "record-start";
inline constexpr char kRequestRecordStop[] = "record-stop";

// The request line for a click on a tab's big button ("" when the button is disabled). The record button sends start
// or stop by the real recording state.
std::string requestFor(Tab t, const TabState& s);

// The laser is on the big button. OpenVR reports the cursor in GL space (origin bottom-left, range [0, w] x [0, h]
// because the overlay's mouse scale is the page size); the page's rows run top-first, so y is flipped here.
bool hitButton(float x, float y);

// A tab's page as RGBA8 (top row first, what SetOverlayRaw wants): the dark card with the tab's header, the one big
// button (lit when `hot` and enabled; red while recording) and the status line. kPanelW x kPanelH x 4.
std::vector<std::uint8_t> drawPage(Tab t, const TabState& s, bool hot);
// A tab's bar icon as RGBA8, kIconSize square: a room outline, the quick circle, or the record dot (a white ring when
// not recording, a solid red dot while recording). The room icons turn amber while a room setup runs.
std::vector<std::uint8_t> drawIcon(Tab t, const TabState& s);

// The capture control file's text (pibridge_record_capture_control.h): true when its `active` line is non-zero.
bool captureActive(const std::string& controlText);
// A /proc/<pid>/cmdline (NUL-separated) is the in-headset room setup: argv[0]'s basename is pibridge-roomsetup and
// --map is among the arguments (headtrack's short `pibridge-roomsetup --vr-standing` and --recenter are not).
bool isRoomSetupCmdline(const std::string& cmdline);

// SteamVR's dashboard bar shows only `dashboard/maxInlineTabs` (default 2) external overlays inline and folds the rest
// into a "N more" button; three PiBridge buttons need more. The value to write, or 0 when `current` (0 = unset) is
// already enough.
inline constexpr int kInlineTabsWanted = 5; // our three + room for two other overlay apps
int inlineTabsToSet(int current);

// The in-game recenter ring texture (square, RGBA8, transparent background): a faint full track with a bright arc
// filling clockwise from the top to 2*pi*progress. Drawn in front of the HMD while the recenter button is held
// (headtrack publishes the progress 0..1); the recenter fires when it completes.
inline constexpr std::uint32_t kRingSize = 256;
inline constexpr int kRingGrid = 5;                      // fill-level sprite sheet is kRingGrid x kRingGrid cells
inline constexpr int kRingFrames = kRingGrid * kRingGrid; // 25 fill levels, 0..1 row-major
std::vector<std::uint8_t> drawRing(double progress);
// One SQUARE RGBA sprite sheet, kRingGrid x kRingGrid ring images (fill 0..1 row-major). Square so the overlay quad
// stays 1:1 and each 1:1 cell shows undistorted. Uploaded ONCE; the overlay animates by selecting a cell with
// SetOverlayTextureBounds -- no per-frame re-upload (which flickered). Side = kRingGrid * kRingSize.
std::vector<std::uint8_t> drawRingSheet();

// The SteamVR application manifest for the dashboard overlay app (is_dashboard_overlay). `binaryPath` and `iconPath`
// are absolute; `appKey` is e.g. "pibridge.dashboard"; `name` is the app's friendly name.
std::string manifestJson(const std::string& appKey, const std::string& binaryPath, const std::string& iconPath,
                         const std::string& name);

} // namespace pibridge::overlay
