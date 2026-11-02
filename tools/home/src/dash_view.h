// SPDX-License-Identifier: MIT
// The VR dashboard's layout (pure: no Qt, no runtime; dash_paint.h draws it, xrhome.cpp points at it). One curved
// panel in front of you (panel.h's cylinder, kWidthM wide), in picture pixels kW x kH:
//
//   +--------------------------------------------------------------------------------+
//   | top bar: PiBridge, the runtime and backend, the room, the clock                 |
//   +-------------+------------------------------------------------------------------+
//   | pages       | the page's rows: a label and its timing tag (NOW / AFTER RESTART)  |
//   |  Headset    | on the left, the control on the right: [-] value [+], a row of     |
//   |  Tracking   | choices, or a button                                               |
//   |  Controllers|                                                                    |
//   |  Room       |                                                                    |
//   +-------------+------------------------------------------------------------------+
//   | bottom bar: what the last click did          [PiBridge window]  [Hide]          |
//   +--------------------------------------------------------------------------------+
//
// Over a game or PiBridge World (the dashboard overlay, Header::overGame) the bottom bar has the quick actions instead,
// each with its icon:
//   | what the last click did  [Desktop] [Record video]  [Quick room setup]  [Recenter]  [Close app] |
//
// With the desktop open (deskLayout) the panel is only a bar under the monitors, kBarH high:
//   | the desktop's state words                                                                   |
//   | [Dashboard]  [1] [2] [All]      [Record video] [Quick room setup] [Recenter] [Close app]       |
// SteamVR's Desktop tab: [Desktop] (shows / hides the monitors) instead of [Dashboard], and the desktop's Zoom and
// Distance as [-] value [+] in the top row ("desk:zoom:-", "desk:zoom:+", "desk:dist:-", "desk:dist:+").
//
// Boxes are what can be clicked: "page:<n>", the settings' boxes (dash_settings.h), "view:window", "view:hide"; over a
// game "view:desktop", "view:record", "view:roomquick", "view:recenter", "view:close"; on the desktop's bar
// "desk:dash", "desk:mon:<i>", "desk:all" and the same "view:*" actions.
#pragma once

#include "dash_settings.h"

#include <optional>
#include <string>
#include <vector>

namespace pibridge::home::dash {

inline constexpr int kW = 2400; // picture pixels
inline constexpr int kH = 1200;
inline constexpr int kBarH = 300; // the desktop's bar: kW x kBarH
inline constexpr double kWidthM = 1.9; // the panel's arc length; its height follows the picture's aspect (0.95 m)

struct Rect {
    double x = 0, y = 0, w = 0, h = 0;
    bool contains(double px, double py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

/// A picture drawn in code left of a box's label (the quick actions).
enum class Icon { None, Desktop, Dashboard, Record, Stop, Room, Recenter, Close };

struct Box {
    enum class Style { Normal, Primary, Selected, Nav, NavSelected, Bar };
    std::string id;
    Rect r;
    std::string label;
    Style style = Style::Normal;
    bool enabled = true;
    Icon icon = Icon::None;
};

struct Label {
    enum class Kind { Title, Info, Clock, RowLabel, Value, Tag, TagPending, Body, Status };
    Rect r;
    std::string text;
    Kind kind = Kind::Body;
};

/// What the bars show.
struct Header {
    std::string backend;  // "Monado"
    std::string runtime;  // the OpenXR runtime's name
    std::string room;     // the active room ("" = none)
    std::string clock;    // "21:07"
    std::string message;  // what the last click did
    bool windowAvailable = false; // the PiBridge window can be shown (the app is linked)
    bool overGame = false;        // the dashboard overlay over a game: the quick actions in the bottom bar
    bool roomQuickAvailable = false; // over a game: the quick room setup can start
    bool recenterAvailable = false;  // over a game: the driver takes a recenter (Monado)
    std::string closeName;           // over a game: the app "Close app" ends ("" = none known: the box is off)
    bool recordAvailable = false;    // over a game: pibridge-record was found
    bool recording = false;          // over a game: a recording runs (the box says Stop recording)
    bool desktopAvailable = false;   // over a game: pibridge-desktop was found (the Desktop box)
    bool closeShown = true;          // over a game: the Close app box is there (not over PiBridge World)
};

/// The desktop's bar (deskLayout).
struct DeskBar {
    std::string words;            // the desktop's state ("2 monitors", "not allowed yet: ...")
    std::uint32_t monitors = 0;   // how many there are
    std::uint32_t mask = 0;       // the ones shown (a bit each)
    bool dashShown = true;        // the Dashboard box (not in SteamVR's dashboard, which is around it already)
    bool toggleShown = false;     // a Desktop box that shows / hides the monitors ("desk:toggle"; SteamVR's tab)
    bool on = true;               // the monitors are shown (the toggle is lit; 1 / 2 / All only then)
    bool zoomShown = false;       // the desktop's Zoom and Distance as steppers in the top row
    double zoom = 1.0, distance = 0.0;
};

struct View {
    int w = kW, h = kH;      // the picture's pixels
    std::vector<Rect> cards; // the bars and the two columns (drawn first)
    std::vector<Box> boxes;
    std::vector<Label> labels;
    std::string hover;   // the box under the pointer ("" = none)
    std::string pressed; // the box the trigger went down on
};

View layout(Page page, const std::vector<Item>& items, const Header& h);
/// The desktop's bar: kW x kBarH, the monitors' choice and the quick actions of `h`.
View deskLayout(const Header& h, const DeskBar& d);
/// Everything a picture of `v` depends on, as one string: the picture is painted again only when it changes.
std::string signature(const View& v);
/// The enabled box at picture pixel (px, py).
std::optional<std::string> hit(const View& v, double px, double py);
/// "page:2" -> Page::Controllers.
std::optional<Page> pageOf(const std::string& box);

/// A click is the trigger going down and up on the same box (moving off it before the release cancels).
class Clicker {
public:
    /// One frame: the box under the pointer and whether the trigger is held; the box clicked now, if one.
    std::optional<std::string> update(const std::optional<std::string>& over, bool pressed);
    const std::optional<std::string>& downOn() const { return downOn_; }

private:
    bool was_ = false;
    std::optional<std::string> downOn_;
};

} // namespace pibridge::home::dash
