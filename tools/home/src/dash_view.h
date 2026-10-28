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
// Over a game (the dashboard overlay, Header::overGame) the bottom bar has the quick actions instead:
//   | what the last click did  [Record video]  [Quick room setup]  [Recenter]  [Close app] |
//
// Boxes are what can be clicked: "page:<n>", the settings' boxes (dash_settings.h), "view:window", "view:hide"; over a
// game "view:record", "view:roomquick", "view:recenter", "view:close".
#pragma once

#include "dash_settings.h"

#include <optional>
#include <string>
#include <vector>

namespace pibridge::home::dash {

inline constexpr int kW = 2400; // picture pixels
inline constexpr int kH = 1200;
inline constexpr double kWidthM = 1.9; // the panel's arc length; its height follows the picture's aspect (0.95 m)

struct Rect {
    double x = 0, y = 0, w = 0, h = 0;
    bool contains(double px, double py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

struct Box {
    enum class Style { Normal, Primary, Selected, Nav, NavSelected, Bar };
    std::string id;
    Rect r;
    std::string label;
    Style style = Style::Normal;
    bool enabled = true;
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
};

struct View {
    std::vector<Rect> cards; // the bars and the two columns (drawn first)
    std::vector<Box> boxes;
    std::vector<Label> labels;
    std::string hover;   // the box under the pointer ("" = none)
    std::string pressed; // the box the trigger went down on
};

View layout(Page page, const std::vector<Item>& items, const Header& h);
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
