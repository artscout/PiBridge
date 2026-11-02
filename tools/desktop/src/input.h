// SPDX-License-Identifier: MIT
// The overlay's pointer (common/pibridge_desktop.h) turned into the desktop's input events (pure; portal.cpp sends them
// through the RemoteDesktop portal): a move to the monitor's logical point when the pointer moves, a button press or
// release when a button bit changes, wheel steps from the running total (120 per notch; a part notch waits for the
// rest). Leaving every monitor releases what is held, so nothing stays pressed on the desktop.
#pragma once

#include "pibridge_desktop.h"

#include <cstdint>
#include <vector>

namespace pibridge::desktopsvc {

struct Event {
    enum class Kind { Move, Button, Wheel };
    Kind kind = Kind::Move;
    std::uint32_t monitor = 0; // Move: the monitor (its stream)
    double x = 0, y = 0;       // Move: logical pixels on that monitor
    int button = 0;            // Button: evdev code (BTN_LEFT 0x110, BTN_RIGHT 0x111, BTN_MIDDLE 0x112)
    bool down = false;
    int steps = 0;             // Wheel: + = down (the portal's sense)
    friend bool operator==(const Event&, const Event&) = default;
};

inline constexpr int kBtnLeft = 0x110;
inline constexpr int kBtnRight = 0x111;
inline constexpr int kBtnMiddle = 0x112;

class InputDiff {
public:
    /// One look at the overlay's side. `mons`: the monitors (their picture and logical sizes). The first call only takes
    /// the wheel's total as the start (no steps from an old total).
    std::vector<Event> update(const desktop::Pointer& p, std::int64_t wheel, const std::vector<desktop::MonitorInfo>& mons);
    /// Forget the state (a new session): held buttons are not released again.
    void reset() { *this = InputDiff(); }

private:
    bool started_ = false;
    desktop::Pointer last_;
    std::uint32_t held_ = 0; // the buttons the desktop has as pressed
    std::int64_t wheel_ = 0;
};

} // namespace pibridge::desktopsvc
