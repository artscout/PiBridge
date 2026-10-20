// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/report.rs (MIT OR Apache-2.0).
// A decoded controller report (libpimax pimax_controller_report) -> Input.
#pragma once

#include "pibridge/ctrl/mapping.h"

#include <cstdint>

namespace pibridge::ctrl {

namespace report {
inline constexpr std::uint32_t BTN_B = 0x01;
inline constexpr std::uint32_t BTN_A = 0x02;
inline constexpr std::uint32_t BTN_STICK = 0x08;
inline constexpr std::uint32_t BTN_MENU = 0x10;
inline constexpr std::uint8_t TOUCH_A = 0x01;
inline constexpr std::uint8_t TOUCH_B = 0x02;
inline constexpr std::uint8_t TOUCH_STICK = 0x08;
} // namespace report

/// The fields of a controller report the hand needs, as the driver receives them.
struct Report {
    std::uint8_t trigger = 0;
    std::uint8_t grip = 0;
    std::int32_t stick_x = 0;
    std::int32_t stick_y = 0;
    std::uint32_t buttons = 0;
    std::uint8_t touch = 0;
};

/// Normalise a report: trigger and grip /255, stick /127 (clamped), buttons and touch bits to flags.
Input input_from_report(const Report& r);

} // namespace pibridge::ctrl
