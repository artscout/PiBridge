// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/report.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/report.h"

#include "pibridge/ctrl/rust.h"

namespace pibridge::ctrl {

Input input_from_report(const Report& r) {
    const auto bit = [](bool set, std::uint32_t flag) { return set ? flag : 0u; };
    using namespace report;
    return Input{
        static_cast<float>(r.trigger) / 255.0f,
        static_cast<float>(r.grip) / 255.0f,
        rs::clampf(static_cast<float>(r.stick_x) / 127.0f, -1.0f, 1.0f),
        rs::clampf(static_cast<float>(r.stick_y) / 127.0f, -1.0f, 1.0f),
        bit((r.touch & TOUCH_A) != 0, flags::TOUCH_BTN1) | bit((r.touch & TOUCH_B) != 0, flags::TOUCH_BTN2) |
            bit((r.touch & TOUCH_STICK) != 0, flags::TOUCH_STICK) | bit((r.buttons & BTN_A) != 0, flags::CLICK_BTN1) |
            bit((r.buttons & BTN_B) != 0, flags::CLICK_BTN2) | bit((r.buttons & BTN_STICK) != 0, flags::CLICK_STICK) |
            bit((r.buttons & BTN_MENU) != 0, flags::CLICK_MENU),
    };
}

} // namespace pibridge::ctrl
