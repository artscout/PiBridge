// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/report.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/report.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl;
using namespace pibridge::ctrl::report;

TEST(idle_report_is_the_default_input) {
    CHECK(input_from_report(Report{}) == Input{});
}

TEST(analog_values_are_normalised) {
    Report r;
    r.trigger = 255;
    r.grip = 51;
    r.stick_x = 127;
    r.stick_y = -128;
    const Input i = input_from_report(r);
    CHECK(i.trigger == 1.0f);
    CHECK(std::fabs(i.grip - 0.2f) < 1e-6f);
    CHECK(i.stick_x == 1.0f);
    CHECK(i.stick_y == -1.0f);
}

TEST(every_bit_maps_to_its_own_flag) {
    const struct {
        std::uint32_t buttons;
        std::uint8_t touch;
        std::uint32_t want;
    } cases[] = {
        {BTN_A, 0, flags::CLICK_BTN1},       {BTN_B, 0, flags::CLICK_BTN2},  {BTN_STICK, 0, flags::CLICK_STICK},
        {BTN_MENU, 0, flags::CLICK_MENU},    {0, TOUCH_A, flags::TOUCH_BTN1}, {0, TOUCH_B, flags::TOUCH_BTN2},
        {0, TOUCH_STICK, flags::TOUCH_STICK},
    };
    for (const auto& c : cases) {
        Report r;
        r.buttons = c.buttons;
        r.touch = c.touch;
        CHECK(input_from_report(r).flags == c.want);
    }
}

TEST(unrelated_bits_are_ignored) {
    Report r;
    r.buttons = 0x04 | 0x20 | 0x80000000u;
    r.touch = 0x10 | 0x04 | 0xe0;
    CHECK(input_from_report(r).flags == 0);
}

PIBRIDGE_TEST_MAIN()
