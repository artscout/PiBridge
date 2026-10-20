// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/demo.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/demo.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl;

namespace {
Input at(double s, bool right) {
    return demo_input(static_cast<std::uint64_t>(s * 1e9), right);
}
} // namespace

TEST(phases_of_the_cycle) {
    CHECK(at(0.5, false) == Input{});
    CHECK(std::fabs(at(2.0, false).grip - 1.0f) < 1e-6f);
    CHECK(at(2.0, false).trigger == 0.0f);
    CHECK(std::fabs(at(4.0, false).trigger - 1.0f) < 1e-6f);
    CHECK(at(4.0, false).grip == 0.0f);
    CHECK(at(5.5, false).flags == flags::TOUCH_BTN1);
    CHECK(at(6.2, false).flags == flags::TOUCH_BTN2);
    CHECK(at(6.7, false).flags == (flags::TOUCH_BTN2 | flags::CLICK_BTN2));
    const Input s = at(7.5, false);
    CHECK(s.flags == flags::TOUCH_STICK);
    CHECK(s.stick_x > 0.5f);
    CHECK(at(9.0, false) == Input{});
}

TEST(periodic) {
    for (double s : {0.3, 2.2, 4.7, 5.5, 6.9, 7.2, 9.9}) {
        CHECK(at(s, false) == at(s + DEMO_PERIOD_S * 7.0, false));
    }
}

TEST(right_hand_is_shifted) {
    for (double s : {0.0, 1.3, 3.7, 6.1, 8.8}) {
        CHECK(at(s + DEMO_RIGHT_OFFSET_S, false) == at(s, true));
    }
    CHECK(!(at(2.0, false) == at(2.0, true)));
}

TEST(values_stay_in_range) {
    for (int k = 0; k < 2000; k++) {
        const Input i = at(static_cast<double>(k) * 0.013, k % 2 == 0);
        CHECK(i.trigger >= 0.0f && i.trigger <= 1.0f && i.grip >= 0.0f && i.grip <= 1.0f);
        CHECK((i.flags & ~flags::ALL) == 0);
    }
}

PIBRIDGE_TEST_MAIN()
