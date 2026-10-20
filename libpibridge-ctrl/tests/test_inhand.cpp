// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/inhand.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/inhand.h"
#include "pibridge/ctrl/report.h"
#include "pibridge_test.h"

using namespace pibridge::ctrl;
using namespace pibridge::ctrl::inhand;

namespace {
std::uint64_t still_for(InHand& d, std::uint64_t t0, double secs) {
    std::uint64_t t = t0;
    const auto n = static_cast<std::uint64_t>(secs * 500.0);
    for (std::uint64_t i = 0; i < n; i++) {
        t = t0 + i * 2000000;
        const auto j = static_cast<std::int16_t>(static_cast<std::int16_t>(i % 3) - 1);
        d.sample(t, {static_cast<std::int16_t>(-5 + j), j, -4},
                 {-17, static_cast<std::int16_t>(669 + j), -726});
    }
    return t;
}
} // namespace

TEST(modes_parse) {
    CHECK(parse_mode("when-held") == Mode::Auto);
    CHECK(parse_mode(" Always ") == Mode::Always);
    CHECK(parse_mode("off") == Mode::Off);
    CHECK(!parse_mode("x"));
    for (Mode m : {Mode::Auto, Mode::Always, Mode::Off}) {
        CHECK(parse_mode(mode_name(m)) == m);
    }
}

TEST(starts_down_and_a_press_picks_up) {
    InHand d;
    const std::uint64_t t = still_for(d, 1, 1.0);
    CHECK(!d.held(t));
    d.input(t, 0, 0, 0, 0, 0, 0);
    CHECK(!d.held(t));
    d.input(t + 4000000, 0, 0, 0, 0, report::BTN_A, 0);
    CHECK(d.held(t + 4000000));
    CHECK(d.last_change().first == Why::Input);
}

TEST(lying_on_the_trigger_is_not_holding_it) {
    InHand d;
    std::uint64_t t = 0;
    for (std::uint64_t i = 0; i < 2000; i++) {
        t = i * 4000000;
        d.input(t, 200, 0, 0, 0, 0, 0);
    }
    CHECK(!d.held(t));
}

TEST(silence_puts_it_down) {
    InHand d;
    d.input(0, 0, 0, 0, 0, 0, 0);
    d.input(1000000, 0, 0, 0, 0, report::BTN_B, 0);
    CHECK(d.held(1000000));
    CHECK(d.held(2000000000 - 600000000));
    CHECK(!d.held(3000000000));
    CHECK(d.last_change().first == Why::NoReports);
}

TEST(env_down_seconds) {
    const Config c;
    CHECK(c.down_windows * c.window_blocks * c.block_ms == 5000);
}

PIBRIDGE_TEST_MAIN()
