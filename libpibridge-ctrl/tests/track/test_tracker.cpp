// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/tracker.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/tracker.h"
#include "pibridge_test.h"

using namespace pibridge::ctrl::track;

TEST(thirty_failed_frames_send_a_slot_back_to_init) {
    FailCount f;
    for (int i = 0; i < 29; i++) {
        CHECK(!f.frame(0, false, 30));
    }
    CHECK(f.frame(0, false, 30));  // the 30th failure resets
    CHECK(!f.frame(0, false, 30)); // the count starts over
    // a good frame clears it; the other slot is independent
    for (int i = 0; i < 29; i++) {
        f.frame(1, false, 30);
    }
    CHECK(!f.frame(1, true, 30));
    CHECK(!f.frame(1, false, 30));
    // limit 0 = off
    for (int i = 0; i < 100; i++) {
        CHECK(!f.frame(0, false, 0));
    }
}

TEST(far_cold_starts_need_four_leds) {
    const TrackCfg c;
    CHECK(cold_start_ok(c, 0.6, 3));  // near: 3 LEDs are fine
    CHECK(cold_start_ok(c, 1.8, 3));  // exactly 1.8 m is near
    CHECK(!cold_start_ok(c, 1.81, 3));
    CHECK(cold_start_ok(c, 2.5, 4));
    CHECK(cold_start_ok(TrackCfg::baseline(), 5.0, 3)); // baseline has the rule off
}

PIBRIDGE_TEST_MAIN()
