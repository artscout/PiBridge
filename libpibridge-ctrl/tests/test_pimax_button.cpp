// SPDX-License-Identifier: MIT
// The controller's Menu button (report byte 47 bit 2, live 2026-10-19 on both hands): PiBridge decodes it as BTN_APP_MENU
// (the Monado driver's menu / system click). Byte 48 bit 0 (the Rust BTN_MENU) is the Pimax button (recenter). With
// PIBRIDGE_CTRL_PARITY=1 the decoder is the Rust one (bit 2 masked).
#include "pibridge/hmd/proto.h"
#include "pibridge_test.h"

using namespace pibridge::hmd;

namespace {
Report report(std::uint8_t slot, std::uint8_t b47, std::uint8_t b48) {
    Report r{};
    r[0] = 0x53;
    r[1] = 0x32;
    r[2] = 0x08;
    r[3] = slot;
    r[47] = b47;
    r[48] = b48;
    return r;
}
} // namespace

TEST(menu_and_pimax_buttons_both_hands) {
    for (const std::uint8_t slot : {SLOT_LEFT, SLOT_RIGHT}) {
        const auto c = decode_controller(report(slot, 0x04, 0x00));
        CHECK(c && c->buttons == BTN_APP_MENU);
        const auto m = decode_controller(report(slot, 0x00, 0x01));
        CHECK(m && m->buttons == BTN_MENU);
        const auto all = decode_controller(report(slot, 0x0f, 0x01));
        CHECK(all && all->buttons == (BTN_A | BTN_B | BTN_APP_MENU | BTN_STICK | BTN_MENU));
    }
}

PIBRIDGE_TEST_MAIN()
