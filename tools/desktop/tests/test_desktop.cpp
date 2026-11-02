// SPDX-License-Identifier: MIT
// The desktop link (common/pibridge_desktop.h) and the pointer -> input events of pibridge-desktop (input.h).
#include "input.h"
#include "pibridge_desktop.h"
#include "pibridge_test.h"

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

using namespace pibridge;
using namespace pibridge::desktopsvc;
using desktop::Pointer;

namespace {

std::string tempShm(const char* name) {
    const char* tmp = std::getenv("TMPDIR");
    return std::string(tmp != nullptr ? tmp : "/tmp") + "/pibridge-desktop-test-" + name + "-" + std::to_string(::getpid());
}

std::vector<desktop::MonitorInfo> twoMonitors() {
    desktop::MonitorInfo a, b;
    a.index = 0;
    a.logicalW = 2560;
    a.logicalH = 1440;
    a.width = 2560;
    a.height = 1440;
    b = a;
    b.index = 1;
    b.x = 2560;
    b.width = 1280; // a picture at half size: the pointer still lands on the logical point
    b.height = 720;
    return {a, b};
}

} // namespace

TEST(pointer_packs_and_unpacks) {
    for (const Pointer p : {Pointer{}, Pointer{true, 0, 0, 0, 0}, Pointer{true, 3, 3839, 2159, 5}, Pointer{false, 0, 0, 0, 1}}) {
        CHECK(desktop::unpackPointer(desktop::packPointer(p)) == p);
    }
    // off: the monitor and the place do not count
    CHECK(!desktop::unpackPointer(desktop::packPointer(Pointer{false, 2, 10, 10, 0})).on);
}

TEST(link_frames_layout_state_and_the_client_side) {
    const std::string path = tempShm("link");
    {
        desktop::Link svc, cli;
        CHECK(svc.open(path) && cli.open(path));
        CHECK(cli.monitors().empty() && cli.state() == desktop::State::Off);
        svc.setState(desktop::State::Running, "2 monitors");
        CHECK(cli.state() == desktop::State::Running && cli.text() == "2 monitors");
        std::vector<desktop::MonitorInfo> mons = twoMonitors();
        mons[0].name = "Monitor 1";
        mons[1].name = "Monitor 2";
        svc.setLayout(mons);
        const auto seen = cli.monitors();
        CHECK(seen.size() == 2 && seen[1].x == 2560 && seen[1].name == "Monitor 2" && seen[0].logicalW == 2560);
        // a BGRx picture of 2 x 1 (row stride 12) arrives as BGRA, alpha opaque
        const std::uint8_t bgrx[12] = {1, 2, 3, 0, 4, 5, 6, 0, 9, 9, 9, 9};
        CHECK(svc.writeFrame(1, bgrx, 2, 1, 12, desktop::Source::BGRX));
        std::vector<std::uint8_t> out;
        std::uint32_t w = 0, h = 0;
        const auto seq = cli.readFrame(1, 0, out, w, h);
        CHECK(seq && w == 2 && h == 1 && out.size() == 8);
        CHECK(out[0] == 1 && out[2] == 3 && out[3] == 255 && out[4] == 4 && out[7] == 255);
        CHECK(!cli.readFrame(1, *seq, out, w, h)); // nothing newer
        CHECK(!cli.readFrame(0, 0, out, w, h));    // monitor 1 has no picture yet
        // RGBx swaps to BGRA
        const std::uint8_t rgbx[4] = {10, 20, 30, 0};
        CHECK(svc.writeFrame(0, rgbx, 1, 1, 4, desktop::Source::RGBX));
        CHECK(cli.readFrame(0, 0, out, w, h) && out[0] == 30 && out[1] == 20 && out[2] == 10 && out[3] == 255);
        CHECK(!svc.writeFrame(0, rgbx, desktop::kMaxW + 1, 1, 4, desktop::Source::RGBX)); // too big
        // the overlay's side
        cli.clientUpdate(0b10, Pointer{true, 1, 7, 8, 1}, 240);
        CHECK(svc.header()->want.load() == 0b10 && svc.header()->wheel.load() == 240);
        CHECK(desktop::unpackPointer(svc.header()->pointer.load()) == (Pointer{true, 1, 7, 8, 1}));
        CHECK(desktop::fresh(svc.header()->clientBeatNs.load(), desktop::monoNs()));
    }
    std::filesystem::remove(path);
}

TEST(pointer_becomes_moves_buttons_and_wheel_steps) {
    InputDiff d;
    const auto mons = twoMonitors();
    // the first look moves the pointer; the wheel's old total gives no steps
    auto ev = d.update(Pointer{true, 0, 100, 200, 0}, 1200, mons);
    CHECK(ev.size() == 1 && ev[0].kind == Event::Kind::Move && ev[0].monitor == 0);
    CHECK(std::abs(ev[0].x - 100.5) < 1e-9 && std::abs(ev[0].y - 200.5) < 1e-9);
    // the same place: nothing
    CHECK(d.update(Pointer{true, 0, 100, 200, 0}, 1200, mons).empty());
    // the trigger: a press, then a release
    ev = d.update(Pointer{true, 0, 100, 200, 1}, 1200, mons);
    CHECK(ev.size() == 1 && ev[0].kind == Event::Kind::Button && ev[0].button == kBtnLeft && ev[0].down);
    ev = d.update(Pointer{true, 0, 100, 200, 0}, 1200, mons);
    CHECK(ev.size() == 1 && !ev[0].down);
    // a half-size picture on monitor 2: the logical point doubles
    ev = d.update(Pointer{true, 1, 10, 20, 0}, 1200, mons);
    CHECK(ev.size() == 1 && ev[0].monitor == 1 && std::abs(ev[0].x - 21.0) < 1e-9 && std::abs(ev[0].y - 41.0) < 1e-9);
    // the wheel: two notches up = two steps up (negative), a part notch waits
    ev = d.update(Pointer{true, 1, 10, 20, 0}, 1200 + 240 + 60, mons);
    CHECK(ev.size() == 1 && ev[0].kind == Event::Kind::Wheel && ev[0].steps == -2);
    ev = d.update(Pointer{true, 1, 10, 20, 0}, 1200 + 240 + 120, mons);
    CHECK(ev.size() == 1 && ev[0].steps == -1);
    ev = d.update(Pointer{true, 1, 10, 20, 0}, 1200, mons);
    CHECK(ev.size() == 1 && ev[0].steps == 3); // down
    // held, then off every monitor: released
    d.update(Pointer{true, 1, 10, 20, 0b011}, 1200, mons);
    ev = d.update(Pointer{}, 1200, mons);
    CHECK(ev.size() == 2 && ev[0].button == kBtnLeft && !ev[0].down && ev[1].button == kBtnRight && !ev[1].down);
    CHECK(d.update(Pointer{}, 1200, mons).empty());
    // a monitor that is not there counts as off
    CHECK(d.update(Pointer{true, 3, 1, 1, 1}, 1200, mons).empty());
}

PIBRIDGE_TEST_MAIN()
