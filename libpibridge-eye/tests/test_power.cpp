// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/power.rs tests (MIT OR Apache-2.0). Fake sysfs trees only; nothing is opened.
#include "pibridge/eye/power.h"

#include "pibridge_test.h"
#include "testutil.h"

#include <string>
#include <vector>

using namespace pibridge::eye;
using namespace pibridge::eye::power;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

UsbId id(std::uint32_t n) {
    return UsbId{"1-5.2.4.1", n};
}

void put(const fs::path& p, const std::string& s) {
    testutil::write_bytes(p, std::span(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()));
}

struct RecordingHid final : HeadsetCommands {
    std::vector<std::string> sent;
    Result<void> send(std::string_view cmd) override {
        sent.emplace_back(cmd);
        return {};
    }
};

} // namespace

TEST(report_is_the_captured_bytes) {
    const auto r = enable_report();
    CHECK(r[0] == 0x58 && r[1] == 0x05 && r[2] == 0x3c && r[3] == 0x05 && r[4] == 0x00 && r[5] == 0x01);
    CHECK(std::all_of(r.begin() + 6, r.end(), [](std::uint8_t b) { return b == 0; }));
    // the HID framing rule (length byte = data length + 3) holds: data `00 01`
    CHECK(r[1] == 2 + 3);
}

TEST(the_enable_goes_out_as_the_named_allowlist_command) {
    RecordingHid hid;
    CHECK(send_enable(hid).has_value());
    CHECK(hid.sent == std::vector<std::string>{"eye_power_on"});
}

TEST(gate_waits_sends_once_per_enumeration) {
    EnableGate g;
    const auto t0 = Clock::now();
    CHECK(g.step(t0, std::nullopt, false) == Gate::idle());
    CHECK(g.step(t0, id(21), true) == Gate::idle());
    CHECK(g.step(t0, id(21), false) == Gate::waiting());
    CHECK(g.step(t0 + 3s, id(21), false) == Gate::waiting());
    CHECK(g.step(t0 + 6s, id(21), false) == Gate::send(id(21)));
    for (int s = 7; s < 30; s++) {
        CHECK(g.step(t0 + std::chrono::seconds(s), id(21), false) == Gate::already_sent());
    }
    // chip appeared then vanished again on the same enumeration: still no second report
    CHECK(g.step(t0 + 31s, id(21), true) == Gate::idle());
    CHECK(g.step(t0 + 40s, id(21), false) == Gate::already_sent());
    // the headset re-enumerated (new devnum): one more report after the 5 s grace
    CHECK(g.step(t0 + 50s, id(30), false) == Gate::waiting());
    CHECK(g.step(t0 + 56s, id(30), false) == Gate::send(id(30)));
}

TEST(gate_restarts_the_grace_when_the_headset_goes_away) {
    EnableGate g;
    const auto t0 = Clock::now();
    CHECK(g.step(t0, id(5), false) == Gate::waiting());
    CHECK(g.step(t0 + 4s, std::nullopt, false) == Gate::idle());
    CHECK(g.step(t0 + 6s, id(6), false) == Gate::waiting());
    CHECK(g.step(t0 + 12s, id(6), false) == Gate::send(id(6)));
}

TEST(sysfs_discovery_on_a_fake_tree) {
    const auto dir = testutil::temp_dir("power");
    const auto usb = dir / "usb";
    const auto mk = [&](const char* name, const char* v, const char* p, const char* devnum) {
        const auto d = usb / name;
        fs::create_directories(d);
        put(d / "idVendor", std::string(v) + "\n");
        put(d / "idProduct", std::string(p) + "\n");
        put(d / "devnum", std::string(devnum) + "\n");
    };
    mk("1-5.2.4.1", "34a4", "0044", "21");
    mk("1-5.2.4.3.3", "34a4", "0043", "24");
    CHECK(headset(usb) == std::optional<UsbId>(id(21)));
    CHECK(!eye_chip_present(usb));
    mk("1-5.2.4.3.4", "2104", "0561", "27");
    CHECK(eye_chip_present(usb));
    // hidraw: interface 0 of 0044 is chosen, the bridge and other interfaces are not
    const auto cls = dir / "class";
    const auto ifdir = [&](const char* iface) {
        const auto p = dir / "devices" / (std::string("if") + iface);
        fs::create_directories(p / "hid");
        put(p / "bInterfaceNumber", std::string(iface) + "\n");
        return p / "hid";
    };
    const auto link = [&](const char* n, const fs::path& target, const char* hid_id) {
        const auto e = cls / n;
        fs::create_directories(e);
        put(target / "uevent", std::string("HID_ID=") + hid_id + "\nHID_NAME=x\n");
        fs::create_directory_symlink(target, e / "device");
    };
    const auto h0 = ifdir("00");
    const auto h1 = ifdir("01");
    const auto hb = dir / "devices/bridge/hid";
    fs::create_directories(hb);
    put(dir / "devices/bridge/bInterfaceNumber", "00\n");
    link("hidraw1", h1, "0003:000034A4:00000044");
    link("hidraw2", hb, "0003:000034A4:00000043");
    link("hidraw3", h0, "0003:000034A4:00000044");
    CHECK(headset_command_node(cls, "/dev") == std::optional<fs::path>("/dev/hidraw3"));
    fs::remove_all(dir);
}

TEST(the_crystal_super_is_a_headset_too) {
    const auto dir = testutil::temp_dir("power-cs");
    const auto d = dir / "3-1";
    fs::create_directories(d);
    put(d / "idVendor", "34a4\n");
    put(d / "idProduct", "0040\n");
    put(d / "devnum", "9\n");
    CHECK(headset(dir) == std::optional<UsbId>(UsbId{"3-1", 9}));
    fs::remove_all(dir);
}

PIBRIDGE_TEST_MAIN()
