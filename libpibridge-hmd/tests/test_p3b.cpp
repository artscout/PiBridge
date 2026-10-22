// SPDX-License-Identifier: MIT
// The original Crystal (P3B): every frame PiBridge may send is the captured one byte for byte (Windows capture
// 2026-10-22), replies decode, and the headset is found in a fake sysfs.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/p3b.h"

#include <cmath>
#include <fstream>

namespace hmd = pibridge::hmd;
namespace discovery = pibridge::hmd::discovery;
using namespace pibridge::hmd::p3b;
namespace fs = std::filesystem;

static std::string hex_used(const Report& r) {
    std::size_t n = r.size();
    while (n > 0 && r[n - 1] == 0) {
        n--;
    }
    std::string s;
    char b[3];
    for (std::size_t i = 0; i < n; i++) {
        std::snprintf(b, sizeof b, "%02x", r[i]);
        s += b;
    }
    return s;
}

TEST(every_command_is_the_captured_frame) {
    for (const Cmd c : {Cmd::ScreenOn, Cmd::KeepRuntime, Cmd::PcVideo120, Cmd::ConnectState, Cmd::GetLighthouseEnable,
                        Cmd::GetLighthouseSupport, Cmd::LighthouseOn, Cmd::LighthouseOff, Cmd::LighthouseTakeEffect,
                        Cmd::Shutdown, Cmd::TobiiEnable}) {
        const auto r = encode(command(c));
        CHECK(r.has_value());
        if (r) {
            CHECK(hex_used(*r) == captured_hex(c));
        }
        CHECK(!name_of(c).empty());
    }
}

TEST(replies_decode_and_stale_bytes_are_cut) {
    // the property read's reply, as captured (20:30:32.494): value "1"
    Report r{};
    const std::string_view h = "3b2100000001000000706572736973742e7379732e706d782e6c69676874686f7573652e656e61626c6531";
    for (std::size_t i = 0; i < h.size() / 2; i++) {
        r[i] = static_cast<std::uint8_t>(std::stoi(std::string(h.substr(2 * i, 2)), nullptr, 16));
    }
    const auto d = decode(r);
    CHECK(d.has_value());
    CHECK(d && d->type == type::GETPROP && d->name == "persist.sys.pmx.lighthouse.enable" && d->value == "1");
    // shutDown's reply says 4 bytes ("true") and the reused buffer goes on: the value stays "true"
    Rpc sd{type::CALL, "shutDown", "true"};
    auto e = encode(sd).value();
    e[9 + 8 + 4] = 'c'; // stale byte after the value
    const auto d2 = decode(e);
    CHECK(d2 && d2->value == "true");
    // setTobiiEyeDeviceEnable's reply as captured (20:28:34.978): "true", then the reused buffer
    // ("usVpsRequest{..." of an earlier, longer reply)
    Report te{};
    const std::string_view th = "1c1700000004000000736574546f626969457965446576696365456e61626c6574727565757356707352";
    for (std::size_t i = 0; i < th.size() / 2; i++) {
        te[i] = static_cast<std::uint8_t>(std::stoi(std::string(th.substr(2 * i, 2)), nullptr, 16));
    }
    const auto d3 = decode(te);
    CHECK(d3 && d3->type == type::CALL && d3->name == "setTobiiEyeDeviceEnable" && d3->value == "true");
    // not an RPC: the tracking stream
    Report t{};
    t[0] = 0x42;
    t[1] = 0x04;
    t[2] = 0x4c;
    t[3] = 0xfb;
    CHECK(!decode(t));
}

TEST(finds_the_crystal_hidraw_in_a_fake_sysfs) {
    const auto root = testutil::tmpdir("sysfs-p3b");
    const fs::path hid = root / "sys/devices/pci/usb2/2-1/2-1:1.2/0003:34A4:0012.0007";
    fs::create_directories(hid);
    std::ofstream(hid / "uevent") << "DRIVER=hid-generic\nHID_ID=0003:000034A4:00000012\nHID_NAME=PIMAX\n";
    const fs::path cls = root / "sys/class/hidraw/hidraw9";
    fs::create_directories(cls);
    fs::create_directory_symlink(hid, cls / "device");
    discovery::Roots r;
    r.hidraw_class = root / "sys/class/hidraw";
    r.dev = root / "dev";
    const auto p = find_hidraw(r);
    CHECK(p && *p == root / "dev/hidraw9");
}

static std::vector<std::uint8_t> from_hex(std::string_view h) {
    std::vector<std::uint8_t> v;
    for (std::size_t i = 0; i + 1 < h.size(); i += 2) {
        v.push_back(static_cast<std::uint8_t>(std::stoi(std::string(h.substr(i, 2)), nullptr, 16)));
    }
    v.resize(1024, 0);
    return v;
}

TEST(head_and_controller_reports_of_the_capture_decode) {
    // head report 0x31 at 20:28:10.000 (cold take): still headset, battery 73 %, IPD 63.1875, volume 53
    const auto h = decode_head(from_hex(
        "311695c6b705000000ba997d3d7d20dcbb639a9f3d236aeb3d33d772be65c82cbf2f70303fd8897d3d114bdbbb83989f3d34e0b93bd16"
        "3debbe4a6273bd00d4c39a5c549bd4bf2e9bbb59de43ea9fa33bf80dc3dbf3290034185d652c1910e04c206fe3ec5904b0cc57b2113444"
        "901000003e95ae1b705000000e653040000000000803f3500000002010022"));
    CHECK(h.has_value());
    if (h) {
        CHECK(h->tracked && h->tracking == 3);
        CHECK(std::abs(h->pose.position[0] - 0.062f) < 0.002f && std::abs(h->pose.position[2] - 0.078f) < 0.002f);
        const auto& q = h->pose.orientation;
        CHECK(std::abs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0f) < 1e-5f);
        CHECK(h->battery == 73);
        CHECK(std::abs(h->ipd_mm - 63.1875f) < 1e-4f);
        CHECK(h->volume == 53 && h->buttons == 0);
        CHECK(h->pose.t_ns > 20'000'000'000ull); // ~24.56 s since the headset booted
    }
    // left controller 0x42 (connected, no pose yet) at 20:28:30.000, battery 50 %
    const auto c = decode_controller(from_hex(
        "42044cfb3b0100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
        "00000000000032"));
    CHECK(c && c->side == 0 && c->state == 4 && c->battery == 50);
    CHECK(!decode_head(from_hex("42044cfb3b01")) && !decode_controller(from_hex("311695c6b705")));
}

TEST(controller_inputs_of_a_linux_recording_decode) {
    // 2026-10-22 on Linux, each control pressed in turn: the right stick pushed right and a little down
    const auto st = decode_controller(from_hex(
        "7302f1f4f9bc000000fe049c3e97a7b9be4180b5bd00d77e3c7bc38bbe54b09f3d416f753f2e20769d1f2406aea3384138dcad3ca96eb7"
        "7fe50000001830"));
    CHECK(st && st->side == 1 && st->state == 7 && st->battery == 48);
    CHECK(st && st->stick[0] == 1.0f && std::abs(st->stick[1] + 27.0f / 127.0f) < 1e-6f);
    CHECK(st && st->buttons == 0 && st->touches == (TOUCH_STICK | TOUCH_TRIGGER));
    // the left trigger pulled all the way
    const auto tr = decode_controller(from_hex(
        "72e26cefdeb3000000ec6e983ddee0abbe138392be123705be8784b03ed8a7dfbc07e26dbf252a14a93f1b403b36318cace8255bb5c9b9"
        "0000ff00001025"));
    CHECK(tr && tr->side == 0 && tr->trigger == 1.0f && tr->grip == 0 && tr->touches == TOUCH_TRIGGER);
    // the right Pimax button
    const auto pm = decode_controller(from_hex(
        "73fa1bc8bcbd00000004038b3e4021cdbe2a8889bdd07e73bcf07d98be41e3a53d8278733ff9256698bb245628b935c4aa43226fb746"
        "3500000000404030"));
    CHECK(pm && pm->buttons == BUTTON_PIMAX && pm->touches == TOUCH_PIMAX);
    CHECK(half_to_float(0x5422) == 66.125f && half_to_float(0x53df) == 62.96875f);
}

PIBRIDGE_TEST_MAIN()
