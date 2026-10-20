// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/shm.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/ctrl/track/shm.h"
#include "pibridge_test.h"

#include <cstdio>
#include <cstring>

using namespace pibridge::ctrl::track::shm;
using pibridge::ctrl::track::testing::tmp;

TEST(inhand_status) {
    const char* t = "pid=7\nmode=auto\nleft=held\nleft_why=motion\nright=down\nright_why=still\n";
    CHECK((parse_inhand(t, [](std::uint32_t p) { return p == 7; }) == std::array<bool, 2>{true, false}));
    CHECK((parse_inhand(t, [](std::uint32_t) { return false; }) == std::array<bool, 2>{true, true}));
    CHECK((parse_inhand("pid=7\nmode=off\nleft=off\nright=off\n", [](std::uint32_t) { return true; }) ==
           std::array<bool, 2>{false, false}));
    CHECK((parse_inhand("pid=7\nmode=always\nleft=shown\nright=held\n", [](std::uint32_t) { return true; }) ==
           std::array<bool, 2>{true, true}));
    CHECK((parse_inhand("garbage", [](std::uint32_t) { return true; }) == std::array<bool, 2>{true, true}));
}

TEST(write_read_roundtrip) {
    const std::string p = tmp("pose");
    auto w = PoseWriter::create(p);
    CHECK(w.has_value());
    if (!w) {
        return;
    }
    HandPose h;
    h.frame_ns = 123;
    h.publish_ns = 456;
    h.pos = {0.1f, -0.2f, -0.3f};
    h.quat = {0.0f, 0.0f, 0.0f, 1.0f};
    h.rms_px = 0.7f;
    h.leds = 11;
    h.cams = 3;
    h.aim_from_imu = {0.0f, 1.0f, 0.0f, 0.0f};
    (*w)->set(1, h);
    auto r = PoseReader::open(p);
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    const auto got = (*r)->read();
    CHECK(got && (*got)[1] == h);
    CHECK(got && (*got)[0].frame_ns == 0);
    const auto b0 = (*r)->read_all();
    CHECK(b0 && !b0->second.age_ns(1000)); // no heartbeat yet
    Beat b;
    b.beat_ns = 5'000;
    b.led_frames = 77;
    b.source_up = true;
    b.reopens = 2;
    (*w)->beat(b);
    const auto all = (*r)->read_all();
    CHECK(all && all->second == b);
    CHECK(all && all->first[1] == h); // the heartbeat leaves the poses alone
    CHECK(all && all->second.age_ns(6'000) == std::optional<std::uint64_t>(1'000));
    w->reset();
    std::remove(p.c_str());
}

// not in the Rust file: the bytes Monado reads (pimax_ctrltrack_shm.h)
TEST(layout_bytes) {
    const std::string p = tmp("layout");
    auto w = PoseWriter::create(p);
    HandPose h;
    h.frame_ns = 0x0102030405060708ULL;
    h.leds = 9;
    h.pos = {1.5f, 0.0f, 0.0f};
    (*w)->set(0, h);
    std::FILE* f = std::fopen(p.c_str(), "rb");
    unsigned char d[256] = {};
    CHECK(f != nullptr && std::fread(d, 1, 256, f) == 256);
    if (f != nullptr) {
        std::fclose(f);
    }
    CHECK(d[0] == 0x50 && d[1] == 0x58 && d[2] == 0x43 && d[3] == 0x54 && d[4] == 1);
    CHECK(d[8] % 2 == 0);
    CHECK(d[16] == 0x08 && d[23] == 0x01);
    CHECK(d[16 + 48] == 9);
    float x = 0;
    std::memcpy(&x, d + 32, 4);
    CHECK(x == 1.5f);
    w->reset();
    std::remove(p.c_str());
}

PIBRIDGE_TEST_MAIN()
