// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/frame.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/cam/frame.h"
#include "pibridge_test.h"

using namespace pibridge::cam;
using pibridge::cam::testing::hex;
using pibridge::cam::testing::synth_frame;

// Real metadata rows (Linux capture 2026-10-13 19:59, frames 0 and 1 of raw_1280x832_yuyv.bin).
static const char* META_F0 =
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 86 77 36 e5 a9 01 00 00 00 ec f9 04 00 aa 28 aa "
    "28 aa 28 aa 28 c1 01 c1 01 c1 01 c1 01 16 01 16 01 16 01 16 01 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 00 00 00 00 00 00 00 00 3c 3d 3e 3f";
static const char* META_F1 =
    "01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 06 8b 84 e5 a9 01 00 00 00 ed f9 04 00 28 00 28 "
    "00 28 00 28 00 3e 00 3e 00 3e 00 3e 00 0a 00 0a 00 0a 00 0a 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 00 00 00 00 00 00 00 00 3c 3d 3e 3f";

TEST(real_metadata_tracking_frame) {
    const auto row = hex(META_F0);
    CHECK(row.size() == META_LEN);
    const auto m = *Meta::parse(row);
    CHECK(m.kind() == FrameKind::Tracking);
    CHECK(m.frame_type == 0x86);
    CHECK(m.ts_us_u32() == 2'850'371'191u);
    CHECK(m.ts_us == (1ull << 32) + 2'850'371'191ull);
    CHECK(m.counter == 0x0004'f9ecu);
    CHECK(static_cast<std::uint16_t>(m.counter) == 63980);
    CHECK((m.exposure == std::array<std::uint16_t, 4>{0x28aa, 0x28aa, 0x28aa, 0x28aa}));
    CHECK((m.gain == std::array<std::uint16_t, 4>{0x01c1, 0x01c1, 0x01c1, 0x01c1}));
    CHECK((m.y_offset == std::array<std::uint16_t, 4>{278, 278, 278, 278}));
    CHECK(m.stream_index == 0);
    CHECK(m.magic_ok);
}

TEST(real_metadata_led_frame) {
    const auto m = *Meta::parse(hex(META_F1));
    CHECK(m.kind() == FrameKind::Led);
    CHECK(m.ts_us_u32() == 2'850'391'179u);
    CHECK(m.ts_us_u32() - 2'850'371'191u == 19'988u);
    CHECK(static_cast<std::uint16_t>(m.counter) == 63981);
    CHECK((m.exposure == std::array<std::uint16_t, 4>{40, 40, 40, 40}));
    CHECK((m.gain == std::array<std::uint16_t, 4>{62, 62, 62, 62}));
    CHECK((m.y_offset == std::array<std::uint16_t, 4>{10, 10, 10, 10}));
    CHECK(m.stream_index == 1);
}

TEST(short_or_garbage_metadata) {
    std::vector<std::uint8_t> z95(95, 0);
    CHECK(!Meta::parse(z95));
    std::vector<std::uint8_t> row(96, 0);
    CHECK(Meta::parse(row)->kind() == FrameKind::Unknown);
    row[32] = 0x86; // frame type alone is enough
    CHECK(Meta::parse(row)->kind() == FrameKind::Tracking);
}

TEST(split_uses_metadata_offsets) {
    const std::array<std::uint16_t, 4> offs{278, 10, 100, 352};
    const auto f = synth_frame(offs, true, 5, 123);
    const auto m = *Meta::parse(std::span(f).first(META_LEN));
    const auto rows = tile_rows(&m, f, FRAME_W);
    CHECK(rows.source == RowSource::Metadata);
    CHECK((rows.y == std::array<std::size_t, 4>{278, 10, 100, 352}));
    for (std::size_t t = 0; t < 4; t++) {
        CHECK(tile_start_consistent(f, FRAME_W, t, rows.y[t]));
        std::vector<std::uint8_t> dst(700 * TILE_H, 0);
        copy_tile(f, FRAME_W, t, rows.y[t], dst, 700);
        for (const auto& [y, x] : {std::pair<std::size_t, std::size_t>{0, 0}, {0, 639}, {479, 0}, {479, 639}, {123, 456}}) {
            CHECK(dst[y * 700 + x] == static_cast<std::uint8_t>(1 + ((t * 61 + y * 7 + x * 3) % 250)));
        }
    }
}

TEST(split_falls_back_to_scan) {
    const auto f = synth_frame({278, 10, 100, 200}, false, 5, 123);
    auto bad = *Meta::parse(std::span(f).first(META_LEN));
    bad.y_offset = {0, 9999, 100, 200};
    auto rows = tile_rows(&bad, f, FRAME_W);
    CHECK(rows.source == RowSource::Scan);
    CHECK((rows.y == std::array<std::size_t, 4>{278, 10, 100, 200}));
    rows = tile_rows(nullptr, f, FRAME_W);
    CHECK((rows.y == std::array<std::size_t, 4>{278, 10, 100, 200}));
    // all-zero tile
    const std::vector<std::uint8_t> z(FRAME_BYTES, 0);
    CHECK((tile_rows(nullptr, z, FRAME_W).y == std::array<std::size_t, 4>{Y_MIN, Y_MIN, Y_MIN, Y_MIN}));
}

TEST(stride_check) {
    std::string e;
    CHECK(check_raw(FRAME_BYTES, FRAME_W, &e));
    CHECK(!check_raw(FRAME_BYTES - 1, FRAME_W, &e));
    CHECK(e == "frame has 2129919 bytes, need 2129920 (stride 2560)");
    CHECK(!check_raw(FRAME_BYTES, 100, &e));
    CHECK(e == "row stride 100 < 2560");
}

PIBRIDGE_TEST_MAIN()
