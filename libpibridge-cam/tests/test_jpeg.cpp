// SPDX-License-Identifier: MIT
// The Motion-JPEG camera profile (on a 480 Mbps USB link the Crystal Super cameras offer MJPEG, no YUYV): JPEG -> grey
// and what the capture pipeline does with a source that has no metadata. No device is opened here.
#include "common.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/jpeg.h"
#include "pibridge/cam/record.h"
#include "pibridge_test.h"

extern "C" {
#include <jpeglib.h>
}

#include <algorithm>
#include <cstdlib>
#include <cstdio>

using namespace pibridge::cam;
using pibridge::cam::testing::VecSource;

namespace {

/// Grey 8-bit image -> JPEG bytes (the encoder side of what the camera firmware does).
std::vector<std::uint8_t> encode_jpeg(const std::vector<std::uint8_t> &grey, std::uint32_t w, std::uint32_t h,
                                      int quality = 90) {
    jpeg_compress_struct ci{};
    jpeg_error_mgr err{};
    ci.err = jpeg_std_error(&err);
    jpeg_create_compress(&ci);
    unsigned char *out = nullptr;
    unsigned long len = 0;
    jpeg_mem_dest(&ci, &out, &len);
    ci.image_width = w;
    ci.image_height = h;
    ci.input_components = 1;
    ci.in_color_space = JCS_GRAYSCALE;
    jpeg_set_defaults(&ci);
    jpeg_set_quality(&ci, quality, TRUE);
    jpeg_start_compress(&ci, TRUE);
    for (std::uint32_t y = 0; y < h; y++) {
        JSAMPROW row = const_cast<JSAMPROW>(&grey[static_cast<std::size_t>(y) * w]);
        (void)jpeg_write_scanlines(&ci, &row, 1);
    }
    jpeg_finish_compress(&ci);
    jpeg_destroy_compress(&ci);
    std::vector<std::uint8_t> v(out, out + len);
    std::free(out);
    return v;
}

/// A canonical frame of the Motion-JPEG profile: nothing in the metadata rows, tiles 0 and 1 carrying an image from
/// row `start`, tiles 2 and 3 zero (the controller cameras of this firmware are not in the stream).
std::vector<std::uint8_t> jpeg_like_frame(std::size_t start) {
    std::vector<std::uint8_t> f(FRAME_BYTES, 0);
    for (std::size_t t = 0; t < 2; t++) {
        for (std::size_t y = 0; y < TILE_H; y++) {
            auto *row = &f[(start + y) * FRAME_W + t * TILE_W];
            for (std::size_t x = 0; x < TILE_W; x++) {
                row[x] = static_cast<std::uint8_t>(8 + ((x + y) % 200));
            }
        }
    }
    return f;
}

} // namespace

TEST(jpeg_decodes_to_the_luma_plane) {
    constexpr std::uint32_t W = 128, H = 96;
    std::vector<std::uint8_t> grey(W * H);
    for (std::uint32_t y = 0; y < H; y++) {
        for (std::uint32_t x = 0; x < W; x++) {
            grey[static_cast<std::size_t>(y) * W + x] = static_cast<std::uint8_t>((x * 2 + y) % 256);
        }
    }
    const std::vector<std::uint8_t> jpg = encode_jpeg(grey, W, H);
    CHECK(looks_like_jpeg(jpg));
    std::vector<std::uint8_t> back(grey.size(), 0);
    std::uint32_t w = 0, h = 0;
    const auto r = decode_grey_jpeg(jpg, back, w, h);
    CHECK(r.has_value());
    CHECK(w == W);
    CHECK(h == H);
    // Lossy: every pixel within a few levels of the original, which is smooth by construction.
    int worst = 0;
    for (std::size_t i = 0; i < grey.size(); i++) {
        worst = std::max(worst, std::abs(int(grey[i]) - int(back[i])));
    }
    CHECK(worst <= 12);
}

TEST(jpeg_rejects_junk_and_small_buffers) {
    std::uint32_t w = 0, h = 0;
    std::vector<std::uint8_t> out(64 * 64, 0);
    const std::vector<std::uint8_t> junk{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    CHECK(!looks_like_jpeg(junk));
    CHECK(!decode_grey_jpeg(junk, out, w, h).has_value());
    // A JPEG header with no image in it must come back as an error, not a crash.
    const std::vector<std::uint8_t> half_jpeg{0xff, 0xd8, 0xff, 0xe0, 0x00, 0x04, 0x00, 0x00};
    CHECK(looks_like_jpeg(half_jpeg));
    CHECK(!decode_grey_jpeg(half_jpeg, out, w, h).has_value());
    const std::vector<std::uint8_t> jpg = encode_jpeg(std::vector<std::uint8_t>(32 * 32, 128), 32, 32);
    std::vector<std::uint8_t> too_small(32 * 32 - 1, 0);
    CHECK(!decode_grey_jpeg(jpg, too_small, w, h).has_value());
}

TEST(jpeg_profile_frames_are_tracking_without_device_time) {
    auto src = std::make_unique<VecSource>();
    src->stream_profile = StreamProfile::JpegTiles2;
    src->frames.emplace_back(jpeg_like_frame(99), 1'000'000'000ull);
    src->frames.emplace_back(jpeg_like_frame(120), 2'000'000'000ull);
    Camera cam(std::move(src));
    std::vector<FrameInfo> infos;
    for (;;) {
        auto f = cam.next_frame(0);
        CHECK(f.has_value());
        if (!*f) {
            break;
        }
        infos.push_back(**f);
    }
    CHECK(infos.size() == 2);
    CHECK(cam.stats.unknown == 0);
    CHECK(cam.stats.tracking == 2);
    for (std::size_t i = 0; i < infos.size(); i++) {
        const FrameInfo &in = infos[i];
        CHECK(in.kind == static_cast<std::uint32_t>(FrameKind::Tracking));
        CHECK((in.flags & FLAG_NO_DEV_TIME) != 0);
        CHECK((in.flags & FLAG_META_VALID) == 0);
        CHECK((in.flags & FLAG_ROWS_SCANNED) != 0);
        CHECK(in.dev_ts_us == 0);
        CHECK(in.counter == 0);
        CHECK(in.mono_ts_ns == (i == 0 ? 1'000'000'000ull : 2'000'000'000ull));
        CHECK(in.y_offset[0] == (i == 0 ? 99 : 120));
    }
}

TEST(jpeg_profile_tiles_two_and_three_are_empty) {
    auto src = std::make_unique<VecSource>();
    src->stream_profile = StreamProfile::JpegTiles2;
    src->frames.emplace_back(jpeg_like_frame(99), 1'000'000'000ull);
    Camera cam(std::move(src));
    auto f = cam.next_frame(0);
    CHECK(f.has_value() && f->has_value());
    auto t0 = cam.tile(0);
    auto t2 = cam.tile(2);
    CHECK(t0.has_value());
    CHECK(t2.has_value());
    CHECK(std::any_of(t0->begin(), t0->end(), [](std::uint8_t v) { return v != 0; }));
    CHECK(std::all_of(t2->begin(), t2->end(), [](std::uint8_t v) { return v == 0; }));
}

TEST(record_keeps_the_stream_profile) {
    const std::string jpeg_path = pibridge::cam::testing::tmp("profile-jpeg.pxcam");
    {
        auto r = Recorder::create(jpeg_path, FRAME_W, std::nullopt, StreamProfile::JpegTiles2);
        CHECK(r.has_value());
        RawFrame fr{jpeg_like_frame(99), FRAME_W, 5ull, 0};
        CHECK((*r)->write(fr).has_value());
        CHECK((*r)->finish().has_value());
    }
    auto rd = Reader::open(jpeg_path);
    CHECK(rd.has_value());
    if (rd) {
        CHECK((*rd)->profile == StreamProfile::JpegTiles2);
    }
    // A file of the grey profile (and one written before the field existed) reads back as GreyTiles4.
    const std::string grey_path = pibridge::cam::testing::tmp("profile-grey.pxcam");
    {
        auto r = Recorder::create(grey_path, FRAME_W, std::nullopt);
        CHECK(r.has_value());
        RawFrame fr{jpeg_like_frame(99), FRAME_W, 5ull, 0};
        CHECK((*r)->write(fr).has_value());
        CHECK((*r)->finish().has_value());
    }
    auto rd2 = Reader::open(grey_path);
    CHECK(rd2.has_value());
    if (rd2) {
        CHECK((*rd2)->profile == StreamProfile::GreyTiles4);
    }
    std::remove(jpeg_path.c_str());
    std::remove(grey_path.c_str());
}

PIBRIDGE_TEST_MAIN()
