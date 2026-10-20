// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/record.rs and tests/replay.rs (MIT OR Apache-2.0). The replay twins
// use the recorded first.pxcam (private lab data: PIBRIDGE_LAB_CAM_DIR) and are skipped without it.
#include "common.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/record.h"
#include "pibridge_test.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <thread>

using namespace pibridge::cam;
using pibridge::cam::testing::tmp;

static std::vector<std::uint8_t> frame(std::uint8_t seed) {
    std::vector<std::uint8_t> v(FRAME_BYTES, 0);
    for (std::size_t i = 0; i < v.size(); i += 97) {
        v[i] = static_cast<std::uint8_t>(static_cast<std::uint8_t>(i) * seed) | 1;
    }
    return v;
}

TEST(roundtrip_raw_and_zlib) {
    for (const std::optional<std::uint8_t> level : {std::optional<std::uint8_t>{}, std::optional<std::uint8_t>{1}}) {
        const std::string p = tmp(level ? "rt1" : "rt0");
        auto rec = *Recorder::create(p, FRAME_W, level);
        std::vector<std::vector<std::uint8_t>> frames;
        for (std::uint8_t s = 1; s < 4; s++) {
            frames.push_back(frame(s));
        }
        for (std::size_t i = 0; i < frames.size(); i++) {
            CHECK(rec->write(RawFrame{frames[i], FRAME_W, 1000 + i * 20'000'000, static_cast<std::uint32_t>(i)})
                      .has_value());
        }
        const auto [n, bytes] = *rec->finish();
        CHECK(n == 3);
        CHECK(bytes == std::filesystem::file_size(p));
        if (level) {
            CHECK(bytes < 3 * FRAME_BYTES / 4);
        }
        auto r = *Reader::open(p);
        Record x;
        for (std::size_t i = 0; i < frames.size(); i++) {
            CHECK(*r->next_record(x));
            CHECK(x.data == frames[i]);
            CHECK(x.sequence == i);
            CHECK(x.host_ts_ns == 1000 + i * 20'000'000);
        }
        CHECK(!*r->next_record(x));
        CHECK(!r->truncated);
        std::filesystem::remove(p);
    }
}

TEST(async_recorder_writes_everything) {
    const std::string p = tmp("async");
    auto rec = *AsyncRecorder::start(p, FRAME_W, std::nullopt, 16);
    const auto f = frame(5);
    for (std::uint32_t i = 0; i < 10; i++) {
        rec->push(RawFrame{f, FRAME_W, i, i});
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(rec->dropped == 0);
    const auto [n, b] = *rec->finish();
    CHECK(n == 10);
    (void)b;
    auto r = *Reader::open(p);
    Record x;
    for (std::uint32_t i = 0; i < 10; i++) {
        CHECK(*r->next_record(x));
        CHECK(x.sequence == i);
    }
    std::filesystem::remove(p);
}

TEST(truncated_tail_and_garbage) {
    const std::string p = tmp("trunc");
    auto rec = *Recorder::create(p, FRAME_W, std::nullopt);
    const auto f = frame(7);
    for (std::uint32_t i = 0; i < 2; i++) {
        CHECK(rec->write(RawFrame{f, FRAME_W, i, i}).has_value());
    }
    CHECK(rec->finish().has_value());
    const auto len = std::filesystem::file_size(p);
    std::filesystem::resize_file(p, len - 1000);
    auto r = *Reader::open(p);
    Record x;
    CHECK(*r->next_record(x));
    CHECK(!*r->next_record(x));
    CHECK(r->truncated);
    const auto garbage = [&](const char* s) {
        std::FILE* o = std::fopen(p.c_str(), "wb");
        std::fwrite(s, 1, std::strlen(s), o);
        std::fclose(o);
    };
    garbage("PXCAMRAWgarbage");
    CHECK(!Reader::open(p));
    garbage("nope");
    CHECK(!Reader::open(p));
    std::filesystem::remove(p);
}

TEST(replay_rebases_and_loops) {
    const std::string p = tmp("loop");
    auto rec = *Recorder::create(p, FRAME_W, std::uint8_t{1});
    const auto f = frame(3);
    for (std::uint64_t i = 0; i < 3; i++) {
        CHECK(rec->write(RawFrame{f, FRAME_W, 7'000'000'000 + i * 20'000'000, static_cast<std::uint32_t>(i)})
                  .has_value());
    }
    CHECK(rec->finish().has_value());
    auto rp = *Replay::open(p, ReplayOptions{true, false});
    std::vector<std::uint64_t> ts;
    for (int i = 0; i < 7; i++) {
        const auto fr = *rp->next(0);
        CHECK(fr.has_value() && fr->data.size() == FRAME_BYTES);
        ts.push_back(fr->host_ts_ns);
    }
    for (std::size_t i = 1; i < ts.size(); i++) {
        CHECK(ts[i] > ts[i - 1]);
    }
    CHECK(ts[1] - ts[0] == 20'000'000);
    CHECK(rp->loops == 2);
    auto once = *Replay::open(p, ReplayOptions{});
    for (int i = 0; i < 3; i++) {
        CHECK((*once->next(0)).has_value());
    }
    CHECK(!(*once->next(0)).has_value());
    CHECK(once->finished());
    std::filesystem::remove(p);
}

// tests/replay.rs real_frames_parse_and_split on first.pxcam: metadata, kinds, tile rows, tile content against an
// independent reading of the raw bytes.
TEST(real_frames_parse_and_split) {
    const std::string clip = PIBRIDGE_LAB_CAM_DIR "/first.pxcam";
    if (!is_file(clip)) {
        std::printf("skipped: %s not present\n", clip.c_str());
        return;
    }
    auto raw = *Reader::open(clip);
    auto cam = *Camera::open(clip, 0, ReplayOptions{});
    Record rec;
    int n = 0;
    std::uint32_t last_counter = 0;
    while (*raw->next_record(rec)) {
        const auto i = **cam->next_frame(0);
        const auto m = *Meta::parse(std::span(rec.data).first(META_LEN));
        CHECK(i.kind == static_cast<std::uint32_t>(m.kind()));
        CHECK(i.counter == m.counter);
        CHECK(i.dev_ts_us == m.ts_us);
        CHECK(i.exposure == m.exposure);
        CHECK(i.y_offset == m.y_offset);
        if (n > 0) {
            CHECK(i.counter == last_counter + 1);
        }
        last_counter = i.counter;
        for (std::size_t t = 0; t < 4; t++) {
            const auto tile = *cam->tile(t);
            const std::size_t row = m.y_offset[t];
            for (const std::size_t y : {std::size_t{0}, std::size_t{1}, std::size_t{240}, std::size_t{479}}) {
                CHECK(std::equal(tile.begin() + static_cast<std::ptrdiff_t>(y * TILE_W),
                                 tile.begin() + static_cast<std::ptrdiff_t>((y + 1) * TILE_W),
                                 rec.data.begin() + static_cast<std::ptrdiff_t>((row + y) * FRAME_W + t * TILE_W)));
            }
        }
        n++;
    }
    CHECK(!(*cam->next_frame(0)).has_value());
    CHECK(n > 100);
    CHECK(cam->stats.row_mismatch == 0);
    CHECK(cam->stats.dropped == 0);
    CHECK(cam->stats.kind_repeats == 0);
}

// tests/replay.rs record_replay_roundtrip_of_real_frames.
TEST(record_replay_roundtrip_of_real_frames) {
    const std::string clip = PIBRIDGE_LAB_CAM_DIR "/first.pxcam";
    if (!is_file(clip)) {
        std::printf("skipped: %s not present\n", clip.c_str());
        return;
    }
    const std::string out = tmp("roundtrip.pxcam");
    auto cam = *Camera::open(clip, 0, ReplayOptions{});
    CHECK(cam->start_recording(out, std::nullopt).has_value());
    std::vector<std::pair<FrameInfo, std::vector<std::vector<std::uint8_t>>>> a;
    for (int k = 0; k < 10; k++) {
        const auto i = **cam->next_frame(0);
        std::vector<std::vector<std::uint8_t>> tiles;
        for (std::size_t t = 0; t < 4; t++) {
            tiles.push_back(*cam->tile(t));
        }
        a.emplace_back(i, std::move(tiles));
    }
    const auto st = *cam->stop_recording();
    CHECK(st && st->first == 10);
    auto again = *Camera::open(out, 0, ReplayOptions{true, false});
    for (std::size_t k = 0; k < 30; k++) {
        const auto i = **again->next_frame(0);
        const auto& [ai, at] = a[k % 10];
        CHECK(i.counter == ai.counter && i.kind == ai.kind && i.y_offset == ai.y_offset && i.exposure == ai.exposure &&
              i.gain == ai.gain);
        for (std::size_t t = 0; t < 4; t++) {
            CHECK(*again->tile(t) == at[t]);
        }
    }
    // looping restarts the counter: no false drops are reported
    CHECK(again->stats.dropped == 0);
    std::filesystem::remove(out);
}

PIBRIDGE_TEST_MAIN()
