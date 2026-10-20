// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/shm.rs tests (MIT OR Apache-2.0). Temp files only: the live daemon's
// /dev/shm/omarchyvr-eye-gaze is never touched.
#include "pibridge/eye/shm.h"

#include "pibridge_test.h"
#include "testutil.h"

#include <cstddef>
#include <cstring>
#include <sys/stat.h>

using namespace pibridge::eye::shm;
namespace fs = std::filesystem;

namespace {

Sample sample() {
    Sample s;
    s.seq = 42;
    s.host_ts_ns = 123'456'789;
    s.ipd_mm = 64.5f;
    s.eyes[0] = EyeRay{{-0.032f, 0.0f, 0.0f}, {0.1f, 0.05f, -0.99f}, true, true};
    s.eyes[1] = EyeRay{{}, {0.08f, 0.05f, -0.99f}, false, true};
    s.gaze = CombinedRay{{0.0f, 0.0f, 0.0f}, {0.09f, 0.05f, -0.99f}};
    return s;
}

std::uint64_t seq_begin(const fs::path& p) {
    const auto b = testutil::read_bytes(p);
    std::uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = (v << 8) | b[static_cast<std::size_t>(i)];
    }
    return v;
}

// The C layout of omarchyvr linux/crystal-super/omarchyvr_eye_gaze.h, copied field for field.
#pragma pack(push, 1)
struct omarchyvr_eye_gaze {
    std::uint64_t seq_begin;
    std::uint32_t magic;
    std::uint32_t version;
    std::int64_t t_ns;
    std::int64_t written_ns;
    std::uint32_t flags;
    float ipd_mm;
    float left_origin[3], left_dir[3];
    float right_origin[3], right_dir[3];
    float combined_origin[3], combined_dir[3];
    std::uint64_t sample;
    std::uint32_t state;
    std::uint32_t reserved;
    std::uint64_t seq_end;
};
#pragma pack(pop)
static_assert(sizeof(omarchyvr_eye_gaze) == SIZE);

} // namespace

TEST(publish_and_read_back) {
    const auto dir = testutil::temp_dir("shm");
    const auto p = dir / "gaze";
    auto w = Publisher::create(p);
    CHECK(w.has_value());
    CHECK(!read(p)); // nothing published yet
    const auto s = sample();
    CHECK(w->publish(&s, 2, 999).has_value());
    auto g = read(p);
    CHECK(g.has_value());
    CHECK(g->t_ns == 123'456'789);
    CHECK(g->written_ns == 999);
    CHECK(g->flags == (LEFT | RIGHT | COMBINED | LEFT_ORIGIN));
    CHECK((g->direction[0] == Vec3{0.1f, 0.05f, -0.99f}));
    CHECK((g->direction[1] == Vec3{0.08f, 0.05f, -0.99f}));
    CHECK((g->origin[0] == Vec3{-0.032f, 0.0f, 0.0f}));
    CHECK(g->ipd_mm == 64.5f);
    CHECK(g->sample == 42 && g->state == 2);
    // no gaze: flags clear, state kept
    CHECK(w->publish(nullptr, 1, 1000).has_value());
    g = read(p);
    CHECK(g && g->flags == 0 && g->state == 1);
    // a new writer continues the sequence
    const auto before = seq_begin(p);
    auto w2 = Publisher::create(p);
    CHECK(w2.has_value());
    CHECK(w2->publish(&s, 2, 1).has_value());
    const auto after = seq_begin(p);
    CHECK(after > before && after % 2 == 0);
    // the file is private to the user
    struct stat st{};
    CHECK(::stat(p.c_str(), &st) == 0 && (st.st_mode & 0777) == 0600);
    CHECK(fs::file_size(p) == SIZE);
    fs::remove_all(dir);
}

TEST(torn_snapshots_are_refused) {
    std::array<std::uint8_t, SIZE> f{};
    const auto s = sample();
    const auto b = body(&s, 2, 5);
    std::copy(b.begin(), b.end(), f.begin() + 8);
    const auto set = [&](std::size_t off, std::uint64_t v) {
        for (int i = 0; i < 8; i++) {
            f[off + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(v >> (8 * i));
        }
    };
    set(0, 4);
    set(128, 4);
    CHECK(decode(f).has_value());
    set(0, 5); // writer in the middle
    CHECK(!decode(f));
    set(0, 6); // begin newer than end
    CHECK(!decode(f));
    set(0, 4);
    f[8] ^= 1; // another layout
    CHECK(!decode(f));
}

TEST(the_layout_is_byte_identical_to_the_c_header) {
    CHECK(offsetof(omarchyvr_eye_gaze, magic) == 8);
    CHECK(offsetof(omarchyvr_eye_gaze, t_ns) == 16);
    CHECK(offsetof(omarchyvr_eye_gaze, flags) == 32);
    CHECK(offsetof(omarchyvr_eye_gaze, left_origin) == 40);
    CHECK(offsetof(omarchyvr_eye_gaze, right_dir) == 76);
    CHECK(offsetof(omarchyvr_eye_gaze, combined_dir) == 100);
    CHECK(offsetof(omarchyvr_eye_gaze, sample) == 112);
    CHECK(offsetof(omarchyvr_eye_gaze, seq_end) == 128);
    const auto dir = testutil::temp_dir("shm-c");
    const auto p = dir / "gaze";
    auto w = Publisher::create(p);
    const auto s = sample();
    CHECK(w && w->publish(&s, 2, 777).has_value());
    const auto bytes = testutil::read_bytes(p);
    CHECK(bytes.size() == SIZE);
    omarchyvr_eye_gaze c{};
    std::memcpy(&c, bytes.data(), sizeof c);
    CHECK(c.magic == 0x4745564fu && c.version == 1);
    CHECK(c.seq_begin == c.seq_end && c.seq_begin % 2 == 0 && c.seq_begin != 0);
    CHECK(c.t_ns == 123'456'789 && c.written_ns == 777);
    CHECK(c.flags == (LEFT | RIGHT | COMBINED | LEFT_ORIGIN));
    CHECK(c.ipd_mm == 64.5f);
    CHECK(c.left_origin[0] == -0.032f && c.left_dir[2] == -0.99f);
    CHECK(c.right_dir[0] == 0.08f && c.right_origin[0] == 0.0f);
    CHECK(c.combined_dir[0] == 0.09f);
    CHECK(c.sample == 42 && c.state == 2 && c.reserved == 0);
    CHECK(bytes[8] == 'O' && bytes[9] == 'V' && bytes[10] == 'E' && bytes[11] == 'G');
    fs::remove_all(dir);
}

TEST(a_negative_or_nan_ipd_is_published_as_zero) {
    auto s = sample();
    s.ipd_mm = -3.0f;
    auto b = body(&s, 2, 0);
    CHECK(b[36 - 8] == 0 && b[37 - 8] == 0 && b[38 - 8] == 0 && b[39 - 8] == 0);
    s.ipd_mm = std::nanf("");
    b = body(&s, 2, 0);
    CHECK(b[36 - 8] == 0 && b[37 - 8] == 0 && b[38 - 8] == 0 && b[39 - 8] == 0);
}

TEST(the_default_path_is_the_deployed_one) {
    CHECK(default_path() == fs::path("/dev/shm/omarchyvr-eye-gaze"));
}

PIBRIDGE_TEST_MAIN()
