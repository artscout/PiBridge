// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/png.rs (MIT OR Apache-2.0).
#include "pibridge/lens/png.h"
#include "pibridge_test.h"

#include <stdexcept>
#include <string_view>

using namespace pibridge::lens;

namespace {

std::span<const std::uint8_t> bytes(std::string_view s) {
    return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

} // namespace

TEST(crc32_known_vectors) {
    CHECK(png::crc32(bytes("")) == 0);
    CHECK(png::crc32(bytes("123456789")) == 0xcbf43926u);
    // the IEND chunk of every PNG
    CHECK(png::crc32(bytes("IEND")) == 0xae426082u);
}

TEST(roundtrip_rgb_rgba_grey) {
    for (const std::size_t channels : {std::size_t{1}, std::size_t{3}, std::size_t{4}}) {
        const std::uint32_t w = 37;
        const std::uint32_t h = 19;
        std::vector<std::uint8_t> data(std::size_t{w} * h * channels);
        for (std::size_t i = 0; i < data.size(); i++) {
            data[i] = static_cast<std::uint8_t>((i * 7 + i / 13) % 251);
        }
        const auto p = png::encode(w, h, channels, data);
        const auto d = png::decode(p);
        CHECK(d.has_value());
        CHECK(d->width == w && d->height == h && d->channels == channels);
        CHECK(d->data == data);
    }
}

TEST(flat_images_compress_well_and_signature_is_right) {
    const std::vector<std::uint8_t> data(640 * 480 * 3, 200);
    const auto p = png::encode_rgb8(640, 480, data);
    const std::uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
    CHECK(std::equal(sig, sig + 8, p.begin()));
    CHECK(p.size() < 3000);
    CHECK(png::decode(p)->data == data);
}

TEST(corruption_is_detected) {
    const std::vector<std::uint8_t> px(48, 7);
    auto p = png::encode_rgb8(4, 4, px);
    const std::size_t n = p.size();
    p[n - 20] ^= 0xff; // inside IDAT or its CRC
    CHECK(!png::decode(p));
    CHECK(!png::decode(bytes("nope")));
}

TEST(wrong_buffer_size_panics) {
    bool thrown = false;
    try {
        const std::vector<std::uint8_t> px(11, 0);
        (void)png::encode_rgb8(2, 2, px);
    } catch (const std::invalid_argument& e) {
        thrown = std::string_view(e.what()).find("pixel buffer size") != std::string_view::npos;
    }
    CHECK(thrown);
}

PIBRIDGE_TEST_MAIN()
