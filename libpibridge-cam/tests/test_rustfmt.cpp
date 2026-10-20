// SPDX-License-Identifier: MIT
// The Rust formatting rules reproduced by rustfmt.h (values checked against rustc 1.99 output).
#include "pibridge/cam/rustfmt.h"
#include "pibridge_test.h"

#include <cmath>
#include <limits>

using namespace pibridge::cam;

TEST(debug_and_display_of_f64) {
    CHECK(rfmt::debug_f64(1e16) == "1e16");
    CHECK(rfmt::debug_f64(1e-5) == "1e-5");
    CHECK(rfmt::debug_f64(0.0001) == "0.0001");
    CHECK(rfmt::debug_f64(-0.0) == "-0.0");
    CHECK(rfmt::debug_f64(123456789.0) == "123456789.0");
    CHECK(rfmt::debug_f64(1.5e300) == "1.5e300");
    CHECK(rfmt::debug_f64(-4.10994e-5) == "-4.10994e-5");
    CHECK(rfmt::debug_f64(0.0637485513) == "0.0637485513");
    CHECK(rfmt::debug_f64(std::numeric_limits<double>::quiet_NaN()) == "NaN");
    CHECK(rfmt::debug_f64(-std::numeric_limits<double>::infinity()) == "-inf");
    CHECK(rfmt::display_f64(3.0) == "3");
    CHECK(rfmt::display_f64(1e-7) == "0.0000001");
    CHECK(rfmt::display_f64(-0.0) == "-0");
    CHECK(rfmt::display_f64(1e21) == "1000000000000000000000");
}

TEST(fixed_precision) {
    CHECK(rfmt::fixed(0.5, 0) == "0");
    CHECK(rfmt::fixed(0.25, 1) == "0.2");
    CHECK(rfmt::fixed(0.125, 2) == "0.12");
    CHECK(rfmt::fixed(-0.0, 3, true) == "-0.000");
    CHECK(rfmt::fixed(std::numeric_limits<double>::quiet_NaN(), 1, true) == "NaN");
    CHECK(rfmt::fixed_w(-0.04, 7, 1) == "   -0.0");
    CHECK(rfmt::fixed_w(std::numeric_limits<double>::infinity(), 6, 1) == "   inf");
}

TEST(debug_str_and_arrays) {
    CHECK(rfmt::debug_str("a\"b\\c\n\t\x01\x7f\xc3\xa9'") == "\"a\\\"b\\\\c\\n\\t\\u{1}\\u{7f}\xc3\xa9'\"");
    const double v[] = {1.0, -0.5};
    CHECK(rfmt::debug_f64s(v) == "[1.0, -0.5]");
    const std::uint16_t u[] = {3500, 3500};
    CHECK(rfmt::debug_u16s(u) == "[3500, 3500]");
}

TEST(parse_like_rust) {
    double x = 0;
    CHECK(rfmt::parse_f64("+1.5", x) && x == 1.5);
    CHECK(rfmt::parse_f64("5.", x) && x == 5.0);
    CHECK(rfmt::parse_f64(".5", x) && x == 0.5);
    CHECK(rfmt::parse_f64("1e999", x) && std::isinf(x));
    CHECK(rfmt::parse_f64("1e-400", x) && x == 0.0);
    CHECK(rfmt::parse_f64("-inf", x) && std::isinf(x) && x < 0);
    CHECK(rfmt::parse_f64("NaN", x) && std::isnan(x));
    CHECK(rfmt::parse_f64("infinity", x));
    for (const char* bad : {"", ".", "1e", "+-1", "0x10", "1_0", "e5", " 1", "nan(1)", "infin"}) {
        CHECK(!rfmt::parse_f64(bad, x));
    }
    std::uint32_t u = 0;
    CHECK(rfmt::parse_u32("+7", u) && u == 7);
    CHECK(!rfmt::parse_u32("-0", u));
    CHECK(!rfmt::parse_u32("4294967296", u));
}

TEST(whitespace_and_utf8) {
    CHECK(rfmt::trim("\xc2\xa0 x \xe3\x80\x80") == "x");
    const auto v = rfmt::split_whitespace("1 \xe2\x80\x83 2\t3");
    CHECK(v.size() == 3 && v[0] == "1" && v[1] == "2" && v[2] == "3");
    CHECK(rfmt::utf8_lossy("a\xff" "b\xe2\x82") == "a\xef\xbf\xbd" "b\xef\xbf\xbd");
    CHECK(rfmt::utf8_valid("\xc3\xa9"));
    CHECK(!rfmt::utf8_valid("\xed\xa0\x80")); // a surrogate
}

TEST(casts_saturate) {
    CHECK(rfmt::f64_to_u32(-1.0) == 0);
    CHECK(rfmt::f64_to_u32(std::nan("")) == 0);
    CHECK(rfmt::f64_to_u32(1e20) == UINT32_MAX);
    CHECK(rfmt::f64_to_u32(640.9) == 640);
}

PIBRIDGE_TEST_MAIN()
