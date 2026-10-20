// SPDX-License-Identifier: MIT
// C++ only: the number format of the files is Rust's. The expected strings below were printed by Rust
// (`format!("{x:.9}|{x:.4}|{x:.3}|{x:.5}|{x:.0}|{x:.1}|{x:.2}")` and `str::parse::<f64>()`), so a file written by
// either side reads the same on the other. Also checks that a non-"C" locale changes nothing.
#include <clocale>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pibridge/eyemath/calib.h"
#include "num.h"
#include "pibridge_test.h"

using pibridge::eyemath::detail::fmt_fixed;
using pibridge::eyemath::detail::parse_f64;

namespace {

std::string row(double x) {
    return fmt_fixed(x, 9) + "|" + fmt_fixed(x, 4) + "|" + fmt_fixed(x, 3) + "|" + fmt_fixed(x, 5) + "|" +
           fmt_fixed(x, 0) + "|" + fmt_fixed(x, 1) + "|" + fmt_fixed(x, 2);
}

constexpr double qnan = std::numeric_limits<double>::quiet_NaN();
constexpr double inf = std::numeric_limits<double>::infinity();

} // namespace

TEST(fixed_format_matches_rust) {
    const std::vector<std::pair<double, const char*>> cases{
        {0.0, "0.000000000|0.0000|0.000|0.00000|0|0.0|0.00"},
        {-0.0, "-0.000000000|-0.0000|-0.000|-0.00000|-0|-0.0|-0.00"},
        {1.0, "1.000000000|1.0000|1.000|1.00000|1|1.0|1.00"},
        {-1.0, "-1.000000000|-1.0000|-1.000|-1.00000|-1|-1.0|-1.00"},
        {0.1, "0.100000000|0.1000|0.100|0.10000|0|0.1|0.10"},
        {1e-10, "0.000000000|0.0000|0.000|0.00000|0|0.0|0.00"},
        {-1e-10, "-0.000000000|-0.0000|-0.000|-0.00000|-0|-0.0|-0.00"},
        {5e-10, "0.000000001|0.0000|0.000|0.00000|0|0.0|0.00"},
        {1.5e-9, "0.000000001|0.0000|0.000|0.00000|0|0.0|0.00"},
        {2.5e-9, "0.000000003|0.0000|0.000|0.00000|0|0.0|0.00"},
        {123456.7890123456, "123456.789012346|123456.7890|123456.789|123456.78901|123457|123456.8|123456.79"},
        {1e20, "100000000000000000000.000000000|100000000000000000000.0000|100000000000000000000.000|"
               "100000000000000000000.00000|100000000000000000000|100000000000000000000.0|100000000000000000000.00"},
        {-0.0000000005, "-0.000000001|-0.0000|-0.000|-0.00000|-0|-0.0|-0.00"},
        {0.125, "0.125000000|0.1250|0.125|0.12500|0|0.1|0.12"},
        {0.0625, "0.062500000|0.0625|0.062|0.06250|0|0.1|0.06"},
        {2.5, "2.500000000|2.5000|2.500|2.50000|2|2.5|2.50"},
        {3.5, "3.500000000|3.5000|3.500|3.50000|4|3.5|3.50"},
        {qnan, "NaN|NaN|NaN|NaN|NaN|NaN|NaN"},
        {inf, "inf|inf|inf|inf|inf|inf|inf"},
        {-inf, "-inf|-inf|-inf|-inf|-inf|-inf|-inf"},
        {1.0 / 3.0, "0.333333333|0.3333|0.333|0.33333|0|0.3|0.33"},
        {2.0 / 3.0, "0.666666667|0.6667|0.667|0.66667|1|0.7|0.67"},
    };
    for (const auto& [x, want] : cases) {
        const auto got = row(x);
        if (got != want) {
            std::fprintf(stderr, "  %.17g: got %s want %s\n", x, got.c_str(), want);
        }
        CHECK(got == want);
    }
    // 1e300 prints all 301 integer digits like Rust
    const auto big = fmt_fixed(1e300, 9);
    CHECK(big.size() == 301 + 10 && big.starts_with("1000000000000000052504760255204420248704468581108159154915854"));
}

TEST(parse_matches_rust) {
    struct Case {
        const char* s;
        bool ok;
        double v;
        const char* err;
    };
    const std::vector<Case> cases{
        {"1", true, 1.0, ""},
        {"+1", true, 1.0, ""},
        {"-1", true, -1.0, ""},
        {".5", true, 0.5, ""},
        {"5.", true, 5.0, ""},
        {".", false, 0, "invalid float literal"},
        {"1e5", true, 100000.0, ""},
        {"1E+5", true, 100000.0, ""},
        {"1e", false, 0, "invalid float literal"},
        {"inf", true, inf, ""},
        {"-infinity", true, -inf, ""},
        {"NaN", true, qnan, ""},
        {"nan", true, qnan, ""},
        {"+nan", true, qnan, ""},
        {"1e400", true, inf, ""},
        {"-1e400", true, -inf, ""},
        {"1e-400", true, 0.0, ""},
        {"0x10", false, 0, "invalid float literal"},
        {"", false, 0, "cannot parse float from empty string"},
        {" 1", false, 0, "invalid float literal"},
        {"1_0", false, 0, "invalid float literal"},
        {"--1", false, 0, "invalid float literal"},
        {"+-1", false, 0, "invalid float literal"},
        {"Infinity", true, inf, ""},
        {"INF", true, inf, ""},
        {"1.e3", true, 1000.0, ""},
        {"e3", false, 0, "invalid float literal"},
    };
    for (const auto& c : cases) {
        const auto r = parse_f64(c.s);
        CHECK(r.has_value() == c.ok);
        if (r.has_value() != c.ok) {
            std::fprintf(stderr, "  case \"%s\"\n", c.s);
        }
        if (r && c.ok) {
            CHECK((std::isnan(c.v) && std::isnan(*r)) || *r == c.v);
        }
        if (!r && !c.ok) {
            CHECK(r.error() == c.err);
        }
    }
    // the parsed value is the nearest double, the same as Rust's
    CHECK(*parse_f64("0.032750000") == 0.03275);
    CHECK(*parse_f64("-0.000400000") == -0.0004);
}

TEST(locale_does_not_change_the_format) {
    // a locale with a decimal comma, when the machine has one; the C++ code must not care
    const char* names[] = {"de_DE.UTF-8", "de_DE.utf8", "fr_FR.UTF-8", "ru_RU.UTF-8"};
    const char* set = nullptr;
    for (const char* n : names) {
        if ((set = std::setlocale(LC_ALL, n)) != nullptr) {
            break;
        }
    }
    if (set == nullptr) {
        std::printf("skip: no locale with a decimal comma installed\n");
    }
    auto c = pibridge::eyemath::Calibration::identity(pibridge::eyemath::FrameConv{}, 0.064);
    c.eyes[1].org.gate_mm = 5.5;
    const auto t = c.to_toml();
    CHECK(t.find("origin_gate_mm = 5.500\n") != std::string::npos);
    CHECK(t.find("origin_m = [0.032000000, 0.000000000, 0.000000000]") != std::string::npos);
    const auto back = pibridge::eyemath::Calibration::parse(t);
    CHECK(back && back->eyes[1].org.gate_mm == 5.5);
    CHECK(*parse_f64("1.25") == 1.25);
    std::setlocale(LC_ALL, "C");
}

PIBRIDGE_TEST_MAIN()
