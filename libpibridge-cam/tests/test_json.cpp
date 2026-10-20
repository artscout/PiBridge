// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/json.rs (MIT OR Apache-2.0).
#include "pibridge/cam/json.h"
#include "pibridge_test.h"

#include <cfloat>
#include <cmath>
#include <limits>

using namespace pibridge::cam;

TEST(parses_documents) {
    const auto v = *json::parse(
        R"( { "a": [1, -2.5e-3, 0, true, false, null], "b": {"c": "x\"y\\z\u00e9\ud83d\ude00"} } )");
    const auto& a = *v.get("a")->as_array();
    CHECK(a[0] == json::Value::number(1.0));
    CHECK(a[1] == json::Value::number(-0.0025));
    CHECK(a[3] == json::Value::boolean(true));
    CHECK(a[5] == json::Value::null());
    CHECK(*v.get("b")->get("c")->as_str() == "x\"y\\z\xc3\xa9\xf0\x9f\x98\x80");
    CHECK((json::parse("[1,2,3]")->as_f64_array<3>() == std::array<double, 3>{1.0, 2.0, 3.0}));
    CHECK(!json::parse("[1,2]")->as_f64_array<3>());
    CHECK(json::parse("{}")->type == json::Value::Type::Obj && json::parse("{}")->obj.empty());
    CHECK(json::parse("[]")->type == json::Value::Type::Arr && json::parse("[]")->arr.empty());
}

TEST(rejects_bad_documents) {
    for (const char* bad : {"", "{", "[1,]", "{\"a\" 1}", "01", "1.", "-", "1e", "\"abc", "\"\\x\"", "tru", "[1] 2",
                            "{1: 2}", "\"\x01\"", "1e999", "\"\\ud800\""}) {
        CHECK(!json::parse(bad));
    }
    std::string deep(100, '[');
    deep += std::string(100, ']');
    CHECK(!json::parse(deep));
    CHECK(json::parse("01").error().text() == "JSON error at byte 1: trailing characters");
    CHECK(json::parse("1e999").error().text() == "JSON error at byte 0: number out of range");
}

TEST(writer_roundtrip) {
    const std::string s = "tab\t quote\" back\\ nl\n ctl\x02 \xc3\xa9";
    CHECK(*json::parse(json::quote(s))->as_str() == s);
    for (const double x : {0.0, -1.5, 1e-12, 0.1 + 0.2, 123456789.0, DBL_MIN}) {
        CHECK(*json::parse(json::num(x))->as_f64() == x);
    }
    CHECK(json::num(std::numeric_limits<double>::quiet_NaN()) == "null");
    const double two[] = {1.0, 2.5};
    CHECK((json::parse(json::num_array(two))->as_f64_array<2>() == std::array<double, 2>{1.0, 2.5}));
    // the Rust {:?} spelling
    CHECK(json::num(1.0) == "1.0");
    CHECK(json::num(0.1 + 0.2) == "0.30000000000000004");
    CHECK(json::num(1e-12) == "1e-12");
    CHECK(json::num(-4.10994e-5) == "-4.10994e-5");
    CHECK(json::num(1e16) == "1e16");
    CHECK(json::num(123456789.0) == "123456789.0");
    CHECK(json::quote("\x02") == "\"\\u0002\"");
}

PIBRIDGE_TEST_MAIN()
