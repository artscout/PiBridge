// SPDX-License-Identifier: MIT
#include "pibridge/room/json.h"
#include "pibridge_test.h"

using namespace pibridge::room;

TEST(round_trip_keeps_order_and_numbers) {
    const std::string text = R"({"b" : 1.50, "a" : [1e-7, -0, "x\"y"], "c" : {"z" : true, "y" : null}})";
    auto j = parseJson(text);
    CHECK(j.has_value());
    CHECK(j->members()[0].first == "b");
    CHECK(j->members()[1].first == "a");
    CHECK(j->get("b").numberTextRef() == "1.50"); // untouched numbers keep their text
    const std::string out = j->dump();
    auto k = parseJson(out);
    CHECK(k.has_value());
    CHECK(*j == *k);
    CHECK(out.find("1.50") != std::string::npos);
    CHECK(out.find("1e-7") != std::string::npos);
    CHECK(k->get("a").at(2).str() == "x\"y");
    CHECK(k->get("c").get("z").boolean());
    CHECK(k->get("c").get("y").isNull());
}

TEST(numbers) {
    CHECK(formatNumber(0.5) == "0.5");
    CHECK(formatNumber(2.43) == "2.43");
    CHECK(formatNumber(3) == "3");
    CHECK(formatNumber(-1.25) == "-1.25");
    CHECK(Json::numberText("12").num() == 12);
    CHECK(Json::string("2004").num() == 2004); // universe ids are strings in SteamVR's files
    CHECK(Json::string("abc").num(7) == 7);
}

TEST(errors) {
    CHECK(!parseJson("{").has_value());
    CHECK(!parseJson("{\"a\" 1}").has_value());
    CHECK(!parseJson("[1,]").has_value());
    CHECK(!parseJson("{} x").has_value());
    CHECK(!parseJson("nul").has_value());
    CHECK(parseJson("  {}  ").has_value());
}

TEST(edit_members) {
    Json o = Json::object();
    o["x"] = Json::number(1);
    o["y"] = Json::number(2);
    o["x"] = Json::number(3);
    CHECK(o.size() == 2);
    CHECK(o.get("x").num() == 3);
    o.erase("x");
    CHECK(!o.has("x"));
    CHECK(o.get("missing").isNull());
}

TEST(unicode_escape) {
    auto j = parseJson(R"("é\n")");
    CHECK(j.has_value());
    CHECK(j->str() == "\xc3\xa9\n");
    CHECK(Json::string("a\nb").dump() == "\"a\\nb\"");
}

PIBRIDGE_TEST_MAIN()
