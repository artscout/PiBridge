// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/toml_lite.rs tests (MIT OR Apache-2.0).
#include "pibridge_test.h"

#include "pibridge/hmd/toml_lite.h"

#include <algorithm>

using namespace pibridge::hmd;

TEST(reads_the_subset) {
    const auto d = toml::parse("top = 1 # c\n[device]\nid = \"a#b\"\nvid = 0x34a4\nf = 58.5\nok = true\nl = [0x44, 2, ]\n"
                               "s = [\"x, y\", \"z\"]\n[[hid]]\nid = \"one\"\n[[hid]]\nid = \"two\"\n");
    CHECK(d.has_value());
    CHECK((*d)[0].integer("top") == std::optional<std::int64_t>(1));
    const auto& dev = (*d)[1];
    CHECK(dev.str("id") == std::optional<std::string>("a#b"));
    CHECK(dev.integer("vid") == std::optional<std::int64_t>(0x34a4));
    CHECK(dev.number("f") == std::optional<double>(58.5));
    CHECK(dev.boolean("ok") == std::optional<bool>(true));
    CHECK((dev.ints("l") == std::vector<std::int64_t>{0x44, 2}));
    CHECK((dev.strs("s") == std::vector<std::string>{"x, y", "z"}));
    CHECK(std::count_if(d->begin(), d->end(), [](const toml::Section& s) { return s.name == "hid" && s.array; }) == 2);
}

TEST(refuses_what_it_does_not_know) {
    for (const char* bad : {"a.b = 1", "a = { x = 1 }", "a = [1,\n", "[t]\n[t]", "a = 1\na = 2", "a = 'lit'"}) {
        if (toml::parse(bad).has_value()) {
            std::fprintf(stderr, "  parsed: %s\n", bad);
            CHECK(false);
        }
    }
}

TEST(errors_carry_the_line) {
    const auto e = toml::parse("a = 1\n\n# x\nb = nope\n");
    CHECK(!e.has_value());
    CHECK(e.error().line == 4);
    const auto n = toml::parse("n = -0x10\nm = 1_000\nf = -2.5\n");
    CHECK(n.has_value());
    CHECK((*n)[0].integer("n") == std::optional<std::int64_t>(-16));
    CHECK((*n)[0].integer("m") == std::optional<std::int64_t>(1000));
    CHECK((*n)[0].number("f") == std::optional<double>(-2.5));
}

PIBRIDGE_TEST_MAIN()
