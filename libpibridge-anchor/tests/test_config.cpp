// SPDX-License-Identifier: MIT
// anchor.toml: round trip, errors, the path rule.
#include "pibridge/anchor/config.h"
#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge_test.h"

#include <cstdlib>
#include <filesystem>

using namespace pibridge;
namespace g = cam::geom;

TEST(round_trip_with_windows) {
    anchor::Anchor a;
    a.id = 7;
    a.size_m = 0.15;
    a.reference = g::Pose{g::rodrigues({0.1, 0.2, -0.3}), {0.5, -0.8, 1.2}};
    a.saved = "unix 1760000000";
    a.windows.push_back({"terminal", g::Pose{g::rodrigues({0.0, 0.4, 0.0}), {0.2, 0.3, -0.1}}});
    const std::string t = a.to_toml();
    CHECK(t.starts_with("# written by `omarchyvr-anchor calibrate`\n[tag]\nfamily = \"tag36h11\"\nid = 7\nsize_m = 0.15\n"));
    const auto b = anchor::Anchor::parse(t);
    CHECK(b && b->id == 7 && b->size_m == 0.15 && b->saved == a.saved && b->windows.size() == 1 &&
          b->windows[0].name == "terminal");
    CHECK(b && g::norm(g::sub(b->reference.t, a.reference.t)) < 1e-6);
    CHECK(b && b->to_toml() == t);
}

TEST(errors_in_plain_words) {
    CHECK(anchor::Anchor::parse("garbage").error() == "bad line: garbage");
    CHECK(anchor::Anchor::parse("[tag]\nid = x\n").error() == "invalid digit found in string");
    CHECK(anchor::Anchor::parse("").error() == "tag.id missing");
    CHECK(anchor::Anchor::parse("[tag]\nid=1\nsize_m=1\n[reference]\npos=[1,,3]\n").error() ==
          "pos: cannot parse float from empty string");
}

TEST(path_and_save) {
    const char* t = std::getenv("TMPDIR");
    const std::string d = std::string(t != nullptr ? t : "/tmp") + "/anchor-cfg-" + std::to_string(anchor::rs::process_id());
    std::filesystem::remove_all(d);
    ::setenv("HOME", d.c_str(), 1);
    ::unsetenv("PIBRIDGE_ANCHOR_CONFIG");
    ::unsetenv("PIMAX_ANCHOR_CONFIG");
    CHECK(anchor::anchor_config_path() == d + "/.config/pibridge/anchor.toml");
    std::filesystem::create_directories(d + "/.config/omarchyvr");
    CHECK(cam::write_file(d + "/.config/omarchyvr/anchor.toml", "[tag]\nid = 2\nsize_m = 0.1\n[reference]\npos = [0, 0, 0]\nquat = [0, 0, 0, 1]\n").has_value());
    CHECK(anchor::anchor_config_path() == d + "/.config/omarchyvr/anchor.toml");
    const auto a = anchor::Anchor::load();
    CHECK(a && a->id == 2);
    ::setenv("PIMAX_ANCHOR_CONFIG", (d + "/x/a.toml").c_str(), 1);
    CHECK(a && a->save() && cam::path_exists(d + "/x/a.toml") && !cam::path_exists(d + "/x/a.toml.tmp"));
    std::filesystem::remove_all(d);
}

PIBRIDGE_TEST_MAIN()
