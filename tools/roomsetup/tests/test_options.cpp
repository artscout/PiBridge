// SPDX-License-Identifier: MIT
#include "pibridge_test.h"
#include "options.h"

using namespace pibridge::roomsetup;

TEST(defaults_and_runs) {
    auto o = parseArgs({});
    CHECK(o && o->run == Run::Headset && o->resDiv == 1);
    CHECK(parseArgs({"--selftest"})->run == Run::SelfTest);
    CHECK(parseArgs({"--list"})->run == Run::List);
    CHECK(parseArgs({"--vr-universe"})->run == Run::VrUniverse);
    CHECK(parseArgs({"--vr-reload"})->run == Run::VrReload);
    CHECK(parseArgs({"--help"})->help);
}

TEST(values) {
    auto o = parseArgs({"--universe", "2", "--config-dir", "/a", "--config-dir", "/b", "--rooms", "/r", "--res-div", "2",
                        "--no-openvr"});
    CHECK(o.has_value());
    CHECK(o->universe == "2");
    CHECK(o->configDirs.size() == 2);
    CHECK(o->roomsDir == "/r");
    CHECK(o->resDiv == 2);
    CHECK(o->noOpenVr);
    auto u = parseArgs({"--use", "Door", "open", "--rooms", "/r"});
    CHECK(u && u->run == Run::Use && u->useName == "Door open" && u->roomsDir == "/r");
    CHECK(parseArgs({"--selftest", "--snapshot", "/tmp/x"})->snapshotDir == "/tmp/x");
}

TEST(errors) {
    CHECK(!parseArgs({"--universe"}).has_value());
    CHECK(!parseArgs({"--bogus"}).has_value());
    CHECK(!parseArgs({"--res-div", "0"}).has_value());
    CHECK(!parseArgs({"--snapshot", "/x"}).has_value()); // only with --selftest
    CHECK(!parseArgs({"--vr-zone", "fade=9"}).has_value());
    CHECK(!usage().empty());
}

TEST(zone_args) {
    auto z = parseZoneArg("fade=0.5");
    CHECK(z && z->key == ZoneArg::Fade && z->value == 0.5);
    CHECK(parseZoneArg("alpha=default")->value == std::nullopt);
    CHECK(parseZoneArg("alpha=0")->key == ZoneArg::Alpha);
    CHECK(!parseZoneArg("alpha=300").has_value());
    CHECK(!parseZoneArg("size=1").has_value());
    CHECK(!parseZoneArg("fade").has_value());
    CHECK(!parseZoneArg("fade=x").has_value());
}

TEST(map_options) {
    auto o = parseArgs({"--map", "--room", "office", "--roommap", "/t/pibridge-roommap", "--camera", "/t/a.pxcam", "--ring", "/t/r.head",
                        "--map-seconds", "40"});
    CHECK(o && o->run == Run::Map && o->room == "office" && o->roommapTool == "/t/pibridge-roommap" && o->camera == "/t/a.pxcam" &&
          o->ringFile == "/t/r.head" && o->mapSeconds == "40");
    auto d = parseArgs({"--map"});
    CHECK(d && d->room == "home" && d->camera == "broker" && d->roommapTool.empty());
    CHECK(!parseArgs({"--map", "--room", "../x"}));
    CHECK(!parseArgs({"--map", "--map-seconds", "5"}));
    CHECK(!parseArgs({"--map", "--room"}));
    auto f = parseArgs({"--floor-height", "--eye-height", "1.68", "--room", "home"});
    CHECK(f && f->run == Run::FloorHeight && f->eyeHeight && *f->eyeHeight == 1.68);
    CHECK(!parseArgs({"--floor-height"}));
    CHECK(!parseArgs({"--map", "--eye-height", "3"}));
    CHECK(parseArgs({"--map", "--eye-height", "1.6"}).value_or(Options{}).eyeHeight == 1.6);
}

PIBRIDGE_TEST_MAIN()
