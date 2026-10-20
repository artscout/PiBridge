// SPDX-License-Identifier: MIT
// The zone settings in steamvr.vrsettings (ZoneTweak's pimaxprofile.go ported to SteamVR's keys).
#include "pibridge/room/files.h"
#include "pibridge/room/zone.h"
#include "pibridge_test.h"
#include "fixtures.h"

using namespace pibridge::room;

TEST(clamp) {
    CHECK_NEAR(clampWarnDistance(0.01), 0.05, 1e-12);
    CHECK_NEAR(clampWarnDistance(3), 2.0, 1e-12);
    CHECK_NEAR(clampWarnDistance(0.4449), 0.44, 1e-12);
}

TEST(file_round_trip) {
    fixtures::TempDir t;
    const std::string p = t.path + "/steamvr.vrsettings";
    fixtures::write(p, R"({ "steamvr" : { "enableHomeApp" : false }, "collisionBounds" : { "CollisionBoundsStyle" : 2 } })");
    VrSettingsFile f(p);
    auto v = f.read();
    CHECK(v.has_value() && !v->fade && v->zoneOn());
    CHECK_NEAR(v->warnDistance(), kSteamVrDefaultFade, 1e-12);
    CHECK_NEAR(f.setWarnDistance(0.42, 1760000000).value_or(0), 0.42, 1e-12);
    CHECK(f.setZoneOn(false, t.path + "/state", 1760000000).has_value());
    v = f.read();
    CHECK(v->fade.has_value() && std::fabs(*v->fade - 0.42) < 1e-12);
    CHECK(!v->zoneOn());
    auto doc = parseJson(readFile(p).value_or(""));
    CHECK(doc->get("steamvr").get("enableHomeApp").isBool()); // other settings stay
    CHECK(doc->get("collisionBounds").get("CollisionBoundsStyle").num() == 2);
    CHECK(f.setZoneOn(true, t.path + "/state", 1760000000).has_value());
    const auto on = f.read();
    CHECK(on.has_value() && on->zoneOn());
    CHECK(!parseJson(readFile(p).value_or(""))->get("collisionBounds").has(kKeyAlpha)); // back to SteamVR's default
    CHECK(f.setWarnDistance(std::nullopt, 1760000000).has_value());
    const auto cleared = f.read();
    CHECK(cleared.has_value() && !cleared->fade);
    CHECK(backupsOf(p).size() == 1);
}

TEST(zone_off_puts_back_the_old_opacity) {
    fixtures::TempDir t;
    const std::string p = t.path + "/steamvr.vrsettings";
    fixtures::write(p, R"({ "collisionBounds" : { "CollisionBoundsColorGammaA" : 120 } })");
    VrSettingsFile f(p);
    CHECK(f.setZoneOn(false, t.path + "/state", 1760000000).has_value());
    const auto off = f.read();
    CHECK(off.has_value() && off->alpha == 0);
    CHECK(f.setZoneOn(true, t.path + "/state", 1760000000).has_value());
    const auto on = f.read();
    CHECK(on.has_value() && on->alpha == 120);
}

TEST(missing_file_is_defaults) {
    fixtures::TempDir t;
    VrSettingsFile f(t.path + "/none.vrsettings");
    CHECK(f.read().has_value());
    CHECK(f.setWarnDistance(1.0, 1760000000).has_value());
    const auto v = f.read();
    CHECK(v.has_value() && v->fade == 1.0);
}

PIBRIDGE_TEST_MAIN()
