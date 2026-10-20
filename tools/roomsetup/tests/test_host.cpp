// SPDX-License-Identifier: MIT
// The SteamVR glue with a fake helper runner: nothing is started.
#include "pibridge/room/files.h"
#include "pibridge_test.h"
#include "host.h"

#include <filesystem>

using namespace pibridge::roomsetup;

namespace {

struct Fake {
    std::vector<std::vector<std::string>> calls;
    HelperResult answer{0, "1234\n"};
    HelperRunner runner() {
        return [this](const std::vector<std::string>& a) {
            calls.push_back(a);
            return answer;
        };
    }
};

std::string tempDir() {
    std::string t = (std::filesystem::temp_directory_path() / "pibridge-roomsetup-host-XXXXXX").string();
    return ::mkdtemp(t.data());
}

} // namespace

TEST(universe) {
    Fake f;
    CHECK(currentUniverse(f.runner()) == "1234");
    CHECK(f.calls.at(0).at(0) == "--vr-universe");
    f.answer = {1, ""};
    CHECK(currentUniverse(f.runner()).empty());
    f.answer = {0, "garbage"};
    CHECK(currentUniverse(f.runner()).empty());
}

TEST(reload) {
    Fake f;
    CHECK(reloadAfterSave(f.runner(), true) == "SteamVR reloaded it");
    CHECK(f.calls.at(0).at(0) == "--vr-reload");
    f.answer = {-1, ""};
    CHECK(reloadAfterSave(f.runner(), true) == "Restart SteamVR to see it");
    CHECK(reloadAfterSave(f.runner(), false).find("next start") != std::string::npos);
    CHECK(f.calls.size() == 2); // not asked without SteamVR
}

TEST(zone_through_steamvr) {
    const std::string d = tempDir();
    Fake f;
    ToolZoneControl z(d + "/steamvr.vrsettings", d + "/state", true, f.runner(), [] { return std::time_t{1760000000}; });
    CHECK(z.setWarnDistance(0.456).has_value());
    CHECK(f.calls.back() == (std::vector<std::string>{"--vr-zone", "fade=0.46"}));
    CHECK(z.values().fade == 0.46);
    CHECK(z.setZoneOn(false).has_value());
    CHECK(f.calls.back().at(1) == "alpha=0");
    CHECK(!z.values().zoneOn());
    CHECK(z.setZoneOn(true).has_value());
    CHECK(f.calls.back().at(1) == "alpha=default");
    CHECK(!std::filesystem::exists(d + "/steamvr.vrsettings")); // SteamVR's file is SteamVR's while it runs
    f.answer = {1, ""};
    CHECK(!z.setWarnDistance(0.5).has_value());
    std::filesystem::remove_all(d);
}

TEST(zone_into_the_file_without_steamvr) {
    const std::string d = tempDir();
    Fake f;
    ToolZoneControl z(d + "/steamvr.vrsettings", d + "/state", false, f.runner(), [] { return std::time_t{1760000000}; });
    CHECK(z.setWarnDistance(0.3).has_value());
    CHECK(f.calls.empty());
    CHECK(z.values().fade == 0.3);
    CHECK(pibridge::room::readFile(d + "/steamvr.vrsettings").has_value());
    std::filesystem::remove_all(d);
}

PIBRIDGE_TEST_MAIN()
