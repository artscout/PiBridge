// SPDX-License-Identifier: MIT
// How pibridge-roomsetup talks to SteamVR around the editor: the universe to edit, the reload after a save, the zone
// settings. Each goes through the helper process (openvr_helper.h); the runner is injected so the tests use a fake.
#pragma once

#include "pibridge/room/editor.h"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::roomsetup {

struct HelperResult {
    int code = -1; // exit code; -1 when it could not run or timed out
    std::string out;
};
using HelperRunner = std::function<HelperResult(const std::vector<std::string>& args)>;

/// Run this binary again with `args` (posix_spawn of /proc/self/exe), wait at most `timeoutMs`.
HelperResult runSelf(const std::vector<std::string>& args, int timeoutMs = 6000);

/// The universe SteamVR tracks in, or "" when it does not answer.
std::string currentUniverse(const HelperRunner& run);

/// Words for the status after a save: SteamVR reloaded it, or what to do.
std::string reloadAfterSave(const HelperRunner& run, bool steamvr);

/// Zone settings: read from steamvr.vrsettings; while SteamVR runs, written through IVRSettings (the helper),
/// otherwise into the file.
class ToolZoneControl : public room::ZoneControl {
public:
    ToolZoneControl(std::string settingsFile, std::string stateDir, bool steamvr, HelperRunner run,
                    std::function<std::time_t()> clock);
    room::ZoneValues values() override;
    std::expected<double, std::string> setWarnDistance(std::optional<double> metres) override;
    std::expected<void, std::string> setZoneOn(bool on) override;

private:
    room::FileZoneControl file_;
    std::string stateDir_;
    bool steamvr_;
    HelperRunner run_;
    std::optional<room::ZoneValues> live_; // what was sent while SteamVR runs (its file lags behind)
};

} // namespace pibridge::roomsetup
