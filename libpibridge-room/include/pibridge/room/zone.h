// SPDX-License-Identifier: MIT
// Ported from ZoneTweak pimaxprofile.go (MIT, Copyright (c) 2026 killdano; see NOTICE), for SteamVR.
//
// The zone settings ZoneTweak edits in Pimax's profile.json, as SteamVR has them in steamvr.vrsettings, section
// "collisionBounds":
//   warning distance   "CollisionBoundsFadeDistance" (metres; SteamVR's default when the key is missing: 0.7)
//   zone on / off      "CollisionBoundsColorGammaA" (the boundary's opacity 0..255): off writes 0, on removes the
//                      key (SteamVR's own default) or puts back the value it had before it was switched off.
// SteamVR keeps its settings in memory and writes the file itself, so the file is only written while SteamVR is not
// running (the app checks); while it runs the in-headset tool sends the same keys through IVRSettings instead.
#pragma once

#include "pibridge/room/json.h"

#include <ctime>
#include <expected>
#include <optional>
#include <string>

namespace pibridge::room {

inline constexpr const char* kZoneSection = "collisionBounds";
inline constexpr const char* kKeyFade = "CollisionBoundsFadeDistance";
inline constexpr const char* kKeyAlpha = "CollisionBoundsColorGammaA";
inline constexpr double kWarnDistMin = 0.05;
inline constexpr double kWarnDistMax = 2.0;
inline constexpr double kSteamVrDefaultFade = 0.7;

/// Clamp to 0.05..2 m and round to 1 cm (ZoneTweak).
double clampWarnDistance(double v);

struct ZoneValues {
    std::optional<double> fade; // nullopt: SteamVR's default
    std::optional<int> alpha;   // nullopt: SteamVR's default (visible)
    bool zoneOn() const { return !alpha || *alpha > 0; }
    double warnDistance() const { return fade.value_or(kSteamVrDefaultFade); }
};

ZoneValues readZone(const Json& settings);
/// nullopt clears the key (SteamVR's default).
void setFade(Json& settings, std::optional<double> metres);
/// Off: alpha 0 (the old value is returned so it can be put back). On: `restore` or the key removed.
std::optional<int> setZoneOn(Json& settings, bool on, std::optional<int> restore);

/// steamvr.vrsettings on disk: read, change, back up once (.bak-stamp), write atomically.
class VrSettingsFile {
public:
    explicit VrSettingsFile(std::string path) : path_(std::move(path)) {}
    const std::string& path() const { return path_; }
    std::expected<ZoneValues, std::string> read() const;
    std::expected<double, std::string> setWarnDistance(std::optional<double> metres, std::time_t now);
    /// `stateDir` keeps the opacity to put back (zone-alpha.txt).
    std::expected<void, std::string> setZoneOn(bool on, const std::string& stateDir, std::time_t now);

private:
    std::expected<Json, std::string> load() const;
    std::expected<void, std::string> write(const Json& j, std::time_t now);
    std::string path_;
    bool backedUp_ = false;
};

} // namespace pibridge::room
