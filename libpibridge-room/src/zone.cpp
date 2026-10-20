// SPDX-License-Identifier: MIT
// Ported from ZoneTweak pimaxprofile.go (MIT, Copyright (c) 2026 killdano; see NOTICE), for SteamVR.
#include "pibridge/room/zone.h"

#include "pibridge/room/files.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>

namespace pibridge::room {

double clampWarnDistance(double v) { return std::round(std::clamp(v, kWarnDistMin, kWarnDistMax) * 100) / 100; }

ZoneValues readZone(const Json& settings) {
    ZoneValues z;
    const Json& cb = settings.get(kZoneSection);
    if (const Json& f = cb.get(kKeyFade); f.isNumber()) {
        z.fade = f.num();
    }
    if (const Json& a = cb.get(kKeyAlpha); a.isNumber()) {
        z.alpha = static_cast<int>(std::lround(a.num()));
    }
    return z;
}

void setFade(Json& settings, std::optional<double> metres) {
    Json& cb = settings[kZoneSection];
    if (!metres) {
        cb.erase(kKeyFade);
        return;
    }
    cb[kKeyFade] = Json::number(clampWarnDistance(*metres));
}

std::optional<int> setZoneOn(Json& settings, bool on, std::optional<int> restore) {
    Json& cb = settings[kZoneSection];
    const ZoneValues before = readZone(settings);
    if (!on) {
        cb[kKeyAlpha] = Json::number(0);
        return before.alpha && *before.alpha > 0 ? before.alpha : std::nullopt;
    }
    if (restore && *restore > 0) {
        cb[kKeyAlpha] = Json::number(*restore);
    } else {
        cb.erase(kKeyAlpha);
    }
    return std::nullopt;
}

std::expected<Json, std::string> VrSettingsFile::load() const {
    auto t = readFile(path_);
    if (!t) {
        return Json::object(); // SteamVR creates it on its first start; every key has its default then
    }
    auto j = parseJson(*t);
    if (!j) {
        return std::unexpected(std::format("steamvr.vrsettings: {}", j.error()));
    }
    if (!j->isObject()) {
        return std::unexpected(std::string("steamvr.vrsettings is not a JSON object"));
    }
    return j;
}

std::expected<void, std::string> VrSettingsFile::write(const Json& j, std::time_t now) {
    if (!backedUp_) {
        if (auto b = readFile(path_)) {
            if (auto w = writeFileAtomic(path_ + ".bak-" + backupStamp(now), *b); !w) {
                return std::unexpected(std::format("couldn't back up {}: {}", path_, w.error()));
            }
        }
        backedUp_ = true;
    }
    return writeFileAtomic(path_, j.dump() + "\n");
}

std::expected<ZoneValues, std::string> VrSettingsFile::read() const {
    auto j = load();
    if (!j) {
        return std::unexpected(j.error());
    }
    return readZone(*j);
}

std::expected<double, std::string> VrSettingsFile::setWarnDistance(std::optional<double> metres, std::time_t now) {
    auto j = load();
    if (!j) {
        return std::unexpected(j.error());
    }
    setFade(*j, metres);
    if (auto w = write(*j, now); !w) {
        return std::unexpected(w.error());
    }
    return metres ? clampWarnDistance(*metres) : kSteamVrDefaultFade;
}

std::expected<void, std::string> VrSettingsFile::setZoneOn(bool on, const std::string& stateDir, std::time_t now) {
    auto j = load();
    if (!j) {
        return std::unexpected(j.error());
    }
    const std::string keep = stateDir + "/zone-alpha.txt";
    std::optional<int> restore;
    if (auto t = readFile(keep)) {
        try {
            restore = std::stoi(*t);
        } catch (...) {
            restore.reset();
        }
    }
    const std::optional<int> old = room::setZoneOn(*j, on, restore);
    if (auto w = write(*j, now); !w) {
        return w;
    }
    std::error_code ec;
    if (!on && old) {
        std::filesystem::create_directories(stateDir, ec);
        (void)writeFileAtomic(keep, std::to_string(*old));
    } else if (on) {
        std::filesystem::remove(keep, ec);
    }
    return {};
}

} // namespace pibridge::room
