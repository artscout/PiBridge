// SPDX-License-Identifier: MIT
// The inside-out pause marker: while an inside-out session runs (the app's START TRACKING with SteamVR + Monado, or
// "Monado directly"), Monado's Pimax driver owns the headset's HID and the eye chip. pibridge-keepalived and pibridge-eyed
// must not talk to the headset then: they close the device, report "paused" in their status file and resume when the
// marker is gone.
//
// $XDG_RUNTIME_DIR/pibridge/inside-out.active, written by the app before it switches SteamVR's drivers or starts the
// camera broker, removed on every stop / restore path:
//   pibridge-inside-out 1
//   owner <pid of the PiBridge app>
//   variant steamvr | monado
// A marker whose owner is not running (PiBridge crashed) is stale and does not count (the app's next start removes it).
#pragma once

#include "pibridge_status.h"

#include <cerrno>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace pibridge::pause {

inline constexpr std::string_view kWords = "paused: inside-out tracking";

inline std::filesystem::path marker_path() {
    return pibridge::status::runtime_dir() / "inside-out.active";
}

struct Marker {
    long owner = 0;
    std::string variant;
};

inline std::string marker_text(long owner, std::string_view variant) {
    return "pibridge-inside-out 1\nowner " + std::to_string(owner) + "\nvariant " + std::string(variant) + "\n";
}

inline std::optional<Marker> parse_marker(std::string_view text) {
    std::istringstream in{std::string(text)};
    std::string line;
    if (!std::getline(in, line) || line != "pibridge-inside-out 1") {
        return std::nullopt;
    }
    Marker m;
    while (std::getline(in, line)) {
        if (line.starts_with("owner ")) {
            try {
                m.owner = std::stol(line.substr(6));
            } catch (...) {
                return std::nullopt;
            }
        } else if (line.starts_with("variant ")) {
            m.variant = line.substr(8);
        }
    }
    if (m.owner <= 0) {
        return std::nullopt;
    }
    return m;
}

inline bool pid_alive(long pid) {
    return pid > 0 && (::kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM);
}

/// The marker when it is there and its owner runs; nullopt otherwise (no marker, broken, or stale).
inline std::optional<Marker> active_marker(const std::filesystem::path& p = marker_path(),
                                           const std::function<bool(long)>& alive = pid_alive) {
    std::ifstream f(p);
    if (!f) {
        return std::nullopt;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    auto m = parse_marker(ss.str());
    if (!m || !(alive ? alive(m->owner) : pid_alive(m->owner))) {
        return std::nullopt;
    }
    return m;
}

} // namespace pibridge::pause
