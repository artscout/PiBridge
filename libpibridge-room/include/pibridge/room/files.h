// SPDX-License-Identifier: MIT
// Where the room files are and how they are written. Every path comes from an Env (home and the XDG variables), so
// tests and the app's fake mode point everything at a temporary directory.
#pragma once

#include <ctime>
#include <expected>
#include <string>
#include <vector>

namespace pibridge::room {

struct Env {
    std::string home;
    std::string xdgConfigHome; // "" = home/.config
    std::string xdgDataHome;   // "" = home/.local/share
    /// HOME, XDG_CONFIG_HOME and XDG_DATA_HOME of this process.
    static Env fromProcess();
    std::string configHome() const;
    std::string dataHome() const;
};

/// SteamVR's config folders on Linux, existing ones only, without duplicates, in this order: the "config" entries of
/// openvrpaths.vrpath (configHome/openvr/), dataHome/Steam/config, home/.steam/steam/config. The chaperone files
/// (chaperone_info*.vrchap) and steamvr.vrsettings are there.
std::vector<std::string> steamConfigDirs(const Env& env);
/// Our saved rooms: configHome/pibridge/rooms.
std::string roomsDir(const Env& env);

std::expected<std::string, std::string> readFile(const std::string& path);
/// Write through `path`.tmp and rename, so a reader never sees half a file.
std::expected<void, std::string> writeFileAtomic(const std::string& path, const std::string& data);
/// "20261019-141205" in local time (the suffix of the .bak-* copies, as ZoneTweak names them).
std::string backupStamp(std::time_t t);
/// The .bak-* copies of `path`, oldest first (by the stamp in the name).
std::vector<std::string> backupsOf(const std::string& path);

} // namespace pibridge::room
