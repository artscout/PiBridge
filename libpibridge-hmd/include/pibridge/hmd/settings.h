// SPDX-License-Identifier: MIT
// The shared settings file `~/.config/pibridge/pibridge.conf` (docs/CONVENTIONS.md): the app writes it, the
// daemons read it at start and on SIGHUP.
//
// Format: `key=value` lines; blank lines and lines whose first non-blank character is `#` are ignored; spaces around
// the key and the value are trimmed; the value is the rest of the line (so a path may contain `#` or spaces). Unknown
// keys are ignored, a later line wins over an earlier one. Numbers are read with std::from_chars, so a decimal point
// works in every locale. A bad value keeps the default and adds a plain-words warning.
//
// Booleans: on/off, 1/0, true/false, yes/no (any case).
#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::hmd {

struct Settings {
    /// The process whose life the daemons follow; empty = always on.
    std::string follow = "vrserver";
    /// 72 or 90.
    int refresh_hz = 90;
    bool audio_to_headset = true;
    /// mic_to_headset: the headset's microphone is the default input while it is on.
    bool mic_to_headset = true;
    bool eye_tracking = true;
    /// Path of the eye start frame list; empty = the default path.
    std::string eye_start_list;
    /// Path of the eye profile; empty = the default path.
    std::string eye_profile;
    bool eye_flip_x = true;
    /// The original Crystal (P3B): the tracking it is switched to when it connects: "lighthouse" (the faceplate,
    /// SteamVR's lighthouse driver + CustomHeadsetOpenVR), "slam" (its own inside-out tracking) or "keep" (as it is).
    std::string crystal_tracking = "lighthouse";
    /// The original Crystal: start SteamVR when the headset connects (it is switched on with its own button).
    bool crystal_autostart_steamvr = true;

    /// Bad lines and values, in plain words with their line number.
    std::vector<std::string> warnings;

    /// Parse the text of a settings file (defaults for everything it does not set).
    static Settings parse(std::string_view text);
    /// Read a settings file; a missing file gives the defaults (no warning), an unreadable one a warning.
    static Settings load(const std::filesystem::path& path);
    /// $XDG_CONFIG_HOME/pibridge/pibridge.conf, or ~/.config/pibridge/pibridge.conf.
    static std::filesystem::path default_path();
    /// The config folder ($XDG_CONFIG_HOME/pibridge or ~/.config/pibridge).
    static std::filesystem::path config_dir();
    /// The file of PiBridge's earlier name, read while default_path() does not exist yet:
    /// $XDG_CONFIG_HOME/pimax-linux/pimax-linux.conf (or under ~/.config).
    static std::filesystem::path legacy_path();
    /// load(path), except that for the default path, while it does not exist, the legacy file is read when it is
    /// there. The app writes only the new file, so the first change in the app moves the settings over.
    static Settings load_or_legacy(const std::filesystem::path& path);
};

/// A boolean settings value (on/off, 1/0, true/false, yes/no); false in `ok` when it is none of them.
bool parse_bool(std::string_view v, bool& ok);

} // namespace pibridge::hmd
