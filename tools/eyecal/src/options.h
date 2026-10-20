// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/main.rs (MIT OR Apache-2.0): the command line, over the settings eyed reads.
//
// The tool opens the eye chip itself, so it takes the same start list and profile as pibridge-eyed: `eye_start_list`
// and `eye_profile` of pibridge.conf, else `eye-start-crystal-super.txt` / `eye-profile-crystal-super.bin` in
// ~/.config/pibridge (then ~/.config/pimax-linux, then ~/.config/omarchyvr). The environment (PIBRIDGE_EYE_FRAMES,
// PIBRIDGE_EYE_PROFILE, PIBRIDGE_EYE_FLIP_X; the PIMAX_EYE_* names when those are not set) wins over the settings, flags win
// over both. The chip's new profile is written where eyed reads it: `eye_profile` when set, else
// ~/.config/pibridge/eye-profile-crystal-super.bin.
#pragma once

#include "procedure.h"

#include "pibridge/hmd/settings.h"

#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eyecal {

/// The folders the default files are looked up in, first match wins (eyed's ConfigDirs).
struct Dirs {
    std::filesystem::path pibridge;
    std::filesystem::path legacy;
    std::filesystem::path omarchyvr;
    /// pibridge::hmd::Settings::config_dir() and its siblings `pimax-linux` and `omarchyvr`.
    static Dirs from_env();
    std::optional<std::filesystem::path> find(std::string_view name) const;
};

/// File names of the Crystal Super's start list and eye profile (the names eyed uses).
inline constexpr std::string_view START_LIST_FILE = "eye-start-crystal-super.txt";
inline constexpr std::string_view PROFILE_FILE = "eye-profile-crystal-super.bin";

/// What the run does.
enum class RunKind {
    /// The chip's own calibration (default).
    Chip,
    /// The host fit without chip calibration (the Rust tool's calibration; fallback).
    HostOnly,
    /// The 5-target drift update of the existing host map.
    Recenter,
};

struct Options {
    RunKind run = RunKind::Chip;
    /// The host fit's path (`--pattern`; the chip mode always uses the Pimax look).
    Pattern pattern = Pattern::Pimax;
    std::size_t points = 9;
    bool pursuit = false;
    /// Device-to-head x / y conversion (flag, else environment, else the settings' eye_flip_x / no flip).
    bool flip_x = true;
    bool flip_y = false;
    double head_limit_deg = 2.5;
    std::uint32_t res_div = 1;
    /// Where the host map is written (and, for --recenter, read).
    std::optional<std::filesystem::path> calibration;
    bool raw_log = true;
    bool simulate = false;
    std::uint64_t seed = 1;
    /// The chip's start list; empty = the built-in base list.
    std::optional<std::filesystem::path> frames;
    /// The profile uploaded at the chip start (the user's current one); empty = the list's own upload.
    std::optional<std::filesystem::path> profile;
    /// No profile upload at all.
    bool no_profile = false;
    /// Where the chip's new profile is written.
    std::filesystem::path profile_out;
    /// Mirrors of the chip's target table (first live test).
    bool chip_mirror_x = false;
    bool chip_mirror_y = false;
    /// The settings file.
    std::filesystem::path settings_path;
    bool help = false;
};

/// Environment lookup by the name without prefix ("EYE_FLIP_X"): PIBRIDGE_, else PIMAX_ (pibridge::env).
using EnvFn = std::function<const char*(std::string_view)>;

/// The command line over the settings and the environment. An error is a usage message.
std::expected<Options, std::string> parse_args(const std::vector<std::string>& args, const pibridge::hmd::Settings& settings,
                                               const Dirs& dirs, const EnvFn& env);
/// The settings path named by `--settings`, or the default one.
std::filesystem::path settings_path_of(const std::vector<std::string>& args);
std::string usage();

/// The procedure settings of `o` (for --recenter `existing` is filled by the caller).
Settings procedure_settings(const Options& o);

} // namespace pibridge::eyecal
