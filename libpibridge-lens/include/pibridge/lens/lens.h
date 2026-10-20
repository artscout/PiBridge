// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/lib.rs (MIT OR Apache-2.0).
//
// The lens model of the Monado Pimax driver (`pimax_lens_rs.h`, monado-abi/src/lens_capi.cpp):
// * config.h: the `lens.conf` parser; model.h: Lens::compute, the distortion callback; lut.h: exact distortion
//   tables (`lut = FILE`, the Crystal Super); mask.h: visibility mask meshes; grid.h (+ font.h, png.h): the debug
//   pictures of PIMAX_LENS_DEBUG_GRID; pvrdump.h: the PVR dump reader (LUT tests).
// * here: loading the configuration like the driver does, with the log lines it prints.
#pragma once

#include "pibridge/lens/config.h"
#include "pibridge/lens/model.h"

#include <string>
#include <string_view>
#include <vector>

namespace pibridge::lens {

/// Severity of a Message (Monado's U_LOG_I / U_LOG_W).
enum class Level { Info, Warn };

/// A log line produced while loading.
struct Message {
    Level level = Level::Info;
    std::string text;
    bool operator==(const Message&) const = default;
};

/// Result of load().
struct Loaded {
    Lens lens;
    std::string path;
    /// The config file existed and was read.
    bool file_found = false;
    /// No user file: the shipped Dream Air profile (reference_lens_conf()) is in use.
    bool reference = false;
    std::vector<Message> messages;
};

/// The log lines describing the active configuration of one eye (the first two have the format of the C driver).
std::vector<std::string> eye_log_lines(const LensConfig& cfg, std::size_t e);

/// The Pimax Dream Air lens profile shipped with omarchyvr (devices/dream-air/reference/lens.conf, embedded).
std::string_view reference_lens_conf();

/// Load the configuration like the driver does: defaults, then the file from `PIMAX_LENS_CONF` /
/// `$XDG_CONFIG_HOME/omarchyvr/lens.conf` / `~/.config/omarchyvr/lens.conf`, then `PIMAX_LENS_OFF`. Without
/// `PIMAX_LENS_CONF` and without the user's file, the shipped Dream Air profile is used (`PIMAX_NO_REFERENCE=1`
/// keeps the code defaults).
Loaded load(float aspect, const Env& env);
/// The shipped Dream Air profile as a ready config (and the parse warnings, none for the shipped file).
std::pair<LensConfig, std::vector<Warning>> reference_config(float aspect);
/// load() when the user has no lens.conf: the shipped profile, parsed like a file.
Loaded load_reference(float aspect, const Env& env, std::string path);
/// Like load() with an explicit path (on the code defaults).
Loaded load_file(float aspect, const Env& env, std::string path);

/// `PIMAX_LENS_DEBUG_GRID=<dir>`: write the UV grid pictures and the text summary of both eyes into `dir`
/// (`div` divides the 3840x3552 panel size). Returns the log lines.
std::vector<Message> write_debug_grid(const Lens& lens, const std::string& dir, std::uint32_t div);

namespace detail {
/// std::fs::create_dir_all (the Rust algorithm, so the error is the same one); "" on success, else the error text.
std::string create_dir_all(const std::string& path);
/// Path::parent: nullopt for "" and "/".
std::optional<std::string> path_parent(const std::string& p);
} // namespace detail

} // namespace pibridge::lens
