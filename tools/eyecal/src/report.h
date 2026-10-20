// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/report.rs (MIT OR Apache-2.0).
//
// Results: the summary (terminal text, head-locked lines, the notes stored in the calibration file) and the JSON
// file. Accuracy is always the *validation* error (targets that were not used for the fit).
#pragma once

#include "pibridge/eyemath/calib.h"

#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eyecal {

class Procedure;

/// Seconds since the Unix epoch.
std::uint64_t unix_now();
/// `YYYYMMDD-HHMMSS` (UTC).
std::string stamp(std::uint64_t unix);
/// Write `text` with a `.bak-<stamp>` copy of an existing file (temporary file + rename); returns the backup path.
std::expected<std::optional<std::filesystem::path>, std::string> write_with_backup(const std::filesystem::path& path,
                                                                                   std::string_view text,
                                                                                   std::string_view stamp);
/// write_with_backup() stamped now.
std::expected<std::optional<std::filesystem::path>, std::string> write_with_backup(const std::filesystem::path& path,
                                                                                   std::string_view text);
/// `~/.local/share/pibridge/eye-calibration/` (or `.` without `$HOME`).
std::filesystem::path results_dir();

/// The summary in three forms.
struct Summary {
    /// Terminal text.
    std::string text;
    /// Head-locked lines (upper case, short).
    std::vector<std::string> hud;
    /// Comment lines for the calibration file.
    std::vector<std::string> notes;
    /// Combined validation error per target.
    pibridge::eyemath::Stats accuracy;
    /// Same targets, raw gaze.
    pibridge::eyemath::Stats raw;
};

/// Build the summary of a procedure.
Summary summary(const Procedure& p);

/// The results file.
std::string json(const Procedure& p);

} // namespace pibridge::eyecal
