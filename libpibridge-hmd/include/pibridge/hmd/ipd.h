// SPDX-License-Identifier: MIT
// The lens IPD motor, set from the app: one line "MM" (millimetres, e.g. 63.5) in $XDG_RUNTIME_DIR/pibridge/ipd-request
// (written to a temporary file and renamed into place). Whoever owns the headset takes it: pibridge-keepalived, or
// Monado's Pimax driver during an inside-out session (pimax_ipd_poll_request). The frame `58 06 5c 05 01 <mm*100 LE>`
// goes through the model's allowlist (`ipd_motor`), so a model without the motor refuses it there.
#pragma once

#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace pibridge::hmd::ipd {

inline constexpr std::string_view REQUEST_FILE = "ipd-request";
/// $XDG_RUNTIME_DIR/pibridge/ipd-request (/tmp/pibridge-<uid>/ipd-request without it).
std::filesystem::path request_path();
/// "63.5" -> 63.5; nullopt unless a finite number in 40..90 (the gate clamps to the model's range).
std::optional<float> parse_request(std::string_view text);
/// Take (rename away, read, remove) the request: nullopt when there is none, an error for a malformed one.
std::expected<std::optional<float>, std::string> take_request(const std::filesystem::path& path);

} // namespace pibridge::hmd::ipd
