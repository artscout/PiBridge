// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-autod/src/{pairing,unpair}.rs (MIT OR Apache-2.0).
//
// Controller pairing and unpairing: the pure parts (firmware log verdicts, sides, the request file the app writes).
//
// Pair (StartPair `58 04 04 00 SS`, SS 2 left / 3 right): the headset acks `4c 04 04 00 01 07` and listens; its
// firmware log (IF3) says `eSpiCmdStartPair` / `Pairing device = N`, then `pair finished` + `radio_pair_success_process`
// (success) or `radio_pair_cancel_or_timeout_process` (about 30 s). One send per user request, never retried.
// Unpair (`58 03 05 SS`): reply `4c 04 05 SS 01 07`, the controller stream stops 20 ms later.
//
// Who sends them: pibridge-keepalived, which holds the headset when no inside-out session runs (docs/keepalived.md). The
// app writes one request line to `$XDG_RUNTIME_DIR/pibridge/controller-request` and reads the result from
// keepalived.json (`ctrl_*`). During an inside-out session Monado's Pimax driver owns the headset and the request is
// refused with a plain reason.
#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace pibridge::hmd::pairing {

/// Firmware log verdicts.
enum class LogEvent { Started, Success, TimeoutOrCancel };

/// Classify one firmware log line (IF3).
std::optional<LogEvent> parse_log(std::string_view line);

/// The app's timer for one pairing attempt (the firmware gives up after about 30 s).
inline constexpr std::chrono::seconds TIMEOUT{35};
/// How long the unpair reply is waited for.
inline constexpr std::chrono::milliseconds UNPAIR_REPLY{1000};

/// What the user should do while the headset listens.
inline constexpr std::string_view INSTRUCTION =
    "Press and hold the controller's pairing buttons (see the Pimax manual) within 30 s";

/// 2 for "left", 3 for "right".
std::optional<std::uint8_t> slot_of(std::string_view side);
/// "left" for slot 2, "right" otherwise.
std::string_view side_of(std::uint8_t slot);

enum class Op { Pair, Unpair };
std::string_view op_name(Op op);

/// One request of the app: `ID pair|unpair left|right\n` (ID: a positive number the result echoes).
struct Request {
    std::uint64_t id = 0;
    Op op = Op::Pair;
    std::uint8_t slot = 0;
    bool operator==(const Request&) const = default;
};

/// File name (in `$XDG_RUNTIME_DIR/pibridge`) of the request.
inline constexpr std::string_view REQUEST_FILE = "controller-request";

std::string format_request(const Request& r);
std::optional<Request> parse_request(std::string_view s);
/// Take (read and remove) a waiting request; nullopt when there is none, an error text for a malformed one (removed
/// too) or a file that cannot be read.
std::expected<std::optional<Request>, std::string> take_request(const std::filesystem::path& path);

} // namespace pibridge::hmd::pairing
