// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/haptics.rs (MIT OR Apache-2.0).
//
// Controller vibration rate limiter (`58 09 07 SS AMP DUR`, pibridge::hmd::haptic_command).
//
// Games stream an amplitude envelope: a new value every few ms per hand. The headset forwards each command over the
// controller radio, and on Windows a new one goes out every 8-10 ms with an 11-12 ms duration (capture 2026-10-15,
// take 15). The limiter keeps at most one command per MIN_GAP per hand: a request that comes too soon replaces the
// waiting one (newest wins) and goes out as soon as the gap has passed. Pure logic, the session thread owns the clock
// and the writes.
//
// The test pulse request file (`$XDG_RUNTIME_DIR/pimax-haptic-request`, `SLOT AMP MS\n`) is written by the apps'
// "Test vibration" button and taken by the Monado Pimax driver on its own session (pimax_haptic_poll_request).
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace pibridge::hmd::haptics {

using Clock = std::chrono::steady_clock;

/// Smallest spacing of two commands for the same hand.
inline constexpr std::chrono::milliseconds MIN_GAP{8};

/// One pulse: amplitude percent, duration ms.
using Pulse = std::pair<std::uint8_t, std::uint16_t>;
/// A command due: slot, amplitude percent, duration ms.
using Due = std::tuple<std::uint8_t, std::uint8_t, std::uint16_t>;

/// Per-hand limiter for slots 2 (left) and 3 (right); other slots are ignored.
class Limiter {
public:
    /// Queue a pulse for `slot`; nullopt (stop) drops a waiting one. False for an unknown slot.
    bool offer(std::uint8_t slot, std::optional<Pulse> pulse);
    /// The commands that may go out at `now`; marks them sent.
    std::vector<Due> take_due(Clock::time_point now);
    /// When the next waiting command may go out (nullopt = nothing waits).
    std::optional<Clock::time_point> next_due() const;
    /// Forget everything (link closed). The counters stay.
    void clear();

    /// Requests replaced by a newer one before they went out.
    std::uint64_t replaced = 0;
    /// Commands handed out by take_due.
    std::uint64_t sent = 0;

private:
    struct Slot {
        std::optional<Clock::time_point> last_sent;
        std::optional<Pulse> pending;
    };
    std::array<Slot, 2> slots_{};
};

/// File name (in `$XDG_RUNTIME_DIR`) of a test pulse request for the running VR session.
inline constexpr std::string_view REQUEST_FILE = "pimax-haptic-request";

/// `$XDG_RUNTIME_DIR/pimax-haptic-request` (`/tmp` when the variable is missing).
std::filesystem::path request_path();
/// The request line for `slot`, `amp` percent, `ms` milliseconds: "SLOT AMP MS\n".
std::string format_request(std::uint8_t slot, std::uint8_t amp, std::uint16_t ms);
/// Parse a request line: slot 2|3, amp 1..=100, ms 1..=65535, nothing else.
std::optional<Due> parse_request(std::string_view s);
/// Take (read and remove) a waiting request; nullopt when there is none, an error text for a malformed one (removed
/// too) or a file that cannot be read.
std::expected<std::optional<Due>, std::string> take_request(const std::filesystem::path& path);

} // namespace pibridge::hmd::haptics
