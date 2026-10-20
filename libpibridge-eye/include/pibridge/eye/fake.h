// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/fake.rs (MIT OR Apache-2.0).
//
// An in-process EyeChip for tests: it frames and answers TTP requests like the real one and, once the profile upload
// (or any configured opcode) completed, emits gaze notifications at the real rate.
//
// It also records every byte the client sent, so tests can assert that a replay sent exactly the plan and nothing
// else. The notification payload comes from `payload` (default: a minimal stream-0x0505 payload); the realistic
// synthetic gaze sample of the Rust fake (FakeChip::sample + gaze::encode_payload) belongs to libpibridge-eyemath, and phase 2
// plugs it in through that hook.
#pragma once

#include "pibridge/eye/transport.h"
#include "pibridge/eye/ttp.h"

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <variant>
#include <vector>

namespace pibridge::eye::fake {

/// Nominal period of the gaze stream (120.5 Hz), microseconds (gaze.rs NOMINAL_PERIOD_US).
inline constexpr std::int64_t NOMINAL_PERIOD_US = 8300;
/// Type id of the gaze compound (gaze.rs TYPE_GAZE).
inline constexpr std::uint16_t TYPE_GAZE = 0x0bb8;

/// A control transfer the client sent: setup and data stage.
struct SentControl {
    Setup setup{};
    Bytes data;
    bool operator==(const SentControl&) const = default;
};
/// A bulk OUT transfer the client sent: endpoint and bytes.
struct SentBulkOut {
    std::uint8_t ep = 0;
    Bytes data;
    bool operator==(const SentBulkOut&) const = default;
};
/// Everything the client sent.
using Sent = std::variant<SentControl, SentBulkOut>;

/// Payload of notification number `n` with device timestamp `timestamp_us`.
using PayloadFn = std::function<Bytes(std::uint32_t n, std::int64_t timestamp_us)>;

/// The default notification payload: `00 00` and an empty gaze compound header (count 0, TYPE_GAZE).
Bytes minimal_gaze_payload(std::uint32_t n, std::int64_t timestamp_us);

/// Fake device.
class FakeChip final : public Transport {
public:
    /// A chip that starts streaming after opcode 0x456.
    FakeChip();

    /// Everything received, in order.
    std::vector<Sent> sent;
    /// Opcodes whose response gets this status instead of 1.
    std::map<std::uint32_t, std::uint32_t> status_override;
    /// Opcodes that never get a response.
    std::vector<std::uint32_t> silent;
    /// Opcode whose completion starts the gaze stream (nullopt: never; the profile upload 0x456 by default).
    std::optional<std::uint32_t> start_on = 0x456;
    /// Stream gaze from the beginning (a session left open by an earlier run).
    bool streaming = false;
    /// Emitted frame period.
    Duration period = std::chrono::microseconds(NOMINAL_PERIOD_US);
    /// Number of notifications emitted so far.
    std::uint32_t emitted = 0;
    /// Notification payload source.
    PayloadFn payload = minimal_gaze_payload;
    /// Payload of the response to these opcodes (default: none). Scripts the calibration answers, e.g. the profile
    /// download 0x44c in the 0x456 layout (frames::profile_payload of a synthetic body).
    std::map<std::uint32_t, Bytes> reply_payload;
    /// Header aux word of the response to these opcodes (default 0; the chip sets one in its 0x407 / 0x42e answers).
    std::map<std::uint32_t, std::uint32_t> reply_aux;
    /// Body bytes per IN frame of a response: the real chip splits the 34-44 KB download into several frames.
    std::size_t in_max_body = std::size_t{1} << 20;

    Result<Bytes> control(const Setup& setup, ByteView out, Duration timeout) override;
    Result<void> bulk_out(std::uint8_t ep, ByteView data, Duration timeout) override;
    Result<std::optional<Bytes>> bulk_in(std::uint8_t ep, std::size_t max_len, Duration timeout) override;

private:
    Bytes notification();
    void answer(const ttp::Message& req);

    ttp::StreamDecoder inbound_{ttp::Direction::Out};
    std::deque<Bytes> queue_;
    std::int64_t next_ts_ = 50'000'000;
};

} // namespace pibridge::eye::fake
