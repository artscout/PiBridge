// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/session.rs (MIT OR Apache-2.0).
//
// Running a replay plan against a Transport (TTP request/response with the captured status check) and reading the
// gaze stream (stream 0x0505 notifications after the subscription 0x4c4; 0x4ce unsubscribes).
//
// The gaze payload decoder (pimax-eye/src/gaze.rs) belongs to libpibridge-eyemath; this library hands every gaze notification
// to a caller-supplied GazeHandler, which decodes it (phase 2 wires eyemath's decoder in).
#pragma once

#include "pibridge/eye/error.h"
#include "pibridge/eye/replay.h"
#include "pibridge/eye/transport.h"
#include "pibridge/eye/ttp.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::eye::session {

/// Size of the IN buffer: larger than any frame seen on the stream.
inline constexpr std::size_t IN_BUFFER = 16384;

/// Subscribe stream (the start list's own 0x4c4 step subscribes the gaze stream 0x0505).
inline constexpr std::uint32_t OP_SUBSCRIBE = 0x4c4;
/// Unsubscribe stream.
inline constexpr std::uint32_t OP_UNSUBSCRIBE = 0x4ce;

/// The unsubscribe request Pimax Play sent when it stopped the gaze stream (`20261013-124248-poweron-hub2.pcap`,
/// device 14, EP 0x05, seq 26, opcode 0x4ce, stream 0x0505). Byte for byte as captured; the device answers with the
/// request's own sequence number, so the stale number is harmless. Only sent on explicit request.
inline constexpr std::array<std::uint8_t, 43> CAPTURED_UNSUBSCRIBE_GAZE = {
    0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00, // wrapper: OUT, 35 body bytes
    0x00, 0x00, 0x00, 0x51, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xce,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, // header: request, seq 26, status 0, op 0x4ce, aux 0, 11 bytes
    0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x05, 0x05, // prefix + tag2(0x0505)
};

/// Options for run_plan().
struct ReplayOptions {
    /// Skip steps whose data is not fully in the capture instead of refusing the whole plan.
    bool allow_incomplete = false;
    /// Timeout of every USB transfer and of every response wait.
    Duration timeout = std::chrono::seconds(2);
};

/// A step that cannot be replayed verbatim.
struct IncompleteStep {
    std::size_t step = 0;
    std::string what;
    /// Bytes missing from the capture.
    std::uint64_t missing = 0;
    /// Real size of the step.
    std::uint64_t total = 0;
    bool operator==(const IncompleteStep&) const = default;
};

/// Replay failure. A failure stops the replay immediately: nothing is retried and nothing further is sent.
struct ReplayError {
    enum class Kind {
        EmptyPlan,  ///< the plan has no steps
        Incomplete, ///< refused before sending anything: `incomplete` lists the steps not fully in the capture
        Io,         ///< USB error at `step` (`what`, `io`)
        NoResponse, ///< no response within the timeout (`step`, `seq`, `opcode`)
        BadStatus,  ///< the device answered with status `got`, Windows saw `expected`
        Framing,    ///< the IN stream could not be framed (`framing`)
    };
    Kind kind = Kind::EmptyPlan;
    std::vector<IncompleteStep> incomplete;
    std::size_t step = 0;
    std::string what;
    Error io;
    std::uint32_t seq = 0;
    std::uint32_t opcode = 0;
    std::uint32_t got = 0;
    std::uint32_t expected = 0;
    ttp::FrameError framing;

    std::string to_string() const;
};

/// One request/response exchange of a replay.
struct Exchange {
    std::size_t step = 0;
    std::uint32_t seq = 0;
    std::uint32_t opcode = 0;
    /// Status word of the response.
    std::uint32_t status = 0;
    /// Payload length of the response.
    std::uint32_t payload_len = 0;
    /// Time from the last OUT frame to the response.
    Duration round_trip{};
};

/// Outcome of a replay.
struct ReplayReport {
    /// Steps sent.
    std::size_t sent = 0;
    /// Steps skipped because they were incomplete (only with allow_incomplete).
    std::vector<std::size_t> skipped;
    /// TTP exchanges.
    std::vector<Exchange> exchanges;
    /// Bytes returned by control IN steps, per step: (step, bytes).
    std::vector<std::pair<std::size_t, std::size_t>> control_in;
    /// Gaze notifications that arrived while the replay was running.
    std::uint64_t notifications = 0;
};

using LogFn = std::function<void(std::string_view)>;

/// Send the plan. Every byte sent comes from `plan`, which comes from a capture.
std::expected<ReplayReport, ReplayError> run_plan(Transport& t, const replay::Plan& plan, const ReplayOptions& opts,
                                                  ttp::StreamDecoder& dec, const LogFn& log = {});

/// Decodes one gaze notification (stream 0x0505); `at` is the host time since the start of the read. Returns an
/// error text when the payload does not decode. Phase 2: eyemath's payload decoder plus the caller's sample sink.
using GazeHandler = std::function<std::expected<void, std::string>(const ttp::Message& m, Duration at)>;

/// Result of read_stream().
struct StreamSummary {
    /// Wall-clock time spent reading.
    Duration elapsed{};
    /// USB bytes read.
    std::uint64_t bytes = 0;
    /// IN transfers that returned data.
    std::uint64_t transfers = 0;
    /// Reads that timed out.
    std::uint64_t timeouts = 0;
    /// Gaze notifications the handler accepted (all of them without a handler).
    std::uint64_t samples = 0;
    /// Gaze notifications whose payload failed to decode.
    std::uint64_t parse_errors = 0;
    /// First decode error.
    std::optional<std::string> first_parse_error;
    /// Framing errors on the IN stream.
    std::uint64_t framing_errors = 0;
    /// Messages that were not gaze notifications.
    std::uint64_t other_messages = 0;
    /// Time of the first sample since the start of the read.
    std::optional<Duration> first_sample_after;
    /// Host-clock time between the first and the last sample.
    Duration sample_span{};

    /// Sample rate by the host clock.
    std::optional<double> host_rate_hz() const;
};

/// Read and decode the stream for `duration`.
Result<StreamSummary> read_stream(Transport& t, ttp::StreamDecoder& dec, Duration duration,
                                  const GazeHandler& on_gaze = {});

/// A fresh decoder for the IN pipe.
ttp::StreamDecoder in_decoder();

} // namespace pibridge::eye::session
