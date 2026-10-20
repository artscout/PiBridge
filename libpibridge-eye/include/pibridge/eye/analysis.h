// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/analysis.rs (MIT OR Apache-2.0), without the gaze statistics.
//
// Offline analysis of USBPcap captures: which devices speak TTP, what was said, how many gaze notifications there
// were, and whether the session start can be replayed. Decoding the gaze payloads (per-eye states, device-clock rate,
// counter gaps, CSV export) needs the gaze decoder of libpibridge-eyemath; an optional GazeCheck hook lets a caller count
// undecodable notifications.
#pragma once

#include "pibridge/eye/pcap.h"
#include "pibridge/eye/replay.h"
#include "pibridge/eye/ttp.h"

#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::eye::analysis {

/// One kind of TTP message seen in a capture.
struct CatalogEntry {
    /// Direction of the bulk transfer.
    ttp::Direction dir = ttp::Direction::Out;
    /// Message class.
    ttp::Kind kind = ttp::Kind::Request;
    /// Opcode or stream id.
    std::uint32_t id = 0;
    /// Messages of this kind.
    std::uint64_t count = 0;
    /// Capture time of the first one.
    double first_time = 0;
    /// Status word of the first one.
    std::uint32_t status = 0;
    /// Smallest and largest announced payload length.
    std::pair<std::uint32_t, std::uint32_t> payload_len{};
    /// Messages whose payload is cut by the capture snap length.
    std::uint64_t cut = 0;
};

/// Gaze stream counts of one device.
struct GazeSummary {
    /// Notifications seen.
    std::uint64_t frames = 0;
    /// Notifications cut by the capture snap length.
    std::uint64_t cut_frames = 0;
    /// Notifications the GazeCheck hook refused (0 without a hook).
    std::uint64_t parse_errors = 0;
    /// Completions whose USB length differs from the frame length in the wrapper (the Windows stack's 16 KiB read
    /// buffers in the `poweron` capture); they repeat stale data and are left out.
    std::uint64_t oversized = 0;
    /// First decode error.
    std::optional<std::string> first_error;
    /// Capture time of the first and last notification.
    std::optional<std::pair<double, double>> span;

    /// Rate by the capture clock.
    std::optional<double> capture_rate_hz() const;
};

/// Everything known about one device of a capture.
struct DeviceReport {
    /// Device address.
    std::uint16_t device = 0;
    /// Packets of this device kept for the analysis.
    std::size_t packets = 0;
    /// TTP message kinds in order of first appearance.
    std::vector<CatalogEntry> catalog;
    /// Gaze stream summary.
    GazeSummary gaze;
    /// Session-start replay plan.
    replay::Plan plan;
};

/// Result of analyze_file().
struct CaptureReport {
    /// Snap length from the pcap header.
    std::uint32_t snaplen = 0;
    /// Packets in the file.
    std::uint64_t packets = 0;
    /// Seconds between the first and the last packet.
    double duration = 0;
    /// One report per TTP device.
    std::vector<DeviceReport> devices;
};

/// Checks one gaze notification payload (`complete` false: cut by the snap length); an error text when it does not
/// decode. Phase 2: eyemath's parse_payload / parse_payload_lenient.
using GazeCheck = std::function<std::expected<void, std::string>(std::span<const std::uint8_t> payload, bool complete)>;

/// Analyse the packets of one device.
DeviceReport analyze_device(std::uint16_t device, std::span<const pcap::UsbPacket> packets,
                            const GazeCheck& check = {});

/// Analyse `path`. With `device` set only that address is examined, otherwise every address that carries TTP traffic.
std::expected<CaptureReport, pcap::PcapError> analyze_file(const std::filesystem::path& path,
                                                           std::optional<std::uint16_t> device,
                                                           const GazeCheck& check = {});

/// Human readable report.
std::string format_report(const CaptureReport& r, bool verbose_plan);

} // namespace pibridge::eye::analysis
