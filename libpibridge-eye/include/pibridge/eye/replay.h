// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/replay.rs (MIT OR Apache-2.0).
//
// Replay planning: turn a USBPcap capture of the Windows session into the exact list of host-to-EyeChip transfers that
// preceded the first gaze notification, and say whether every byte of that list is present in the capture.
//
// The plan contains
//  * every vendor control transfer (bmRequestType type bits 0x40: 0x41/0x30 session info, 0xc1/0x30, 0xc1/0x46 module
//    clock, 0x41/0x41 TTP session start) in capture order, and
//  * every bulk OUT frame on EP 0x05 (the TTP requests), grouped into messages,
// up to the first stream-0x0505 notification. Standard requests (descriptors, SET_CONFIGURATION) are not part of it:
// the Linux kernel did its own enumeration. The data comes straight from the capture, so nothing is ever invented; a
// step is incomplete when the capture snap length cut its data (captured < real length).
#pragma once

#include "pibridge/eye/pcap.h"
#include "pibridge/eye/transport.h"

#include <cstdint>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace pibridge::eye::replay {

/// Bulk OUT endpoint used for TTP requests.
inline constexpr std::uint8_t EP_BULK_OUT = 0x05;
/// Bulk IN endpoint carrying responses and notifications.
inline constexpr std::uint8_t EP_BULK_IN = 0x83;
/// USB vendor id of the EyeChip.
inline constexpr std::uint16_t EYE_VID = 0x2104;
/// USB product id of the EyeChip (Dream Air, Crystal Super).
inline constexpr std::uint16_t EYE_PID = 0x0561;
/// USB product id of the eye tracker of the original Crystal (P3B, platform VR4PIMAXP3B). It speaks the same TTP on the
/// same endpoints (Windows capture 2026-10-22: one vendor interface, bulk 0x83 IN / 0x04 / 0x05 OUT, the same vendor
/// control requests, gaze stream 0x0505 in 1242-byte frames with the same 35 fields) and needs its own start list
/// (frames::p3b_list).
inline constexpr std::uint16_t EYE_PID_P3B = 0x0220;
/// Whether `pid` (vendor EYE_VID) is one of the eye chips above.
constexpr bool is_eye_pid(std::uint16_t pid) { return pid == EYE_PID || pid == EYE_PID_P3B; }

/// What the captured response to a request looked like.
struct Expect {
    /// Status word of the response header (1 = ok in every capture).
    std::uint32_t status = 0;
    /// Payload length announced by the response.
    std::uint32_t payload_len = 0;
    bool operator==(const Expect&) const = default;
};

/// Vendor control OUT: 8-byte setup packet plus data stage as captured.
struct ControlOut {
    Setup setup{};
    Bytes data;
    bool operator==(const ControlOut&) const = default;
};

/// Vendor control IN: only the setup packet is sent; the reply is read and ignored.
struct ControlIn {
    Setup setup{};
    bool operator==(const ControlIn&) const = default;
};

/// One TTP request: one or more bulk OUT frames (each is its own USB transfer).
struct Request {
    std::uint32_t seq = 0;
    std::uint32_t opcode = 0;
    /// Payload length announced by the header.
    std::uint32_t payload_len = 0;
    /// Frames exactly as captured (wrapper included).
    std::vector<Bytes> frames;
    /// Captured response, when seen.
    std::optional<Expect> expect;
    bool operator==(const Request&) const = default;
};

/// One transfer to send.
using StepKind = std::variant<ControlOut, ControlIn, Request>;

/// A planned transfer plus where it came from.
struct Step {
    /// Record number of the (first) submission in the capture.
    std::uint64_t packet_index = 0;
    /// Capture time in seconds.
    double time = 0;
    /// What to send.
    StepKind kind;
    /// Bytes of real transfer data that the capture does not contain.
    std::uint64_t missing_bytes = 0;
    /// Real transfer data bytes of this step.
    std::uint64_t total_bytes = 0;

    /// All bytes of the step are present.
    bool is_complete() const { return missing_bytes == 0; }
    /// Short description for listings.
    std::string label() const;
    bool operator==(const Step&) const = default;
};

/// The list of transfers of one session start.
struct Plan {
    /// Device address the plan was extracted for (per-capture, re-assigned on every enumeration).
    std::uint16_t device = 0;
    /// Transfers in capture order.
    std::vector<Step> steps;
    /// Capture time of the first gaze notification, when the capture reaches it.
    std::optional<double> stream_start;
    /// OUT frames that continue no message (should be 0).
    std::uint32_t orphan_frames = 0;

    /// Every byte of every step is in the capture, and there is at least one step.
    bool is_complete() const;
    /// Steps whose data was cut by the capture snap length.
    std::vector<const Step*> incomplete_steps() const;
    /// Sum of the missing bytes.
    std::uint64_t missing_bytes() const;
    /// TTP requests of the plan.
    std::vector<const Step*> requests() const;
    /// Requests that look like the authentication exchange (0x76c, 0x777, 0x77b): their answers depend on a device
    /// challenge, so a verbatim replay may be refused even when the capture is complete.
    std::vector<const Step*> auth_steps() const;
    /// Multi-line listing of the plan.
    std::string describe() const;
};

/// Collects the device addresses that carry TTP traffic on EP 0x05 / 0x83 while packets are streamed through it.
class DeviceFinder {
public:
    /// Look at one packet.
    void observe(const pcap::UsbPacket& p);
    /// Device addresses seen, ascending.
    std::vector<std::uint16_t> devices() const;

private:
    std::set<std::uint16_t> found_;
};

/// Device addresses in a capture that carry TTP traffic on EP 0x05 / 0x83.
std::vector<std::uint16_t> find_eye_devices(std::span<const pcap::UsbPacket> packets);

/// Build the plan from the packets of ONE device (any order-preserving filter of the capture).
Plan build_plan(std::uint16_t device, std::span<const pcap::UsbPacket> packets);

} // namespace pibridge::eye::replay
