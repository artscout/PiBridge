// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/frames.rs (MIT OR Apache-2.0).
//
// Eye chip start lists (`eye-start-*.txt`, the format of the public base list `base-frames.txt`, derived from the
// Windows capture `20261014-043545-cold2-hub2.pcap`) and the send gate.
//
// Safety model: the list is the only source of bytes. Guard refuses any control transfer or bulk frame that is not
// byte-identical to one in the list, refuses the calibration opcodes outright, and (pinned) refuses a 0x456 message
// other than the one captured blob (length and FNV-1a hash pinned below).
#pragma once

#include "pibridge/eye/error.h"
#include "pibridge/eye/replay.h"
#include "pibridge/eye/transport.h"
#include "pibridge/eye/ttp.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace pibridge::eye::frames {

/// The public BASE list (user decision 2026-10-17): the Dream Air cold start without the two auth steps (0x76c,
/// 0x777) and with the developer's calibration blob (0x456) as the built-in test calibration. Compiled in from
/// `libpibridge-eye/data/base-frames.txt`.
std::string_view base_list();

/// Which list `list()` chose.
enum class ListKind {
    Base, ///< the public base list
    Full, ///< a private full list (never compiled into this tree; load it from a file)
};

/// The built-in list the eye start uses. The omarchyvr build could link a private full list in; this tree never
/// does, so it is always the base list. Users' own lists are files (`pibridge-eye frames`).
std::pair<std::string_view, ListKind> list();

/// Calibration opcodes that are never sent through this Guard. Only calibration::Guard (calibration.h) lets them out,
/// and only while a calibration session is explicitly open.
inline constexpr std::array<std::uint32_t, 5> DENIED_OPS = {0x3f2, 0x407, 0x42e, 0x3fc, 0x44c};
/// Auth opcodes (`--skip-auth`).
inline constexpr std::array<std::uint32_t, 2> AUTH_OPS = {0x76c, 0x777};
/// Calibration upload opcode (`--skip-cal`).
inline constexpr std::uint32_t CAL_OP = 0x456;
/// Payload length of the captured calibration message (header payload size field).
inline constexpr std::uint32_t CAL_PAYLOAD_LEN = 0x8640;
/// FNV-1a 64 of the whole captured 0x456 payload.
inline constexpr std::uint64_t CAL_PAYLOAD_FNV = 7053722925501851203ULL;

bool is_denied(std::uint32_t op);
bool is_auth(std::uint32_t op);

/// Standard enumeration (descriptors, SET_CONFIGURATION): never sent by this library.
struct Enumeration {
    bool operator==(const Enumeration&) const = default;
};
/// Control transfer.
struct Control {
    /// 8-byte setup packet.
    Setup setup{};
    /// Data stage of an OUT request.
    Bytes data;
    bool operator==(const Control&) const = default;
};
/// One TTP request, split into one or more bulk OUT frames.
struct Ttp {
    std::uint32_t seq = 0;
    std::uint32_t op = 0;
    /// Bulk OUT transfers (each with its 8-byte wrapper).
    std::vector<Bytes> frames;
    bool operator==(const Ttp&) const = default;
};

/// What a step does.
using Action = std::variant<Enumeration, Control, Ttp>;

/// One step of the list.
struct FStep {
    /// Step number in the file.
    std::uint32_t n = 0;
    /// Class flag: R, C, AUTH or CAL.
    std::string cls;
    /// Description line.
    std::string desc;
    /// Capture time of the request in seconds, when given.
    std::optional<double> t;
    /// The action.
    Action action;
    bool operator==(const FStep&) const = default;
};

/// Parse a frame list.
std::expected<std::vector<FStep>, std::string> parse(std::string_view text);

/// Reassemble the frames of one request and check they hold exactly one message (no bytes left over).
std::expected<ttp::Message, std::string> ttp_info(const std::vector<Bytes>& frames);

/// FNV-1a 64.
std::uint64_t fnv1a(ByteView b);

/// Which optional parts are left out.
struct Skips {
    /// Omit 0x76c / 0x777.
    bool auth = false;
    /// Omit 0x456.
    bool cal = false;
    /// Omit the vendor control write bReq 48.
    bool vendor_write = false;
};

/// Select the steps to send (enumeration is never sent).
std::vector<const FStep*> select(std::span<const FStep> steps, Skips s);

/// The send gate: only list members, never denied opcodes.
class Guard {
public:
    /// Build from a parsed list; fails if the list contains a denied opcode or a 0x456 other than the captured blob.
    static std::expected<Guard, std::string> make(std::span<const FStep> steps);

    /// As make(); `pin_cal` false accepts the 0x456 the list carries (a list made from the user's own capture by
    /// from_plan(), which holds that headset's own profile upload) instead of only the built-in blob.
    static std::expected<Guard, std::string> make_with(std::span<const FStep> steps, bool pin_cal);

    /// Allow one more frame that is not in the list (the captured gaze unsubscribe). Denied opcodes stay refused.
    void allow_frame(ByteView frame);

    /// Control transfer through the gate.
    Result<Bytes> control(Transport& t, const Setup& setup, ByteView data, Duration timeout) const;

    /// Bulk OUT (EP 0x05) through the gate.
    Result<void> bulk_out(Transport& t, ByteView frame, Duration timeout) const;

private:
    Guard() = default;
    std::set<Bytes> frames_;
    std::set<std::pair<Setup, Bytes>> controls_;
};

/// Hex string (lower case, no separators).
std::string hex(ByteView b);

/// A frame list (the format of the base list) from the start plan of a Windows capture: every vendor control transfer
/// and TTP request up to the first gaze notification, byte for byte as captured, in capture order. The two auth steps
/// are left out unless `keep_auth` (their answer depends on a challenge of that session). Fails when the capture cut
/// any data (a list is never padded or guessed).
std::expected<std::string, std::string> from_plan(const replay::Plan& plan, std::string_view source, bool keep_auth);

/// Bytes before the profile body in a 0x456 upload or 0x44c download payload: `00 00 | 15 | u32 BE len+4 | u32 BE len`.
inline constexpr std::size_t PROFILE_PREFIX_LEN = 11;
/// The Tobii `calibration.setpm` header: `u32 LE 4 | u32 LE 1 | u32 LE len+4 | 4 bytes`, then the profile body.
inline constexpr std::size_t SETPM_HEADER_LEN = 16;
/// Largest profile body accepted (the captured ones are 30-38 KB).
inline constexpr std::size_t PROFILE_MAX_LEN = 256 * 1024;
/// Body bytes per bulk OUT frame of a long request (Windows: 4095-byte URBs = wrapper + 4087).
inline constexpr std::size_t OUT_MAX_BODY = 4087;

/// Check a profile body's own layout: it starts with `u32 LE len - 168` (both captured profiles, 30 875 and 37 401 B).
std::expected<void, std::string> check_profile(ByteView body);

/// The profile body of a Tobii `calibration.setpm` (what Pimax's calibration saved under Windows), checked.
std::expected<Bytes, std::string> profile_from_setpm(ByteView file);

/// The profile body inside a 0x456 upload or 0x44c download payload, when the payload has that layout.
std::optional<ByteView> profile_body(ByteView payload);

/// The 0x456 payload that uploads `body` (the layout of the captured uploads and downloads).
Bytes profile_payload(ByteView body);

/// Put the upload of `body` in place of each 0x456 step of the list (the Crystal Super's start uploads the profile
/// twice): same header (seq, status, aux) as that captured upload, the new payload, split into frames as Windows
/// splits it. The list must have a 0x456 step.
std::expected<void, std::string> with_profile(std::vector<FStep>& steps, ByteView body);

} // namespace pibridge::eye::frames
