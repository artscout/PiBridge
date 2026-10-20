// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/hid.rs (MIT OR Apache-2.0).
//
// The HID command allowlist and the send gate.
//
// Every device description (`devices/<id>.toml`) declares the exact byte patterns the stack may send (`[[hid]]`). A
// pattern is a list of literal bytes plus named parameter fields with an allowed range or set; the rest of the report
// must be zero padding. Gate::send checks every report against the list BEFORE it writes anything: a report that
// matches no pattern, has a parameter out of range, or matches an explicitly denied prefix is refused and nothing
// reaches the device. A description without `[[hid]]` entries can send nothing at all.
//
// Pattern syntax (`bytes`): space separated tokens, each a hex byte (`58`) or a field `{name:u8}` / `{name:u16le}`.
// Constraints (`params`): `"name=LO..HI"` (inclusive, decimal or 0x hex) or `"name=A|B|C"`.
#pragma once

#include "pibridge/hmd/toml_lite.h"

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace pibridge::hmd {

/// One token of a pattern: a literal byte or a little-endian parameter field.
struct Tok {
    bool field = false;
    std::uint8_t lit = 0;
    std::string name;
    /// Field width in bytes (1 or 2).
    std::size_t width = 0;
    bool operator==(const Tok&) const = default;
};

/// Allowed values of one parameter: an inclusive range, or a set.
struct Constraint {
    bool is_range = true;
    std::uint32_t lo = 0, hi = 0;
    std::vector<std::uint32_t> set;
    bool allows(std::uint32_t v) const;
    bool operator==(const Constraint&) const = default;
};

/// How often a command may go out (documentation for callers; the gate enforces min_interval).
enum class Policy {
    /// Part of a periodic or session sequence (keepalive, wake, video prepare, reads).
    Session,
    /// Exactly once per explicit user request (IPD apply, StartPair). Never retried automatically.
    OncePerRequest,
    /// Once per USB enumeration (eye-module power-on).
    OncePerEnumeration,
};

struct SpecError {
    std::string msg;
};

/// Why a report was refused.
struct Refused {
    enum class Kind {
        /// Matches a denied prefix (`why`).
        Denied,
        /// Matches no approved pattern.
        NotListed,
        /// The pattern matched but field `param` = `value` is outside its constraint.
        OutOfRange,
        /// Bytes after the pattern are not zero.
        Padding,
        /// Wrong report length (`got`).
        Length,
        /// build() was asked for an id the description does not have.
        UnknownCommand,
        /// build() lacks a parameter value.
        MissingParam,
        /// Sent again before min_interval passed.
        TooSoon,
    };
    Kind kind = Kind::NotListed;
    /// Command id (OutOfRange, Padding, UnknownCommand, MissingParam, TooSoon).
    std::string cmd;
    std::string param;
    std::uint32_t value = 0;
    std::size_t got = 0;
    std::string why;

    Refused() = default;
    explicit Refused(Kind k) : kind(k) {}
    Refused(Kind k, std::string command) : kind(k), cmd(std::move(command)) {}
    std::string message() const;
    bool operator==(const Refused&) const = default;
};

/// One approved command.
struct CommandSpec {
    std::string id;
    /// USB interface the report goes to.
    std::uint8_t interface = 0;
    std::vector<Tok> toks;
    std::vector<std::pair<std::string, Constraint>> params;
    /// Report length without the hidraw report-id prefix.
    std::size_t report_len = 64;
    Policy policy = Policy::Session;
    /// Minimum time between two sends of this command through one gate.
    std::optional<std::chrono::milliseconds> min_interval;
    /// Where it was seen (capture). Required.
    std::string source;
    /// Approval note, required for anything that changes device state.
    std::string approved;

    /// From a `[[hid]]` section.
    static std::expected<CommandSpec, SpecError> from_section(const toml::Section& s);
    std::size_t pattern_len() const;
    /// Does `r` (exact report, no report id) match? The error carries the most specific reason.
    std::expected<void, Refused> matches(std::span<const std::uint8_t> r) const;
    /// Encode with the given values.
    std::expected<std::vector<std::uint8_t>, Refused>
    encode(std::span<const std::pair<std::string_view, std::uint32_t>> vals) const;
    bool operator==(const CommandSpec&) const = default;
};

/// An explicitly refused prefix (with the reason shown to the user). nullopt bytes are `??` (any).
struct Denied {
    std::vector<std::optional<std::uint8_t>> prefix;
    std::string why;
    static std::expected<Denied, SpecError> parse(std::string_view prefix, std::string_view why);
    bool hits(std::span<const std::uint8_t> r) const;
    bool operator==(const Denied&) const = default;
};

std::expected<std::vector<Tok>, SpecError> parse_pattern(std::string_view s);
std::expected<std::pair<std::string, Constraint>, SpecError> parse_constraint(std::string_view s);

using ParamValues = std::vector<std::pair<std::string_view, std::uint32_t>>;

/// A description's approved commands plus its denied prefixes.
struct Allowlist {
    std::vector<CommandSpec> commands;
    std::vector<Denied> denied;

    /// Check one report for `interface`. Accepts the bare report or the hidraw write buffer (leading report id 0;
    /// every real report starts with a non-zero magic, so the prefix is unambiguous).
    std::expected<const CommandSpec*, Refused> check(std::uint8_t interface, std::span<const std::uint8_t> report) const;
    /// Build the report for command `id` with parameter values; refused exactly like a send would be.
    std::expected<std::vector<std::uint8_t>, Refused> build(std::string_view id, const ParamValues& vals = {}) const;
    const CommandSpec* get(std::string_view id) const;
    bool operator==(const Allowlist&) const = default;
};

/// Send error: refused before writing, or the write failed (errno).
struct SendError {
    std::variant<Refused, int> e;
    bool refused() const { return std::holds_alternative<Refused>(e); }
    const Refused* refusal() const { return std::get_if<Refused>(&e); }
    std::string message() const;
};

/// The send layer: the only way to write to a headset interface. Owns the writer (a write to an open hidraw node in
/// real use, a byte vector in tests): writer(buf) returns the bytes written or -errno. Opening the node is the
/// caller's job.
class Gate {
public:
    using Writer = std::function<long(std::span<const std::uint8_t>)>;
    using Clock = std::chrono::steady_clock;

    Gate(const Allowlist& allow, std::uint8_t interface, Writer out);

    /// Check, then write `00 + report` in one write. Returns the matched command id.
    std::expected<std::string, SendError> send(std::span<const std::uint8_t> report);
    /// Forget the min_interval history (a new USB enumeration).
    void reset_history() { last_.clear(); }

private:
    const Allowlist* allow_;
    std::uint8_t interface_;
    Writer out_;
    std::map<std::string, Clock::time_point, std::less<>> last_;
};

/// A Gate writer that appends to a byte vector (tests).
Gate::Writer vector_writer(std::vector<std::uint8_t>& v);

} // namespace pibridge::hmd
