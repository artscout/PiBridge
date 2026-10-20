// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/toml_lite.rs (MIT OR Apache-2.0).
//
// A tiny reader for the TOML subset the device descriptions use. No dependency.
// Supported: `# comments`, `[table]`, `[[array-of-tables]]`, `key = value` on one line, where value is a basic string
// `"..."` (escapes `\"`, `\\`, `\n`, `\t`), an integer (decimal or `0x` hex, optional `-`, `_` separators), a float,
// `true`/`false`, or a one-line array of those. Anything else is an error with its line number, so a description that
// uses more TOML than this fails loudly in the tests instead of being half read.
#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace pibridge::hmd::toml {

struct Value;
using Array = std::vector<Value>;

struct Value {
    std::variant<std::string, std::int64_t, double, bool, Array> v;
    bool operator==(const Value&) const = default;
};

/// One `[name]` or `[[name]]` section (the root section has the name "").
struct Section {
    std::string name;
    /// True for `[[name]]`.
    bool array = false;
    std::vector<std::pair<std::string, Value>> entries;
    /// 1-based line of the header (0 for the root).
    std::size_t line = 0;

    const Value* get(std::string_view key) const;
    std::optional<std::string> str(std::string_view key) const;
    std::optional<std::int64_t> integer(std::string_view key) const;
    /// A float, or an integer as float.
    std::optional<double> number(std::string_view key) const;
    std::optional<bool> boolean(std::string_view key) const;
    /// The integers of an array (other items skipped); empty when missing.
    std::vector<std::int64_t> ints(std::string_view key) const;
    /// The strings of an array; empty when missing.
    std::vector<std::string> strs(std::string_view key) const;
};

struct ParseError {
    std::size_t line = 0;
    std::string msg;
    std::string message() const { return "line " + std::to_string(line) + ": " + msg; }
};

/// Parse a document into its sections, in file order (root first).
std::expected<std::vector<Section>, ParseError> parse(std::string_view src);

} // namespace pibridge::hmd::toml
