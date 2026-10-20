// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/json.rs (MIT OR Apache-2.0).
//
// Minimal JSON reader and writer helpers (RFC 8259 subset that covers every valid document: objects, arrays, strings
// with escapes, numbers, true/false/null). Used for the calibration overlay (overlay.h) and, later, the SLAM files;
// the writer prints numbers exactly like Rust's {:?} so the files stay byte-identical to pimax-cam's.
#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::cam::json {

/// A parsed JSON value. Object members keep their order; duplicate keys keep the first occurrence on lookup.
struct Value {
    enum class Type { Null, Bool, Num, Str, Arr, Obj };
    Type type = Type::Null;
    bool b = false;
    double num = 0;
    std::string str;
    std::vector<Value> arr;
    std::vector<std::pair<std::string, Value>> obj;

    static Value null() { return {}; }
    static Value boolean(bool v) {
        Value x;
        x.type = Type::Bool;
        x.b = v;
        return x;
    }
    static Value number(double v) {
        Value x;
        x.type = Type::Num;
        x.num = v;
        return x;
    }
    static Value string(std::string v) {
        Value x;
        x.type = Type::Str;
        x.str = std::move(v);
        return x;
    }

    /// Member of an object (nullptr when absent or not an object).
    const Value* get(std::string_view key) const;
    std::optional<double> as_f64() const;
    const std::string* as_str() const;
    const std::vector<Value>* as_array() const;
    /// Array of exactly N numbers.
    template <std::size_t N>
    std::optional<std::array<double, N>> as_f64_array() const {
        const auto* a = as_array();
        if (a == nullptr || a->size() != N) {
            return std::nullopt;
        }
        std::array<double, N> out{};
        for (std::size_t i = 0; i < N; i++) {
            const auto x = (*a)[i].as_f64();
            if (!x) {
                return std::nullopt;
            }
            out[i] = *x;
        }
        return out;
    }
    bool operator==(const Value&) const = default;
};

/// Parse error with byte offset.
struct JsonError {
    std::size_t pos = 0;
    const char* msg = "";
    /// "JSON error at byte {pos}: {msg}".
    std::string text() const;
};

/// Parse a complete JSON document.
std::expected<Value, JsonError> parse(std::string_view text);

/// JSON string literal (with quotes) for s.
std::string quote(std::string_view s);

/// JSON number for a finite x (shortest representation that round-trips, Rust {:?}); "null" for NaN/inf.
std::string num(double x);

/// "[a, b, c]" of numbers.
std::string num_array(std::span<const double> v);

} // namespace pibridge::cam::json
