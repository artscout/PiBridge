// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/tlv.rs (MIT OR Apache-2.0).
//
// TLV items of a TTP payload.
//
// Every item is `tag u8 | len u32 BE | value[len]`. The tags below are the only ones seen in the Windows captures
// (cold start, calibration, settings; 4 000+ messages). Unknown tags are carried as Unknown, never rejected.
//
// `tag 5` is a compound header: its 32-bit value is `count << 16 | type_id` and announces that the next `count` items
// belong together (a vec3 is `0x31f42` followed by three fixed-point items; a gaze field is `0x20bb9` followed by
// `field number` and the value). The count/type split is inferred from the data: it holds for all 364 full-size gaze
// packets (`0x230bb8` = 35 fields, `0x20bb9` = 2 elements, `0x31f42` = 3, `0x21f43` = 2, `0x21f40` = 2).
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace pibridge::eye::tlv {

/// Tag bytes seen on the wire.
namespace tag {
/// Unsigned 32-bit integer (seen only inside the configuration upload 0x852, whose capture is cut).
inline constexpr std::uint8_t UINT32 = 0x01;
/// Signed 32-bit integer (counters, validity flags, field numbers, stream ids).
inline constexpr std::uint8_t INT32 = 0x02;
/// Signed fixed point, 16 integer bits and 16 fraction bits (`raw / 65536`), 4 bytes.
inline constexpr std::uint8_t FIXED_16_16 = 0x03;
/// Signed fixed point, 22 integer bits and 42 fraction bits (`raw / 2^42`), 8 bytes.
inline constexpr std::uint8_t FIXED_22_42 = 0x04;
/// Compound header, 4 bytes: `count << 16 | type_id`.
inline constexpr std::uint8_t COMPOUND = 0x05;
/// Signed 64-bit integer (microsecond timestamps).
inline constexpr std::uint8_t INT64 = 0x06;
/// String: `u32 BE length` followed by that many bytes.
inline constexpr std::uint8_t STRING = 0x14;
/// Blob: `u32 BE length` followed by that many bytes.
inline constexpr std::uint8_t BLOB = 0x15;
/// Array: `u32 BE count` followed by `count` 4-byte elements.
inline constexpr std::uint8_t ARRAY = 0x17;
/// Array of S22.42 values: `u32 BE count` followed by `count` 8-byte elements (seen only inside 0x852).
inline constexpr std::uint8_t ARRAY_FIXED_22_42 = 0x19;
} // namespace tag

/// Size of an item header (`tag` + `len`).
inline constexpr std::size_t ITEM_HEADER_LEN = 5;

/// A borrowed byte range that compares by content (so the value types below can default operator==).
struct View {
    std::span<const std::uint8_t> bytes;
    friend bool operator==(const View& a, const View& b) { return std::ranges::equal(a.bytes, b.bytes); }
};

/// Decoded values. Fixed-point values keep their raw integer so nothing is lost.
struct Uint32 { std::uint32_t v; bool operator==(const Uint32&) const = default; };
struct Int32 { std::int32_t v; bool operator==(const Int32&) const = default; };
/// Raw value; real value is `raw / 65536`.
struct Fixed16_16 { std::int32_t raw; bool operator==(const Fixed16_16&) const = default; };
/// Raw value; real value is `raw / 2^42`.
struct Fixed22_42 { std::int64_t raw; bool operator==(const Fixed22_42&) const = default; };
/// `count` items that follow belong to this compound of type `type_id`.
struct Compound { std::uint16_t count; std::uint16_t type_id; bool operator==(const Compound&) const = default; };
struct Int64 { std::int64_t v; bool operator==(const Int64&) const = default; };
/// `tag 0x14` without the length prefix.
struct Str { View s; bool operator==(const Str&) const = default; };
/// `tag 0x15` without the length prefix.
struct Blob { View b; bool operator==(const Blob&) const = default; };
/// `tag 0x17`: `count` 4-byte elements.
struct Array { std::uint32_t count; View elements; bool operator==(const Array&) const = default; };
/// `tag 0x19`: `count` S22.42 elements of 8 bytes each.
struct ArrayFixed22_42 { std::uint32_t count; View elements; bool operator==(const ArrayFixed22_42&) const = default; };
/// Unknown tag, or a known tag with an unexpected length.
struct Unknown { std::uint8_t tag; View value; bool operator==(const Unknown&) const = default; };

using Value = std::variant<Uint32, Int32, Fixed16_16, Fixed22_42, Compound, Int64, Str, Blob, Array, ArrayFixed22_42,
                           Unknown>;

/// One undecoded item.
struct RawItem {
    /// Tag byte.
    std::uint8_t tag = 0;
    /// Value bytes (exactly the length announced in the header).
    std::span<const std::uint8_t> value;
    /// Offset of the item header inside the buffer being iterated.
    std::size_t offset = 0;

    /// Decode the value according to the tag table.
    Value decode() const;
};

/// TLV walking error: the item header or its value runs past the end of the buffer.
struct TlvError {
    /// Offset of the offending item header.
    std::size_t offset = 0;
    /// Bytes the item needs (header + value).
    std::size_t needed = 0;
    /// Bytes left in the buffer from `offset`.
    std::size_t available = 0;

    bool operator==(const TlvError&) const = default;
    std::string to_string() const;
};

/// Iterator over the items of a buffer. After the first error the iterator ends.
class Items {
public:
    explicit Items(std::span<const std::uint8_t> buf) : buf_(buf) {}

    /// The next item, an error (once), or nullopt at the end.
    std::optional<std::expected<RawItem, TlvError>> next();

    /// Offset of the next item header.
    std::size_t position() const { return pos_; }

    /// All remaining results (the error, if any, last).
    std::vector<std::expected<RawItem, TlvError>> collect();

private:
    std::span<const std::uint8_t> buf_;
    std::size_t pos_ = 0;
    bool failed_ = false;
};

/// `raw / 65536`.
double fixed16_16_to_f64(std::int32_t raw);
/// `raw / 2^42`.
double fixed22_42_to_f64(std::int64_t raw);

/// Append one item. Values longer than u32::MAX bytes are a programming error (std::abort, as the Rust panic).
void push_item(std::vector<std::uint8_t>& out, std::uint8_t tag, std::span<const std::uint8_t> value);
/// Append a `tag 2` item.
void push_i32(std::vector<std::uint8_t>& out, std::int32_t v);
/// Append a `tag 3` item with the given raw S15.16 value.
void push_fixed16_16(std::vector<std::uint8_t>& out, std::int32_t raw);
/// Append a `tag 4` item with the given raw S22.42 value.
void push_fixed22_42(std::vector<std::uint8_t>& out, std::int64_t raw);
/// Append a `tag 5` compound header.
void push_compound(std::vector<std::uint8_t>& out, std::uint16_t count, std::uint16_t type_id);
/// Append a `tag 6` item.
void push_i64(std::vector<std::uint8_t>& out, std::int64_t v);

} // namespace pibridge::eye::tlv
