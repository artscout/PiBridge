// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/tlv.rs (MIT OR Apache-2.0).
#include "pibridge/eye/tlv.h"

#include "bytes.h"

#include <cstdlib>
#include <format>
#include <limits>

namespace pibridge::eye::tlv {

using detail::be32;
using detail::be64;

std::string TlvError::to_string() const {
    return std::format("TLV item at offset {} needs {} bytes but only {} are left", offset, needed, available);
}

std::optional<std::expected<RawItem, TlvError>> Items::next() {
    if (failed_ || pos_ >= buf_.size()) {
        return std::nullopt;
    }
    const auto rest = buf_.subspan(pos_);
    if (rest.size() < ITEM_HEADER_LEN) {
        failed_ = true;
        return std::unexpected(TlvError{pos_, ITEM_HEADER_LEN, rest.size()});
    }
    const std::size_t len = be32(rest, 1);
    // saturating: a length near u32::MAX must not wrap on 32-bit size_t
    const std::size_t needed =
        len > std::numeric_limits<std::size_t>::max() - ITEM_HEADER_LEN ? std::numeric_limits<std::size_t>::max()
                                                                         : ITEM_HEADER_LEN + len;
    if (needed > rest.size()) {
        failed_ = true;
        return std::unexpected(TlvError{pos_, needed, rest.size()});
    }
    RawItem item{rest[0], rest.subspan(ITEM_HEADER_LEN, len), pos_};
    pos_ += needed;
    return item;
}

std::vector<std::expected<RawItem, TlvError>> Items::collect() {
    std::vector<std::expected<RawItem, TlvError>> out;
    while (auto r = next()) {
        out.push_back(*r);
    }
    return out;
}

Value RawItem::decode() const {
    const auto v = value;
    const std::size_t n = v.size();
    const Unknown unknown{tag, View{v}};
    switch (tag) {
    case tag::UINT32:
        return n == 4 ? Value{Uint32{be32(v, 0)}} : Value{unknown};
    case tag::INT32:
        return n == 4 ? Value{Int32{static_cast<std::int32_t>(be32(v, 0))}} : Value{unknown};
    case tag::FIXED_16_16:
        return n == 4 ? Value{Fixed16_16{static_cast<std::int32_t>(be32(v, 0))}} : Value{unknown};
    case tag::FIXED_22_42:
        return n == 8 ? Value{Fixed22_42{static_cast<std::int64_t>(be64(v, 0))}} : Value{unknown};
    case tag::COMPOUND:
        if (n == 4) {
            const std::uint32_t w = be32(v, 0);
            return Compound{static_cast<std::uint16_t>(w >> 16), static_cast<std::uint16_t>(w & 0xffff)};
        }
        return unknown;
    case tag::INT64:
        return n == 8 ? Value{Int64{static_cast<std::int64_t>(be64(v, 0))}} : Value{unknown};
    case tag::STRING:
    case tag::BLOB:
        if (n >= 4 && be32(v, 0) == n - 4) {
            const View body{v.subspan(4)};
            return tag == tag::STRING ? Value{Str{body}} : Value{Blob{body}};
        }
        return unknown;
    case tag::ARRAY:
    case tag::ARRAY_FIXED_22_42:
        if (n >= 4) {
            const std::uint64_t c = be32(v, 0);
            const std::uint64_t width = tag == tag::ARRAY ? 4 : 8;
            if (c * width == n - 4) {
                const View el{v.subspan(4)};
                const auto count = static_cast<std::uint32_t>(c);
                return tag == tag::ARRAY ? Value{Array{count, el}} : Value{ArrayFixed22_42{count, el}};
            }
        }
        return unknown;
    default:
        return unknown;
    }
}

double fixed16_16_to_f64(std::int32_t raw) {
    return static_cast<double>(raw) / 65536.0;
}

double fixed22_42_to_f64(std::int64_t raw) {
    return static_cast<double>(raw) / 4'398'046'511'104.0;
}

void push_item(std::vector<std::uint8_t>& out, std::uint8_t t, std::span<const std::uint8_t> value) {
    if (value.size() > std::numeric_limits<std::uint32_t>::max()) {
        std::abort(); // "TLV value longer than u32::MAX": a programming error, as in the Rust original
    }
    out.push_back(t);
    detail::put_be32(out, static_cast<std::uint32_t>(value.size()));
    out.insert(out.end(), value.begin(), value.end());
}

namespace {
void push_be32_item(std::vector<std::uint8_t>& out, std::uint8_t t, std::uint32_t w) {
    std::vector<std::uint8_t> v;
    detail::put_be32(v, w);
    push_item(out, t, v);
}
void push_be64_item(std::vector<std::uint8_t>& out, std::uint8_t t, std::uint64_t w) {
    std::vector<std::uint8_t> v;
    detail::put_be64(v, w);
    push_item(out, t, v);
}
} // namespace

void push_i32(std::vector<std::uint8_t>& out, std::int32_t v) {
    push_be32_item(out, tag::INT32, static_cast<std::uint32_t>(v));
}

void push_fixed16_16(std::vector<std::uint8_t>& out, std::int32_t raw) {
    push_be32_item(out, tag::FIXED_16_16, static_cast<std::uint32_t>(raw));
}

void push_fixed22_42(std::vector<std::uint8_t>& out, std::int64_t raw) {
    push_be64_item(out, tag::FIXED_22_42, static_cast<std::uint64_t>(raw));
}

void push_compound(std::vector<std::uint8_t>& out, std::uint16_t count, std::uint16_t type_id) {
    push_be32_item(out, tag::COMPOUND, (static_cast<std::uint32_t>(count) << 16) | type_id);
}

void push_i64(std::vector<std::uint8_t>& out, std::int64_t v) {
    push_be64_item(out, tag::INT64, static_cast<std::uint64_t>(v));
}

} // namespace pibridge::eye::tlv
