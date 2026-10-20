// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/tlv.rs (MIT OR Apache-2.0).
// The part of the TTP TLV items the gaze decoder needs: walk `tag u8 | len u32 BE | value[len]` items, decode the
// scalar and compound tags of a gaze notification, and append items (for encode_payload and the tests). The full TLV
// layer (strings, blobs, arrays) lives in libpibridge-eye; this copy keeps libpibridge-eyemath free of any dependency.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace pibridge::eyemath::detail {

namespace tag {
inline constexpr std::uint8_t INT32 = 0x02;
inline constexpr std::uint8_t FIXED_16_16 = 0x03;
inline constexpr std::uint8_t FIXED_22_42 = 0x04;
inline constexpr std::uint8_t COMPOUND = 0x05;
inline constexpr std::uint8_t INT64 = 0x06;
} // namespace tag

inline constexpr std::size_t ITEM_HEADER_LEN = 5;

/// A decoded item value; every tag or length the gaze decoder does not use is `Other` (Rust: the other variants and
/// `Value::Unknown`).
struct Value {
    enum class Kind { Int32, Int64, Fixed16_16, Fixed22_42, Compound, Other };
    Kind kind = Kind::Other;
    std::int64_t i = 0;       // Int32, Int64, Fixed16_16 (raw), Fixed22_42 (raw)
    std::uint16_t count = 0;  // Compound
    std::uint16_t type_id = 0;
};

inline std::uint32_t be32(const std::uint8_t* p) {
    return (std::uint32_t{p[0]} << 24) | (std::uint32_t{p[1]} << 16) | (std::uint32_t{p[2]} << 8) | std::uint32_t{p[3]};
}

inline std::uint64_t be64(const std::uint8_t* p) {
    return (std::uint64_t{be32(p)} << 32) | std::uint64_t{be32(p + 4)};
}

/// One item: tag and value bytes.
struct RawItem {
    std::uint8_t tag = 0;
    std::span<const std::uint8_t> value;

    Value decode() const {
        Value v;
        const auto n = value.size();
        const auto* p = value.data();
        if ((tag == tag::INT32 || tag == tag::FIXED_16_16) && n == 4) {
            v.kind = tag == tag::INT32 ? Value::Kind::Int32 : Value::Kind::Fixed16_16;
            v.i = static_cast<std::int32_t>(be32(p));
        } else if ((tag == tag::FIXED_22_42 || tag == tag::INT64) && n == 8) {
            v.kind = tag == tag::INT64 ? Value::Kind::Int64 : Value::Kind::Fixed22_42;
            v.i = static_cast<std::int64_t>(be64(p));
        } else if (tag == tag::COMPOUND && n == 4) {
            const auto w = be32(p);
            v.kind = Value::Kind::Compound;
            v.count = static_cast<std::uint16_t>(w >> 16);
            v.type_id = static_cast<std::uint16_t>(w & 0xffff);
        }
        return v;
    }
};

/// Iterator over the items of a buffer; after the first truncated item it ends (`failed`).
class Items {
public:
    explicit Items(std::span<const std::uint8_t> buf) : buf_(buf) {}

    enum class Step { Item, End, Error };

    /// Next item into `out`.
    Step next(RawItem& out) {
        if (failed_ || pos_ >= buf_.size()) {
            return Step::End;
        }
        const auto rest = buf_.subspan(pos_);
        if (rest.size() < ITEM_HEADER_LEN) {
            failed_ = true;
            return Step::Error;
        }
        const std::uint64_t len = be32(rest.data() + 1);
        const std::uint64_t needed = ITEM_HEADER_LEN + len;
        if (needed > rest.size()) {
            failed_ = true;
            return Step::Error;
        }
        out.tag = rest[0];
        out.value = rest.subspan(ITEM_HEADER_LEN, static_cast<std::size_t>(len));
        pos_ += static_cast<std::size_t>(needed);
        return Step::Item;
    }

    /// Offset of the next item header.
    std::size_t position() const { return pos_; }

private:
    std::span<const std::uint8_t> buf_;
    std::size_t pos_ = 0;
    bool failed_ = false;
};

inline void push_be32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) {
        out.push_back(static_cast<std::uint8_t>(v >> s));
    }
}

inline void push_be64(std::vector<std::uint8_t>& out, std::uint64_t v) {
    push_be32(out, static_cast<std::uint32_t>(v >> 32));
    push_be32(out, static_cast<std::uint32_t>(v));
}

/// Append one item with a raw value.
inline void push_item(std::vector<std::uint8_t>& out, std::uint8_t t, std::span<const std::uint8_t> value) {
    out.push_back(t);
    push_be32(out, static_cast<std::uint32_t>(value.size()));
    out.insert(out.end(), value.begin(), value.end());
}

inline void push_i32(std::vector<std::uint8_t>& out, std::int32_t v) {
    out.push_back(tag::INT32);
    push_be32(out, 4);
    push_be32(out, static_cast<std::uint32_t>(v));
}

inline void push_fixed16_16(std::vector<std::uint8_t>& out, std::int32_t raw) {
    out.push_back(tag::FIXED_16_16);
    push_be32(out, 4);
    push_be32(out, static_cast<std::uint32_t>(raw));
}

inline void push_fixed22_42(std::vector<std::uint8_t>& out, std::int64_t raw) {
    out.push_back(tag::FIXED_22_42);
    push_be32(out, 8);
    push_be64(out, static_cast<std::uint64_t>(raw));
}

inline void push_compound(std::vector<std::uint8_t>& out, std::uint16_t count, std::uint16_t type_id) {
    out.push_back(tag::COMPOUND);
    push_be32(out, 4);
    push_be32(out, (std::uint32_t{count} << 16) | std::uint32_t{type_id});
}

inline void push_i64(std::vector<std::uint8_t>& out, std::int64_t v) {
    out.push_back(tag::INT64);
    push_be32(out, 8);
    push_be64(out, static_cast<std::uint64_t>(v));
}

} // namespace pibridge::eyemath::detail
