// SPDX-License-Identifier: MIT
// Internal helpers: fixed-width loads and stores in both byte orders (bounds are checked by the callers).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace pibridge::eye::detail {

inline std::uint16_t le16(std::span<const std::uint8_t> b, std::size_t i) {
    return static_cast<std::uint16_t>(b[i] | (b[i + 1] << 8));
}
inline std::uint32_t le32(std::span<const std::uint8_t> b, std::size_t i) {
    return static_cast<std::uint32_t>(b[i]) | (static_cast<std::uint32_t>(b[i + 1]) << 8) |
           (static_cast<std::uint32_t>(b[i + 2]) << 16) | (static_cast<std::uint32_t>(b[i + 3]) << 24);
}
inline std::uint64_t le64(std::span<const std::uint8_t> b, std::size_t i) {
    return static_cast<std::uint64_t>(le32(b, i)) | (static_cast<std::uint64_t>(le32(b, i + 4)) << 32);
}
inline std::uint32_t be32(std::span<const std::uint8_t> b, std::size_t i) {
    return (static_cast<std::uint32_t>(b[i]) << 24) | (static_cast<std::uint32_t>(b[i + 1]) << 16) |
           (static_cast<std::uint32_t>(b[i + 2]) << 8) | static_cast<std::uint32_t>(b[i + 3]);
}
inline std::uint64_t be64(std::span<const std::uint8_t> b, std::size_t i) {
    return (static_cast<std::uint64_t>(be32(b, i)) << 32) | static_cast<std::uint64_t>(be32(b, i + 4));
}

inline void put_le16(std::vector<std::uint8_t>& o, std::uint16_t v) {
    o.push_back(static_cast<std::uint8_t>(v));
    o.push_back(static_cast<std::uint8_t>(v >> 8));
}
inline void put_le32(std::vector<std::uint8_t>& o, std::uint32_t v) {
    for (int s = 0; s < 32; s += 8) {
        o.push_back(static_cast<std::uint8_t>(v >> s));
    }
}
inline void put_le64(std::vector<std::uint8_t>& o, std::uint64_t v) {
    for (int s = 0; s < 64; s += 8) {
        o.push_back(static_cast<std::uint8_t>(v >> s));
    }
}
inline void put_be32(std::vector<std::uint8_t>& o, std::uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) {
        o.push_back(static_cast<std::uint8_t>(v >> s));
    }
}
inline void put_be64(std::vector<std::uint8_t>& o, std::uint64_t v) {
    for (int s = 56; s >= 0; s -= 8) {
        o.push_back(static_cast<std::uint8_t>(v >> s));
    }
}
inline void store_le32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}
inline void store_le64(std::uint8_t* p, std::uint64_t v) {
    for (int i = 0; i < 8; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}
inline void store_be32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (24 - 8 * i));
    }
}

} // namespace pibridge::eye::detail
