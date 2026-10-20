// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/frame.rs (MIT OR Apache-2.0).
#include "pibridge/cam/frame.h"

#include <algorithm>
#include <cstring>
#include <format>
#include <stdexcept>

namespace pibridge::cam {

const char* kind_name(FrameKind k) {
    switch (k) {
    case FrameKind::Tracking: return "tracking";
    case FrameKind::Led: return "led";
    default: return "unknown";
    }
}

namespace {

std::uint16_t le16(std::span<const std::uint8_t> b, std::size_t o) {
    return static_cast<std::uint16_t>(b[o] | (b[o + 1] << 8));
}

} // namespace

std::optional<Meta> Meta::parse(std::span<const std::uint8_t> row) {
    if (row.size() < META_LEN) {
        return std::nullopt;
    }
    Meta m;
    for (std::size_t i = 0; i < 12; i++) {
        if (row[i] != 0) {
            m.stream_index = static_cast<std::uint16_t>(m.stream_index | (1u << i));
        }
    }
    m.frame_type = row[32];
    std::uint64_t ts = 0;
    for (int i = 7; i >= 0; i--) {
        ts = (ts << 8) | row[33 + static_cast<std::size_t>(i)];
    }
    m.ts_us = ts;
    m.counter = static_cast<std::uint32_t>(row[41]) | (static_cast<std::uint32_t>(row[42]) << 8) |
                (static_cast<std::uint32_t>(row[43]) << 16) | (static_cast<std::uint32_t>(row[44]) << 24);
    for (std::size_t i = 0; i < 4; i++) {
        m.exposure[i] = le16(row, 45 + 2 * i);
        m.gain[i] = le16(row, 53 + 2 * i);
        m.y_offset[i] = le16(row, 61 + 2 * i);
    }
    m.magic_ok = std::equal(META_MAGIC.begin(), META_MAGIC.end(), row.begin() + 92);
    return m;
}

FrameKind Meta::kind() const {
    if (!looks_valid()) {
        return FrameKind::Unknown;
    }
    return (frame_type & 0x80) != 0 ? FrameKind::Tracking : FrameKind::Led;
}

std::span<const std::uint8_t> tile_row(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t,
                                       std::size_t y) {
    const std::size_t o = y * stride + t * TILE_W;
    if (o + TILE_W > frame.size()) {
        // a Rust slice index out of range panics; the C ABI turns that into an error
        throw std::out_of_range("tile row outside the frame");
    }
    return frame.subspan(o, TILE_W);
}

std::optional<std::size_t> scan_tile_start(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t) {
    for (std::size_t y = Y_MIN; y < FRAME_H; y++) {
        const auto r = tile_row(frame, stride, t, y);
        if (std::any_of(r.begin(), r.end(), [](std::uint8_t b) { return b != 0; })) {
            return std::min(y, Y_MAX);
        }
    }
    return std::nullopt;
}

TileRows tile_rows(const Meta* meta, std::span<const std::uint8_t> frame, std::size_t stride) {
    TileRows rows;
    rows.y.fill(Y_MIN);
    for (std::size_t t = 0; t < TILE_COUNT; t++) {
        std::optional<std::size_t> from_meta;
        if (meta != nullptr && meta->looks_valid()) {
            const std::size_t v = meta->y_offset[t];
            if (v >= Y_MIN && v <= Y_MAX) {
                from_meta = v;
            }
        }
        if (from_meta) {
            rows.y[t] = *from_meta;
        } else {
            rows.source = RowSource::Scan;
            rows.y[t] = scan_tile_start(frame, stride, t).value_or(Y_MIN);
        }
    }
    return rows;
}

bool tile_start_consistent(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t, std::size_t y) {
    if (y < Y_MIN || y > Y_MAX) {
        return false;
    }
    bool above_zero = true;
    if (y != Y_MIN) {
        const auto a = tile_row(frame, stride, t, y - 1);
        above_zero = std::all_of(a.begin(), a.end(), [](std::uint8_t b) { return b == 0; });
    }
    if (!above_zero) {
        return false;
    }
    const auto h = tile_row(frame, stride, t, y);
    return std::any_of(h.begin(), h.end(), [](std::uint8_t b) { return b != 0; });
}

void copy_tile(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t, std::size_t y,
               std::span<std::uint8_t> dst, std::size_t dst_stride) {
    if (dst_stride < TILE_W || dst.size() < dst_stride * (TILE_H - 1) + TILE_W) {
        throw std::invalid_argument("copy_tile: destination too small");
    }
    for (std::size_t r = 0; r < TILE_H; r++) {
        const auto src = tile_row(frame, stride, t, y + r);
        std::memcpy(dst.data() + r * dst_stride, src.data(), TILE_W);
    }
}

bool check_raw(std::size_t len, std::size_t stride, std::string* err) {
    if (stride < FRAME_W) {
        *err = std::format("row stride {} < {}", stride, FRAME_W);
        return false;
    }
    const std::size_t need = stride * (FRAME_H - 1) + FRAME_W;
    if (len < need) {
        *err = std::format("frame has {} bytes, need {} (stride {})", len, need, stride);
        return false;
    }
    return true;
}

} // namespace pibridge::cam
