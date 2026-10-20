// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/frame.rs (MIT OR Apache-2.0).
//
// Frame layout and metadata of the Pimax tracking-camera stream (UVC 34a4:0014).
//
// The device advertises YUYV 1280x832 (bytesperline 2560, 2 129 920 B per frame) but every byte is one 8-bit grey
// pixel: the picture is 2560x832 grey, four 640-px tiles side by side (factory camera ids 0..3, left to right). Rows
// 0..7 carry a 96-byte metadata block. Each tile's full 640x480 OV7251 image starts at its own row, y_offset, which
// the metadata reports (Linux captures: 278 for tracking frames, 10 for LED frames; the readout position depends on
// the exposure, like the Rift S). Every other row is zero.
//
// Frames alternate between two exposure sets (byte 32): 0x86 = tracking/SLAM exposure (usable for hands and SLAM)
// and 0x06 = short controller-LED exposure (almost black, LED blobs only). At 50 fps each kind runs at 25 Hz. This
// is the same scheme as the Rift S (rift_s_camera.c: frame_type 0x86/0x06, u64 us timestamp).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace pibridge::cam {

/// Width of the grey picture in pixels (= bytes per row).
inline constexpr std::size_t FRAME_W = 2560;
/// Rows per frame.
inline constexpr std::size_t FRAME_H = 832;
/// Bytes per raw frame (sizeimage of the UVC format).
inline constexpr std::size_t FRAME_BYTES = FRAME_W * FRAME_H;
/// Number of camera tiles.
inline constexpr std::size_t TILE_COUNT = 4;
/// Tile width (one camera).
inline constexpr std::size_t TILE_W = 640;
/// Tile height (full OV7251 sensor height, matches the factory calibration size 640x480).
inline constexpr std::size_t TILE_H = 480;
/// Rows that repeat the metadata block.
inline constexpr std::size_t META_ROWS = 8;
/// Bytes of metadata parsed from row 0.
inline constexpr std::size_t META_LEN = 96;
/// Lowest legal tile start row.
inline constexpr std::size_t Y_MIN = META_ROWS;
/// Highest legal tile start row (the tile must fit in the frame).
inline constexpr std::size_t Y_MAX = FRAME_H - TILE_H;
/// Constant bytes seen at offset 92..96 of the metadata on Linux.
inline constexpr std::array<std::uint8_t, 4> META_MAGIC{0x3c, 0x3d, 0x3e, 0x3f};

/// Which exposure set a frame was taken with.
enum class FrameKind : std::uint32_t {
    /// Metadata missing or not recognised.
    Unknown = 0,
    /// frame_type bit 7 set (0x86): normal tracking exposure. Feed these to hand tracking / SLAM.
    Tracking = 1,
    /// frame_type bit 7 clear (0x06): short exposure for the controller LEDs.
    Led = 2,
};

/// Short lower-case name.
const char* kind_name(FrameKind k);

/// Parsed metadata block (row 0 of the frame).
///
/// | bytes | field | confidence |
/// |---|---|---|
/// | 0..12 | stream_index: one bit per byte (0/1), LSB first; counts frames since the stream started | M (5 frames) |
/// | 32 | frame_type 0x86 tracking / 0x06 LED | H |
/// | 33..41 | ts_us u64 LE, MCU microseconds (the IMU's u32 timestamp is its low 32 bits) | H (low 32), M (high) |
/// | 41..45 | counter u32 LE, +1 per frame, continuous across streams | H |
/// | 45..53 | exposure[4] u16 LE per camera | H |
/// | 53..61 | gain[4] u16 LE per camera | H |
/// | 61..69 | y_offset[4] u16 LE: first image row of each tile | H on Linux |
/// | 92..96 | 3c 3d 3e 3f | constant |
struct Meta {
    std::uint16_t stream_index = 0;
    std::uint8_t frame_type = 0;
    std::uint64_t ts_us = 0;
    std::uint32_t counter = 0;
    std::array<std::uint16_t, 4> exposure{};
    std::array<std::uint16_t, 4> gain{};
    std::array<std::uint16_t, 4> y_offset{};
    bool magic_ok = false;

    /// Parse the metadata from the first row of a frame. nullopt if the row is too short.
    static std::optional<Meta> parse(std::span<const std::uint8_t> row);
    /// Plausibility: either the constant tail is present or the frame type looks like 0x06/0x86.
    bool looks_valid() const { return magic_ok || (frame_type & 0x7f) == 0x06; }
    /// Exposure set of the frame.
    FrameKind kind() const;
    /// Low 32 bits of the device timestamp (what the IMU reports).
    std::uint32_t ts_us_u32() const { return static_cast<std::uint32_t>(ts_us); }
    bool operator==(const Meta&) const = default;
};

/// How the tile start rows of a frame were determined.
enum class RowSource {
    /// All four from the metadata y_offset.
    Metadata,
    /// At least one tile was located by scanning for its first non-zero row.
    Scan,
};

/// Start row of each tile.
struct TileRows {
    std::array<std::size_t, TILE_COUNT> y{};
    RowSource source = RowSource::Metadata;
    bool operator==(const TileRows&) const = default;
};

/// Row y of tile t (640 bytes).
std::span<const std::uint8_t> tile_row(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t,
                                       std::size_t y);

/// First row >= Y_MIN of tile t holding any non-zero byte, clamped to Y_MAX.
std::optional<std::size_t> scan_tile_start(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t);

/// Locate the four tiles: metadata y_offset when in range, else a scan for the first non-zero row (falls back to
/// Y_MIN for an all-zero tile).
TileRows tile_rows(const Meta* meta, std::span<const std::uint8_t> frame, std::size_t stride);

/// Diagnostics: does the picture of tile t really start at y? True when the row above (if it is not a metadata row)
/// is all zero and row y has data. Dark rows can be legitimately zero, so treat a false as a hint.
bool tile_start_consistent(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t, std::size_t y);

/// Copy tile t (640x480 starting at row y) into dst with row pitch dst_stride (>= 640). The caller checked the sizes
/// (dst holds dst_stride * 479 + 640 bytes, the frame holds the tile).
void copy_tile(std::span<const std::uint8_t> frame, std::size_t stride, std::size_t t, std::size_t y,
               std::span<std::uint8_t> dst, std::size_t dst_stride);

/// Check that a raw buffer has the expected geometry; returns the row stride or the Rust error text.
bool check_raw(std::size_t len, std::size_t stride, std::string* err);

} // namespace pibridge::cam
