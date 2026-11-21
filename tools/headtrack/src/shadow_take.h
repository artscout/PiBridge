// SPDX-License-Identifier: MIT
// Head shadow take: exactly what the shadow engine (Basalt) was fed in a live session, so that the same session can be
// replayed through Basalt offline with another configuration and compared with the driving engine's poses of the
// live log (pibridge-headtrack-replay, docs/pibridge-headtrack.md "Head shadow mode").
//
// The frame sets go here (PIBRIDGE_HEAD_SHADOW_TAKE=1: head-shadow-*-take.bin next to the log); the IMU samples go to
// head-shadow-*-imu.csv through libpibridge-cam's imulog (I lines: ts, accel, gyro as pushed, body axes).
//
// File, little endian:
//   header, 32 bytes: [u8; 8] "PXSTAKE1", u16 version = 1, u16 tiles, u16 width, u16 height, u32 flags (bit 0: records
//     may be zlib), [u8; 12] reserved
//   record, 32-byte header + payload: u32 magic "FSET", u32 flags (bit 0: payload is zlib), u32 stored length, u32 raw
//     length (= tiles * width * height), i64 ts_ns (the frame time given to the engine: mid-exposure on the IMU
//     clock, plus the camera offset), i64 arrival_ns (CLOCK_MONOTONIC when it was pushed; the replay keeps the same
//     lead of the IMU over the frames)
//   payload: the tiles one after the other, width * height Y8 each.
//
// The writer never blocks the camera thread: a copy goes into a bounded queue, a thread of its own compresses and
// writes; frame sets are dropped (and counted) when the queue is full. A truncated last record is the end of the file.
#pragma once

#include <cstdint>
#include <cstdio>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace pibridge::headtrack {

inline constexpr char SHADOW_TAKE_MAGIC[9] = "PXSTAKE1";
inline constexpr std::uint32_t SHADOW_TAKE_REC_MAGIC = 0x54455346; // "FSET"
inline constexpr std::uint32_t SHADOW_TAKE_ZLIB = 1;

class ShadowTakeWriter {
public:
    /// Create path for frame sets of `tiles` images of w x h; zlib compresses each record (level 1). nullptr with
    /// `error` set when the file cannot be created.
    static std::unique_ptr<ShadowTakeWriter> start(const std::string& path, std::uint32_t tiles, std::uint32_t w,
                                                   std::uint32_t h, bool zlib, std::string* error,
                                                   std::size_t depth = 64);
    /// Stops the thread and flushes what is queued.
    ~ShadowTakeWriter();
    ShadowTakeWriter(const ShadowTakeWriter&) = delete;
    ShadowTakeWriter& operator=(const ShadowTakeWriter&) = delete;

    /// One frame set (tiles[i] = w * h bytes); never blocks.
    void push(std::int64_t ts_ns, std::int64_t arrival_ns, const std::vector<const std::uint8_t*>& tiles);
    /// Frame sets queued / dropped (queue full) / written to the file.
    std::uint64_t queued() const;
    std::uint64_t dropped() const;
    std::uint64_t written() const;
    /// Bytes written including headers.
    std::uint64_t bytes() const;
    const std::string& path() const { return path_; }

private:
    struct Item {
        std::int64_t ts = 0;
        std::int64_t arrival = 0;
        std::vector<std::uint8_t> data;
    };
    ShadowTakeWriter() = default;
    void run();

    std::string path_;
    std::FILE* f_ = nullptr;
    std::uint32_t tiles_ = 0, w_ = 0, h_ = 0;
    bool zlib_ = false;
    std::size_t depth_ = 64;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Item> queue_;
    std::vector<std::vector<std::uint8_t>> pool_;
    bool closed_ = false;
    std::thread thread_;
    std::uint64_t queued_ = 0, dropped_ = 0, written_ = 0, bytes_ = 0;
    std::vector<std::uint8_t> zbuf_;
};

/// One frame set read back.
struct ShadowTakeFrame {
    std::int64_t ts_ns = 0;
    std::int64_t arrival_ns = 0;
    /// tiles * w * h bytes, tile after tile.
    std::vector<std::uint8_t> data;
};

class ShadowTakeReader {
public:
    /// Open and check the header; nullptr with `error` set when it is not a take.
    static std::unique_ptr<ShadowTakeReader> open(const std::string& path, std::string* error);
    ~ShadowTakeReader();
    ShadowTakeReader(const ShadowTakeReader&) = delete;
    ShadowTakeReader& operator=(const ShadowTakeReader&) = delete;

    /// The next frame set into f; false at the end (a truncated tail too, see truncated) or on a bad record (error
    /// set).
    bool next(ShadowTakeFrame& f, std::string* error);
    /// Back to the first record.
    bool rewind();

    std::uint32_t tiles = 0, w = 0, h = 0;
    std::uint32_t flags = 0;
    bool truncated = false;

private:
    ShadowTakeReader() = default;
    std::FILE* f_ = nullptr;
    std::vector<std::uint8_t> stored_;
};

} // namespace pibridge::headtrack
