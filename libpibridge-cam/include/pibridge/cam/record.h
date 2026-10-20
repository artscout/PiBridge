// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/record.rs (MIT OR Apache-2.0).
//
// Raw recording format (.pxcam) for offline tests and replay into Monado.
//
//   file header, 32 bytes, little endian:
//     0  [u8; 8]  magic "PXCAMRAW"
//     8  u16      version = 1
//    10  u16      header length = 32
//    12  u16      width  = 2560 (grey bytes per image row)
//    14  u16      height = 832
//    16  u32      stride of the stored rows (2560)
//    20  u32      flags (bit 0: records may be zlib-compressed)
//    24  [u8; 8]  reserved
//   record, 32-byte header + payload:
//     0  u32  magic "FRAM" (0x4d415246)
//     4  u32  flags (bit 0: payload is zlib)
//     8  u32  stored payload length
//    12  u32  raw length (stride * height)
//    16  u64  host CLOCK_MONOTONIC ns (V4L2 buffer timestamp)
//    24  u32  V4L2 sequence
//    28  u32  reserved
//
// The payload is the complete frame as delivered by the driver (metadata rows, zero rows and all four tiles), so a
// replay goes through exactly the same parsing as live capture. A truncated last record (recording killed) is
// reported as the end of the file. Raw recordings are byte-identical to the Rust writer's; zlib payloads come from
// zlib instead of miniz_oxide, so their compressed bytes differ (they decompress to the same frames).
#pragma once

#include "pibridge/cam/source.h"

#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace pibridge::cam {

/// File magic.
inline constexpr char PXCAM_MAGIC[9] = "PXCAMRAW";
/// Record magic ("FRAM").
inline constexpr std::uint32_t PXCAM_REC_MAGIC = 0x4d41'5246;
/// Record flag: payload is zlib.
inline constexpr std::uint32_t PXCAM_REC_ZLIB = 1;

/// Writer.
class Recorder {
public:
    /// Create path. compress_level: nullopt = raw, 1..9 = zlib per frame.
    static Result<std::unique_ptr<Recorder>> create(const std::string& path, std::size_t stride,
                                                    std::optional<std::uint8_t> compress_level);
    ~Recorder();
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    /// Append one frame (must have the stride given at creation).
    Result<void> write(const RawFrame& f);
    /// Flush and close; returns (frames, bytes).
    Result<std::pair<std::uint64_t, std::uint64_t>> finish();

    /// Frames written.
    std::uint64_t frames = 0;
    /// Bytes written including headers.
    std::uint64_t bytes = 0;

private:
    Recorder() = default;
    std::FILE* f_ = nullptr;
    std::optional<std::uint8_t> compress_;
    std::size_t stride_ = 0;
    std::vector<std::uint8_t> zbuf_;
};

/// Recorder on its own thread: the capture loop only copies the frame into a bounded queue, so disk stalls never
/// block capture. Frames are dropped (and counted) when the queue is full.
class AsyncRecorder {
public:
    /// Start a writer thread for path with a queue of depth frames.
    static Result<std::unique_ptr<AsyncRecorder>> start(const std::string& path, std::size_t stride,
                                                        std::optional<std::uint8_t> compress_level, std::size_t depth);
    ~AsyncRecorder();
    AsyncRecorder(const AsyncRecorder&) = delete;
    AsyncRecorder& operator=(const AsyncRecorder&) = delete;

    /// Queue a copy of the frame (never blocks).
    void push(const RawFrame& f);
    /// Flush, stop the thread; returns (frames written, bytes).
    Result<std::pair<std::uint64_t, std::uint64_t>> finish();

    /// Frames dropped because the queue was full.
    std::uint64_t dropped = 0;
    /// Frames queued.
    std::uint64_t queued = 0;

private:
    struct Item {
        std::vector<std::uint8_t> data;
        std::uint64_t ts = 0;
        std::uint32_t seq = 0;
    };
    AsyncRecorder() = default;
    void run();

    std::unique_ptr<Recorder> rec_;
    std::size_t stride_ = 0;
    std::size_t depth_ = 1;
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Item> queue_;
    std::vector<std::vector<std::uint8_t>> pool_;
    bool closed_ = false;
    bool running_ = false;
    std::thread thread_;
    std::optional<IoError> result_err_;
    std::pair<std::uint64_t, std::uint64_t> result_{0, 0};
};

/// One frame read back.
struct Record {
    std::vector<std::uint8_t> data;
    std::size_t stride = 0;
    std::uint64_t host_ts_ns = 0;
    std::uint32_t sequence = 0;
};

/// Sequential reader.
class Reader {
public:
    /// Open and validate the header.
    static Result<std::unique_ptr<Reader>> open(const std::string& path);
    ~Reader();
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;

    /// Rewind to the first record.
    Result<void> rewind();
    /// Next record into rec; false at the end (also for a truncated tail, see truncated).
    Result<bool> next_record(Record& rec);

    /// Stored row stride.
    std::size_t stride = 0;
    /// File-level flags.
    std::uint32_t flags = 0;
    /// True if the last read hit a truncated record.
    bool truncated = false;

private:
    Reader() = default;
    std::FILE* f_ = nullptr;
    std::vector<std::uint8_t> payload_;
};

/// A recording played back as a Source. Host timestamps are rebased onto the current monotonic clock.
class Replay final : public Source {
public:
    static Result<std::unique_ptr<Replay>> open(const std::string& path, ReplayOptions opts);

    Result<bool> advance(int timeout_ms) override;
    std::optional<RawFrame> current() const override;
    std::string describe() const override;
    bool finished() const override { return done_; }
    bool is_replay() const override { return true; }

    /// Number of times the file was restarted.
    std::uint32_t loops = 0;

private:
    Replay() = default;
    std::unique_ptr<Reader> reader_;
    ReplayOptions opts_;
    std::string path_;
    Record cur_;
    Record next_;
    bool have_cur_ = false;
    std::optional<std::uint64_t> base_rec_ns_;
    std::uint64_t base_mono_ns_ = 0;
    std::uint64_t last_out_ns_ = 0;
    std::uint64_t loop_offset_ns_ = 0;
    bool done_ = false;
};

} // namespace pibridge::cam
