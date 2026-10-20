// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/shm.rs (MIT OR Apache-2.0).
// The shared-memory ring between Monado's recorder tap (the producer, comp_recorder.c in our Monado fork) and
// pibridge-record (the consumer). Byte for byte the layout of src/xrt/compositor/main/comp_recorder_shm.h (version 1):
// the static_asserts below repeat the C header's and add every other offset.
//
// The consumer creates the file under a temporary name, fills the request and renames it into place, so the
// producer never maps a half-written header. Live fields are only touched through atomics. Slot states: 0 free,
// 1 producer writing, 2 ready, 3 consumer reading.
#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace pibridge::record {

inline constexpr std::uint64_t kMagic = 0x3143455258414d50ull; // "PMAXREC1"
inline constexpr std::uint32_t kVersion = 1;
inline constexpr std::uint32_t kMaxSlots = 8;
inline constexpr std::uint32_t kHeaderSize = 4096;

inline constexpr std::uint32_t kSlotFree = 0;
inline constexpr std::uint32_t kSlotWriting = 1;
inline constexpr std::uint32_t kSlotReady = 2;
inline constexpr std::uint32_t kSlotReading = 3;

struct ShmSlot {
    std::uint32_t state; // live
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t _pad;
    std::uint64_t seq;
    std::uint64_t timestamp_ns; // predicted display time, CLOCK_MONOTONIC
};

struct ShmHeader {
    std::uint64_t magic;
    std::uint32_t version;
    std::uint32_t header_size;
    // request, written by the consumer before the rename
    std::uint32_t eyes;
    std::uint32_t fps;
    std::uint32_t max_height;
    std::uint32_t scale_permille;
    std::uint32_t slot_count;
    std::uint32_t _pad0;
    std::uint64_t slot_size;
    // consumer, live
    std::uint64_t consumer_heartbeat_ns;
    std::uint32_t stop;
    // producer, live
    std::uint32_t producer_pid;
    std::uint64_t producer_heartbeat_ns;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t src_width;
    std::uint32_t src_height;
    std::uint64_t produced;
    std::uint64_t dropped_busy; // GPU readback still busy
    std::uint64_t dropped_full; // every slot full (consumer behind)
    std::uint64_t skipped;      // no squashed eye image this frame
    char producer_note[64];
    ShmSlot slot[kMaxSlots]; // "slots" in the C header (renamed: Qt defines a slots macro)
};

static_assert(offsetof(ShmHeader, eyes) == 16 && offsetof(ShmHeader, fps) == 20 && offsetof(ShmHeader, max_height) == 24);
static_assert(offsetof(ShmHeader, scale_permille) == 28 && offsetof(ShmHeader, slot_count) == 32);
static_assert(offsetof(ShmHeader, slot_size) == 40);
static_assert(offsetof(ShmHeader, consumer_heartbeat_ns) == 48, "layout");
static_assert(offsetof(ShmHeader, stop) == 56 && offsetof(ShmHeader, producer_pid) == 60);
static_assert(offsetof(ShmHeader, producer_heartbeat_ns) == 64, "layout");
static_assert(offsetof(ShmHeader, width) == 72 && offsetof(ShmHeader, src_width) == 80);
static_assert(offsetof(ShmHeader, produced) == 88 && offsetof(ShmHeader, dropped_busy) == 96);
static_assert(offsetof(ShmHeader, dropped_full) == 104 && offsetof(ShmHeader, skipped) == 112);
static_assert(offsetof(ShmHeader, producer_note) == 120, "layout");
static_assert(offsetof(ShmHeader, slot) == 184, "layout");
static_assert(sizeof(ShmSlot) == 32, "layout");
static_assert(sizeof(ShmHeader) <= kHeaderSize, "layout");

// What the consumer asks for.
struct Request {
    std::uint32_t eyes = 1;
    std::uint32_t fps = 30;
    std::uint32_t max_height = 1080;
    std::uint32_t scale_permille = 0;
    std::uint32_t slot_count = 4;
    std::uint64_t slot_size = 0;
    // Slots big enough for any eye aspect up to 1.25 at max_height (the producer shrinks the frame if needed).
    static Request make(std::uint32_t eyes, std::uint32_t fps, std::uint32_t maxHeight, std::uint32_t scalePermille);
    std::uint64_t fileSize() const { return kHeaderSize + std::uint64_t(slot_count) * slot_size; }
    bool operator==(const Request&) const = default;
};

// CLOCK_MONOTONIC in ns (the producer's clock).
std::uint64_t monotonicNs();
// $PIMAX_RECORD_SHM (the name Monado reads), else <runtime dir>/pimax-record.shm (Monado computes the same).
std::filesystem::path defaultShmPath(const std::filesystem::path& runtimeDir);

struct Counters {
    std::uint64_t produced = 0;
    std::uint64_t dropped_busy = 0; // GPU readback still busy (the compositor never waits for it)
    std::uint64_t dropped_full = 0; // every slot full: this recorder (or the encoder) was behind
    std::uint64_t skipped = 0;      // no squashed eye image that frame
    bool operator==(const Counters&) const = default;
};

class Ring;

// A frame the consumer holds (slot state READING) until it is destroyed.
class Frame {
public:
    Frame(const Ring* r, std::uint32_t i, std::uint32_t w, std::uint32_t h, std::uint64_t sq, std::uint64_t ts)
        : ring_(r), index_(i), width(w), height(h), seq(sq), timestamp_ns(ts) {}
    Frame(Frame&& o) noexcept;
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;
    Frame& operator=(Frame&&) = delete;
    ~Frame();
    std::span<const std::uint8_t> pixels() const;

private:
    const Ring* ring_;
    std::uint32_t index_;

public:
    std::uint32_t width, height;
    std::uint64_t seq, timestamp_ns;
};

// A mapping of the ring (either side). The consumer owns the file and removes it when destroyed.
class Ring {
public:
    // Consumer: create the file (mode 0600) under a temporary name, write the request, rename it to `path`.
    // Fails if `path` exists and a live consumer still uses it ("another recorder is running").
    static std::expected<Ring, std::string> create(const std::filesystem::path& path, const Request& req);
    // Producer (and tests): map an existing ring.
    static std::expected<Ring, std::string> open(const std::filesystem::path& path);
    Ring(Ring&& o) noexcept;
    Ring(const Ring&) = delete;
    Ring& operator=(const Ring&) = delete;
    Ring& operator=(Ring&&) = delete;
    ~Ring();
    // A crashed consumer: unmap without removing the file (tests).
    void forget();

    const std::filesystem::path& path() const { return path_; }
    const ShmHeader& hdr() const { return *reinterpret_cast<const ShmHeader*>(base_); }

    void heartbeat() const;
    void setConsumerHeartbeat(std::uint64_t ns) const; // tests
    void requestStop() const;                          // ask the producer to detach
    std::uint32_t producerPid() const;
    // Seconds since the producer's last heartbeat (nullopt before it attached).
    std::optional<double> producerAgeS() const;
    Counters counters() const;
    std::pair<std::uint32_t, std::uint32_t> size() const;
    std::string producerNote() const;
    // Consumer: the oldest ready frame, if any (its slot is held until the frame is destroyed).
    std::optional<Frame> take() const;

    // Producer side (the fake producer of the tests): attach with an output size, heartbeat, publish, detach.
    // `srcWidth`/`srcHeight`: the source eye size (0: the output size).
    void attachProducer(std::uint32_t width, std::uint32_t height, std::string_view note, std::uint32_t pid,
                        std::uint32_t srcWidth = 0, std::uint32_t srcHeight = 0) const;
    // Producer counters: a frame dropped because its readback was still busy / a frame with no image.
    void countDroppedBusy() const;
    void countSkipped() const;
    void producerHeartbeat() const;
    // Publish a frame into a free slot; false (and dropped_full counted) if every slot is busy.
    bool publish(std::uint32_t width, std::uint32_t height, std::uint64_t seq, std::uint64_t timestampNs,
                 const std::function<void(std::span<std::uint8_t>)>& fill) const;
    void detachProducer() const;
    bool stopRequested() const;
    std::uint32_t load32(std::size_t offset) const;
    std::uint64_t load64(std::size_t offset) const;

private:
    friend class Frame;
    Ring(std::uint8_t* base, std::size_t len, std::filesystem::path path, bool owner)
        : base_(base), len_(len), path_(std::move(path)), owner_(owner) {}
    ShmHeader& mut() const { return *reinterpret_cast<ShmHeader*>(base_); }
    std::uint8_t* slotPtr(std::uint32_t i) const;
    void store32(std::size_t offset, std::uint32_t v) const;
    void store64(std::size_t offset, std::uint64_t v) const;
    bool cas32(std::size_t offset, std::uint32_t expect, std::uint32_t desired) const;
    void add64(std::size_t offset, std::uint64_t v) const;

    std::uint8_t* base_ = nullptr;
    std::size_t len_ = 0;
    std::filesystem::path path_;
    bool owner_ = false;
};

// The output size Monado picks for an eye (pick_eye_extent in comp_recorder.c; kept in step by the tests).
std::optional<std::pair<std::uint32_t, std::uint32_t>> pickEyeExtent(std::pair<std::uint32_t, std::uint32_t> src,
                                                                     std::uint32_t maxHeight, std::uint32_t scalePermille,
                                                                     std::uint32_t eyes, std::uint64_t slotSize);

} // namespace pibridge::record
