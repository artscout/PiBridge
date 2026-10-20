// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/shm.rs (MIT OR Apache-2.0).
#include "pibridge/record/shm.h"

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <format>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;

Request Request::make(std::uint32_t eyes, std::uint32_t fps, std::uint32_t maxHeight, std::uint32_t scalePermille) {
    const std::uint64_t w = ((std::uint64_t(maxHeight) * 5 + 3) / 4) * eyes;
    Request r;
    r.eyes = eyes;
    r.fps = fps;
    r.max_height = maxHeight;
    r.scale_permille = scalePermille;
    r.slot_count = 4;
    r.slot_size = ((w * maxHeight * 4 + 4095) / 4096) * 4096;
    return r;
}

std::uint64_t monotonicNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::uint64_t(ts.tv_sec) * 1'000'000'000ull + std::uint64_t(ts.tv_nsec);
}

fs::path defaultShmPath(const fs::path& runtimeDir) {
    if (const char* p = std::getenv("PIMAX_RECORD_SHM"); p != nullptr && *p != '\0') {
        return fs::path(p);
    }
    return runtimeDir / "pimax-record.shm";
}

// ---------------------------------------------------------------- atomics on the mapping

std::uint32_t Ring::load32(std::size_t off) const {
    return std::atomic_ref<std::uint32_t>(*reinterpret_cast<std::uint32_t*>(base_ + off)).load(std::memory_order_acquire);
}
std::uint64_t Ring::load64(std::size_t off) const {
    return std::atomic_ref<std::uint64_t>(*reinterpret_cast<std::uint64_t*>(base_ + off)).load(std::memory_order_acquire);
}
void Ring::store32(std::size_t off, std::uint32_t v) const {
    std::atomic_ref<std::uint32_t>(*reinterpret_cast<std::uint32_t*>(base_ + off)).store(v, std::memory_order_release);
}
void Ring::store64(std::size_t off, std::uint64_t v) const {
    std::atomic_ref<std::uint64_t>(*reinterpret_cast<std::uint64_t*>(base_ + off)).store(v, std::memory_order_release);
}
bool Ring::cas32(std::size_t off, std::uint32_t expect, std::uint32_t desired) const {
    return std::atomic_ref<std::uint32_t>(*reinterpret_cast<std::uint32_t*>(base_ + off))
        .compare_exchange_strong(expect, desired, std::memory_order_acq_rel, std::memory_order_acquire);
}
void Ring::add64(std::size_t off, std::uint64_t v) const {
    std::atomic_ref<std::uint64_t>(*reinterpret_cast<std::uint64_t*>(base_ + off)).fetch_add(v, std::memory_order_relaxed);
}

namespace {
constexpr std::size_t slotStateOff(std::uint32_t i) { return offsetof(ShmHeader, slot) + i * sizeof(ShmSlot); }

std::expected<std::uint8_t*, std::string> mapFd(int fd, std::size_t len) {
    void* p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        return std::unexpected(std::string(std::strerror(errno)));
    }
    return static_cast<std::uint8_t*>(p);
}
} // namespace

// ---------------------------------------------------------------- Frame

Frame::Frame(Frame&& o) noexcept
    : ring_(std::exchange(o.ring_, nullptr)), index_(o.index_), width(o.width), height(o.height), seq(o.seq),
      timestamp_ns(o.timestamp_ns) {}

Frame::~Frame() {
    if (ring_ != nullptr) {
        ring_->store32(slotStateOff(index_), kSlotFree);
    }
}

std::span<const std::uint8_t> Frame::pixels() const {
    return {ring_->slotPtr(index_), std::size_t(width) * height * 4};
}

// ---------------------------------------------------------------- Ring

Ring::Ring(Ring&& o) noexcept
    : base_(std::exchange(o.base_, nullptr)), len_(o.len_), path_(std::move(o.path_)), owner_(std::exchange(o.owner_, false)) {}

Ring::~Ring() {
    if (base_ == nullptr) {
        return;
    }
    if (owner_) {
        std::error_code ec;
        fs::remove(path_, ec);
    }
    munmap(base_, len_);
}

void Ring::forget() {
    if (base_ != nullptr) {
        munmap(base_, len_);
        base_ = nullptr;
    }
}

std::expected<Ring, std::string> Ring::create(const fs::path& path, const Request& req) {
    std::error_code ec;
    if (fs::exists(path, ec)) {
        if (auto old = Ring::open(path)) {
            const std::uint64_t t = old->load64(offsetof(ShmHeader, consumer_heartbeat_ns));
            if (t != 0 && monotonicNs() - std::min(t, monotonicNs()) < 3'000'000'000ull) {
                return std::unexpected(std::string("another recorder is running"));
            }
        }
        fs::remove(path, ec);
    }
    fs::path tmp = path;
    tmp.replace_extension(std::format("tmp{}", getpid()));
    const int fd = ::open(tmp.c_str(), O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) {
        return std::unexpected(std::string(std::strerror(errno)));
    }
    const std::uint64_t len = req.fileSize();
    if (ftruncate(fd, static_cast<off_t>(len)) != 0) {
        const std::string e = std::strerror(errno);
        ::close(fd);
        fs::remove(tmp, ec);
        return std::unexpected(e);
    }
    auto base = mapFd(fd, len);
    ::close(fd);
    if (!base) {
        fs::remove(tmp, ec);
        return std::unexpected(base.error());
    }
    Ring ring(*base, len, path, true);
    {
        // nobody else can see the temporary file yet
        ShmHeader& h = ring.mut();
        h.magic = kMagic;
        h.version = kVersion;
        h.header_size = kHeaderSize;
        h.eyes = req.eyes;
        h.fps = req.fps;
        h.max_height = req.max_height;
        h.scale_permille = req.scale_permille;
        h.slot_count = req.slot_count;
        h.slot_size = req.slot_size;
        h.consumer_heartbeat_ns = monotonicNs();
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        ring.owner_ = false;
        fs::remove(tmp, ec);
        return std::unexpected(ec.message());
    }
    return ring;
}

std::expected<Ring, std::string> Ring::open(const fs::path& path) {
    const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        return std::unexpected(std::string(std::strerror(errno)));
    }
    struct stat st{};
    if (fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(kHeaderSize)) {
        ::close(fd);
        return std::unexpected(std::string("too small"));
    }
    const auto len = static_cast<std::size_t>(st.st_size);
    auto base = mapFd(fd, len);
    ::close(fd);
    if (!base) {
        return std::unexpected(base.error());
    }
    Ring ring(*base, len, path, false);
    const ShmHeader& h = ring.hdr();
    if (h.magic != kMagic || h.version != kVersion || h.slot_count > kMaxSlots ||
        kHeaderSize + std::uint64_t(h.slot_count) * h.slot_size > len) {
        return std::unexpected(std::string("not a pibridge-record ring"));
    }
    return ring;
}

std::uint8_t* Ring::slotPtr(std::uint32_t i) const { return base_ + kHeaderSize + std::size_t(i) * hdr().slot_size; }

void Ring::heartbeat() const { store64(offsetof(ShmHeader, consumer_heartbeat_ns), monotonicNs()); }
void Ring::setConsumerHeartbeat(std::uint64_t ns) const { store64(offsetof(ShmHeader, consumer_heartbeat_ns), ns); }
void Ring::requestStop() const { store32(offsetof(ShmHeader, stop), 1); }
std::uint32_t Ring::producerPid() const { return load32(offsetof(ShmHeader, producer_pid)); }

std::optional<double> Ring::producerAgeS() const {
    const std::uint64_t t = load64(offsetof(ShmHeader, producer_heartbeat_ns));
    if (t == 0) {
        return std::nullopt;
    }
    const std::uint64_t now = monotonicNs();
    return double(now > t ? now - t : 0) / 1e9;
}

Counters Ring::counters() const {
    return {load64(offsetof(ShmHeader, produced)), load64(offsetof(ShmHeader, dropped_busy)),
            load64(offsetof(ShmHeader, dropped_full)), load64(offsetof(ShmHeader, skipped))};
}

std::pair<std::uint32_t, std::uint32_t> Ring::size() const {
    return {load32(offsetof(ShmHeader, width)), load32(offsetof(ShmHeader, height))};
}

std::string Ring::producerNote() const {
    const char* n = hdr().producer_note;
    return std::string(n, strnlen(n, sizeof hdr().producer_note));
}

std::optional<Frame> Ring::take() const {
    const ShmHeader& h = hdr();
    std::optional<std::pair<std::uint32_t, std::uint64_t>> best;
    for (std::uint32_t i = 0; i < std::min(h.slot_count, kMaxSlots); i++) {
        if (load32(slotStateOff(i)) == kSlotReady) {
            const std::uint64_t seq = h.slot[i].seq;
            if (!best || seq < best->second) {
                best = std::pair{i, seq};
            }
        }
    }
    if (!best) {
        return std::nullopt;
    }
    const std::uint32_t i = best->first;
    if (!cas32(slotStateOff(i), kSlotReady, kSlotReading)) {
        return std::nullopt;
    }
    const ShmSlot& s = h.slot[i];
    const std::uint32_t w = s.width, ht = s.height;
    if (std::uint64_t(w) * ht * 4 > h.slot_size || w == 0 || ht == 0) {
        store32(slotStateOff(i), kSlotFree);
        return std::nullopt;
    }
    return std::optional<Frame>(std::in_place, this, i, w, ht, s.seq, s.timestamp_ns);
}

void Ring::attachProducer(std::uint32_t width, std::uint32_t height, std::string_view note, std::uint32_t pid,
                          std::uint32_t srcWidth, std::uint32_t srcHeight) const {
    ShmHeader& h = mut();
    h.width = width;
    h.height = height;
    h.src_width = srcWidth != 0 ? srcWidth : width;
    h.src_height = srcHeight != 0 ? srcHeight : height;
    std::memset(h.producer_note, 0, sizeof h.producer_note);
    std::memcpy(h.producer_note, note.data(), std::min<std::size_t>(note.size(), 63));
    store64(offsetof(ShmHeader, producer_heartbeat_ns), monotonicNs());
    store32(offsetof(ShmHeader, producer_pid), pid);
}

void Ring::countDroppedBusy() const { add64(offsetof(ShmHeader, dropped_busy), 1); }
void Ring::countSkipped() const { add64(offsetof(ShmHeader, skipped), 1); }

void Ring::producerHeartbeat() const { store64(offsetof(ShmHeader, producer_heartbeat_ns), monotonicNs()); }

bool Ring::publish(std::uint32_t width, std::uint32_t height, std::uint64_t seq, std::uint64_t timestampNs,
                   const std::function<void(std::span<std::uint8_t>)>& fill) const {
    const std::uint32_t count = std::min(hdr().slot_count, kMaxSlots);
    const std::uint64_t slotSize = hdr().slot_size;
    for (std::uint32_t i = 0; i < count; i++) {
        if (cas32(slotStateOff(i), kSlotFree, kSlotWriting)) {
            const std::size_t n = std::size_t(width) * height * 4;
            if (n > slotSize) {
                store32(slotStateOff(i), kSlotFree);
                return false;
            }
            fill(std::span<std::uint8_t>(slotPtr(i), n));
            ShmSlot& s = mut().slot[i];
            s.width = width;
            s.height = height;
            s.seq = seq;
            s.timestamp_ns = timestampNs;
            store32(slotStateOff(i), kSlotReady);
            add64(offsetof(ShmHeader, produced), 1);
            return true;
        }
    }
    add64(offsetof(ShmHeader, dropped_full), 1);
    return false;
}

void Ring::detachProducer() const { store32(offsetof(ShmHeader, producer_pid), 0); }
bool Ring::stopRequested() const { return load32(offsetof(ShmHeader, stop)) != 0; }

std::optional<std::pair<std::uint32_t, std::uint32_t>> pickEyeExtent(std::pair<std::uint32_t, std::uint32_t> src,
                                                                     std::uint32_t maxHeight, std::uint32_t scalePermille,
                                                                     std::uint32_t eyes, std::uint64_t slotSize) {
    double h = src.second;
    if (scalePermille > 0 && scalePermille <= 1000) {
        h = h * scalePermille / 1000.0;
    }
    if (maxHeight > 0 && h > maxHeight) {
        h = maxHeight;
    }
    const double aspect = double(src.first) / double(src.second);
    for (;;) {
        const std::uint32_t eh = static_cast<std::uint32_t>(h) & ~1u;
        const std::uint32_t ew = static_cast<std::uint32_t>(h * aspect + 0.5) & ~1u;
        if (eh < 64 || ew < 64) {
            return std::nullopt;
        }
        if (std::uint64_t(ew) * eyes * eh * 4 <= slotSize && ew * eyes <= 4096) {
            return std::pair{ew, eh};
        }
        h -= 16.0;
    }
}

} // namespace pibridge::record
