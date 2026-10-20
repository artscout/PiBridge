// SPDX-License-Identifier: MIT
// The PiBridge window in the VR home (docs/home.md "The PiBridge window in the headset"): one shared-memory file between
// the app and pibridge-home, $XDG_RUNTIME_DIR/pibridge/vrpanel.shm (tmpfs; pages are used only as far as written).
//
//   the app   -> picture: its main window rendered offscreen (QWidget::render), BGRA 8-bit sRGB, top row first,
//                under a sequence lock (frameSeq odd while writing); appBeatNs every check
//   pibridge-home -> homeBeatNs while it shows the panel (the app renders only while that is under 2 s old), and the
//                controller pointer: `pointer` (x, y in window pixels, buttons) and `wheel` (a running total, 120 per
//                notch; the app applies the difference)
//
// Either side may be missing or restart at any time: the file is made by whoever comes first, a stale beat means "not
// there", and nothing ever waits on the other side (a leaf: the home runs without the app and the other way round).
// Header-only, no Qt (pibridge-home and the app both include it).
#pragma once

#include "pibridge_status.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace pibridge::vrpanel {

inline constexpr std::uint32_t kMagic = 0x4c4e5046; // "FPNL"
inline constexpr std::uint32_t kVersion = 1;
inline constexpr std::uint32_t kMaxW = 2048;
inline constexpr std::uint32_t kMaxH = 2048;
inline constexpr std::size_t kHeaderBytes = 4096;
inline constexpr std::size_t kFileBytes = kHeaderBytes + static_cast<std::size_t>(kMaxW) * kMaxH * 4;
inline constexpr std::uint64_t kBeatTimeoutNs = 2'000'000'000ULL;

struct Header {
    std::uint32_t magic;
    std::uint32_t version;
    // the app's side
    std::atomic<std::uint64_t> frameSeq; // even: a whole picture; odd: being written
    std::atomic<std::uint32_t> width;
    std::atomic<std::uint32_t> height;
    std::atomic<std::uint64_t> appBeatNs;
    std::atomic<std::uint32_t> appPid;
    // pibridge-home's side
    std::atomic<std::uint64_t> homeBeatNs;
    std::atomic<std::uint32_t> homePid;
    std::atomic<std::uint64_t> pointer; // packPointer
    std::atomic<std::int64_t> wheel;    // running total, 120 per notch (+ = away from you: scroll up)
};
static_assert(sizeof(Header) <= kHeaderBytes);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

/// The pointer on the window: x, y in pixels (nullopt: off the panel), buttons bit 0 = left.
struct Pointer {
    bool on = false;
    std::uint32_t x = 0, y = 0;
    std::uint32_t buttons = 0;
    friend bool operator==(const Pointer&, const Pointer&) = default;
};
inline std::uint64_t packPointer(const Pointer& p) {
    if (!p.on) {
        return static_cast<std::uint64_t>(p.buttons & 0xff) << 32;
    }
    return (static_cast<std::uint64_t>(std::min<std::uint32_t>(p.x, 0xfffe) + 1)) |
           (static_cast<std::uint64_t>(std::min<std::uint32_t>(p.y, 0xffff)) << 16) |
           (static_cast<std::uint64_t>(p.buttons & 0xff) << 32);
}
inline Pointer unpackPointer(std::uint64_t v) {
    Pointer p;
    const auto x1 = static_cast<std::uint32_t>(v & 0xffff);
    p.on = x1 != 0;
    p.x = p.on ? x1 - 1 : 0;
    p.y = static_cast<std::uint32_t>((v >> 16) & 0xffff);
    p.buttons = static_cast<std::uint32_t>((v >> 32) & 0xff);
    return p;
}

inline std::uint64_t monoNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}
inline bool fresh(std::uint64_t beatNs, std::uint64_t now) { return beatNs != 0 && now >= beatNs && now - beatNs < kBeatTimeoutNs; }

inline std::string defaultPath() { return (status::runtime_dir() / "vrpanel.shm").string(); }

/// The mapped file. Both sides use the same class.
class Link {
public:
    Link() = default;
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;
    ~Link() { close(); }

    /// Map `path` (made at its full size when missing or short). False with errno words in `error()`.
    bool open(const std::string& path = defaultPath()) {
        close();
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
        if (fd < 0) {
            error_ = "cannot open " + path + ": " + std::strerror(errno);
            return false;
        }
        struct stat st{};
        if (::fstat(fd, &st) != 0 || static_cast<std::size_t>(st.st_size) < kFileBytes) {
            if (::ftruncate(fd, static_cast<off_t>(kFileBytes)) != 0) {
                error_ = "cannot size " + path + ": " + std::strerror(errno);
                ::close(fd);
                return false;
            }
        }
        void* m = ::mmap(nullptr, kFileBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (m == MAP_FAILED) {
            error_ = "cannot map " + path + ": " + std::strerror(errno);
            return false;
        }
        base_ = static_cast<std::uint8_t*>(m);
        auto* h = header();
        if (h->magic != kMagic || h->version != kVersion) {
            // a new (zero) file or another version: start clean; the other side sees the magic on its next check
            std::memset(base_, 0, kHeaderBytes);
            h->version = kVersion;
            std::atomic_thread_fence(std::memory_order_release);
            h->magic = kMagic;
        }
        return true;
    }
    void close() {
        if (base_ != nullptr) {
            ::munmap(base_, kFileBytes);
            base_ = nullptr;
        }
    }
    bool isOpen() const { return base_ != nullptr; }
    const std::string& error() const { return error_; }
    Header* header() const { return reinterpret_cast<Header*>(base_); }
    std::uint8_t* pixels() const { return base_ + kHeaderBytes; }

    // ---- the app
    /// Write one picture (BGRA, `stride` bytes per row); false when too big or not open.
    bool writeFrame(const std::uint8_t* src, std::uint32_t w, std::uint32_t h, std::size_t stride) {
        if (!isOpen() || w == 0 || h == 0 || w > kMaxW || h > kMaxH) {
            return false;
        }
        auto* hd = header();
        const std::uint64_t s = hd->frameSeq.load(std::memory_order_relaxed);
        hd->frameSeq.store(s | 1, std::memory_order_relaxed); // odd: writing
        std::atomic_thread_fence(std::memory_order_release);
        const std::size_t row = static_cast<std::size_t>(w) * 4;
        for (std::uint32_t y = 0; y < h; y++) {
            std::memcpy(pixels() + y * row, src + y * stride, row);
        }
        hd->width.store(w, std::memory_order_relaxed);
        hd->height.store(h, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        hd->frameSeq.store((s | 1) + 1, std::memory_order_release);
        return true;
    }

    // ---- pibridge-home
    /// Copy the newest whole picture newer than `afterSeq` into `out` (w * h * 4 bytes); its seq, or nullopt.
    std::optional<std::uint64_t> readFrame(std::uint64_t afterSeq, std::vector<std::uint8_t>& out, std::uint32_t& w,
                                           std::uint32_t& h) const {
        if (!isOpen()) {
            return std::nullopt;
        }
        const auto* hd = header();
        const std::uint64_t s1 = hd->frameSeq.load(std::memory_order_acquire);
        if ((s1 & 1) != 0 || s1 == 0 || s1 == afterSeq) {
            return std::nullopt;
        }
        const std::uint32_t ww = hd->width.load(std::memory_order_relaxed);
        const std::uint32_t hh = hd->height.load(std::memory_order_relaxed);
        if (ww == 0 || hh == 0 || ww > kMaxW || hh > kMaxH) {
            return std::nullopt;
        }
        out.resize(static_cast<std::size_t>(ww) * hh * 4);
        std::memcpy(out.data(), pixels(), out.size());
        std::atomic_thread_fence(std::memory_order_acquire);
        if (hd->frameSeq.load(std::memory_order_relaxed) != s1) {
            return std::nullopt; // torn: the next frame tries again
        }
        w = ww;
        h = hh;
        return s1;
    }

private:
    std::uint8_t* base_ = nullptr;
    std::string error_;
};

} // namespace pibridge::vrpanel
