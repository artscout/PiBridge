// SPDX-License-Identifier: MIT
// The desktop in the headset (docs/desktop.md): one shared-memory file between pibridge-desktop (the capture service)
// and the overlay that shows it (pibridge-home's dashboard overlay under Monado, pibridge-overlay under SteamVR),
// $XDG_RUNTIME_DIR/pibridge/desktop.shm (tmpfs: pages are used only as far as written).
//
//   pibridge-desktop -> the monitors (where each one is on the desktop, its size), each monitor's picture (BGRA 8-bit
//                       sRGB, top row first, under its own sequence lock: frameSeq odd while writing), the state and a
//                       line of status words; serviceBeatNs every tick
//   the overlay      -> clientBeatNs while it shows the desktop (the service captures only while that is under 2 s
//                       old), `want` (a bit per monitor it shows), the controller pointer: `pointer` (the monitor, x, y
//                       in its pixels, buttons) and `wheel` (a running total, 120 per notch; the service sends the
//                       difference)
//
// Either side may be missing or restart at any time: the file is made by whoever comes first, a stale beat means "not
// there", and nothing waits on the other side. Header-only, no Qt (both sides include it).
#pragma once

#include "pibridge_status.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace pibridge::desktop {

inline constexpr std::uint32_t kMagic = 0x4b534544; // "DESK"
inline constexpr std::uint32_t kVersion = 1;
inline constexpr std::uint32_t kMaxMonitors = 4;
inline constexpr std::uint32_t kMaxW = 3840; // a monitor's picture; bigger ones are scaled down by the service
inline constexpr std::uint32_t kMaxH = 2160;
inline constexpr std::size_t kHeaderBytes = 8192;
inline constexpr std::size_t kSlotBytes = static_cast<std::size_t>(kMaxW) * kMaxH * 4;
inline constexpr std::size_t kFileBytes = kHeaderBytes + kSlotBytes * kMaxMonitors;
inline constexpr std::uint64_t kBeatTimeoutNs = 2'000'000'000ULL;
inline constexpr std::size_t kTextBytes = 256;
inline constexpr std::size_t kNameBytes = 32;
/// `want` before the overlay knows the monitors: every one (the service learns them only once it captures).
inline constexpr std::uint32_t kWantAll = (1U << kMaxMonitors) - 1;

/// What the service is doing.
enum class State : std::uint32_t {
    Off = 0,             // no capture (nobody asked, or it stopped)
    Starting = 1,        // the portal session is being set up
    Running = 2,         // the monitors' pictures come in
    NeedsPermission = 3, // KDE has not allowed it yet: `pibridge-desktop allow` on the desktop (the app's button)
    Failed = 4,          // see the status words
};

struct Monitor {
    std::atomic<std::uint64_t> frameSeq; // even: a whole picture; odd: being written
    std::atomic<std::uint32_t> width;    // the picture's pixels
    std::atomic<std::uint32_t> height;
    std::atomic<std::int32_t> x; // its place on the desktop (logical pixels, the portal's position)
    std::atomic<std::int32_t> y;
    std::atomic<std::uint32_t> logicalW; // its size on the desktop (logical pixels: the pointer's units)
    std::atomic<std::uint32_t> logicalH;
    char name[kNameBytes]; // under the layout's lock
};

struct Header {
    std::uint32_t magic;
    std::uint32_t version;
    // the service's side
    std::atomic<std::uint64_t> serviceBeatNs;
    std::atomic<std::uint32_t> servicePid;
    std::atomic<std::uint32_t> state; // State
    std::atomic<std::uint64_t> textSeq; // the status words' lock (odd while writing)
    char text[kTextBytes];
    std::atomic<std::uint64_t> layoutSeq; // the monitors' places and names (odd while writing)
    std::atomic<std::uint32_t> monitorCount;
    Monitor monitors[kMaxMonitors];
    // the overlay's side
    std::atomic<std::uint64_t> clientBeatNs;
    std::atomic<std::uint32_t> clientPid;
    std::atomic<std::uint32_t> want;    // bit i: monitor i is shown
    std::atomic<std::uint64_t> pointer; // packPointer
    std::atomic<std::int64_t> wheel;    // running total, 120 per notch (+ = away from you: scroll up)
};
static_assert(sizeof(Header) <= kHeaderBytes);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
// PiBridge World reads and writes the file by these offsets (world/scripts/desktop.gd): keep both in step
static_assert(offsetof(Header, serviceBeatNs) == 8 && offsetof(Header, state) == 20 && offsetof(Header, textSeq) == 24 &&
              offsetof(Header, text) == 32 && offsetof(Header, layoutSeq) == 288 && offsetof(Header, monitorCount) == 296 &&
              offsetof(Header, monitors) == 304 && sizeof(Monitor) == 64 && offsetof(Monitor, width) == 8 &&
              offsetof(Monitor, x) == 16 && offsetof(Monitor, logicalW) == 24 && offsetof(Monitor, name) == 32 &&
              offsetof(Header, clientBeatNs) == 560 && offsetof(Header, want) == 572 && offsetof(Header, pointer) == 576 &&
              offsetof(Header, wheel) == 584);

/// The pointer on a monitor: x, y in that monitor's picture pixels (on = false: off every monitor), buttons bit 0 =
/// left, bit 1 = right, bit 2 = middle.
struct Pointer {
    bool on = false;
    std::uint32_t monitor = 0;
    std::uint32_t x = 0, y = 0;
    std::uint32_t buttons = 0;
    friend bool operator==(const Pointer&, const Pointer&) = default;
};
inline std::uint64_t packPointer(const Pointer& p) {
    std::uint64_t v = static_cast<std::uint64_t>(p.buttons & 0xff) << 32;
    if (p.on) {
        v |= static_cast<std::uint64_t>(std::min<std::uint32_t>(p.x, 0xfffe) + 1) |
             (static_cast<std::uint64_t>(std::min<std::uint32_t>(p.y, 0xffff)) << 16) |
             (static_cast<std::uint64_t>(p.monitor & 0x7) << 40);
    }
    return v;
}
inline Pointer unpackPointer(std::uint64_t v) {
    Pointer p;
    const auto x1 = static_cast<std::uint32_t>(v & 0xffff);
    p.on = x1 != 0;
    p.x = p.on ? x1 - 1 : 0;
    p.y = p.on ? static_cast<std::uint32_t>((v >> 16) & 0xffff) : 0;
    p.buttons = static_cast<std::uint32_t>((v >> 32) & 0xff);
    p.monitor = p.on ? static_cast<std::uint32_t>((v >> 40) & 0x7) : 0;
    return p;
}

inline std::uint64_t monoNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}
inline bool fresh(std::uint64_t beatNs, std::uint64_t now) { return beatNs != 0 && now >= beatNs && now - beatNs < kBeatTimeoutNs; }

inline std::string defaultPath() { return (status::runtime_dir() / "desktop.shm").string(); }

/// pibridge-desktop: next to the running program (the install, or the build tree's tools/desktop), else on PATH; ""
/// when there is none.
inline std::string toolPath() {
    char self[4096];
    const ssize_t n = ::readlink("/proc/self/exe", self, sizeof self - 1);
    if (n > 0) {
        self[n] = '\0';
        std::string dir(self);
        dir = dir.substr(0, dir.rfind('/'));
        for (const std::string& c : {dir + "/pibridge-desktop", dir + "/../desktop/pibridge-desktop"}) {
            if (::access(c.c_str(), X_OK) == 0) {
                return c;
            }
        }
    }
    if (const char* path = std::getenv("PATH"); path != nullptr) {
        const std::string ps(path);
        std::size_t at = 0;
        while (at <= ps.size()) {
            const std::size_t end = std::min(ps.find(':', at), ps.size());
            const std::string c = ps.substr(at, end - at) + "/pibridge-desktop";
            if (end > at && ::access(c.c_str(), X_OK) == 0) {
                return c;
            }
            at = end + 1;
        }
    }
    return {};
}

/// `<tool> run` on its own: a grandchild in its own session (never the caller's zombie); it is one at a time by itself
/// and ends itself once nobody looks at the desktop.
inline void startService(const std::string& tool) {
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::setsid();
        if (::fork() == 0) {
            const char* argv[] = {tool.c_str(), "run", nullptr};
            ::execv(tool.c_str(), const_cast<char* const*>(argv));
            ::_exit(127);
        }
        ::_exit(0);
    }
    if (pid > 0) {
        ::waitpid(pid, nullptr, 0);
    }
}

/// How the source's pixels are laid out (the service converts them to BGRA while copying).
enum class Source { BGRA, BGRX, RGBA, RGBX };

/// One monitor as the overlay sees it.
struct MonitorInfo {
    std::uint32_t index = 0;
    std::int32_t x = 0, y = 0;
    std::uint32_t logicalW = 0, logicalH = 0;
    std::uint32_t width = 0, height = 0; // the picture (0 until the first frame)
    std::string name;
};

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
    std::uint8_t* slot(std::uint32_t i) const { return base_ + kHeaderBytes + kSlotBytes * i; }

    // ---- both
    std::string text() const {
        const auto* h = header();
        for (int tries = 0; tries < 4; tries++) {
            const std::uint64_t s = h->textSeq.load(std::memory_order_acquire);
            if ((s & 1) != 0) {
                continue;
            }
            std::string t(h->text, strnlen(h->text, kTextBytes));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (h->textSeq.load(std::memory_order_relaxed) == s) {
                return t;
            }
        }
        return {};
    }
    State state() const { return static_cast<State>(header()->state.load(std::memory_order_acquire)); }
    /// The monitors, in their order (left to right), with the pictures' sizes.
    std::vector<MonitorInfo> monitors() const {
        const auto* h = header();
        for (int tries = 0; tries < 4; tries++) {
            const std::uint64_t s = h->layoutSeq.load(std::memory_order_acquire);
            if ((s & 1) != 0) {
                continue;
            }
            std::vector<MonitorInfo> out;
            const std::uint32_t n = std::min(h->monitorCount.load(std::memory_order_relaxed), kMaxMonitors);
            for (std::uint32_t i = 0; i < n; i++) {
                const Monitor& m = h->monitors[i];
                MonitorInfo mi;
                mi.index = i;
                mi.x = m.x.load(std::memory_order_relaxed);
                mi.y = m.y.load(std::memory_order_relaxed);
                mi.logicalW = m.logicalW.load(std::memory_order_relaxed);
                mi.logicalH = m.logicalH.load(std::memory_order_relaxed);
                mi.width = m.width.load(std::memory_order_relaxed);
                mi.height = m.height.load(std::memory_order_relaxed);
                mi.name.assign(m.name, strnlen(m.name, kNameBytes));
                out.push_back(std::move(mi));
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if (h->layoutSeq.load(std::memory_order_relaxed) == s) {
                return out;
            }
        }
        return {};
    }

    // ---- pibridge-desktop
    void setState(State st, const std::string& words) {
        auto* h = header();
        const std::uint64_t s = h->textSeq.load(std::memory_order_relaxed);
        h->textSeq.store(s | 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        const std::size_t n = std::min(words.size(), kTextBytes - 1);
        std::memcpy(h->text, words.data(), n);
        h->text[n] = '\0';
        h->textSeq.store((s | 1) + 1, std::memory_order_release);
        h->state.store(static_cast<std::uint32_t>(st), std::memory_order_release);
    }
    /// The monitors' places, sizes and names (their pictures stay until written again).
    void setLayout(const std::vector<MonitorInfo>& mons) {
        auto* h = header();
        const std::uint64_t s = h->layoutSeq.load(std::memory_order_relaxed);
        h->layoutSeq.store(s | 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        const auto n = static_cast<std::uint32_t>(std::min<std::size_t>(mons.size(), kMaxMonitors));
        for (std::uint32_t i = 0; i < n; i++) {
            Monitor& m = h->monitors[i];
            m.x.store(mons[i].x, std::memory_order_relaxed);
            m.y.store(mons[i].y, std::memory_order_relaxed);
            m.logicalW.store(mons[i].logicalW, std::memory_order_relaxed);
            m.logicalH.store(mons[i].logicalH, std::memory_order_relaxed);
            const std::size_t nn = std::min(mons[i].name.size(), kNameBytes - 1);
            std::memcpy(m.name, mons[i].name.data(), nn);
            m.name[nn] = '\0';
        }
        h->monitorCount.store(n, std::memory_order_relaxed);
        h->layoutSeq.store((s | 1) + 1, std::memory_order_release);
    }
    /// Write monitor `i`'s picture (`stride` bytes per row, `src` laid out as `fmt`), as BGRA; false when too big.
    bool writeFrame(std::uint32_t i, const std::uint8_t* src, std::uint32_t w, std::uint32_t h, std::size_t stride, Source fmt) {
        if (!isOpen() || i >= kMaxMonitors || w == 0 || h == 0 || w > kMaxW || h > kMaxH) {
            return false;
        }
        Monitor& m = header()->monitors[i];
        const std::uint64_t s = m.frameSeq.load(std::memory_order_relaxed);
        m.frameSeq.store(s | 1, std::memory_order_relaxed); // odd: writing
        std::atomic_thread_fence(std::memory_order_release);
        std::uint8_t* dst = slot(i);
        const std::size_t row = static_cast<std::size_t>(w) * 4;
        for (std::uint32_t y = 0; y < h; y++) {
            convertRow(dst + y * row, src + y * stride, w, fmt);
        }
        m.width.store(w, std::memory_order_relaxed);
        m.height.store(h, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        m.frameSeq.store((s | 1) + 1, std::memory_order_release);
        return true;
    }
    static void convertRow(std::uint8_t* dst, const std::uint8_t* src, std::uint32_t w, Source fmt) {
        switch (fmt) {
        case Source::BGRA: std::memcpy(dst, src, static_cast<std::size_t>(w) * 4); return;
        case Source::BGRX:
            std::memcpy(dst, src, static_cast<std::size_t>(w) * 4);
            for (std::uint32_t x = 0; x < w; x++) {
                dst[x * 4 + 3] = 0xff;
            }
            return;
        case Source::RGBA:
        case Source::RGBX:
            for (std::uint32_t x = 0; x < w; x++) {
                dst[x * 4 + 0] = src[x * 4 + 2];
                dst[x * 4 + 1] = src[x * 4 + 1];
                dst[x * 4 + 2] = src[x * 4 + 0];
                dst[x * 4 + 3] = fmt == Source::RGBA ? src[x * 4 + 3] : 0xff;
            }
            return;
        }
    }

    // ---- the overlay
    /// Copy monitor `i`'s newest whole picture newer than `afterSeq` into `out` (w * h * 4 bytes); its seq, or nullopt.
    std::optional<std::uint64_t> readFrame(std::uint32_t i, std::uint64_t afterSeq, std::vector<std::uint8_t>& out,
                                           std::uint32_t& w, std::uint32_t& h) const {
        if (!isOpen() || i >= kMaxMonitors) {
            return std::nullopt;
        }
        const Monitor& m = header()->monitors[i];
        const std::uint64_t s1 = m.frameSeq.load(std::memory_order_acquire);
        if ((s1 & 1) != 0 || s1 == 0 || s1 == afterSeq) {
            return std::nullopt;
        }
        const std::uint32_t ww = m.width.load(std::memory_order_relaxed);
        const std::uint32_t hh = m.height.load(std::memory_order_relaxed);
        if (ww == 0 || hh == 0 || ww > kMaxW || hh > kMaxH) {
            return std::nullopt;
        }
        out.resize(static_cast<std::size_t>(ww) * hh * 4);
        std::memcpy(out.data(), slot(i), out.size());
        std::atomic_thread_fence(std::memory_order_acquire);
        if (m.frameSeq.load(std::memory_order_relaxed) != s1) {
            return std::nullopt; // torn: the next frame tries again
        }
        w = ww;
        h = hh;
        return s1;
    }
    /// The overlay's beat, the monitors it shows and the pointer, once a frame while the desktop is shown.
    void clientUpdate(std::uint32_t wantMask, const Pointer& p, std::int64_t wheelTotal) {
        auto* h = header();
        h->want.store(wantMask, std::memory_order_relaxed);
        h->pointer.store(packPointer(p), std::memory_order_relaxed);
        h->wheel.store(wheelTotal, std::memory_order_relaxed);
        h->clientPid.store(static_cast<std::uint32_t>(::getpid()), std::memory_order_relaxed);
        h->clientBeatNs.store(monoNs(), std::memory_order_release);
    }

private:
    std::uint8_t* base_ = nullptr;
    std::string error_;
};

} // namespace pibridge::desktop
