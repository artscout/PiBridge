// SPDX-License-Identifier: MIT
//
// The fuser -> tracker hint file (PiBridge; docs/research/ctrl-tracking-improvements.md). The controller fuser runs in
// Monado and knows each controller's IMU orientation and inertial position; the optical tracker (pibridge-ctrltrack live)
// runs in its own process and knows only the LED frames. The fuser writes the pose it shows, relative to the head, into
// a small shared file; the tracker reads it at each LED frame's time and uses it as the rotation prior of few-LED fixes,
// as a prior to relock from, and to check left / right in the search.
//
// File `$XDG_RUNTIME_DIR/pibridge-ctrl-hint` (`/tmp` without XDG_RUNTIME_DIR), 5168 bytes, little-endian, CLOCK_MONOTONIC
// nanoseconds (the clock of the LED frames and of the pose file):
//
// | offset | type | field |
// |---|---|---|
// | 0 | u32 | magic 0x54484346 ("FCHT") |
// | 4 | u32 | version 3 |
// | 8 | u32 | writer pid |
// | 12 | u32 | reserved (0) |
// | 16 + 2576 h | | hand h (0 left, 1 right): |
// | +0 | u32 | seqlock: odd while the writer changes this hand's block |
// | +4 | u32 | entries written so far (the ring position is count % 16) |
// | +8 | u64 | reserved (0) |
// | +16 + 160 k | | ring entry k of 16: |
// | +0 | u64 | t_ns: time of the pose |
// | +8 | u64 | fix_ns: frame time of the last optical fix the fuser accepted (0 = none) |
// | +16 | u32 | state: 0 none, 1 optical (fresh fix), 2 held (out of view) |
// | +20 | u32 | flags: bit 0 = gyro calibrated (fitted or saved), bit 1 = up_aim reads about 1 g, bit 2 = pose valid |
// | +24 | f32[3] | head_from_aim position (m, OpenXR head axes) |
// | +36 | f32[4] | head_from_aim orientation x, y, z, w |
// | +52 | f32[3] | angular velocity in the aim frame (rad/s), from the gyro |
// | +64 | f32[4] | gyro-integrated orientation x, y, z, w (aim frame, arbitrary origin: conj(q0) q1 is the turn) |
// | +80 | f32[3] | gravity up in the aim frame (unit, low-passed accelerometer) |
// | +92 | f32 | seconds the gyro has been quiet (0 while it moves) |
// | +96 | f32[3] | gravity up in the head frame (unit, from the head pose) |
// | +108 | f32[3] | mean specific force in the aim frame since the previous entry (m/s^2, accelerometer) |
// | +120 | f32[3] | world_from_head position (the head pose at t_ns) |
// | +132 | f32[4] | world_from_head orientation x, y, z, w |
// | +148 | u32, u64 | reserved (0) |
//
// The IMU fields (+64..+148) are written whenever the gyro streams, also when the fuser has no pose (state 0).
//
// One writer per hand (each hand's fuser has its own lock), so each hand has its own seqlock; a reader copies a block
// and retries while the seqlock is odd or changed. A missing file, a wrong magic or version, or a writer that is gone
// (no /proc/<pid>) reads as no hint.
#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace pibridge::ctrl::hint {

inline constexpr std::uint32_t MAGIC = 0x54484346;
inline constexpr std::uint32_t VERSION = 3;
inline constexpr std::size_t RING = 16;
inline constexpr std::size_t ENTRY = 160;
inline constexpr std::size_t HAND = 16 + RING * ENTRY;
inline constexpr std::size_t SIZE = 16 + 2 * HAND;

struct Entry {
    std::uint64_t t_ns = 0;
    std::uint64_t fix_ns = 0;
    std::uint32_t state = 0;
    std::uint32_t flags = 0;
    std::array<float, 3> pos{};
    std::array<float, 4> quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> w{};
    std::array<float, 4> qg{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> up_aim{};
    float still_s = 0.0f;
    std::array<float, 3> up_head{0.0f, 1.0f, 0.0f};
    std::array<float, 3> f_aim{};
    std::array<float, 3> head_pos{};
    std::array<float, 4> head_quat{0.0f, 0.0f, 0.0f, 1.0f};
};

inline std::string default_path() {
    const char* d = std::getenv("XDG_RUNTIME_DIR");
    return std::string(d != nullptr && *d != 0 ? d : "/tmp") + "/pibridge-ctrl-hint";
}

namespace detail {
template <class T>
void put(std::uint8_t* p, std::size_t off, T v) {
    std::memcpy(p + off, &v, sizeof v);
}
template <class T>
T get(const std::uint8_t* p, std::size_t off) {
    T v;
    std::memcpy(&v, p + off, sizeof v);
    return v;
}
inline std::atomic_ref<std::uint32_t> seq(std::uint8_t* p, std::size_t hand) {
    return std::atomic_ref<std::uint32_t>(*reinterpret_cast<std::uint32_t*>(p + 16 + hand * HAND));
}
inline void put_entry(std::uint8_t* p, const Entry& e) {
    put(p, 0, e.t_ns);
    put(p, 8, e.fix_ns);
    put(p, 16, e.state);
    put(p, 20, e.flags);
    for (std::size_t i = 0; i < 3; i++) {
        put(p, 24 + 4 * i, e.pos[i]);
        put(p, 52 + 4 * i, e.w[i]);
    }
    for (std::size_t i = 0; i < 4; i++) {
        put(p, 36 + 4 * i, e.quat[i]);
        put(p, 64 + 4 * i, e.qg[i]);
    }
    for (std::size_t i = 0; i < 3; i++) {
        put(p, 80 + 4 * i, e.up_aim[i]);
        put(p, 96 + 4 * i, e.up_head[i]);
    }
    put(p, 92, e.still_s);
    for (std::size_t i = 0; i < 3; i++) {
        put(p, 108 + 4 * i, e.f_aim[i]);
        put(p, 120 + 4 * i, e.head_pos[i]);
    }
    for (std::size_t i = 0; i < 4; i++) {
        put(p, 132 + 4 * i, e.head_quat[i]);
    }
}
inline Entry get_entry(const std::uint8_t* p) {
    Entry e;
    e.t_ns = get<std::uint64_t>(p, 0);
    e.fix_ns = get<std::uint64_t>(p, 8);
    e.state = get<std::uint32_t>(p, 16);
    e.flags = get<std::uint32_t>(p, 20);
    for (std::size_t i = 0; i < 3; i++) {
        e.pos[i] = get<float>(p, 24 + 4 * i);
        e.w[i] = get<float>(p, 52 + 4 * i);
    }
    for (std::size_t i = 0; i < 4; i++) {
        e.quat[i] = get<float>(p, 36 + 4 * i);
        e.qg[i] = get<float>(p, 64 + 4 * i);
    }
    for (std::size_t i = 0; i < 3; i++) {
        e.up_aim[i] = get<float>(p, 80 + 4 * i);
        e.up_head[i] = get<float>(p, 96 + 4 * i);
    }
    e.still_s = get<float>(p, 92);
    for (std::size_t i = 0; i < 3; i++) {
        e.f_aim[i] = get<float>(p, 108 + 4 * i);
        e.head_pos[i] = get<float>(p, 120 + 4 * i);
    }
    for (std::size_t i = 0; i < 4; i++) {
        e.head_quat[i] = get<float>(p, 132 + 4 * i);
    }
    return e;
}
inline std::uint8_t* map(const std::string& path, bool write) {
    const int fd = ::open(path.c_str(), (write ? O_RDWR | O_CREAT : O_RDONLY) | O_CLOEXEC, 0644);
    if (fd < 0) {
        return nullptr;
    }
    struct stat st {};
    if (write ? ::ftruncate(fd, static_cast<off_t>(SIZE)) != 0
              : (::fstat(fd, &st) != 0 || static_cast<std::uint64_t>(st.st_size) < SIZE)) {
        ::close(fd);
        return nullptr;
    }
    void* p = ::mmap(nullptr, SIZE, PROT_READ | (write ? PROT_WRITE : 0), MAP_SHARED, fd, 0);
    ::close(fd);
    return p == MAP_FAILED ? nullptr : static_cast<std::uint8_t*>(p);
}
} // namespace detail

/// The fuser's side. Created once per process (both hands share it); `write` is called with that hand's lock held.
class Writer {
public:
    static Writer* open(const std::string& path) {
        std::uint8_t* p = detail::map(path, true);
        if (p == nullptr) {
            return nullptr;
        }
        std::memset(p, 0, SIZE);
        detail::put(p, 4, VERSION);
        detail::put(p, 8, static_cast<std::uint32_t>(::getpid()));
        std::atomic_thread_fence(std::memory_order_release);
        detail::put(p, 0, MAGIC);
        auto* w = new Writer();
        w->p_ = p;
        return w;
    }
    void write(std::size_t hand, const Entry& e) {
        auto s = detail::seq(p_, hand);
        const std::uint32_t v = s.load(std::memory_order_relaxed);
        s.store(v + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        std::uint8_t* h = p_ + 16 + hand * HAND;
        const std::uint32_t n = detail::get<std::uint32_t>(h, 4);
        detail::put_entry(h + 16 + (n % RING) * ENTRY, e);
        detail::put(h, 4, n + 1);
        std::atomic_thread_fence(std::memory_order_release);
        s.store(v + 2, std::memory_order_release);
    }

private:
    std::uint8_t* p_ = nullptr;
};

/// The tracker's side.
class Reader {
public:
    static Reader* open(const std::string& path) {
        std::uint8_t* p = detail::map(path, false);
        if (p == nullptr) {
            return nullptr;
        }
        auto* r = new Reader();
        r->p_ = p;
        return r;
    }
    ~Reader() {
        if (p_ != nullptr) {
            ::munmap(p_, SIZE);
        }
    }
    /// The writer is there (magic, version, a live pid).
    bool live() const {
        if (detail::get<std::uint32_t>(p_, 0) != MAGIC || detail::get<std::uint32_t>(p_, 4) != VERSION) {
            return false;
        }
        const std::uint32_t pid = detail::get<std::uint32_t>(p_, 8);
        struct stat st {};
        return pid != 0 && ::stat(("/proc/" + std::to_string(pid)).c_str(), &st) == 0;
    }
    /// The hand's ring, oldest first (empty when the writer is busy for too long or nothing was written).
    std::vector<Entry> read(std::size_t hand) const {
        for (int tries = 0; tries < 64; tries++) {
            auto s = detail::seq(p_, hand);
            const std::uint32_t a = s.load(std::memory_order_acquire);
            if (a & 1u) {
                continue;
            }
            std::array<std::uint8_t, HAND> copy{};
            std::memcpy(copy.data(), p_ + 16 + hand * HAND, HAND);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (s.load(std::memory_order_relaxed) != a) {
                continue;
            }
            const std::uint32_t n = detail::get<std::uint32_t>(copy.data(), 4);
            std::vector<Entry> out;
            const std::uint32_t have = n < RING ? n : static_cast<std::uint32_t>(RING);
            for (std::uint32_t k = n - have; k != n; k++) {
                out.push_back(detail::get_entry(copy.data() + 16 + (k % RING) * ENTRY));
            }
            return out;
        }
        return {};
    }

private:
    std::uint8_t* p_ = nullptr;
};

/// A hint at time t from a hand's ring: interpolated between the two entries around t (nlerp), or the newest one
/// carried on by its angular velocity for up to `max_ahead_ns` past it. nullopt when t is outside the ring, or the
/// entries there have no pose (state 0).
inline std::optional<Entry> at(const std::vector<Entry>& ring, std::uint64_t t, std::uint64_t max_ahead_ns = 60000000) {
    if (ring.empty()) {
        return std::nullopt;
    }
    const Entry& last = ring.back();
    if (t >= last.t_ns) {
        if (t - last.t_ns > max_ahead_ns || last.state == 0) {
            return std::nullopt;
        }
        Entry e = last;
        const double dt = static_cast<double>(t - last.t_ns) * 1e-9;
        const double wx = e.w[0] * dt, wy = e.w[1] * dt, wz = e.w[2] * dt;
        const double a = std::sqrt(wx * wx + wy * wy + wz * wz);
        if (a > 1e-9) {
            const double s = std::sin(a / 2) / a;
            const double dq[4] = {wx * s, wy * s, wz * s, std::cos(a / 2)};
            const double q[4] = {e.quat[0], e.quat[1], e.quat[2], e.quat[3]};
            // q * dq (the rate is in the aim frame)
            e.quat = {static_cast<float>(q[3] * dq[0] + q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1]),
                      static_cast<float>(q[3] * dq[1] - q[0] * dq[2] + q[1] * dq[3] + q[2] * dq[0]),
                      static_cast<float>(q[3] * dq[2] + q[0] * dq[1] - q[1] * dq[0] + q[2] * dq[3]),
                      static_cast<float>(q[3] * dq[3] - q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2])};
        }
        e.t_ns = t;
        return e;
    }
    for (std::size_t k = ring.size() - 1; k > 0; k--) {
        const Entry& b = ring[k];
        const Entry& a = ring[k - 1];
        if (a.t_ns <= t && t <= b.t_ns) {
            if (a.state == 0 || b.state == 0) {
                return std::nullopt;
            }
            const double f = b.t_ns > a.t_ns ? static_cast<double>(t - a.t_ns) / static_cast<double>(b.t_ns - a.t_ns) : 0.0;
            Entry e = f < 0.5 ? a : b;
            e.t_ns = t;
            double dot = 0.0;
            for (std::size_t i = 0; i < 4; i++) {
                dot += static_cast<double>(a.quat[i]) * b.quat[i];
            }
            const double sb = dot < 0.0 ? -1.0 : 1.0;
            double n = 0.0;
            std::array<double, 4> q{};
            for (std::size_t i = 0; i < 4; i++) {
                q[i] = a.quat[i] + (sb * b.quat[i] - a.quat[i]) * f;
                n += q[i] * q[i];
            }
            n = std::sqrt(n);
            for (std::size_t i = 0; i < 4; i++) {
                e.quat[i] = static_cast<float>(q[i] / n);
            }
            for (std::size_t i = 0; i < 3; i++) {
                e.pos[i] = static_cast<float>(a.pos[i] + (b.pos[i] - a.pos[i]) * f);
            }
            return e;
        }
    }
    return std::nullopt;
}

} // namespace pibridge::ctrl::hint
