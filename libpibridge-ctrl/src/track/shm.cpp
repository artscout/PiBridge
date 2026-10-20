// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/shm.rs (MIT OR Apache-2.0).
// The seqlock is the Rust one: the same atomic orders and fence placement on the u32 at offset 8.
#include "pibridge/ctrl/track/shm.h"

#include "pibridge/cam/rustfmt.h"

#include <atomic>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::ctrl::track::shm {

namespace {

std::atomic_ref<std::uint32_t> seq(std::uint8_t* p) {
    return std::atomic_ref<std::uint32_t>(*reinterpret_cast<std::uint32_t*>(p + 8));
}

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

/// Map::open of the Rust code: the file opened (created and sized for a writer), mapped shared, closed again.
cam::Result<std::uint8_t*> map_open(const std::string& path, bool write) {
    const int fd = ::open(path.c_str(), (write ? O_RDWR | O_CREAT : O_RDONLY) | O_CLOEXEC, 0644);
    if (fd < 0) {
        return cam::fail(cam::IoError::last_os());
    }
    const auto close_fail = [fd](cam::IoError e) {
        ::close(fd);
        return cam::fail(std::move(e));
    };
    if (write) {
        if (::ftruncate(fd, static_cast<off_t>(SIZE)) != 0) {
            return close_fail(cam::IoError::last_os());
        }
    } else {
        struct stat st {};
        if (::fstat(fd, &st) != 0) {
            return close_fail(cam::IoError::last_os());
        }
        if (static_cast<std::uint64_t>(st.st_size) < SIZE) {
            return close_fail(cam::IoError::custom(cam::ErrorKind::InvalidData, "pose file too short"));
        }
    }
    void* p = ::mmap(nullptr, SIZE, PROT_READ | (write ? PROT_WRITE : 0), MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        return close_fail(cam::IoError::last_os());
    }
    ::close(fd);
    return static_cast<std::uint8_t*>(p);
}

} // namespace

std::string default_path() {
    return cam::path_join(cam::env_os("XDG_RUNTIME_DIR").value_or("/tmp"), "pimax-ctrltrack.pose");
}

std::string inhand_status_path() {
    // default_path().with_file_name("pimax-ctrl-inhand")
    const std::string p = default_path();
    const std::size_t slash = p.rfind('/');
    return (slash == std::string::npos ? std::string() : p.substr(0, slash + 1)) + "pimax-ctrl-inhand";
}

std::array<bool, 2> inhand_wanted(const std::string& path) {
    const auto text = cam::read_to_string(path);
    if (!text) {
        return {true, true};
    }
    return parse_inhand(*text, [](std::uint32_t pid) { return cam::path_exists("/proc/" + std::to_string(pid)); });
}

std::array<bool, 2> parse_inhand(std::string_view text, const std::function<bool(std::uint32_t)>& alive) {
    const auto get_key = [&](std::string_view k) -> std::optional<std::string_view> {
        std::size_t at = 0;
        while (at < text.size()) {
            std::size_t nl = text.find('\n', at);
            std::string_view line = text.substr(at, nl == std::string_view::npos ? std::string_view::npos : nl - at);
            if (!line.empty() && line.back() == '\r') {
                line.remove_suffix(1);
            }
            if (line.starts_with(k) && line.size() > k.size() && line[k.size()] == '=') {
                return cam::rfmt::trim(line.substr(k.size() + 1));
            }
            if (nl == std::string_view::npos) {
                break;
            }
            at = nl + 1;
        }
        return std::nullopt;
    };
    const auto pid_s = get_key("pid");
    std::uint32_t pid = 0;
    if (!pid_s || !cam::rfmt::parse_u32(*pid_s, pid) || !alive(pid)) {
        return {true, true};
    }
    const auto want = [&](std::string_view k) {
        const auto v = get_key(k);
        return !(v && (*v == "down" || *v == "off"));
    };
    return {want("left"), want("right")};
}

std::uint64_t monotonic_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

cam::Result<std::unique_ptr<PoseWriter>> PoseWriter::create(const std::string& path) {
    auto m = map_open(path, true);
    if (!m) {
        return cam::fail(m.error());
    }
    std::unique_ptr<PoseWriter> w(new PoseWriter());
    w->p_ = *m;
    put<std::uint32_t>(w->p_, 0, MAGIC);
    put<std::uint32_t>(w->p_, 4, VERSION);
    put<std::uint32_t>(w->p_, 12, static_cast<std::uint32_t>(::getpid()));
    seq(w->p_).store(0, std::memory_order_release);
    w->flush();
    return w;
}

PoseWriter::~PoseWriter() {
    if (p_ != nullptr) {
        ::munmap(p_, SIZE);
    }
}

void PoseWriter::set(std::size_t hand, const HandPose& h) {
    hands_[hand] = h;
    flush();
}

void PoseWriter::beat(const Beat& b) {
    beat_ = b;
    flush();
}

void PoseWriter::flush() {
    auto s = seq(p_);
    std::uint32_t v = s.load(std::memory_order_relaxed);
    s.store((v + 1) | 1u, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (std::size_t i = 0; i < 2; i++) {
        const HandPose& h = hands_[i];
        const std::size_t o = HAND_OFF + i * HAND_STRIDE;
        put(p_, o, h.frame_ns);
        put(p_, o + 8, h.publish_ns);
        for (std::size_t k = 0; k < 3; k++) {
            put(p_, o + 16 + 4 * k, h.pos[k]);
        }
        for (std::size_t k = 0; k < 4; k++) {
            put(p_, o + 28 + 4 * k, h.quat[k]);
            put(p_, o + 56 + 4 * k, h.aim_from_imu[k]);
        }
        put(p_, o + 44, h.rms_px);
        put(p_, o + 48, h.leds);
        put(p_, o + 52, h.cams);
    }
    put(p_, BEAT_OFF, beat_.beat_ns);
    put(p_, BEAT_OFF + 8, beat_.led_frames);
    put<std::uint32_t>(p_, BEAT_OFF + 16, beat_.source_up ? 1u : 0u);
    put(p_, BEAT_OFF + 20, beat_.reopens);
    std::atomic_thread_fence(std::memory_order_release);
    v = s.load(std::memory_order_relaxed);
    s.store((v + 1) & ~1u, std::memory_order_release);
}

cam::Result<std::unique_ptr<PoseReader>> PoseReader::open(const std::string& path) {
    auto m = map_open(path, false);
    if (!m) {
        return cam::fail(m.error());
    }
    std::unique_ptr<PoseReader> r(new PoseReader());
    r->p_ = *m;
    return r;
}

PoseReader::~PoseReader() {
    if (p_ != nullptr) {
        ::munmap(p_, SIZE);
    }
}

std::optional<std::array<HandPose, 2>> PoseReader::read() const {
    auto a = read_all();
    if (!a) {
        return std::nullopt;
    }
    return a->first;
}

std::optional<std::pair<std::array<HandPose, 2>, Beat>> PoseReader::read_all() const {
    auto s = seq(p_);
    for (int tries = 0; tries < 1000; tries++) {
        const std::uint32_t s1 = s.load(std::memory_order_acquire);
        if ((s1 & 1u) == 1u) {
            continue;
        }
        if (get<std::uint32_t>(p_, 0) != MAGIC) {
            return std::nullopt;
        }
        std::array<HandPose, 2> out{};
        for (std::size_t i = 0; i < 2; i++) {
            HandPose& h = out[i];
            const std::size_t o = HAND_OFF + i * HAND_STRIDE;
            h.frame_ns = get<std::uint64_t>(p_, o);
            h.publish_ns = get<std::uint64_t>(p_, o + 8);
            h.pos = {get<float>(p_, o + 16), get<float>(p_, o + 20), get<float>(p_, o + 24)};
            h.quat = {get<float>(p_, o + 28), get<float>(p_, o + 32), get<float>(p_, o + 36), get<float>(p_, o + 40)};
            h.rms_px = get<float>(p_, o + 44);
            h.leds = get<std::uint32_t>(p_, o + 48);
            h.cams = get<std::uint32_t>(p_, o + 52);
            h.aim_from_imu = {get<float>(p_, o + 56), get<float>(p_, o + 60), get<float>(p_, o + 64),
                              get<float>(p_, o + 68)};
        }
        Beat beat;
        beat.beat_ns = get<std::uint64_t>(p_, BEAT_OFF);
        beat.led_frames = get<std::uint64_t>(p_, BEAT_OFF + 8);
        beat.source_up = get<std::uint32_t>(p_, BEAT_OFF + 16) != 0;
        beat.reopens = get<std::uint32_t>(p_, BEAT_OFF + 20);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s.load(std::memory_order_relaxed) == s1) {
            return std::pair{out, beat};
        }
    }
    return std::nullopt;
}

} // namespace pibridge::ctrl::track::shm
