// SPDX-License-Identifier: MIT
#include "pibridge/hmd/pose_shm.h"

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::hmd::pose_shm {

std::filesystem::path default_path() {
    return "/dev/shm/pibridge-crystal-pose";
}

namespace {

static_assert(std::endian::native == std::endian::little, "the file is little endian, stored with memcpy");

template <typename T> void put(std::uint8_t* b, std::size_t o, T v) {
    std::memcpy(b + o, &v, sizeof v);
}

template <typename T> T at(std::span<const std::uint8_t> f, std::size_t o) {
    T v;
    std::memcpy(&v, f.data() + o, sizeof v);
    return v;
}

/// v rotated by the unit quaternion q (x y z w): q v q*.
std::array<float, 3> rotate(const std::array<float, 4>& q, const std::array<float, 3>& v) {
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    // t = 2 (q.xyz x v); v' = v + w t + q.xyz x t
    const float tx = 2 * (y * v[2] - z * v[1]);
    const float ty = 2 * (z * v[0] - x * v[2]);
    const float tz = 2 * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)};
}

/// a * b
std::array<float, 4> multiply(const std::array<float, 4>& a, const std::array<float, 4>& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

/// The head reports' tracking space has x up (2026-10-22, in the headset: looking straight ahead its up axis was world
/// (1.0, -0.01, 0.02); looking at the ceiling turned it +86 degrees about its own x, so the head axes themselves are
/// OpenXR's). The file's space is y up: a quarter turn about z, x -> y, y -> -x.
const std::array<float, 4> TO_Y_UP{0, 0, 0.70710678f, 0.70710678f};

void to_y_up(Device& d) {
    d.position = rotate(TO_Y_UP, d.position);
    d.orientation = multiply(TO_Y_UP, d.orientation);
    d.linear_velocity = rotate(TO_Y_UP, d.linear_velocity);
    d.angular_velocity = rotate(TO_Y_UP, d.angular_velocity);
}

bool write_all_at(int fd, const std::uint8_t* p, std::size_t n, off_t off) {
    while (n > 0) {
        const ssize_t w = ::pwrite(fd, p, n, off);
        if (w < 0 && errno == EINTR) {
            continue;
        }
        if (w <= 0) {
            return false;
        }
        p += w;
        n -= static_cast<std::size_t>(w);
        off += w;
    }
    return true;
}

std::int64_t monotonic_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
}

} // namespace

std::int64_t Clock::to_host(std::uint64_t headset_ns, std::int64_t received_ns) {
    constexpr std::int64_t WINDOW = 100'000'000; // ~67 head reports; at a 1.4 % rate difference it drifts < 3 ms
    const std::int64_t o = received_ns - static_cast<std::int64_t>(headset_ns);
    // more than 1 s off the current estimate: the headset clock restarted, start over
    if (window_min_ && (o > *window_min_ + 1'000'000'000 || o < *window_min_ - 1'000'000'000)) {
        window_min_.reset();
        previous_min_.reset();
    }
    if (!window_min_ || received_ns - window_start_ >= WINDOW) {
        previous_min_ = window_min_;
        window_min_ = o;
        window_start_ = received_ns;
    } else if (o < *window_min_) {
        window_min_ = o;
    }
    const std::int64_t offset = previous_min_ ? std::min(*window_min_, *previous_min_) : *window_min_;
    return static_cast<std::int64_t>(headset_ns) + offset;
}

Device from_head(const p3b::Head& h, std::int64_t t_ns, std::uint64_t sample) {
    Device d;
    d.t_ns = t_ns;
    d.state = h.tracked ? 1 : 0;
    d.battery = h.battery;
    d.position = h.pose.position;
    d.orientation = h.pose.orientation;
    d.linear_velocity = h.linear_velocity;
    d.angular_velocity = rotate(h.pose.orientation, h.angular_velocity);
    d.buttons = static_cast<std::uint32_t>(h.buttons);
    d.sample = sample;
    to_y_up(d);
    return d;
}

Device from_controller(const p3b::Controller& c, std::int64_t t_ns, std::uint64_t sample) {
    Device d;
    d.t_ns = t_ns;
    d.state = static_cast<std::uint32_t>(c.state);
    d.battery = c.battery;
    d.position = c.pose.position;
    d.orientation = c.pose.orientation;
    d.buttons = static_cast<std::uint32_t>(c.buttons) | static_cast<std::uint32_t>(c.touches) << 8;
    d.axes = {c.trigger, c.grip, c.stick[0], c.stick[1]};
    d.sample = sample;
    // the controllers come y up already (2026-10-22, the right one held out ahead at eye level: its up axis world
    // (-0.21, 0.95, 0.24), its -z along the head's forward to cos 0.995); only the head's space is x up
    return d;
}

std::array<std::uint8_t, SIZE - 16> body(const State& s) {
    // the whole file indexed by its offsets, then bytes 8..320 of it
    std::array<std::uint8_t, SIZE> f{};
    std::uint8_t* b = f.data();
    put(b, 8, MAGIC);
    put(b, 12, VERSION);
    put(b, 16, s.written_ns);
    put(b, 24, s.tracking);
    put(b, 28, s.source);
    for (std::size_t i = 0; i < 3; i++) {
        const Device& d = s.devices[i];
        const std::size_t o = 32 + i * DEVICE_SIZE;
        put(b, o + 0, d.t_ns);
        put(b, o + 8, d.state);
        put(b, o + 12, d.battery);
        for (std::size_t k = 0; k < 3; k++) {
            put(b, o + 16 + 4 * k, d.position[k]);
            put(b, o + 44 + 4 * k, d.linear_velocity[k]);
            put(b, o + 56 + 4 * k, d.angular_velocity[k]);
        }
        for (std::size_t k = 0; k < 4; k++) {
            put(b, o + 28 + 4 * k, d.orientation[k]);
            put(b, o + 72 + 4 * k, d.axes[k]);
        }
        put(b, o + 68, d.buttons);
        put(b, o + 88, d.sample);
    }
    std::array<std::uint8_t, SIZE - 16> a{};
    std::copy(f.begin() + 8, f.begin() + 320, a.begin());
    return a;
}

std::optional<State> decode(std::span<const std::uint8_t> f) {
    if (f.size() < SIZE) {
        return std::nullopt;
    }
    const auto begin = at<std::uint64_t>(f, 0), end = at<std::uint64_t>(f, 320);
    if (begin != end || begin == 0 || (begin & 1) || at<std::uint32_t>(f, 8) != MAGIC ||
        at<std::uint32_t>(f, 12) != VERSION) {
        return std::nullopt;
    }
    State s;
    s.written_ns = at<std::int64_t>(f, 16);
    s.tracking = at<std::uint32_t>(f, 24);
    s.source = at<std::uint32_t>(f, 28);
    for (std::size_t i = 0; i < 3; i++) {
        Device& d = s.devices[i];
        const std::size_t o = 32 + i * DEVICE_SIZE;
        d.t_ns = at<std::int64_t>(f, o);
        d.state = at<std::uint32_t>(f, o + 8);
        d.battery = at<std::int32_t>(f, o + 12);
        for (std::size_t k = 0; k < 3; k++) {
            d.position[k] = at<float>(f, o + 16 + 4 * k);
            d.linear_velocity[k] = at<float>(f, o + 44 + 4 * k);
            d.angular_velocity[k] = at<float>(f, o + 56 + 4 * k);
        }
        for (std::size_t k = 0; k < 4; k++) {
            d.orientation[k] = at<float>(f, o + 28 + 4 * k);
            d.axes[k] = at<float>(f, o + 72 + 4 * k);
        }
        d.buttons = at<std::uint32_t>(f, o + 68);
        d.sample = at<std::uint64_t>(f, o + 88);
    }
    return s;
}

std::optional<Publisher> Publisher::create(const std::filesystem::path& path) {
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) {
        return std::nullopt;
    }
    if (::ftruncate(fd, static_cast<off_t>(SIZE)) != 0) {
        ::close(fd);
        return std::nullopt;
    }
    // continue after what is there, so a reader never sees the same sequence number for other data
    std::uint64_t old = 0;
    if (::pread(fd, &old, sizeof old, 320) != static_cast<ssize_t>(sizeof old)) {
        old = 0;
    }
    return Publisher(fd, (old + 2) & ~std::uint64_t{1});
}

Publisher::Publisher(Publisher&& o) noexcept : fd_(o.fd_), seq_(o.seq_), state_(o.state_), clock_(o.clock_) {
    o.fd_ = -1;
}

Publisher& Publisher::operator=(Publisher&& o) noexcept {
    if (this != &o) {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = o.fd_;
        seq_ = o.seq_;
        state_ = o.state_;
        clock_ = o.clock_;
        o.fd_ = -1;
    }
    return *this;
}

Publisher::~Publisher() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

void Publisher::publish(const State& s, std::int64_t now_ns) {
    state_ = s;
    write(now_ns);
}

void Publisher::head(const p3b::Head& h, std::int64_t received_ns) {
    state_.source = SOURCE_CRYSTAL;
    Device& d = state_.devices[0];
    d = from_head(h, clock_.to_host(h.pose.t_ns, received_ns), d.sample + 1);
    state_.tracking = static_cast<std::uint32_t>(h.tracking);
    write(received_ns);
}

void Publisher::controller(const p3b::Controller& c, std::int64_t received_ns) {
    Device& d = state_.devices[c.side == 0 ? 1 : 2];
    d = from_controller(c, clock_.to_host(c.pose.t_ns, received_ns), d.sample + 1);
    write(received_ns);
}

void Publisher::gone(std::int64_t now_ns) {
    for (Device& d : state_.devices) {
        d.state = 0;
    }
    write(now_ns);
}

void Publisher::write(std::int64_t now_ns) {
    state_.written_ns = now_ns != 0 ? now_ns : monotonic_ns();
    const auto b = body(state_);
    std::uint64_t w = seq_ + 1;
    if (!write_all_at(fd_, reinterpret_cast<const std::uint8_t*>(&w), 8, 0) ||
        !write_all_at(fd_, b.data(), b.size(), 8)) {
        return;
    }
    seq_ += 2;
    w = seq_;
    if (write_all_at(fd_, reinterpret_cast<const std::uint8_t*>(&w), 8, 320)) {
        write_all_at(fd_, reinterpret_cast<const std::uint8_t*>(&w), 8, 0);
    }
}

} // namespace pibridge::hmd::pose_shm
