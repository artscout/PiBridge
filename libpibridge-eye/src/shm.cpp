// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/shm.rs (MIT OR Apache-2.0).
#include "pibridge/eye/shm.h"

#include "bytes.h"

#include <bit>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::eye::shm {

std::filesystem::path default_path() {
    return "/dev/shm/omarchyvr-eye-gaze";
}

namespace {

void put_f32(std::uint8_t* p, float v) {
    detail::store_le32(p, std::bit_cast<std::uint32_t>(v));
}

float f32_at(std::span<const std::uint8_t> f, std::size_t o) {
    return std::bit_cast<float>(detail::le32(f, o));
}

Result<void> write_all_at(int fd, const std::uint8_t* p, std::size_t n, off_t off) {
    while (n > 0) {
        const ssize_t w = ::pwrite(fd, p, n, off);
        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }
            return std::unexpected(Error::os(errno, "gaze shared memory write"));
        }
        if (w == 0) {
            return std::unexpected(Error::make(Errc::short_transfer, "gaze shared memory write returned 0"));
        }
        p += w;
        n -= static_cast<std::size_t>(w);
        off += w;
    }
    return {};
}

bool read_exact_at(int fd, std::uint8_t* p, std::size_t n, off_t off) {
    while (n > 0) {
        const ssize_t r = ::pread(fd, p, n, off);
        if (r < 0 && errno == EINTR) {
            continue;
        }
        if (r <= 0) {
            return false;
        }
        p += r;
        n -= static_cast<std::size_t>(r);
        off += r;
    }
    return true;
}

} // namespace

std::array<std::uint8_t, 120> body(const Sample* s, std::uint32_t state, std::int64_t written_ns) {
    std::array<std::uint8_t, 120> b{};
    // offsets below are file offsets; the body starts at file offset 8
    const auto at = [&](std::size_t off) { return b.data() + (off - 8); };
    detail::store_le32(at(8), MAGIC);
    detail::store_le32(at(12), VERSION);
    detail::store_le64(at(24), static_cast<std::uint64_t>(written_ns));
    detail::store_le32(at(120), state);
    if (s == nullptr) {
        return b;
    }
    detail::store_le64(at(16), static_cast<std::uint64_t>(s->host_ts_ns));
    std::uint32_t flags = 0;
    const auto v3 = [&](std::size_t off, const Vec3& v) {
        for (std::size_t i = 0; i < 3; i++) {
            put_f32(at(off + i * 4), v[i]);
        }
    };
    const struct {
        std::size_t off;
        std::uint32_t dbit, obit;
    } eyes[2] = {{40, LEFT, LEFT_ORIGIN}, {64, RIGHT, RIGHT_ORIGIN}};
    for (std::size_t e = 0; e < 2; e++) {
        const EyeRay& r = s->eyes[e];
        if (r.origin_valid) {
            flags |= eyes[e].obit;
            v3(eyes[e].off, r.origin);
        }
        if (r.direction_valid) {
            flags |= eyes[e].dbit;
            v3(eyes[e].off + 12, r.direction);
        }
    }
    if (s->gaze) {
        flags |= COMBINED;
        v3(88, s->gaze->origin);
        v3(100, s->gaze->direction);
    }
    detail::store_le32(at(32), flags);
    put_f32(at(36), std::fmax(s->ipd_mm, 0.0f)); // NaN becomes 0, as f32::max in the Rust original
    detail::store_le64(at(112), s->seq);
    return b;
}

std::optional<Gaze> decode(std::span<const std::uint8_t> f) {
    if (f.size() < SIZE) {
        return std::nullopt;
    }
    const std::uint64_t b = detail::le64(f, 0);
    const std::uint64_t e = detail::le64(f, 128);
    if (b != e || b % 2 != 0 || b == 0 || detail::le32(f, 8) != MAGIC || detail::le32(f, 12) != VERSION) {
        return std::nullopt;
    }
    const auto v3 = [&](std::size_t o) { return Vec3{f32_at(f, o), f32_at(f, o + 4), f32_at(f, o + 8)}; };
    Gaze g;
    g.t_ns = static_cast<std::int64_t>(detail::le64(f, 16));
    g.written_ns = static_cast<std::int64_t>(detail::le64(f, 24));
    g.flags = detail::le32(f, 32);
    g.ipd_mm = f32_at(f, 36);
    g.origin = {v3(40), v3(64), v3(88)};
    g.direction = {v3(52), v3(76), v3(100)};
    g.sample = detail::le64(f, 112);
    g.state = detail::le32(f, 120);
    return g;
}

Result<Publisher> Publisher::create(const std::filesystem::path& path) {
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd < 0) {
        return std::unexpected(Error::os(errno, path.string()));
    }
    if (::ftruncate(fd, static_cast<off_t>(SIZE)) != 0) {
        const int e = errno;
        ::close(fd);
        return std::unexpected(Error::os(e, path.string()));
    }
    // continue after what is there, so a reader never sees the same sequence number for other data
    std::array<std::uint8_t, 8> old{};
    if (!read_exact_at(fd, old.data(), old.size(), 128)) {
        old.fill(0);
    }
    const std::uint64_t seq = (detail::le64(old, 0) + 2) & ~std::uint64_t{1};
    return Publisher(fd, seq);
}

Publisher::Publisher(Publisher&& o) noexcept : fd_(o.fd_), seq_(o.seq_) {
    o.fd_ = -1;
}

Publisher& Publisher::operator=(Publisher&& o) noexcept {
    if (this != &o) {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = o.fd_;
        seq_ = o.seq_;
        o.fd_ = -1;
    }
    return *this;
}

Publisher::~Publisher() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

Result<void> Publisher::publish(const Sample* sample, std::uint32_t state, std::int64_t written_ns) {
    const auto b = body(sample, state, written_ns);
    std::array<std::uint8_t, 8> w{};
    detail::store_le64(w.data(), seq_ + 1);
    if (auto r = write_all_at(fd_, w.data(), w.size(), 0); !r) {
        return r;
    }
    if (auto r = write_all_at(fd_, b.data(), b.size(), 8); !r) {
        return r;
    }
    seq_ += 2;
    detail::store_le64(w.data(), seq_);
    if (auto r = write_all_at(fd_, w.data(), w.size(), 128); !r) {
        return r;
    }
    return write_all_at(fd_, w.data(), w.size(), 0);
}

std::optional<Gaze> read(const std::filesystem::path& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return std::nullopt;
    }
    std::optional<Gaze> out;
    for (int i = 0; i < 8 && !out; i++) {
        std::array<std::uint8_t, SIZE> f{};
        if (!read_exact_at(fd, f.data() + 128, 8, 128) || !read_exact_at(fd, f.data() + 8, 120, 8) ||
            !read_exact_at(fd, f.data(), 8, 0)) {
            break;
        }
        out = decode(f);
        if (!out) {
            std::this_thread::yield();
        }
    }
    ::close(fd);
    return out;
}

} // namespace pibridge::eye::shm
