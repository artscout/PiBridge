// SPDX-License-Identifier: MIT
#include "gazeshm.h"

#include <array>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>

namespace pibridge::app {

namespace {

template <typename T>
T at(std::span<const uint8_t> b, size_t off) {
    T v;
    std::memcpy(&v, b.data() + off, sizeof v); // the file is little endian, as is every machine this runs on
    return v;
}

bool preadAll(int fd, uint8_t* dst, size_t n, off_t off) {
    while (n > 0) {
        const ssize_t r = ::pread(fd, dst, n, off);
        if (r <= 0) {
            return false;
        }
        dst += r;
        n -= static_cast<size_t>(r);
        off += r;
    }
    return true;
}

} // namespace

std::optional<GazeSnapshot> decodeGaze(std::span<const uint8_t> b) {
    if (b.size() < kGazeSize) {
        return std::nullopt;
    }
    const auto seqBegin = at<uint64_t>(b, 0), seqEnd = at<uint64_t>(b, 128);
    if (seqBegin != seqEnd || seqBegin % 2 != 0 || seqBegin == 0 || at<uint32_t>(b, 8) != kGazeMagic ||
        at<uint32_t>(b, 12) != kGazeVersion) {
        return std::nullopt;
    }
    GazeSnapshot g;
    g.tNs = at<int64_t>(b, 16);
    g.writtenNs = at<int64_t>(b, 24);
    g.flags = at<uint32_t>(b, 32);
    g.ipdMm = at<float>(b, 36);
    for (size_t e = 0; e < 3; e++) {
        for (size_t i = 0; i < 3; i++) {
            g.origin[e][i] = at<float>(b, 40 + e * 24 + i * 4);
            g.direction[e][i] = at<float>(b, 52 + e * 24 + i * 4);
        }
    }
    g.sample = at<uint64_t>(b, 112);
    g.state = at<uint32_t>(b, 120);
    return g;
}

std::optional<GazeSnapshot> readGaze(const char* path) {
    const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return std::nullopt;
    }
    std::optional<GazeSnapshot> out;
    for (int attempt = 0; attempt < 8 && !out; attempt++) {
        std::array<uint8_t, kGazeSize> f{};
        if (!preadAll(fd, f.data() + 128, 8, 128) || !preadAll(fd, f.data() + 8, 120, 8) ||
            !preadAll(fd, f.data(), 8, 0)) {
            break;
        }
        out = decodeGaze(f);
        if (!out) {
            sched_yield();
        }
    }
    ::close(fd);
    return out;
}

bool gazeTangent(const GazeSnapshot& g, float& tx, float& ty) {
    float d[3]{};
    if (g.flags & kGazeCombined) {
        std::memcpy(d, g.direction[2], sizeof d);
    } else {
        int n = 0;
        for (int e = 0; e < 2; e++) {
            if (g.flags & (e == 0 ? kGazeLeft : kGazeRight)) {
                for (int i = 0; i < 3; i++) {
                    d[i] += g.direction[e][i];
                }
                n++;
            }
        }
        if (n == 0) {
            return false;
        }
    }
    if (!(d[2] < -1e-4f)) {
        return false;
    }
    tx = d[0] / -d[2];
    ty = d[1] / -d[2];
    return std::isfinite(tx) && std::isfinite(ty);
}

int64_t monotonicNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

} // namespace pibridge::app
