// SPDX-License-Identifier: MIT
// Helpers shared by the libpibridge-cam tests (twins of the Rust test helpers in pimax-cam).
#pragma once

#include "pibridge/cam/frame.h"
#include "pibridge/cam/source.h"

#include <cstdint>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <vector>

namespace pibridge::cam::testing {

inline std::vector<std::uint8_t> hex(const std::string& s) {
    std::string t;
    for (const char c : s) {
        if (c != ' ' && c != '\n') {
            t += c;
        }
    }
    std::vector<std::uint8_t> v;
    for (std::size_t i = 0; i + 1 < t.size(); i += 2) {
        v.push_back(static_cast<std::uint8_t>(std::stoul(t.substr(i, 2), nullptr, 16)));
    }
    return v;
}

/// A fresh path in $TMPDIR (ctest sets it to the build directory).
inline std::string tmp(const std::string& name) {
    const char* d = std::getenv("TMPDIR");
    std::string p = std::string(d != nullptr && *d != 0 ? d : "/tmp") + "/pibridge-cam-test-" +
                    std::to_string(::getpid()) + "-" + name;
    ::unlink(p.c_str());
    return p;
}

inline void put_le(std::uint8_t* p, std::uint64_t v, int n) {
    for (int i = 0; i < n; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

/// frame.rs synth_frame: metadata row block plus four tiles with per-tile offsets and a pattern.
inline std::vector<std::uint8_t> synth_frame(std::array<std::uint16_t, 4> offsets, bool kind_bit, std::uint32_t counter,
                                             std::uint64_t ts_us) {
    std::vector<std::uint8_t> f(FRAME_BYTES, 0);
    std::uint8_t meta[META_LEN] = {};
    meta[32] = kind_bit ? 0x86 : 0x06;
    put_le(meta + 33, ts_us, 8);
    put_le(meta + 41, counter, 4);
    for (std::size_t i = 0; i < 4; i++) {
        put_le(meta + 45 + 2 * i, 1000 + i, 2);
        put_le(meta + 53 + 2 * i, 10 + i, 2);
        put_le(meta + 61 + 2 * i, offsets[i], 2);
    }
    for (std::size_t i = 0; i < 4; i++) {
        meta[92 + i] = META_MAGIC[i];
    }
    for (std::size_t r = 0; r < META_ROWS; r++) {
        std::copy(meta, meta + META_LEN, f.begin() + static_cast<std::ptrdiff_t>(r * FRAME_W));
    }
    for (std::size_t t = 0; t < 4; t++) {
        for (std::size_t y = 0; y < TILE_H; y++) {
            for (std::size_t x = 0; x < TILE_W; x++) {
                f[(offsets[t] + y) * FRAME_W + t * TILE_W + x] =
                    static_cast<std::uint8_t>(1 + ((t * 61 + y * 7 + x * 3) % 250));
            }
        }
    }
    return f;
}

/// camera.rs synth: one row offset for all tiles; tile t, row y holds 1 + 50 t + y % 40.
inline std::vector<std::uint8_t> synth(bool kind_bit, std::uint32_t counter, std::uint64_t ts_us, std::uint16_t off) {
    std::vector<std::uint8_t> f(FRAME_BYTES, 0);
    std::uint8_t meta[META_LEN] = {};
    meta[32] = kind_bit ? 0x86 : 0x06;
    put_le(meta + 33, ts_us, 8);
    put_le(meta + 41, counter, 4);
    for (std::size_t i = 0; i < 4; i++) {
        put_le(meta + 61 + 2 * i, off, 2);
    }
    for (std::size_t i = 0; i < 4; i++) {
        meta[92 + i] = META_MAGIC[i];
    }
    for (std::size_t r = 0; r < META_ROWS; r++) {
        std::copy(meta, meta + META_LEN, f.begin() + static_cast<std::ptrdiff_t>(r * FRAME_W));
    }
    for (std::size_t y = 0; y < TILE_H; y++) {
        for (std::size_t x = 0; x < FRAME_W; x++) {
            f[(off + y) * FRAME_W + x] = static_cast<std::uint8_t>(1 + (x / TILE_W) * 50 + y % 40);
        }
    }
    return f;
}

/// camera.rs VecSource: frames from memory.
class VecSource final : public Source {
public:
    std::vector<std::pair<std::vector<std::uint8_t>, std::uint64_t>> frames;
    std::size_t pos = 0;
    std::optional<std::size_t> cur;

    Result<bool> advance(int) override {
        if (pos >= frames.size()) {
            return false;
        }
        cur = pos++;
        return true;
    }
    std::optional<RawFrame> current() const override {
        if (!cur) {
            return std::nullopt;
        }
        return RawFrame{frames[*cur].first, FRAME_W, frames[*cur].second, static_cast<std::uint32_t>(*cur)};
    }
    std::string describe() const override { return "vec"; }
};

} // namespace pibridge::cam::testing
