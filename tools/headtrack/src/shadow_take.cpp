// SPDX-License-Identifier: MIT
#include "shadow_take.h"

#include <zlib.h>

#include <algorithm>
#include <cstring>

namespace pibridge::headtrack {

namespace {

constexpr std::size_t HDR_LEN = 32;
constexpr std::size_t REC_LEN = 32;

void put16(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}
void put32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}
void put64(std::uint8_t* p, std::uint64_t v) {
    for (int i = 0; i < 8; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}
std::uint16_t rd16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
std::uint32_t rd32(const std::uint8_t* p) {
    std::uint32_t v = 0;
    for (int i = 3; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    return v;
}
std::uint64_t rd64(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    return v;
}

} // namespace

std::unique_ptr<ShadowTakeWriter> ShadowTakeWriter::start(const std::string& path, std::uint32_t tiles, std::uint32_t w,
                                                          std::uint32_t h, bool zlib, std::string* error,
                                                          std::size_t depth) {
    if (tiles == 0 || w == 0 || h == 0 || tiles > 0xffff || w > 0xffff || h > 0xffff) {
        if (error != nullptr) {
            *error = "bad take geometry";
        }
        return nullptr;
    }
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        if (error != nullptr) {
            *error = path + ": " + std::strerror(errno);
        }
        return nullptr;
    }
    std::uint8_t hdr[HDR_LEN] = {};
    std::memcpy(hdr, SHADOW_TAKE_MAGIC, 8);
    put16(hdr + 8, 1);
    put16(hdr + 10, static_cast<std::uint16_t>(tiles));
    put16(hdr + 12, static_cast<std::uint16_t>(w));
    put16(hdr + 14, static_cast<std::uint16_t>(h));
    put32(hdr + 16, zlib ? SHADOW_TAKE_ZLIB : 0);
    if (std::fwrite(hdr, 1, HDR_LEN, f) != HDR_LEN) {
        if (error != nullptr) {
            *error = path + ": " + std::strerror(errno);
        }
        std::fclose(f);
        return nullptr;
    }
    std::unique_ptr<ShadowTakeWriter> wr(new ShadowTakeWriter());
    wr->path_ = path;
    wr->f_ = f;
    wr->tiles_ = tiles;
    wr->w_ = w;
    wr->h_ = h;
    wr->zlib_ = zlib;
    wr->depth_ = std::max<std::size_t>(depth, 1);
    wr->bytes_ = HDR_LEN;
    wr->thread_ = std::thread([p = wr.get()] { p->run(); });
    return wr;
}

ShadowTakeWriter::~ShadowTakeWriter() {
    {
        std::lock_guard g(mu_);
        closed_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    if (f_ != nullptr) {
        std::fclose(f_);
    }
}

void ShadowTakeWriter::push(std::int64_t ts_ns, std::int64_t arrival_ns, const std::vector<const std::uint8_t*>& tiles) {
    const std::size_t tile = static_cast<std::size_t>(w_) * h_;
    std::vector<std::uint8_t> buf;
    {
        std::lock_guard g(mu_);
        queued_++;
        if (queue_.size() >= depth_) {
            dropped_++;
            return;
        }
        if (!pool_.empty()) {
            buf = std::move(pool_.back());
            pool_.pop_back();
        }
    }
    buf.resize(tile * tiles_);
    for (std::uint32_t t = 0; t < tiles_; t++) {
        if (t < tiles.size() && tiles[t] != nullptr) {
            std::memcpy(buf.data() + tile * t, tiles[t], tile);
        } else {
            std::memset(buf.data() + tile * t, 0, tile);
        }
    }
    {
        std::lock_guard g(mu_);
        queue_.push_back(Item{ts_ns, arrival_ns, std::move(buf)});
    }
    cv_.notify_one();
}

void ShadowTakeWriter::run() {
    for (;;) {
        Item it;
        {
            std::unique_lock l(mu_);
            cv_.wait(l, [&] { return closed_ || !queue_.empty(); });
            if (queue_.empty()) {
                return; // closed and drained
            }
            it = std::move(queue_.front());
            queue_.pop_front();
        }
        const std::size_t raw = it.data.size();
        const std::uint8_t* payload = it.data.data();
        std::size_t stored = raw;
        std::uint32_t flags = 0;
        if (zlib_) {
            uLongf dlen = compressBound(static_cast<uLong>(raw));
            zbuf_.resize(dlen);
            if (compress2(zbuf_.data(), &dlen, it.data.data(), static_cast<uLong>(raw), 1) == Z_OK && dlen < raw) {
                payload = zbuf_.data();
                stored = dlen;
                flags = SHADOW_TAKE_ZLIB;
            }
        }
        std::uint8_t hdr[REC_LEN] = {};
        put32(hdr, SHADOW_TAKE_REC_MAGIC);
        put32(hdr + 4, flags);
        put32(hdr + 8, static_cast<std::uint32_t>(stored));
        put32(hdr + 12, static_cast<std::uint32_t>(raw));
        put64(hdr + 16, static_cast<std::uint64_t>(it.ts));
        put64(hdr + 24, static_cast<std::uint64_t>(it.arrival));
        bool ok = f_ != nullptr && std::fwrite(hdr, 1, REC_LEN, f_) == REC_LEN &&
                  std::fwrite(payload, 1, stored, f_) == stored;
        std::lock_guard g(mu_);
        if (ok) {
            written_++;
            bytes_ += REC_LEN + stored;
        }
        if (pool_.size() < depth_) {
            pool_.push_back(std::move(it.data));
        }
    }
}

std::uint64_t ShadowTakeWriter::queued() const {
    std::lock_guard g(mu_);
    return queued_;
}
std::uint64_t ShadowTakeWriter::dropped() const {
    std::lock_guard g(mu_);
    return dropped_;
}
std::uint64_t ShadowTakeWriter::written() const {
    std::lock_guard g(mu_);
    return written_;
}
std::uint64_t ShadowTakeWriter::bytes() const {
    std::lock_guard g(mu_);
    return bytes_;
}

std::unique_ptr<ShadowTakeReader> ShadowTakeReader::open(const std::string& path, std::string* error) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) {
        if (error != nullptr) {
            *error = path + ": " + std::strerror(errno);
        }
        return nullptr;
    }
    std::uint8_t hdr[HDR_LEN];
    if (std::fread(hdr, 1, HDR_LEN, f) != HDR_LEN || std::memcmp(hdr, SHADOW_TAKE_MAGIC, 8) != 0 || rd16(hdr + 8) != 1) {
        if (error != nullptr) {
            *error = path + ": not a head shadow take";
        }
        std::fclose(f);
        return nullptr;
    }
    std::unique_ptr<ShadowTakeReader> r(new ShadowTakeReader());
    r->f_ = f;
    r->tiles = rd16(hdr + 10);
    r->w = rd16(hdr + 12);
    r->h = rd16(hdr + 14);
    r->flags = rd32(hdr + 16);
    if (r->tiles == 0 || r->w == 0 || r->h == 0) {
        if (error != nullptr) {
            *error = path + ": bad take geometry";
        }
        return nullptr;
    }
    return r;
}

ShadowTakeReader::~ShadowTakeReader() {
    if (f_ != nullptr) {
        std::fclose(f_);
    }
}

bool ShadowTakeReader::rewind() {
    truncated = false;
    return f_ != nullptr && std::fseek(f_, static_cast<long>(HDR_LEN), SEEK_SET) == 0;
}

bool ShadowTakeReader::next(ShadowTakeFrame& fr, std::string* error) {
    std::uint8_t hdr[REC_LEN];
    const std::size_t got = std::fread(hdr, 1, REC_LEN, f_);
    if (got == 0) {
        return false;
    }
    if (got != REC_LEN) {
        truncated = true;
        return false;
    }
    if (rd32(hdr) != SHADOW_TAKE_REC_MAGIC) {
        if (error != nullptr) {
            *error = "bad record magic";
        }
        return false;
    }
    const std::uint32_t flags = rd32(hdr + 4);
    const std::size_t stored = rd32(hdr + 8);
    const std::size_t raw = rd32(hdr + 12);
    const std::size_t expect = static_cast<std::size_t>(tiles) * w * h;
    if (raw != expect || stored > raw + raw / 2 + 1024) {
        if (error != nullptr) {
            *error = "bad record lengths";
        }
        return false;
    }
    fr.ts_ns = static_cast<std::int64_t>(rd64(hdr + 16));
    fr.arrival_ns = static_cast<std::int64_t>(rd64(hdr + 24));
    if ((flags & SHADOW_TAKE_ZLIB) != 0) {
        stored_.resize(stored);
        if (std::fread(stored_.data(), 1, stored, f_) != stored) {
            truncated = true;
            return false;
        }
        fr.data.resize(raw);
        uLongf out = static_cast<uLongf>(raw);
        if (uncompress(fr.data.data(), &out, stored_.data(), static_cast<uLong>(stored)) != Z_OK || out != raw) {
            if (error != nullptr) {
                *error = "bad zlib payload";
            }
            return false;
        }
    } else {
        fr.data.resize(raw);
        if (std::fread(fr.data.data(), 1, raw, f_) != raw) {
            truncated = true;
            return false;
        }
    }
    return true;
}

} // namespace pibridge::headtrack
