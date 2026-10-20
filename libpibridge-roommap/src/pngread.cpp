// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/pngread.rs (MIT OR Apache-2.0). See pngread.h.
#include "pibridge/roommap/pngread.h"

#include "pibridge/cam/io.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <zlib.h>

namespace pibridge::roommap::pngread {

namespace {

std::uint32_t be32(const std::uint8_t* p) {
    return (std::uint32_t{p[0]} << 24) | (std::uint32_t{p[1]} << 16) | (std::uint32_t{p[2]} << 8) | p[3];
}

std::uint8_t paeth(std::uint8_t a8, std::uint8_t b8, std::uint8_t c8) {
    const std::int16_t a = a8, b = b8, c = c8;
    const std::int16_t p = static_cast<std::int16_t>(a + b - c);
    const std::int16_t pa = static_cast<std::int16_t>(std::abs(p - a));
    const std::int16_t pb = static_cast<std::int16_t>(std::abs(p - b));
    const std::int16_t pc = static_cast<std::int16_t>(std::abs(p - c));
    if (pa <= pb && pa <= pc) {
        return a8;
    }
    if (pb <= pc) {
        return b8;
    }
    return c8;
}

/// zlib stream -> bytes; false on a corrupt or truncated stream.
bool inflate_all(const std::vector<std::uint8_t>& in, std::vector<std::uint8_t>& out, std::size_t hint) {
    z_stream zs{};
    if (inflateInit(&zs) != Z_OK) {
        return false;
    }
    out.assign(std::max<std::size_t>(hint, 1024), 0);
    zs.next_in = const_cast<Bytef*>(in.data());
    zs.avail_in = static_cast<uInt>(in.size());
    std::size_t have = 0;
    int r = Z_OK;
    for (;;) {
        if (have == out.size()) {
            out.resize(out.size() * 2);
        }
        zs.next_out = out.data() + have;
        zs.avail_out = static_cast<uInt>(out.size() - have);
        r = inflate(&zs, Z_NO_FLUSH);
        have = out.size() - zs.avail_out;
        if (r == Z_STREAM_END) {
            break;
        }
        if (r != Z_OK && !(r == Z_BUF_ERROR && zs.avail_out == 0)) {
            inflateEnd(&zs);
            return false;
        }
        if (zs.avail_in == 0 && zs.avail_out != 0) {
            // input ended before the stream did
            inflateEnd(&zs);
            return false;
        }
    }
    inflateEnd(&zs);
    out.resize(have);
    return true;
}

} // namespace

std::expected<Grey, std::string> decode(std::span<const std::uint8_t> b) {
    static const std::uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    if (b.size() < 8 || std::memcmp(b.data(), sig, 8) != 0) {
        return std::unexpected("not a PNG");
    }
    std::size_t i = 8;
    std::vector<std::uint8_t> idat;
    std::size_t w = 0, h = 0;
    std::uint8_t depth = 0, ctype = 0, interlace = 0;
    while (i + 8 <= b.size()) {
        const std::size_t len = be32(b.data() + i);
        const std::uint8_t* kind = b.data() + i + 4;
        if (i + 8 + len > b.size()) {
            return std::unexpected("truncated PNG chunk");
        }
        const std::uint8_t* body = b.data() + i + 8;
        if (std::memcmp(kind, "IHDR", 4) == 0) {
            if (len < 13) {
                // the Rust slice indexing panics here; a short IHDR is a broken file either way
                return std::unexpected("truncated PNG chunk");
            }
            w = be32(body);
            h = be32(body + 4);
            depth = body[8];
            ctype = body[9];
            interlace = body[12];
        } else if (std::memcmp(kind, "IDAT", 4) == 0) {
            idat.insert(idat.end(), body, body + len);
        } else if (std::memcmp(kind, "IEND", 4) == 0) {
            break;
        }
        i += 12 + len;
    }
    if (depth != 8 || interlace != 0 || !(ctype == 0 || ctype == 2 || ctype == 6) || w == 0 || h == 0) {
        return std::unexpected("unsupported PNG (depth " + std::to_string(depth) + ", colour type " +
                               std::to_string(ctype) + ", interlace " + std::to_string(interlace) + ")");
    }
    const std::size_t bpp = ctype == 0 ? 1 : (ctype == 2 ? 3 : 4);
    std::vector<std::uint8_t> raw;
    const std::size_t stride = w * bpp;
    if (!inflate_all(idat, raw, h * (stride + 1))) {
        return std::unexpected("PNG inflate: corrupt or truncated zlib stream");
    }
    if (raw.size() < h * (stride + 1)) {
        return std::unexpected("PNG data too short");
    }
    std::vector<std::uint8_t> prev(stride, 0);
    std::vector<std::uint8_t> cur(stride, 0);
    Grey g;
    g.w = w;
    g.h = h;
    g.data.reserve(w * h);
    for (std::size_t y = 0; y < h; y++) {
        const std::uint8_t* row = raw.data() + y * (stride + 1);
        const std::uint8_t f = row[0];
        if (f > 4) {
            return std::unexpected("bad PNG filter " + std::to_string(f));
        }
        for (std::size_t x = 0; x < stride; x++) {
            const std::uint8_t a = x >= bpp ? cur[x - bpp] : 0;
            const std::uint8_t up = prev[x];
            const std::uint8_t c = x >= bpp ? prev[x - bpp] : 0;
            const std::uint8_t v = row[1 + x];
            switch (f) {
            case 0: cur[x] = v; break;
            case 1: cur[x] = static_cast<std::uint8_t>(v + a); break;
            case 2: cur[x] = static_cast<std::uint8_t>(v + up); break;
            case 3: cur[x] = static_cast<std::uint8_t>(v + static_cast<std::uint8_t>((std::uint16_t{a} + up) / 2)); break;
            default: cur[x] = static_cast<std::uint8_t>(v + paeth(a, up, c)); break;
            }
        }
        if (bpp == 1) {
            g.data.insert(g.data.end(), cur.begin(), cur.end());
        } else {
            for (std::size_t x = 0; x < w; x++) {
                const std::uint8_t* p = cur.data() + x * bpp;
                g.data.push_back(static_cast<std::uint8_t>(
                    (std::uint16_t{p[0]} * 77 + std::uint16_t{p[1]} * 150 + std::uint16_t{p[2]} * 29) >> 8));
            }
        }
        std::swap(prev, cur);
    }
    return g;
}

std::expected<Grey, std::string> read(const std::string& path) {
    auto b = cam::read_file(path);
    if (!b) {
        return std::unexpected(path + ": " + b.error().text());
    }
    auto g = decode(std::span(reinterpret_cast<const std::uint8_t*>(b->data()), b->size()));
    if (!g) {
        return std::unexpected(path + ": " + g.error());
    }
    return g;
}

} // namespace pibridge::roommap::pngread
