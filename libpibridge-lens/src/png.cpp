// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/png.rs (MIT OR Apache-2.0).
#include "pibridge/lens/png.h"

#include "pibridge/cam/io.h"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <format>
#include <stdexcept>
#include <unistd.h>
#include <zlib.h>

namespace pibridge::lens::png {

namespace {

constexpr std::array<std::uint8_t, 8> SIGNATURE = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};

const std::array<std::uint32_t, 256>& crc_table() {
    static const std::array<std::uint32_t, 256> t = [] {
        std::array<std::uint32_t, 256> x{};
        for (std::uint32_t n = 0; n < 256; n++) {
            std::uint32_t c = n;
            for (int k = 0; k < 8; k++) {
                c = (c & 1) != 0 ? 0xedb88320u ^ (c >> 1) : c >> 1;
            }
            x[n] = c;
        }
        return x;
    }();
    return t;
}

void put_be(std::vector<std::uint8_t>& out, std::uint32_t v) {
    out.push_back(static_cast<std::uint8_t>(v >> 24));
    out.push_back(static_cast<std::uint8_t>(v >> 16));
    out.push_back(static_cast<std::uint8_t>(v >> 8));
    out.push_back(static_cast<std::uint8_t>(v));
}

std::uint32_t get_be(const std::uint8_t* p) {
    return (std::uint32_t{p[0]} << 24) | (std::uint32_t{p[1]} << 16) | (std::uint32_t{p[2]} << 8) | p[3];
}

void chunk(std::vector<std::uint8_t>& out, const char* kind, std::span<const std::uint8_t> data) {
    put_be(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t start = out.size();
    out.insert(out.end(), kind, kind + 4);
    out.insert(out.end(), data.begin(), data.end());
    const std::uint32_t crc = crc32(std::span(out).subspan(start));
    put_be(out, crc);
}

std::uint8_t color_type(std::size_t channels) {
    switch (channels) {
    case 1:
        return 0;
    case 3:
        return 2;
    case 4:
        return 6;
    default:
        throw std::invalid_argument(std::format("unsupported channel count {}", channels));
    }
}

} // namespace

std::uint32_t crc32(std::span<const std::uint8_t> data) {
    const auto& t = crc_table();
    std::uint32_t c = 0xffffffffu;
    for (const std::uint8_t b : data) {
        c = t[(c ^ b) & 0xff] ^ (c >> 8);
    }
    return c ^ 0xffffffffu;
}

std::vector<std::uint8_t> encode(std::uint32_t width, std::uint32_t height, std::size_t channels,
                                 std::span<const std::uint8_t> data) {
    const std::size_t w = width;
    const std::size_t h = height;
    if (data.size() != w * h * channels) {
        throw std::invalid_argument("pixel buffer size");
    }
    const std::uint8_t ctype = color_type(channels);
    const std::size_t stride = w * channels;
    std::vector<std::uint8_t> raw;
    raw.reserve((stride + 1) * h);
    const std::vector<std::uint8_t> zero(stride, 0);
    std::array<std::vector<std::uint8_t>, 3> cand = {std::vector<std::uint8_t>(stride),
                                                     std::vector<std::uint8_t>(stride),
                                                     std::vector<std::uint8_t>(stride)};
    for (std::size_t y = 0; y < h; y++) {
        const std::uint8_t* row = data.data() + y * stride;
        const std::uint8_t* prev = y > 0 ? data.data() + (y - 1) * stride : zero.data();
        for (std::size_t i = 0; i < stride; i++) {
            const std::uint8_t left = i >= channels ? row[i - channels] : 0;
            cand[0][i] = row[i];
            cand[1][i] = static_cast<std::uint8_t>(row[i] - left);
            cand[2][i] = static_cast<std::uint8_t>(row[i] - prev[i]);
        }
        // min_by_key: the first of equal minima
        std::size_t best = 0;
        std::uint64_t best_sum = 0;
        for (std::size_t f = 0; f < 3; f++) {
            std::uint64_t s = 0;
            for (const std::uint8_t b : cand[f]) {
                const int v = static_cast<std::int8_t>(b);
                s += static_cast<std::uint64_t>(v < 0 ? -v : v);
            }
            if (f == 0 || s < best_sum) {
                best = f;
                best_sum = s;
            }
        }
        raw.push_back(static_cast<std::uint8_t>(best));
        raw.insert(raw.end(), cand[best].begin(), cand[best].end());
    }
    uLongf zlen = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> z(zlen);
    if (compress2(z.data(), &zlen, raw.data(), static_cast<uLong>(raw.size()), 6) != Z_OK) {
        throw std::runtime_error("zlib compress failed");
    }
    z.resize(zlen);
    std::vector<std::uint8_t> out;
    out.reserve(z.size() + 64);
    out.insert(out.end(), SIGNATURE.begin(), SIGNATURE.end());
    std::vector<std::uint8_t> ihdr;
    put_be(ihdr, width);
    put_be(ihdr, height);
    ihdr.insert(ihdr.end(), {8, ctype, 0, 0, 0});
    chunk(out, "IHDR", ihdr);
    chunk(out, "IDAT", z);
    chunk(out, "IEND", {});
    return out;
}

std::vector<std::uint8_t> encode_rgb8(std::uint32_t width, std::uint32_t height, std::span<const std::uint8_t> rgb) {
    return encode(width, height, 3, rgb);
}

std::expected<void, std::string> write_rgb8(const std::string& path, std::uint32_t width, std::uint32_t height,
                                            std::span<const std::uint8_t> rgb) {
    const auto bytes = encode_rgb8(width, height, rgb);
    // std::fs::write: File::create (O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0o666) + write_all
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
    if (fd < 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    std::size_t done = 0;
    while (done < bytes.size()) {
        const ssize_t n = ::write(fd, bytes.data() + done, bytes.size() - done);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            const auto e = cam::IoError::last_os().text();
            ::close(fd);
            return std::unexpected(e);
        }
        if (n == 0) {
            ::close(fd);
            return std::unexpected(std::string("failed to write whole buffer"));
        }
        done += static_cast<std::size_t>(n);
    }
    ::close(fd);
    return {};
}

std::expected<Decoded, std::string> decode(std::span<const std::uint8_t> png) {
    if (png.size() < 8 || !std::equal(SIGNATURE.begin(), SIGNATURE.end(), png.begin())) {
        return std::unexpected("not a PNG");
    }
    std::size_t pos = 8;
    Decoded d;
    std::vector<std::uint8_t> idat;
    while (pos + 12 <= png.size()) {
        const std::size_t len = get_be(png.data() + pos);
        const std::uint8_t* kind = png.data() + pos + 4;
        if (pos + 8 + len > png.size()) {
            return std::unexpected("truncated chunk");
        }
        const std::uint8_t* body = png.data() + pos + 8;
        if (pos + 12 + len > png.size()) {
            // the Rust code slices past the end here (a panic); a clean error instead
            return std::unexpected("truncated chunk");
        }
        const std::uint32_t crc = get_be(png.data() + pos + 8 + len);
        if (crc != crc32(png.subspan(pos + 4, len + 4))) {
            return std::unexpected(std::format("bad CRC in {}", std::string(reinterpret_cast<const char*>(kind), 4)));
        }
        const std::string_view k(reinterpret_cast<const char*>(kind), 4);
        if (k == "IHDR") {
            d.width = get_be(body);
            d.height = get_be(body + 4);
            if (body[8] != 8 || body[12] != 0) {
                return std::unexpected("only 8-bit non-interlaced");
            }
            switch (body[9]) {
            case 0:
                d.channels = 1;
                break;
            case 2:
                d.channels = 3;
                break;
            case 6:
                d.channels = 4;
                break;
            default:
                return std::unexpected(std::format("colour type {}", body[9]));
            }
        } else if (k == "IDAT") {
            idat.insert(idat.end(), body, body + len);
        } else if (k == "IEND") {
            break;
        }
        pos += 12 + len;
    }
    const std::size_t stride = static_cast<std::size_t>(d.width) * d.channels;
    const std::size_t want = (stride + 1) * d.height;
    std::vector<std::uint8_t> raw(want + 1);
    z_stream zs{};
    if (inflateInit(&zs) != Z_OK) {
        return std::unexpected("inflate: init");
    }
    zs.next_in = idat.data();
    zs.avail_in = static_cast<uInt>(idat.size());
    zs.next_out = raw.data();
    zs.avail_out = static_cast<uInt>(raw.size());
    const int r = inflate(&zs, Z_FINISH);
    const std::size_t got = raw.size() - zs.avail_out;
    inflateEnd(&zs);
    if (r != Z_STREAM_END) {
        return std::unexpected(r == Z_BUF_ERROR && got == raw.size() ? std::string("size mismatch")
                                                                      : std::format("inflate: {}", r));
    }
    if (got != want) {
        return std::unexpected("size mismatch");
    }
    d.data.assign(stride * d.height, 0);
    for (std::size_t y = 0; y < d.height; y++) {
        const std::uint8_t f = raw[y * (stride + 1)];
        for (std::size_t i = 0; i < stride; i++) {
            const std::uint8_t v = raw[y * (stride + 1) + 1 + i];
            const std::uint8_t left = i >= d.channels ? d.data[y * stride + i - d.channels] : 0;
            const std::uint8_t up = y > 0 ? d.data[(y - 1) * stride + i] : 0;
            switch (f) {
            case 0:
                d.data[y * stride + i] = v;
                break;
            case 1:
                d.data[y * stride + i] = static_cast<std::uint8_t>(v + left);
                break;
            case 2:
                d.data[y * stride + i] = static_cast<std::uint8_t>(v + up);
                break;
            default:
                return std::unexpected(std::format("filter {}", f));
            }
        }
    }
    return d;
}

} // namespace pibridge::lens::png
