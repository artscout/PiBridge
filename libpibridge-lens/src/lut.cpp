// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/lut.rs (MIT OR Apache-2.0).
#include "pibridge/lens/lut.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/lens/fmt.h"
#include "pibridge/lens/png.h"
#include "pibridge/lens/pvrdump.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <format>

namespace pibridge::lens {

const std::array<Orient, 8> ORIENTS = {{
    {{{{1.0, 0.0}, {0.0, 1.0}}}, "user x = panel u, user y = panel v"},
    {{{{0.0, -1.0}, {1.0, 0.0}}}, "user x = -panel v, user y = panel u"},
    {{{{-1.0, 0.0}, {0.0, -1.0}}}, "user x = -panel u, user y = -panel v"},
    {{{{0.0, 1.0}, {-1.0, 0.0}}}, "user x = panel v, user y = -panel u"},
    {{{{-1.0, 0.0}, {0.0, 1.0}}}, "user x = -panel u, user y = panel v"},
    {{{{1.0, 0.0}, {0.0, -1.0}}}, "user x = panel u, user y = -panel v"},
    {{{{0.0, 1.0}, {1.0, 0.0}}}, "user x = panel v, user y = panel u"},
    {{{{0.0, -1.0}, {-1.0, 0.0}}}, "user x = -panel v, user y = -panel u"},
}};

namespace {

constexpr std::size_t HEADER = 56 + 2 * 36 + 16;

std::uint32_t rd_u32(std::span<const std::uint8_t> b, std::size_t o) {
    return std::uint32_t{b[o]} | (std::uint32_t{b[o + 1]} << 8) | (std::uint32_t{b[o + 2]} << 16) |
           (std::uint32_t{b[o + 3]} << 24);
}

float rd_f32(std::span<const std::uint8_t> b, std::size_t o) {
    return std::bit_cast<float>(rd_u32(b, o));
}

void put_u32(std::vector<std::uint8_t>& b, std::uint32_t v) {
    for (int i = 0; i < 4; i++) {
        b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
}

void put_f32(std::vector<std::uint8_t>& b, float v) {
    put_u32(b, std::bit_cast<std::uint32_t>(v));
}

// Resample panel-order data into the user-display grid for orientation matrix `d` (a signed permutation).
std::vector<Tan> reorient(const std::vector<Tan>& raw, std::size_t n, const Dm& d) {
    const double h = static_cast<double>(n - 1);
    std::vector<Tan> out(n * n, Tan{0.0f, 0.0f});
    for (std::size_t j = 0; j < n; j++) {
        for (std::size_t i = 0; i < n; i++) {
            // user_n = D panel_n, so panel_n = D^T user_n (D is orthogonal)
            const double x = static_cast<double>(i) / h - 0.5;
            const double y = 0.5 - static_cast<double>(j) / h;
            const double pu = d[0][0] * x + d[1][0] * y + 0.5;
            const double pv = d[0][1] * x + d[1][1] * y + 0.5;
            const auto pi = fmt::sat_cast<std::size_t>(std::round(pu * h));
            const auto pj = fmt::sat_cast<std::size_t>(std::round(pv * h));
            out[j * n + i] = raw[std::min(pj, n - 1) * n + std::min(pi, n - 1)];
        }
    }
    return out;
}

bool finite2(const Tan& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]);
}

} // namespace

Lut::EyeLut::EyeLut(EyeMeta m, std::array<std::vector<Tan>, CHANNELS> r, std::size_t n)
    : meta(m), raw(std::move(r)) {
    const Dm& d = ORIENTS[meta.orient].d;
    for (std::size_t c = 0; c < CHANNELS; c++) {
        data[c] = reorient(raw[c], n, d);
    }
}

std::pair<double, double> panel_to_display(const Dm& d, double pu, double pv) {
    const double px = pu - 0.5;
    const double py = pv - 0.5;
    const double x = d[0][0] * px + d[0][1] * py;
    const double y = d[1][0] * px + d[1][1] * py;
    return {0.5 + x, 0.5 - y};
}

std::expected<Lut, std::string> Lut::from_dump(const Dump& dump) {
    const std::size_t n = dump.eyes[0].grid_n;
    if (dump.eyes[1].grid_n != n || !(n >= 2 && n <= 1025)) {
        return std::unexpected(std::format("dump: unusable grid size {}", n));
    }
    Lut lut;
    for (std::size_t e = 0; e < 2; e++) {
        const EyeDump& ed = dump.eyes[e];
        const auto [up, dn, l, r] = ed.fov_tan;
        // render UV V = 0 is the bottom of the FOV port
        std::array<std::vector<Tan>, CHANNELS> raw;
        for (std::size_t c = 0; c < CHANNELS; c++) {
            raw[c].reserve(n * n);
            for (std::size_t k = 0; k < n * n; k++) {
                const double ou = ed.grid[c][2 * k];
                const double ov = ed.grid[c][2 * k + 1];
                const double t0 = -l + ou * (l + r);
                const double t1 = -dn + ov * (up + dn);
                if (!(std::isfinite(t0) && std::isfinite(t1))) {
                    return std::unexpected(std::format("dump: eye {} channel {} sample {} is not finite", e, c, k));
                }
                raw[c].push_back({static_cast<float>(t0), static_cast<float>(t1)});
            }
        }
        EyeMeta meta;
        meta.orient = e == 0 ? ORIENT_LEFT : ORIENT_RIGHT;
        meta.panel = {static_cast<float>(ed.panel.first), static_cast<float>(ed.panel.second)};
        meta.fov_tan = {static_cast<float>(up), static_cast<float>(dn), static_cast<float>(l), static_cast<float>(r)};
        lut.eyes_[e] = EyeLut(meta, std::move(raw), n);
    }
    lut.serial = dump.serial;
    lut.n = n;
    return lut;
}

std::array<float, 4> Lut::fov_deg(std::size_t eye) const {
    const auto [up, dn, l, r] = eyes_[eye].meta.fov_tan;
    std::array<float, 4> t = {l, r, up, dn};
    for (float& x : t) {
        x = fmt::to_degrees(std::atan(x));
    }
    return t;
}

std::vector<std::uint8_t> Lut::to_bytes() const {
    std::vector<std::uint8_t> b;
    b.reserve(HEADER + 2 * CHANNELS * n * n * 8 + 4);
    b.insert(b.end(), LUT_MAGIC, LUT_MAGIC + 8);
    for (const std::uint32_t v : {LUT_VERSION, static_cast<std::uint32_t>(n), static_cast<std::uint32_t>(CHANNELS),
                                  std::uint32_t{2}}) {
        put_u32(b, v);
    }
    std::array<std::uint8_t, 32> s{};
    const std::size_t sl = std::min<std::size_t>(serial.size(), 31);
    std::memcpy(s.data(), serial.data(), sl);
    b.insert(b.end(), s.begin(), s.end());
    for (const auto& e : eyes_) {
        put_u32(b, e.meta.orient);
        for (const float v : e.meta.panel) {
            put_f32(b, v);
        }
        for (const float v : e.meta.fov_tan) {
            put_f32(b, v);
        }
        put_f32(b, 0.0f);
        put_f32(b, 0.0f);
    }
    b.resize(HEADER, 0);
    for (const auto& e : eyes_) {
        for (const auto& ch : e.raw) {
            for (const auto& p : ch) {
                put_f32(b, p[0]);
                put_f32(b, p[1]);
            }
        }
    }
    put_u32(b, png::crc32(b));
    return b;
}

Lut Lut::rotated_yaw(std::array<double, 2> yaw_rad) const {
    Lut out = *this;
    for (std::size_t e = 0; e < 2; e++) {
        const double c = std::cos(yaw_rad[e]), s = std::sin(yaw_rad[e]);
        // the ray (tx, ty, 1) turned about +y: x' = x c + s, z' = c - x s; tangents divide by z'
        const auto turn = [&](Tan t) -> Tan {
            const double x = t[0], y = t[1], z = c - x * s;
            return {static_cast<float>((x * c + s) / z), static_cast<float>(y / z)};
        };
        for (std::size_t ch = 0; ch < CHANNELS; ch++) {
            for (auto& p : out.eyes_[e].raw[ch]) {
                p = turn(p);
            }
            for (auto& p : out.eyes_[e].data[ch]) {
                p = turn(p);
            }
        }
        const auto [up, dn, l, r] = eyes_[e].meta.fov_tan;
        const double zl = c + static_cast<double>(l) * s, zr = c - static_cast<double>(r) * s;
        const double zmin = std::min(zl, zr);
        out.eyes_[e].meta.fov_tan = {static_cast<float>(up / zmin), static_cast<float>(dn / zmin),
                                     static_cast<float>(-((-static_cast<double>(l) * c + s) / zl)),
                                     static_cast<float>((static_cast<double>(r) * c + s) / zr)};
    }
    return out;
}

std::expected<Lut, std::string> Lut::from_bytes(std::span<const std::uint8_t> b) {
    if (b.size() < HEADER + 4 || std::memcmp(b.data(), LUT_MAGIC, 8) != 0) {
        return std::unexpected("not a pimax LUT file (bad magic)");
    }
    const std::uint32_t ver = rd_u32(b, 8);
    if (ver != LUT_VERSION) {
        return std::unexpected(std::format("unsupported LUT version {} (expected {})", ver, LUT_VERSION));
    }
    const std::size_t n = rd_u32(b, 12);
    if (rd_u32(b, 16) != CHANNELS || rd_u32(b, 20) != 2 || !(n >= 2 && n <= 1025)) {
        return std::unexpected("bad LUT header (channels/eyes/grid size)");
    }
    const std::size_t want = HEADER + 2 * CHANNELS * n * n * 8 + 4;
    if (b.size() != want) {
        return std::unexpected(std::format("bad LUT size {} (expected {})", b.size(), want));
    }
    if (png::crc32(b.first(want - 4)) != rd_u32(b, want - 4)) {
        return std::unexpected("LUT checksum mismatch");
    }
    Lut lut;
    std::string serial = cam::rfmt::utf8_lossy(std::string_view(reinterpret_cast<const char*>(b.data()) + 24, 32));
    while (!serial.empty() && serial.back() == '\0') {
        serial.pop_back();
    }
    std::size_t off = HEADER;
    for (std::size_t e = 0; e < 2; e++) {
        const std::size_t h = 56 + e * 36;
        const std::uint32_t orient = rd_u32(b, h);
        if (orient >= ORIENTS.size()) {
            return std::unexpected(std::format("bad LUT orientation {}", orient));
        }
        EyeMeta meta;
        meta.orient = orient;
        meta.panel = {rd_f32(b, h + 4), rd_f32(b, h + 8)};
        meta.fov_tan = {rd_f32(b, h + 12), rd_f32(b, h + 16), rd_f32(b, h + 20), rd_f32(b, h + 24)};
        for (const float t : meta.fov_tan) {
            if (!(std::isfinite(t) && t > 0.0f)) {
                return std::unexpected("bad LUT FOV port");
            }
        }
        std::array<std::vector<Tan>, CHANNELS> raw;
        for (auto& rc : raw) {
            rc.reserve(n * n);
            for (std::size_t k = 0; k < n * n; k++) {
                const Tan p = {rd_f32(b, off), rd_f32(b, off + 4)};
                if (!finite2(p)) {
                    return std::unexpected("non-finite value in LUT");
                }
                rc.push_back(p);
                off += 8;
            }
        }
        lut.eyes_[e] = EyeLut(meta, std::move(raw), n);
    }
    lut.serial = std::move(serial);
    lut.n = n;
    return lut;
}

std::expected<Lut, std::string> Lut::load(const std::string& path) {
    const auto bytes = cam::read_file(path);
    if (!bytes) {
        return std::unexpected(std::format("{}: {}", path, bytes.error().text()));
    }
    auto l = from_bytes(std::span(reinterpret_cast<const std::uint8_t*>(bytes->data()), bytes->size()));
    if (!l) {
        return std::unexpected(std::format("{}: {}", path, l.error()));
    }
    return l;
}

namespace detail {

std::array<float, 4> catmull_rom(float t) {
    const float t2 = t * t;
    const float t3 = t * t * t;
    return {0.5f * (-t3 + 2.0f * t2 - t), 0.5f * (3.0f * t3 - 5.0f * t2 + 2.0f), 0.5f * (-3.0f * t3 + 4.0f * t2 + t),
            0.5f * (t3 - t2)};
}

double ray_angle_deg(std::array<double, 2> a, std::array<double, 2> b) {
    const double na = std::sqrt(1.0 + a[0] * a[0] + a[1] * a[1]);
    const double nb = std::sqrt(1.0 + b[0] * b[0] + b[1] * b[1]);
    const double dot = (a[0] * b[0] + a[1] * b[1] + 1.0) / (na * nb);
    return cam::rfmt::to_degrees(std::acos(cam::rfmt::clamp(dot, -1.0, 1.0)));
}

} // namespace detail

Tan Lut::sample(std::size_t eye, std::size_t c, float u, float v, Interp interp) const {
    const std::size_t nn = n;
    const auto& g = eyes_[eye].data[c];
    const float h = static_cast<float>(nn - 1);
    const float fx = fmt::clampf(u, 0.0f, 1.0f) * h;
    const float fy = fmt::clampf(v, 0.0f, 1.0f) * h;
    const std::size_t i0 = std::min(fmt::sat_cast<std::size_t>(std::floor(fx)), nn - 2);
    const std::size_t j0 = std::min(fmt::sat_cast<std::size_t>(std::floor(fy)), nn - 2);
    const float tx = fx - static_cast<float>(i0);
    const float ty = fy - static_cast<float>(j0);
    const auto last = static_cast<std::ptrdiff_t>(nn) - 1;
    const auto at = [&](std::ptrdiff_t i, std::ptrdiff_t j) -> const Tan& {
        return g[static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(j, 0, last)) * nn +
                 static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(i, 0, last))];
    };
    const auto i0s = static_cast<std::ptrdiff_t>(i0);
    const auto j0s = static_cast<std::ptrdiff_t>(j0);
    if (interp == Interp::Bilinear) {
        const Tan& a = at(i0s, j0s);
        const Tan& b = at(i0s + 1, j0s);
        const Tan& c2 = at(i0s, j0s + 1);
        const Tan& d = at(i0s + 1, j0s + 1);
        Tan o = {0.0f, 0.0f};
        for (std::size_t k = 0; k < 2; k++) {
            const float top = a[k] + (b[k] - a[k]) * tx;
            const float bot = c2[k] + (d[k] - c2[k]) * tx;
            o[k] = top + (bot - top) * ty;
        }
        return o;
    }
    const auto wx = detail::catmull_rom(tx);
    const auto wy = detail::catmull_rom(ty);
    Tan o = {0.0f, 0.0f};
    for (std::size_t dj = 0; dj < 4; dj++) {
        for (std::size_t di = 0; di < 4; di++) {
            const Tan& p = at(i0s + static_cast<std::ptrdiff_t>(di) - 1, j0s + static_cast<std::ptrdiff_t>(dj) - 1);
            const float w = wx[di] * wy[dj];
            o[0] += p[0] * w;
            o[1] += p[1] * w;
        }
    }
    return o;
}

LineError line_error(const Lut& lut, const Dump& dump, Interp interp) {
    double sum = 0.0;
    double max = 0.0;
    std::size_t cnt = 0;
    LineError le;
    for (std::size_t e = 0; e < 2; e++) {
        const EyeDump& ed = dump.eyes[e];
        const Dm& d = ORIENTS[lut.meta(e).orient].d;
        const auto [up, dn, l, r] = ed.fov_tan;
        for (const auto& line : ed.lines) {
            for (std::size_t c = 0; c < CHANNELS; c++) {
                for (std::size_t k = 0; k < ed.line_n; k++) {
                    const double ou = line.out[c][2 * k];
                    const double ov = line.out[c][2 * k + 1];
                    if (!(0.0 <= ou && ou <= 1.0) || !(0.0 <= ov && ov <= 1.0)) {
                        continue;
                    }
                    const double s = static_cast<double>(k) / static_cast<double>(ed.line_n - 1);
                    const double pu = 0.5 + line.dir[0] * 0.5 * s;
                    const double pv = 0.5 + line.dir[1] * 0.5 * s;
                    const auto [u, v] = panel_to_display(d, pu, pv);
                    const std::array<double, 2> want = {-l + ou * (l + r), -dn + ov * (up + dn)};
                    const Tan got = lut.sample(e, c, static_cast<float>(u), static_cast<float>(v), interp);
                    const double a = detail::ray_angle_deg(want, {double{got[0]}, double{got[1]}});
                    sum += a * a;
                    if (a > max) {
                        max = a;
                        le.worst_eye = e;
                        le.worst_line = line.name;
                        le.worst_channel = c;
                        le.worst_pos = s;
                    }
                    cnt++;
                }
            }
        }
    }
    le.samples = cnt;
    le.rms_deg = cnt > 0 ? std::sqrt(sum / static_cast<double>(cnt)) : 0.0;
    le.max_deg = max;
    return le;
}

} // namespace pibridge::lens
