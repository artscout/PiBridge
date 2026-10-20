// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/orb.rs (MIT OR Apache-2.0). See orb.h.
#include "pibridge/roommap/orb.h"

#include "pibridge/anchor/rustx.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>

namespace pibridge::roommap::orb {

namespace rs = anchor::rs;
using namespace detail;

namespace {

constexpr double HARRIS_K = 0.04;
constexpr double HARRIS_NORM = 1.0 / (49.0 * 49.0 * 2040.0 * 2040.0 * 2040.0 * 2040.0);
constexpr std::uint16_t GAUSS7[7] = {18, 34, 49, 54, 49, 34, 18};
constexpr std::uint64_t PATTERN_SEED = 0x0B5E55ED01234567ull;

double sane_scale(float scale) {
    if (scale >= 1.0f) {
        return std::fmin(static_cast<double>(scale), 8.0);
    }
    return 1.0;
}

/// One pattern coordinate: Irwin-Hall Gaussian (twelve 16-bit uniforms), sigma 6.2, rounded, clamped to -15..15.
std::pair<std::uint64_t, std::int32_t> gauss_coord(std::uint64_t state) {
    std::uint64_t st = state;
    std::int64_t sum = 0;
    for (int n = 0; n < 3; n++) {
        const auto [next, z] = splitmix64(st);
        st = next;
        sum += static_cast<std::int64_t>(z & 0xFFFF) + static_cast<std::int64_t>((z >> 16) & 0xFFFF) +
               static_cast<std::int64_t>((z >> 32) & 0xFFFF) + static_cast<std::int64_t>(z >> 48);
    }
    const std::int64_t num = (sum - 393210) * 62;
    const std::int64_t a = num + 327680;
    // div_euclid by a positive divisor: floor division
    std::int64_t v = a / 655360;
    if (a % 655360 < 0) {
        v -= 1;
    }
    if (v > 15) {
        v = 15;
    } else if (v < -15) {
        v = -15;
    }
    return {st, static_cast<std::int32_t>(v)};
}

std::array<std::array<std::int8_t, 4>, 256> make_pattern() {
    std::array<std::array<std::int8_t, 4>, 256> pat{};
    std::uint64_t st = PATTERN_SEED;
    std::size_t k = 0;
    while (k < 256) {
        const auto [s1, ax] = gauss_coord(st);
        const auto [s2, ay] = gauss_coord(s1);
        const auto [s3, bx] = gauss_coord(s2);
        const auto [s4, by] = gauss_coord(s3);
        st = s4;
        const std::int32_t dx = ax - bx;
        const std::int32_t dy = ay - by;
        if (dx * dx + dy * dy < MIN_PAIR_SEP2) {
            continue;
        }
        pat[k] = {static_cast<std::int8_t>(ax), static_cast<std::int8_t>(ay), static_cast<std::int8_t>(bx),
                  static_cast<std::int8_t>(by)};
        k++;
    }
    return pat;
}

std::array<std::size_t, IC_RADIUS + 1> make_umax() {
    std::array<std::size_t, IC_RADIUS + 1> t{};
    for (std::size_t v = 0; v <= IC_RADIUS; v++) {
        std::size_t u = IC_RADIUS;
        while (u * u + v * v > IC_RADIUS * IC_RADIUS) {
            u--;
        }
        t[v] = u;
    }
    return t;
}

const std::array<std::size_t, IC_RADIUS + 1> UMAX = make_umax();

/// Bilinear resampling on a corner-aligned grid: dst(x, y) = src(x * scale, y * scale), 8-bit fixed-point weights.
std::vector<std::uint8_t> resize(const Image& src, std::size_t dw, std::size_t dh, double scale) {
    const std::size_t sw = src.w;
    const std::size_t sh = src.h;
    struct Col {
        std::size_t x0, x1;
        std::uint32_t wl, wr;
    };
    std::vector<Col> cols(dw);
    for (std::size_t x = 0; x < dw; x++) {
        const double pos = static_cast<double>(x) * scale;
        const std::size_t x0 = std::min(rs::to_usize(pos), sw - 1);
        const std::size_t x1 = std::min(x0 + 1, sw - 1);
        const std::uint32_t f = std::min(rs::to_u32((pos - static_cast<double>(x0)) * 256.0 + 0.5), 256u);
        cols[x] = {x0, x1, 256 - f, f};
    }
    std::vector<std::uint8_t> dst(dw * dh);
    std::vector<std::uint16_t> vrow(sw);
    for (std::size_t y = 0; y < dh; y++) {
        const double pos = static_cast<double>(y) * scale;
        const std::size_t y0 = std::min(rs::to_usize(pos), sh - 1);
        const std::size_t y1 = std::min(y0 + 1, sh - 1);
        const std::uint32_t f = std::min(rs::to_u32((pos - static_cast<double>(y0)) * 256.0 + 0.5), 256u);
        const std::uint8_t* r0 = src.data.data() + y0 * src.stride;
        const std::uint8_t* r1 = src.data.data() + y1 * src.stride;
        for (std::size_t i = 0; i < sw; i++) {
            vrow[i] = static_cast<std::uint16_t>(static_cast<std::uint32_t>(r0[i]) * (256 - f) +
                                                 static_cast<std::uint32_t>(r1[i]) * f);
        }
        std::uint8_t* out = dst.data() + y * dw;
        for (std::size_t x = 0; x < dw; x++) {
            const Col& c = cols[x];
            out[x] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(vrow[c.x0]) * c.wl +
                                                static_cast<std::uint32_t>(vrow[c.x1]) * c.wr + 32768) >>
                                               16);
        }
    }
    return dst;
}

/// True if the 16-bit circular mask has nine consecutive set bits.
inline bool has_run9(std::uint32_t m) {
    const std::uint32_t m2 = m | (m << 16);
    std::uint32_t r = m2 & (m2 >> 1);
    r &= r >> 2;
    r &= r >> 4;
    r &= m2 >> 8;
    return (r & 0xFFFF) != 0;
}

/// Strongest first; ties broken by position so the order is total and deterministic.
int cand_cmp(const Cand& a, const Cand& b) {
    // b.resp.total_cmp(&a.resp) (f32)
    std::int32_t x;
    std::int32_t y;
    std::memcpy(&x, &b.resp, 4);
    std::memcpy(&y, &a.resp, 4);
    x ^= static_cast<std::int32_t>(static_cast<std::uint32_t>(x >> 31) >> 1);
    y ^= static_cast<std::int32_t>(static_cast<std::uint32_t>(y >> 31) >> 1);
    if (x != y) {
        return x < y ? -1 : 1;
    }
    if (a.level != b.level) {
        return a.level < b.level ? -1 : 1;
    }
    if (a.y != b.y) {
        return a.y < b.y ? -1 : 1;
    }
    if (a.x != b.x) {
        return a.x < b.x ? -1 : 1;
    }
    return 0;
}

bool cand_less(const Cand& a, const Cand& b) {
    return cand_cmp(a, b) < 0;
}

bool far_enough(const std::vector<Cand>& picked, const Cand& c) {
    for (const Cand& p : picked) {
        const std::size_t dx = p.x > c.x ? p.x - c.x : c.x - p.x;
        const std::size_t dy = p.y > c.y ? p.y - c.y : c.y - p.y;
        if (dx * dx + dy * dy < MIN_KP_DIST2) {
            return false;
        }
    }
    return true;
}

/// Per-level corner detection state (reused scratch buffers).
struct Detector {
    const Cfg& cfg;
    const Mask* mask;
    std::vector<std::uint8_t> scratch;
    std::vector<Corner> corners;

    void run_fast(const Image& img, Rect rect, std::uint8_t thr, float lscale) {
        fast_cell(img, rect, thr, scratch, corners);
        if (mask != nullptr) {
            const Mask& m = *mask;
            std::erase_if(corners, [&](const Corner& c) {
                return !m(static_cast<float>(c.x) * lscale, static_cast<float>(c.y) * lscale);
            });
        }
    }

    std::vector<Cand> detect_level(const Image& img, std::uint8_t level, float lscale, std::size_t budget) {
        const std::size_t n_cells = cfg.cols * cfg.rows;
        const std::size_t cap = std::max<std::size_t>(((budget + n_cells - 1) / n_cells) * 4, 16);
        const std::size_t rx0 = BORDER, ry0 = BORDER, rx1 = img.w - BORDER, ry1 = img.h - BORDER;
        const std::size_t rw = rx1 - rx0, rh = ry1 - ry0;
        std::vector<Cand> out;
        out.reserve(n_cells * std::min<std::size_t>(cap, 32));
        for (std::size_t cy = 0; cy < cfg.rows; cy++) {
            const std::size_t y0 = ry0 + cy * rh / cfg.rows;
            const std::size_t y1 = ry0 + (cy + 1) * rh / cfg.rows;
            for (std::size_t cx = 0; cx < cfg.cols; cx++) {
                const std::size_t x0 = rx0 + cx * rw / cfg.cols;
                const std::size_t x1 = rx0 + (cx + 1) * rw / cfg.cols;
                if (x1 <= x0 || y1 <= y0) {
                    continue;
                }
                const Rect rect{x0, y0, x1, y1};
                corners.clear();
                run_fast(img, rect, cfg.thr, lscale);
                if (corners.empty() && cfg.min_thr < cfg.thr) {
                    run_fast(img, rect, cfg.min_thr, lscale);
                }
                if (corners.size() > cap) {
                    std::sort(corners.begin(), corners.end(), [](const Corner& a, const Corner& b) {
                        if (a.score != b.score) {
                            return a.score > b.score;
                        }
                        if (a.y != b.y) {
                            return a.y < b.y;
                        }
                        return a.x < b.x;
                    });
                    corners.resize(cap);
                }
                const auto cell = static_cast<std::uint16_t>(cy * cfg.cols + cx);
                for (const Corner& c : corners) {
                    out.push_back(Cand{c.x, c.y, harris(img, c.x, c.y), level, cell});
                }
            }
        }
        return out;
    }
};

inline std::array<std::uint64_t, 4> words(const Descriptor& d) {
    std::array<std::uint64_t, 4> w;
    std::memcpy(w.data(), d.data(), 32);
    return w;
}

/// Population count without a libgcc call (the build does not assume the POPCNT instruction).
inline std::uint64_t pop64(std::uint64_t x) {
    x = x - ((x >> 1) & 0x5555555555555555ull);
    x = (x & 0x3333333333333333ull) + ((x >> 2) & 0x3333333333333333ull);
    x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0Full;
    return (x * 0x0101010101010101ull) >> 56;
}

inline std::uint32_t hamming_words(const std::array<std::uint64_t, 4>& a, const std::array<std::uint64_t, 4>& b) {
    return static_cast<std::uint32_t>(pop64(a[0] ^ b[0]) + pop64(a[1] ^ b[1]) + pop64(a[2] ^ b[2]) + pop64(a[3] ^ b[3]));
}

template <class F>
std::vector<Match> match_core(std::span<const Descriptor> a, std::span<const Descriptor> b, std::uint32_t max_dist,
                              float ratio, const F& allow) {
    std::vector<Match> out;
    if (a.empty() || b.empty()) {
        return out;
    }
    std::vector<std::array<std::uint64_t, 4>> aw(a.size());
    std::vector<std::array<std::uint64_t, 4>> bw(b.size());
    for (std::size_t i = 0; i < a.size(); i++) {
        aw[i] = words(a[i]);
    }
    for (std::size_t j = 0; j < b.size(); j++) {
        bw[j] = words(b[j]);
    }
    constexpr std::uint32_t UMAX32 = std::numeric_limits<std::uint32_t>::max();
    constexpr std::size_t NONE = std::numeric_limits<std::size_t>::max();
    std::vector<std::uint32_t> col_d(b.size(), UMAX32);
    std::vector<std::size_t> col_i(b.size(), NONE);
    struct Fwd {
        std::uint32_t d1;
        std::size_t j1;
        std::uint32_t d2;
    };
    std::vector<Fwd> fwd(a.size());
    for (std::size_t i = 0; i < a.size(); i++) {
        std::uint32_t d1 = UMAX32;
        std::size_t j1 = NONE;
        std::uint32_t d2 = UMAX32;
        const auto& wa = aw[i];
        for (std::size_t j = 0; j < b.size(); j++) {
            if (!allow(i, j)) {
                continue;
            }
            const std::uint32_t d = hamming_words(wa, bw[j]);
            if (d < d1) {
                d2 = d1;
                d1 = d;
                j1 = j;
            } else if (d < d2) {
                d2 = d;
            }
            if (d < col_d[j]) {
                col_d[j] = d;
                col_i[j] = i;
            }
        }
        fwd[i] = {d1, j1, d2};
    }
    for (std::size_t i = 0; i < a.size(); i++) {
        const auto [d1, j1, d2] = fwd[i];
        if (j1 == NONE || d1 > max_dist || col_i[j1] != i) {
            continue;
        }
        if (d2 == UMAX32 || static_cast<float>(d1) < ratio * static_cast<float>(d2)) {
            out.emplace_back(i, j1, d1);
        }
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------

std::optional<Image> Image::make(std::size_t w, std::size_t h, std::span<const std::uint8_t> data) {
    Image img{w, h, w, data};
    if (!img.is_valid()) {
        return std::nullopt;
    }
    return img;
}

bool Image::is_valid() const {
    if (w == 0 || h == 0 || stride < w) {
        return false;
    }
    std::size_t need;
    if (__builtin_mul_overflow(h - 1, stride, &need) || __builtin_add_overflow(need, w, &need)) {
        return false;
    }
    return data.size() >= need;
}

float OrbParams::level_scale(std::uint8_t level) const {
    return static_cast<float>(rs::powi(sane_scale(scale), static_cast<int>(level)));
}

namespace detail {

std::pair<std::uint64_t, std::uint64_t> splitmix64(std::uint64_t state) {
    state += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return {state, z ^ (z >> 31)};
}

const std::array<std::array<std::int8_t, 4>, 256>& pattern() {
    static const auto p = make_pattern();
    return p;
}

Cfg::Cfg(const OrbParams& p)
    : n_features(std::min(p.n_features, MAX_FEATURES)),
      n_levels(std::clamp<std::size_t>(p.n_levels, 1, MAX_LEVELS)),
      scale(sane_scale(p.scale)),
      thr(p.fast_threshold),
      min_thr(std::min(p.min_fast_threshold, p.fast_threshold)),
      cols(std::clamp<std::size_t>(p.grid_cols, 1, 64)),
      rows(std::clamp<std::size_t>(p.grid_rows, 1, 64)) {}

std::vector<Geom> level_geometry(std::size_t img_w, std::size_t img_h, const Cfg& cfg) {
    std::vector<Geom> v;
    v.reserve(cfg.n_levels);
    for (std::size_t l = 0; l < cfg.n_levels; l++) {
        const double s = rs::powi(cfg.scale, static_cast<int>(l));
        const std::size_t w = rs::to_usize(std::floor(static_cast<double>(img_w - 1) / s)) + 1;
        const std::size_t h = rs::to_usize(std::floor(static_cast<double>(img_h - 1) / s)) + 1;
        if (w < MIN_LEVEL_DIM || h < MIN_LEVEL_DIM) {
            break;
        }
        v.push_back(Geom{w, h, s});
    }
    return v;
}

std::vector<std::vector<std::uint8_t>> build_pyramid(const Image& img, const std::vector<Geom>& geom, double scale) {
    std::vector<std::vector<std::uint8_t>> bufs;
    bufs.reserve(geom.empty() ? 0 : geom.size() - 1);
    for (std::size_t l = 1; l < geom.size(); l++) {
        Image prev = img;
        if (l != 1) {
            prev = Image{geom[l - 1].w, geom[l - 1].h, geom[l - 1].w, bufs[l - 2]};
        }
        bufs.push_back(resize(prev, geom[l].w, geom[l].h, scale));
    }
    return bufs;
}

std::vector<Image> level_images(const Image& img, const std::vector<std::vector<std::uint8_t>>& bufs,
                                const std::vector<Geom>& geom) {
    std::vector<Image> v;
    v.reserve(geom.size());
    v.push_back(img);
    for (std::size_t i = 0; i < bufs.size() && i + 1 < geom.size(); i++) {
        v.push_back(Image{geom[i + 1].w, geom[i + 1].h, geom[i + 1].w, bufs[i]});
    }
    return v;
}

std::vector<std::uint8_t> gaussian_blur(const Image& img) {
    const std::size_t w = img.w;
    const std::size_t h = img.h;
    const auto& k = GAUSS7;
    std::vector<std::uint16_t> tmp(w * h);
    std::vector<std::uint8_t> padded(w + 6);
    for (std::size_t y = 0; y < h; y++) {
        const std::uint8_t* row = img.data.data() + y * img.stride;
        std::fill(padded.begin(), padded.begin() + 3, row[0]);
        std::memcpy(padded.data() + 3, row, w);
        std::fill(padded.begin() + 3 + static_cast<std::ptrdiff_t>(w), padded.end(), row[w - 1]);
        std::uint16_t* out = tmp.data() + y * w;
        for (std::size_t x = 0; x < w; x++) {
            const std::uint8_t* p = padded.data() + x;
            out[x] = static_cast<std::uint16_t>(k[3] * p[3] + k[2] * (p[2] + p[4]) + k[1] * (p[1] + p[5]) +
                                                k[0] * (p[0] + p[6]));
        }
    }
    std::vector<std::uint8_t> dst(w * h);
    const auto hh = static_cast<std::ptrdiff_t>(h);
    for (std::size_t y = 0; y < h; y++) {
        auto row = [&](std::ptrdiff_t d) {
            const std::ptrdiff_t yy = std::clamp<std::ptrdiff_t>(static_cast<std::ptrdiff_t>(y) + d, 0, hh - 1);
            return tmp.data() + static_cast<std::size_t>(yy) * w;
        };
        const std::uint16_t *r0 = row(-3), *r1 = row(-2), *r2 = row(-1), *r3 = row(0), *r4 = row(1), *r5 = row(2),
                            *r6 = row(3);
        std::uint8_t* out = dst.data() + y * w;
        for (std::size_t x = 0; x < w; x++) {
            const std::uint32_t s = std::uint32_t{k[3]} * r3[x] + std::uint32_t{k[2]} * (std::uint32_t{r2[x]} + r4[x]) +
                                    std::uint32_t{k[1]} * (std::uint32_t{r1[x]} + r5[x]) +
                                    std::uint32_t{k[0]} * (std::uint32_t{r0[x]} + r6[x]);
            out[x] = static_cast<std::uint8_t>((s + 32768) >> 16);
        }
    }
    return dst;
}

std::uint8_t ring_score(const std::array<std::uint8_t, 16>& ring, std::uint8_t center, std::uint8_t thr) {
    const std::int16_t c = center;
    const std::int16_t t = thr;
    std::int16_t d[16];
    std::uint32_t bright = 0;
    std::uint32_t dark = 0;
    for (int k = 0; k < 16; k++) {
        d[k] = static_cast<std::int16_t>(ring[k] - c);
        bright |= static_cast<std::uint32_t>(d[k] > t) << k;
        dark |= static_cast<std::uint32_t>(d[k] < -t) << k;
    }
    std::int16_t sign;
    if (has_run9(bright)) {
        sign = 1;
    } else if (has_run9(dark)) {
        sign = -1;
    } else {
        return 0;
    }
    std::int16_t s[24], m2[23], m4[21], m8[17];
    for (int i = 0; i < 24; i++) {
        s[i] = static_cast<std::int16_t>(sign * d[i % 16]);
    }
    for (int i = 0; i < 23; i++) {
        m2[i] = std::min(s[i], s[i + 1]);
    }
    for (int i = 0; i < 21; i++) {
        m4[i] = std::min(m2[i], m2[i + 2]);
    }
    for (int i = 0; i < 17; i++) {
        m8[i] = std::min(m4[i], m4[i + 4]);
    }
    std::int16_t best = std::numeric_limits<std::int16_t>::min();
    for (int i = 0; i < 16; i++) {
        best = std::max(best, std::min(m8[i], s[i + 8]));
    }
    return static_cast<std::uint8_t>(std::clamp<std::int16_t>(best, 0, 255));
}

void fast_cell(const Image& img, Rect r, std::uint8_t thr, std::vector<std::uint8_t>& scratch, std::vector<Corner>& out) {
    const std::size_t rw = r.x1 - r.x0 + 2;
    const std::size_t rh = r.y1 - r.y0 + 2;
    scratch.assign(rw * rh, 0);
    const std::size_t lo = r.x0 - 1 - 3;
    const std::int16_t t = thr;
    const std::uint8_t* base = img.data.data();
    for (std::size_t j = 0; j < rh; j++) {
        const std::size_t y = r.y0 - 1 + j;
        const std::uint8_t* r0 = base + (y - 3) * img.stride + lo;
        const std::uint8_t* r1 = r0 + img.stride;
        const std::uint8_t* r2 = r1 + img.stride;
        const std::uint8_t* r3 = r2 + img.stride;
        const std::uint8_t* r4 = r3 + img.stride;
        const std::uint8_t* r5 = r4 + img.stride;
        const std::uint8_t* r6 = r5 + img.stride;
        std::uint8_t* srow = scratch.data() + j * rw;
        for (std::size_t i = 0; i < rw; i++) {
            const std::int16_t c = r3[i + 3];
            const std::int16_t hi = static_cast<std::int16_t>(c + t);
            const std::int16_t lo_t = static_cast<std::int16_t>(c - t);
            const std::int16_t p0 = r0[i + 3], p4 = r3[i + 6], p8 = r6[i + 3], p12 = r3[i];
            const bool b0 = p0 > hi, b1 = p4 > hi, b2 = p8 > hi, b3 = p12 > hi;
            const bool d0 = p0 < lo_t, d1 = p4 < lo_t, d2 = p8 < lo_t, d3 = p12 < lo_t;
            const bool any_b = (b0 & b1) | (b1 & b2) | (b2 & b3) | (b3 & b0);
            const bool any_d = (d0 & d1) | (d1 & d2) | (d2 & d3) | (d3 & d0);
            if (!(any_b | any_d)) {
                continue;
            }
            const std::array<std::uint8_t, 16> ring = {r0[i + 3], r0[i + 4], r1[i + 5], r2[i + 6], r3[i + 6], r4[i + 6],
                                                       r5[i + 5], r6[i + 4], r6[i + 3], r6[i + 2], r5[i + 1], r4[i],
                                                       r3[i],     r2[i],     r1[i + 1], r0[i + 2]};
            srow[i] = ring_score(ring, r3[i + 3], thr);
        }
    }
    for (std::size_t y = 1; y + 1 < rh; y++) {
        for (std::size_t x = 1; x + 1 < rw; x++) {
            const std::uint8_t s = scratch[y * rw + x];
            if (s == 0) {
                continue;
            }
            const std::uint8_t* up = scratch.data() + (y - 1) * rw + x - 1;
            const std::uint8_t* mid = scratch.data() + y * rw + x - 1;
            const std::uint8_t* dn = scratch.data() + (y + 1) * rw + x - 1;
            const bool earlier = up[0] < s && up[1] < s && up[2] < s && mid[0] < s;
            const bool later = mid[2] <= s && dn[0] <= s && dn[1] <= s && dn[2] <= s;
            if (earlier && later) {
                out.push_back(Corner{r.x0 + x - 1, r.y0 + y - 1, s});
            }
        }
    }
}

float harris(const Image& img, std::size_t x, std::size_t y) {
    std::int64_t sxx = 0, syy = 0, sxy = 0;
    for (std::size_t yy = y - 3; yy <= y + 3; yy++) {
        const std::uint8_t* top = img.data.data() + (yy - 1) * img.stride + x - 4;
        const std::uint8_t* mid = top + img.stride;
        const std::uint8_t* bot = mid + img.stride;
        for (std::size_t k = 0; k < 7; k++) {
            const std::int32_t t0 = top[k], t1 = top[k + 1], t2 = top[k + 2];
            const std::int32_t m0 = mid[k], m2 = mid[k + 2];
            const std::int32_t b0 = bot[k], b1 = bot[k + 1], b2 = bot[k + 2];
            const std::int32_t ix = (t2 - t0) + 2 * (m2 - m0) + (b2 - b0);
            const std::int32_t iy = (b0 - t0) + 2 * (b1 - t1) + (b2 - t2);
            sxx += ix * ix;
            syy += iy * iy;
            sxy += ix * iy;
        }
    }
    const double det = static_cast<double>(sxx * syy - sxy * sxy);
    const double tr = static_cast<double>(sxx + syy);
    return static_cast<float>((det - HARRIS_K * tr * tr) * HARRIS_NORM);
}

float ic_angle(std::span<const std::uint8_t> img, std::size_t stride, std::size_t x, std::size_t y) {
    std::int32_t m10 = 0, m01 = 0;
    const auto rad = static_cast<std::ptrdiff_t>(IC_RADIUS);
    for (std::ptrdiff_t v = -rad; v <= rad; v++) {
        const std::size_t um = UMAX[static_cast<std::size_t>(v < 0 ? -v : v)];
        const std::size_t start = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(y) + v) * stride + x - um;
        std::int32_t s = 0;
        std::int32_t mu = 0;
        for (std::size_t k = 0; k <= 2 * um; k++) {
            const std::int32_t p = img[start + k];
            s += p;
            mu += (static_cast<std::int32_t>(k) - static_cast<std::int32_t>(um)) * p;
        }
        m10 += mu;
        m01 += static_cast<std::int32_t>(v) * s;
    }
    return std::atan2(static_cast<float>(m01), static_cast<float>(m10));
}

namespace {

inline float sample(const std::uint8_t* sm, std::size_t stride, float px, float py) {
    const auto ix = static_cast<std::size_t>(px);
    const auto iy = static_cast<std::size_t>(py);
    const float tx = px - static_cast<float>(ix);
    const float ty = py - static_cast<float>(iy);
    const std::size_t i = iy * stride + ix;
    const float p00 = sm[i], p01 = sm[i + 1];
    const float p10 = sm[i + stride], p11 = sm[i + stride + 1];
    const float top = p00 + (p01 - p00) * tx;
    const float bot = p10 + (p11 - p10) * tx;
    return top + (bot - top) * ty;
}

} // namespace

Descriptor describe(std::span<const std::uint8_t> sm, std::size_t stride, std::size_t x, std::size_t y, float angle) {
    const float s = std::sin(angle);
    const float c = std::cos(angle);
    const float fx = static_cast<float>(x);
    const float fy = static_cast<float>(y);
    Descriptor out{};
    const auto& pat = pattern();
    for (std::size_t k = 0; k < 256; k++) {
        const float ax = pat[k][0], ay = pat[k][1], bx = pat[k][2], by = pat[k][3];
        const float a = sample(sm.data(), stride, fx + (c * ax - s * ay), fy + (s * ax + c * ay));
        const float b = sample(sm.data(), stride, fx + (c * bx - s * by), fy + (s * bx + c * by));
        out[k >> 3] = static_cast<std::uint8_t>(out[k >> 3] | (static_cast<std::uint8_t>(a < b) << (k & 7)));
    }
    return out;
}

std::pair<std::vector<Cand>, std::vector<Cand>> select_level(const std::vector<Cand>& cands, std::size_t n_cells,
                                                             std::size_t budget) {
    std::vector<std::size_t> order(cands.size());
    for (std::size_t i = 0; i < order.size(); i++) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return cand_less(cands[a], cands[b]); });
    std::vector<bool> taken(cands.size(), false);
    std::vector<bool> cell_done(n_cells, false);
    std::vector<Cand> picked;
    picked.reserve(std::min(budget, cands.size()));
    for (std::size_t i : order) {
        if (picked.size() >= budget) {
            break;
        }
        const Cand& c = cands[i];
        if (!cell_done[c.cell] && far_enough(picked, c)) {
            picked.push_back(c);
            taken[i] = true;
            cell_done[c.cell] = true;
        }
    }
    for (std::size_t i : order) {
        if (picked.size() >= budget) {
            break;
        }
        if (!taken[i] && far_enough(picked, cands[i])) {
            picked.push_back(cands[i]);
            taken[i] = true;
        }
    }
    std::vector<Cand> rest;
    for (std::size_t i : order) {
        if (!taken[i]) {
            rest.push_back(cands[i]);
        }
    }
    return {std::move(picked), std::move(rest)};
}

void top_up(std::vector<std::vector<Cand>>& picked, std::vector<Cand> unused, std::size_t n_features) {
    std::size_t total = 0;
    for (const auto& v : picked) {
        total += v.size();
    }
    if (total >= n_features) {
        return;
    }
    std::sort(unused.begin(), unused.end(), cand_less);
    for (const Cand& c : unused) {
        if (total >= n_features) {
            break;
        }
        auto& lv = picked[c.level];
        if (far_enough(lv, c)) {
            lv.push_back(c);
            total++;
        }
    }
}

std::vector<std::size_t> level_budgets(std::size_t n, const std::vector<Geom>& geom) {
    std::vector<double> areas;
    for (const Geom& g : geom) {
        areas.push_back(static_cast<double>(g.w * g.h));
    }
    double total = -0.0;
    for (double a : areas) {
        total += a;
    }
    std::vector<double> exact;
    for (double a : areas) {
        exact.push_back(static_cast<double>(n) * a / total);
    }
    std::vector<std::size_t> out;
    std::size_t sum = 0;
    for (double e : exact) {
        out.push_back(rs::to_usize(std::floor(e)));
        sum += out.back();
    }
    const std::size_t rem = n > sum ? n - sum : 0;
    std::vector<std::size_t> idx(out.size());
    for (std::size_t i = 0; i < idx.size(); i++) {
        idx[i] = i;
    }
    std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
        const double fa = exact[a] - std::floor(exact[a]);
        const double fb = exact[b] - std::floor(exact[b]);
        const int c = rs::total_cmp(fb, fa);
        if (c != 0) {
            return c < 0;
        }
        return a < b;
    });
    for (std::size_t k = 0; k < rem && k < idx.size(); k++) {
        out[idx[k]] += 1;
    }
    return out;
}

} // namespace detail

std::vector<std::pair<Keypoint, Descriptor>> extract(const Image& img, const Mask* mask, const OrbParams& p) {
    const Cfg cfg(p);
    std::vector<std::pair<Keypoint, Descriptor>> out;
    if (cfg.n_features == 0 || !img.is_valid()) {
        return out;
    }
    const auto geom = level_geometry(img.w, img.h, cfg);
    if (geom.empty()) {
        return out;
    }
    const auto bufs = build_pyramid(img, geom, cfg.scale);
    const auto levels = level_images(img, bufs, geom);
    const auto budgets = level_budgets(cfg.n_features, geom);
    const std::size_t n_cells = cfg.cols * cfg.rows;
    Detector det{cfg, mask, {}, {}};
    std::vector<std::vector<Cand>> picked;
    picked.reserve(levels.size());
    std::vector<Cand> unused;
    for (std::size_t l = 0; l < levels.size(); l++) {
        if (budgets[l] == 0) {
            picked.emplace_back();
            continue;
        }
        const auto cands =
            det.detect_level(levels[l], static_cast<std::uint8_t>(l), static_cast<float>(geom[l].scale), budgets[l]);
        auto [sel, rest] = select_level(cands, n_cells, budgets[l]);
        picked.push_back(std::move(sel));
        unused.insert(unused.end(), rest.begin(), rest.end());
    }
    top_up(picked, std::move(unused), cfg.n_features);
    std::size_t total = 0;
    for (const auto& v : picked) {
        total += v.size();
    }
    out.reserve(total);
    for (std::size_t l = 0; l < picked.size(); l++) {
        auto& sel = picked[l];
        if (sel.empty()) {
            continue;
        }
        std::sort(sel.begin(), sel.end(), [](const Cand& a, const Cand& b) {
            return a.y != b.y ? a.y < b.y : a.x < b.x;
        });
        const Image& im = levels[l];
        const auto sm = gaussian_blur(im);
        const float lscale = static_cast<float>(geom[l].scale);
        for (const Cand& c : sel) {
            const float angle = ic_angle(sm, im.w, c.x, c.y);
            const Descriptor desc = describe(sm, im.w, c.x, c.y, angle);
            out.emplace_back(Keypoint{static_cast<float>(c.x) * lscale, static_cast<float>(c.y) * lscale,
                                      static_cast<std::uint8_t>(l), angle, c.resp},
                             desc);
        }
    }
    return out;
}

std::uint32_t hamming(const Descriptor& a, const Descriptor& b) {
    return hamming_words(words(a), words(b));
}

std::vector<Match> match_bf(std::span<const Descriptor> a, std::span<const Descriptor> b, std::uint32_t max_dist,
                            float ratio) {
    return match_core(a, b, max_dist, ratio, [](std::size_t, std::size_t) { return true; });
}

std::vector<Match> match_gated(std::span<const Descriptor> a, std::span<const Descriptor> b, std::uint32_t max_dist,
                               float ratio, const std::function<bool(std::size_t, std::size_t)>& allow) {
    return match_core(a, b, max_dist, ratio, allow);
}

} // namespace pibridge::roommap::orb
