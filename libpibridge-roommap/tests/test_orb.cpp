// SPDX-License-Identifier: MIT
// Twins of omarchyvr-room orb.rs `tests` (27 tests; the #[ignore] timing report is left out).
#include "pibridge/roommap/orb.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <bit>
#include <cstdio>
#include <cstring>
#include <numbers>
#include <optional>
#include <vector>

using namespace pibridge::roommap::orb;
using namespace pibridge::roommap::orb::detail;

namespace {

constexpr float PI = std::numbers::pi_v<float>;
constexpr float TAU = 2.0f * std::numbers::pi_v<float>;
constexpr std::size_t W = 640;
constexpr std::size_t H = 480;
constexpr float CX = (W - 1) / 2.0f;
constexpr float CY = (H - 1) / 2.0f;

struct Rng {
    std::uint64_t s;
    std::uint64_t next_u64() {
        auto [n, z] = splitmix64(s);
        s = n;
        return z;
    }
    float unit() { return static_cast<float>(next_u64() >> 40) / static_cast<float>(1ull << 24); }
    float range(float lo, float hi) { return lo + (hi - lo) * unit(); }
    float gauss() {
        float sum = -0.0f;
        for (int i = 0; i < 12; i++) {
            sum += unit();
        }
        return sum - 6.0f;
    }
    Descriptor descriptor() {
        Descriptor d{};
        for (int k = 0; k < 4; k++) {
            const std::uint64_t w = next_u64();
            std::memcpy(d.data() + 8 * k, &w, 8);
        }
        return d;
    }
};

bool in_triangle(float u, float v, float a, float b) {
    const float p[3][2] = {{a, 0.0f}, {-a * 0.5f, b}, {-a * 0.5f, -b}};
    auto e = [&](int i) {
        const float x0 = p[i][0], y0 = p[i][1];
        const float x1 = p[(i + 1) % 3][0], y1 = p[(i + 1) % 3][1];
        return (x1 - x0) * (v - y0) - (y1 - y0) * (u - x0);
    };
    const float e0 = e(0), e1 = e(1), e2 = e(2);
    return (e0 >= 0.0f && e1 >= 0.0f && e2 >= 0.0f) || (e0 <= 0.0f && e1 <= 0.0f && e2 <= 0.0f);
}

std::vector<float> blur3(const std::vector<float>& src, std::size_t w, std::size_t h) {
    std::vector<float> tmp(w * h), out(w * h);
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const std::size_t l = x == 0 ? 0 : x - 1, r = std::min(x + 1, w - 1);
            tmp[y * w + x] = 0.25f * src[y * w + l] + 0.5f * src[y * w + x] + 0.25f * src[y * w + r];
        }
    }
    for (std::size_t y = 0; y < h; y++) {
        const std::size_t u = y == 0 ? 0 : y - 1, d = std::min(y + 1, h - 1);
        for (std::size_t x = 0; x < w; x++) {
            out[y * w + x] = 0.25f * tmp[u * w + x] + 0.5f * tmp[y * w + x] + 0.25f * tmp[d * w + x];
        }
    }
    return out;
}

std::vector<float> canvas(std::size_t w, std::size_t h, std::uint64_t seed) {
    Rng rng{seed};
    const float gx = rng.range(-0.06f, 0.06f);
    const float gy = rng.range(-0.06f, 0.06f);
    const float base = rng.range(90.0f, 130.0f);
    std::vector<float> img(w * h);
    for (std::size_t i = 0; i < w * h; i++) {
        img[i] = base + gx * static_cast<float>(i % w) + gy * static_cast<float>(i / w);
    }
    for (std::size_t n = 0; n < w * h / 600; n++) {
        const float val = rng.range(20.0f, 240.0f);
        const float cx = rng.range(0.0f, static_cast<float>(w));
        const float cy = rng.range(0.0f, static_cast<float>(h));
        const float size = rng.range(5.0f, 26.0f);
        const float a = size * rng.range(0.5f, 1.4f);
        const float b = size * rng.range(0.5f, 1.4f);
        const std::uint64_t kind = rng.next_u64() % 3;
        const float ang = rng.range(0.0f, TAU);
        const float s = std::sin(ang), c = std::cos(ang);
        const float r = std::ceil(std::fmax(a, b) * 1.5f + 2.0f);
        const std::size_t x0 = static_cast<std::size_t>(std::fmax(cx - r, 0.0f));
        const std::size_t x1 = std::min(static_cast<std::size_t>(std::fmax(cx + r, 0.0f)), w - 1);
        const std::size_t y0 = static_cast<std::size_t>(std::fmax(cy - r, 0.0f));
        const std::size_t y1 = std::min(static_cast<std::size_t>(std::fmax(cy + r, 0.0f)), h - 1);
        for (std::size_t y = y0; y <= y1; y++) {
            for (std::size_t x = x0; x <= x1; x++) {
                const float dx = static_cast<float>(x) - cx, dy = static_cast<float>(y) - cy;
                const float u = c * dx + s * dy, v = -s * dx + c * dy;
                bool inside;
                if (kind == 0) {
                    inside = std::fabs(u) <= a && std::fabs(v) <= b;
                } else if (kind == 1) {
                    inside = in_triangle(u, v, a, b);
                } else {
                    inside = (u / a) * (u / a) + (v / b) * (v / b) <= 1.0f;
                }
                if (inside) {
                    img[y * w + x] = val;
                }
            }
        }
    }
    for (auto& p : img) {
        p += 4.0f * rng.gauss();
    }
    return blur3(img, w, h);
}

std::vector<std::uint8_t> to_u8(const std::vector<float>& v) {
    std::vector<std::uint8_t> o(v.size());
    for (std::size_t i = 0; i < v.size(); i++) {
        o[i] = static_cast<std::uint8_t>(std::clamp(std::round(v[i]), 0.0f, 255.0f));
    }
    return o;
}

float bilinear(const std::vector<float>& img, std::size_t w, std::size_t h, float x, float y) {
    x = std::clamp(x, 0.0f, static_cast<float>(w - 1) - 1e-3f);
    y = std::clamp(y, 0.0f, static_cast<float>(h - 1) - 1e-3f);
    const std::size_t x0 = static_cast<std::size_t>(x), y0 = static_cast<std::size_t>(y);
    const float fx = x - static_cast<float>(x0), fy = y - static_cast<float>(y0);
    const float a = img[y0 * w + x0] * (1.0f - fx) + img[y0 * w + x0 + 1] * fx;
    const float b = img[(y0 + 1) * w + x0] * (1.0f - fx) + img[(y0 + 1) * w + x0 + 1] * fx;
    return a * (1.0f - fy) + b * fy;
}

struct Pair {
    std::vector<std::uint8_t> a, b;
    float ang, zoom;
    std::pair<float, float> shift;

    Pair(std::uint64_t seed, float ang_, float zoom_, std::pair<float, float> shift_, float gain, float offset, float sigma)
        : ang(ang_), zoom(zoom_), shift(shift_) {
        const std::size_t m = 120;
        const std::size_t cw = W + 2 * m, ch = H + 2 * m;
        const auto cv = canvas(cw, ch, seed);
        Rng rng{seed ^ 0x5EED};
        const float s = std::sin(ang), c = std::cos(ang);
        a.assign(W * H, 0);
        b.assign(W * H, 0);
        for (std::size_t y = 0; y < H; y++) {
            for (std::size_t x = 0; x < W; x++) {
                a[y * W + x] = static_cast<std::uint8_t>(std::clamp(std::round(cv[(y + m) * cw + x + m]), 0.0f, 255.0f));
                const float qx = static_cast<float>(x) - CX - shift.first, qy = static_cast<float>(y) - CY - shift.second;
                const float ux = c * qx + s * qy, uy = -s * qx + c * qy;
                const float px = CX + ux / zoom, py = CY + uy / zoom;
                const float v = bilinear(cv, cw, ch, px + static_cast<float>(m), py + static_cast<float>(m));
                b[y * W + x] = static_cast<std::uint8_t>(std::clamp(std::round(v * gain + offset + sigma * rng.gauss()), 0.0f, 255.0f));
            }
        }
    }
    std::pair<float, float> forward(float x, float y) const {
        const float s = std::sin(ang), c = std::cos(ang);
        const float dx = x - CX, dy = y - CY;
        return {CX + zoom * (c * dx - s * dy) + shift.first, CY + zoom * (s * dx + c * dy) + shift.second};
    }
};

Image img(std::size_t w, std::size_t h, const std::vector<std::uint8_t>& d) {
    return *Image::make(w, h, d);
}

std::vector<Descriptor> descs(const std::vector<std::pair<Keypoint, Descriptor>>& k) {
    std::vector<Descriptor> d;
    for (const auto& x : k) {
        d.push_back(x.second);
    }
    return d;
}

std::tuple<std::size_t, std::size_t, std::size_t, std::size_t> run_pair(const Pair& p, float tol) {
    const OrbParams params;
    const auto ka = extract(img(W, H, p.a), nullptr, params);
    const auto kb = extract(img(W, H, p.b), nullptr, params);
    const auto m = match_bf(descs(ka), descs(kb), 64, 0.8f);
    std::size_t correct = 0;
    for (const auto& [i, j, d] : m) {
        (void)d;
        const auto [ex, ey] = p.forward(ka[i].first.x, ka[i].first.y);
        if (std::hypot(kb[j].first.x - ex, kb[j].first.y - ey) <= tol) {
            correct++;
        }
    }
    return {ka.size(), kb.size(), m.size(), correct};
}

std::vector<std::uint8_t> textured(std::uint64_t seed) {
    return to_u8(canvas(W, H, seed));
}

std::uint32_t naive_hamming(const Descriptor& a, const Descriptor& b) {
    std::uint32_t n = 0;
    for (std::size_t i = 0; i < 32; i++) {
        for (int bit = 0; bit < 8; bit++) {
            n += ((a[i] >> bit) & 1) != ((b[i] >> bit) & 1);
        }
    }
    return n;
}

Descriptor nested(std::size_t k) {
    Descriptor d{};
    for (std::size_t bit = 0; bit < k; bit++) {
        d[bit / 8] |= static_cast<std::uint8_t>(1u << (bit % 8));
    }
    return d;
}

std::vector<Match> naive_match(const std::vector<Descriptor>& a, const std::vector<Descriptor>& b, std::uint32_t max_dist,
                               float ratio, const std::function<bool(std::size_t, std::size_t)>& allow) {
    std::vector<Match> out;
    for (std::size_t i = 0; i < a.size(); i++) {
        std::vector<std::pair<std::uint32_t, std::size_t>> cand;
        for (std::size_t j = 0; j < b.size(); j++) {
            if (allow(i, j)) {
                cand.emplace_back(hamming(a[i], b[j]), j);
            }
        }
        if (cand.empty()) {
            continue;
        }
        const auto [d1, j1] = *std::min_element(cand.begin(), cand.end());
        std::optional<std::uint32_t> second;
        for (const auto& c : cand) {
            if (c.second != j1 && (!second || c.first < *second)) {
                second = c.first;
            }
        }
        std::optional<std::pair<std::uint32_t, std::size_t>> back;
        for (std::size_t k = 0; k < a.size(); k++) {
            if (allow(k, j1)) {
                const std::pair<std::uint32_t, std::size_t> c{hamming(a[k], b[j1]), k};
                if (!back || c < *back) {
                    back = c;
                }
            }
        }
        const bool mutual = back && back->second == i;
        const bool ratio_ok = !second || static_cast<float>(d1) < ratio * static_cast<float>(*second);
        if (mutual && d1 <= max_dist && ratio_ok) {
            out.emplace_back(i, j1, d1);
        }
    }
    return out;
}

std::array<std::uint8_t, 16> ring_at(const Image& im, std::size_t x, std::size_t y) {
    static const int off[16][2] = {{0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3},
                                   {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3}};
    std::array<std::uint8_t, 16> r{};
    for (int k = 0; k < 16; k++) {
        r[k] = im.data[static_cast<std::size_t>(static_cast<long>(y) + off[k][1]) * im.stride +
                       static_cast<std::size_t>(static_cast<long>(x) + off[k][0])];
    }
    return r;
}

int naive_fast_score(const std::array<std::uint8_t, 16>& ring, std::uint8_t c) {
    int d[16];
    for (int k = 0; k < 16; k++) {
        d[k] = ring[k] - c;
    }
    int best = 0;
    for (int start = 0; start < 16; start++) {
        int mn = 1 << 20, mx = -(1 << 20);
        for (int k = 0; k < 9; k++) {
            mn = std::min(mn, d[(start + k) % 16]);
            mx = std::max(mx, d[(start + k) % 16]);
        }
        best = std::max(best, std::max(mn, -mx));
    }
    return best;
}

Cand cand(std::size_t x, std::size_t y, float resp, std::uint16_t cell, std::uint8_t level = 0) {
    return Cand{x, y, resp, level, cell};
}

std::vector<float> resp_sorted(const std::vector<Cand>& v) {
    std::vector<float> r;
    for (const auto& c : v) {
        r.push_back(c.resp);
    }
    std::sort(r.begin(), r.end(), [](float a, float b) { return a > b; });
    return r;
}

} // namespace

TEST(extract_fills_the_budget_inside_the_margins_and_spreads_over_the_grid) {
    const auto data = textured(1);
    const OrbParams p;
    const auto kps = extract(img(W, H, data), nullptr, p);
    const float n = static_cast<float>(p.n_features);
    std::printf("keypoints: %zu of %zu\n", kps.size(), p.n_features);
    CHECK(static_cast<float>(kps.size()) >= 0.8f * n && kps.size() <= p.n_features);
    std::vector<std::size_t> per_level(p.n_levels, 0);
    bool cells[48] = {};
    for (const auto& [k, d] : kps) {
        (void)d;
        CHECK(k.level < p.n_levels);
        per_level[k.level]++;
        CHECK(k.x >= BORDER && k.x <= static_cast<float>(W - 1 - BORDER));
        CHECK(k.y >= BORDER && k.y <= static_cast<float>(H - 1 - BORDER));
        CHECK(k.angle > -PI - 1e-6f && k.angle <= PI + 1e-6f);
        CHECK(std::isfinite(k.response));
        const std::size_t cx = std::min<std::size_t>(static_cast<std::size_t>(k.x / (W / 8.0f)), 7);
        const std::size_t cy = std::min<std::size_t>(static_cast<std::size_t>(k.y / (H / 6.0f)), 5);
        cells[cy * 8 + cx] = true;
    }
    const auto covered = std::count(std::begin(cells), std::end(cells), true);
    CHECK(static_cast<float>(covered) >= 0.7f * 48.0f);
    for (std::size_t l = 0; l < per_level.size(); l++) {
        CHECK(per_level[l] > 0);
        if (l > 0) {
            CHECK(per_level[l - 1] >= per_level[l]);
        }
    }
}

TEST(repeatable_under_rotation_shift_brightness_and_noise) {
    const Pair pair(2, 15.0f * PI / 180.0f, 1.0f, {7.3f, 7.3f}, 0.7f, 20.0f, 3.0f);
    const auto [na, nb, matches, correct] = run_pair(pair, 2.5f);
    std::printf("keypoints %zu/%zu, matches %zu, correct %zu\n", na, nb, matches, correct);
    CHECK(correct >= 100);
    CHECK(correct * 10 >= matches * 6);
}

TEST(identical_images_match_themselves_exactly) {
    const auto data = textured(3);
    const auto kps = extract(img(W, H, data), nullptr, OrbParams{});
    const auto d = descs(kps);
    const auto m = match_bf(d, d, 0, 0.8f);
    CHECK(m.size() + 5 >= d.size());
    for (const auto& [i, j, dist] : m) {
        CHECK(i == j && dist == 0);
    }
}

TEST(repeatable_across_a_scale_change) {
    const Pair pair(4, 20.0f * PI / 180.0f, 1.2f, {-5.1f, 3.7f}, 0.8f, 10.0f, 2.0f);
    const auto [na, nb, matches, correct] = run_pair(pair, 3.0f);
    std::printf("scale 1.2: keypoints %zu/%zu, matches %zu, correct %zu\n", na, nb, matches, correct);
    CHECK(correct >= 50);
    CHECK(correct * 10 >= matches * 7);
}

TEST(hamming_agrees_with_a_naive_bit_count) {
    Rng rng{10};
    for (int i = 0; i < 2000; i++) {
        const auto a = rng.descriptor();
        const auto b = rng.descriptor();
        CHECK(hamming(a, b) == naive_hamming(a, b));
    }
    const Descriptor zero{};
    Descriptor ones;
    ones.fill(0xFF);
    CHECK(hamming(zero, zero) == 0);
    CHECK(hamming(zero, ones) == 256);
    CHECK(hamming(ones, zero) == 256);
    Descriptor one_bit = zero;
    one_bit[31] = 0x80;
    CHECK(hamming(zero, one_bit) == 1);
    CHECK(hamming(one_bit, ones) == 255);
}

TEST(match_bf_applies_the_mutual_distance_and_ratio_rules) {
    const std::vector<Descriptor> a = {nested(0), nested(8), nested(100)};
    const std::vector<Descriptor> b = {nested(10), nested(12), nested(120)};
    CHECK((match_bf(a, b, 64, 0.8f) == std::vector<Match>{{1, 0, 2}, {2, 2, 20}}));
    CHECK((match_bf(a, b, 10, 0.8f) == std::vector<Match>{{1, 0, 2}}));
    CHECK((match_bf(a, b, 20, 0.8f) == std::vector<Match>{{1, 0, 2}, {2, 2, 20}}));
    CHECK(match_bf(a, b, 1, 0.8f).empty());
    const std::vector<Descriptor> one = {nested(0)};
    const std::vector<Descriptor> two = {nested(10), nested(12)};
    CHECK(match_bf(one, two, 64, 0.8f).empty());
    CHECK((match_bf(one, two, 64, 0.9f) == std::vector<Match>{{0, 0, 10}}));
    CHECK(match_bf(one, two, 64, 10.0f / 12.0f).empty());
    CHECK((match_bf(one, std::vector<Descriptor>{nested(10)}, 64, 0.5f) == std::vector<Match>{{0, 0, 10}}));
    CHECK(match_bf(one, std::vector<Descriptor>{nested(10), nested(10)}, 64, 0.99f).empty());
    CHECK(match_bf(one, std::vector<Descriptor>{nested(0), nested(0)}, 64, 0.8f).empty());
    CHECK(match_bf(std::vector<Descriptor>{}, two, 64, 0.8f).empty());
    CHECK(match_bf(one, std::vector<Descriptor>{}, 64, 0.8f).empty());
}

TEST(matching_agrees_with_a_naive_implementation_on_random_data_with_ties) {
    Rng rng{11};
    std::size_t total = 0;
    for (std::size_t trial = 0; trial < 12; trial++) {
        const std::size_t nb = 20 + trial * 7;
        std::vector<Descriptor> b;
        for (std::size_t i = 0; i < nb; i++) {
            b.push_back(rng.descriptor());
        }
        std::vector<Descriptor> a;
        for (std::size_t i = 0; i < 15 + trial * 5; i++) {
            Descriptor d = b[rng.next_u64() % nb];
            const std::uint64_t flips = rng.next_u64() % 60;
            for (std::uint64_t f = 0; f < flips; f++) {
                const std::size_t bit = rng.next_u64() % 256;
                d[bit / 8] ^= static_cast<std::uint8_t>(1u << (bit % 8));
            }
            a.push_back(d);
        }
        for (int i = 0; i < 5; i++) {
            a.push_back(rng.descriptor());
        }
        a.push_back(a[0]);
        a.push_back(b[1]);
        b.push_back(b[2]);
        b.push_back(a[3]);
        const std::pair<std::uint32_t, float> cases[] = {{64, 0.8f}, {100, 0.9f}, {256, 1.0f}, {30, 0.6f}, {256, 2.0f}, {0, 0.8f}};
        for (const auto& [max_dist, ratio] : cases) {
            const auto all = [](std::size_t, std::size_t) { return true; };
            const auto expect = naive_match(a, b, max_dist, ratio, all);
            total += expect.size();
            CHECK(match_bf(a, b, max_dist, ratio) == expect);
            const auto allow = [](std::size_t i, std::size_t j) { return (i * 7 + j * 3) % 5 < 4; };
            CHECK(match_gated(a, b, max_dist, ratio, allow) == naive_match(a, b, max_dist, ratio, allow));
        }
    }
    CHECK(total > 100);
}

TEST(match_gated_respects_the_predicate_and_asks_once_per_pair) {
    Rng rng{12};
    std::vector<Descriptor> b;
    for (int i = 0; i < 40; i++) {
        b.push_back(rng.descriptor());
    }
    const std::vector<Descriptor> a(b.begin(), b.begin() + 30);
    const std::size_t na = a.size(), nb = b.size();
    std::vector<int> calls(na * nb, 0);
    const auto same_parity = [&](std::size_t i, std::size_t j) {
        calls[i * nb + j]++;
        return i % 2 == j % 2;
    };
    auto m = match_gated(a, b, 0, 0.8f, same_parity);
    CHECK(std::all_of(calls.begin(), calls.end(), [](int c) { return c == 1; }));
    CHECK(m.size() == na);
    for (const auto& [i, j, d] : m) {
        CHECK(i == j && d == 0);
    }
    m = match_gated(a, b, 256, 1.0f, [](std::size_t i, std::size_t j) { return i != j; });
    for (const auto& [i, j, d] : m) {
        (void)d;
        CHECK(i != j);
    }
    const auto free = match_bf(a, b, 256, 1.0f);
    CHECK(free.size() == na);
    for (const auto& [i, j, d] : free) {
        (void)d;
        CHECK(i == j);
    }
    m = match_gated(a, b, 256, 0.5f, [&](std::size_t i, std::size_t j) { return j == (i + 1) % nb; });
    CHECK(m.size() == na);
    for (const auto& [i, j, d] : m) {
        (void)d;
        CHECK(j == i + 1);
    }
    CHECK(match_gated(a, b, 256, 1.0f, [](std::size_t, std::size_t) { return false; }).empty());
}

TEST(mask_excludes_locations) {
    const auto data = textured(5);
    const Image im = img(W, H, data);
    const OrbParams p;
    const Mask right_half = [](float x, float) { return x >= W / 2.0f; };
    auto kps = extract(im, &right_half, p);
    std::printf("left half masked: %zu keypoints\n", kps.size());
    for (const auto& [k, d] : kps) {
        (void)d;
        CHECK(k.x >= W / 2.0f);
    }
    CHECK(static_cast<float>(kps.size()) >= 0.8f * static_cast<float>(p.n_features));
    const float r = 200.0f;
    const Mask disc = [&](float x, float y) { return std::hypot(x - CX, y - CY) <= r; };
    kps = extract(im, &disc, p);
    CHECK(static_cast<float>(kps.size()) >= 0.8f * static_cast<float>(p.n_features));
    for (const auto& [k, d] : kps) {
        (void)d;
        CHECK(std::hypot(k.x - CX, k.y - CY) <= r);
    }
    const Mask none = [](float, float) { return false; };
    CHECK(extract(im, &none, p).empty());
    const Mask all = [](float, float) { return true; };
    CHECK(extract(im, &all, p) == extract(im, nullptr, p));
}

TEST(pyramid_samples_the_image_on_a_corner_aligned_grid) {
    std::vector<std::uint8_t> data;
    for (std::size_t y = 0; y < H; y++) {
        for (std::size_t x = 0; x < W; x++) {
            data.push_back(static_cast<std::uint8_t>(std::round(0.2f * static_cast<float>(x) + 0.1f * static_cast<float>(y))));
        }
    }
    const Image im = img(W, H, data);
    const Cfg cfg(OrbParams{});
    const auto geom = level_geometry(W, H, cfg);
    CHECK(geom.size() == 4);
    const auto bufs = build_pyramid(im, geom, cfg.scale);
    const auto levels = level_images(im, bufs, geom);
    CHECK(levels[1].w == 533 && levels[1].h == 400);
    for (std::size_t l = 0; l < levels.size(); l++) {
        const auto& li = levels[l];
        const auto& g = geom[l];
        CHECK(static_cast<double>(li.w - 1) * g.scale <= static_cast<double>(W - 1) + 1e-6);
        CHECK(static_cast<double>(li.w) * g.scale > static_cast<double>(W - 1));
        CHECK(static_cast<double>(li.h - 1) * g.scale <= static_cast<double>(H - 1) + 1e-6);
        CHECK(static_cast<double>(li.h) * g.scale > static_cast<double>(H - 1));
        const std::pair<std::size_t, std::size_t> pts[] = {
            {0, 0}, {li.w - 1, 0}, {0, li.h - 1}, {li.w - 1, li.h - 1}, {li.w / 2, li.h / 3}, {37, 91}};
        for (const auto& [x, y] : pts) {
            const double expect = 0.2 * (static_cast<double>(x) * g.scale) + 0.1 * (static_cast<double>(y) * g.scale);
            const double got = li.data[y * li.stride + x];
            CHECK(std::fabs(got - expect) <= 1.3);
        }
    }
}

TEST(intensity_centroid_points_towards_the_bright_side) {
    const std::size_t n = 61, cx = 30, cy = 30;
    for (float deg : {0.0f, 30.0f, 90.0f, 135.0f, 180.0f, -60.0f, -150.0f, 7.0f}) {
        const float phi = deg * PI / 180.0f;
        const float bx = cx + 7.0f * std::cos(phi), by = cy + 7.0f * std::sin(phi);
        std::vector<std::uint8_t> im(n * n);
        for (std::size_t i = 0; i < n * n; i++) {
            const float x = static_cast<float>(i % n), y = static_cast<float>(i / n);
            const float d2 = (x - bx) * (x - bx) + (y - by) * (y - by);
            im[i] = static_cast<std::uint8_t>(std::round(20.0f + 200.0f * std::exp(-d2 / 18.0f)));
        }
        const float a = ic_angle(im, n, cx, cy);
        const float diff = std::fabs(std::fmod(std::fmod(a - phi + PI, TAU) + TAU, TAU) - PI);
        CHECK(diff < 3.0f * PI / 180.0f);
    }
    std::vector<std::uint8_t> im(n * n);
    for (std::size_t i = 0; i < n * n; i++) {
        const float x = static_cast<float>(i % n), y = static_cast<float>(i / n);
        im[i] = static_cast<std::uint8_t>(
            std::round(60.0f + 100.0f * std::exp(-((x - 36.0f) * (x - 36.0f) + (y - 25.0f) * (y - 25.0f)) / 18.0f)));
    }
    const float a = ic_angle(im, n, cx, cy);
    CHECK(std::fabs(a - std::atan2(-5.0f, 6.0f)) < 3.0f * PI / 180.0f);
}

TEST(descriptors_are_covariant_with_exact_quarter_turns) {
    const std::size_t n = 101;
    auto cur = to_u8(canvas(n, n, 6));
    std::size_t x = 47, y = 52;
    auto sm = gaussian_blur(img(n, n, cur));
    const float a0 = ic_angle(sm, n, x, y);
    const auto d0 = describe(sm, n, x, y, a0);
    for (int quarter = 1; quarter < 4; quarter++) {
        std::vector<std::uint8_t> next(n * n);
        for (std::size_t yy = 0; yy < n; yy++) {
            for (std::size_t xx = 0; xx < n; xx++) {
                next[xx * n + (n - 1 - yy)] = cur[yy * n + xx];
            }
        }
        cur = next;
        const std::size_t nx = n - 1 - y, ny = x;
        x = nx;
        y = ny;
        sm = gaussian_blur(img(n, n, cur));
        const float a = ic_angle(sm, n, x, y);
        const float expect = a0 + static_cast<float>(quarter) * PI / 2.0f;
        const float diff = std::fabs(std::fmod(std::fmod(a - expect + PI, TAU) + TAU, TAU) - PI);
        CHECK(diff < 1e-3f);
        CHECK(hamming(d0, describe(sm, n, x, y, a)) <= 4);
    }
}

TEST(descriptors_separate_true_matches_from_different_places) {
    const Pair pair(13, 12.0f * PI / 180.0f, 1.0f, {3.4f, -2.2f}, 0.8f, 10.0f, 2.0f);
    const OrbParams params;
    const auto ka = extract(img(W, H, pair.a), nullptr, params);
    const auto kb = extract(img(W, H, pair.b), nullptr, params);
    std::uint64_t ones = 0;
    for (const auto& k : ka) {
        for (auto b : k.second) {
            ones += static_cast<std::uint64_t>(std::popcount(b));
        }
    }
    const double density = static_cast<double>(ones) / static_cast<double>(ka.size() * 256);
    std::uint64_t sum = 0, n = 0;
    for (std::size_t i = 0; i < ka.size(); i++) {
        for (std::size_t j = i + 1; j < ka.size(); j++) {
            sum += hamming(ka[i].second, ka[j].second);
            n++;
        }
    }
    const double unrelated = static_cast<double>(sum) / static_cast<double>(n);
    std::vector<std::uint32_t> dists;
    for (const auto& [a, da] : ka) {
        const auto [ex, ey] = pair.forward(a.x, a.y);
        std::optional<std::uint32_t> best;
        for (const auto& [b, db] : kb) {
            if (std::hypot(b.x - ex, b.y - ey) <= 1.5f && b.level == a.level) {
                const auto h = hamming(da, db);
                if (!best || h < *best) {
                    best = h;
                }
            }
        }
        if (best) {
            dists.push_back(*best);
        }
    }
    double rs = 0;
    for (auto d : dists) {
        rs += d;
    }
    const double related = rs / static_cast<double>(dists.size());
    std::printf("bit density %.3f; unrelated %.1f bits, true %.1f bits (%zu)\n", density, unrelated, related, dists.size());
    CHECK(density >= 0.35 && density < 0.65);
    CHECK(unrelated >= 60.0 && unrelated < 150.0);
    CHECK(dists.size() >= 100);
    CHECK(related < 0.55 * unrelated);
}

TEST(pattern_is_well_formed_and_pinned) {
    double sum = 0, sum2 = 0;
    for (const auto& p : pattern()) {
        for (auto v : p) {
            CHECK(v >= -15 && v <= 15);
        }
        const int dx = p[0] - p[2], dy = p[1] - p[3];
        CHECK(dx * dx + dy * dy >= MIN_PAIR_SEP2);
        for (auto v : p) {
            sum += v;
            sum2 += static_cast<double>(v) * v;
        }
    }
    const double n = 256.0 * 4.0;
    const double mean = sum / n;
    const double sd = std::sqrt(sum2 / n - mean * mean);
    CHECK(std::fabs(mean) < 0.7);
    CHECK(sd >= 5.0 && sd < 7.0);
    std::uint64_t h = 0xCBF29CE484222325ull;
    for (const auto& p : pattern()) {
        for (auto v : p) {
            h = (h ^ static_cast<std::uint8_t>(v)) * 0x100000001B3ull;
        }
    }
    CHECK(DESCRIPTOR_VERSION == 1);
    CHECK(h == 0x0645acfa93b082e8ull);
}

TEST(row_padding_is_ignored) {
    const auto data = textured(7);
    const std::size_t stride = W + 13;
    std::vector<std::uint8_t> padded(stride * H);
    Rng rng{99};
    for (std::size_t y = 0; y < H; y++) {
        std::copy(data.begin() + static_cast<long>(y * W), data.begin() + static_cast<long>((y + 1) * W),
                  padded.begin() + static_cast<long>(y * stride));
        for (std::size_t x = W; x < stride; x++) {
            padded[y * stride + x] = static_cast<std::uint8_t>(rng.next_u64() & 0xFF);
        }
    }
    const OrbParams p;
    const auto tight = extract(img(W, H, data), nullptr, p);
    CHECK(!tight.empty());
    const Image strided{W, H, stride, padded};
    CHECK(extract(strided, nullptr, p) == tight);
    const Image shortimg{W, H, stride, std::span(padded).subspan(0, stride * (H - 1) + W)};
    CHECK(shortimg.is_valid());
    CHECK(extract(shortimg, nullptr, p) == tight);
}

TEST(invalid_or_tiny_inputs_give_no_keypoints) {
    const OrbParams p;
    const auto data = textured(8);
    const std::span<const std::uint8_t> d(data);
    const Image bad[] = {
        Image{W, H, W, d.subspan(0, W * H - 1)}, Image{W, H, W - 1, d}, Image{0, H, 0, d}, Image{W, 0, W, d},
        Image{SIZE_MAX, 2, SIZE_MAX, d},
    };
    for (const auto& b : bad) {
        CHECK(!b.is_valid());
        CHECK(extract(b, nullptr, p).empty());
    }
    CHECK(!Image::make(W, H, d.subspan(0, 10)));
    CHECK(Image::make(W, H, d).has_value());
    const std::vector<std::uint8_t> tiny(50 * 50, 128);
    CHECK(extract(img(50, 50, tiny), nullptr, p).empty());
    const std::vector<std::uint8_t> flat(W * H, 128);
    CHECK(extract(img(W, H, flat), nullptr, p).empty());
    OrbParams zero;
    zero.n_features = 0;
    CHECK(extract(img(W, H, data), nullptr, zero).empty());
}

TEST(odd_sizes_extreme_pixels_and_random_parameters_never_panic) {
    Rng rng{77};
    const std::pair<std::size_t, std::size_t> sizes[] = {{1, 1},   {1, 500},  {500, 1},  {7, 7},   {43, 43},
                                                         {59, 61}, {60, 60},  {61, 61},  {65, 300}, {300, 65},
                                                         {131, 97}, {254, 255}, {641, 479}};
    std::size_t total = 0;
    for (const auto& [w, h] : sizes) {
        for (int variant = 0; variant < 4; variant++) {
            std::vector<std::uint8_t> data(w * h);
            if (variant == 0) {
                for (auto& v : data) {
                    v = static_cast<std::uint8_t>(rng.next_u64());
                }
            } else if (variant == 1) {
                for (auto& v : data) {
                    v = (rng.next_u64() & 1) == 0 ? 0 : 255;
                }
            } else if (variant == 2) {
                for (std::size_t i = 0; i < w * h; i++) {
                    data[i] = ((i % w / 2 + i / w / 2) & 1) == 0 ? 0 : 255;
                }
            } else if (w >= 8 && h >= 8) {
                data = to_u8(canvas(w, h, rng.next_u64()));
            } else {
                data.assign(w * h, 255);
            }
            OrbParams p;
            p.n_features = rng.next_u64() % 1500;
            p.n_levels = rng.next_u64() % 12;
            p.scale = rng.range(0.5f, 3.0f);
            p.fast_threshold = static_cast<std::uint8_t>(rng.next_u64() % 60);
            p.min_fast_threshold = static_cast<std::uint8_t>(rng.next_u64() % 60);
            p.grid_cols = rng.next_u64() % 20;
            p.grid_rows = rng.next_u64() % 20;
            const Mask stripes = [](float x, float y) {
                return ((static_cast<std::uint32_t>(x) + static_cast<std::uint32_t>(y)) / 7 & 1) == 0;
            };
            const Mask* mask = variant % 2 == 0 ? &stripes : nullptr;
            const auto kps = extract(img(w, h, data), mask, p);
            total += kps.size();
            CHECK(kps.size() <= p.n_features);
            for (const auto& [k, d] : kps) {
                (void)d;
                CHECK(k.x >= BORDER && k.x <= static_cast<float>(w - 1 - BORDER));
                CHECK(k.y >= BORDER && k.y <= static_cast<float>(h - 1 - BORDER));
                CHECK(std::isfinite(k.angle) && std::isfinite(k.response));
                CHECK(k.level < MAX_LEVELS);
                CHECK(mask == nullptr || (*mask)(k.x, k.y));
            }
        }
    }
    CHECK(total > 500);
}

TEST(extraction_is_deterministic) {
    const auto data = textured(14);
    CHECK(extract(img(W, H, data), nullptr, OrbParams{}) == extract(img(W, H, data), nullptr, OrbParams{}));
}

TEST(absurd_parameters_are_sanitised) {
    const auto data = textured(9);
    const Image im = img(W, H, data);
    OrbParams weird;
    weird.n_features = 100;
    weird.n_levels = 0;
    weird.scale = NAN;
    weird.fast_threshold = 5;
    weird.min_fast_threshold = 50;
    weird.grid_cols = 0;
    weird.grid_rows = 0;
    auto k = extract(im, nullptr, weird);
    CHECK(!k.empty() && k.size() <= 100);
    for (const auto& x : k) {
        CHECK(x.first.level == 0);
    }
    OrbParams many;
    many.n_levels = 100;
    many.scale = 1.5f;
    k = extract(im, nullptr, many);
    std::uint8_t top = 0;
    for (const auto& x : k) {
        top = std::max(top, x.first.level);
    }
    CHECK(top == 5);
    CHECK(static_cast<float>(k.size()) >= 0.8f * 500.0f);
    OrbParams huge;
    huge.scale = 1000.0f;
    k = extract(im, nullptr, huge);
    for (const auto& x : k) {
        CHECK(x.first.level <= 1);
    }
    CHECK(huge.level_scale(1) == 8.0f);
    OrbParams one;
    one.n_levels = 1;
    k = extract(im, nullptr, one);
    CHECK(static_cast<float>(k.size()) >= 0.8f * 500.0f);
    for (const auto& x : k) {
        CHECK(x.first.level == 0);
    }
}

TEST(keypoints_are_ordered_spaced_and_on_the_level_grid) {
    const auto data = textured(15);
    const OrbParams p;
    const auto kps = extract(img(W, H, data), nullptr, p);
    for (std::size_t i = 1; i < kps.size(); i++) {
        CHECK(kps[i - 1].first.level <= kps[i].first.level);
    }
    for (std::uint8_t l = 0; l < p.n_levels; l++) {
        const float s = p.level_scale(l);
        std::vector<std::pair<float, float>> pts;
        for (const auto& [k, d] : kps) {
            (void)d;
            if (k.level == l) {
                pts.emplace_back(k.x / s, k.y / s);
            }
        }
        for (std::size_t i = 1; i < pts.size(); i++) {
            CHECK(std::pair(pts[i - 1].second, pts[i - 1].first) < std::pair(pts[i].second, pts[i].first));
        }
        for (const auto& [x, y] : pts) {
            CHECK(std::fabs(x - std::round(x)) < 2e-3f && std::fabs(y - std::round(y)) < 2e-3f);
        }
        for (std::size_t i = 0; i < pts.size(); i++) {
            for (std::size_t j = i + 1; j < pts.size(); j++) {
                const float dx = pts[i].first - pts[j].first, dy = pts[i].second - pts[j].second;
                CHECK(dx * dx + dy * dy >= MIN_KP_DIST2 - 0.1f);
            }
        }
    }
}

TEST(level_budgets_are_proportional_to_the_area_and_exact) {
    const Cfg cfg(OrbParams{});
    const auto geom = level_geometry(W, H, cfg);
    double total = 0;
    for (const auto& g : geom) {
        total += static_cast<double>(g.w * g.h);
    }
    for (std::size_t n : {1u, 7u, 100u, 500u, 1000u, 1234u}) {
        const auto b = level_budgets(n, geom);
        std::size_t s = 0;
        for (auto x : b) {
            s += x;
        }
        CHECK(s == n);
        for (std::size_t i = 0; i < b.size(); i++) {
            const double exact = static_cast<double>(n) * static_cast<double>(geom[i].w * geom[i].h) / total;
            CHECK(std::fabs(static_cast<double>(b[i]) - exact) < 1.0);
        }
    }
    CHECK((level_budgets(500, geom) == std::vector<std::size_t>{199, 138, 96, 67}));
}

TEST(ring_score_matches_the_textbook_definition) {
    Rng rng{21};
    std::size_t corners = 0;
    for (int n = 0; n < 200000; n++) {
        const auto c = static_cast<std::uint8_t>(rng.next_u64() & 0xFF);
        std::array<std::uint8_t, 16> ring{};
        for (auto& p : ring) {
            p = static_cast<std::uint8_t>(rng.next_u64() & 0xFF);
        }
        if (n % 2 == 0) {
            const std::size_t start = rng.next_u64() % 16;
            const std::size_t len = 7 + rng.next_u64() % 5;
            const int sign = (rng.next_u64() & 1) == 0 ? 1 : -1;
            const int margin = static_cast<int>(rng.next_u64() % 60) + 1;
            for (std::size_t k = 0; k < len; k++) {
                const int jitter = static_cast<int>(rng.next_u64() % 9) - 4;
                ring[(start + k) % 16] = static_cast<std::uint8_t>(std::clamp(c + sign * margin + jitter, 0, 255));
            }
        }
        const auto thr = static_cast<std::uint8_t>(rng.next_u64() % 64);
        const int naive = naive_fast_score(ring, c);
        const int expect = naive > thr ? naive : 0;
        CHECK(ring_score(ring, c, thr) == expect);
        corners += expect > 0;
    }
    CHECK(corners > 10000);
    std::array<std::uint8_t, 16> ring;
    ring.fill(100);
    for (int k = 0; k < 8; k++) {
        ring[(12 + k) % 16] = 160;
    }
    CHECK(ring_score(ring, 100, 20) == 0);
    ring[(12 + 8) % 16] = 140;
    CHECK(ring_score(ring, 100, 20) == 40);
    CHECK(ring_score(ring, 100, 39) == 40);
    CHECK(ring_score(ring, 100, 40) == 0);
    ring.fill(200);
    for (int k = 0; k < 9; k++) {
        ring[k] = 120;
    }
    CHECK(ring_score(ring, 200, 7) == 80);
}

TEST(fast_cell_reports_exactly_the_local_maxima_of_the_score_map) {
    const auto data = textured(16);
    const Image im = img(W, H, data);
    const Rect rect{100, 90, 180, 150};
    for (std::uint8_t thr : {std::uint8_t{20}, std::uint8_t{7}}) {
        std::vector<Corner> found;
        std::vector<std::uint8_t> scratch;
        fast_cell(im, rect, thr, scratch, found);
        auto score = [&](std::size_t x, std::size_t y) -> int {
            const int s = naive_fast_score(ring_at(im, x, y), im.data[y * im.stride + x]);
            return s > thr ? std::min(s, 255) : 0;
        };
        std::vector<std::tuple<std::size_t, std::size_t, int>> expect;
        for (std::size_t y = rect.y0; y < rect.y1; y++) {
            for (std::size_t x = rect.x0; x < rect.x1; x++) {
                const int s = score(x, y);
                const bool earlier = score(x - 1, y - 1) < s && score(x, y - 1) < s && score(x + 1, y - 1) < s &&
                                     score(x - 1, y) < s;
                const bool later = score(x + 1, y) <= s && score(x - 1, y + 1) <= s && score(x, y + 1) <= s &&
                                   score(x + 1, y + 1) <= s;
                if (s > 0 && earlier && later) {
                    expect.emplace_back(x, y, s);
                }
            }
        }
        std::vector<std::tuple<std::size_t, std::size_t, int>> got;
        for (const auto& c : found) {
            got.emplace_back(c.x, c.y, c.score);
        }
        CHECK(got.size() > 20);
        CHECK(got == expect);
        for (std::size_t i = 0; i < got.size(); i++) {
            for (std::size_t j = i + 1; j < got.size(); j++) {
                const auto dx = std::get<0>(got[i]) > std::get<0>(got[j]) ? std::get<0>(got[i]) - std::get<0>(got[j])
                                                                         : std::get<0>(got[j]) - std::get<0>(got[i]);
                const auto dy = std::get<1>(got[i]) > std::get<1>(got[j]) ? std::get<1>(got[i]) - std::get<1>(got[j])
                                                                         : std::get<1>(got[j]) - std::get<1>(got[i]);
                CHECK(dx > 1 || dy > 1);
            }
        }
    }
}

TEST(harris_prefers_corners_to_edges_and_is_invariant_to_quarter_turns) {
    const std::size_t n = 41;
    std::vector<std::uint8_t> quadrant(n * n), edge(n * n), flat(n * n, 90);
    for (std::size_t i = 0; i < n * n; i++) {
        quadrant[i] = (i % n >= 20 && i / n >= 20) ? 220 : 30;
        edge[i] = i % n >= 20 ? 220 : 30;
    }
    auto at = [&](const std::vector<std::uint8_t>& d) { return harris(img(n, n, d), 20, 20); };
    const float corner = at(quadrant), along_edge = at(edge), nothing = at(flat);
    CHECK(corner > 0.0f);
    CHECK(along_edge < 0.0f);
    CHECK(nothing == 0.0f);
    std::vector<std::uint8_t> strong(n * n);
    for (std::size_t i = 0; i < n * n; i++) {
        strong[i] = quadrant[i] > 100 ? 255 : 0;
    }
    const float r4 = std::pow(255.0f / 190.0f, 4.0f);
    CHECK(std::fabs(at(strong) / corner - r4) < 0.05f * r4);
    std::vector<std::uint8_t> turned(n * n);
    for (std::size_t y = 0; y < n; y++) {
        for (std::size_t x = 0; x < n; x++) {
            turned[x * n + (n - 1 - y)] = quadrant[y * n + x];
        }
    }
    const float t = at(turned);
    CHECK(std::fabs(t - corner) <= 1e-6f * std::fabs(corner));
}

TEST(gaussian_blur_keeps_constants_spreads_an_impulse_symmetrically_and_respects_the_stride) {
    const std::size_t w = 40, h = 30;
    const std::vector<std::uint8_t> flat(w * h, 137);
    const auto fb = gaussian_blur(img(w, h, flat));
    CHECK(std::all_of(fb.begin(), fb.end(), [](std::uint8_t p) { return p == 137; }));
    std::vector<std::uint8_t> impulse(w * h, 0);
    impulse[15 * w + 20] = 255;
    const auto out = gaussian_blur(img(w, h, impulse));
    auto at = [&](std::size_t x, std::size_t y) { return static_cast<int>(out[y * w + x]); };
    for (std::size_t d = 1; d <= 3; d++) {
        CHECK(at(20 + d, 15) == at(20 - d, 15));
        CHECK(at(20, 15 + d) == at(20, 15 - d));
        CHECK(at(20 + d, 15) == at(20, 15 + d));
        CHECK(at(20 + d, 15) < at(20 + d - 1, 15));
    }
    CHECK(at(24, 15) == 0);
    int sum = 0;
    for (auto p : out) {
        sum += p;
    }
    CHECK(sum >= 255 - 24 && sum <= 255 + 24);
    const auto textured_ = textured(18);
    const std::size_t tw = 64, th = 48, stride = 64 + 5;
    std::vector<std::uint8_t> padded(stride * th, 0xA5), tight;
    for (std::size_t y = 0; y < th; y++) {
        std::copy(textured_.begin() + static_cast<long>(y * W), textured_.begin() + static_cast<long>(y * W + tw),
                  padded.begin() + static_cast<long>(y * stride));
        tight.insert(tight.end(), textured_.begin() + static_cast<long>(y * W), textured_.begin() + static_cast<long>(y * W + tw));
    }
    CHECK(gaussian_blur(img(tw, th, tight)) == gaussian_blur(Image{tw, th, stride, padded}));
}

TEST(selection_takes_the_best_of_every_cell_first_then_the_strongest) {
    const std::vector<Cand> cands = {cand(30, 30, 9.0f, 0), cand(60, 30, 8.0f, 0), cand(90, 30, 7.0f, 0),
                                     cand(30, 60, 1.0f, 1), cand(60, 60, 4.0f, 2)};
    auto [picked, rest] = select_level(cands, 3, 3);
    CHECK((resp_sorted(picked) == std::vector<float>{9.0f, 4.0f, 1.0f}));
    CHECK((resp_sorted(rest) == std::vector<float>{8.0f, 7.0f}));
    CHECK((resp_sorted(select_level(cands, 3, 5).first) == std::vector<float>{9.0f, 8.0f, 7.0f, 4.0f, 1.0f}));
    CHECK((resp_sorted(select_level(cands, 3, 4).first) == std::vector<float>{9.0f, 8.0f, 4.0f, 1.0f}));
    CHECK((resp_sorted(select_level(cands, 3, 2).first) == std::vector<float>{9.0f, 4.0f}));
    CHECK((resp_sorted(select_level(cands, 3, 1).first) == std::vector<float>{9.0f}));
    CHECK(select_level(cands, 3, 0).first.empty());
    const std::vector<Cand> close = {cand(50, 50, 5.0f, 0), cand(52, 50, 4.0f, 0), cand(50, 53, 3.0f, 0),
                                     cand(60, 60, 2.0f, 1)};
    auto [p2, r2] = select_level(close, 2, 10);
    CHECK((resp_sorted(p2) == std::vector<float>{5.0f, 3.0f, 2.0f}));
    CHECK((resp_sorted(r2) == std::vector<float>{4.0f}));
    const std::vector<Cand> tied = {cand(40, 40, 1.0f, 0), cand(30, 30, 1.0f, 0), cand(30, 40, 1.0f, 0)};
    const auto t1 = select_level(tied, 1, 1).first;
    std::vector<Cand> shuffled(tied.rbegin(), tied.rend());
    const auto t2 = select_level(shuffled, 1, 1).first;
    CHECK(t1[0].x == 30 && t1[0].y == 30);
    CHECK(t2[0].x == 30 && t2[0].y == 30);
}

TEST(short_levels_are_topped_up_with_the_strongest_unused_candidates) {
    auto at_level = [](std::uint8_t level, std::size_t x, float resp) { return Cand{x, 40, resp, level, 0}; };
    std::vector<std::vector<Cand>> picked = {{at_level(0, 30, 9.0f), at_level(0, 60, 8.0f)}, {at_level(1, 30, 3.0f)}, {}};
    const std::vector<Cand> unused = {at_level(0, 90, 6.0f), at_level(1, 60, 5.0f), at_level(1, 31, 5.5f),
                                      at_level(0, 120, 2.0f)};
    top_up(picked, unused, 5);
    CHECK(picked[0].size() == 3 && picked[1].size() == 2 && picked[2].empty());
    CHECK(std::any_of(picked[0].begin(), picked[0].end(), [](const Cand& c) { return c.resp == 6.0f; }));
    CHECK(std::any_of(picked[1].begin(), picked[1].end(), [](const Cand& c) { return c.resp == 5.0f; }));
    std::vector<std::vector<Cand>> full = {{at_level(0, 30, 9.0f)}, {}};
    top_up(full, unused, 1);
    CHECK(full[0].size() + full[1].size() == 1);
    std::vector<std::vector<Cand>> greedy(3);
    top_up(greedy, unused, 100);
    CHECK(greedy[0].size() == 2 && greedy[1].size() == 2 && greedy[2].empty());
}

TEST(a_large_request_is_met_by_redistributing_between_levels) {
    const auto data = textured(19);
    OrbParams p;
    p.n_features = 4000;
    const auto kps = extract(img(W, H, data), nullptr, p);
    std::printf("request 4000: %zu keypoints\n", kps.size());
    CHECK(kps.size() <= 4000);
    CHECK(kps.size() >= 3200);
}

PIBRIDGE_TEST_MAIN()
