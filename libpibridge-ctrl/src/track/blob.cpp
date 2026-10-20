// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/blob.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/blob.h"

#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace pibridge::ctrl::track {

BlobParams BlobParams::live() {
    BlobParams p;
    p.threshold = 80;
    return p;
}

BlobParams BlobParams::improved() {
    BlobParams p = live();
    p.core = true;
    p.dim = true;
    return p;
}

BlobParams BlobParams::from_args(std::span<const std::string> a) {
    BlobParams bp = live();
    const auto get = [&](std::string_view k) -> std::optional<std::uint8_t> {
        for (std::size_t i = 0; i < a.size(); i++) {
            if (a[i] == k) {
                if (i + 1 >= a.size()) {
                    return std::nullopt;
                }
                std::uint32_t v = 0;
                if (!cam::rfmt::parse_u32(a[i + 1], v) || v > 255) {
                    return std::nullopt;
                }
                return static_cast<std::uint8_t>(v);
            }
        }
        return std::nullopt;
    };
    if (const auto v = get("--thr")) {
        bp.threshold = *v;
    }
    if (const auto v = get("--peak")) {
        bp.required_peak = *v;
    }
    return bp;
}

namespace {

std::uint32_t find(std::vector<std::uint32_t>& p, std::uint32_t a) {
    while (p[a] != a) {
        p[a] = p[p[a]];
        a = p[a];
    }
    return a;
}

struct Acc {
    double sw = 0;
    double sx = 0;
    double sy = 0;
    std::uint32_t n = 0;
    std::uint8_t peak = 0;
    std::size_t x0 = std::numeric_limits<std::size_t>::max();
    std::size_t x1 = 0;
    std::size_t y0 = std::numeric_limits<std::size_t>::max();
    std::size_t y1 = 0;
};


/// PiBridge's core measurement (BlobParams::core): each component that passes the area / peak tests at the threshold
/// is re-cut at max(threshold, core_frac * peak); every 8-connected core becomes a blob (centroid weighted over the
/// core above its cut), and the side / area limits apply to the cores.
void detect_cores(std::span<const std::uint8_t> img, std::size_t w, std::size_t stride, const BlobParams& prm,
                  const std::vector<std::uint32_t>& labels, std::vector<std::uint32_t>& parent,
                  const std::vector<Acc>& acc, std::vector<Blob>& out) {
    std::vector<std::int32_t> local;
    std::vector<std::size_t> stack;
    for (std::size_t r = 0; r < acc.size(); r++) {
        const Acc& a = acc[r];
        if (a.n == 0 || a.n < prm.min_area || a.peak < prm.required_peak) {
            continue;
        }
        const double cut_d = std::max(static_cast<double>(prm.threshold), std::ceil(prm.core_frac * a.peak));
        const auto cut = static_cast<std::uint8_t>(std::min(cut_d, 255.0));
        const std::size_t bw = a.x1 - a.x0 + 1;
        const std::size_t bh = a.y1 - a.y0 + 1;
        // a halo bigger than 4x the side limit is a lamp or a window, whatever its core
        if (bw > 4u * prm.max_side || bh > 4u * prm.max_side) {
            continue;
        }
        local.assign(bw * bh, -1);
        const std::size_t out0 = out.size();
        const auto inside = [&](std::size_t x, std::size_t y) {
            const std::uint32_t l = labels[y * w + x];
            return l != 0 && find(parent, l) == r && img[y * stride + x] >= cut;
        };
        std::int32_t ncore = 0;
        for (std::size_t y = a.y0; y <= a.y1; y++) {
            for (std::size_t x = a.x0; x <= a.x1; x++) {
                const std::size_t li = (y - a.y0) * bw + (x - a.x0);
                if (local[li] >= 0 || !inside(x, y)) {
                    continue;
                }
                // flood fill one core
                Acc c;
                const std::int32_t id = ncore++;
                local[li] = id;
                stack.assign(1, li);
                const double bg = static_cast<double>(cut) - 1.0;
                while (!stack.empty()) {
                    const std::size_t k = stack.back();
                    stack.pop_back();
                    const std::size_t px = a.x0 + k % bw;
                    const std::size_t py = a.y0 + k / bw;
                    const std::uint8_t v = img[py * stride + px];
                    const double wt = static_cast<double>(v) - bg;
                    c.sw += wt;
                    c.sx += wt * static_cast<double>(px);
                    c.sy += wt * static_cast<double>(py);
                    c.n += 1;
                    c.peak = std::max(c.peak, v);
                    c.x0 = std::min(c.x0, px);
                    c.x1 = std::max(c.x1, px);
                    c.y0 = std::min(c.y0, py);
                    c.y1 = std::max(c.y1, py);
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            const long nx = static_cast<long>(px) + dx;
                            const long ny = static_cast<long>(py) + dy;
                            if (nx < static_cast<long>(a.x0) || ny < static_cast<long>(a.y0) ||
                                nx > static_cast<long>(a.x1) || ny > static_cast<long>(a.y1)) {
                                continue;
                            }
                            const std::size_t nk = (static_cast<std::size_t>(ny) - a.y0) * bw +
                                                   (static_cast<std::size_t>(nx) - a.x0);
                            if (local[nk] < 0 && inside(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny))) {
                                local[nk] = id;
                                stack.push_back(nk);
                            }
                        }
                    }
                }
                const auto cw = static_cast<std::uint16_t>(c.x1 - c.x0 + 1);
                const auto ch = static_cast<std::uint16_t>(c.y1 - c.y0 + 1);
                if (c.n <= prm.max_area && cw <= prm.max_side && ch <= prm.max_side && c.peak >= prm.required_peak) {
                    // the area reported is the whole component's when it has one core (the LED's size), else
                    // the core's
                    out.push_back(Blob{c.sx / c.sw, c.sy / c.sw, c.n, c.peak, cw, ch});
                }
            }
        }
        if (ncore == 1 && out.size() == out0 + 1) {
            out.back().area = a.n;
        }
    }
}

} // namespace

std::vector<Blob> detect(std::span<const std::uint8_t> img, std::size_t w, std::size_t h, std::size_t stride,
                         const BlobParams& prm_in) {
    BlobParams prm = prm_in;
    if (prm.dim) {
        std::uint8_t m = 0;
        for (std::size_t y = 0; y < h; y++) {
            for (std::size_t x = 0; x < w; x++) {
                m = std::max(m, img[y * stride + x]);
            }
        }
        if (m < prm.required_peak && m >= prm.dim_min) {
            const double k = static_cast<double>(m) / 255.0;
            prm.threshold = static_cast<std::uint8_t>(std::max(1.0, std::round(prm.threshold * k)));
            prm.required_peak = static_cast<std::uint8_t>(std::max(1.0, std::round(prm.required_peak * k)));
        }
    }
    std::vector<std::uint32_t> labels(w * h, 0);
    std::vector<std::uint32_t> parent{0};
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            if (img[y * stride + x] < prm.threshold) {
                continue;
            }
            std::uint32_t nb[4] = {0, 0, 0, 0};
            std::size_t n = 0;
            if (x > 0 && labels[y * w + x - 1] != 0) {
                nb[n++] = labels[y * w + x - 1];
            }
            if (y > 0) {
                for (const long dx : {-1L, 0L, 1L}) {
                    const long xx = static_cast<long>(x) + dx;
                    if (xx >= 0 && static_cast<std::size_t>(xx) < w) {
                        const std::uint32_t l = labels[(y - 1) * w + static_cast<std::size_t>(xx)];
                        if (l != 0) {
                            nb[n++] = l;
                        }
                    }
                }
            }
            if (n == 0) {
                const auto l = static_cast<std::uint32_t>(parent.size());
                parent.push_back(l);
                labels[y * w + x] = l;
            } else {
                std::uint32_t m = find(parent, nb[0]);
                for (std::size_t i = 1; i < n; i++) {
                    const std::uint32_t r = find(parent, nb[i]);
                    if (r != m) {
                        const std::uint32_t lo = r < m ? r : m;
                        const std::uint32_t hi = r < m ? m : r;
                        parent[hi] = lo;
                        m = lo;
                    }
                }
                labels[y * w + x] = m;
            }
        }
    }
    std::vector<Acc> acc(parent.size());
    const double bg = static_cast<double>(prm.threshold) - 1.0;
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const std::uint32_t l = labels[y * w + x];
            if (l == 0) {
                continue;
            }
            const std::uint32_t r = find(parent, l);
            const std::uint8_t v = img[y * stride + x];
            Acc& a = acc[r];
            const double wt = static_cast<double>(v) - bg;
            a.sw += wt;
            a.sx += wt * static_cast<double>(x);
            a.sy += wt * static_cast<double>(y);
            a.n += 1;
            a.peak = std::max(a.peak, v);
            a.x0 = std::min(a.x0, x);
            a.x1 = std::max(a.x1, x);
            a.y0 = std::min(a.y0, y);
            a.y1 = std::max(a.y1, y);
        }
    }
    std::vector<Blob> out;
    if (prm.core) {
        detect_cores(img, w, stride, prm, labels, parent, acc, out);
        return out;
    }
    for (const Acc& a : acc) {
        if (a.n == 0) {
            continue;
        }
        const auto bw = static_cast<std::uint16_t>(a.x1 - a.x0 + 1);
        const auto bh = static_cast<std::uint16_t>(a.y1 - a.y0 + 1);
        if (a.n >= prm.min_area && a.n <= prm.max_area && a.peak >= prm.required_peak && bw <= prm.max_side &&
            bh <= prm.max_side) {
            out.push_back(Blob{a.sx / a.sw, a.sy / a.sw, a.n, a.peak, bw, bh});
        }
    }
    return out;
}

} // namespace pibridge::ctrl::track
