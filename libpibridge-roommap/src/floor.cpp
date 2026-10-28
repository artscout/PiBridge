// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/floor.rs (MIT OR Apache-2.0). See floor.h.
#include "pibridge/roommap/floor.h"

#include "pibridge/anchor/backend.h"
#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace pibridge::roommap::floor {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;

namespace {

double median(std::vector<double>& v) {
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::stable_sort(v.begin(), v.end(), rs::total_less);
    const std::size_t n = v.size();
    if (n % 2 == 1) {
        return v[n / 2];
    }
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double quantile(std::vector<double>& v, double q) {
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::stable_sort(v.begin(), v.end(), rs::total_less);
    return v[rs::to_usize(std::round(static_cast<double>(v.size() - 1) * q))];
}

struct Lcg {
    std::uint64_t s;
    std::size_t next(std::size_t n) {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<std::size_t>(s >> 33) % std::max<std::size_t>(n, 1);
    }
};

struct Plane {
    double a, b, c;
};

std::optional<Plane> plane3(const std::array<Vec3, 3>& p) {
    const Vec3 n = g::cross(g::sub(p[1], p[0]), g::sub(p[2], p[0]));
    if (std::fabs(n[1]) < 1e-9 || g::norm(n) < 1e-6) {
        return std::nullopt;
    }
    const double a = -n[0] / n[1], b = -n[2] / n[1];
    return Plane{a, b, p[0][1] - a * p[0][0] - b * p[0][2]};
}

std::optional<Plane> plane_ls(const std::vector<Vec3>& pts, std::array<double, 2> c0) {
    double sxx = 0.0, sxz = 0.0, szz = 0.0, sx = 0.0, sz = 0.0, sxy = 0.0, szy = 0.0, sy = 0.0;
    const double n = static_cast<double>(pts.size());
    for (const auto& p : pts) {
        const double x = p[0] - c0[0], z = p[2] - c0[1], y = p[1];
        sxx += x * x;
        sxz += x * z;
        szz += z * z;
        sx += x;
        sz += z;
        sxy += x * y;
        szy += z * y;
        sy += y;
    }
    const g::Mat3 m{{{sxx, sxz, sx}, {sxz, szz, sz}, {sx, sz, n}}};
    const double d = g::det(m);
    if (std::fabs(d) < 1e-12) {
        return std::nullopt;
    }
    auto solve = [&](int col) {
        g::Mat3 k = m;
        const double r[3] = {sxy, szy, sy};
        for (int i = 0; i < 3; i++) {
            k[i][col] = r[i];
        }
        return g::det(k) / d;
    };
    const double a = solve(0), b = solve(1), c = solve(2);
    return Plane{a, b, c - a * c0[0] - b * c0[1]};
}

double tilt_deg(double a, double b) {
    return rf::to_degrees(std::atan(std::sqrt(a * a + b * b)));
}

double confidence(const Layer& l, double above, const std::vector<Layer>& all) {
    auto s = [](double x, double half) { return x / (x + half); };
    const double support = s(static_cast<double>(l.inliers), 150.0);
    const double extent = s(l.area_m2, 1.0);
    const double flat = l.tilt_deg < 2.0 ? 1.0 : 0.7;
    const double noise = l.spread_m < 0.02 ? 1.0 : 0.02 / l.spread_m;
    const double head = (above >= 0.95 && above <= 2.0) ? 1.0 : 0.6;
    bool doubt_b = false;
    for (const auto& o : all) {
        if (o.y < l.y && o.inliers > l.inliers / 2) {
            doubt_b = true;
            break;
        }
    }
    const double doubt = doubt_b ? 0.8 : 1.0;
    return rf::fmin((support + extent) / 1.6, 1.0) * flat * noise * head * doubt;
}

std::vector<double> col(std::span<const Vec3> v, int k) {
    std::vector<double> x;
    x.reserve(v.size());
    for (const auto& p : v) {
        x.push_back(p[k]);
    }
    return x;
}

} // namespace

std::vector<Layer> layers(std::span<const Vec3> pts, std::array<double, 2> at, const FloorParams& p) {
    (void)at;
    std::vector<Layer> out;
    if (pts.size() < p.min_support) {
        return out;
    }
    std::vector<double> ys = col(pts, 1);
    const double lo = quantile(ys, 0.0);
    const double hi = quantile(ys, 1.0);
    const std::size_t nb = std::min<std::size_t>(rs::to_usize(std::ceil((hi - lo) / p.bin_m)) + 1, 100000);
    std::vector<double> h(nb + 4, 0.0);
    for (const auto& q : pts) {
        const std::size_t i = rs::to_usize((q[1] - lo) / p.bin_m) + 2;
        if (i >= h.size()) {
            rs::panic("omarchyvr-roommap/src/floor.rs", "index out of bounds");
        }
        h[i] += 1.0;
    }
    const double k[5] = {1.0, 2.0, 3.0, 2.0, 1.0};
    std::vector<double> hs(h.size());
    for (std::size_t i = 0; i < h.size(); i++) {
        double s = -0.0;
        for (std::size_t j = 0; j < 5; j++) {
            s += (i + j >= 2 && i + j - 2 < h.size()) ? h[i + j - 2] * k[j] : 0.0;
        }
        hs[i] = s / 9.0;
    }
    const std::size_t w = rs::to_usize(rf::fmax(std::round(p.band_m / p.bin_m), 1.0));
    std::vector<std::size_t> peaks;
    for (std::size_t i = 0; i < hs.size(); i++) {
        if (!(hs[i] > 0.0)) {
            continue;
        }
        const std::size_t a = i > w ? i - w : 0;
        const std::size_t b = std::min(i + w, hs.size() - 1);
        bool ok = true;
        for (std::size_t j = a; j <= b; j++) {
            if (!(hs[j] < hs[i] || (hs[j] == hs[i] && j >= i))) {
                ok = false;
                break;
            }
        }
        if (ok) {
            peaks.push_back(i);
        }
    }
    std::stable_sort(peaks.begin(), peaks.end(), [&](std::size_t a, std::size_t b) { return rs::total_less(hs[b], hs[a]); });
    if (peaks.size() > 12) {
        peaks.resize(12);
    }
    Lcg rng{0x5eed};
    for (std::size_t pk : peaks) {
        const double yc = lo + (static_cast<double>(pk) - 2.0 + 0.5) * p.bin_m;
        std::vector<Vec3> band;
        for (const auto& q : pts) {
            if (std::fabs(q[1] - yc) <= p.band_m) {
                band.push_back(q);
            }
        }
        if (band.size() < p.min_support) {
            continue;
        }
        const double max_slope = std::tan(rf::to_radians(p.max_tilt_deg));
        auto count = [&](const Plane& pl) {
            std::size_t n = 0;
            for (const auto& q : band) {
                if (std::fabs(q[1] - (pl.a * q[0] + pl.b * q[2] + pl.c)) <= p.inlier_m) {
                    n++;
                }
            }
            return n;
        };
        std::vector<double> by = col(band, 1);
        Plane best{0.0, 0.0, median(by)};
        std::size_t best_n = count(best);
        for (std::size_t it = 0; it < p.iters; it++) {
            const std::size_t i0 = rng.next(band.size());
            const std::size_t i1 = rng.next(band.size());
            const std::size_t i2 = rng.next(band.size());
            const std::array<Vec3, 3> s{band[i0], band[i1], band[i2]};
            const double span = rf::fmin(g::norm(g::sub(s[1], s[0])), g::norm(g::sub(s[2], s[0])));
            if (span < 0.3) {
                continue;
            }
            const auto pl = plane3(s);
            if (!pl) {
                continue;
            }
            if (std::sqrt(pl->a * pl->a + pl->b * pl->b) > max_slope) {
                continue;
            }
            const std::size_t n = count(*pl);
            if (n > best_n) {
                best = *pl;
                best_n = n;
            }
        }
        std::vector<Vec3> inl;
        for (const auto& q : band) {
            if (std::fabs(q[1] - (best.a * q[0] + best.b * q[2] + best.c)) <= p.inlier_m) {
                inl.push_back(q);
            }
        }
        if (inl.size() < p.min_support) {
            continue;
        }
        double sxs = -0.0, szs = -0.0;
        for (const auto& q : inl) {
            sxs += q[0];
        }
        for (const auto& q : inl) {
            szs += q[2];
        }
        const std::array<double, 2> c0{sxs / static_cast<double>(inl.size()), szs / static_cast<double>(inl.size())};
        double a = best.a, b = best.b;
        if (const auto ls = plane_ls(inl, c0); ls && std::sqrt(ls->a * ls->a + ls->b * ls->b) <= max_slope) {
            a = ls->a;
            b = ls->b;
        }
        std::vector<double> r = col(inl, 1);
        const double y = median(r);
        r.clear();
        for (const auto& q : inl) {
            r.push_back(q[1] - a * (q[0] - c0[0]) - b * (q[2] - c0[1]));
        }
        const double ycm = median(r);
        std::vector<double> dev;
        for (double v : r) {
            dev.push_back(std::fabs(v - ycm));
        }
        const double spread = 1.4826 * median(dev);
        std::vector<double> xs = col(inl, 0);
        std::vector<double> zs = col(inl, 2);
        const double area = (quantile(xs, 0.9) - quantile(xs, 0.1)) * (quantile(zs, 0.9) - quantile(zs, 0.1));
        if (std::any_of(out.begin(), out.end(), [&](const Layer& l) { return std::fabs(l.y - y) < p.band_m; })) {
            continue;
        }
        out.push_back(Layer{y, inl.size(), spread, spread / std::sqrt(static_cast<double>(inl.size())), tilt_deg(a, b), area});
    }
    std::stable_sort(out.begin(), out.end(), [](const Layer& x, const Layer& z) { return rs::total_less(x.y, z.y); });
    return out;
}

std::expected<Floor, std::string> detect(std::span<const Vec3> pts, std::span<const Vec3> heads, const FloorParams& p) {
    if (heads.empty()) {
        return std::unexpected("no head positions");
    }
    std::vector<double> hy = col(heads, 1), hx = col(heads, 0), hz = col(heads, 2);
    const double head_y = median(hy);
    const std::array<double, 2> at{median(hx), median(hz)};
    const auto ls = layers(pts, at, p);
    if (ls.empty()) {
        return std::unexpected("no horizontal layer with " + std::to_string(p.min_support) + " points (" +
                               std::to_string(pts.size()) + " points in all)");
    }
    std::size_t strongest_n = 0;
    for (const auto& l : ls) {
        strongest_n = std::max(strongest_n, l.inliers);
    }
    const double strongest = static_cast<double>(strongest_n);
    std::vector<std::string> rejected;
    for (const auto& l : ls) {
        const double above = head_y - l.y;
        std::optional<std::string> why;
        if (above > p.head_max_m) {
            why = rf::fixed(above, 1) + " m below the head";
        } else if (above < p.head_min_m) {
            why = "only " + rf::fixed(above, 2) + " m below the head";
        } else if (static_cast<double>(l.inliers) < p.min_rel_support * strongest) {
            why = std::to_string(l.inliers) + " points vs " + rf::display_f64(strongest) + " in the strongest layer";
        } else if (l.area_m2 < p.min_area_m2) {
            why = rf::fixed(l.area_m2, 2) + " m^2 only";
        }
        if (why) {
            rejected.push_back("layer at " + rf::fixed(l.y, 3, true) + " m: " + *why);
        } else {
            Floor f;
            f.layer = l;
            f.head_above_m = above;
            f.confidence = confidence(l, above, ls);
            f.layers = ls;
            f.rejected = rejected;
            return f;
        }
    }
    std::string j;
    for (std::size_t i = 0; i < rejected.size(); i++) {
        j += (i > 0 ? "; " : "") + rejected[i];
    }
    return std::unexpected("no layer passes: " + j);
}

std::optional<Floor> track(std::span<const Vec3> pts, std::span<const Vec3> heads, double prior, double gate_m,
                           const FloorParams& p) {
    if (heads.empty()) {
        return std::nullopt;
    }
    std::vector<double> hy = col(heads, 1);
    const double head_y = median(hy);
    double y = prior;
    std::vector<Vec3> sel;
    for (double r : {0.05, 0.05, 0.025, 0.025, 0.025}) {
        sel.clear();
        for (const auto& q : pts) {
            if (std::fabs(q[1] - y) <= r) {
                sel.push_back(q);
            }
        }
        if (sel.size() < p.min_support) {
            return std::nullopt;
        }
        std::vector<double> sy = col(sel, 1);
        y = median(sy);
        if (std::fabs(y - prior) > gate_m) {
            return std::nullopt;
        }
    }
    std::size_t shell = 0;
    for (const auto& q : pts) {
        if (std::fabs(std::fabs(q[1] - y) - 0.075) <= 0.025) {
            shell++;
        }
    }
    if (sel.size() * 3 < shell * 2) {
        return std::nullopt;
    }
    std::vector<double> dev;
    for (const auto& q : sel) {
        dev.push_back(std::fabs(q[1] - y));
    }
    const double spread = 1.4826 * median(dev);
    std::vector<double> xs = col(sel, 0), zs = col(sel, 2);
    const double area = (quantile(xs, 0.9) - quantile(xs, 0.1)) * (quantile(zs, 0.9) - quantile(zs, 0.1));
    if (area < p.min_area_m2) {
        return std::nullopt;
    }
    const Layer l{y, sel.size(), spread, spread / std::sqrt(static_cast<double>(sel.size())), 0.0, area};
    const double above = head_y - y;
    Floor f;
    f.confidence = confidence(l, above, {});
    f.layers = {l};
    f.layer = l;
    f.head_above_m = above;
    return f;
}

std::pair<std::vector<Vec3>, std::vector<Vec3>> map_points(const map::RoomMap& map, double max_range) {
    std::vector<Vec3> pts, heads;
    for (const auto& k : map.keyframes) {
        heads.push_back(k.pose.t);
        for (const auto& m : k.points) {
            if (g::norm(g::sub(m.p, k.pose.t)) <= max_range) {
                pts.push_back(m.p);
            }
        }
    }
    return {pts, heads};
}

const char* source_name(FloorSource s) {
    switch (s) {
    case FloorSource::Points: return "points";
    case FloorSource::Touch: return "touch";
    case FloorSource::Manual: return "manual";
    case FloorSource::Standing: return "standing";
    }
    return "";
}

std::optional<FloorSource> source_parse(std::string_view s) {
    for (FloorSource f : {FloorSource::Points, FloorSource::Touch, FloorSource::Manual, FloorSource::Standing}) {
        if (s == source_name(f)) {
            return f;
        }
    }
    return std::nullopt;
}

std::string StoredFloor::to_toml() const {
    return "# omarchyvr-roommap floor: floor plane y = floor_y in the room frame (y up, gravity aligned)\nfloor_y = " +
           rf::fixed(y, 5) + "\nsource = \"" + source_name(source) + "\"\nconfidence = " + rf::fixed(confidence, 3) +
           "\nsigma_m = " + rf::fixed(sigma_m, 5) + "\ninliers = " + std::to_string(inliers) +
           "\nhead_above_m = " + rf::fixed(head_above_m, 4) + "\nwhen = " + std::to_string(when) + "\n";
}

std::expected<StoredFloor, std::string> StoredFloor::parse(std::string_view s) {
    StoredFloor f;
    f.y = std::numeric_limits<double>::quiet_NaN();
    f.head_above_m = std::numeric_limits<double>::quiet_NaN();
    for (std::string_view line : rs::lines(s)) {
        const std::size_t h = line.find('#');
        const std::string_view l = rf::trim(h == std::string_view::npos ? line : line.substr(0, h));
        const auto kv = rs::split_once(l, '=');
        if (!kv) {
            continue;
        }
        const std::string_view k = rf::trim(kv->first);
        const std::string_view v = rs::trim_matches(rf::trim(kv->second), "\"");
        std::string err;
        auto num = [&]() -> std::optional<double> { return rs::parse_f64(v, &err); };
        auto bad = [&]() { return std::unexpected(std::string(k) + ": " + err); };
        if (k == "floor_y") {
            auto x = num();
            if (!x) {
                return bad();
            }
            f.y = *x;
        } else if (k == "source") {
            auto x = source_parse(v);
            if (!x) {
                return std::unexpected("source " + rf::debug_str(v));
            }
            f.source = *x;
        } else if (k == "confidence") {
            auto x = num();
            if (!x) {
                return bad();
            }
            f.confidence = *x;
        } else if (k == "sigma_m") {
            auto x = num();
            if (!x) {
                return bad();
            }
            f.sigma_m = *x;
        } else if (k == "inliers") {
            auto x = num();
            if (!x) {
                return bad();
            }
            f.inliers = rs::to_usize(*x);
        } else if (k == "head_above_m") {
            auto x = num();
            if (!x) {
                return bad();
            }
            f.head_above_m = *x;
        } else if (k == "when") {
            auto x = num();
            if (!x) {
                return bad();
            }
            f.when = rs::to_u64(*x);
        }
    }
    if (!std::isfinite(f.y)) {
        return std::unexpected("floor_y missing");
    }
    return f;
}

std::expected<StoredFloor, std::string> StoredFloor::load(const std::string& dir) {
    const std::string p = cam::path_join(dir, "floor.toml");
    auto t = cam::read_to_string(p);
    if (!t) {
        return std::unexpected(p + ": " + t.error().text());
    }
    return parse(*t);
}

std::expected<void, std::string> StoredFloor::save(const std::string& dir) const {
    if (auto r = cam::create_dir_all(dir); !r) {
        return std::unexpected(r.error().text());
    }
    const std::string tmp = cam::path_join(dir, "floor.toml.tmp");
    if (auto w = cam::write_file(tmp, to_toml()); !w) {
        return std::unexpected(w.error().text());
    }
    if (std::rename(tmp.c_str(), cam::path_join(dir, "floor.toml").c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    return {};
}

double floor_shift(double floor_y_app, double origin_height) {
    return -origin_height - floor_y_app;
}

std::string steamvr_stage_path() {
    if (auto v = cam::env_os("PIBRIDGE_STEAMVR_STAGE_FILE"); v && !v->empty()) {
        return *v;
    }
    const std::string rt = cam::env_os("XDG_RUNTIME_DIR").value_or("/tmp");
    return cam::path_join(rt, "pibridge/steamvr-stage");
}

std::optional<SteamVrStage> parse_steamvr_stage(std::string_view text) {
    SteamVrStage st;
    bool magic = false, have_m = false;
    double m[3][4] = {};
    std::size_t i = 0;
    while (i < text.size()) {
        std::size_t e = text.find('\n', i);
        if (e == std::string_view::npos) {
            e = text.size();
        }
        const std::string line(text.substr(i, e - i));
        i = e + 1;
        if (line == "pibridge-steamvr-stage 1") {
            magic = true;
        } else if (line.starts_with("vrserver ")) {
            st.pid = static_cast<std::uint32_t>(std::strtoul(line.c_str() + 9, nullptr, 10));
        } else if (line.starts_with("universe ")) {
            st.universe = std::strtoull(line.c_str() + 9, nullptr, 10);
        } else if (line.starts_with("standing_from_raw ")) {
            const char* p = line.c_str() + 18;
            int n = 0;
            for (; n < 12; n++) {
                char* end = nullptr;
                const double v = std::strtod(p, &end);
                if (end == p || !std::isfinite(v)) {
                    break;
                }
                m[n / 4][n % 4] = v;
                p = end;
            }
            have_m = n == 12;
        }
    }
    if (!magic || !have_m || st.pid == 0) {
        return std::nullopt;
    }
    // standing_from_raw = [R | t]; raw_from_stage = [R^T | -R^T t]; R a yaw about +y: R = [[c,0,s],[0,1,0],[-s,0,c]]
    const double yaw_sr = std::atan2(m[0][2], m[0][0]);
    st.yaw = -yaw_sr;
    for (int r = 0; r < 3; r++) {
        double v = 0;
        for (int k = 0; k < 3; k++) {
            v -= m[k][r] * m[k][3];
        }
        st.t[static_cast<std::size_t>(r)] = v;
    }
    st.tilt_deg = std::acos(std::clamp(m[1][1], -1.0, 1.0)) * 180.0 / M_PI;
    return st;
}

namespace {

std::string comm_of(std::uint32_t pid) {
    if (pid == 0) {
        return {};
    }
    auto c = cam::read_file("/proc/" + std::to_string(pid) + "/comm");
    if (!c) {
        return {};
    }
    std::string s = *c;
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) {
        s.pop_back();
    }
    return s;
}

} // namespace

bool is_vrserver(std::uint32_t pid) {
    return comm_of(pid) == "vrserver";
}

bool is_headtrack(std::uint32_t pid) {
    return comm_of(pid) == "pibridge-headtr"; // pibridge-headtrack, cut to 15 characters
}

std::optional<SteamVrStage> steamvr_stage(std::uint32_t pid) {
    // only the SteamVR backend has a SteamVR stage (anchor/backend.h); the ring's writer is vrserver (driver_monado) or
    // pibridge-headtrack feeding SteamVR through CustomHeadsetOpenVR (then the stage file names the running vrserver)
    if (anchor::backend_from_env() != anchor::Backend::SteamVr) {
        return std::nullopt;
    }
    const bool headtrack = is_headtrack(pid);
    auto b = cam::read_file(steamvr_stage_path());
    if (!b) {
        return std::nullopt;
    }
    auto st = parse_steamvr_stage(*b);
    if (!st || (headtrack ? !is_vrserver(st->pid) : st->pid != pid)) {
        return std::nullopt;
    }
    return st;
}

double origin_height(std::uint32_t monado_pid) {
    if (anchor::backend_from_env() == anchor::Backend::SteamVr) {
        // driver_monado: tracking origin offset 0; the STAGE floor is SteamVR's standing frame
        if (const auto st = steamvr_stage(monado_pid)) {
            return -st->t[1];
        }
        return 0.0;
    }
    std::optional<double> proc_env;
    if (monado_pid != 0) {
        if (auto b = cam::read_file("/proc/" + std::to_string(monado_pid) + "/environ")) {
            std::size_t i = 0;
            const std::string& s = *b;
            while (i <= s.size() && !proc_env) {
                const std::size_t z = s.find('\0', i);
                const std::string_view kv(s.data() + i, (z == std::string::npos ? s.size() : z) - i);
                if (rf::utf8_valid(kv) && kv.starts_with("XRT_TRACKING_ORIGIN_OFFSET_Y=")) {
                    proc_env = rs::parse_f64(kv.substr(29));
                }
                if (z == std::string::npos) {
                    break;
                }
                i = z + 1;
            }
        }
    }
    std::optional<double> own;
    if (auto v = cam::env_utf8("XRT_TRACKING_ORIGIN_OFFSET_Y")) {
        own = rs::parse_f64(*v);
    }
    return MONADO_ORIGIN_HEIGHT + (proc_env ? *proc_env : own.value_or(0.0));
}

std::expected<std::pair<double, double>, std::string> touch_floor(std::span<const Vec3> origins, double bottom_m,
                                                                  double max_spread_m) {
    if (origins.size() < 10) {
        return std::unexpected("only " + std::to_string(origins.size()) + " controller poses");
    }
    std::vector<double> ys = col(origins, 1), xs = col(origins, 0), zs = col(origins, 2);
    const double y = median(ys);
    const double cx = median(xs);
    const double cz = median(zs);
    std::vector<double> d;
    for (const auto& p : origins) {
        d.push_back(g::norm(g::sub(p, {cx, y, cz})));
    }
    const double spread = quantile(d, 0.9);
    if (spread > max_spread_m) {
        return std::unexpected("the controller moved (" + rf::fixed(spread * 100.0, 1) + " cm): lay it still on the floor");
    }
    return std::pair{y - bottom_m, spread};
}

std::optional<std::pair<double, double>> combine(std::optional<std::pair<double, double>> points,
                                                 std::optional<std::pair<double, double>> touch, double gate_m) {
    if (points && touch) {
        const auto [yp, sp] = *points;
        const auto [yt, st] = *touch;
        if (std::fabs(yp - yt) > gate_m) {
            return std::pair{yt, st};
        }
        const double wp = 1.0 / rf::fmax(sp * sp, 1e-8), wt = 1.0 / rf::fmax(st * st, 1e-8);
        return std::pair{(yp * wp + yt * wt) / (wp + wt), std::sqrt(1.0 / (wp + wt))};
    }
    return points ? points : touch;
}

void Follower::measure(double shift, double dt_s) {
    if (!target) {
        target = shift;
        return;
    }
    const double t = *target;
    if (std::fabs(shift - t) > outlier_m) {
        pending.push_back(shift);
        if (pending.size() >= confirm) {
            std::vector<double> p = pending;
            const double m = median(p);
            if (std::all_of(p.begin(), p.end(), [&](double x) { return std::fabs(x - m) <= outlier_m; })) {
                target = m;
            }
            pending.clear();
        }
        return;
    }
    pending.clear();
    const double a = 1.0 - std::exp(-rf::fmax(dt_s, 0.0) / tau_s);
    target = t + a * (shift - t);
}

std::optional<double> Follower::step(double dt_s) {
    if (!target) {
        return std::nullopt;
    }
    const double t = *target;
    double a;
    if (!applied) {
        a = t;
    } else {
        const double lim = max_rate * rf::fmax(dt_s, 0.0);
        a = *applied + rf::clamp(t - *applied, -lim, lim);
    }
    applied = a;
    return a;
}

std::string preview_text(std::span<const Vec3> pts, std::span<const Vec3> heads, std::optional<double> floor_y,
                         std::size_t max_points) {
    const std::size_t step = std::max<std::size_t>(pts.size() / std::max<std::size_t>(max_points, 1), 1);
    std::string s = "# omarchyvr-roommap preview: room frame, cm. h = keyframe (head) x z y, p = point x z y\n";
    if (floor_y) {
        s += "floor " + rf::fixed(*floor_y * 100.0, 1) + "\n";
    }
    for (const auto& h : heads) {
        s += "h " + rf::fixed(h[0] * 100.0, 0) + " " + rf::fixed(h[2] * 100.0, 0) + " " + rf::fixed(h[1] * 100.0, 0) + "\n";
    }
    for (std::size_t i = 0; i < pts.size(); i += step) {
        const auto& p = pts[i];
        s += "p " + rf::fixed(p[0] * 100.0, 0) + " " + rf::fixed(p[2] * 100.0, 0) + " " + rf::fixed(p[1] * 100.0, 0) + "\n";
    }
    return s;
}

} // namespace pibridge::roommap::floor
