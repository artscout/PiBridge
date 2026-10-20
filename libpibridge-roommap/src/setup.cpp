// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/setup.rs (MIT OR Apache-2.0). See setup.h.
#include "pibridge/roommap/setup.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/roommap/align.h"
#include "pibridge/roommap/store.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

namespace pibridge::roommap::setup {

namespace rf = cam::rfmt;
namespace rs = anchor::rs;

const char* mode_name(SetupMode m) {
    return m == SetupMode::Fast ? "fast" : "full";
}

std::optional<SetupMode> mode_parse(std::string_view s) {
    if (s == "fast") {
        return SetupMode::Fast;
    }
    if (s == "full") {
        return SetupMode::Full;
    }
    return std::nullopt;
}

Alignment RoomSetup::room_from_setup() const {
    return Alignment{yaw, {centre[0], 0.0, centre[1]}};
}

Alignment RoomSetup::setup_from_room() const {
    return store::inverse(room_from_setup());
}

std::vector<P2> RoomSetup::boundary_setup() const {
    std::vector<P2> v;
    for (const auto& p : boundary) {
        v.push_back(to_setup(centre, yaw, p));
    }
    return v;
}

double RoomSetup::area() const {
    return std::fabs(setup::area(boundary));
}

std::string RoomSetup::describe() const {
    return "play area " + rf::fixed(play_area[0], 1) + " x " + rf::fixed(play_area[1], 1) + " m (" + rf::fixed(area(), 1) +
           " m^2 inside the boundary), " + mode_name(mode) + " setup";
}

std::string RoomSetup::to_toml() const {
    std::string lab = label;
    for (auto& c : lab) {
        if (c == '"' || c == '\\' || c == '\n') {
            c = ' ';
        }
    }
    std::string pts;
    for (std::size_t i = 0; i < boundary.size(); i++) {
        if (i > 0) {
            pts += ", ";
        }
        pts += "[" + rf::fixed(boundary[i][0], 4) + ", " + rf::fixed(boundary[i][1], 4) + "]";
    }
    return "# omarchyvr-roommap room setup (docs/ROOM-SETUP.md): room frame of the map (y up); setup frame = centre on the "
           "floor, -Z forward\nlabel = \"" +
           lab + "\"\nmode = \"" + mode_name(mode) + "\"\ncentre = [" + rf::fixed(centre[0], 4) + ", " +
           rf::fixed(centre[1], 4) + "]\nyaw_deg = " + rf::fixed(rf::to_degrees(yaw), 4) + "\nplay_area = [" +
           rf::fixed(play_area[0], 3) + ", " + rf::fixed(play_area[1], 3) + "]\nhead_m = " + rf::fixed(head_m, 4) +
           "\nwhen = " + std::to_string(when) + "\nboundary = [" + pts + "]\n";
}

std::expected<RoomSetup, std::string> RoomSetup::parse(std::string_view s) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    RoomSetup r;
    r.centre = {nan, nan};
    r.yaw = nan;
    r.head_m = nan;
    auto nums = [](std::string_view v) {
        std::vector<double> out;
        std::size_t i = 0;
        while (i <= v.size()) {
            std::size_t j = v.find_first_of("[],", i);
            if (j == std::string_view::npos) {
                j = v.size();
            }
            double d = 0;
            if (rf::parse_f64(rf::trim(v.substr(i, j - i)), d)) {
                out.push_back(d);
            }
            i = j + 1;
        }
        return out;
    };
    for (std::string_view l0 : rs::lines(s)) {
        const std::string_view l = rf::trim(l0);
        if (l.starts_with('#')) {
            continue;
        }
        const auto kv = rs::split_once(l, '=');
        if (!kv) {
            continue;
        }
        const std::string_view k = rf::trim(kv->first);
        const std::string_view v = rf::trim(kv->second);
        if (k == "label") {
            r.label = std::string(rs::trim_matches(v, "\""));
        } else if (k == "mode") {
            auto m = mode_parse(rs::trim_matches(v, "\""));
            if (!m) {
                return std::unexpected("mode " + std::string(v));
            }
            r.mode = *m;
        } else if (k == "centre") {
            const auto n = nums(v);
            if (n.size() == 2) {
                r.centre = {n[0], n[1]};
            }
        } else if (k == "yaw_deg") {
            std::string e;
            auto d = rs::parse_f64(v, &e);
            if (!d) {
                return std::unexpected("yaw_deg: " + e);
            }
            r.yaw = rf::to_radians(*d);
        } else if (k == "play_area") {
            const auto n = nums(v);
            if (n.size() == 2) {
                r.play_area = {n[0], n[1]};
            }
        } else if (k == "head_m") {
            r.head_m = rs::parse_f64(v).value_or(nan);
        } else if (k == "when") {
            r.when = rs::parse_u64(v).value_or(0);
        } else if (k == "boundary") {
            const auto n = nums(v);
            r.boundary.clear();
            for (std::size_t i = 0; i + 1 < n.size(); i += 2) {
                r.boundary.push_back({n[i], n[i + 1]});
            }
        }
    }
    if (!std::isfinite(r.centre[0]) || !std::isfinite(r.centre[1]) || !std::isfinite(r.yaw)) {
        return std::unexpected("centre / yaw_deg missing");
    }
    if (r.boundary.size() < 3) {
        return std::unexpected("boundary needs at least 3 points");
    }
    return r;
}

std::expected<RoomSetup, std::string> RoomSetup::load(const std::string& dir) {
    const std::string p = cam::path_join(dir, FILE);
    auto t = cam::read_to_string(p);
    if (!t) {
        return std::unexpected(p + ": " + t.error().text());
    }
    return parse(*t);
}

std::expected<void, std::string> RoomSetup::save(const std::string& dir) const {
    if (auto r = cam::create_dir_all(dir); !r) {
        return std::unexpected(r.error().text());
    }
    const std::string tmp = cam::path_join(dir, std::string(FILE) + ".tmp");
    if (auto w = cam::write_file(tmp, to_toml()); !w) {
        return std::unexpected(w.error().text());
    }
    if (std::rename(tmp.c_str(), cam::path_join(dir, FILE).c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    return {};
}

P2 to_setup(P2 centre, double yaw, P2 p) {
    const double s = std::sin(yaw), c = std::cos(yaw);
    const double dx = p[0] - centre[0], dz = p[1] - centre[1];
    return {c * dx - s * dz, s * dx + c * dz};
}

P2 to_room(P2 centre, double yaw, P2 p) {
    const double s = std::sin(yaw), c = std::cos(yaw);
    return {c * p[0] + s * p[1] + centre[0], -s * p[0] + c * p[1] + centre[1]};
}

double heading(double dir_x, double dir_z) {
    return std::atan2(-dir_x, -dir_z);
}

Alignment final_offset(const Alignment& room_from_slam, double floor_dy, const RoomSetup* setup) {
    const Alignment a{room_from_slam.yaw, {room_from_slam.t[0], room_from_slam.t[1] + floor_dy, room_from_slam.t[2]}};
    if (setup != nullptr) {
        return store::compose(setup->setup_from_room(), a);
    }
    return a;
}

double area(const std::vector<P2>& p) {
    const std::size_t n = p.size();
    if (n < 3) {
        return 0.0;
    }
    double s = -0.0;
    for (std::size_t i = 0; i < n; i++) {
        s += p[i][0] * p[(i + 1) % n][1] - p[(i + 1) % n][0] * p[i][1];
    }
    return s / 2.0;
}

double perimeter(const std::vector<P2>& p) {
    const std::size_t n = p.size();
    double s = -0.0;
    for (std::size_t i = 0; i < n; i++) {
        s += dist(p[i], p[(i + 1) % n]);
    }
    return s;
}

double dist(P2 a, P2 b) {
    const double dx = a[0] - b[0], dz = a[1] - b[1];
    return std::sqrt(dx * dx + dz * dz);
}

bool inside(const std::vector<P2>& poly, P2 q) {
    const std::size_t n = poly.size();
    bool c = false;
    std::size_t j = n - 1;
    for (std::size_t i = 0; i < n; i++) {
        const P2 a = poly[i], b = poly[j];
        if ((a[1] > q[1]) != (b[1] > q[1]) && q[0] < (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0]) {
            c = !c;
        }
        j = i;
    }
    return c;
}

namespace {

double cross(P2 o, P2 a, P2 b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

void dp(std::span<const P2> p, double tol, std::vector<P2>& out) {
    if (p.size() < 3) {
        for (std::size_t i = 0; i + 1 < p.size(); i++) {
            out.push_back(p[i]);
        }
        return;
    }
    const P2 a = p[0], b = p[p.size() - 1];
    const double len = rf::fmax(dist(a, b), 1e-9);
    double best = 0.0;
    std::size_t bi = 0;
    for (std::size_t i = 1; i + 1 < p.size(); i++) {
        const double d = std::fabs(cross(a, b, p[i])) / len;
        if (d > best) {
            best = d;
            bi = i;
        }
    }
    if (best > tol) {
        dp(p.subspan(0, bi + 1), tol, out);
        dp(p.subspan(bi), tol, out);
    } else {
        out.push_back(a);
    }
}

} // namespace

bool segments_cross(P2 a, P2 b, P2 c, P2 d) {
    const double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
    return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) && d1 != 0.0 && d2 != 0.0 && d3 != 0.0 && d4 != 0.0;
}

bool self_intersects(const std::vector<P2>& p) {
    const std::size_t n = p.size();
    for (std::size_t i = 0; i < n; i++) {
        for (std::size_t j = i + 1; j < n; j++) {
            if (j == i + 1 || (i == 0 && j == n - 1)) {
                continue;
            }
            if (segments_cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) {
                return true;
            }
        }
    }
    return false;
}

std::vector<P2> convex_hull(const std::vector<P2>& pts) {
    std::vector<P2> v = pts;
    std::stable_sort(v.begin(), v.end(), [](const P2& a, const P2& b) {
        const int c = rs::total_cmp(a[0], b[0]);
        return c != 0 ? c < 0 : rs::total_cmp(a[1], b[1]) < 0;
    });
    // dedup: consecutive equal points (PartialEq: NaN never equal)
    std::vector<P2> u;
    for (const auto& p : v) {
        if (u.empty() || !(u.back()[0] == p[0] && u.back()[1] == p[1])) {
            u.push_back(p);
        }
    }
    v = std::move(u);
    if (v.size() < 3) {
        return v;
    }
    std::vector<P2> h;
    for (int pass = 0; pass < 2; pass++) {
        const std::size_t start = h.size();
        for (std::size_t k = 0; k < v.size(); k++) {
            const P2 p = pass == 0 ? v[k] : v[v.size() - 1 - k];
            while (h.size() >= start + 2 && cross(h[h.size() - 2], h[h.size() - 1], p) <= 0.0) {
                h.pop_back();
            }
            h.push_back(p);
        }
        h.pop_back();
    }
    return h;
}

std::expected<std::vector<P2>, std::string> clean_boundary(const std::vector<P2>& trace, double tol) {
    std::vector<P2> v;
    for (const auto& p : trace) {
        if (v.empty() || dist(v.back(), p) >= 0.05) {
            v.push_back(p);
        }
    }
    if (v.size() < 3 || perimeter(v) < 2.0) {
        return std::unexpected("the traced line is too short: walk all the way around the play space");
    }
    std::size_t far = 0;
    for (std::size_t i = 1; i < v.size(); i++) {
        if (rs::total_cmp(dist(v[0], v[i]), dist(v[0], v[far])) >= 0) {
            far = i;
        }
    }
    std::vector<P2> a(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(far) + 1);
    std::vector<P2> b(v.begin() + static_cast<std::ptrdiff_t>(far), v.end());
    b.push_back(v[0]);
    std::vector<P2> out;
    dp(a, tol, out);
    dp(b, tol, out);
    if (out.size() < 3) {
        out = convex_hull(v);
    }
    if (self_intersects(out)) {
        out = convex_hull(out);
    }
    if (std::fabs(area(out)) < 1.0) {
        return std::unexpected("the traced area is only " + rf::fixed(std::fabs(area(out)), 1) +
                               " m^2: trace at least 1 x 1 m");
    }
    return out;
}

bool rect_inside(const std::vector<P2>& poly, double hx, double hz) {
    const P2 c[4] = {{-hx, -hz}, {hx, -hz}, {hx, hz}, {-hx, hz}};
    for (const auto& q : c) {
        if (!inside(poly, q)) {
            return false;
        }
    }
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; i++) {
        const P2 a = poly[i], b = poly[(i + 1) % n];
        if (std::fabs(a[0]) < hx && std::fabs(a[1]) < hz) {
            return false;
        }
        for (int k = 0; k < 4; k++) {
            if (segments_cross(a, b, c[k], c[(k + 1) % 4])) {
                return false;
            }
        }
    }
    return true;
}

std::array<double, 2> play_area(const std::vector<P2>& poly) {
    if (poly.size() < 3 || !inside(poly, {0.0, 0.0})) {
        return {0.0, 0.0};
    }
    double b0 = 0.0, b1 = 0.0, b2 = 0.0;
    for (int k = 1; k < 60; k++) {
        const double th = static_cast<double>(k) / 60.0 * (std::numbers::pi / 2.0);
        const double dx = std::cos(th), dz = std::sin(th);
        double lo = 0.0, hi = 30.0;
        for (int i = 0; i < 40; i++) {
            const double m = 0.5 * (lo + hi);
            if (rect_inside(poly, m * dx, m * dz)) {
                lo = m;
            } else {
                hi = m;
            }
        }
        const double hx = lo * dx, hz = lo * dz;
        if (hx * hz > b0) {
            b0 = hx * hz;
            b1 = hx;
            b2 = hz;
        }
    }
    return {2.0 * b1, 2.0 * b2};
}

std::vector<P2> free_area(const std::vector<P2>& obstacles, std::optional<std::array<double, 4>> extent, double max_half) {
    std::array<double, 4> h;
    bool grow;
    if (obstacles.empty() && !extent) {
        h = {DEFAULT_HALF_M, DEFAULT_HALF_M, DEFAULT_HALF_M, DEFAULT_HALF_M};
        grow = false;
    } else {
        h = {0.4, 0.4, 0.4, 0.4};
        grow = true;
    }
    if (grow) {
        const double step = 0.05;
        bool open[4] = {true, true, true, true};
        while (open[0] || open[1] || open[2] || open[3]) {
            for (int s = 0; s < 4; s++) {
                if (!open[s]) {
                    continue;
                }
                const double nh = h[s] + step;
                const double x0 = -h[0], x1 = h[1], z0 = -h[2], z1 = h[3];
                double s0, s1, s2, s3;
                switch (s) {
                case 0: s0 = -nh; s1 = x0; s2 = z0; s3 = z1; break;
                case 1: s0 = x1; s1 = nh; s2 = z0; s3 = z1; break;
                case 2: s0 = x0; s1 = x1; s2 = -nh; s3 = z0; break;
                default: s0 = x0; s1 = x1; s2 = z1; s3 = nh; break;
                }
                std::size_t n = 0;
                for (const auto& p : obstacles) {
                    if (p[0] >= s0 && p[0] <= s1 && p[1] >= s2 && p[1] <= s3) {
                        n++;
                    }
                }
                bool within = true;
                if (extent) {
                    const auto& e = *extent;
                    switch (s) {
                    case 0: within = -nh >= e[0]; break;
                    case 1: within = nh <= e[1]; break;
                    case 2: within = -nh >= e[2]; break;
                    default: within = nh <= e[3]; break;
                    }
                }
                if (n >= 3 || !within || nh > max_half + 1e-9) {
                    open[s] = false;
                } else {
                    h[s] = nh;
                }
            }
        }
    }
    return {{-h[0], -h[2]}, {h[1], -h[2]}, {h[1], h[3]}, {-h[0], h[3]}};
}

std::pair<std::vector<P2>, std::optional<std::array<double, 4>>> obstacles_from_points(
    const std::vector<std::array<double, 3>>& points, double floor_y, P2 centre, double yaw) {
    std::vector<P2> obs;
    std::vector<double> xs, zs;
    for (const auto& p : points) {
        const P2 q = to_setup(centre, yaw, {p[0], p[2]});
        if (std::hypot(q[0], q[1]) > 4.0) {
            continue;
        }
        xs.push_back(q[0]);
        zs.push_back(q[1]);
        const double h = p[1] - floor_y;
        if (h >= 0.15 && h <= 2.0) {
            obs.push_back(q);
        }
    }
    if (xs.size() < 50) {
        return {obs, std::nullopt};
    }
    auto q = [](std::vector<double>& v, double f) {
        std::stable_sort(v.begin(), v.end(), rs::total_less);
        return v[rs::to_usize(std::round(static_cast<double>(v.size() - 1) * f))];
    };
    const std::array<double, 4> e{q(xs, 0.02), q(xs, 0.98), q(zs, 0.02), q(zs, 0.98)};
    return {obs, e};
}

std::vector<std::array<std::array<double, 3>, 4>> walls(const std::vector<P2>& poly, double height) {
    std::vector<std::array<std::array<double, 3>, 4>> v;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; i++) {
        const P2 a = poly[i], b = poly[(i + 1) % n];
        v.push_back({{{a[0], 0.0, a[1]}, {a[0], height, a[1]}, {b[0], height, b[1]}, {b[0], 0.0, b[1]}}});
    }
    return v;
}

namespace {
constexpr double PRESS = 0.7;
constexpr double RELEASE = 0.3;
} // namespace

std::expected<std::tuple<P2, double, double>, std::string> confirm_forward(Guide& g, double timeout_s) {
    g.say("Centre and forward",
          "Stand in the middle of your play space and look straight ahead (where games should face). Pull a trigger, or "
          "hold still for 3 seconds.",
          std::nullopt);
    bool was_down[2] = {true, true};
    struct Still {
        double t0;
        std::array<double, 3> p0;
        double y0;
    };
    std::optional<Still> still_since;
    for (;;) {
        const Sample s = g.sample();
        if (s.t > timeout_s) {
            return std::unexpected("no confirmation in time: run the setup again");
        }
        if (!s.head) {
            still_since.reset();
            continue;
        }
        const auto [hp, hy] = *s.head;
        for (int i = 0; i < 2; i++) {
            const bool down = s.hands[i].active && s.hands[i].value >= PRESS;
            const bool up = !s.hands[i].active || s.hands[i].value <= RELEASE;
            if (down && !was_down[i] && s.t > 0.5) {
                return std::tuple{P2{hp[0], hp[2]}, hy, hp[1]};
            }
            if (up) {
                was_down[i] = false;
            } else if (down) {
                was_down[i] = true;
            }
        }
        if (still_since && dist({still_since->p0[0], still_since->p0[2]}, {hp[0], hp[2]}) < 0.03 &&
            std::fabs(align::wrap(hy - still_since->y0)) < 3.0 * std::numbers::pi / 180.0) {
            const double left = 3.0 - (s.t - still_since->t0);
            if (s.t > 2.0 && left <= 0.0) {
                return std::tuple{P2{hp[0], hp[2]}, hy, hp[1]};
            }
            if (s.t > 2.0) {
                g.say("Centre and forward", "Hold still... " + rf::fixed(std::ceil(left), 0),
                      static_cast<float>(rf::clamp(1.0 - left / 3.0, 0.0, 1.0)));
            }
        } else {
            still_since = Still{s.t, hp, hy};
        }
    }
}

std::expected<std::vector<P2>, std::string> trace_boundary(Guide& g, double timeout_s) {
    g.say("Play area boundary",
          "Hold a trigger and walk along the edge of your free space, controller at hip height. Let go when you are back "
          "at the start.",
          std::nullopt);
    std::optional<std::size_t> hand;
    std::vector<P2> path;
    double length = 0.0;
    bool saw_input = false;
    for (;;) {
        const Sample s = g.sample();
        if (s.t > timeout_s) {
            if (saw_input) {
                return std::unexpected("the boundary was not finished in time: run the setup again");
            }
            return std::unexpected(
                "no controller or hand input: switch a controller on (or turn on camera hands) and run the setup again");
        }
        saw_input |= s.hands[0].active || s.hands[1].active;
        if (!hand) {
            for (std::size_t i = 0; i < 2; i++) {
                if (s.hands[i].active && s.hands[i].value >= PRESS && s.hands[i].pos) {
                    hand = i;
                    const auto p = *s.hands[i].pos;
                    path = {{p[0], p[2]}};
                    length = 0.0;
                    break;
                }
            }
            continue;
        }
        const HandSample& h = s.hands[*hand];
        if (h.pos) {
            const P2 q{(*h.pos)[0], (*h.pos)[2]};
            const double d = dist(path.back(), q);
            if (d >= 0.02) {
                length += d;
                path.push_back(q);
                g.say("Play area boundary", "Tracing... " + rf::fixed(length, 1) + " m. Let go when you are back at the start.",
                      std::nullopt);
            }
        }
        const bool released = !h.active || h.value <= RELEASE;
        const bool closed = length > 3.0 && dist(path[0], path.back()) < 0.3;
        if (released || closed) {
            if (length < 2.0) {
                g.say("Play area boundary", "That was too short. Hold the trigger again and walk all the way around.",
                      std::nullopt);
                hand.reset();
                path.clear();
                continue;
            }
            return path;
        }
    }
}

} // namespace pibridge::roommap::setup
