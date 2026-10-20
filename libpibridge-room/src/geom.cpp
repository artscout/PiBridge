// SPDX-License-Identifier: MIT
// Ported from ZoneTweak geom.go and geom2.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
#include "pibridge/room/geom.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace pibridge::room {

namespace {

std::size_t nxt(std::size_t i, std::size_t n) { return (i + 1) % n; }
std::size_t prv(std::size_t i, std::size_t n) { return (i + n - 1) % n; }

} // namespace

double dist(Pt2 a, Pt2 b) { return std::hypot(b.x - a.x, b.z - a.z); }

double polyArea(const Poly& p) {
    double a = 0;
    for (std::size_t i = 0; i < p.size(); i++) {
        const Pt2 u = p[i];
        const Pt2 v = p[nxt(i, p.size())];
        a += u.x * v.z - v.x * u.z;
    }
    return a / 2;
}

double polyPerimeter(const Poly& p) {
    double l = 0;
    for (std::size_t i = 0; i < p.size(); i++) {
        l += dist(p[i], p[nxt(i, p.size())]);
    }
    return l;
}

bool pointInPoly(const Poly& p, double x, double z) {
    bool c = false;
    const std::size_t n = p.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i, i++) {
        const Pt2 a = p[i];
        const Pt2 b = p[j];
        if ((a.z > z) != (b.z > z) && x < (b.x - a.x) * (z - a.z) / (b.z - a.z) + a.x) {
            c = !c;
        }
    }
    return c;
}

SegDist segDist(Pt2 p, Pt2 a, Pt2 b) {
    const double dx = b.x - a.x;
    const double dz = b.z - a.z;
    const double l = dx * dx + dz * dz;
    double t = 0;
    if (l > 0) {
        t = ((p.x - a.x) * dx + (p.z - a.z) * dz) / l;
    }
    t = std::clamp(t, 0.0, 1.0);
    return {std::hypot(p.x - a.x - t * dx, p.z - a.z - t * dz), t};
}

double domAngle(const Poly& p) {
    double s = 0;
    double c = 0;
    for (std::size_t i = 0; i < p.size(); i++) {
        const Pt2 a = p[i];
        const Pt2 b = p[nxt(i, p.size())];
        const double dx = b.x - a.x;
        const double dz = b.z - a.z;
        const double l2 = dx * dx + dz * dz;
        const double t = std::atan2(dz, dx) * 4;
        s += l2 * std::sin(t);
        c += l2 * std::cos(t);
    }
    return std::atan2(s, c) / 4;
}

Poly simplify(const Poly& p, double tol) {
    Poly q = p;
    while (q.size() > 4) {
        long bi = -1;
        double bd = tol;
        for (std::size_t i = 0; i < q.size(); i++) {
            const double d = segDist(q[i], q[prv(i, q.size())], q[nxt(i, q.size())]).d;
            if (d < bd) {
                bd = d;
                bi = static_cast<long>(i);
            }
        }
        if (bi < 0) {
            break;
        }
        q.erase(q.begin() + bi);
    }
    return q;
}

namespace {

struct WallGroup {
    int k;     // direction quadrant 0..3
    int o;     // orientation: 0 along u, 1 along v
    double l;  // total length
    double cw; // length-weighted offset
};

} // namespace

Poly squareUp(const Poly& p, double theta, double minLen) {
    const Frame f = Frame::of(theta);
    const std::size_t n = p.size();
    std::vector<WallGroup> g;
    for (std::size_t i = 0; i < n; i++) {
        const Pt2 a = f.to(p[i]);
        const Pt2 b = f.to(p[nxt(i, n)]);
        const double du = b.x - a.x;
        const double dv = b.z - a.z;
        const double l = std::hypot(du, dv);
        if (l < 1e-6) {
            continue;
        }
        const int k = ((static_cast<int>(std::round(std::atan2(dv, du) / (std::numbers::pi / 2))) % 4) + 4) % 4;
        const int o = k % 2;
        double c = (a.z + b.z) / 2;
        if (o == 1) {
            c = (a.x + b.x) / 2;
        }
        g.push_back({k, o, l, c * l});
    }
    for (int guard = 0; guard < 10000; guard++) {
        const std::size_t m = g.size();
        if (m < 4) {
            return {};
        }
        bool merged = false;
        for (std::size_t i = 0; i < m; i++) {
            const std::size_t j = nxt(i, m);
            if (g[i].k == g[j].k) {
                g[i] = {g[i].k, g[i].o, g[i].l + g[j].l, g[i].cw + g[j].cw};
                g.erase(g.begin() + static_cast<long>(j));
                merged = true;
                break;
            }
        }
        if (merged) {
            continue;
        }
        long wi = -1;
        double wl = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < m; i++) {
            const std::size_t j = nxt(i, m);
            if (g[i].o == g[j].o) {
                std::size_t s = i;
                if (g[j].l < g[i].l) {
                    s = j;
                }
                if (g[s].l < wl) {
                    wl = g[s].l;
                    wi = static_cast<long>(s);
                }
            }
        }
        if (wi < 0) {
            for (std::size_t i = 0; i < m; i++) {
                if (g[i].l < minLen && g[i].l < wl) {
                    wl = g[i].l;
                    wi = static_cast<long>(i);
                }
            }
        }
        if (wi < 0) {
            break;
        }
        g.erase(g.begin() + wi);
    }
    if (g.size() < 4) {
        return {};
    }
    Poly out;
    out.reserve(g.size());
    for (std::size_t i = 0; i < g.size(); i++) {
        const WallGroup& a = g[i];
        const WallGroup& b = g[nxt(i, g.size())];
        const double ca = a.cw / a.l;
        const double cb = b.cw / b.l;
        if (a.o == 0) {
            out.push_back(f.from({cb, ca}));
        } else {
            out.push_back(f.from({ca, cb}));
        }
    }
    return out;
}

Pt2 snapToNeighbors(Pt2 p, const Pt2* prev, const Pt2* next, double theta) {
    const Frame f = Frame::of(theta);
    Pt2 q = f.to(p);
    const double t = std::tan(7 * std::numbers::pi / 180);
    for (const Pt2* nb : {prev, next}) {
        if (nb == nullptr) {
            continue;
        }
        const Pt2 r = f.to(*nb);
        const double du = q.x - r.x;
        const double dv = q.z - r.z;
        if (std::fabs(du) < std::fabs(dv) * t) {
            q.x = r.x;
        } else if (std::fabs(dv) < std::fabs(du) * t) {
            q.z = r.z;
        }
    }
    return f.from(q);
}

Poly Rigid2::all(const Poly& p) const {
    Poly out;
    out.reserve(p.size());
    for (const Pt2 q : p) {
        out.push_back(apply(q));
    }
    return out;
}

Rigid2 rigidFromYaw(double yaw, double x, double z) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {c, -s, s, c, x, z};
}

double distToBoundary(const Poly& p, Pt2 c) {
    if (p.size() < 3 || !pointInPoly(p, c.x, c.z)) {
        return 0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < p.size(); i++) {
        best = std::min(best, segDist(c, p[i], p[nxt(i, p.size())]).d);
    }
    return best;
}

Poly insetPoly(const Poly& p, double d) {
    const std::size_t n = p.size();
    if (n < 3) {
        return {};
    }
    const double sg = polyArea(p) < 0 ? -1.0 : 1.0;
    struct Line {
        Pt2 p, dir, nrm;
    };
    std::vector<Line> ls(n);
    for (std::size_t i = 0; i < n; i++) {
        const Pt2 a = p[i];
        const Pt2 b = p[nxt(i, n)];
        const double dx = b.x - a.x;
        const double dz = b.z - a.z;
        double l = std::hypot(dx, dz);
        if (l < 1e-9) {
            l = 1e-9;
        }
        const double nx = -dz / l * sg;
        const double nz = dx / l * sg;
        ls[i] = {{a.x + nx * d, a.z + nz * d}, {dx / l, dz / l}, {nx, nz}};
    }
    Poly out(n);
    for (std::size_t i = 0; i < n; i++) {
        const Line& l1 = ls[prv(i, n)];
        const Line& l2 = ls[i];
        const double den = l1.dir.x * l2.dir.z - l1.dir.z * l2.dir.x;
        const Pt2 fallback{p[i].x + (l1.nrm.x + l2.nrm.x) / 2 * d, p[i].z + (l1.nrm.z + l2.nrm.z) / 2 * d};
        if (std::fabs(den) < 1e-6) {
            out[i] = fallback;
            continue;
        }
        const double wx = l2.p.x - l1.p.x;
        const double wz = l2.p.z - l1.p.z;
        const double t = (wx * l2.dir.z - wz * l2.dir.x) / den;
        Pt2 q{l1.p.x + l1.dir.x * t, l1.p.z + l1.dir.z * t};
        if (dist(q, p[i]) > 3 * d) {
            q = fallback;
        }
        out[i] = q;
    }
    return out;
}

bool segsCross(Pt2 a, Pt2 b, Pt2 c, Pt2 d) {
    const auto o = [](Pt2 p, Pt2 q, Pt2 r) { return (q.x - p.x) * (r.z - p.z) - (q.z - p.z) * (r.x - p.x); };
    const double d1 = o(c, d, a);
    const double d2 = o(c, d, b);
    const double d3 = o(a, b, c);
    const double d4 = o(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

bool rectFits(const Poly& p, double hw, double hh) {
    const std::array<Pt2, 4> cs{{{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}}};
    for (const Pt2 c : cs) {
        if (!pointInPoly(p, c.x, c.z)) {
            return false;
        }
    }
    for (std::size_t i = 0; i < p.size(); i++) {
        const Pt2 a = p[i];
        const Pt2 b = p[nxt(i, p.size())];
        for (std::size_t k = 0; k < 4; k++) {
            if (segsCross(a, b, cs[k], cs[(k + 1) % 4])) {
                return false;
            }
        }
        if (std::fabs(a.x) < hw && std::fabs(a.z) < hh) {
            return false;
        }
    }
    return true;
}

std::optional<Size2> fitArea(const Poly& p, double margin) {
    if (p.size() < 3 || !pointInPoly(p, 0, 0)) {
        return std::nullopt;
    }
    double bestA = 0;
    double bw = 0;
    double bh = 0;
    double bestE = 0;
    const auto attempt = [&](double ex) {
        const double asp = std::pow(10.0, ex);
        double lo = 0;
        double hi = 15;
        for (int k = 0; k < 40; k++) {
            const double mid = (lo + hi) / 2;
            if (rectFits(p, mid, mid * asp)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        if (const double a = lo * lo * asp; a > bestA) {
            bestA = a;
            bw = 2 * lo;
            bh = 2 * lo * asp;
            bestE = ex;
        }
    };
    for (int i = 0; i <= 60; i++) {
        attempt(-0.7 + 1.4 * i / 60.0);
    }
    double step = 1.4 / 60;
    for (int r = 0; r < 3; r++) {
        const double c = bestE;
        for (int i = -10; i <= 10; i++) {
            attempt(c + step * i / 10.0);
        }
        step /= 10;
    }
    if (bestA <= 0) {
        return std::nullopt;
    }
    return Size2{std::max(0.5, bw - 2 * margin), std::max(0.5, bh - 2 * margin)};
}

double outsideDist(const Poly& p, Pt2 c) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < p.size(); i++) {
        best = std::min(best, segDist(c, p[i], p[nxt(i, p.size())]).d);
    }
    return best;
}

std::optional<Pt2> centroid(const Poly& p) {
    double cx = 0;
    double cz = 0;
    double a = 0;
    for (std::size_t i = 0; i < p.size(); i++) {
        const Pt2 u = p[i];
        const Pt2 v = p[nxt(i, p.size())];
        const double cr = u.x * v.z - v.x * u.z;
        a += cr;
        cx += (u.x + v.x) * cr;
        cz += (u.z + v.z) * cr;
    }
    if (std::fabs(a) < 1e-9) {
        return std::nullopt;
    }
    return Pt2{cx / (3 * a), cz / (3 * a)};
}

bool isSimple(const Poly& p) {
    const std::size_t n = p.size();
    if (n < 3 || std::fabs(polyArea(p)) < 1e-4) {
        return false;
    }
    for (std::size_t i = 0; i < n; i++) {
        if (dist(p[i], p[nxt(i, n)]) < 0.01) {
            return false;
        }
    }
    for (std::size_t i = 0; i < n; i++) {
        for (std::size_t j = i + 1; j < n; j++) {
            if (j == i || nxt(i, n) == j || nxt(j, n) == i) {
                continue; // adjacent edges share a corner
            }
            if (segsCross(p[i], p[nxt(i, n)], p[j], p[nxt(j, n)])) {
                return false;
            }
        }
    }
    return true;
}

Poly ccw(const Poly& p) {
    if (polyArea(p) >= 0) {
        return p;
    }
    return Poly(p.rbegin(), p.rend());
}

namespace {

void dpRange(const Poly& path, std::size_t a, std::size_t b, double tol, std::vector<bool>& keep) {
    if (b <= a + 1) {
        return;
    }
    double best = -1;
    std::size_t bi = a;
    for (std::size_t i = a + 1; i < b; i++) {
        const double d = segDist(path[i], path[a], path[b]).d;
        if (d > best) {
            best = d;
            bi = i;
        }
    }
    if (best > tol) {
        keep[bi] = true;
        dpRange(path, a, bi, tol, keep);
        dpRange(path, bi, b, tol, keep);
    }
}

} // namespace

Poly douglasPeucker(const Poly& path, double tol) {
    if (path.size() < 3) {
        return path;
    }
    std::vector<bool> keep(path.size(), false);
    keep.front() = true;
    keep.back() = true;
    dpRange(path, 0, path.size() - 1, tol, keep);
    Poly out;
    for (std::size_t i = 0; i < path.size(); i++) {
        if (keep[i]) {
            out.push_back(path[i]);
        }
    }
    return out;
}

Poly polygonFromTrace(const Poly& path, double tol) {
    Poly thin;
    for (const Pt2 q : path) {
        if (thin.empty() || dist(thin.back(), q) >= 0.05) {
            thin.push_back(q);
        }
    }
    if (thin.size() < 3) {
        return {};
    }
    // close the loop explicitly so the simplification keeps the corner where the walk started
    thin.push_back(thin.front());
    Poly s = douglasPeucker(thin, tol);
    s.pop_back(); // the closing duplicate
    // the start point may lie on a straight wall: drop it when it does
    if (s.size() > 3 && segDist(s.front(), s.back(), s[1]).d < tol) {
        s.erase(s.begin());
    }
    if (s.size() < 3) {
        return {};
    }
    s = ccw(s);
    return isSimple(s) ? s : Poly{};
}

Poly rectangle(double w, double d) {
    return {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, d / 2}, {-w / 2, d / 2}};
}

} // namespace pibridge::room
