// SPDX-License-Identifier: MIT
// Ported from ZoneTweak main.go, area.go, features.go and tool.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
#include "pibridge/room/scene.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <set>

namespace pibridge::room {

namespace {

constexpr Vec3 kUp{0, 1, 0};
constexpr double kPi = std::numbers::pi;

Rgba mix(Rgba a, Rgba b, float k) {
    return {a.r + (b.r - a.r) * k, a.g + (b.g - a.g) * k, a.b + (b.b - a.b) * k, a.a + (b.a - a.a) * k};
}

/// Facing the camera, upright (ZoneTweak billboard).
std::pair<Vec3, Vec3> billboard(Vec3 cam, Vec3 p) {
    Vec3 f = cam - p;
    f.y = 0;
    if (f.length() < 1e-4) {
        f = {0, 0, 1};
    }
    return {kUp.cross(f.norm()).norm(), kUp};
}

void ring(Batch& b, Vec3 c, double r, Rgba col, int n = 28) {
    for (int i = 0; i < n; i++) {
        const double a0 = 2 * kPi * i / n;
        const double a1 = 2 * kPi * (i + 1) / n;
        b.line(c + Vec3{std::cos(a0) * r, 0, std::sin(a0) * r}, c + Vec3{std::cos(a1) * r, 0, std::sin(a1) * r}, col);
    }
}

/// A square post `s` half-wide and `h` tall on the floor at p.
void post(Batch& b, Vec3 p, double s, double h, Rgba col) {
    const Vec3 a = p + Vec3{-s, 0, -s};
    const Vec3 c = p + Vec3{s, 0, -s};
    const Vec3 d = p + Vec3{s, 0, s};
    const Vec3 f = p + Vec3{-s, 0, s};
    const Vec3 u{0, h, 0};
    b.quad(a, c, c + u, a + u, col);
    b.quad(c, d, d + u, c + u, col);
    b.quad(d, f, f + u, d + u, col);
    b.quad(f, a, a + u, f + u, col);
    b.quad(a + u, c + u, d + u, f + u, col);
}

std::pair<Rgba, Rgba> wallColors(double level, double t) {
    if (level <= 0) {
        return {col::green.alpha(0.10F), col::green};
    }
    const double pulse = 0.8 + 0.2 * std::sin(t * 11);
    const auto l = static_cast<float>(level * pulse);
    const Rgba m = mix(col::green, col::danger, l);
    return {m.alpha(0.12F + 0.5F * l), m};
}

void floorGrid(Batch& b) {
    // 8 x 8 m around the stage origin: 0.5 m faint, 1 m brighter
    for (int i = -16; i <= 16; i++) {
        const double v = i * 0.5;
        const Rgba c = i % 2 == 0 ? col::grid1 : col::grid;
        b.line({v, 0.001, -8}, {v, 0.001, 8}, c);
        b.line({-8, 0.001, v}, {8, 0.001, v}, c);
    }
}

void drawWalls(Editor& e, Batch& b, double level, const std::set<int>& hotV, const std::set<int>& hotE, double t) {
    const auto n = e.pts.size();
    const Vec3 cam = e.hmd.pos();
    const auto [sheet, edge] = wallColors(level, t);
    for (std::size_t i = 0; i < n; i++) {
        const Pt2 a = e.pts[i];
        const Pt2 c = e.pts[(i + 1) % n];
        const bool hot = hotE.contains(static_cast<int>(i));
        const Rgba ec = hot ? col::amber : edge;
        b.quad(fl(a, 0), fl(c, 0), fl(c, e.height), fl(a, e.height), hot ? col::amber.alpha(0.18F) : sheet);
        b.line(fl(a, 0.003), fl(c, 0.003), ec);
        b.line(fl(a, e.height), fl(c, e.height), ec.alpha(0.6F));
        for (double y = 0.5; y < e.height; y += 0.5) {
            b.line(fl(a, y), fl(c, y), ec.alpha(0.18F));
        }
        if (const double l = dist(a, c); l > 0.25) {
            const Vec3 mid = fl({(a.x + c.x) / 2, (a.z + c.z) / 2}, 1.15);
            const auto [rr, uu] = billboard(cam, mid);
            drawTextCentered(b, e.fmtLen(l), mid, rr, uu, kTextPx * 1.4, col::white);
        }
    }
    for (std::size_t i = 0; i < n; i++) {
        const bool hot = hotV.contains(static_cast<int>(i));
        post(b, fl(e.pts[i], 0), hot ? 0.035 : 0.025, e.height, hot ? col::amber : edge);
        if (hot) {
            ring(b, fl(e.pts[i], 0.004), e.grabR(), col::amber.alpha(0.6F));
        }
    }
}

void drawLock(Editor& e, Batch& b) {
    const Rigid2 g = e.centerPose();
    const Pt2 p = e.lockPos();
    Rgba c = e.areaMode ? col::area : col::muted;
    for (const Hand& hd : e.hands) {
        if (hd.valid) {
            if (const auto q = hd.pointer(e.laser); q && dist(*q, p) < 0.15) {
                c = col::amber;
            }
        }
    }
    const Vec3 r{g.a, 0, g.c};
    const Vec3 f = Vec3{-g.b, 0, -g.d} * -1;
    const Vec3 base = fl(p, 0.004);
    const double bw = 0.07;
    const double bh = 0.055;
    const auto corner = [&](double x, double z) { return base + r * x + f * z; };
    b.quad(corner(-bw, -bh), corner(bw, -bh), corner(bw, bh), corner(-bw, bh), c);
    const double lift = e.areaMode ? 0.05 : 0.0;
    constexpr int kN = 12;
    for (int i = 0; i < kN; i++) {
        const double a0 = kPi * i / kN;
        const double a1 = kPi * (i + 1) / kN;
        b.line(corner(-0.045 * std::cos(a0), bh + lift + 0.06 * std::sin(a0)),
               corner(-0.045 * std::cos(a1), bh + lift + 0.06 * std::sin(a1)), c);
    }
    b.line(corner(-0.045, bh), corner(-0.045, bh + lift), c);
    if (!e.areaMode) {
        b.line(corner(0.045, bh), corner(0.045, bh + lift), c);
    }
    const auto [rr, uu] = billboard(e.hmd.pos(), base);
    drawTextCentered(b, e.areaMode ? "LOCK PLAY AREA" : "UNLOCK PLAY AREA", base + Vec3{0, 0.1, 0}, rr, uu,
                     kSmallPx * 1.1, c);
}

void drawArcs(Editor& e, Batch& b, bool hot) {
    const Rigid2 g = e.centerPose();
    const Rgba c = hot ? col::amber : col::yellow;
    for (const double mid : {0.0, kPi}) {
        constexpr int kN = 24;
        std::vector<Pt2> p;
        for (int i = 0; i <= kN; i++) {
            const double a = mid - kArcSpan / 2 + kArcSpan * i / kN;
            p.push_back(g.apply({kArcR * std::sin(a), -kArcR * std::cos(a)}));
        }
        for (const double off : {0.004, 0.012}) {
            for (int i = 0; i < kN; i++) {
                b.line(fl(p[static_cast<std::size_t>(i)], off), fl(p[static_cast<std::size_t>(i + 1)], off), c);
            }
        }
        for (const double end : {-1.0, 1.0}) {
            const double a = mid + end * kArcSpan / 2;
            const Pt2 tip{kArcR * std::sin(a + end * 0.12), -kArcR * std::cos(a + end * 0.12)};
            const Pt2 in{(kArcR - 0.08) * std::sin(a), -(kArcR - 0.08) * std::cos(a)};
            const Pt2 out{(kArcR + 0.08) * std::sin(a), -(kArcR + 0.08) * std::cos(a)};
            b.tri(fl(g.apply(in), 0.006), fl(g.apply(out), 0.006), fl(g.apply(tip), 0.006), c);
        }
    }
}

void drawArea(Editor& e, Batch& b) {
    const Rigid2 g = e.centerPose();
    const Pt2 o = g.apply({0, 0});
    const Pt2 fwd = g.apply({0, -0.45});
    b.line(fl(g.apply({-0.25, 0}), 0.003), fl(g.apply({0.25, 0}), 0.003), col::yellow);
    b.line(fl(g.apply({0, 0.25}), 0.003), fl(fwd, 0.003), col::yellow);
    b.line(fl(fwd, 0.003), fl(g.apply({-0.08, -0.35}), 0.003), col::yellow);
    b.line(fl(fwd, 0.003), fl(g.apply({0.08, -0.35}), 0.003), col::yellow);
    const auto [r, u] = billboard(e.hmd.pos(), fl(o, 0));
    std::string name = "PLAY CENTER";
    if (e.centerDirty || e.areaDirty) {
        name += " (UNSAVED)";
    }
    drawTextCentered(b, name, fl(o, 0.14), r, u, kSmallPx * 1.5, col::yellow);
    const double w = e.area.w;
    const double h = e.area.h;
    const auto cs = e.areaCorners(g, w, h);
    const Rgba ac = e.areaMode ? col::area : col::area.alpha(0.45F);
    for (std::size_t i = 0; i < 4; i++) {
        b.line(fl(cs[i], 0.005), fl(cs[(i + 1) % 4], 0.005), ac);
    }
    drawLock(e, b);
    if (!e.areaMode) {
        return;
    }
    for (std::size_t i = 0; i < 4; i++) {
        b.quad(fl(cs[i], 0), fl(cs[(i + 1) % 4], 0), fl(cs[(i + 1) % 4], 0.03), fl(cs[i], 0.03), col::area.alpha(0.25F));
    }
    std::set<int> hot;
    for (const Hand& hd : e.hands) {
        if (hd.valid) {
            if (hd.areaGrab != 0) {
                hot.insert(hd.areaGrab);
            } else if (hd.areaHover != 0) {
                hot.insert(hd.areaHover);
            }
        }
    }
    const auto hs = e.areaHandles();
    for (const auto& [k, p] : hs) {
        if (k == 10) {
            continue;
        }
        double size = 0.035;
        Rgba hc = col::area;
        if (k == 1) {
            size = 0.06;
            hc = col::yellow;
        } else if (k >= 6) {
            size = 0.05;
        }
        if (hot.contains(k)) {
            hc = col::amber;
            size *= 1.3;
        }
        post(b, fl(p, 0), size, 0.06, hc);
    }
    drawArcs(e, b, hot.contains(10));
    for (const Hand& hd : e.hands) {
        const int k = hd.areaGrab != 0 ? hd.areaGrab : hd.areaHover;
        if (!hd.valid || k == 0) {
            continue;
        }
        std::string txt = k == 1 ? "MOVE PLAY CENTER" : k == 10 ? "DRAG ALONG THE ARROW TO TURN" : "STRETCH PLAY AREA";
        Pt2 hp{};
        for (const auto& [kk, p] : hs) {
            if (kk == k) {
                hp = p;
            }
        }
        const Vec3 p = fl(hp, 0.22);
        const auto [rr, uu] = billboard(e.hmd.pos(), p);
        drawTextCentered(b, txt, p, rr, uu, kSmallPx * 1.3, col::amber);
    }
    const Vec3 lp = fl(g.apply({0, h / 2 + 0.2}), 0.25);
    const auto [r2, u2] = billboard(e.hmd.pos(), lp);
    drawTextCentered(b, std::format("PLAY AREA {} X {}", e.fmtLen(w), e.fmtLen(h)), lp, r2, u2, kSmallPx * 1.4, col::area);
    drawTextCentered(b, "FORWARD", fl(g.apply({0, -0.6}), 0.05), r2, u2, kSmallPx * 1.1, col::yellow.alpha(0.7F));
}

void drawProximity(Editor& e, Batch& b, double level, double t) {
    if (!e.proxTest || e.pts.size() < 3) {
        return;
    }
    const double d = e.currentWarnLine();
    const Poly in = insetPoly(e.pts, d);
    Rgba c = col::warn;
    for (const Hand& h : e.hands) {
        if (h.lineGrab) {
            c = col::amber;
        }
    }
    for (std::size_t i = 0; i < in.size(); i++) {
        const Pt2 a = in[i];
        const Pt2 q = in[(i + 1) % in.size()];
        for (double k = 0; k < 1; k += 0.1) {
            const Pt2 p0{a.x + (q.x - a.x) * k, a.z + (q.z - a.z) * k};
            const Pt2 p1{a.x + (q.x - a.x) * (k + 0.06), a.z + (q.z - a.z) * (k + 0.06)};
            b.line(fl(p0, 0.006), fl(p1, 0.006), c);
        }
        b.quad(fl(a, 0), fl(q, 0), fl(q, 0.04), fl(a, 0.04), c.alpha(0.25F));
    }
    bool known = false;
    (void)e.warnDist(&known);
    std::string label = "WARNING DISTANCE " + e.fmtLen(d);
    if (!known) {
        label += " (STEAMVR DEFAULT, ASSUMED)";
    }
    if (!in.empty()) {
        const Vec3 mid = fl({(in[0].x + in[1 % in.size()].x) / 2, (in[0].z + in[1 % in.size()].z) / 2}, 0.35);
        const auto [r, u] = billboard(e.hmd.pos(), mid);
        drawTextCentered(b, label, mid, r, u, kSmallPx * 1.4, c);
    }
    if (level > 0) {
        const auto pulse = static_cast<float>(0.75 + 0.25 * std::sin(t * 11));
        const Vec3 cam = e.hmd.pos();
        Vec3 fwd = e.hmd.axis(2) * -1;
        fwd.y = 0;
        fwd = fwd.norm();
        const Vec3 p = cam + fwd * 0.7 + Vec3{0, 0.12, 0};
        const auto [r, u] = billboard(cam, p);
        drawTextCentered(b, "SAFETY ZONE WOULD SHOW HERE", p, r, u, kTextPx * 1.2, col::danger.alpha(pulse));
    }
}

void drawTools(Editor& e, Batch& b) {
    for (const Hand& h : e.hands) {
        if (!h.valid) {
            continue;
        }
        // the controller: a small box along its axes
        const Vec3 o = h.m.pos();
        const Vec3 x = h.m.axis(0) * 0.02;
        const Vec3 y = h.m.axis(1) * 0.015;
        const Vec3 z = h.m.axis(2) * 0.06;
        const Rgba bc = h.idx == e.laserHand ? col::beam : col::muted;
        for (const Vec3 s : {x, x * -1}) {
            b.line(o + s + y - z, o + s + y + z, bc);
            b.line(o + s - y - z, o + s - y + z, bc);
        }
        b.line(o - x + y - z, o + x + y - z, bc);
        b.line(o - x - y - z, o + x - y - z, bc);
        if (h.idx != e.laserHand) {
            continue;
        }
        const Vec3 muzzle = h.tip();
        if (!e.laser) {
            const Vec3 end = fl(h.cursor(), 0.003);
            b.line(muzzle, end, col::beam.alpha(0.8F));
            ring(b, end, 0.05, col::beam);
            ring(b, fl(h.cursor(), 0.002), e.grabR(), col::beam.alpha(0.2F), 48);
        } else if (h.aimValid) {
            const Vec3 dir = h.aim.axis(2) * -1;
            Vec3 end = h.aim.pos() + dir * 15;
            if (h.menuRay) {
                end = h.menuPt + e.menuN * 0.004;
            } else if (h.rayHitOK) {
                end = h.rayHit;
            } else if (const auto c = h.pointer(true)) {
                end = fl(*c, 0.004);
                ring(b, end, 0.06, col::laser);
            }
            b.line(h.aim.pos(), end, col::laser);
        }
        const int g = h.grab >= 0 ? h.grab : h.hoverV;
        if (g >= 0 && g < static_cast<int>(e.pts.size()) && !e.drawing) {
            const Pt2 p = e.pts[static_cast<std::size_t>(g)];
            const double yy = std::clamp(muzzle.y, 0.45, std::max(0.45, e.height - 0.3));
            const Rgba tc = h.grab >= 0 ? col::amber : col::beam;
            if (!(e.laser && h.rayHitOK)) {
                b.line(muzzle, Vec3{p.x, yy, p.z}, tc);
            }
            ring(b, Vec3{p.x, yy, p.z}, 0.075, tc, 24);
        }
        if (e.menuOpen) {
            const Rgba tc = h.btn >= 0 ? col::buttonHot : col::muted;
            const Vec3 tp = h.tip();
            b.line(tp - Vec3{0.01, 0, 0}, tp + Vec3{0.01, 0, 0}, tc);
            b.line(tp - Vec3{0, 0.01, 0}, tp + Vec3{0, 0.01, 0}, tc);
        }
    }
}

Vec3 panelPoint(const Editor& e, double x, double y, double z) { return e.menuPos + e.menuR * x + e.menuU * y + e.menuN * z; }

void panelQuad(const Editor& e, Batch& b, double x0, double y0, double x1, double y1, double z, Rgba c) {
    b.quad(panelPoint(e, x0, y0, z), panelPoint(e, x1, y0, z), panelPoint(e, x1, y1, z), panelPoint(e, x0, y1, z), c);
}

void panelRect(const Editor& e, Batch& b, double x0, double y0, double x1, double y1, double z, Rgba c) {
    const Vec3 a = panelPoint(e, x0, y0, z);
    const Vec3 bb = panelPoint(e, x1, y0, z);
    const Vec3 cc = panelPoint(e, x1, y1, z);
    const Vec3 d = panelPoint(e, x0, y1, z);
    b.line(a, bb, c);
    b.line(bb, cc, c);
    b.line(cc, d, c);
    b.line(d, a, c);
}

std::string clip(const std::string& s, std::size_t n) { return s.size() > n ? s.substr(0, n) : s; }

void buildPanel(Editor& e, Batch& b, double t) {
    const auto [slots, h] = e.layout();
    const double w2 = kPanelW / 2;
    const double h2 = h / 2;
    const Vec3 r = e.menuR;
    const Vec3 u = e.menuU;
    panelQuad(e, b, -w2 - 0.01, -h2 - 0.01, w2 + 0.01, h2 + 0.01, -0.004, col::panel);
    panelRect(e, b, -w2, -h2, w2, h2, 0.002, col::beam.alpha(0.45F));
    const double hdrY = h2 - kHeaderH + 0.008;
    b.line(panelPoint(e, -w2 + 0.01, hdrY, 0.002), panelPoint(e, w2 - 0.01, hdrY, 0.002), col::green.alpha(0.8F));
    const auto header = e.page->header();
    for (std::size_t i = 0; i < header.size(); i++) {
        const std::string line = clip(header[i], 64);
        // shrink a long line to the panel's width
        const double px = std::min(i == 0 ? kTextPx : kSmallPx, (kPanelW - 2 * kPanelPad) / std::max(1, textWidth(line)));
        const Rgba c = i == 0 ? col::green : col::muted;
        drawText(b, line, panelPoint(e, -w2 + kPanelPad, h2 - 0.03 - static_cast<double>(i) * 0.026, 0.003),
                 r, u, px, c);
    }
    std::set<int> hot;
    for (const Hand& hd : e.hands) {
        if (hd.valid && hd.btn >= 0) {
            hot.insert(hd.btn);
        }
    }
    for (std::size_t i = 0; i < e.cur.size() && i < slots.size(); i++) {
        const Button& bt = e.cur[i];
        const Slot& s = slots[i];
        const std::string label = bt.label ? bt.label() : std::string();
        if (label.empty()) {
            continue;
        }
        const bool on = bt.on && bt.on();
        const bool isHot = hot.contains(static_cast<int>(i));
        const double x0 = s.cx - s.w / 2;
        const double x1 = s.cx + s.w / 2;
        const double y0 = s.cy - s.h / 2;
        const double y1 = s.cy + s.h / 2;
        const double z = isHot ? 0.008 : 0.004;
        Rgba c = on ? col::buttonOn : col::button;
        if (isHot) {
            c = Rgba{0.16F, 0.24F, 0.22F, 1};
        }
        panelQuad(e, b, x0, y0, x1, y1, z, c);
        if (isHot) {
            const auto pulse = static_cast<float>(0.75 + 0.25 * std::sin(t * 8));
            panelRect(e, b, x0, y0, x1, y1, z + 0.0005, col::green.alpha(0.9F * pulse));
        } else {
            panelRect(e, b, x0 + 0.002, y0 + 0.002, x1 - 0.002, y1 - 0.002, z + 0.0005, col::beam.alpha(0.18F));
        }
        if (on) {
            b.line(panelPoint(e, x0 + 0.006, y0 + 0.008, z + 0.001), panelPoint(e, x0 + 0.006, y1 - 0.008, z + 0.001),
                   col::green);
        }
        double px = kTextPx;
        if (const double tw = textWidth(label) * px; tw > s.w - 0.02) {
            px = (s.w - 0.02) / textWidth(label);
        }
        drawTextCentered(b, label, panelPoint(e, s.cx, s.cy, z + 0.001), r, u, px, isHot ? Rgba{0.95F, 1, 0.97F, 1} : col::white);
    }
    // the tooltip beside the hot button
    int tipIdx = -1;
    for (const int i : hot) {
        tipIdx = i;
    }
    if (tipIdx >= 0 && static_cast<std::size_t>(tipIdx) < e.cur.size() && e.cur[static_cast<std::size_t>(tipIdx)].label) {
        if (const std::string tip = tipFor(e.cur[static_cast<std::size_t>(tipIdx)].label()); !tip.empty()) {
            constexpr double kBoxW = 0.34;
            constexpr double kPad = 0.014;
            constexpr double kTpx = 0.0019;
            constexpr double kLh = 0.02;
            const auto chars = static_cast<std::size_t>((kBoxW - 2 * kPad) / (6 * kTpx));
            const auto lines = wrapText(tip, chars);
            const double boxH = static_cast<double>(lines.size()) * kLh + 2 * kPad - (kLh - 7 * kTpx);
            const Slot& s = slots[static_cast<std::size_t>(tipIdx)];
            const double x0 = s.cx < 0 ? -w2 - 0.06 - kBoxW : w2 + 0.06;
            const double yTop = std::min(h2, std::max(-h2 + boxH, s.cy + boxH / 2));
            panelQuad(e, b, x0, yTop - boxH, x0 + kBoxW, yTop, 0.01, Rgba{0.02F, 0.05F, 0.07F, 0.93F});
            panelRect(e, b, x0, yTop - boxH, x0 + kBoxW, yTop, 0.0105, col::beam.alpha(0.7F));
            for (std::size_t i = 0; i < lines.size(); i++) {
                drawText(b, lines[i], panelPoint(e, x0 + kPad, yTop - kPad - static_cast<double>(i) * kLh - 7 * kTpx, 0.0112), r, u,
                         kTpx, col::laser);
            }
        }
    }
    b.line(panelPoint(e, -w2 + 0.01, -h2 + kFooterH - 0.006, 0.002), panelPoint(e, w2 - 0.01, -h2 + kFooterH - 0.006, 0.002),
           col::beam.alpha(0.4F));
    drawText(b, clip(e.status(), 60), panelPoint(e, -w2 + kPanelPad, -h2 + 0.03, 0.003), r, u, kSmallPx, col::white);
    drawText(b, "POINT INTO A BUTTON AND PULL THE TRIGGER", panelPoint(e, -w2 + kPanelPad, -h2 + 0.011, 0.003), r, u,
             kSmallPx * 0.9, col::muted);
}

} // namespace

void buildScene(Editor& e, Batch& b, double t) {
    b.clear();
    floorGrid(b);
    const double level = e.proximityLevel();
    drawArea(e, b);
    drawProximity(e, b, level, t);
    if (e.ghost && e.changed() && e.orig.size() > 2) {
        for (std::size_t i = 0; i < e.orig.size(); i++) {
            b.line(fl(e.orig[i], 0.004), fl(e.orig[(i + 1) % e.orig.size()], 0.004), col::ghost);
        }
    }
    std::set<int> hotV;
    std::set<int> hotE;
    for (const Hand& h : e.hands) {
        if (!h.valid || e.drawing || e.tracing) {
            continue;
        }
        if (h.grab >= 0) {
            hotV.insert(h.grab);
        } else if (h.hoverV >= 0) {
            hotV.insert(h.hoverV);
        } else if (h.hoverE >= 0) {
            hotE.insert(h.hoverE);
        }
    }
    if (e.drawing) {
        for (std::size_t i = 0; i < e.draft.size(); i++) {
            post(b, fl(e.draft[i], 0), 0.025, e.height, col::beam);
            if (i > 0) {
                b.quad(fl(e.draft[i - 1], 0), fl(e.draft[i], 0), fl(e.draft[i], e.height), fl(e.draft[i - 1], e.height),
                       col::beam.alpha(0.10F));
            }
        }
        if (e.draftHand >= 0 && !e.draft.empty()) {
            const Hand& dh = e.hands[static_cast<std::size_t>(e.draftHand)];
            if (dh.valid) {
                if (const auto dc = dh.pointer(e.laser)) {
                    b.line(fl(e.draft.back(), 0.004), fl(*dc, 0.004), col::beam.alpha(0.5F));
                    if (e.draft.size() >= 3 && dist(*dc, e.draft.front()) < kCloseRadius) {
                        ring(b, fl(e.draft.front(), 0.003), kCloseRadius, col::amber);
                    }
                }
            }
        }
    } else {
        drawWalls(e, b, level, hotV, hotE, t);
    }
    if (e.tracing) {
        const Poly& p = e.trace.path();
        for (std::size_t i = 1; i < p.size(); i++) {
            b.line(fl(p[i - 1], 0.006), fl(p[i], 0.006), col::beam);
        }
    }
    drawTools(e, b);
    const Vec3 cam = e.hmd.pos();
    Vec3 fwd = e.hmd.axis(2) * -1;
    fwd.y = 0;
    fwd = fwd.norm();
    if (e.floorAt != 0) {
        const Vec3 p = cam + fwd * 0.8;
        const auto [rr, uu] = billboard(cam, p);
        const double left = std::max(0.0, std::ceil(e.floorAt - t));
        drawTextCentered(b, std::format("REST A CONTROLLER ON THE FLOOR  {:.0f}", left), p, rr, uu, kTextPx * 1.5, col::amber);
    } else if (!e.menuOpen && e.statusAge() < 4 && !e.status().empty()) {
        const Vec3 p = cam + fwd * 0.9 + Vec3{0, -0.3, 0};
        const auto [rr, uu] = billboard(cam, p);
        drawTextCentered(b, e.status(), p, rr, uu, kTextPx * 1.3, col::white);
    }
    if (e.menuOpen) {
        buildPanel(e, b, t);
    }
}

} // namespace pibridge::room
