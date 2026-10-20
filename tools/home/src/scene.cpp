// SPDX-License-Identifier: MIT
#include "scene.h"

#include "pibridge/room/files.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pibridge::home {

namespace {

using room::Batch;
using room::Rgba;
using room::Vec3;

// PiBridge's colours (app/src/ui/brand.h): the accent blue, a cool teal for the room, white text on a dark panel.
constexpr Rgba kGrid{0.29F, 0.35F, 0.91F, 0.55F};
constexpr Rgba kHorizon{0.45F, 0.50F, 0.95F, 0.60F};
constexpr Rgba kRoom{0.25F, 0.85F, 0.75F, 0.90F};
constexpr Rgba kCtrl{0.90F, 0.92F, 1.00F, 0.90F};
constexpr Rgba kPanelBg{0.05F, 0.06F, 0.11F, 0.88F};
constexpr Rgba kPanelEdge{0.29F, 0.35F, 0.91F, 0.80F};
constexpr Rgba kText{0.93F, 0.94F, 1.00F, 1.00F};
constexpr Rgba kTextDim{0.62F, 0.66F, 0.80F, 1.00F};

// the grid's alpha at a distance from the centre: full near, nothing at kGridRadius
float fade(double d) {
    const double f = std::clamp(1.0 - d / kGridRadius, 0.0, 1.0);
    return static_cast<float>(f * f);
}

void fadedLine(Batch& b, Vec3 a, Vec3 c, Rgba col) {
    const double da = std::hypot(a.x, a.z);
    const double dc = std::hypot(c.x, c.z);
    b.vert(b.lines, a, col.alpha(col.a * fade(da)));
    b.vert(b.lines, c, col.alpha(col.a * fade(dc)));
}

} // namespace

Panel placePanel(const room::Mat34& head, double floorY) {
    Vec3 fwd = head.axis(2) * -1.0; // OpenXR looks along -z
    fwd.y = 0;
    if (fwd.length() < 1e-3) {
        fwd = {0, 0, -1}; // looking straight up or down: the stage's forward
    }
    fwd = fwd.norm();
    Panel p;
    p.center = head.pos() + fwd * 1.8;
    p.center.y = std::clamp(head.pos().y - 0.1, floorY + 0.9, floorY + 2.2);
    p.right = {-fwd.z, 0, fwd.x};
    p.up = {0, 1, 0};
    return p;
}

std::size_t addGrid(Batch& b, const World& w) {
    const std::size_t before = b.lines.size();
    const double y = w.floorY;
    const int n = static_cast<int>(kGridRadius);
    // 1 m lines in 1 m pieces, so each piece fades with its own distance
    for (int i = -n; i <= n; i++) {
        for (int j = -n; j < n; j++) {
            const auto x = static_cast<double>(i);
            const auto z0 = static_cast<double>(j);
            fadedLine(b, {x, y, z0}, {x, y, z0 + 1}, kGrid);
            fadedLine(b, {z0, y, x}, {z0 + 1, y, x}, kGrid);
        }
    }
    // the play centre: a small ring
    constexpr int seg = 32;
    for (int k = 0; k < seg; k++) {
        const double a0 = 2 * std::numbers::pi * k / seg;
        const double a1 = 2 * std::numbers::pi * (k + 1) / seg;
        b.line({0.3 * std::cos(a0), y, 0.3 * std::sin(a0)}, {0.3 * std::cos(a1), y, 0.3 * std::sin(a1)}, kGrid.alpha(0.9F));
    }
    return (b.lines.size() - before) / 2;
}

std::size_t addHorizon(Batch& b, const World& w) {
    const std::size_t before = b.lines.size();
    const double y = w.floorY + kEyeHeight;
    constexpr int seg = 128;
    for (int k = 0; k < seg; k++) {
        const double a0 = 2 * std::numbers::pi * k / seg;
        const double a1 = 2 * std::numbers::pi * (k + 1) / seg;
        b.line({kHorizonRadius * std::cos(a0), y, kHorizonRadius * std::sin(a0)},
               {kHorizonRadius * std::cos(a1), y, kHorizonRadius * std::sin(a1)}, kHorizon);
    }
    return (b.lines.size() - before) / 2;
}

std::size_t addRoom(Batch& b, const World& w) {
    if (!w.room || w.room->size() < 3) {
        return 0;
    }
    const std::size_t before = b.lines.size();
    const room::Poly& p = *w.room;
    const double y0 = w.floorY;
    const double y1 = w.floorY + std::clamp(w.roomHeight, 0.5, 4.0);
    for (std::size_t i = 0; i < p.size(); i++) {
        const room::Pt2 a = p[i];
        const room::Pt2 c = p[(i + 1) % p.size()];
        b.line({a.x, y0, a.z}, {c.x, y0, c.z}, kRoom);                // on the floor
        b.line({a.x, y1, a.z}, {c.x, y1, c.z}, kRoom.alpha(0.25F));   // the top of the walls
        b.line({a.x, y0, a.z}, {a.x, y1, a.z}, kRoom.alpha(0.35F));   // a corner post
    }
    return (b.lines.size() - before) / 2;
}

std::size_t addControllers(Batch& b, const Frame& f) {
    const std::size_t before = b.lines.size();
    for (std::size_t h = 0; h < 2; h++) {
        if (!f.grip[h]) {
            continue;
        }
        const room::Mat34& g = *f.grip[h];
        const Vec3 o = g.pos();
        // a small cross (6 cm) and a little pyramid, so the hand shows from every side
        for (int axis = 0; axis < 3; axis++) {
            const Vec3 d = g.axis(axis) * 0.03;
            b.line(o - d, o + d, kCtrl);
        }
        const Vec3 tip = o - g.axis(2) * 0.05;
        const Vec3 l = o + g.axis(0) * 0.02;
        const Vec3 r = o - g.axis(0) * 0.02;
        const Vec3 u = o + g.axis(1) * 0.02;
        b.tri(tip, l, u, kCtrl.alpha(0.55F));
        b.tri(tip, u, r, kCtrl.alpha(0.55F));
        b.tri(tip, r, l, kCtrl.alpha(0.40F));
        // the pointer, 25 cm along the aim
        if (f.aim[h]) {
            const room::Mat34& a = *f.aim[h];
            b.line(a.pos(), a.pos() - a.axis(2) * 0.25, kCtrl.alpha(0.6F));
        }
    }
    return (b.lines.size() - before) / 2;
}

std::size_t addPanel(Batch& b, const Panel& p) {
    const std::size_t before = b.lines.size();
    constexpr double hw = 0.62;
    constexpr double hh = 0.20;
    const Vec3 c = p.center;
    const Vec3 r = p.right;
    const Vec3 u = p.up;
    const Vec3 n = r.cross(u) * 0.002; // the text just in front of the panel (towards the viewer)
    const Vec3 tl = c - r * hw + u * hh;
    const Vec3 tr = c + r * hw + u * hh;
    const Vec3 br = c + r * hw - u * hh;
    const Vec3 bl = c - r * hw - u * hh;
    b.quad(bl, br, tr, tl, kPanelBg);
    b.line(tl, tr, kPanelEdge);
    b.line(tr, br, kPanelEdge);
    b.line(br, bl, kPanelEdge);
    b.line(bl, tl, kPanelEdge);
    room::drawTextCentered(b, kPanelTitle, c + u * 0.07 + n, r, u, 0.0092, kText);
    room::drawTextCentered(b, kPanelHint, c - u * 0.08 + n, r, u, 0.0068, kTextDim);
    return (b.lines.size() - before) / 2;
}

void buildScene(Batch& b, const World& w, const Frame& f) {
    b.clear();
    if (w.grid) {
        addGrid(b, w);
        addHorizon(b, w);
    }
    addRoom(b, w);
    addControllers(b, f);
    if (f.panel) {
        addPanel(b, *f.panel);
    }
}

std::optional<room::Poly> roomOutline(const room::Room& r, room::FrameMode mode) {
    if (r.points.size() < 3) {
        return std::nullopt;
    }
    if (r.frame == "stage" || mode == room::FrameMode::Off || !r.standing) {
        return r.points;
    }
    // as room::Store::toStageMode: rotate (p - standing) by the standing yaw (B: the yaw negated)
    const double y = mode == room::FrameMode::A ? r.standing->yaw : -r.standing->yaw;
    const double c = std::cos(y);
    const double s = std::sin(y);
    room::Poly out;
    out.reserve(r.points.size());
    for (const room::Pt2 p : r.points) {
        const double dx = p.x - r.standing->x;
        const double dz = p.z - r.standing->z;
        out.push_back({c * dx - s * dz, s * dx + c * dz});
    }
    return out;
}

World worldFor(bool stage, const std::string& roomsDir) {
    World w;
    w.stage = stage;
    w.floorY = stage ? 0.0 : kLocalFloorY;
    const room::RoomLib lib(roomsDir);
    const std::string active = lib.active();
    if (active.empty()) {
        return w;
    }
    if (const auto r = lib.find(active)) {
        w.room = roomOutline(*r, lib.frameMode(room::FrameMode::A));
        w.roomHeight = r->height;
        w.roomName = r->name;
    }
    return w;
}

} // namespace pibridge::home
