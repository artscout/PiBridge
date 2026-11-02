// SPDX-License-Identifier: MIT
#include "desk.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace pibridge::home::desk {

std::uint32_t maskOne(std::uint32_t monitor) { return monitor < desktop::kMaxMonitors ? 1U << monitor : 0U; }

std::uint32_t maskAll(const std::vector<desktop::MonitorInfo>& mons) {
    std::uint32_t m = 0;
    for (const auto& mi : mons) {
        m |= maskOne(mi.index);
    }
    return m;
}

std::vector<Screen> place(const room::Mat34& head, double floorY, const std::vector<desktop::MonitorInfo>& mons,
                          std::uint32_t mask, double scale) {
    std::vector<const desktop::MonitorInfo*> shown;
    for (const auto& mi : mons) {
        if ((mask & maskOne(mi.index)) != 0 && mi.logicalW > 0 && mi.logicalH > 0) {
            shown.push_back(&mi);
        }
    }
    std::ranges::stable_sort(shown, [](const auto* a, const auto* b) { return a->x != b->x ? a->x < b->x : a->y < b->y; });
    std::vector<Screen> out;
    if (shown.empty()) {
        return out;
    }
    room::Vec3 fwd = head.axis(2) * -1.0;
    fwd.y = 0;
    if (fwd.length() < 1e-6) {
        fwd = {0, 0, -1};
    }
    fwd = fwd.norm();
    const room::Vec3 up{0, 1, 0};
    const room::Vec3 rightOfView = fwd.cross(up).norm();
    room::Vec3 axis = head.pos();
    const double eyeY = head.pos().y;
    axis.y = 0;
    const double mpp = kMetresPerPixel * std::clamp(scale, 0.3, 3.0);
    // the arc: widths and gaps along it, the whole centred straight ahead
    double total = 0;
    for (const auto* m : shown) {
        total += m->logicalW * mpp;
    }
    total += kGap * static_cast<double>(shown.size() - 1);
    // the screens' vertical middle (desktop y differences kept), a little below the eyes, never into the floor
    int minY = shown.front()->y, maxY = shown.front()->y + static_cast<int>(shown.front()->logicalH);
    for (const auto* m : shown) {
        minY = std::min(minY, m->y);
        maxY = std::max(maxY, m->y + static_cast<int>(m->logicalH));
    }
    const double blockH = (maxY - minY) * mpp;
    const double middleY = std::max(eyeY - 0.1, floorY + 0.3 + blockH / 2);
    double along = -total / 2;
    for (const auto* m : shown) {
        Screen s;
        s.monitor = m->index;
        s.w = m->logicalW * mpp;
        s.h = m->logicalH * mpp;
        s.pixW = m->width;
        s.pixH = m->height;
        const double a = (along + s.w / 2) / kDistance; // the screen's angle, + to the right
        along += s.w + kGap;
        const room::Vec3 dir = fwd * std::cos(a) + rightOfView * std::sin(a);
        s.center = axis + dir * kDistance;
        // its middle row at the block's height: desktop y grows down
        const double top = middleY + blockH / 2 - (m->y - minY) * mpp;
        s.center.y = top - s.h / 2;
        s.toward = (dir * -1.0).norm();
        s.right = up.cross(s.toward).norm(); // facing the viewer: right = up x toward
        s.up = up;
        out.push_back(s);
    }
    return out;
}

std::array<double, 4> orientation(const Screen& s) {
    // the rotation whose columns are right, up, toward (Shepperd's method)
    const double m00 = s.right.x, m01 = s.up.x, m02 = s.toward.x;
    const double m10 = s.right.y, m11 = s.up.y, m12 = s.toward.y;
    const double m20 = s.right.z, m21 = s.up.z, m22 = s.toward.z;
    const double tr = m00 + m11 + m22;
    double x, y, z, w;
    if (tr > 0) {
        const double k = std::sqrt(tr + 1.0) * 2;
        w = 0.25 * k;
        x = (m21 - m12) / k;
        y = (m02 - m20) / k;
        z = (m10 - m01) / k;
    } else if (m00 > m11 && m00 > m22) {
        const double k = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        w = (m21 - m12) / k;
        x = 0.25 * k;
        y = (m01 + m10) / k;
        z = (m02 + m20) / k;
    } else if (m11 > m22) {
        const double k = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        w = (m02 - m20) / k;
        x = (m01 + m10) / k;
        y = 0.25 * k;
        z = (m12 + m21) / k;
    } else {
        const double k = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        w = (m10 - m01) / k;
        x = (m02 + m20) / k;
        y = (m12 + m21) / k;
        z = 0.25 * k;
    }
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    return {x / n, y / n, z / n, w / n};
}

WorldDesk parseWorldDesk(const std::string& text) {
    WorldDesk d;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string kind;
        ls >> kind;
        if (kind == "screen") {
            WorldScreen s;
            if (ls >> s.monitor >> s.pos[0] >> s.pos[1] >> s.pos[2] >> s.quat[0] >> s.quat[1] >> s.quat[2] >> s.quat[3] >> s.w >> s.h &&
                s.monitor < desktop::kMaxMonitors && s.w > 0 && s.h > 0) {
                d.screens.push_back(s);
            }
        } else if (kind == "ray") {
            room::Vec3 o, e;
            if (ls >> o.x >> o.y >> o.z >> e.x >> e.y >> e.z) {
                d.rays.emplace_back(o, e);
            }
        }
    }
    return d;
}

double bottomY(const std::vector<Screen>& s) {
    double y = 1e9;
    for (const Screen& sc : s) {
        y = std::min(y, sc.center.y - sc.h / 2);
    }
    return s.empty() ? 0.0 : y;
}

namespace {

std::optional<Hit> planeHit(const Screen& s, room::Vec3 o, room::Vec3 d) {
    const double den = d.dot(s.toward);
    if (den > -1e-6) {
        return std::nullopt; // parallel, or from behind
    }
    const double t = (s.center - o).dot(s.toward) / den;
    if (t <= 1e-4) {
        return std::nullopt;
    }
    const room::Vec3 p = o + d * t - s.center;
    Hit h;
    h.monitor = s.monitor;
    h.u = p.dot(s.right) / s.w + 0.5;
    h.v = 0.5 - p.dot(s.up) / s.h;
    h.t = t;
    return h;
}

} // namespace

std::optional<Hit> hit(const std::vector<Screen>& s, room::Vec3 o, room::Vec3 d) {
    std::optional<Hit> best;
    for (const Screen& sc : s) {
        if (const auto h = planeHit(sc, o, d); h && h->u >= 0 && h->u <= 1 && h->v >= 0 && h->v <= 1 && (!best || h->t < best->t)) {
            best = h;
        }
    }
    return best;
}

std::optional<Hit> hitPlane(const std::vector<Screen>& s, std::uint32_t monitor, room::Vec3 o, room::Vec3 d) {
    for (const Screen& sc : s) {
        if (sc.monitor == monitor) {
            return planeHit(sc, o, d);
        }
    }
    return std::nullopt;
}

void Pointer::update(const std::optional<room::Vec3>& o, const std::optional<room::Vec3>& d, double trigger, double squeeze,
                     double stickY, double dt, const std::vector<Screen>& screens) {
    hit_.reset();
    if (o && d) {
        hit_ = desk::hit(screens, *o, *d);
        if (pressed_ && (!hit_ || hit_->monitor != dragMonitor_)) {
            // a drag stays on the monitor it began on, clamped to its edge
            if (auto h = hitPlane(screens, dragMonitor_, *o, *d)) {
                h->u = std::clamp(h->u, 0.0, 1.0);
                h->v = std::clamp(h->v, 0.0, 1.0);
                hit_ = h;
            }
        }
    }
    if (!pressed_ && trigger > 0.6 && hit_) {
        pressed_ = true;
        dragMonitor_ = hit_->monitor;
    } else if (pressed_ && trigger < 0.4) {
        pressed_ = false;
    }
    // a short squeeze (under 0.4 s) is a right click, on its release
    if (squeeze > 0.7) {
        squeezeHeld_ = squeezeHeld_ < 0 ? 0 : squeezeHeld_ + dt;
    } else if (squeeze < 0.4 && squeezeHeld_ >= 0) {
        if (squeezeHeld_ < 0.4 && hit_) {
            rightFrames_ = 3;
        }
        squeezeHeld_ = -1;
    }
    const Screen* sc = nullptr;
    if (hit_) {
        for (const Screen& s : screens) {
            if (s.monitor == hit_->monitor) {
                sc = &s;
            }
        }
    }
    if (hit_ && sc != nullptr && sc->pixW > 0 && sc->pixH > 0) {
        state_.on = true;
        state_.monitor = hit_->monitor;
        state_.x = static_cast<std::uint32_t>(std::clamp(hit_->u * sc->pixW, 0.0, sc->pixW - 1.0));
        state_.y = static_cast<std::uint32_t>(std::clamp(hit_->v * sc->pixH, 0.0, sc->pixH - 1.0));
    } else {
        state_.on = false;
    }
    std::uint32_t buttons = pressed_ ? 1U : 0U;
    if (rightFrames_ > 0) {
        buttons |= 2U;
        rightFrames_--;
    }
    state_.buttons = state_.on ? buttons : 0U;
    if (state_.on && std::fabs(stickY) > 0.2) {
        const double s = (std::fabs(stickY) - 0.2) / 0.8 * (stickY > 0 ? 1 : -1);
        wheelAcc_ += s * 8.0 * 120.0 * dt;
        const auto whole = static_cast<std::int64_t>(wheelAcc_);
        wheel_ += whole;
        wheelAcc_ -= static_cast<double>(whole);
    }
}

} // namespace pibridge::home::desk
