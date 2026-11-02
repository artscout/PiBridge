// SPDX-License-Identifier: MIT
#include "frame_layout.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pibridge::home::frame {

UiScale uiScale(const std::string& name) {
    if (name == "Large") {
        return {2.3, 1.0};
    }
    if (name == "Small") {
        return {0.85, 0.5};
    }
    if (name == "Miniature") {
        return {0.4, 0.3};
    }
    return {1.55, 0.75};
}

std::vector<Placed> place(const room::Mat34& head, const std::vector<Monitor>& mons, std::uint32_t mask, double zoom,
                          double distance, const std::string& uiScaleName) {
    std::vector<const Monitor*> shown;
    for (const Monitor& m : mons) {
        if (m.index < 32 && (mask & (1U << m.index)) != 0 && m.logicalW > 0 && m.logicalH > 0) {
            shown.push_back(&m);
        }
    }
    std::ranges::stable_sort(shown, [](const Monitor* a, const Monitor* b) { return a->x != b->x ? a->x < b->x : a->y < b->y; });
    const UiScale ui = uiScale(uiScaleName);
    const double z = std::clamp(zoom, kZoomMin, kZoomMax);
    const double d = std::clamp(distance, kDistanceMin, kDistanceMax);
    // the frame: at the eyes, level, `forward` ahead; x right, y up, z towards you
    room::Vec3 fwd = head.axis(2) * -1.0;
    fwd.y = 0;
    if (fwd.length() < 1e-6) {
        fwd = {0, 0, -1};
    }
    fwd = fwd.norm();
    const room::Vec3 up{0, 1, 0};
    const room::Vec3 right = fwd.cross(up).norm();
    const room::Vec3 toward = fwd * -1.0;
    const room::Vec3 origin = head.pos() + fwd * ui.forward;
    std::vector<Placed> out;
    // one in the middle; two: the first in the middle, the second on the right; three: + the left
    const int sides[3] = {0, 1, -1};
    for (std::size_t i = 0; i < shown.size() && i < 3; i++) {
        const int side = sides[i];
        // the monitor in the frame (its units), then the Zoom about the frame's middle and the Distance away
        const double lx = side * kSideX, ly = kCenterY, lz = side == 0 ? kCenterZ : kSideZ;
        const double yaw = -side * kSideYawDeg * std::numbers::pi / 180.0; // the right one turned towards -x
        const double fx = lx * z, fy = ly * z, fz = lz * z - d * ui.scale;
        Placed p;
        p.monitor = shown[i]->index;
        const room::Vec3 c = origin + (right * fx + up * fy + toward * fz) * ui.scale;
        // the monitor's axes: its x and z turned by yaw about up
        const room::Vec3 mx = right * std::cos(yaw) - toward * std::sin(yaw);
        const room::Vec3 mz = right * std::sin(yaw) + toward * std::cos(yaw);
        for (int r = 0; r < 3; r++) {
            const double cv[3] = {c.x, c.y, c.z};
            const double xv[3] = {mx.x, mx.y, mx.z}, yv[3] = {up.x, up.y, up.z}, zv[3] = {mz.x, mz.y, mz.z};
            p.pose.m[r] = {xv[r], yv[r], zv[r], cv[r]};
        }
        const double a = static_cast<double>(shown[i]->logicalW) / shown[i]->logicalH;
        const double w = a > 1.0 ? kMonW : kMonH * a;
        p.w = w * z * ui.scale;
        p.h = w / a * z * ui.scale;
        out.push_back(p);
    }
    return out;
}

} // namespace pibridge::home::frame
