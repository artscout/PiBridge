// SPDX-License-Identifier: MIT
#include "pibridge/room/pose.h"

#include <algorithm>
#include <cmath>

namespace pibridge::room {

FakePoseSource::FakePoseSource(Poly room, double speed, double startDelay)
    : room_(ccw(std::move(room))), speed_(speed), delay_(startDelay) {
    lap_ = polyPerimeter(room_);
}

Pt2 FakePoseSource::at(double s) const {
    if (room_.empty()) {
        return {};
    }
    s = std::fmod(std::max(s, 0.0), std::max(lap_, 1e-9));
    for (std::size_t i = 0; i < room_.size(); i++) {
        const Pt2 a = room_[i];
        const Pt2 b = room_[(i + 1) % room_.size()];
        const double l = dist(a, b);
        if (s <= l || i + 1 == room_.size()) {
            const double t = l > 0 ? std::min(s / l, 1.0) : 0;
            return {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        }
        s -= l;
    }
    return room_.front();
}

std::optional<PoseSample> FakePoseSource::sample(double t) {
    if (room_.size() < 3) {
        return std::nullopt;
    }
    PoseSample p;
    p.t = t;
    const double walked = std::clamp((t - delay_) * speed_, 0.0, lap_);
    const bool tracing = t >= delay_ && (t - delay_) * speed_ < lap_;
    // the walker goes round 0.3 m inside the walls, the controller is held out to the wall
    const Pt2 onWall = at(walked);
    const Pt2 ahead = at(walked + 0.05);
    Vec3 dir{ahead.x - onWall.x, 0, ahead.z - onWall.z};
    dir = dir.norm();
    // inwards for a counter-clockwise polygon in (x, z): the left normal (-dz, dx)
    const Vec3 in{-dir.z, 0, dir.x};
    const Pt2 headAt{onWall.x + in.x * 0.3, onWall.z + in.z * 0.3};
    p.head = {true, {headAt.x, 1.7, headAt.z}, dir.length() > 0.5 ? dir : Vec3{0, 0, -1}};
    p.hands[1] = {true, {onWall.x, 1.0, onWall.z}, dir};
    p.hands[0] = {true, {headAt.x - dir.z * 0.2, 1.0, headAt.z + dir.x * 0.2}, dir};
    p.trigger[1] = tracing;
    return p;
}

void TraceRecorder::add(Pt2 p) {
    if (!running_) {
        return;
    }
    if (path_.empty() || dist(path_.back(), p) >= 0.05) {
        path_.push_back(p);
    }
}

Poly TraceRecorder::finish(double tol) {
    running_ = false;
    return polygonFromTrace(path_, tol);
}

std::optional<Poly> TraceFollow::step(const PoseSample& s, int hand) {
    const auto h = static_cast<std::size_t>(hand);
    const bool down = s.trigger[h] && s.hands[h].valid;
    std::optional<Poly> out;
    if (down && !wasDown) {
        rec.start();
    }
    if (down) {
        rec.add(flat(s.hands[h].pos));
    }
    if (!down && wasDown) {
        out = rec.finish();
    }
    wasDown = down;
    return out;
}

} // namespace pibridge::room
