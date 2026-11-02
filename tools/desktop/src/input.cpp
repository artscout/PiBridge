// SPDX-License-Identifier: MIT
#include "input.h"

namespace pibridge::desktopsvc {

namespace {

constexpr int kButtons[3] = {kBtnLeft, kBtnRight, kBtnMiddle};

} // namespace

std::vector<Event> InputDiff::update(const desktop::Pointer& p, std::int64_t wheel,
                                     const std::vector<desktop::MonitorInfo>& mons) {
    std::vector<Event> out;
    if (!started_) {
        started_ = true;
        wheel_ = wheel;
    }
    const desktop::MonitorInfo* m = nullptr;
    if (p.on) {
        for (const auto& mi : mons) {
            if (mi.index == p.monitor && mi.width > 0 && mi.height > 0 && mi.logicalW > 0 && mi.logicalH > 0) {
                m = &mi;
            }
        }
    }
    if (m == nullptr) {
        // off every monitor: let go of what is held, the wheel's notches are dropped
        for (int b = 0; b < 3; b++) {
            if ((held_ & (1U << b)) != 0) {
                out.push_back({Event::Kind::Button, 0, 0, 0, kButtons[b], false, 0});
            }
        }
        held_ = 0;
        wheel_ = wheel;
        last_ = p;
        return out;
    }
    if (!(last_.on && last_.monitor == p.monitor && last_.x == p.x && last_.y == p.y)) {
        const double x = (static_cast<double>(p.x) + 0.5) * m->logicalW / m->width;
        const double y = (static_cast<double>(p.y) + 0.5) * m->logicalH / m->height;
        out.push_back({Event::Kind::Move, p.monitor, x, y, 0, false, 0});
    }
    for (int b = 0; b < 3; b++) {
        const bool want = (p.buttons & (1U << b)) != 0;
        const bool has = (held_ & (1U << b)) != 0;
        if (want != has) {
            out.push_back({Event::Kind::Button, 0, 0, 0, kButtons[b], want, 0});
            held_ ^= 1U << b;
        }
    }
    const std::int64_t notches = (wheel - wheel_) / 120;
    if (notches != 0) {
        wheel_ += notches * 120;
        out.push_back({Event::Kind::Wheel, 0, 0, 0, 0, false, static_cast<int>(-notches)}); // + total = up = - steps
    }
    last_ = p;
    return out;
}

} // namespace pibridge::desktopsvc
