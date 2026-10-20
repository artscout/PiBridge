// SPDX-License-Identifier: MIT
#include "recenter.h"

#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>

namespace pibridge::roomsetup {

namespace {

constexpr std::uint32_t kMagic = 0x434E4150;
constexpr std::size_t kHdr = 16, kSlots = 128, kSlot = 40;

template <typename T>
T le(std::string_view b, std::size_t o) {
    T v{};
    std::memcpy(&v, b.data() + o, sizeof v); // the ring is little endian, as this machine
    return v;
}

bool pressed(std::uint64_t now, std::uint64_t prev, unsigned bit) {
    return (now & (1ULL << bit)) != 0 && (prev & (1ULL << bit)) == 0;
}

} // namespace

std::optional<RingSample> parseRing(std::string_view b) {
    if (b.size() < kHdr + kSlots * kSlot || le<std::uint32_t>(b, 0) != kMagic || le<std::uint32_t>(b, 4) != 1 ||
        (le<std::uint32_t>(b, 8) & 1u) != 0) {
        return std::nullopt;
    }
    std::optional<RingSample> best;
    for (std::size_t i = 0; i < kSlots; i++) {
        const std::size_t o = kHdr + i * kSlot;
        const auto ts = le<std::uint64_t>(b, o);
        if (ts == 0 || (le<std::uint32_t>(b, o + 36) & 1u) == 0 || (best && best->tsNs >= ts)) {
            continue;
        }
        RingSample s;
        s.pid = le<std::uint32_t>(b, 12);
        s.tsNs = ts;
        for (std::size_t k = 0; k < 7; k++) {
            s.pose[k] = le<float>(b, o + 8 + 4 * k);
        }
        best = s;
    }
    return best;
}

std::optional<RingSample> readRing(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        return std::nullopt;
    }
    const std::string b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parseRing(b);
}

std::string recenterResultText(const RecenterResult& r) {
    const auto& p = r.head.pose;
    return std::format("pibridge-recenter 1\npid {}\nhead {:.6f} {:.6f} {:.6f} {:.7f} {:.7f} {:.7f} {:.7f}\nfloor {}\n", r.head.pid,
                       p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                       r.floorStage ? std::format("{:.4f}", *r.floorStage) : std::string("keep"));
}

void RecenterFlow::frame(const room::FrameInput& in, const std::function<std::optional<RingSample>()>& ring) {
    if (!placed_ && in.headValid) {
        // 1.4 m in front of the head, level, facing it
        room::Vec3 f = in.head.axis(2) * -1.0;
        f.y = 0;
        const double n = std::hypot(f.x, f.z);
        if (n > 1e-6) {
            f = f * (1.0 / n);
            at_ = in.head.pos() + f * 1.4;
            right_ = {-f.z, 0, f.x};
            placed_ = true;
        }
    }
    for (std::size_t i = 0; i < 2; i++) {
        const room::HandInput& h = in.hands[i];
        const std::uint64_t now = h.buttons;
        const std::uint64_t prev = prev_[i];
        prev_[i] = now;
        if (step_ == Step::Centre && pressed(now, prev, room::bit::Trigger)) {
            const auto s = ring ? ring() : std::nullopt;
            if (!s || s->pid == 0) {
                error_ = "NO HEAD POSE FROM MONADO. IS ROOM ANCHOR ON (TRACKING PAGE)?";
                continue;
            }
            error_.clear();
            head_ = *s;
            step_ = Step::Floor;
        } else if (step_ == Step::Floor && pressed(now, prev, room::bit::Menu)) {
            result_ = RecenterResult{head_, std::nullopt};
            step_ = Step::Done;
        } else if (step_ == Step::Floor && pressed(now, prev, room::bit::Trigger) && h.valid) {
            // the floor touch is checked against the head, not against the stage's y = 0: the stage floor is what
            // is being fixed and can be off by more than a metre (live 2026-10-19: "not the floor" on the real floor)
            const double y = h.grip.pos().y;
            if (!recenterFloorPlausible(in.headValid, in.head.pos().y, y)) {
                error_ = "THAT IS NOT THE FLOOR. TRY AGAIN, OR PRESS B TO KEEP IT.";
                continue;
            }
            result_ = RecenterResult{head_, y};
            step_ = Step::Done;
        }
    }
}

bool recenterFloorPlausible(bool headValid, double headY, double controllerY) {
    if (!headValid) {
        return std::fabs(controllerY) <= 1.5; // no head: the old stage check
    }
    const double below = headY - controllerY;
    return below >= kFloorMinBelowHead && below <= kFloorMaxBelowHead;
}

std::vector<std::string> RecenterFlow::lines() const {
    std::vector<std::string> l;
    switch (step_) {
    case Step::Centre:
        l = {"RECENTER", "STAND WHERE THE PLAY AREA CENTRE", "SHOULD BE. LOOK FORWARD.", "PULL THE TRIGGER."};
        break;
    case Step::Floor:
        l = {"FLOOR", "TOUCH THE FLOOR WITH A CONTROLLER", "AND PULL ITS TRIGGER,", "OR PRESS B / MENU TO KEEP IT."};
        break;
    case Step::Done: l = {"DONE"}; break;
    }
    if (!error_.empty()) {
        l.push_back(error_);
    }
    return l;
}

void RecenterFlow::build(room::Batch& b) const {
    b.clear();
    const room::Vec3 up{0, 1, 0};
    double y = 0.12;
    for (const std::string& s : lines()) {
        room::drawTextCentered(b, s, at_ + up * y, right_, up, 0.008, {0.9f, 0.95f, 1.0f, 1.0f});
        y -= 0.08;
    }
}

} // namespace pibridge::roomsetup
