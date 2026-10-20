// SPDX-License-Identifier: MIT
// Scripted frames for the editor tests: a head and two controllers held upright, buttons by bit.
#pragma once

#include "pibridge/room/editor.h"
#include "fixtures.h"

#include <initializer_list>
#include <memory>

namespace edtest {

using namespace pibridge::room;

inline Mat34 at(Vec3 p) {
    Mat34 m;
    m.m[0][3] = p.x;
    m.m[1][3] = p.y;
    m.m[2][3] = p.z;
    return m;
}

inline std::uint64_t bits(std::initializer_list<unsigned> b) {
    std::uint64_t v = 0;
    for (const unsigned x : b) {
        v |= 1ULL << x;
    }
    return v;
}

/// A room store in a temp dir, an editor over it, a clock the test moves.
struct Rig {
    fixtures::TempDir dir;
    std::unique_ptr<Store> store;
    std::unique_ptr<FileZoneControl> zone;
    std::unique_ptr<Editor> ed;
    double now = 100;
    int haptics = 0;
    int reloads = 0;
    Vec3 head{0, 1.7, 0.5};

    explicit Rig(const std::string& universe = "1111111111") {
        const std::string cfg = dir.sub("config");
        fixtures::write(cfg + "/chaperone_info.vrchap", fixtures::vrchap());
        auto s = Store::load({cfg}, universe);
        store = std::make_unique<Store>(std::move(*s));
        zone = std::make_unique<FileZoneControl>(cfg + "/steamvr.vrsettings", dir.sub("rooms"), [] { return std::time_t{1760000000}; });
        EditorHost host;
        host.now = [this] { return now; };
        host.wallClock = [] { return std::time_t{1760000000}; };
        host.haptic = [this](int, int, float) { haptics++; };
        host.afterSave = [this](const SaveResult&) {
            reloads++;
            return std::string("SteamVR reloads it");
        };
        host.zone = zone.get();
        ed = std::make_unique<Editor>(*store, RoomLib(dir.path + "/rooms"), host);
    }
    std::string chapFile() const { return dir.path + "/config/chaperone_info.vrchap"; }

    /// One frame with the right hand at `p` (the left one idle at the side).
    void frame(Vec3 p, std::uint64_t b = 0, Vec3 left = {-0.4, 1.0, 0.4}, std::uint64_t lb = 0) {
        FrameInput in;
        in.headValid = true;
        in.head = at(head);
        in.hands[0] = {true, at(left), true, at(left), lb};
        in.hands[1] = {true, at(p), true, at(p), b};
        ed->frame(in);
        now += 1.0 / 90;
    }
    /// Press and release a button at `p`.
    void click(Vec3 p, unsigned b) {
        frame(p);
        frame(p, bits({b}));
        frame(p);
    }
    /// Where the right controller must be for its tip to be in menu button `i`.
    Vec3 buttonPos(std::size_t i) const {
        const auto [slots, h] = ed->layout();
        const Slot& s = slots.at(i);
        const Vec3 tip = ed->menuPos + ed->menuR * s.cx + ed->menuU * s.cy;
        return tip + Vec3{0, 0, 0.09}; // the tip is 9 cm in front (-z) of an upright controller
    }
    std::size_t buttonIndex(const std::string& label) const {
        for (std::size_t i = 0; i < ed->cur.size(); i++) {
            if (ed->cur[i].label && ed->cur[i].label().starts_with(label)) {
                return i;
            }
        }
        return 999;
    }
    void pressMenu(const std::string& label) {
        if (!ed->menuOpen) {
            click({0.3, 1.0, 0.3}, bit::Menu);
        }
        const std::size_t i = buttonIndex(label);
        click(buttonPos(i), bit::Trigger);
    }
};

} // namespace edtest
