// SPDX-License-Identifier: MIT
// The desktop in the headset (docs/desktop.md), the overlay's side (pure: no runtime; xrhome.cpp feeds it): the shown
// monitors as flat screens on an arc in front of you, in their desktop order and at their desktop sizes (one desktop
// pixel = kMetresPerPixel), each turned to face you; where a controller ray meets one, in that monitor's pixels; and
// the pointer rules: the trigger is the left button (a drag keeps going past the edge of the monitor it began on), a
// short squeeze is a right click, the stick scrolls.
#pragma once

#include "pibridge/room/vmath.h"
#include "pibridge_desktop.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::home::desk {

inline constexpr double kDistance = 1.6;              // m: the arc's radius
inline constexpr double kMetresPerPixel = 1.3 / 2560; // a 2560-pixel monitor is 1.3 m wide (~44 degrees at 1.6 m)
inline constexpr double kGap = 0.04;                  // m between neighbouring monitors

/// The monitors to show: one of them, or all (a bit per monitor, as desktop.shm's `want`).
std::uint32_t maskOne(std::uint32_t monitor);
std::uint32_t maskAll(const std::vector<desktop::MonitorInfo>& mons);

struct Screen {
    std::uint32_t monitor = 0;
    room::Vec3 center;
    room::Vec3 right;  // unit, along the picture's rows
    room::Vec3 up;     // unit, against the picture's columns
    room::Vec3 toward; // unit, from the screen to the viewer (the quad's +Z)
    double w = 0, h = 0; // metres
    std::uint32_t pixW = 0, pixH = 0; // the picture's pixels
    /// The stage point at u (0 left .. 1 right), v (0 top .. 1 bottom).
    room::Vec3 at(double u, double v) const { return center + right * ((u - 0.5) * w) + up * ((0.5 - v) * h); }
};

/// Where the shown monitors go: on an arc around the head's place (yaw only), the arc's middle straight ahead, the
/// screens' middle a little below the eyes. `scale` makes them bigger or smaller (the distance stays).
std::vector<Screen> place(const room::Mat34& head, double floorY, const std::vector<desktop::MonitorInfo>& mons,
                          std::uint32_t mask, double scale = 1.0);
/// The screen's orientation as a quaternion (x, y, z, w): its right, up and toward as the quad's x, y and z.
std::array<double, 4> orientation(const Screen& s);
/// The lowest edge of the screens (stage y), for the bar under them.
double bottomY(const std::vector<Screen>& s);

struct Hit {
    std::uint32_t monitor = 0;
    double u = 0, v = 0, t = 0;
};
/// The nearest screen the ray (origin, unit direction) meets from its front.
std::optional<Hit> hit(const std::vector<Screen>& s, room::Vec3 o, room::Vec3 d);
/// Where the ray meets the plane of screen `monitor` (u, v may lie outside 0..1): a drag past the edge.
std::optional<Hit> hitPlane(const std::vector<Screen>& s, std::uint32_t monitor, room::Vec3 o, room::Vec3 d);

/// PiBridge World's desktop (world/scripts/desktop.gd): the world places the monitors (the frame's places, its
/// zoom and distance) and drives them with its lasers; this overlay only draws them, as quad layers (a GPU upload in
/// this process instead of the world's main thread), and the lasers over them. The world writes
/// $XDG_RUNTIME_DIR/pibridge/world-desktop every frame while the desktop is shown:
///   screen <monitor> <px> <py> <pz> <qx> <qy> <qz> <qw> <width m> <height m>     (the stage = the world's space)
///   ray <ox> <oy> <oz> <ex> <ey> <ez>                                            (a laser on a screen)
struct WorldScreen {
    std::uint32_t monitor = 0;
    std::array<double, 3> pos{};
    std::array<double, 4> quat{0, 0, 0, 1};
    double w = 0, h = 0;
};
struct WorldDesk {
    std::vector<WorldScreen> screens;
    std::vector<std::pair<room::Vec3, room::Vec3>> rays;
};
WorldDesk parseWorldDesk(const std::string& text);

class Pointer {
public:
    /// One frame: the active hand's ray (nullopt: none), its trigger (0..1), its squeeze (0..1), stick y (-1..1), the
    /// frame time; the screens and their pictures.
    void update(const std::optional<room::Vec3>& o, const std::optional<room::Vec3>& d, double trigger, double squeeze,
                double stickY, double dt, const std::vector<Screen>& screens);
    desktop::Pointer state() const { return state_; }
    std::int64_t wheel() const { return wheel_; }
    bool pressed() const { return pressed_; }
    /// The hit drawn this frame (the ray's length, the cursor).
    const std::optional<Hit>& hit() const { return hit_; }

private:
    desktop::Pointer state_;
    std::optional<Hit> hit_;
    bool pressed_ = false;
    std::uint32_t dragMonitor_ = 0;
    double squeezeHeld_ = -1; // seconds the squeeze has been held (-1: not held)
    int rightFrames_ = 0;     // a right click: the button stays down this many frames
    double wheelAcc_ = 0;
    std::int64_t wheel_ = 0;
};

} // namespace pibridge::home::desk
