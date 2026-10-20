// SPDX-License-Identifier: MIT
// The PiBridge window in the VR home (common/pibridge_vrpanel.h has the link): a curved panel (a piece of a cylinder
// around the place you stood when it was placed) showing the app's window, a controller ray that points at it, the
// trigger as the left mouse button and the thumbstick as the wheel. Pure: the geometry, the mesh, the ray hit, the
// pointer rules and the CPU drawing for the snapshots; xrhome.cpp feeds them from OpenXR.
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/raster.h"
#include "pibridge/room/vmath.h"
#include "pibridge_vrpanel.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace pibridge::home::win {

inline constexpr double kRadius = 1.3;     // m: the cylinder's radius = the panel's distance
inline constexpr double kWidth = 1.5;      // m along the arc
inline constexpr double kMaxHeight = 1.2;  // m
inline constexpr int kSegments = 32;

struct Geometry {
    room::Vec3 axis;     // a point on the cylinder's axis (the eye at the placement)
    room::Vec3 right;    // horizontal, to the right of the panel
    room::Vec3 toward;   // horizontal, from the panel towards the viewer
    double centerY = 1.5;
    double radius = kRadius;
    double width = kWidth;   // arc length
    double height = 0.9;
    double halfArc() const { return width / 2 / radius; }
    /// The stage point at panel coordinates u (0 left .. 1 right), v (0 top .. 1 bottom).
    room::Vec3 at(double u, double v) const;
};
/// Placed in front of the head (yaw only), at the head's height a little lower, for a window of w x h pixels.
Geometry place(const room::Mat34& head, double floorY, std::uint32_t w, std::uint32_t h);
/// The same panel for another window size (the height follows the aspect).
void fitAspect(Geometry& g, std::uint32_t w, std::uint32_t h);

struct Hit {
    double u = 0, v = 0, t = 0; // panel coordinates and the distance along the ray
};
/// Where the ray (origin, unit direction) meets the panel's front, if it does.
std::optional<Hit> hit(const Geometry& g, room::Vec3 o, room::Vec3 d);

struct MeshVertex {
    float x, y, z, u, v;
};
/// The panel as triangles (2 per segment).
std::vector<MeshVertex> mesh(const Geometry& g, int segments = kSegments);

/// The pointer rules: trigger above 0.6 presses, below 0.4 releases; while pressed the position follows the ray along
/// the panel's plane even past its edge (a drag), clamped to the window; the stick's y scrolls up to 8 notches per
/// second (dead zone 0.2).
class Pointer {
public:
    /// One frame: the hit of the active hand's ray (nullopt: none), its trigger (0..1), stick y (-1..1), the frame
    /// time, the window size.
    void update(const std::optional<Hit>& h, double trigger, double stickY, double dt, std::uint32_t w, std::uint32_t hgt);
    vrpanel::Pointer state() const { return state_; }
    std::int64_t wheel() const { return wheel_; }
    bool pressed() const { return pressed_; }

private:
    vrpanel::Pointer state_;
    bool pressed_ = false;
    double wheelAcc_ = 0;
    std::int64_t wheel_ = 0;
};

/// The controller ray (to the panel or 3 m) and a cursor dot where it meets it, into `b` (the overlay lists).
void addRay(room::Batch& b, room::Vec3 o, room::Vec3 d, const std::optional<Hit>& h, const Geometry& g, bool pressed);

/// The panel drawn into an eye image with the picture `bgra` (w x h, the app's bytes), the CPU reference of the GPU
/// pass (snapshots).
void drawOnto(room::Image& img, const room::Mat34& stageFromEye, double tanL, double tanR, double tanU, double tanD,
              const Geometry& g, const std::vector<std::uint8_t>& bgra, std::uint32_t w, std::uint32_t h, double opacity = 1.0);

/// A made-up window picture (a test card in the app's colours) for the self test when no app is there.
std::vector<std::uint8_t> testCard(std::uint32_t w, std::uint32_t h);

} // namespace pibridge::home::win
