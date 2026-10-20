// SPDX-License-Identifier: MIT
// Ported from ZoneTweak geom.go, geom2.go and vmath.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// Floor-plane geometry of a room. A point is (x, z) in metres on the floor, the way OpenVR and OpenXR stage space
// see it from above: x to the right, z backwards (forward is -z), y up. Polygons are closed implicitly (the last
// point joins the first). The ZoneTweak functions keep their behaviour (tolerances, guards, fallbacks); added here:
// validity (simple polygon), orientation, centroid, Douglas-Peucker for a traced path.
#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace pibridge::room {

struct Pt2 {
    double x = 0;
    double z = 0;
    friend bool operator==(const Pt2&, const Pt2&) = default;
};
using Poly = std::vector<Pt2>;

double dist(Pt2 a, Pt2 b);
/// Signed area (shoelace): positive when the points run counter-clockwise in (x, z).
double polyArea(const Poly& p);
double polyPerimeter(const Poly& p);
/// Even-odd point in polygon.
bool pointInPoly(const Poly& p, double x, double z);
/// Distance of `p` to the segment a-b and the parameter t (0..1) of the closest point.
struct SegDist {
    double d;
    double t;
};
SegDist segDist(Pt2 p, Pt2 a, Pt2 b);
/// The dominant wall direction (radians, modulo 90 degrees): the length-squared weighted mean of 4x the edge angles.
double domAngle(const Poly& p);

/// A rotation of the plane (ZoneTweak `frame`): to() into the rotated frame, from() back.
struct Frame {
    double c = 1;
    double s = 0;
    static Frame of(double theta) { return {std::cos(theta), std::sin(theta)}; }
    Pt2 to(Pt2 p) const { return {p.x * c + p.z * s, -p.x * s + p.z * c}; }
    Pt2 from(Pt2 q) const { return {q.x * c - q.z * s, q.x * s + q.z * c}; }
};

/// Remove points that lie within `tol` of the line through their neighbours, the closest first, never below 4 points.
Poly simplify(const Poly& p, double tol);
/// Straight walls at right angles in the frame of `theta`; walls shorter than `minLen` are merged away. Empty when
/// fewer than 4 walls remain.
Poly squareUp(const Poly& p, double theta, double minLen);
/// Pull `p` square to its neighbours (within 7 degrees of the frame's axes).
Pt2 snapToNeighbors(Pt2 p, const Pt2* prev, const Pt2* next, double theta);

/// A rigid motion of the floor plane: (x, z) -> (a x + b z + tx, c x + d z + tz).
struct Rigid2 {
    double a = 1, b = 0, c = 0, d = 1, tx = 0, tz = 0;
    Pt2 apply(Pt2 p) const { return {a * p.x + b * p.z + tx, c * p.x + d * p.z + tz}; }
    Rigid2 inv() const { return {a, c, b, d, -(a * tx + c * tz), -(b * tx + d * tz)}; }
    /// First this, then `s`.
    Rigid2 then(const Rigid2& s) const {
        return {s.a * a + s.b * c, s.a * b + s.b * d, s.c * a + s.d * c, s.c * b + s.d * d,
                s.a * tx + s.b * tz + s.tx, s.c * tx + s.d * tz + s.tz};
    }
    Poly all(const Poly& p) const;
    /// The rotation angle as rigidFromYaw takes it.
    double angle() const { return std::atan2(c, a); }
    friend bool operator==(const Rigid2&, const Rigid2&) = default;
};
Rigid2 rigidFromYaw(double yaw, double x, double z);

/// How far `c` is inside the polygon (0 outside).
double distToBoundary(const Poly& p, Pt2 c);
/// The polygon moved inwards by `d` (mitre joins, with ZoneTweak's fallback at sharp corners).
Poly insetPoly(const Poly& p, double d);
bool segsCross(Pt2 a, Pt2 b, Pt2 c, Pt2 d);
/// Whether the axis-aligned rectangle +-hw x +-hh around the origin lies inside the polygon.
bool rectFits(const Poly& p, double hw, double hh);
/// The largest axis-aligned rectangle centred on the origin inside the polygon (searched over aspect ratios), minus
/// `margin` on each side, at least 0.5 m. nullopt when the origin is outside.
struct Size2 {
    double w;
    double h;
};
std::optional<Size2> fitArea(const Poly& p, double margin);
/// Distance of `c` to the nearest edge (inside or outside).
double outsideDist(const Poly& p, Pt2 c);

// ---------------------------------------------------------------- added for PiBridge

/// Area centroid (ZoneTweak's CENTER IN ROOM); nullopt for a degenerate polygon.
std::optional<Pt2> centroid(const Poly& p);
/// At least 3 points, no two consecutive points closer than 1 cm, no two non-adjacent edges crossing, non-zero area.
bool isSimple(const Poly& p);
/// The same polygon counter-clockwise in (x, z) (polyArea > 0).
Poly ccw(const Poly& p);
/// Douglas-Peucker on an open path: keeps the first and last point and every point further than `tol` from the
/// simplified line.
Poly douglasPeucker(const Poly& path, double tol);
/// A closed boundary from a walked path: points closer than 5 cm to the last kept one are dropped, the path is
/// simplified with `tol`, the closing duplicate removed and the result made counter-clockwise. Empty when it is not a
/// simple polygon with at least 3 corners.
Poly polygonFromTrace(const Poly& path, double tol);
/// A w x d rectangle centred on the origin (counter-clockwise).
Poly rectangle(double w, double d);

} // namespace pibridge::room
