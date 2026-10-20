// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/geom.rs (MIT OR Apache-2.0).
//
// Small 3D helpers (f64): vectors, 3x3 matrices, rigid poses, Rodrigues, quaternions. Every expression keeps the
// Rust evaluation order (and the build uses -ffp-contract=off), so the results are bit-identical to pimax-cam's.
#pragma once

#include <array>

namespace pibridge::cam::geom {

/// 3-vector.
using Vec3 = std::array<double, 3>;
/// Row-major 3x3 matrix.
using Mat3 = std::array<std::array<double, 3>, 3>;

/// Identity matrix.
inline constexpr Mat3 IDENTITY{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

double dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
double norm(const Vec3& a);
/// Unit vector (zero stays zero).
Vec3 normalize(const Vec3& a);
Vec3 scale(const Vec3& a, double s);
Vec3 add(const Vec3& a, const Vec3& b);
Vec3 sub(const Vec3& a, const Vec3& b);
Vec3 mat_vec(const Mat3& m, const Vec3& v);
Mat3 mat_mul(const Mat3& a, const Mat3& b);
Mat3 transpose(const Mat3& m);
/// Column j.
Vec3 col(const Mat3& m, int j);
/// Matrix from columns.
Mat3 from_cols(const Vec3& a, const Vec3& b, const Vec3& c);
double det(const Mat3& m);
/// Largest deviation of m^T m from the identity (0 for a perfect rotation).
double orthonormality_error(const Mat3& m);
/// Rotation matrix from a rotation vector (axis * angle, radians).
Mat3 rodrigues(const Vec3& w);
/// Rotation about X by deg degrees.
Mat3 rot_x(double deg);
/// Angle between two unit-ish vectors in degrees.
double angle_deg(const Vec3& a, const Vec3& b);
/// Quaternion (x, y, z, w) of a rotation matrix.
std::array<double, 4> quat_from_mat(const Mat3& m);

/// Rigid transform p' = r * p + t. Named a_from_b: maps coordinates in frame b into frame a.
struct Pose {
    Mat3 r = IDENTITY;
    Vec3 t{0.0, 0.0, 0.0};

    /// Apply to a point.
    Vec3 apply(const Vec3& p) const { return add(mat_vec(r, p), t); }
    /// Inverse transform.
    Pose inverse() const;
    /// self o other (apply other first): with self = a_from_b, other = b_from_c gives a_from_c.
    Pose compose(const Pose& other) const;
    /// Origin of frame b expressed in frame a (for self = a_from_b).
    Vec3 origin() const { return t; }
    bool operator==(const Pose&) const = default;
};

} // namespace pibridge::cam::geom
