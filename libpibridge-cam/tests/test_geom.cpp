// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/geom.rs (MIT OR Apache-2.0).
#include "pibridge/cam/geom.h"
#include "pibridge_test.h"

#include <cmath>
#include <numbers>

using namespace pibridge::cam::geom;

TEST(rodrigues_basic) {
    const Mat3 r = rodrigues({0.0, 0.0, std::numbers::pi / 2});
    const Vec3 v = mat_vec(r, {1.0, 0.0, 0.0});
    CHECK_NEAR(v[0], 0.0, 1e-12);
    CHECK_NEAR(v[1], 1.0, 1e-12);
    CHECK(orthonormality_error(r) < 1e-12);
    CHECK_NEAR(det(r), 1.0, 1e-12);
}

TEST(pose_inverse_compose) {
    const Pose a{rodrigues({0.1, -0.7, 0.3}), {0.1, 0.2, -0.3}};
    const Pose i = a.compose(a.inverse());
    CHECK(orthonormality_error(i.r) < 1e-12 && norm(i.t) < 1e-12);
    const Vec3 p{0.3, -1.0, 2.0};
    const Vec3 q = a.inverse().apply(a.apply(p));
    CHECK(norm(sub(p, q)) < 1e-12);
}

TEST(quaternion_roundtrip) {
    for (const Vec3& w : {Vec3{0.1, 0.2, 0.3}, Vec3{3.0, 0.0, 0.0}, Vec3{0.0, -3.1, 0.2}, Vec3{-0.07417, -2.57303, 0.62804}}) {
        const Mat3 m = rodrigues(w);
        const auto q = quat_from_mat(m);
        // rotate a vector with q and with m
        const Vec3 v{0.3, -0.5, 0.8};
        const Vec3 u{q[0], q[1], q[2]};
        const Vec3 uv = cross(u, v);
        const Vec3 uuv = cross(u, uv);
        const Vec3 rv = add(v, add(scale(uv, 2.0 * q[3]), scale(uuv, 2.0)));
        CHECK(norm(sub(rv, mat_vec(m, v))) < 1e-9);
    }
}

PIBRIDGE_TEST_MAIN()
