// Copyright 2026, PiBridge.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The local spaces follow a driver's recenter (u_recenter_announce), each app's own LOCAL too.
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_space.h"
#include "xrt/xrt_session.h"
#include "xrt/xrt_tracking.h"
#include "math/m_api.h"
#include "util/u_recenter.h"
#include "b_space_overseer.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstring>

namespace {

struct FakeHead
{
	struct xrt_device base;
	struct xrt_pose pose;
};

xrt_result_t
fakeTrackedPose(struct xrt_device *xdev, enum xrt_input_name, int64_t, struct xrt_space_relation *out)
{
	auto *h = reinterpret_cast<FakeHead *>(xdev);
	*out = XRT_SPACE_RELATION_ZERO;
	out->pose = h->pose;
	out->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	return XRT_SUCCESS;
}

struct CountingSink
{
	struct xrt_session_event_sink base;
	int count = 0;
};

xrt_result_t
countEvent(struct xrt_session_event_sink *xses, const union xrt_session_event *)
{
	reinterpret_cast<CountingSink *>(xses)->count++;
	return XRT_SUCCESS;
}

struct xrt_pose
headAt(float x, float y, float z, float yawDeg)
{
	const float a = yawDeg * float(M_PI) / 180.f;
	struct xrt_pose p = XRT_POSE_IDENTITY;
	p.orientation = {0, std::sin(a / 2), 0, std::cos(a / 2)};
	p.position = {x, y, z};
	return p;
}

float
yawOf(const struct xrt_quat &q)
{
	const struct xrt_vec3 mz = {0, 0, -1};
	struct xrt_vec3 f;
	math_quat_rotate_vec3(&q, &mz, &f);
	return std::atan2(-f.x, -f.z) * 180.f / float(M_PI);
}

} // namespace

TEST_CASE("app local spaces follow a driver recenter")
{
	struct xrt_tracking_origin origin = {};
	std::strcpy(origin.name, "test");
	origin.type = XRT_TRACKING_TYPE_OTHER;
	origin.initial_offset = XRT_POSE_IDENTITY;

	FakeHead head = {};
	head.base.name = XRT_DEVICE_GENERIC_HMD;
	head.base.device_type = XRT_DEVICE_TYPE_HMD;
	head.base.tracking_origin = &origin;
	head.base.get_tracked_pose = fakeTrackedPose;
	std::strcpy(head.base.str, "fake head");
	head.pose = headAt(1.f, 1.2f, 2.f, 90.f);

	CountingSink sink;
	sink.base.push_event = countEvent;

	struct b_space_overseer *uso = b_space_overseer_create(&sink.base);
	struct xrt_device *xdevs[1] = {&head.base};
	const struct xrt_pose localOffset = {XRT_QUAT_IDENTITY, {0, 1.6f, 0}};
	b_space_overseer_legacy_setup(uso, xdevs, 1, &head.base, &localOffset, false, true);
	auto *xso = reinterpret_cast<struct xrt_space_overseer *>(uso);

	struct xrt_space *local = nullptr;
	struct xrt_space *floor = nullptr;
	REQUIRE(xrt_space_overseer_create_local_space(xso, &local, &floor) == XRT_SUCCESS);

	const struct xrt_pose id = XRT_POSE_IDENTITY;
	auto headIn = [&](struct xrt_space *base) {
		struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
		REQUIRE(xrt_space_overseer_locate_space(xso, base, &id, 0, xso->semantic.view, &id, &rel) ==
		        XRT_SUCCESS);
		return rel.pose;
	};

	// made where the head was: the head at its origin, facing its forward
	struct xrt_pose p = headIn(local);
	CHECK(std::fabs(p.position.x) < 1e-4f);
	CHECK(std::fabs(p.position.z) < 1e-4f);
	CHECK(std::fabs(yawOf(p.orientation)) < 0.01f);

	// the head moves and turns (the driver's recenter moves the whole tracking), the app's LOCAL stays behind
	head.pose = headAt(-0.5f, 1.1f, 0.3f, -40.f);
	p = headIn(local);
	CHECK(std::fabs(yawOf(p.orientation) - (-130.f)) < 0.05f);

	// the announcement: the next locate puts LOCAL back onto the head
	const int before = sink.count;
	u_recenter_announce();
	p = headIn(local);
	CHECK(std::fabs(p.position.x) < 1e-4f);
	CHECK(std::fabs(p.position.y) < 1e-4f);
	CHECK(std::fabs(p.position.z) < 1e-4f);
	CHECK(std::fabs(yawOf(p.orientation)) < 0.01f);
	CHECK(sink.count > before); // the apps hear of the change (reference space change pending)

	// LOCAL_FLOOR: same place and yaw, on the floor
	p = headIn(floor);
	CHECK(std::fabs(p.position.x) < 1e-4f);
	CHECK(std::fabs(p.position.y - 1.1f) < 1e-4f);
	CHECK(std::fabs(yawOf(p.orientation)) < 0.01f);

	// the shared LOCAL follows too (yaw and the horizontal place, its height kept)
	p = headIn(xso->semantic.local);
	CHECK(std::fabs(p.position.x) < 1e-4f);
	CHECK(std::fabs(yawOf(p.orientation)) < 0.01f);

	// a destroyed app space leaves the list; a later recenter does not touch freed memory
	xrt_space_reference(&local, nullptr);
	xrt_space_reference(&floor, nullptr);
	head.pose = headAt(0.2f, 1.0f, 0.2f, 10.f);
	u_recenter_announce();
	(void)headIn(xso->semantic.local);

	xrt_space_overseer_destroy(&xso);
}
