// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The original Crystal's devices from PiBridge's shared memory (pimax_poseshm.h): the head and the
 *         controllers' poses and flags from a file in keepalived's layout, the controller's tip turned into the
 *         OpenXR grip, the buttons, a stale file, the Pimax button handed to the recenter.
 */

#include "../src/xrt/drivers/pimax/pimax_poseshm.h"

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "os/os_time.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CHECK(x)                                                                                                       \
	do {                                                                                                           \
		if (!(x)) {                                                                                            \
			fprintf(stderr, "FAILED line %d: %s\n", __LINE__, #x);                                         \
			return 1;                                                                                      \
		}                                                                                                      \
	} while (0)

// the Pimax driver's side, stubbed: no recenter yet; the recenter button's last state
static int g_button = -1;
void
pimax_hmd_get_recenter(struct xrt_pose *out)
{
	*out = (struct xrt_pose)XRT_POSE_IDENTITY;
}
void
pimax_hmd_recenter_button(bool down)
{
	g_button = down ? 1 : 0;
}
struct xrt_binding_profile *
pimax_ctrl_binding_profiles(bool left, size_t *out_count)
{
	(void)left;
	*out_count = 0;
	return NULL;
}

static char path[256];

static void
put(unsigned char *b, size_t off, const void *v, size_t n)
{
	memcpy(b + off, v, n);
}

//! keepalived's file: the head at (0, 1.6, 0) tracked with @p flags; the left controller optical at (-0.2, 1.1, -0.3)
//! pointing straight ahead with A/X + menu pressed; the right one IMU only; written @p age_s ago.
static void
write_file(uint32_t flags, double age_s, uint32_t left_buttons)
{
	unsigned char b[328] = {0};
	const int64_t now = (int64_t)os_monotonic_get_ns() - (int64_t)(age_s * 1e9);
	const uint64_t seq = 2;
	const uint32_t magic = 0x53504250u, version = 1, source = 1;
	put(b, 0, &seq, 8);
	put(b, 8, &magic, 4);
	put(b, 12, &version, 4);
	put(b, 16, &now, 8);
	put(b, 24, &flags, 4);
	put(b, 28, &source, 4);
	const struct
	{
		uint32_t state;
		float pos[3];
		uint32_t buttons;
		float axes[4];
	} devs[3] = {{1, {0, 1.6f, 0}, 0, {0}}, {7, {-0.2f, 1.1f, -0.3f}, left_buttons, {0.5f, 0.25f, 0.1f, -0.4f}},
	             {5, {0.2f, 1.1f, -0.3f}, 0, {0}}};
	for (int i = 0; i < 3; i++) {
		unsigned char *d = b + 32 + i * 96;
		const int32_t battery = 60;
		const float quat[4] = {0, 0, 0, 1};
		put(d, 0, &now, 8);
		put(d, 8, &devs[i].state, 4);
		put(d, 12, &battery, 4);
		put(d, 16, devs[i].pos, 12);
		put(d, 28, quat, 16);
		put(d, 68, &devs[i].buttons, 4);
		put(d, 72, devs[i].axes, 16);
	}
	put(b, 320, &seq, 8);
	FILE *f = fopen(path, "wb");
	fwrite(b, 1, sizeof(b), f);
	fclose(f);
}

int
main(void)
{
	snprintf(path, sizeof(path), "/tmp/tests_pimax_poseshm_%d", (int)getpid());
	setenv("PIMAX_POSE_SHM", path, 1);
	write_file(3, 0.0, 0x01 | 0x04 | 0x40);

	struct xrt_device *head = NULL, *ctrls[2] = {NULL, NULL};
	CHECK(pimax_poseshm_create(&head, ctrls));
	const int64_t now = (int64_t)os_monotonic_get_ns();

	// the head: tracked with its position (flags 3), where the file has it
	struct xrt_space_relation rel;
	CHECK(xrt_device_get_tracked_pose(head, XRT_INPUT_GENERIC_HEAD_POSE, now, &rel) == XRT_SUCCESS);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0);
	CHECK(fabsf(rel.pose.position.y - 1.6f) < 1e-3f);

	// the left controller: optical, the grip is the model's openxr_grip from the tip: ~9 cm behind it (+z), turned
	// 56.25 deg about x (20.6 + 35.65); the buttons
	CHECK(xrt_device_get_tracked_pose(ctrls[0], XRT_INPUT_TOUCH_GRIP_POSE, now, &rel) == XRT_SUCCESS);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0);
	struct xrt_vec3 d = {rel.pose.position.x + 0.2f, rel.pose.position.y - 1.1f, rel.pose.position.z + 0.3f};
	CHECK(m_vec3_len(d) > 0.07f && m_vec3_len(d) < 0.11f && d.z > 0.0f);
	const float angle = 2.0f * acosf(fminf(1.0f, fabsf(rel.pose.orientation.w))) * 180.0f / (float)M_PI;
	CHECK(fabsf(angle - 56.25f) < 0.1f);
	CHECK(xrt_device_update_inputs(ctrls[0]) == XRT_SUCCESS);
	CHECK(ctrls[0]->inputs[0].active && ctrls[0]->inputs[0].value.boolean);   // X click
	CHECK(ctrls[0]->inputs[4].value.boolean);                                    // menu
	CHECK(fabsf(ctrls[0]->inputs[7].value.vec1.x - 0.5f) < 1e-6f);               // trigger
	CHECK(fabsf(ctrls[0]->inputs[10].value.vec2.y + 0.4f) < 1e-6f);              // stick y
	CHECK(g_button == 1);                                                        // the Pimax button: the recenter
	bool present = false, charging = true;
	float charge = 0;
	CHECK(xrt_device_get_battery_status(ctrls[0], &present, &charging, &charge) == XRT_SUCCESS);
	CHECK(present && fabsf(charge - 0.6f) < 1e-6f);

	// the right controller: the IMU only: orientation tracked, the position not
	CHECK(xrt_device_get_tracked_pose(ctrls[1], XRT_INPUT_TOUCH_AIM_POSE, now, &rel) == XRT_SUCCESS);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) == 0);

	// Lighthouse mode (flags 0): no head pose of its own
	write_file(0, 0.0, 0);
	CHECK(xrt_device_get_tracked_pose(head, XRT_INPUT_GENERIC_HEAD_POSE, now, &rel) == XRT_SUCCESS);
	CHECK(rel.relation_flags == 0);
	CHECK(xrt_device_update_inputs(ctrls[0]) == XRT_SUCCESS && g_button == 0);

	// a stale file (keepalived gone): no poses, inputs not active
	write_file(3, 2.0, 0x01);
	CHECK(xrt_device_get_tracked_pose(ctrls[0], XRT_INPUT_TOUCH_GRIP_POSE, now, &rel) == XRT_SUCCESS);
	CHECK(rel.relation_flags == 0);
	CHECK(xrt_device_update_inputs(ctrls[0]) == XRT_SUCCESS && !ctrls[0]->inputs[0].active);

	xrt_device_destroy(&ctrls[0]);
	xrt_device_destroy(&ctrls[1]);
	xrt_device_destroy(&head);
	unlink(path);
	printf("tests_pimax_poseshm: ok\n");
	return 0;
}
