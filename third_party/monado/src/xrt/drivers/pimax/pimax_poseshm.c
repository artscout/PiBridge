// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The original Pimax Crystal's head and controllers from PiBridge's shared memory (pimax_poseshm.h).
 *
 * The file (little endian, 328 bytes; PiBridge libpibridge-hmd pose_shm.h is the writer's side):
 *     0 u64 seq_begin, 8 u32 magic "PBPS", 12 u32 version 1, 16 i64 written (CLOCK_MONOTONIC ns),
 *    24 u32 the headset's own tracking flags (1 orientation, 2 position; 0 in Lighthouse mode), 28 u32 source,
 *    32 three devices (head, left, right) of 96 bytes: +0 i64 sample time, +8 u32 state (head 1 tracked; controller
 *       7 optical, 5 IMU only, 4 no pose, 0 never), +12 i32 battery %, +16 f32x3 position, +28 f32x4 orientation
 *       x y z w, +44 f32x3 linear velocity, +56 f32x3 angular velocity, +68 u32 buttons (controllers: pressed
 *       0x01 A/X, 0x02 B/Y, 0x04 menu, 0x08 stick, 0x40 Pimax | touched << 8: 0x01 A/X, 0x02 B/Y, 0x08 stick,
 *       0x10 trigger, 0x20 grip), +72 f32x4 trigger, grip, stick x, stick y, +88 u64 sample number
 *   320 u64 seq_end
 * A body is consistent when seq_end, read first, equals seq_begin, read last, and is even.
 *
 * The headset reports a controller's tip (the Crystal controller model's "tip" frame); the OpenXR grip and aim are
 * the model's openxr_grip / openxr_aim frames (crystal_controller_{left,right}.json: tip (0, -0.019, 0.014) rot x
 * -35.65; openxr_grip (+-0.007, -0.0018294, 0.1019482) rot x 20.6; openxr_aim (+-0.007, -0.0389477, 0.0094969) rot x
 * -39.4; x mirrored on the right).
 *
 * @ingroup drv_pimax
 */

#include "pimax_poseshm.h"
#include "pimax_interface.h"

#include "math/m_api.h"
#include "math/m_predict.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

DEBUG_GET_ONCE_OPTION(pimax_pose_shm, "PIMAX_POSE_SHM", "/dev/shm/pibridge-crystal-pose")

#define SHM_SIZE 328
#define SHM_MAGIC 0x53504250u
#define SHM_STALE_NS (500ll * 1000 * 1000) // the file older than this: the headset or keepalived is gone
#define PREDICT_MAX_S 0.1                  // never extrapolate further (a clock step must not throw the pose away)

struct shm_device
{
	int64_t t_ns;
	uint32_t state;
	int32_t battery;
	float pos[3], quat[4], lin[3], ang[3];
	uint32_t buttons;
	float axes[4];
};

struct shm_state
{
	int64_t written_ns;
	uint32_t tracking;
	struct shm_device dev[3]; // head, left, right
};

//! The file, shared by the three devices (opened on demand).
static struct
{
	pthread_mutex_t lock;
	int fd;
	int64_t retry_ns;
} g_shm = {PTHREAD_MUTEX_INITIALIZER, -1, 0};

static struct xrt_tracking_origin g_origin = {
    .name = "Pimax Crystal (its own inside-out)",
    .type = XRT_TRACKING_TYPE_OTHER,
    .initial_offset = XRT_POSE_IDENTITY,
};

static uint32_t
rd_u32(const uint8_t *p)
{
	uint32_t v;
	memcpy(&v, p, 4);
	return v;
}

static int64_t
rd_i64(const uint8_t *p)
{
	int64_t v;
	memcpy(&v, p, 8);
	return v;
}

static float
rd_f32(const uint8_t *p)
{
	float v;
	memcpy(&v, p, 4);
	return v;
}

//! A consistent snapshot of the file; false: no file, torn twice, or not this layout.
static bool
shm_read(struct shm_state *out)
{
	uint8_t b[SHM_SIZE];
	pthread_mutex_lock(&g_shm.lock);
	const int64_t now = (int64_t)os_monotonic_get_ns();
	if (g_shm.fd < 0 && now >= g_shm.retry_ns) {
		g_shm.fd = open(debug_get_option_pimax_pose_shm(), O_RDONLY | O_CLOEXEC);
		g_shm.retry_ns = now + 1000ll * 1000 * 1000; // looked for again once a second
	}
	bool ok = false;
	for (int attempt = 0; g_shm.fd >= 0 && attempt < 2 && !ok; attempt++) {
		uint8_t end[8], begin[8];
		if (pread(g_shm.fd, end, 8, 320) != 8 || pread(g_shm.fd, b, SHM_SIZE, 0) != SHM_SIZE ||
		    pread(g_shm.fd, begin, 8, 0) != 8) {
			close(g_shm.fd); // replaced (keepalived restarted): open it again
			g_shm.fd = -1;
			break;
		}
		ok = memcmp(end, begin, 8) == 0 && (rd_i64(end) & 1) == 0;
	}
	pthread_mutex_unlock(&g_shm.lock);
	if (!ok || rd_u32(b + 8) != SHM_MAGIC || rd_u32(b + 12) != 1) {
		return false;
	}
	out->written_ns = rd_i64(b + 16);
	out->tracking = rd_u32(b + 24);
	for (int i = 0; i < 3; i++) {
		const uint8_t *d = b + 32 + i * 96;
		struct shm_device *o = &out->dev[i];
		o->t_ns = rd_i64(d);
		o->state = rd_u32(d + 8);
		o->battery = (int32_t)rd_u32(d + 12);
		for (int k = 0; k < 3; k++) {
			o->pos[k] = rd_f32(d + 16 + 4 * k);
			o->lin[k] = rd_f32(d + 44 + 4 * k);
			o->ang[k] = rd_f32(d + 56 + 4 * k);
		}
		for (int k = 0; k < 4; k++) {
			o->quat[k] = rd_f32(d + 28 + 4 * k);
			o->axes[k] = rd_f32(d + 72 + 4 * k);
		}
		o->buttons = rd_u32(d + 68);
	}
	return true;
}

static bool
shm_fresh(const struct shm_state *s)
{
	return (int64_t)os_monotonic_get_ns() - s->written_ns < SHM_STALE_NS;
}

//! The device's pose at @p at_ns (extrapolated from its sample, at most PREDICT_MAX_S), with these flags.
static void
relation_of(const struct shm_device *d, uint32_t flags, int64_t at_ns, struct xrt_space_relation *out)
{
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose.position = (struct xrt_vec3){d->pos[0], d->pos[1], d->pos[2]};
	rel.pose.orientation = (struct xrt_quat){d->quat[0], d->quat[1], d->quat[2], d->quat[3]};
	math_quat_normalize(&rel.pose.orientation);
	rel.angular_velocity = (struct xrt_vec3){d->ang[0], d->ang[1], d->ang[2]};
	if ((flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
		rel.linear_velocity = (struct xrt_vec3){d->lin[0], d->lin[1], d->lin[2]};
		flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
	}
	rel.relation_flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	double dt = (double)(at_ns - d->t_ns) / 1e9;
	dt = dt < -PREDICT_MAX_S ? -PREDICT_MAX_S : (dt > PREDICT_MAX_S ? PREDICT_MAX_S : dt);
	m_predict_relation(&rel, dt, out);
}

//! The head's manual recenter on top (as the head gets it from the Pimax HMD).
static void
apply_recenter(struct xrt_space_relation *rel)
{
	struct xrt_pose w;
	pimax_hmd_get_recenter(&w);
	math_pose_transform(&w, &rel->pose, &rel->pose);
	math_quat_rotate_vec3(&w.orientation, &rel->linear_velocity, &rel->linear_velocity);
	math_quat_rotate_vec3(&w.orientation, &rel->angular_velocity, &rel->angular_velocity);
}


/*
 *
 * The head.
 *
 */

static xrt_result_t
head_get_tracked_pose(struct xrt_device *xdev, enum xrt_input_name name, int64_t at_ns, struct xrt_space_relation *out)
{
	(void)xdev;
	*out = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	struct shm_state s;
	if (name != XRT_INPUT_GENERIC_HEAD_POSE || !shm_read(&s) || !shm_fresh(&s)) {
		return XRT_SUCCESS;
	}
	// its own inside-out: orientation (flag 1), and the position with flag 2; nothing in Lighthouse mode
	const struct shm_device *h = &s.dev[0];
	if (h->state != 1 || (s.tracking & 1) == 0) {
		return XRT_SUCCESS;
	}
	uint32_t flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                 XRT_SPACE_RELATION_POSITION_VALID_BIT;
	if ((s.tracking & 2) != 0) {
		flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	relation_of(h, flags, at_ns, out);
	return XRT_SUCCESS;
}

static void
head_destroy(struct xrt_device *xdev)
{
	u_device_free(xdev);
}


/*
 *
 * The controllers (Touch).
 *
 */

enum
{
	C_BTN1_CLICK, // left X / right A
	C_BTN1_TOUCH,
	C_BTN2_CLICK, // left Y / right B
	C_BTN2_TOUCH,
	C_MENU, // left MENU_CLICK, right SYSTEM_CLICK
	C_SQUEEZE,
	C_TRIGGER_TOUCH,
	C_TRIGGER_VALUE,
	C_STICK_CLICK,
	C_STICK_TOUCH,
	C_STICK,
	C_GRIP_POSE,
	C_AIM_POSE,
	C_COUNT,
};

struct shm_controller
{
	struct xrt_device base;
	int index; // 0 left, 1 right
	struct xrt_pose tip_to_grip, tip_to_aim;
};

//! Whether the Pimax button is held, per hand (either one held recenters, timed by the Pimax HMD).
static atomic_bool g_pimax_button[2];

static struct xrt_pose
model_frame(float x, float y, float z, float rot_x_deg)
{
	struct xrt_pose p = XRT_POSE_IDENTITY;
	p.position = (struct xrt_vec3){x, y, z};
	const struct xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector(rot_x_deg * (float)M_PI / 180.0f, &axis, &p.orientation);
	return p;
}

static xrt_result_t
ctl_update_inputs(struct xrt_device *xdev)
{
	struct shm_controller *c = (struct shm_controller *)xdev;
	struct xrt_input *in = c->base.inputs;
	struct shm_state s;
	const bool have = shm_read(&s) && shm_fresh(&s);
	const struct shm_device *d = have ? &s.dev[1 + c->index] : NULL;
	const bool on = d != NULL && d->state != 0 && d->state != 4;
	const int64_t now = (int64_t)os_monotonic_get_ns();
	for (int i = 0; i < C_COUNT; i++) {
		in[i].active = on;
		in[i].timestamp = now;
	}
	const uint32_t pressed = on ? d->buttons & 0xff : 0;
	const uint32_t touched = on ? (d->buttons >> 8) & 0xff : 0;
	in[C_BTN1_CLICK].value.boolean = (pressed & 0x01) != 0;
	in[C_BTN1_TOUCH].value.boolean = (touched & 0x01) != 0 || (pressed & 0x01) != 0;
	in[C_BTN2_CLICK].value.boolean = (pressed & 0x02) != 0;
	in[C_BTN2_TOUCH].value.boolean = (touched & 0x02) != 0 || (pressed & 0x02) != 0;
	in[C_MENU].value.boolean = (pressed & 0x04) != 0;
	in[C_SQUEEZE].value.vec1.x = on ? d->axes[1] : 0.0f;
	in[C_TRIGGER_VALUE].value.vec1.x = on ? d->axes[0] : 0.0f;
	in[C_TRIGGER_TOUCH].value.boolean = (touched & 0x10) != 0 || (on && d->axes[0] > 0.06f);
	in[C_STICK_CLICK].value.boolean = (pressed & 0x08) != 0;
	in[C_STICK_TOUCH].value.boolean = (touched & 0x08) != 0 || (pressed & 0x08) != 0;
	in[C_STICK].value.vec2.x = on ? d->axes[2] : 0.0f;
	in[C_STICK].value.vec2.y = on ? d->axes[3] : 0.0f;
	// the Pimax button is PiBridge's recenter (held), not an app input, as on the Crystal Super
	atomic_store(&g_pimax_button[c->index], (pressed & 0x40) != 0);
	pimax_hmd_recenter_button(atomic_load(&g_pimax_button[0]) || atomic_load(&g_pimax_button[1]));
	return XRT_SUCCESS;
}

static xrt_result_t
ctl_get_tracked_pose(struct xrt_device *xdev, enum xrt_input_name name, int64_t at_ns, struct xrt_space_relation *out)
{
	struct shm_controller *c = (struct shm_controller *)xdev;
	*out = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	if (name != XRT_INPUT_TOUCH_GRIP_POSE && name != XRT_INPUT_TOUCH_AIM_POSE) {
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}
	struct shm_state s;
	if (!shm_read(&s) || !shm_fresh(&s)) {
		return XRT_SUCCESS;
	}
	// 7: seen by the headset's cameras; 5: the IMU only (orientation, the position held); else no pose (apps hide it)
	const struct shm_device *d = &s.dev[1 + c->index];
	if (d->state != 7 && d->state != 5) {
		return XRT_SUCCESS;
	}
	uint32_t flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                 XRT_SPACE_RELATION_POSITION_VALID_BIT;
	if (d->state == 7) {
		flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	struct xrt_space_relation tip;
	relation_of(d, flags, at_ns, &tip);
	const struct xrt_pose *frame = name == XRT_INPUT_TOUCH_GRIP_POSE ? &c->tip_to_grip : &c->tip_to_aim;
	*out = tip;
	math_pose_transform(&tip.pose, frame, &out->pose);
	math_quat_normalize(&out->pose.orientation);
	apply_recenter(out);
	return XRT_SUCCESS;
}

static xrt_result_t
ctl_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct shm_controller *c = (struct shm_controller *)xdev;
	struct shm_state s;
	const bool have = shm_read(&s) && shm_fresh(&s) && s.dev[1 + c->index].battery >= 0;
	*out_present = have;
	*out_charging = false;
	*out_charge = have ? (float)s.dev[1 + c->index].battery / 100.0f : 0.0f;
	return XRT_SUCCESS;
}

static void
ctl_destroy(struct xrt_device *xdev)
{
	u_device_free(xdev);
}

static struct xrt_device *
ctl_create(int index)
{
	const bool left = index == 0;
	struct shm_controller *c = U_DEVICE_ALLOCATE(struct shm_controller, U_DEVICE_ALLOC_TRACKING_NONE, C_COUNT, 0);
	if (c == NULL) {
		return NULL;
	}
	c->index = index;
	// the Crystal controller model: tip -> raw (inverse of "tip"), raw -> openxr_grip / openxr_aim
	const float mx = left ? 1.0f : -1.0f;
	const struct xrt_pose tip = model_frame(0.0f, -0.019f, 0.014f, -35.65f);
	struct xrt_pose raw_from_tip;
	math_pose_invert(&tip, &raw_from_tip);
	const struct xrt_pose grip = model_frame(0.007f * mx, -0.00182941f, 0.1019482f, 20.6f);
	const struct xrt_pose aim = model_frame(0.007f * mx, -0.03894766f, 0.00949694f, -39.4f);
	math_pose_transform(&raw_from_tip, &grip, &c->tip_to_grip);
	math_pose_transform(&raw_from_tip, &aim, &c->tip_to_aim);

	struct xrt_input *in = c->base.inputs;
	in[C_BTN1_CLICK].name = left ? XRT_INPUT_TOUCH_X_CLICK : XRT_INPUT_TOUCH_A_CLICK;
	in[C_BTN1_TOUCH].name = left ? XRT_INPUT_TOUCH_X_TOUCH : XRT_INPUT_TOUCH_A_TOUCH;
	in[C_BTN2_CLICK].name = left ? XRT_INPUT_TOUCH_Y_CLICK : XRT_INPUT_TOUCH_B_CLICK;
	in[C_BTN2_TOUCH].name = left ? XRT_INPUT_TOUCH_Y_TOUCH : XRT_INPUT_TOUCH_B_TOUCH;
	in[C_MENU].name = left ? XRT_INPUT_TOUCH_MENU_CLICK : XRT_INPUT_TOUCH_SYSTEM_CLICK;
	in[C_SQUEEZE].name = XRT_INPUT_TOUCH_SQUEEZE_VALUE;
	in[C_TRIGGER_TOUCH].name = XRT_INPUT_TOUCH_TRIGGER_TOUCH;
	in[C_TRIGGER_VALUE].name = XRT_INPUT_TOUCH_TRIGGER_VALUE;
	in[C_STICK_CLICK].name = XRT_INPUT_TOUCH_THUMBSTICK_CLICK;
	in[C_STICK_TOUCH].name = XRT_INPUT_TOUCH_THUMBSTICK_TOUCH;
	in[C_STICK].name = XRT_INPUT_TOUCH_THUMBSTICK;
	in[C_GRIP_POSE].name = XRT_INPUT_TOUCH_GRIP_POSE;
	in[C_AIM_POSE].name = XRT_INPUT_TOUCH_AIM_POSE;

	c->base.update_inputs = ctl_update_inputs;
	c->base.get_tracked_pose = ctl_get_tracked_pose;
	c->base.get_battery_status = ctl_get_battery_status;
	c->base.destroy = ctl_destroy;
	c->base.tracking_origin = &g_origin;
	c->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	c->base.device_type = left ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	snprintf(c->base.str, XRT_DEVICE_NAME_LEN, "Pimax Crystal %s Controller", left ? "Left" : "Right");
	snprintf(c->base.serial, XRT_DEVICE_NAME_LEN, "pimax-crystal-ctrl-%s", left ? "left" : "right");
	size_t profiles = 0;
	c->base.binding_profiles = pimax_ctrl_binding_profiles(left, &profiles);
	c->base.binding_profile_count = profiles;
	c->base.supported.orientation_tracking = true;
	c->base.supported.position_tracking = true;
	c->base.supported.battery_status = true;
	return &c->base;
}

bool
pimax_poseshm_create(struct xrt_device **out_head, struct xrt_device *out_ctrls[2])
{
	struct xrt_device *head = U_DEVICE_ALLOCATE(struct xrt_device, U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	if (head == NULL) {
		return false;
	}
	u_device_populate_function_pointers(head, head_get_tracked_pose, head_destroy);
	head->inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	head->inputs[0].active = true;
	head->tracking_origin = &g_origin;
	head->name = XRT_DEVICE_GENERIC_HMD;
	head->device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER; // not the head role: the Pimax HMD is (this is its pose)
	snprintf(head->str, XRT_DEVICE_NAME_LEN, "Pimax Crystal head (its own inside-out)");
	snprintf(head->serial, XRT_DEVICE_NAME_LEN, "pimax-crystal-head");
	head->supported.orientation_tracking = true;
	head->supported.position_tracking = true;
	for (int i = 0; i < 2; i++) {
		out_ctrls[i] = ctl_create(i);
		if (out_ctrls[i] == NULL) {
			xrt_device_destroy(&head);
			xrt_device_destroy(&out_ctrls[0]);
			return false;
		}
	}
	*out_head = head;
	U_LOG_I("Pimax Crystal: head and controllers from %s", debug_get_option_pimax_pose_shm());
	return true;
}
