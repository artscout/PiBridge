// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax camera hands (Mercury), world-locked through the HMD's head pose. See pimax_ht.h.
 *
 * Data flow:
 *   camera thread: pimax_ht_push(view0, view1)  -> newest-pair mailbox (dropped when busy / over the rate or duty)
 *   tracker thread (nice): Mercury sync process  -> hands in the view-0 camera frame at the frame time T
 *                          head(T) from the HMD (6DoF when SLAM tracks, else 3DoF; room anchor applied)
 *                          world = head(T) * head_from_cam * hand; pinch / grasp / aim / poke derived here
 *   OpenXR (any thread): get_hand_tracking / get_tracked_pose / update_inputs read the published world state.
 *
 * World-locking (instead of the old ATTACHED override that composed camera-space hands with the head pose at the
 * *query* time): a hand seen at T is placed with the head pose at T, so turning the head does not drag the hands
 * along while Mercury catches up, and SLAM's 6DoF head position moves them correctly.
 *
 * @ingroup drv_pimax
 */

#include "pimax_ht.h"
#include "pimax_ht_calib.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_var.h"

#include "tracking/t_hand_tracking.h"
#include "hg_interface.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#define HT_INFO(ht, ...) U_LOG_IFL_I((ht)->cfg.log_level, __VA_ARGS__)
#define HT_WARN(ht, ...) U_LOG_IFL_W((ht)->cfg.log_level, __VA_ARGS__)
#define HT_ERROR(ht, ...) U_LOG_IFL_E((ht)->cfg.log_level, __VA_ARGS__)

enum ht_state
{
	HT_STARTING = 0,
	HT_RUNNING = 1,
	HT_FAILED = 2,
	HT_STOPPED = 3,
};

//! Input slots of each hand device.
enum
{
	IN_HT = 0, // XRT_INPUT_HT_UNOBSTRUCTED_LEFT / RIGHT
	IN_AIM_POSE,
	IN_GRIP_POSE,
	IN_PINCH_POSE,
	IN_POKE_POSE,
	IN_PINCH_VALUE,
	IN_PINCH_READY,
	IN_AIM_ACTIVATE_VALUE,
	IN_AIM_ACTIVATE_READY,
	IN_GRASP_VALUE,
	IN_GRASP_READY,
	IN_SELECT_CLICK, // simple controller select = pinch (with hysteresis)
	IN_COUNT,
};

//! One hand's published state, world space (the HMD's tracking space).
struct ht_hand
{
	struct xrt_hand_joint_set set; //!< hand_pose = world from Mercury's camera frame at T
	bool active;
	int64_t ts;                    //!< frame time T
	struct xrt_pose aim, grip, pinch, poke;
	float pinch_value, grasp_value;
	bool pinch_ready, grasp_ready, select;
	struct xrt_vec3 wrist_vel; //!< world, m/s (for optional prediction)
	struct xrt_vec3 last_wrist;
	int64_t last_wrist_ts;
};

struct ht_dev
{
	struct xrt_device base;
	struct pimax_ht *ht;
	int hand; // 0 left, 1 right
};

struct pimax_ht
{
	struct pimax_ht_config cfg;
	struct xrt_device *hmd;
	pimax_ht_head_fn head_fn;
	struct t_stereo_camera_calibration *stereo;
	struct t_camera_extra_info extra;
	char models[1024];

	_Atomic int refs;
	_Atomic int state;

	//! Mailbox (camera thread -> tracker thread).
	struct os_thread_helper thread;
	bool thread_started;
	struct xrt_frame *pending[2];
	_Atomic bool busy;
	_Atomic int64_t next_allowed_ns;

	//! Published state.
	struct os_mutex lock;
	struct ht_hand hands[2];

	struct ht_dev *dev[2];
	bool devs_taken;

	//! Stats (tracker thread writes, logged from the camera thread; races are harmless).
	_Atomic uint64_t offered, dropped_busy, dropped_rate, processed, active[2];
	double proc_ms_sum, proc_ms_max;
	uint64_t win_processed, win_active[2], win_offered, win_dropped;
	int64_t win_start;
	_Atomic uint64_t slow; //!< processings over 200 ms
	float last_ms;
};

static void
ht_unref(struct pimax_ht *ht);

/*
 *
 * Geometry helpers.
 *
 */

static struct xrt_vec3
joint_pos(const struct xrt_hand_joint_set *s, enum xrt_hand_joint j)
{
	return s->values.hand_joint_set_default[j].relation.pose.position;
}

static struct xrt_vec3
cross3(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static float
clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

//! Orientation with -Z along @p fwd and +Y as close to @p up as possible.
static struct xrt_quat
look_rotation(struct xrt_vec3 fwd, struct xrt_vec3 up)
{
	struct xrt_vec3 z = m_vec3_mul_scalar(m_vec3_normalize(fwd), -1.0f);
	struct xrt_vec3 x = cross3(up, z);
	if (m_vec3_len(x) < 1e-4f) {
		struct xrt_vec3 alt = {0, 0, 1};
		x = cross3(alt, z);
	}
	x = m_vec3_normalize(x);
	struct xrt_vec3 y = cross3(z, x);
	struct xrt_matrix_3x3 m = {.v = {x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z}};
	struct xrt_quat q;
	math_quat_from_matrix_3x3(&m, &q);
	math_quat_normalize(&q);
	return q;
}

/*!
 * Derive the hand-interaction values of one hand. @p cam = Mercury's set (camera frame), @p world_from_cam its
 * world placement, @p head the head pose at T (world). Writes the poses in world space.
 */
static void
derive(struct ht_hand *h, int hand, const struct xrt_hand_joint_set *cam, const struct xrt_pose *world_from_cam,
       const struct xrt_pose *head)
{
	// Distances are frame independent: use the camera-frame joints.
	struct xrt_vec3 thumb = joint_pos(cam, XRT_HAND_JOINT_THUMB_TIP);
	struct xrt_vec3 index = joint_pos(cam, XRT_HAND_JOINT_INDEX_TIP);
	struct xrt_vec3 palm = joint_pos(cam, XRT_HAND_JOINT_PALM);
	float d = m_vec3_len(m_vec3_sub(thumb, index));
	h->pinch_value = clampf((0.045f - d) / (0.045f - 0.015f), 0.0f, 1.0f);
	h->pinch_ready = d < 0.08f;
	// Simple-profile select with hysteresis.
	if (h->pinch_value >= 0.9f) {
		h->select = true;
	} else if (h->pinch_value < 0.6f) {
		h->select = false;
	}
	const enum xrt_hand_joint tips[3] = {XRT_HAND_JOINT_MIDDLE_TIP, XRT_HAND_JOINT_RING_TIP,
	                                     XRT_HAND_JOINT_LITTLE_TIP};
	float m = 0;
	for (int i = 0; i < 3; i++) {
		m += m_vec3_len(m_vec3_sub(joint_pos(cam, tips[i]), palm)) / 3.0f;
	}
	h->grasp_value = clampf((0.09f - m) / (0.09f - 0.045f), 0.0f, 1.0f);
	h->grasp_ready = m < 0.11f;

	// World poses of the joints we need.
	struct xrt_pose wj[XRT_HAND_JOINT_COUNT];
	const enum xrt_hand_joint need[] = {XRT_HAND_JOINT_PALM, XRT_HAND_JOINT_INDEX_PROXIMAL, XRT_HAND_JOINT_INDEX_TIP,
	                                    XRT_HAND_JOINT_THUMB_TIP};
	for (size_t i = 0; i < ARRAY_SIZE(need); i++) {
		math_pose_transform(world_from_cam, &cam->values.hand_joint_set_default[need[i]].relation.pose,
		                    &wj[need[i]]);
	}
	h->grip = wj[XRT_HAND_JOINT_PALM];
	h->poke = wj[XRT_HAND_JOINT_INDEX_TIP];

	// Aim: from an estimated shoulder (head yaw frame: 0.17 m to the side, 0.20 m down) through the index knuckle.
	struct xrt_vec3 fwd_in = {0, 0, -1}, fwd;
	math_quat_rotate_vec3(&head->orientation, &fwd_in, &fwd);
	fwd.y = 0;
	if (m_vec3_len(fwd) < 1e-3f) {
		fwd = (struct xrt_vec3){0, 0, -1};
	}
	fwd = m_vec3_normalize(fwd);
	struct xrt_vec3 up = {0, 1, 0};
	struct xrt_vec3 right = cross3(fwd, up);
	float side = hand == 0 ? -0.17f : 0.17f;
	struct xrt_vec3 shoulder = m_vec3_add(head->position, m_vec3_add(m_vec3_mul_scalar(right, side),
	                                                                 m_vec3_mul_scalar(up, -0.20f)));
	struct xrt_vec3 knuckle = wj[XRT_HAND_JOINT_INDEX_PROXIMAL].position;
	struct xrt_vec3 dir = m_vec3_sub(knuckle, shoulder);
	if (m_vec3_len(dir) < 0.05f) {
		dir = fwd;
	}
	h->aim.orientation = look_rotation(dir, up);
	h->aim.position = knuckle;
	h->pinch.orientation = h->aim.orientation;
	h->pinch.position =
	    m_vec3_mul_scalar(m_vec3_add(wj[XRT_HAND_JOINT_THUMB_TIP].position, wj[XRT_HAND_JOINT_INDEX_TIP].position),
	                      0.5f);
}

/*
 *
 * Tracker thread.
 *
 */

static void
publish(struct pimax_ht *ht, struct xrt_hand_joint_set out[2], int64_t ts)
{
	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	ht->head_fn(ht->hmd, ts, &head);
	bool head_ok = (head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
	if (!head_ok) {
		head.pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	}
	// world_from_cam = head(T) * head_from_cam
	struct xrt_pose world_from_cam;
	math_pose_transform(&head.pose, &ht->cfg.head_from_cam, &world_from_cam);

	struct ht_hand next[2];
	os_mutex_lock(&ht->lock);
	memcpy(next, ht->hands, sizeof(next));
	os_mutex_unlock(&ht->lock);

	for (int h = 0; h < 2; h++) {
		struct ht_hand *n = &next[h];
		n->set = out[h];
		n->active = out[h].is_active && head_ok;
		n->ts = ts;
		if (!n->active) {
			n->set.is_active = false;
			n->pinch_value = n->grasp_value = 0;
			n->pinch_ready = n->grasp_ready = n->select = false;
			n->last_wrist_ts = 0;
			continue;
		}
		// Joints stay relative to hand_pose (Mercury: the camera frame); hand_pose places them in the world.
		n->set.hand_pose.pose = world_from_cam;
		n->set.hand_pose.relation_flags = head.relation_flags &
		                                  (XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		                                   XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                                   XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                                   XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		n->set.hand_pose.relation_flags |=
		    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT;
		derive(n, h, &out[h], &world_from_cam, &head.pose);

		struct xrt_pose wrist;
		math_pose_transform(&world_from_cam,
		                    &out[h].values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose, &wrist);
		if (n->last_wrist_ts != 0 && ts > n->last_wrist_ts) {
			float dt = (float)(ts - n->last_wrist_ts) * 1e-9f;
			struct xrt_vec3 v = m_vec3_mul_scalar(m_vec3_sub(wrist.position, n->last_wrist), 1.0f / dt);
			// Light smoothing; reset on implausible jumps (re-detection).
			if (m_vec3_len(v) > 5.0f) {
				v = (struct xrt_vec3){0, 0, 0};
			}
			n->wrist_vel = m_vec3_add(m_vec3_mul_scalar(n->wrist_vel, 0.5f), m_vec3_mul_scalar(v, 0.5f));
		} else {
			n->wrist_vel = (struct xrt_vec3){0, 0, 0};
		}
		n->last_wrist = wrist.position;
		n->last_wrist_ts = ts;
		atomic_fetch_add(&ht->active[h], 1);
	}

	os_mutex_lock(&ht->lock);
	memcpy(ht->hands, next, sizeof(next));
	os_mutex_unlock(&ht->lock);
}

static void *
tracker_thread(void *ptr)
{
	struct pimax_ht *ht = (struct pimax_ht *)ptr;
	os_thread_name_self("pimax-hands");

	// Lower our priority first: Mercury's worker pool (created below) inherits it. Never above the default.
	if (ht->cfg.nice > 0) {
		pid_t tid = (pid_t)syscall(SYS_gettid);
		if (setpriority(PRIO_PROCESS, (id_t)tid, ht->cfg.nice) != 0) {
			HT_WARN(ht, "hands: could not set nice %d: %s", ht->cfg.nice, strerror(errno));
		}
	}

	int64_t t0 = os_monotonic_get_ns();
	struct t_hand_tracking_create_info ci = {.cams_info = ht->extra, .masks_sink = NULL};
	struct t_hand_tracking_sync *sync = t_hand_tracking_sync_mercury_create(ht->stereo, ci, ht->models);
	if (sync == NULL) {
		HT_ERROR(ht, "hands: Mercury could not start (ONNX Runtime / models); VR continues without camera hands");
		atomic_store(&ht->state, HT_FAILED);
		return NULL;
	}
	HT_INFO(ht, "hands: Mercury up in %.0f ms (models %s, nice %d, max %.0f Hz, duty %.0f %%)",
	        (double)(os_monotonic_get_ns() - t0) / 1e6, ht->models, ht->cfg.nice, ht->cfg.max_hz,
	        ht->cfg.duty * 100.0);
	atomic_store(&ht->state, HT_RUNNING);

	os_thread_helper_lock(&ht->thread);
	while (os_thread_helper_is_running_locked(&ht->thread)) {
		if (ht->pending[0] == NULL || ht->pending[1] == NULL) {
			os_thread_helper_wait_locked(&ht->thread);
			continue;
		}
		struct xrt_frame *f[2] = {ht->pending[0], ht->pending[1]};
		ht->pending[0] = ht->pending[1] = NULL; // ownership moves to us
		os_thread_helper_unlock(&ht->thread);

		struct xrt_hand_joint_set out[2];
		int64_t ts = 0;
		int64_t a = os_monotonic_get_ns();
		t_ht_sync_process(sync, f[0], f[1], &out[0], &out[1], &ts);
		int64_t b = os_monotonic_get_ns();
		int64_t frame_ts = f[0]->timestamp;
		xrt_frame_reference(&f[0], NULL);
		xrt_frame_reference(&f[1], NULL);

		double ms = (double)(b - a) / 1e6;
		ht->last_ms = (float)ms;
		ht->proc_ms_sum += ms;
		if (ms > ht->proc_ms_max) {
			ht->proc_ms_max = ms;
		}
		if (ms > 200.0) {
			atomic_fetch_add(&ht->slow, 1);
		}
		atomic_fetch_add(&ht->processed, 1);

		publish(ht, out, ts != 0 ? ts : frame_ts);

		// Rate and duty limits: the next pair is accepted no earlier than both allow.
		// 80 % of the period: pairs arrive on a fixed 40 ms grid with jitter, so a full period would drop every
		// other one at 25 Hz.
		int64_t gap_rate = ht->cfg.max_hz > 0 ? (int64_t)(0.8e9 / ht->cfg.max_hz) : 0;
		int64_t busy = b - a;
		int64_t gap_duty = ht->cfg.duty < 1.0f ? (int64_t)((double)busy * (1.0 / ht->cfg.duty - 1.0)) : 0;
		int64_t next = a + gap_rate;
		if (b + gap_duty > next) {
			next = b + gap_duty;
		}
		atomic_store(&ht->next_allowed_ns, next);
		atomic_store(&ht->busy, false);

		os_thread_helper_lock(&ht->thread);
	}
	// Drop anything left in the mailbox.
	xrt_frame_reference(&ht->pending[0], NULL);
	xrt_frame_reference(&ht->pending[1], NULL);
	os_thread_helper_unlock(&ht->thread);

	t_ht_sync_destroy(&sync);
	atomic_store(&ht->state, HT_STOPPED);
	return NULL;
}

void
pimax_ht_push(struct pimax_ht *ht, struct xrt_frame *view0, struct xrt_frame *view1)
{
	if (ht == NULL || atomic_load(&ht->state) != HT_RUNNING) {
		return;
	}
	atomic_fetch_add(&ht->offered, 1);
	ht->win_offered++;
	if (atomic_load(&ht->busy)) {
		atomic_fetch_add(&ht->dropped_busy, 1);
		ht->win_dropped++;
		return;
	}
	if (os_monotonic_get_ns() < atomic_load(&ht->next_allowed_ns)) {
		atomic_fetch_add(&ht->dropped_rate, 1);
		ht->win_dropped++;
		return;
	}
	atomic_store(&ht->busy, true);
	os_thread_helper_lock(&ht->thread);
	xrt_frame_reference(&ht->pending[0], view0);
	xrt_frame_reference(&ht->pending[1], view1);
	os_thread_helper_signal_locked(&ht->thread);
	os_thread_helper_unlock(&ht->thread);
}

/*
 *
 * Devices.
 *
 */

static inline struct ht_dev *
ht_dev(struct xrt_device *xdev)
{
	return (struct ht_dev *)xdev;
}

//! Copy of the hand, inactive when stale or the tracker is not running.
static struct ht_hand
get_hand(struct pimax_ht *ht, int hand, int64_t now)
{
	struct ht_hand h;
	os_mutex_lock(&ht->lock);
	h = ht->hands[hand];
	os_mutex_unlock(&ht->lock);
	if (atomic_load(&ht->state) != HT_RUNNING || !h.active ||
	    now - h.ts > (int64_t)ht->cfg.stale_ms * U_TIME_1MS_IN_NS) {
		h.active = false;
		h.set.is_active = false;
	}
	return h;
}

//! Optional world-space extrapolation of the whole hand by the wrist velocity, capped at predict_ms.
static void
predict(struct pimax_ht *ht, struct ht_hand *h, int64_t at, struct xrt_vec3 *out_shift)
{
	*out_shift = (struct xrt_vec3){0, 0, 0};
	if (ht->cfg.predict_ms <= 0 || !h->active || at <= h->ts) {
		return;
	}
	float dt = (float)(at - h->ts) * 1e-9f;
	float cap = ht->cfg.predict_ms * 1e-3f;
	if (dt > cap) {
		dt = cap;
	}
	*out_shift = m_vec3_mul_scalar(h->wrist_vel, dt);
}

static xrt_result_t
ht_dev_get_hand_tracking(struct xrt_device *xdev,
                         enum xrt_input_name name,
                         int64_t at_timestamp_ns,
                         struct xrt_hand_joint_set *out_value,
                         int64_t *out_timestamp_ns)
{
	struct ht_dev *d = ht_dev(xdev);
	enum xrt_input_name want = d->hand == 0 ? XRT_INPUT_HT_UNOBSTRUCTED_LEFT : XRT_INPUT_HT_UNOBSTRUCTED_RIGHT;
	if (name != want) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(xdev, d->ht->cfg.log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}
	struct ht_hand h = get_hand(d->ht, d->hand, os_monotonic_get_ns());
	*out_value = h.set;
	out_value->is_active = h.active;
	if (h.active) {
		struct xrt_vec3 shift;
		predict(d->ht, &h, at_timestamp_ns, &shift);
		out_value->hand_pose.pose.position = m_vec3_add(out_value->hand_pose.pose.position, shift);
	}
	*out_timestamp_ns = at_timestamp_ns;
	return XRT_SUCCESS;
}

static xrt_result_t
ht_dev_get_tracked_pose(struct xrt_device *xdev,
                        enum xrt_input_name name,
                        int64_t at_timestamp_ns,
                        struct xrt_space_relation *out_relation)
{
	struct ht_dev *d = ht_dev(xdev);
	struct ht_hand h = get_hand(d->ht, d->hand, os_monotonic_get_ns());
	const struct xrt_pose *p = NULL;
	switch (name) {
	case XRT_INPUT_HAND_AIM_POSE: p = &h.aim; break;
	case XRT_INPUT_HAND_GRIP_POSE: p = &h.grip; break;
	case XRT_INPUT_HAND_PINCH_POSE: p = &h.pinch; break;
	case XRT_INPUT_HAND_POKE_POSE: p = &h.poke; break;
	default: U_LOG_XDEV_UNSUPPORTED_INPUT(xdev, d->ht->cfg.log_level, name); return XRT_ERROR_INPUT_UNSUPPORTED;
	}
	*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	if (!h.active) {
		return XRT_SUCCESS; // no flags: not located
	}
	struct xrt_vec3 shift;
	predict(d->ht, &h, at_timestamp_ns, &shift);
	out_relation->pose = *p;
	out_relation->pose.position = m_vec3_add(p->position, shift);
	out_relation->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                               XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	return XRT_SUCCESS;
}

static xrt_result_t
ht_dev_update_inputs(struct xrt_device *xdev)
{
	struct ht_dev *d = ht_dev(xdev);
	int64_t now = os_monotonic_get_ns();
	struct ht_hand h = get_hand(d->ht, d->hand, now);
	struct xrt_input *in = xdev->inputs;
	for (int i = 0; i < IN_COUNT; i++) {
		in[i].timestamp = now;
	}
	in[IN_PINCH_VALUE].value.vec1.x = h.active ? h.pinch_value : 0.0f;
	in[IN_AIM_ACTIVATE_VALUE].value.vec1.x = h.active ? h.pinch_value : 0.0f;
	in[IN_GRASP_VALUE].value.vec1.x = h.active ? h.grasp_value : 0.0f;
	in[IN_PINCH_READY].value.boolean = h.active && h.pinch_ready;
	in[IN_AIM_ACTIVATE_READY].value.boolean = h.active && h.pinch_ready;
	in[IN_GRASP_READY].value.boolean = h.active && h.grasp_ready;
	in[IN_SELECT_CLICK].value.boolean = h.active && h.select;
	return XRT_SUCCESS;
}

static void
ht_dev_destroy(struct xrt_device *xdev)
{
	struct ht_dev *d = ht_dev(xdev);
	struct pimax_ht *ht = d->ht;
	u_var_remove_root(d);
	os_mutex_lock(&ht->lock);
	ht->dev[d->hand] = NULL;
	os_mutex_unlock(&ht->lock);
	u_device_free(&d->base);
	ht_unref(ht);
}

static struct xrt_binding_input_pair simple_inputs[3] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_SIMPLE_SELECT_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_HAND_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_HAND_AIM_POSE},
};

static struct xrt_binding_profile binding_profiles[1] = {
    {
        .name = XRT_DEVICE_SIMPLE_CONTROLLER,
        .inputs = simple_inputs,
        .input_count = ARRAY_SIZE(simple_inputs),
        .outputs = NULL,
        .output_count = 0,
    },
};

static struct ht_dev *
make_dev(struct pimax_ht *ht, int hand)
{
	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_NO_FLAGS | U_DEVICE_ALLOC_TRACKING_NONE;
	struct ht_dev *d = U_DEVICE_ALLOCATE(struct ht_dev, flags, IN_COUNT, 0);
	if (d == NULL) {
		return NULL;
	}
	d->ht = ht;
	d->hand = hand;
	bool left = hand == 0;
	// The HMD's tracking origin: hand poses are in the head's tracking space (like the Pimax controllers).
	if (ht->hmd != NULL && ht->hmd->tracking_origin != NULL) {
		d->base.tracking_origin = ht->hmd->tracking_origin;
	}
	d->base.name = XRT_DEVICE_EXT_HAND_INTERACTION;
	d->base.device_type = ht->cfg.hand_roles
	                          ? (left ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER)
	                          : XRT_DEVICE_TYPE_HAND_TRACKER;
	snprintf(d->base.str, XRT_DEVICE_NAME_LEN, "Pimax Dream Air %s Camera Hand", left ? "Left" : "Right");
	snprintf(d->base.serial, XRT_DEVICE_NAME_LEN, "pimax-camera-hand-%s", left ? "left" : "right");
	struct xrt_input *in = d->base.inputs;
	in[IN_HT].name = left ? XRT_INPUT_HT_UNOBSTRUCTED_LEFT : XRT_INPUT_HT_UNOBSTRUCTED_RIGHT;
	in[IN_AIM_POSE].name = XRT_INPUT_HAND_AIM_POSE;
	in[IN_GRIP_POSE].name = XRT_INPUT_HAND_GRIP_POSE;
	in[IN_PINCH_POSE].name = XRT_INPUT_HAND_PINCH_POSE;
	in[IN_POKE_POSE].name = XRT_INPUT_HAND_POKE_POSE;
	in[IN_PINCH_VALUE].name = XRT_INPUT_HAND_PINCH_VALUE;
	in[IN_PINCH_READY].name = XRT_INPUT_HAND_PINCH_READY;
	in[IN_AIM_ACTIVATE_VALUE].name = XRT_INPUT_HAND_AIM_ACTIVATE_VALUE;
	in[IN_AIM_ACTIVATE_READY].name = XRT_INPUT_HAND_AIM_ACTIVATE_READY;
	in[IN_GRASP_VALUE].name = XRT_INPUT_HAND_GRASP_VALUE;
	in[IN_GRASP_READY].name = XRT_INPUT_HAND_GRASP_READY;
	in[IN_SELECT_CLICK].name = XRT_INPUT_SIMPLE_SELECT_CLICK;
	for (int i = 0; i < IN_COUNT; i++) {
		in[i].active = true;
	}
	d->base.binding_profiles = binding_profiles;
	d->base.binding_profile_count = ARRAY_SIZE(binding_profiles);
	d->base.supported.orientation_tracking = true;
	d->base.supported.position_tracking = true;
	d->base.supported.hand_tracking = true;
	d->base.update_inputs = ht_dev_update_inputs;
	d->base.get_tracked_pose = ht_dev_get_tracked_pose;
	d->base.get_hand_tracking = ht_dev_get_hand_tracking;
	d->base.destroy = ht_dev_destroy;
	atomic_fetch_add(&ht->refs, 1);

	u_var_add_root(d, d->base.str, true);
	u_var_add_bool(d, &ht->hands[hand].active, "active (last result)");
	u_var_add_f32(d, &ht->hands[hand].pinch_value, "pinch");
	u_var_add_f32(d, &ht->hands[hand].grasp_value, "grasp");
	return d;
}

/*
 *
 * Lifetime.
 *
 */

static void
ht_unref(struct pimax_ht *ht)
{
	if (atomic_fetch_sub(&ht->refs, 1) != 1) {
		return;
	}
	t_stereo_camera_calibration_reference(&ht->stereo, NULL);
	os_mutex_destroy(&ht->lock);
	os_thread_helper_destroy(&ht->thread);
	free(ht);
}

struct pimax_ht *
pimax_ht_create(struct xrt_device *hmd,
                pimax_ht_head_fn head_fn,
                const pimax_cam_calib *cal,
                const struct pimax_ht_config *cfg)
{
	enum u_logging_level ll = cfg->log_level;
	char models[1024], why[1400];
	if (!pimax_ht_models_check(models, sizeof(models), why, sizeof(why))) {
		U_LOG_IFL_E(ll, "hands: Mercury models not usable (%s); VR continues without camera hands", why);
		return NULL;
	}
	double baseline = 0;
	struct t_stereo_camera_calibration *stereo = pimax_ht_make_stereo(cal, cfg->cams[0], cfg->cams[1], &baseline);
	if (stereo == NULL) {
		U_LOG_IFL_E(ll, "hands: stereo calibration %u-%u: %s", cfg->cams[0], cfg->cams[1], pimax_cam_last_error());
		return NULL;
	}
	if (baseline < 0.02 || baseline > 0.5) {
		U_LOG_IFL_E(ll, "hands: implausible baseline %.1f mm for cameras %u-%u", baseline * 1000.0, cfg->cams[0],
		            cfg->cams[1]);
		t_stereo_camera_calibration_reference(&stereo, NULL);
		return NULL;
	}

	struct pimax_ht *ht = U_TYPED_CALLOC(struct pimax_ht);
	ht->cfg = *cfg;
	if (ht->cfg.duty <= 0.0f || ht->cfg.duty > 1.0f) {
		ht->cfg.duty = 1.0f;
	}
	ht->hmd = hmd;
	ht->head_fn = head_fn;
	ht->stereo = stereo;
	snprintf(ht->models, sizeof(ht->models), "%s", models);
	atomic_store(&ht->refs, 1);
	atomic_store(&ht->state, HT_STARTING);
	for (int v = 0; v < 2; v++) {
		bool rolled = false;
		ht->extra.views[v].boundary_type = HT_IMAGE_BOUNDARY_NONE;
		ht->extra.views[v].camera_orientation = cfg->orient[v] >= 0
		                                            ? (enum t_camera_orientation)cfg->orient[v]
		                                            : pimax_ht_orientation_of(cal, cfg->cams[v], &rolled);
		if (rolled) {
			U_LOG_IFL_W(ll, "hands: camera %u is rolled ~90 deg; orientation 90/270 not verified, using 0",
			            cfg->cams[v]);
		}
	}
	os_mutex_init(&ht->lock);
	os_thread_helper_init(&ht->thread);

	U_LOG_IFL_I(ll,
	            "hands: cameras %u (%s) + %u (%s), baseline %.1f mm, view orientation %d/%d deg; world-locked via "
	            "the head pose at the frame time; roles %s",
	            cfg->cams[0], cal->cameras[cfg->cams[0]].name, cfg->cams[1], cal->cameras[cfg->cams[1]].name,
	            baseline * 1000.0, (int)ht->extra.views[0].camera_orientation,
	            (int)ht->extra.views[1].camera_orientation,
	            cfg->hand_roles ? "left/right hands (controllers lose their roles)" : "hand tracking only");

	for (int h = 0; h < 2; h++) {
		ht->dev[h] = make_dev(ht, h);
	}
	if (ht->dev[0] == NULL || ht->dev[1] == NULL || os_thread_helper_start(&ht->thread, tracker_thread, ht) != 0) {
		U_LOG_IFL_E(ll, "hands: could not start; VR continues without camera hands");
		for (int h = 0; h < 2; h++) {
			if (ht->dev[h] != NULL) {
				xrt_device_destroy((struct xrt_device **)&ht->dev[h]);
			}
		}
		ht_unref(ht);
		return NULL;
	}
	ht->thread_started = true;
	return ht;
}

void
pimax_ht_take_devices(struct pimax_ht *ht, struct xrt_device *out[2])
{
	out[0] = out[1] = NULL;
	if (ht == NULL || ht->devs_taken) {
		return;
	}
	ht->devs_taken = true;
	out[0] = &ht->dev[0]->base;
	out[1] = &ht->dev[1]->base;
}

void
pimax_ht_log_stats(struct pimax_ht *ht)
{
	if (ht == NULL || !ht->cfg.stats) {
		return;
	}
	int64_t now = os_monotonic_get_ns();
	if (ht->win_start == 0) {
		ht->win_start = now;
	}
	double dt = (double)(now - ht->win_start) / 1e9;
	uint64_t p = atomic_load(&ht->processed), a0 = atomic_load(&ht->active[0]), a1 = atomic_load(&ht->active[1]);
	uint64_t dp = p - ht->win_processed;
	static const char *names[] = {"starting", "running", "FAILED", "stopped"};
	U_LOG_IFL_I(ht->cfg.log_level,
	            "hands (%s): %.1f pairs/s processed of %.1f offered (%" PRIu64 " dropped: busy/rate/duty), mean %.1f "
	            "ms, max %.1f ms, %" PRIu64 " over 200 ms; left active %.0f %%, right %.0f %%",
	            names[atomic_load(&ht->state) & 3], dt > 0 ? dp / dt : 0.0, dt > 0 ? ht->win_offered / dt : 0.0,
	            ht->win_dropped, dp ? ht->proc_ms_sum / (double)dp : 0.0, ht->proc_ms_max, atomic_load(&ht->slow),
	            dp ? 100.0 * (double)(a0 - ht->win_active[0]) / (double)dp : 0.0,
	            dp ? 100.0 * (double)(a1 - ht->win_active[1]) / (double)dp : 0.0);
	ht->win_processed = p;
	ht->win_active[0] = a0;
	ht->win_active[1] = a1;
	ht->win_offered = ht->win_dropped = 0;
	ht->proc_ms_sum = ht->proc_ms_max = 0;
	ht->win_start = now;
}

void
pimax_ht_destroy(struct pimax_ht **ht_ptr)
{
	struct pimax_ht *ht = *ht_ptr;
	if (ht == NULL) {
		return;
	}
	*ht_ptr = NULL;
	if (ht->thread_started) {
		// Waits for the current Mercury step (tens of ms) to finish; Mercury is destroyed on its thread.
		os_thread_helper_stop_and_wait(&ht->thread);
		ht->thread_started = false;
	}
	// Devices never handed to the system are ours.
	if (!ht->devs_taken) {
		for (int h = 0; h < 2; h++) {
			if (ht->dev[h] != NULL) {
				struct xrt_device *x = &ht->dev[h]->base;
				xrt_device_destroy(&x);
			}
		}
	}
	// The head is going away: from now on the devices only report inactive hands.
	atomic_store(&ht->state, HT_STOPPED);
	ht->hmd = NULL;
	ht_unref(ht);
}
