// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air controllers. See analysis/monado-controllers.md.
 *
 * Two xrt_devices (left slot 2, right slot 3) exposed with the Oculus Touch profile. No optical tracking:
 * PIMAX_CTRL_POSE=fixed (default): synthesized poses from rust/pimax-hands (rest.rs): a fixed place in the head-yaw
 * frame, in the lower part of the view (PIMAX_CTRL_REST=desk|legacy|side,down,fwd[,grip_pitch_deg]), aim level or
 * aimed where the head looks (PIMAX_CTRL_AIM=level|head[:m]), grip pitched like a held handle (analysis/hands-desk.md).
 * PIMAX_CTRL_POSE=imu: same position, orientation from the controller IMU (accel 1024 LSB/g, gyro 8.2 LSB/(deg/s) =
 * +-4000 deg/s full scale, PIMAX_CTRL_GYRO_LSB_DPS; raw axes == what Pimax's engine is fed, same signs).
 * The gyro scale and axes were measured by matching the raw USB HID reports with what Pimax Play's service hands its
 * engine (frida pass 5 + USBPcap, Beat Saber Extreme: accel scale 1.000x and axes identical, gyro exactly 2x the old
 * 16.4 guess, correlation 1.000; re-dll/README.md).
 * PIMAX_CTRL_TRACK=1: optical 6DoF from `pimax-ctrltrack live` (pimax-re rust/pimax-ctrltrack: controller LEDs in the
 * camera LED frames -> pose in the head frame, seqlock file pimax_ctrltrack_shm.h). The pose is composed with the head
 * pose at the LED frame's time and handed to the fuser in rust/pimax-hands (fuse.rs, pimax_ctrlfuse.h): gyro
 * orientation between fixes (calibration learned online from the optical poses, ~/.config/pibridge/ctrl-imu.json), short
 * extrapolation, then an arm-model hold relative to the head, the synthesized pose above only after
 * PIMAX_CTRL_TRACK_LOST_S (4 s); reacquire blends over 100 ms, optical outliers are dropped. The optical grip is
 * aim * the Quest 2 Touch openxr_aim -> openxr_grip transform (PIMAX_TOUCH_GRIP), so SteamVR's aim equals ours.
 * Also offered as /interaction_profiles/ext/hand_interaction_ext (pinch = trigger, grasp = squeeze) for apps that
 * only suggest hand-interaction bindings; apps with Touch bindings keep the Touch profile.
 *
 * In hand (PIMAX_CTRL_INHAND, rust/pimax-hands inhand.rs, docs/CONTROLLERS-INHAND.md): auto = a controller is only
 * shown while it is in the hand (IMU motion or an input change picks it up, 5 s still or 1.5 s without reports puts it
 * down); while down it reports no pose and inactive inputs, so WayVR draws neither model nor ray for a controller
 * lying on the desk. always (the driver's default, games) = as before; off = never shown. `pimax desk` passes auto.
 * Run-time change: $XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode; status: $XDG_RUNTIME_DIR/pimax-ctrl-inhand.
 *
 * Controller angle (PIMAX_CTRL_ANGLE, degrees -20..+70, default +30; live file $XDG_RUNTIME_DIR/pimax-ctrl-angle): grip and
 * aim of both controllers are pitched up by this much about their own x axis (docs/CONTROLLERS.md).
 *
 * Hand tracking (XR_EXT_hand_tracking, data source CONTROLLER): the finger curls come from the Rust crate pimax-hands
 * (trigger -> index, grip -> middle/ring/little, touch bits -> thumb; filters + thumb state machine live there), this
 * file only feeds it the decoded reports and runs Monado's u_hand_sim_simulate_generic on the result. The hand is
 * posed from the controller's grip pose through a fixed grip->wrist offset (also from pimax-hands).
 * PIMAX_CTRL_HANDS=0 turns it off, PIMAX_CTRL_HANDS_DEMO=1 replaces the reports by a scripted pattern (test aid).
 * The reports are only queued (plain C, a small ring) on the libpimax session thread; the Rust filters run when a
 * client asks for the hand joints (get_hand_tracking), so a session without hand tracking does no hand work at all
 * (hang when worn, analysis/hang-worn-debug.md). PIMAX_CTRL_HANDS_FEED=report restores the old per-report feed on the
 * session thread (A/B test only).
 *
 * @ingroup drv_pimax
 */

#include "os/os_time.h"
#include "os/os_threading.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_config_have.h"

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_vec3.h"
#include "math/m_mathinclude.h"
#include "math/m_space.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_hand_simulation.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_var.h"

#include "pimax_ctrl.h"
#include "pimax_interface.h"
#include "pimax_ctrlfuse.h"
#include "pimax_ctrltrack_shm.h"
#include "pimax_hands.h"
#include "pimax_winbridge.h"

#ifndef PIMAX_BTN_APP_MENU
//! The Menu button (report byte 47 bit 2): PiBridge's libpibridge_pimax reports it; the Rust libpimax never does.
//! PIMAX_BTN_MENU (byte 48 bit 0, the old name) is in fact the Pimax button (live 2026-10-19, both hands).
#define PIMAX_BTN_APP_MENU 0x04
#endif
#include "pimax_inhand.h"

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>
#include <time.h>

DEBUG_GET_ONCE_OPTION(pimax_ctrl_pose, "PIMAX_CTRL_POSE", "fixed")
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ctrl_gyro_lsb_dps, "PIMAX_CTRL_GYRO_LSB_DPS", 8.2f)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_hands, "PIMAX_CTRL_HANDS", true)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_hands_demo, "PIMAX_CTRL_HANDS_DEMO", false)
DEBUG_GET_ONCE_OPTION(pimax_ctrl_hands_feed, "PIMAX_CTRL_HANDS_FEED", "lazy")
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_track, "PIMAX_CTRL_TRACK", false)
//! Log the controller report's button and touch bytes (44..50) at every change: find which bit a button sends.
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_button_log, "PIMAX_CTRL_BUTTON_LOG", false)
// PIMAX_CTRL_ANGLE: the controller angle in degrees, -20..+70, default +30 (docs/CONTROLLERS.md)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ctrl_angle, "PIMAX_CTRL_ANGLE", 30.0f)
// PIMAX_CTRL_ENGINE: controller poses from Pimax's own LED engine through PiBridge's winbridge (pimax_winbridge.h)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_engine, "PIMAX_CTRL_ENGINE", false)
// The aim frame in the LED model frame: rot_x(-pitch), as pimax-ctrltrack publishes its aim (live.cpp, 52 deg)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ctrl_engine_aim_pitch, "PIMAX_CTRLTRACK_AIM_PITCH_DEG", 52.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ctrl_engine_lost_s, "PIMAX_CTRL_TRACK_LOST_S", 4.0f)

#define CTRL_ANGLE_MIN_DEG (-20.0f)
#define CTRL_ANGLE_MAX_DEG (70.0f)

#define ACTIVE_TIMEOUT_NS (1500ull * 1000 * 1000) // no report for this long -> inputs inactive
#define REATTACH_MIN_NS (2500ull * 1000 * 1000)
#define BIAS_SAMPLES 250 // ~1 s at 250 Hz
#define HANDS_RING 128   // queued reports for the lazy hand feed: ~0.5 s at 250 Hz (older ones are dropped)

enum
{
	I_BTN1_CLICK, // left X / right A  (report A bit)
	I_BTN1_TOUCH,
	I_BTN2_CLICK, // left Y / right B  (report B bit)
	I_BTN2_TOUCH,
	I_MENU, // the Menu button (byte 47 bit 2, both hands): left MENU_CLICK, right SYSTEM_CLICK
	I_SQUEEZE,
	I_TRIGGER_TOUCH,
	I_TRIGGER_VALUE,
	I_STICK_CLICK,
	I_STICK_TOUCH,
	I_STICK,
	I_GRIP_POSE,
	I_AIM_POSE,
	I_HAND_TRACKING, // XRT_INPUT_HT_CONFORMING_*; last, only allocated when hand tracking is on
	I_COUNT,
};

struct pimax_ctrl_ctx
{
	struct xrt_reference ref;
	struct xrt_device *head;
	enum u_logging_level log_level;
	bool imu_mode;
	float gyro_scale; // rad/s per LSB
	bool hands;       // controller-driven hand tracking (pimax-hands)
	bool hands_demo;  // scripted input instead of reports (PIMAX_CTRL_HANDS_DEMO)
	bool hands_eager; // PIMAX_CTRL_HANDS_FEED=report: feed Rust on the session thread (old behaviour)
	struct xrt_pose aim_to_grip; // fixed grip pitch relative to the aim (IMU mode), from pimax-hands
	struct xrt_pose tracked_aim_to_grip; // optical: Touch (Quest 2) geometry, rotation + offset, from pimax-hands

	// Optical 6DoF (PIMAX_CTRL_TRACK=1): poses from `pimax-ctrltrack live` (seqlock file, pimax_ctrltrack_shm.h).
	bool track;
	char track_path[256];
	pthread_mutex_t track_lock; // reader (re)mapping, ctl[] / fuse while the feed thread uses them
	pthread_t track_thread;     // polls the pose file (4 ms) and feeds every new fix to the fusers
	bool track_thread_running;
	atomic_bool track_stop;
	struct pimax_ctrltrack_reader track_reader;
	uint64_t track_next_open_ns;

	struct pimax_controller *ctl[2]; // [0] left (slot 2), [1] right (slot 3)

	// Haptics (PIMAX_HAPTICS): the HMD's session, set by pimax_ctrl_start, cleared by pimax_ctrl_stop. set_output runs
	// on IPC/compositor threads; it only queues into libpimax (the session thread writes, rate limited).
	_Atomic(pimax_t *) hap_lib;
	atomic_uint hap_calls;

	// Attach thread.
	pimax_t *lib;
	pthread_t thread;
	bool thread_running;
	atomic_bool stop;
	atomic_bool need_attach_all; // set on (re)connect

	// In hand (PIMAX_CTRL_INHAND): PIMAX_INHAND_MODE_*, changed at run time by the attach thread.
	atomic_int inhand_mode;

	// PIMAX_CTRL_ANGLE: pitch (centi-degrees) added to the grip AND aim pose of both controllers; positive = the
	// pointer tilts up. Start value from the env, live changes from $XDG_RUNTIME_DIR/pimax-ctrl-angle.
	atomic_int angle_cdeg;
	int64_t recenter_request_mtime; //!< PiBridge recenter-request: last seen mtime (0 = not looked yet, -1 = absent)

	// PIMAX_CTRL_ENGINE: Pimax's LED engine (pimax_winbridge.h) gets every IMU report and gives the poses.
	bool engine;
	struct pimax_wb wb;
	struct xrt_quat device_from_aim; // rot_x(-aim pitch)
	uint64_t engine_lost_ns;         // PIMAX_CTRL_TRACK_LOST_S: gyro-only hold, then the rest pose
};

struct pimax_controller
{
	struct xrt_device base;
	struct pimax_ctrl_ctx *ctx;
	int index; // 0 left, 1 right
	uint8_t slot;

	pthread_mutex_t lock;
	// Latest state (lock).
	pimax_controller_report rep;
	bool have_report;
	uint64_t last_report_ns;
	uint32_t reports;
	float trigger, grip, stick_x, stick_y;
	bool btn_a, btn_b, btn_menu, btn_stick, t_a, t_b, t_stick;
	bool btn_system;         //!< the Pimax button, byte 48 bit 0 (recenter on a LONG press; not an app input)
	uint64_t system_press_ns; //!< when the Pimax button went down (for the long-press recenter)
	bool system_recentered;   //!< a recenter already fired for the current press
	int system_progress_q;    //!< the last published hold progress, in 1/40 (-1: none yet)
	uint8_t raw46;
	uint8_t last_btn_bytes[7]; //!< PIMAX_CTRL_BUTTON_LOG: report bytes 44..50 at the last log line

	// Hand tracking (lock): Rust state (curl filters, thumb state machine) and its latest result.
	pimax_hands *hands;
	bool hand_active_logged; // last activity reported by get_hand_tracking (log on change)
	pimax_hands_out hands_out;
	struct xrt_pose grip_to_wrist; // wrist in the grip pose frame
	// Lazy feed (lock): reports queued by the session thread, drained into pimax-hands by get_hand_tracking.
	struct
	{
		uint64_t ns;
		uint32_t buttons;
		int16_t stick_x, stick_y;
		uint8_t trigger, grip, touch;
	} hq[HANDS_RING];
	uint32_t hq_head;  // next write index
	uint32_t hq_count; // queued, <= HANDS_RING

	// IMU orientation (lock).
	struct m_imu_3dof fusion;
	bool fusion_init;
	bool bias_done;
	uint32_t bias_n;
	double gyro_sum[3], acc_sum[3];
	struct xrt_quat yaw_ref; // head yaw when the IMU tilt was seeded
	struct xrt_quat imu_rot;

	// Optical fix (lock): the last pose from pimax-ctrltrack, anchored in the tracking origin at its frame time.
	int fuse_state;        // last PIMAX_CTRLFUSE_* returned (log on change)
	pimax_ctrlfuse *fuse;  // rust/pimax-hands fuse.rs: out-of-view hold, gyro, reacquire blend, gyro calibration

	// In hand: the detector (own lock) and the last visibility logged (attach thread).
	pimax_inhand *inhand;
	int shown_logged; // -1 unknown, 0 hidden, 1 shown

	// PIMAX_CTRL_ENGINE (lock): the last tracked aim pose (world), when, the IMU orientation then (to turn it with the
	// gyro while the engine does not see the controller), the IMU -> body rotation, the state logged.
	bool eng_have;
	struct xrt_pose eng_aim;
	uint64_t eng_last_ns;
	struct xrt_quat eng_imu_rot;
	struct xrt_quat eng_dev_from_imu;
	int eng_state; // -1 unknown, 0 rest pose, 1 engine, 2 held (gyro)

	// Attach logic (written from the session thread and the attach thread, atomics).
	_Atomic uint64_t last_status_ns;
	_Atomic uint64_t last_attach_ns;
	atomic_uint attaches;
};

static inline struct pimax_controller *
ctl(struct xrt_device *xdev)
{
	return (struct pimax_controller *)xdev;
}

#define C_INFO(c, ...) U_LOG_XDEV_IFL_I(&(c)->base, (c)->ctx->log_level, __VA_ARGS__)
#define C_WARN(c, ...) U_LOG_XDEV_IFL_W(&(c)->base, (c)->ctx->log_level, __VA_ARGS__)

/*
 * Binding profiles (khr/simple_controller and Valve Index emulate this device).
 */
static struct xrt_binding_input_pair simple_inputs_left[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};
static struct xrt_binding_input_pair simple_inputs_right[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_SYSTEM_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};
static struct xrt_binding_output_pair simple_outputs[1] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

#define INDEX_COMMON                                                                                                   \
	{XRT_INPUT_INDEX_TRIGGER_VALUE, XRT_INPUT_TOUCH_TRIGGER_VALUE},                                                \
	    {XRT_INPUT_INDEX_TRIGGER_TOUCH, XRT_INPUT_TOUCH_TRIGGER_TOUCH},                                            \
	    {XRT_INPUT_INDEX_SQUEEZE_VALUE, XRT_INPUT_TOUCH_SQUEEZE_VALUE},                                            \
	    {XRT_INPUT_INDEX_THUMBSTICK, XRT_INPUT_TOUCH_THUMBSTICK},                                                  \
	    {XRT_INPUT_INDEX_THUMBSTICK_CLICK, XRT_INPUT_TOUCH_THUMBSTICK_CLICK},                                      \
	    {XRT_INPUT_INDEX_THUMBSTICK_TOUCH, XRT_INPUT_TOUCH_THUMBSTICK_TOUCH},                                      \
	    {XRT_INPUT_INDEX_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE}, {XRT_INPUT_INDEX_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE}
static struct xrt_binding_input_pair index_inputs_left[10] = {
    INDEX_COMMON,
    {XRT_INPUT_INDEX_A_CLICK, XRT_INPUT_TOUCH_X_CLICK},
    {XRT_INPUT_INDEX_B_CLICK, XRT_INPUT_TOUCH_Y_CLICK},
};
static struct xrt_binding_input_pair index_inputs_right[10] = {
    INDEX_COMMON,
    {XRT_INPUT_INDEX_A_CLICK, XRT_INPUT_TOUCH_A_CLICK},
    {XRT_INPUT_INDEX_B_CLICK, XRT_INPUT_TOUCH_B_CLICK},
};
static struct xrt_binding_output_pair index_outputs[1] = {
    {XRT_OUTPUT_NAME_INDEX_HAPTIC, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

// XR_EXT_hand_interaction for hand-only apps: pinch / aim activate = trigger (ready = finger on the trigger),
// grasp = squeeze. Monado only offers the profile when a device maps it; the Touch profile still wins when suggested.
static struct xrt_binding_input_pair hand_interaction_inputs[8] = {
    {XRT_INPUT_HAND_PINCH_VALUE, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_HAND_PINCH_READY, XRT_INPUT_TOUCH_TRIGGER_TOUCH},
    {XRT_INPUT_HAND_AIM_ACTIVATE_VALUE, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_HAND_AIM_ACTIVATE_READY, XRT_INPUT_TOUCH_TRIGGER_TOUCH},
    {XRT_INPUT_HAND_GRASP_VALUE, XRT_INPUT_TOUCH_SQUEEZE_VALUE},
    {XRT_INPUT_HAND_GRASP_READY, XRT_INPUT_TOUCH_SQUEEZE_VALUE},
    {XRT_INPUT_HAND_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
    {XRT_INPUT_HAND_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
};

#define HAND_INTERACTION_PROFILE                                                                                       \
	{                                                                                                              \
		.name = XRT_DEVICE_EXT_HAND_INTERACTION, .inputs = hand_interaction_inputs,                            \
		.input_count = ARRAY_SIZE(hand_interaction_inputs), .outputs = NULL, .output_count = 0,                \
	}

static struct xrt_binding_profile profiles_left[3] = {
    {.name = XRT_DEVICE_SIMPLE_CONTROLLER,
     .inputs = simple_inputs_left,
     .input_count = ARRAY_SIZE(simple_inputs_left),
     .outputs = simple_outputs,
     .output_count = ARRAY_SIZE(simple_outputs)},
    {.name = XRT_DEVICE_INDEX_CONTROLLER,
     .inputs = index_inputs_left,
     .input_count = ARRAY_SIZE(index_inputs_left),
     .outputs = index_outputs,
     .output_count = ARRAY_SIZE(index_outputs)},
    HAND_INTERACTION_PROFILE,
};
static struct xrt_binding_profile profiles_right[3] = {
    {.name = XRT_DEVICE_SIMPLE_CONTROLLER,
     .inputs = simple_inputs_right,
     .input_count = ARRAY_SIZE(simple_inputs_right),
     .outputs = simple_outputs,
     .output_count = ARRAY_SIZE(simple_outputs)},
    {.name = XRT_DEVICE_INDEX_CONTROLLER,
     .inputs = index_inputs_right,
     .input_count = ARRAY_SIZE(index_inputs_right),
     .outputs = index_outputs,
     .output_count = ARRAY_SIZE(index_outputs)},
    HAND_INTERACTION_PROFILE,
};

/*
 * Device functions.
 */

static void
ctx_unref(struct pimax_ctrl_ctx *ctx)
{
	if (xrt_reference_dec_and_is_zero(&ctx->ref)) {
		pimax_ctrltrack_close(&ctx->track_reader);
		pthread_mutex_destroy(&ctx->track_lock);
		free(ctx);
	}
}

static void
pimax_ctl_destroy(struct xrt_device *xdev)
{
	struct pimax_controller *c = ctl(xdev);
	struct pimax_ctrl_ctx *ctx = c->ctx;
	pthread_mutex_lock(&ctx->track_lock); // the feed thread must not see this controller any more
	ctx->ctl[c->index] = NULL;
	pthread_mutex_unlock(&ctx->track_lock);
	u_var_remove_root(c);
	if (c->fusion_init) {
		m_imu_3dof_close(&c->fusion);
	}
	pthread_mutex_lock(&c->lock);
	pimax_hands *hands = c->hands; // a late report must not find a freed hand state
	c->hands = NULL;
	pimax_ctrlfuse *fuse = c->fuse;
	c->fuse = NULL;
	pthread_mutex_unlock(&c->lock);
	pimax_hands_destroy(hands);
	pimax_inhand *inhand = c->inhand;
	c->inhand = NULL;
	pimax_inhand_destroy(inhand);
	if (fuse != NULL) {
		char d[256];
		pimax_ctrlfuse_describe(fuse, d, sizeof(d));
		C_INFO(c, "optical tracking: %s", d);
		pimax_ctrlfuse_destroy(fuse); // saves an improved gyro calibration
	}
	pthread_mutex_destroy(&c->lock);
	u_device_free(&c->base);
	ctx_unref(ctx);
}

static float
clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/*!
 * PIMAX_CTRL_INHAND: should this controller be shown now? always: yes; off: no; auto: only while it is in the hand
 * (rust/pimax-hands inhand.rs). The demo pattern is always shown.
 */
static bool
ctl_shown(struct pimax_controller *c, uint64_t now)
{
	if (c->ctx->hands_demo) {
		return true;
	}
	switch (atomic_load(&c->ctx->inhand_mode)) {
	case PIMAX_INHAND_MODE_ALWAYS: return true;
	case PIMAX_INHAND_MODE_OFF: return false;
	default: return pimax_inhand_held(c->inhand, now) != 0;
	}
}

static xrt_result_t
pimax_ctl_update_inputs(struct xrt_device *xdev)
{
	struct pimax_controller *c = ctl(xdev);
	bool left = c->index == 0;
	uint64_t now = os_monotonic_get_ns();

	bool shown = ctl_shown(c, now);
	pthread_mutex_lock(&c->lock);
	bool active = shown && c->have_report && (now - c->last_report_ns) < ACTIVE_TIMEOUT_NS;
	uint64_t ts = c->have_report ? (int64_t)c->last_report_ns : (int64_t)now;
	if (c->ctx->hands_demo) {
		// Test aid: the scripted demo input (the one that drives the fingers) also drives trigger and squeeze, so
		// pinch / click paths can be tested end to end without a human (or a headset, with PIMAX_NO_DEVICE=1).
		pimax_hands_input d;
		pimax_hands_demo_input(now, !left, &d);
		c->trigger = d.trigger;
		c->grip = d.grip;
		active = true;
		ts = now;
	}
	struct xrt_input *in = c->base.inputs;
	in[I_BTN1_CLICK].value.boolean = c->btn_a;
	in[I_BTN1_TOUCH].value.boolean = c->t_a || c->btn_a;
	in[I_BTN2_CLICK].value.boolean = c->btn_b;
	in[I_BTN2_TOUCH].value.boolean = c->t_b || c->btn_b;
	in[I_MENU].value.boolean = c->btn_menu;
	in[I_SQUEEZE].value.vec1.x = c->grip;
	in[I_TRIGGER_VALUE].value.vec1.x = c->trigger;
	in[I_TRIGGER_TOUCH].value.boolean = c->trigger > 0.06f;
	in[I_STICK_CLICK].value.boolean = c->btn_stick;
	in[I_STICK_TOUCH].value.boolean = c->t_stick || c->btn_stick;
	in[I_STICK].value.vec2.x = c->stick_x;
	in[I_STICK].value.vec2.y = c->stick_y;
	pthread_mutex_unlock(&c->lock);
	(void)left;

	for (int i = 0; i < I_GRIP_POSE; i++) {
		in[i].active = active;
		in[i].timestamp = (int64_t)ts;
	}
	// Poses are synthesized or tracked, so they are active, unless the controller is put down (PIMAX_CTRL_INHAND).
	in[I_GRIP_POSE].active = in[I_AIM_POSE].active = shown;
	in[I_GRIP_POSE].timestamp = in[I_AIM_POSE].timestamp = (int64_t)now;
	return XRT_SUCCESS;
}

//! Yaw-only quaternion of the head orientation (about +Y). Identity if the head has no valid orientation.
static struct xrt_quat
head_yaw(struct pimax_controller *c, int64_t at_ns)
{
	struct xrt_quat q = XRT_QUAT_IDENTITY;
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	if (c->ctx->head == NULL ||
	    xrt_device_get_tracked_pose(c->ctx->head, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, &rel) != XRT_SUCCESS ||
	    (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return q;
	}
	struct xrt_vec3 fwd_in = {0, 0, -1}, fwd;
	math_quat_rotate_vec3(&rel.pose.orientation, &fwd_in, &fwd);
	if (fabsf(fwd.x) + fabsf(fwd.z) < 1e-3f) {
		return q; // looking straight up/down: yaw undefined
	}
	float yaw = atan2f(-fwd.x, -fwd.z);
	struct xrt_vec3 up = {0, 1, 0};
	math_quat_from_angle_vector(yaw, &up, &q);
	return q;
}

static struct xrt_pose
pose_from_rust(const pimax_hands_pose *p)
{
	struct xrt_pose out = {
	    .orientation = {p->orientation[0], p->orientation[1], p->orientation[2], p->orientation[3]},
	    .position = {p->position[0], p->position[1], p->position[2]},
	};
	math_quat_normalize(&out.orientation);
	return out;
}

static pimax_hands_pose
to_hp(const struct xrt_pose *p)
{
	pimax_hands_pose o = {
	    .orientation = {p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w},
	    .position = {p->position.x, p->position.y, p->position.z},
	};
	return o;
}

//! Head pose at `at_ns` (identity when the head has no valid orientation).
static struct xrt_pose
head_pose_at(struct pimax_ctrl_ctx *ctx, int64_t at_ns)
{
	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	struct xrt_pose p = XRT_POSE_IDENTITY;
	if (ctx->head != NULL &&
	    xrt_device_get_tracked_pose(ctx->head, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, &head) == XRT_SUCCESS &&
	    (head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		p = head.pose;
		math_quat_normalize(&p.orientation);
	}
	return p;
}

//! Feed thread: every new fix in the pose file goes to its controller's fuser with the head pose at its frame time.
static void *
track_thread(void *arg)
{
	struct pimax_ctrl_ctx *ctx = arg;
	uint64_t next_open = 0;
	uint64_t seen[2] = {0, 0};
	while (!atomic_load(&ctx->track_stop)) {
		uint64_t now = os_monotonic_get_ns();
		pthread_mutex_lock(&ctx->track_lock);
		if (now >= next_open) {
			// first map, and remap when a restarted writer created a new file
			next_open = now + 1000ull * 1000 * 1000;
			bool had = ctx->track_reader.map != NULL;
			if (pimax_ctrltrack_open(&ctx->track_reader, ctx->track_path) && !had) {
				U_LOG_IFL_I(ctx->log_level, "Pimax controllers: optical poses from %s", ctx->track_path);
			}
		}
		struct pimax_ctrltrack_hand hands[2];
		if (pimax_ctrltrack_read(&ctx->track_reader, hands)) {
			for (int i = 0; i < 2; i++) {
				const struct pimax_ctrltrack_hand *h = &hands[i];
				struct pimax_controller *c = ctx->ctl[i];
				if (c == NULL || c->fuse == NULL || h->frame_ns == 0 || h->frame_ns == seen[i]) {
					continue;
				}
				if (now > h->frame_ns + 2000ull * 1000 * 1000) {
					seen[i] = h->frame_ns; // left over from an earlier tracker run
					continue;
				}
				seen[i] = h->frame_ns;
				struct xrt_pose body_aim = {
				    .orientation = {h->quat[0], h->quat[1], h->quat[2], h->quat[3]},
				    .position = {h->pos[0], h->pos[1], h->pos[2]},
				};
				math_quat_normalize(&body_aim.orientation);
				struct xrt_pose world_head = head_pose_at(ctx, (int64_t)h->frame_ns);
				struct xrt_pose world_aim;
				math_pose_transform(&world_head, &body_aim, &world_aim);
				pimax_hands_pose wa = to_hp(&world_aim), wh = to_hp(&world_head);
				pimax_ctrlfuse_optical(c->fuse, h->frame_ns, &wa, &wh, h->rms_px, h->leds);
			}
		}
		pthread_mutex_unlock(&ctx->track_lock);
		struct timespec ts = {0, 4 * 1000 * 1000};
		nanosleep(&ts, NULL);
	}
	return NULL;
}

/*!
 * Optical 6DoF from pimax-ctrltrack: each new fix (controller pose in the head frame at the LED frame time) is
 * composed with the head pose at that time and handed to the fuser (rust/pimax-hands fuse.rs), which answers with
 * the pose at the requested time: fresh optical (+ gyro, + short extrapolation), out of view an arm-model hold
 * relative to the head, or nothing after PIMAX_CTRL_TRACK_LOST_S (then the caller uses the rest pose).
 */
static bool
optical_pose(struct pimax_controller *c, int64_t at_ns, enum xrt_input_name name, struct xrt_space_relation *out)
{
	struct pimax_ctrl_ctx *ctx = c->ctx;
	if (c->fuse == NULL) {
		return false;
	}
	struct xrt_pose head_now = head_pose_at(ctx, at_ns);
	pimax_hands_pose hn = to_hp(&head_now), aim;
	pimax_ctrlfuse_info info = {0};
	int st = pimax_ctrlfuse_get(c->fuse, (uint64_t)(at_ns > 0 ? at_ns : 0), &hn, &aim, &info);
	if (st != c->fuse_state) {
		static const char *names[] = {"rest pose (no optical fix)", "optical", "out of view: held relative to the head"};
		C_INFO(c, "controller pose: %s%s", names[st >= 0 && st <= 2 ? st : 0],
		       st != PIMAX_CTRLFUSE_NONE && info.gyro ? " (gyro orientation)" : "");
		c->fuse_state = st;
	}
	if (st == PIMAX_CTRLFUSE_NONE) {
		return false;
	}
	struct xrt_pose pose = pose_from_rust(&aim);
	if (name == XRT_INPUT_TOUCH_GRIP_POSE) {
		// grip = aim * tracked_aim_to_grip: the Oculus Touch (Quest 2) openxr_aim -> openxr_grip transform (60 deg
		// pitch, handle centre 3.0 cm below and 9.5 cm behind the ray origin). SteamVR (Touch raw = grip *
		// inverse(openxr_grip), aim = raw * openxr_aim) then derives exactly this aim; the old orientation-only 45 deg
		// rule put SteamVR's ray 15 deg lower and the grip at the LED ring (rust/pimax-hands touch.rs).
		struct xrt_pose grip;
		math_pose_transform(&pose, &ctx->tracked_aim_to_grip, &grip);
		math_quat_normalize(&grip.orientation);
		pose = grip;
	}
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose = pose;
	uint32_t flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                 XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	if (st == PIMAX_CTRLFUSE_OPTICAL) {
		flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	rel.relation_flags = (enum xrt_space_relation_flags)flags;
	*out = rel;
	return true;
}

//! PIMAX_CTRL_ANGLE: clamp to the allowed range (NaN -> default 30).
static float
ctrl_angle_clamp(float deg)
{
	if (!(deg == deg)) {
		return 30.0f;
	}
	return deg < CTRL_ANGLE_MIN_DEG ? CTRL_ANGLE_MIN_DEG : (deg > CTRL_ANGLE_MAX_DEG ? CTRL_ANGLE_MAX_DEG : deg);
}

/*!
 * PIMAX_CTRL_ANGLE: pitch the pose about its own lateral (+X) axis. In OpenXR space the pointing ray is -Z and a
 * positive rotation about +X turns -Z towards +Y (up), so +30 raises the pointer 30 deg for grip and aim alike (the
 * grip is rigidly attached to the aim, so the SteamVR / xrizer aim derived from the grip moves with it). The position
 * stays. Left and right hands use the same sign.
 */
static void
apply_ctrl_angle(struct pimax_ctrl_ctx *ctx, struct xrt_pose *pose)
{
	int cdeg = atomic_load(&ctx->angle_cdeg);
	if (cdeg == 0) {
		return;
	}
	struct xrt_vec3 x = {1.0f, 0.0f, 0.0f};
	struct xrt_quat q, out;
	math_quat_from_angle_vector((float)cdeg * 0.01f * (float)(M_PI / 180.0), &x, &q);
	math_quat_rotate(&pose->orientation, &q, &out);
	math_quat_normalize(&out);
	pose->orientation = out;
}

static struct xrt_vec3
eng_rot(const struct xrt_quat *q, struct xrt_vec3 v)
{
	struct xrt_vec3 o;
	math_quat_rotate_vec3(q, &v, &o);
	return o;
}

/*!
 * PIMAX_CTRL_ENGINE: the pose of Pimax's LED engine. The engine gives the controller body (LED model frame) and the
 * head IMU pose it used, both in its world; the controller relative to the head goes under this HMD's own head pose
 * at the sample time (so recenter and room anchor apply as to the head), then on to @p at_ns with the engine's
 * velocity and the controller gyro, then into the aim frame (rot_x(-52 deg), as pimax-ctrltrack) and the Touch grip.
 * Not seen: position held, orientation turned by the gyro, for PIMAX_CTRL_TRACK_LOST_S; then false (rest pose).
 */
static bool
engine_pose(struct pimax_controller *c, int64_t at_ns, enum xrt_input_name name, struct xrt_space_relation *out)
{
	struct pimax_ctrl_ctx *ctx = c->ctx;
	const uint64_t now = os_monotonic_get_ns();
	struct pibridge_wb_ctrl_pose p;
	const bool seen = ctx->wb.ctrl_pose((uint32_t)c->index, &p) == 1 && (int64_t)now - p.ts_ns < 300000000LL;
	struct xrt_pose aim;
	uint32_t flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                 XRT_SPACE_RELATION_POSITION_VALID_BIT;
	int state;
	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	if (seen && ctx->head != NULL &&
	    xrt_device_get_tracked_pose(ctx->head, XRT_INPUT_GENERIC_HEAD_POSE, p.ts_ns, &head) == XRT_SUCCESS &&
	    (head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		// controller relative to the head IMU: inverse(head) * body, both from the engine
		const struct xrt_quat hq = {p.head_q[0], p.head_q[1], p.head_q[2], p.head_q[3]};
		const struct xrt_quat cq = {p.q[0], p.q[1], p.q[2], p.q[3]};
		struct xrt_quat hqi, rq;
		math_quat_invert(&hq, &hqi);
		math_quat_rotate(&hqi, &cq, &rq);
		const struct xrt_vec3 d = {p.p[0] - p.head_p[0], p.p[1] - p.head_p[1], p.p[2] - p.head_p[2]};
		struct xrt_vec3 rt = eng_rot(&hqi, d);
		// the HMD's head frame is the IMU frame moved to the head origin
		double h[3];
		pimax_camera_head_in_imu(h);
		rt.x -= (float)h[0];
		rt.y -= (float)h[1];
		rt.z -= (float)h[2];
		// under this HMD's head pose at the sample time
		struct xrt_quat Hq = head.pose.orientation;
		math_quat_normalize(&Hq);
		struct xrt_quat q;
		math_quat_rotate(&Hq, &rq, &q);
		struct xrt_vec3 pos = eng_rot(&Hq, rt);
		pos.x += head.pose.position.x;
		pos.y += head.pose.position.y;
		pos.z += head.pose.position.z;
		// on to at_ns: the engine's linear velocity (its world -> this world: Hq * inverse(engine head)), the
		// controller gyro for the turn (IMU -> body frame)
		struct xrt_quat R;
		math_quat_rotate(&Hq, &hqi, &R);
		const struct xrt_vec3 v = eng_rot(&R, (struct xrt_vec3){p.lin_vel[0], p.lin_vel[1], p.lin_vel[2]});
		float dt = (float)((double)(at_ns - p.ts_ns) / 1e9);
		dt = dt < -0.05f ? -0.05f : (dt > 0.025f ? 0.025f : dt); // Pimax caps prediction at 25 ms (frida pass 5)
		const struct xrt_quat dfi = {p.dev_from_imu[0], p.dev_from_imu[1], p.dev_from_imu[2], p.dev_from_imu[3]};
		pthread_mutex_lock(&c->lock);
		const struct xrt_vec3 g_imu = {(float)c->rep.raw_gyro[0] * ctx->gyro_scale,
		                               (float)c->rep.raw_gyro[1] * ctx->gyro_scale,
		                               (float)c->rep.raw_gyro[2] * ctx->gyro_scale};
		pthread_mutex_unlock(&c->lock);
		const struct xrt_vec3 w_body = eng_rot(&dfi, g_imu);
		// The engine's quaternion is world_from_IMU (confirmed: it rotates the raw accel to a steady world gravity;
		// frida pass 5). The device/LED-model frame is world_from_imu * inverse(ImuToDevice) (ImuToDevice = dfi =
		// device_from_imu, mirrored between the hands), then rot_x(-52) to the aim, as pimax-ctrltrack defines it.
		struct xrt_quat imu_from_dev;
		math_quat_invert(&dfi, &imu_from_dev);
		struct xrt_quat qdev;
		math_quat_rotate(&q, &imu_from_dev, &qdev); // world_from_device
		struct xrt_quat qp;
		math_quat_integrate_velocity(&qdev, &w_body, dt, &qp); // device-frame gyro, device-frame pose
		math_quat_normalize(&qp);
		pos.x += v.x * dt;
		pos.y += v.y * dt;
		pos.z += v.z * dt;
		aim.position = pos;
		math_quat_rotate(&qp, &ctx->device_from_aim, &aim.orientation);
		math_quat_normalize(&aim.orientation);
		flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
		state = 1;
		pthread_mutex_lock(&c->lock);
		c->eng_have = true;
		c->eng_aim = aim;
		c->eng_last_ns = now;
		c->eng_imu_rot = c->imu_rot;
		c->eng_dev_from_imu = dfi;
		pthread_mutex_unlock(&c->lock);
	} else {
		pthread_mutex_lock(&c->lock);
		const bool hold = c->eng_have && now - c->eng_last_ns < ctx->engine_lost_ns;
		if (hold) {
			// position held; orientation turned by what the IMU turned since the last engine pose (IMU frame ->
			// body -> aim frame)
			aim = c->eng_aim;
			if (c->bias_done) {
				struct xrt_quat i0, di, t1, dd, dfii, da, dai;
				math_quat_invert(&c->eng_imu_rot, &i0);
				math_quat_rotate(&i0, &c->imu_rot, &di);
				math_quat_rotate(&c->eng_dev_from_imu, &di, &t1);
				math_quat_invert(&c->eng_dev_from_imu, &dfii);
				math_quat_rotate(&t1, &dfii, &dd);
				math_quat_invert(&ctx->device_from_aim, &dai);
				math_quat_rotate(&dai, &dd, &t1);
				math_quat_rotate(&t1, &ctx->device_from_aim, &da);
				math_quat_rotate(&c->eng_aim.orientation, &da, &aim.orientation);
				math_quat_normalize(&aim.orientation);
			}
		}
		pthread_mutex_unlock(&c->lock);
		state = hold ? 2 : 0;
		if (!hold) {
			aim = (struct xrt_pose)XRT_POSE_IDENTITY;
		}
	}
	if (state != c->eng_state) {
		static const char *names[] = {"rest pose (Pimax's engine does not see it)", "Pimax's LED engine",
		                              "held, turned by the gyro (out of the cameras' view)"};
		C_INFO(c, "controller pose: %s", names[state]);
		c->eng_state = state;
	}
	if (state == 0) {
		return false;
	}
	struct xrt_pose pose = aim;
	if (name == XRT_INPUT_TOUCH_GRIP_POSE) {
		// the same Touch (Quest 2) aim -> grip transform as the optical path
		math_pose_transform(&aim, &ctx->tracked_aim_to_grip, &pose);
		math_quat_normalize(&pose.orientation);
	}
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose = pose;
	rel.relation_flags = (enum xrt_space_relation_flags)flags;
	*out = rel;
	return true;
}

static xrt_result_t
pimax_ctl_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct pimax_controller *c = ctl(xdev);
	if (name != XRT_INPUT_TOUCH_GRIP_POSE && name != XRT_INPUT_TOUCH_AIM_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&c->base, c->ctx->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	if (!ctl_shown(c, os_monotonic_get_ns())) {
		// PIMAX_CTRL_INHAND: lying on the desk (or switched off): no pose at all, so nothing is drawn for it
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return XRT_SUCCESS;
	}

	if (c->ctx->engine && engine_pose(c, at_timestamp_ns, name, out_relation)) {
		apply_ctrl_angle(c->ctx, &out_relation->pose);
		return XRT_SUCCESS;
	}

	if (c->ctx->track && optical_pose(c, at_timestamp_ns, name, out_relation)) {
		apply_ctrl_angle(c->ctx, &out_relation->pose);
		return XRT_SUCCESS;
	}

	// Synthesized placement (rust/pimax-hands rest.rs) from the head orientation at that time.
	float head_q[4] = {0, 0, 0, 1};
	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	if (c->ctx->head != NULL &&
	    xrt_device_get_tracked_pose(c->ctx->head, XRT_INPUT_GENERIC_HEAD_POSE, at_timestamp_ns, &head) == XRT_SUCCESS &&
	    (head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		head_q[0] = head.pose.orientation.x;
		head_q[1] = head.pose.orientation.y;
		head_q[2] = head.pose.orientation.z;
		head_q[3] = head.pose.orientation.w;
	}
	pimax_hands_pose aim, grip;
	pimax_hands_controller_poses(c->index == 1, head_q, &aim, &grip);
	struct xrt_pose pose = pose_from_rust(name == XRT_INPUT_TOUCH_AIM_POSE ? &aim : &grip);

	if (c->ctx->imu_mode) {
		pthread_mutex_lock(&c->lock);
		if (c->bias_done) {
			// Orientation from the controller IMU (aim); the grip keeps its fixed pitch relative to the aim.
			struct xrt_quat orient;
			math_quat_rotate(&c->yaw_ref, &c->imu_rot, &orient);
			math_quat_normalize(&orient);
			if (name == XRT_INPUT_TOUCH_GRIP_POSE) {
				struct xrt_quat a2g = c->ctx->aim_to_grip.orientation;
				math_quat_rotate(&orient, &a2g, &orient);
				math_quat_normalize(&orient);
			}
			pose.orientation = orient;
		}
		pthread_mutex_unlock(&c->lock);
	}

	apply_ctrl_angle(c->ctx, &pose);
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose = pose;
	// Position is a fixed guess, not tracked: valid but not "tracked".
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	*out_relation = rel;
	return XRT_SUCCESS;
}

//! Lock held: feed the queued reports (oldest first) into pimax-hands and refresh hands_out.
static void
hands_drain_locked(struct pimax_controller *c)
{
	if (c->hands == NULL || c->hq_count == 0) {
		return;
	}
	uint32_t start = (c->hq_head + HANDS_RING - c->hq_count) % HANDS_RING;
	for (uint32_t k = 0; k < c->hq_count; k++) {
		const uint32_t i = (start + k) % HANDS_RING;
		pimax_hands_update_report(c->hands, c->hq[i].ns, c->hq[i].trigger, c->hq[i].grip, c->hq[i].stick_x,
		                          c->hq[i].stick_y, c->hq[i].buttons, c->hq[i].touch);
	}
	c->hq_count = 0;
	pimax_hands_get(c->hands, &c->hands_out);
}

//! Fill the joint set of this hand from the Rust curl state; the hand follows the grip pose.
static xrt_result_t
pimax_ctl_get_hand_tracking(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            int64_t at_timestamp_ns,
                            struct xrt_hand_joint_set *out_value,
                            int64_t *out_timestamp_ns)
{
	struct pimax_controller *c = ctl(xdev);
	bool left = c->index == 0;
	if (name != (left ? XRT_INPUT_HT_CONFORMING_LEFT : XRT_INPUT_HT_CONFORMING_RIGHT)) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&c->base, c->ctx->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	uint64_t now = os_monotonic_get_ns();
	pimax_hands_out ho;
	bool shown = ctl_shown(c, now);
	pthread_mutex_lock(&c->lock);
	bool active = shown && c->have_report && (now - c->last_report_ns) < ACTIVE_TIMEOUT_NS;
	if (c->ctx->hands_demo) {
		// Scripted input instead of reports; always "active" so a test needs no awake controller.
		pimax_hands_input demo;
		pimax_hands_demo_input(now, !left, &demo);
		pimax_hands_update(c->hands, now, &demo);
		pimax_hands_get(c->hands, &c->hands_out);
		active = true;
	} else {
		hands_drain_locked(c);
	}
	ho = c->hands_out;
	if (active != c->hand_active_logged) {
		c->hand_active_logged = active;
		U_LOG_IFL_I(c->ctx->log_level, "hand tracking %s: %s (last report %.1f s ago)", left ? "left" : "right",
		            active ? "active" : (shown ? "inactive (no controller report)" : "inactive (controller put down)"),
		            c->have_report ? (double)(now - c->last_report_ns) / 1e9 : -1.0);
	}
	pthread_mutex_unlock(&c->lock);

	// Wrist = grip pose * fixed offset (the joint set is expressed relative to its hand_pose, in tracking-origin space).
	struct xrt_space_relation grip = XRT_SPACE_RELATION_ZERO;
	xrt_result_t xret = pimax_ctl_get_tracked_pose(xdev, XRT_INPUT_TOUCH_GRIP_POSE, at_timestamp_ns, &grip);
	if (xret != XRT_SUCCESS) {
		return xret;
	}
	struct xrt_relation_chain chain = {0};
	m_relation_chain_push_pose(&chain, &c->grip_to_wrist);
	m_relation_chain_push_relation(&chain, &grip);
	struct xrt_space_relation wrist = XRT_SPACE_RELATION_ZERO;
	m_relation_chain_resolve(&chain, &wrist);

	// Same order as struct u_hand_tracking_values.
	const pimax_hands_finger *src[5] = {&ho.little, &ho.ring, &ho.middle, &ho.index, &ho.thumb};
	struct u_hand_tracking_values values;
	struct u_hand_tracking_finger_value *dst[5] = {&values.little, &values.ring, &values.middle, &values.index,
	                                               &values.thumb};
	for (int f = 0; f < 5; f++) {
		dst[f]->splay = src[f]->splay;
		for (int j = 0; j < 4; j++) {
			dst[f]->joint_curls[j] = src[f]->joint_curls[j];
		}
		dst[f]->joint_count = 4;
	}

	u_hand_sim_simulate_generic(&values, left ? XRT_HAND_LEFT : XRT_HAND_RIGHT, &wrist, out_value);

	// The simulation marks the hand pose fully tracked; keep the real flags of the (synthesized) grip pose.
	out_value->hand_pose = wrist;
	out_value->is_active = active;
	*out_timestamp_ns = at_timestamp_ns;
	return XRT_SUCCESS;
}

/*!
 * PIMAX_HAPTICS: OpenXR vibration -> cmd 0x07 `58 09 07 SS AMP DUR_lo DUR_hi` (Windows capture 2026-10-15). libpimax
 * maps amplitude 0..1 -> 1..100 %, duration ns -> ms (XR_MIN_HAPTIC_DURATION -> 12 ms, what games stream), ignores the
 * frequency (not in the protocol), and sends at most one command per 8 ms per hand (newest wins) on its session
 * thread. A stop (amplitude 0) sends nothing, like Pimax Play: the pulse just ends.
 */
static xrt_result_t
pimax_ctl_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	struct pimax_controller *c = ctl(xdev);
	if (name != XRT_OUTPUT_NAME_TOUCH_HAPTIC || value == NULL || value->type != XRT_OUTPUT_VALUE_TYPE_VIBRATION) {
		return XRT_SUCCESS;
	}
	pimax_t *lib = atomic_load(&c->ctx->hap_lib);
	if (lib == NULL) {
		return XRT_SUCCESS; // not started yet / stopping
	}
	int rc = pimax_haptic(lib, c->slot, value->vibration.amplitude, value->vibration.duration_ns);
	if (atomic_fetch_add(&c->ctx->hap_calls, 1) == 0) {
		U_LOG_IFL_I(c->ctx->log_level, "PIMAX_HAPTICS: first vibration (slot %u, amplitude %.2f, %lld ns) -> %d",
		            c->slot, value->vibration.amplitude, (long long)value->vibration.duration_ns, rc);
	}
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_ctl_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct pimax_controller *c = ctl(xdev);
	pthread_mutex_lock(&c->lock);
	bool have = c->have_report;
	uint8_t b = c->raw46;
	pthread_mutex_unlock(&c->lock);
	*out_present = have;
	*out_charging = (b & 0x80) != 0;
	*out_charge = have ? clampf((float)(b & 0x7f) / 100.0f, 0.0f, 1.0f) : 0.0f;
	return XRT_SUCCESS;
}

/*
 * Reports (session thread).
 */

static void
imu_update(struct pimax_controller *c, const pimax_controller_report *r, uint64_t now)
{
	struct pimax_ctrl_ctx *ctx = c->ctx;
	float a[3], g[3];
	for (int i = 0; i < 3; i++) {
		a[i] = (float)r->raw_accel[i] / 1024.0f; // g
		g[i] = (float)r->raw_gyro[i] * ctx->gyro_scale;
	}

	if (!c->fusion_init) {
		m_imu_3dof_init(&c->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
		c->fusion_init = true;
	}

	if (!c->bias_done) {
		// Average ~1 s while still (|gyro| below ~12 deg/s sum), else restart.
		float gm = (fabsf(g[0]) + fabsf(g[1]) + fabsf(g[2])) * (float)(180.0 / M_PI);
		if (gm > 12.0f) {
			c->bias_n = 0;
			memset(c->gyro_sum, 0, sizeof(c->gyro_sum));
			memset(c->acc_sum, 0, sizeof(c->acc_sum));
			return;
		}
		for (int i = 0; i < 3; i++) {
			c->gyro_sum[i] += g[i];
			c->acc_sum[i] += a[i];
		}
		if (++c->bias_n < BIAS_SAMPLES) {
			return;
		}
		struct xrt_vec3 gb = {(float)(c->gyro_sum[0] / c->bias_n), (float)(c->gyro_sum[1] / c->bias_n),
		                      (float)(c->gyro_sum[2] / c->bias_n)};
		struct xrt_vec3 ac = {(float)(c->acc_sum[0] / c->bias_n), (float)(c->acc_sum[1] / c->bias_n),
		                      (float)(c->acc_sum[2] / c->bias_n)};
		c->fusion.gyro_bias.value = gb;
		struct xrt_vec3 up = {0, 1, 0};
		struct xrt_vec3 acn = m_vec3_normalize(ac);
		math_quat_from_vec_a_to_vec_b(&acn, &up, &c->fusion.rot);
		math_quat_normalize(&c->fusion.rot);
		c->yaw_ref = head_yaw(c, (int64_t)now);
		c->bias_done = true;
		C_INFO(c, "IMU bias done (gyro %.4f %.4f %.4f rad/s, |acc| %.2f g)", gb.x, gb.y, gb.z,
		       m_vec3_len(ac));
		return;
	}

	struct xrt_vec3 accel = {a[0] * 9.80665f, a[1] * 9.80665f, a[2] * 9.80665f};
	struct xrt_vec3 gyro = {g[0], g[1], g[2]};
	m_imu_3dof_update(&c->fusion, now, &accel, &gyro);
	c->imu_rot = c->fusion.rot;
}

//! $XDG_RUNTIME_DIR/pibridge/recenter-progress (PiBridge's recenter ring): written atomically, only when the value moved
//! by 1/40 (not a file write per report).
static void
recenter_progress_publish(double prog, int *last_q)
{
	const int q = (int)(prog * 40.0 + 0.5);
	if (q == *last_q) {
		return;
	}
	*last_q = q;
	const char *rt = getenv("XDG_RUNTIME_DIR");
	char dir[512], path[600], tmp[620];
	snprintf(dir, sizeof(dir), "%s/pibridge", (rt != NULL && rt[0] != '\0') ? rt : "/tmp");
	(void)mkdir(dir, 0700);
	snprintf(path, sizeof(path), "%s/recenter-progress", dir);
	snprintf(tmp, sizeof(tmp), "%s.tmp", path);
	FILE *f = fopen(tmp, "w");
	if (f == NULL) {
		return;
	}
	fprintf(f, "%.3f\n", prog);
	fclose(f);
	(void)rename(tmp, path);
}

void
pimax_ctrl_on_report(struct pimax_ctrl_ctx *ctx, const pimax_controller_report *r)
{
	if (ctx == NULL || (r->slot != PIMAX_SLOT_LEFT && r->slot != PIMAX_SLOT_RIGHT)) {
		return;
	}
	struct pimax_controller *c = ctx->ctl[r->slot == PIMAX_SLOT_LEFT ? 0 : 1];
	if (c == NULL) {
		return;
	}
	uint64_t now = os_monotonic_get_ns();
	pthread_mutex_lock(&c->lock);
	if (!c->have_report) {
		C_INFO(c, "first report (slot %u)", r->slot);
	}
	c->have_report = true;
	c->last_report_ns = now;
	c->reports++;
	c->rep = *r;
	c->trigger = (float)r->trigger / 255.0f;
	c->grip = (float)r->grip / 255.0f;
	c->stick_x = clampf((float)r->stick_x / 127.0f, -1.0f, 1.0f);
	c->stick_y = clampf((float)r->stick_y / 127.0f, -1.0f, 1.0f);
	c->btn_a = (r->buttons & PIMAX_BTN_A) != 0;
	c->btn_b = (r->buttons & PIMAX_BTN_B) != 0;
	// The Menu button (byte 47 bit 2, PiBridge's libpibridge_pimax decodes it): a plain click (left MENU_CLICK, right
	// SYSTEM_CLICK: the SteamVR dashboard).
	c->btn_menu = (r->buttons & PIMAX_BTN_APP_MENU) != 0;
	// The Pimax button (byte 48 bit 0, PIMAX_BTN_MENU in pimax.h): recenter on a LONG press (~0.8 s hold), so a brief
	// touch never recenters by accident; not forwarded to the apps.
	const bool sys = (r->buttons & PIMAX_BTN_MENU) != 0;
	const uint64_t now_ns = os_monotonic_get_ns();
	if (sys && !c->btn_system) {
		c->system_press_ns = now_ns;
		c->system_recentered = false;
	}
	// PiBridge's recenter ring (the overlay draws it from this file, as under SteamVR from pibridge-headtrack's): the
	// hold progress 0..1 of the holding hand, only on a change of 1/40, 0 at the release
	if (sys || c->btn_system) {
		const double prog = !sys ? 0.0 : (double)(now_ns - c->system_press_ns) / (800.0 * 1000 * 1000);
		recenter_progress_publish(prog > 1.0 ? 1.0 : prog, &c->system_progress_q);
	}
	if (sys && !c->system_recentered && now_ns - c->system_press_ns > 800 * 1000 * 1000ULL) {
		C_WARN(c, "PIMAX_RECENTER: Pimax button long-press (%s): recentering",
		       r->slot == PIMAX_SLOT_LEFT ? "left" : "right");
		pimax_hmd_request_recenter();
		c->system_recentered = true;
	}
	c->btn_system = sys;
	c->btn_stick = (r->buttons & PIMAX_BTN_STICK) != 0;
	c->t_a = (r->touch & 0x01) != 0;
	c->t_b = (r->touch & 0x02) != 0;
	c->t_stick = (r->touch & PIMAX_TOUCH_STICK) != 0;
	c->raw46 = r->raw[46];
	if (debug_get_bool_option_pimax_ctrl_button_log() && memcmp(c->last_btn_bytes, &r->raw[44], 7) != 0) {
		memcpy(c->last_btn_bytes, &r->raw[44], 7);
		C_WARN(c,
		       "PIMAX_CTRL_BUTTON_LOG: slot %u (%s) bytes 44..50 = %02x %02x %02x %02x %02x %02x %02x (buttons 0x%x "
		       "touch 0x%x)",
		       (unsigned)r->slot, r->slot == PIMAX_SLOT_LEFT ? "left" : "right", r->raw[44], r->raw[45], r->raw[46],
		       r->raw[47], r->raw[48], r->raw[49], r->raw[50], (unsigned)r->buttons, (unsigned)r->touch);
	}
	if (c->hands != NULL && !ctx->hands_demo) {
		// Bit meanings (touch, buttons), normalisation, filters and the thumb state machine are all in Rust. Only
		// queue the report here (session thread); get_hand_tracking runs the filters when a client wants hands.
		const uint32_t i = c->hq_head;
		c->hq[i].ns = now;
		c->hq[i].buttons = r->buttons;
		c->hq[i].stick_x = (int16_t)r->stick_x;
		c->hq[i].stick_y = (int16_t)r->stick_y;
		c->hq[i].trigger = r->trigger;
		c->hq[i].grip = r->grip;
		c->hq[i].touch = r->touch;
		c->hq_head = (i + 1) % HANDS_RING;
		if (c->hq_count < HANDS_RING) {
			c->hq_count++;
		}
		if (ctx->hands_eager) {
			hands_drain_locked(c);
		}
	}
	if (c->fuse != NULL) {
		pimax_ctrlfuse_report(c->fuse, now, r->raw); // own lock
	}
	// own lock; IMU + input changes -> in the hand or not
	pimax_inhand_report(c->inhand, now, r->raw, r->trigger, r->grip, r->stick_x, r->stick_y, r->buttons, r->touch);
	if (ctx->engine) {
		// every IMU sample to Pimax's LED engine, in the controller's raw axes (m/s^2, rad/s), as Pimax Play does
		float a[3], g[3];
		for (int i = 0; i < 3; i++) {
			a[i] = (float)r->raw_accel[i] / 1024.0f * 9.80665f;
			g[i] = (float)r->raw_gyro[i] * ctx->gyro_scale;
		}
		ctx->wb.push_ctrl_imu((uint32_t)c->index, (int64_t)now, a, g);
	}
	if (ctx->imu_mode || ctx->engine) {
		imu_update(c, r, now); // the engine mode turns a held pose with the gyro
	}
	pthread_mutex_unlock(&c->lock);
}

void
pimax_ctrl_on_status(struct pimax_ctrl_ctx *ctx, const pimax_controller_status *s)
{
	if (ctx == NULL || (s->slot != PIMAX_SLOT_LEFT && s->slot != PIMAX_SLOT_RIGHT)) {
		return;
	}
	struct pimax_controller *c = ctx->ctl[s->slot == PIMAX_SLOT_LEFT ? 0 : 1];
	if (c != NULL) {
		atomic_store(&c->last_status_ns, os_monotonic_get_ns());
	}
}

void
pimax_ctrl_on_state(struct pimax_ctrl_ctx *ctx, pimax_state state)
{
	if (ctx != NULL && state == PIMAX_CONNECTED) {
		atomic_store(&ctx->need_attach_all, true);
	}
}

/*
 * Attach thread: libpimax_controller_attach() sleeps, so it cannot run on the session thread.
 */

static void
do_attach(struct pimax_ctrl_ctx *ctx, struct pimax_controller *c, const char *why)
{
	if (c == NULL) {
		return;
	}
	atomic_store(&c->last_attach_ns, os_monotonic_get_ns());
	int rc = pimax_controller_attach(ctx->lib, c->slot);
	unsigned n = atomic_fetch_add(&c->attaches, 1) + 1;
	U_LOG_IFL_I(ctx->log_level, "controller slot %u attach burst #%u (%s) -> %d", c->slot, n, why, rc);
}

static void *
attach_thread(void *arg)
{
	struct pimax_ctrl_ctx *ctx = arg;
	atomic_store(&ctx->need_attach_all, true);

	while (!atomic_load(&ctx->stop)) {
		uint64_t now = os_monotonic_get_ns();
		if (atomic_exchange(&ctx->need_attach_all, false)) {
			for (int i = 0; i < 2 && !atomic_load(&ctx->stop); i++) {
				do_attach(ctx, ctx->ctl[i], "start/reconnect");
			}
		}
		for (int i = 0; i < 2 && !atomic_load(&ctx->stop); i++) {
			struct pimax_controller *c = ctx->ctl[i];
			if (c == NULL) {
				continue;
			}
			pthread_mutex_lock(&c->lock);
			uint64_t last_rep = c->have_report ? c->last_report_ns : 0;
			pthread_mutex_unlock(&c->lock);
			uint64_t st = atomic_load(&c->last_status_ns);
			uint64_t at = atomic_load(&c->last_attach_ns);
			// 0x37 heartbeat = awake but not attached. Valid if newer than our last attach and than the
			// last report, with no report for >1.5 s.
			if (st > at && st > last_rep && now - last_rep > 1500ull * 1000 * 1000 && now - at > REATTACH_MIN_NS) {
				do_attach(ctx, c, "0x37 heartbeat without reports");
			}
		}
		// PIMAX_CTRL_INHAND: a mode change from the app / CLI / voice, the visibility log and the status file.
		int req = pimax_inhand_poll_mode();
		if (req >= 0 && req != atomic_load(&ctx->inhand_mode)) {
			atomic_store(&ctx->inhand_mode, req);
			U_LOG_IFL_I(ctx->log_level, "PIMAX_CTRL_INHAND: controllers %s (run-time change)",
			            pimax_inhand_mode_name(req));
		}
		// PIMAX_CTRL_ANGLE: live change from the app / CLI ($XDG_RUNTIME_DIR/pimax-ctrl-angle holds the degrees)
		{
			const char *rt = getenv("XDG_RUNTIME_DIR");
			if (rt != NULL && rt[0] != '\0') {
				char path[256];
				snprintf(path, sizeof(path), "%s/pimax-ctrl-angle", rt);
				FILE *af = fopen(path, "r");
				if (af != NULL) {
					float deg = 0.0f;
					int ok = fscanf(af, "%f", &deg) == 1;
					fclose(af);
					int cd = (int)lroundf(ctrl_angle_clamp(deg) * 100.0f);
					if (ok && cd != atomic_load(&ctx->angle_cdeg)) {
						atomic_store(&ctx->angle_cdeg, cd);
						U_LOG_IFL_I(ctx->log_level, "PIMAX_CTRL_ANGLE: controller angle %+.1f deg (run-time change)",
						            (double)cd / 100.0);
					}
				}
			}
		}
		// PiBridge: a recenter asked for by the dashboards ($XDG_RUNTIME_DIR/pibridge/recenter-request rewritten):
		// the same recenter as the Pimax button's long press
		{
			const char *rt = getenv("XDG_RUNTIME_DIR");
			char path[300];
			snprintf(path, sizeof(path), "%s/pibridge/recenter-request", (rt != NULL && rt[0] != '\0') ? rt : "/tmp");
			struct stat st;
			if (stat(path, &st) == 0) {
				const int64_t m = (int64_t)st.st_mtim.tv_sec * 1000000000LL + st.st_mtim.tv_nsec;
				if (ctx->recenter_request_mtime != 0 && m != ctx->recenter_request_mtime) {
					U_LOG_IFL_W(ctx->log_level, "PIMAX_RECENTER: recenter asked for by PiBridge's dashboard");
					pimax_hmd_request_recenter();
				}
				ctx->recenter_request_mtime = m;
			} else {
				ctx->recenter_request_mtime = -1; // absent: the first request after this one counts
			}
		}
		for (int i = 0; i < 2; i++) {
			struct pimax_controller *c = ctx->ctl[i];
			if (c == NULL) {
				continue;
			}
			int shown = ctl_shown(c, now) ? 1 : 0;
			if (shown != c->shown_logged) {
				char d[96];
				pimax_inhand_describe(c->inhand, now, d, sizeof(d));
				U_LOG_IFL_I(ctx->log_level, "PIMAX_CTRL_INHAND: %s controller %s (%s, mode %s)", i == 0 ? "left" : "right",
				            shown ? "shown" : "hidden", d, pimax_inhand_mode_name(atomic_load(&ctx->inhand_mode)));
				c->shown_logged = shown;
			}
		}
		pimax_inhand_publish(atomic_load(&ctx->inhand_mode), ctx->ctl[0] ? ctx->ctl[0]->inhand : NULL,
		                     ctx->ctl[1] ? ctx->ctl[1]->inhand : NULL, now);

		// Test pulse from the app / `pimax controllers buzz` ($XDG_RUNTIME_DIR/pimax-haptic-request).
		int hs = pimax_haptic_poll_request(ctx->lib);
		if (hs > 0) {
			U_LOG_IFL_I(ctx->log_level, "PIMAX_HAPTICS: test pulse on slot %d (app request)", hs);
		} else if (hs < 0) {
			U_LOG_IFL_W(ctx->log_level, "PIMAX_HAPTICS: malformed test pulse request ignored");
		}
#ifdef PIMAX_IPD
		// Lens IPD motor from the app's IPD slider ($XDG_RUNTIME_DIR/pibridge/ipd-request): Monado owns the headset
		// during an inside-out session, so the request is sent from here (pibridge-keepalived does it otherwise).
		int ipd = pimax_ipd_poll_request(ctx->lib);
		if (ipd > 0) {
			U_LOG_IFL_I(ctx->log_level, "PIMAX_IPD: lens motor set to %.2f mm (app request)", ipd / 100.0);
		} else if (ipd < 0) {
			U_LOG_IFL_W(ctx->log_level, "PIMAX_IPD: IPD request not sent (%d)", ipd);
		}
#endif
		struct timespec ts = {0, 100 * 1000 * 1000};
		nanosleep(&ts, NULL);
	}
	return NULL;
}

void
pimax_ctrl_start(struct pimax_ctrl_ctx *ctx, pimax_t *lib)
{
	ctx->lib = lib;
	atomic_store(&ctx->hap_lib, lib);
	U_LOG_IFL_I(ctx->log_level, "PIMAX_HAPTICS: controller vibration on (cmd 0x07, <= 1 command per 8 ms per hand)");
	atomic_store(&ctx->stop, false);
	if (pthread_create(&ctx->thread, NULL, attach_thread, ctx) == 0) {
		ctx->thread_running = true;
	} else {
		U_LOG_IFL_E(ctx->log_level, "pimax controllers: could not start attach thread");
	}
}

void
pimax_ctrl_stop(struct pimax_ctrl_ctx *ctx)
{
	if (ctx == NULL) {
		return;
	}
	atomic_store(&ctx->hap_lib, NULL);
	if (ctx->thread_running) {
		atomic_store(&ctx->stop, true);
		pthread_join(ctx->thread, NULL);
		ctx->thread_running = false;
	}
	if (ctx->track_thread_running) {
		atomic_store(&ctx->track_stop, true);
		pthread_join(ctx->track_thread, NULL);
		ctx->track_thread_running = false;
	}
	ctx->lib = NULL;
	ctx->head = NULL; // the HMD is going away
	ctx_unref(ctx);
}

/*
 * Creation.
 */

static struct pimax_controller *
ctl_create(struct pimax_ctrl_ctx *ctx, int index)
{
	bool left = index == 0;
	// The hand tracking input is the last one; leave it out when hand tracking is off.
	const int input_count = ctx->hands ? I_COUNT : I_HAND_TRACKING;
	struct pimax_controller *c =
	    U_DEVICE_ALLOCATE(struct pimax_controller, U_DEVICE_ALLOC_TRACKING_NONE, input_count, 1);
	if (c == NULL) {
		return NULL;
	}
	c->ctx = ctx;
	c->index = index;
	c->slot = left ? PIMAX_SLOT_LEFT : PIMAX_SLOT_RIGHT;
	pthread_mutex_init(&c->lock, NULL);
	c->imu_rot = (struct xrt_quat)XRT_QUAT_IDENTITY;
	c->yaw_ref = (struct xrt_quat)XRT_QUAT_IDENTITY;

	struct xrt_input *in = c->base.inputs;
	in[I_BTN1_CLICK].name = left ? XRT_INPUT_TOUCH_X_CLICK : XRT_INPUT_TOUCH_A_CLICK;
	in[I_BTN1_TOUCH].name = left ? XRT_INPUT_TOUCH_X_TOUCH : XRT_INPUT_TOUCH_A_TOUCH;
	in[I_BTN2_CLICK].name = left ? XRT_INPUT_TOUCH_Y_CLICK : XRT_INPUT_TOUCH_B_CLICK;
	in[I_BTN2_TOUCH].name = left ? XRT_INPUT_TOUCH_Y_TOUCH : XRT_INPUT_TOUCH_B_TOUCH;
	// the Menu button: Touch has /input/menu/click on the left hand only and /input/system/click on the right only
	// (bindings.json), so the right one must be SYSTEM_CLICK or no app can bind it (it was MENU_CLICK on both hands:
	// the right menu button did nothing anywhere)
	in[I_MENU].name = left ? XRT_INPUT_TOUCH_MENU_CLICK : XRT_INPUT_TOUCH_SYSTEM_CLICK;
	in[I_SQUEEZE].name = XRT_INPUT_TOUCH_SQUEEZE_VALUE;
	in[I_TRIGGER_TOUCH].name = XRT_INPUT_TOUCH_TRIGGER_TOUCH;
	in[I_TRIGGER_VALUE].name = XRT_INPUT_TOUCH_TRIGGER_VALUE;
	in[I_STICK_CLICK].name = XRT_INPUT_TOUCH_THUMBSTICK_CLICK;
	in[I_STICK_TOUCH].name = XRT_INPUT_TOUCH_THUMBSTICK_TOUCH;
	in[I_STICK].name = XRT_INPUT_TOUCH_THUMBSTICK;
	in[I_GRIP_POSE].name = XRT_INPUT_TOUCH_GRIP_POSE;
	in[I_AIM_POSE].name = XRT_INPUT_TOUCH_AIM_POSE;
	for (int i = 0; i < input_count; i++) {
		in[i].active = (i >= I_GRIP_POSE);
	}
	c->base.outputs[0].name = XRT_OUTPUT_NAME_TOUCH_HAPTIC;

	// Share the HMD's tracking origin so the controllers sit in the head frame (poses are head-relative); otherwise
	// the builder adds its default 3DoF hand offsets (+-0.2, 1.3, -0.5) on top of ours. The origin lives inside the
	// HMD allocation, which outlives every use (it is only read while the device list is alive).
	if (ctx->head != NULL && ctx->head->tracking_origin != NULL) {
		c->base.tracking_origin = ctx->head->tracking_origin;
	}

	c->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	c->base.device_type = left ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	snprintf(c->base.str, XRT_DEVICE_NAME_LEN, "Pimax Dream Air %s Controller", left ? "Left" : "Right");
	snprintf(c->base.serial, XRT_DEVICE_NAME_LEN, "pimax-ctrl-%s", left ? "left" : "right");
	c->base.binding_profiles = left ? profiles_left : profiles_right;
	c->base.binding_profile_count = ARRAY_SIZE(profiles_left);
	c->base.supported.orientation_tracking = ctx->imu_mode || ctx->track;
	c->base.supported.position_tracking = ctx->track;
	c->base.supported.battery_status = true;

	if (ctx->hands) {
		c->hands = pimax_hands_create();
	}
	c->inhand = pimax_inhand_create(left ? 0 : 1);
	c->shown_logged = -1;
	c->eng_state = -1;
	c->eng_imu_rot = (struct xrt_quat)XRT_QUAT_IDENTITY;
	c->eng_dev_from_imu = (struct xrt_quat)XRT_QUAT_IDENTITY;
	if (ctx->track) {
		c->fuse = pimax_ctrlfuse_create(left ? 0 : 1);
		char d[256];
		pimax_ctrlfuse_describe(c->fuse, d, sizeof(d));
		U_LOG_IFL_I(ctx->log_level, "Pimax %s controller optical tracking: %s", left ? "left" : "right", d);
	}
	if (ctx->hands) {
		if (c->hands == NULL) {
			U_LOG_IFL_E(ctx->log_level, "pimax-hands: out of memory, hand tracking off for this controller");
			c->base.input_count = I_HAND_TRACKING; // drop the unused hand tracking input
		}
	}
	if (c->hands != NULL) {
		pimax_hands_get(c->hands, &c->hands_out);
		pimax_hands_pose p;
		pimax_hands_grip_to_wrist(left ? 0 : 1, &p);
		c->grip_to_wrist.orientation = (struct xrt_quat){p.orientation[0], p.orientation[1], p.orientation[2],
		                                                  p.orientation[3]};
		c->grip_to_wrist.position = (struct xrt_vec3){p.position[0], p.position[1], p.position[2]};
		in[I_HAND_TRACKING].name = left ? XRT_INPUT_HT_CONFORMING_LEFT : XRT_INPUT_HT_CONFORMING_RIGHT;
		in[I_HAND_TRACKING].active = true;
		c->base.get_hand_tracking = pimax_ctl_get_hand_tracking;
		c->base.supported.hand_tracking = true;
	}

	c->base.update_inputs = pimax_ctl_update_inputs;
	c->base.get_tracked_pose = pimax_ctl_get_tracked_pose;
	c->base.set_output = pimax_ctl_set_output;
	c->base.get_battery_status = pimax_ctl_get_battery_status;
	c->base.destroy = pimax_ctl_destroy;

	u_var_add_root(c, c->base.str, true);
	u_var_add_f32(c, &c->trigger, "trigger");
	u_var_add_f32(c, &c->grip, "grip");
	u_var_add_f32(c, &c->stick_x, "stick x");
	u_var_add_f32(c, &c->stick_y, "stick y");
	u_var_add_bool(c, &c->btn_a, "A/X");
	u_var_add_bool(c, &c->btn_b, "B/Y");
	u_var_add_bool(c, &c->btn_menu, "menu");
	if (c->hands != NULL) {
		u_var_add_ro_f32(c, &c->hands_out.curl[3], "hand: index curl");
		u_var_add_ro_f32(c, &c->hands_out.curl[2], "hand: middle curl");
		u_var_add_ro_f32(c, &c->hands_out.curl[4], "hand: thumb curl");
	}
	return c;
}

bool
pimax_ctrl_demo_enabled(void)
{
	return debug_get_bool_option_pimax_ctrl_hands() && debug_get_bool_option_pimax_ctrl_hands_demo();
}

struct pimax_ctrl_ctx *
pimax_ctrl_create(struct xrt_device *head, enum u_logging_level log_level, struct xrt_device *out_devs[2])
{
	struct pimax_ctrl_ctx *ctx = U_TYPED_CALLOC(struct pimax_ctrl_ctx);
	if (ctx == NULL) {
		return NULL;
	}
	ctx->ref.count = 3; // HMD + 2 controllers
	ctx->head = head;
	ctx->log_level = log_level;
	const char *mode = debug_get_option_pimax_ctrl_pose();
	ctx->imu_mode = mode != NULL && strcmp(mode, "imu") == 0;
	float lsb = debug_get_float_option_pimax_ctrl_gyro_lsb_dps();
	ctx->gyro_scale = (float)(M_PI / 180.0) / (lsb > 0.0f ? lsb : 8.2f);
	ctx->hands = debug_get_bool_option_pimax_ctrl_hands();
	ctx->hands_demo = ctx->hands && debug_get_bool_option_pimax_ctrl_hands_demo();
	const char *feed = debug_get_option_pimax_ctrl_hands_feed();
	ctx->hands_eager = ctx->hands && feed != NULL && strcmp(feed, "report") == 0;
	ctx->track = debug_get_bool_option_pimax_ctrl_track();
	if (debug_get_bool_option_pimax_ctrl_engine()) {
		ctx->engine = pimax_wb_load(&ctx->wb);
		const float pitch = debug_get_float_option_pimax_ctrl_engine_aim_pitch();
		struct xrt_vec3 x = {1.0f, 0.0f, 0.0f};
		math_quat_from_angle_vector(-pitch * (float)(M_PI / 180.0), &x, &ctx->device_from_aim);
		const float lost = debug_get_float_option_pimax_ctrl_engine_lost_s();
		ctx->engine_lost_ns = (uint64_t)((lost > 0.0f ? lost : 4.0f) * 1e9);
		if (ctx->engine) {
			U_LOG_IFL_I(log_level,
			            "Pimax controllers: PIMAX_CTRL_ENGINE on: poses from Pimax's LED engine (winbridge), aim pitch "
			            "%.1f deg, held by the gyro for %.1f s out of view",
			            (double)pitch, (double)ctx->engine_lost_ns / 1e9);
		} else {
			U_LOG_IFL_E(log_level,
			            "Pimax controllers: PIMAX_CTRL_ENGINE=1 but VIT_SYSTEM_LIBRARY_PATH is not PiBridge's winbridge "
			            "library: no engine poses");
		}
	}
	// in hand: always unless PIMAX_CTRL_INHAND says otherwise (`pimax desk` passes auto, games keep always)
	atomic_store(&ctx->inhand_mode, pimax_inhand_mode_from_env(PIMAX_INHAND_MODE_ALWAYS));
	pimax_inhand_clear_request();
	atomic_store(&ctx->angle_cdeg, (int)lroundf(ctrl_angle_clamp(debug_get_float_option_pimax_ctrl_angle()) * 100.0f));
	U_LOG_IFL_I(log_level,
	            "Pimax controllers: PIMAX_CTRL_ANGLE=%+.1f deg (pointer tilted up by this much; -20..+70, live file "
	            "$XDG_RUNTIME_DIR/pimax-ctrl-angle)",
	            (double)atomic_load(&ctx->angle_cdeg) / 100.0);
	U_LOG_IFL_I(log_level,
	            "Pimax controllers: PIMAX_CTRL_INHAND=%s (auto: shown only while in the hand; 5 s still or no reports "
	            "for 1.5 s = put down, motion > 3.7 deg/s for 120 ms, a tilt or an input = picked up)",
	            pimax_inhand_mode_name(atomic_load(&ctx->inhand_mode)));
	pthread_mutex_init(&ctx->track_lock, NULL);
	pimax_ctrltrack_default_path(ctx->track_path, sizeof(ctx->track_path));
	if (ctx->track) {
		U_LOG_IFL_I(log_level, "Pimax controllers: optical 6DoF on (PIMAX_CTRL_TRACK=1): poses from %s (pimax-ctrltrack live)",
		            ctx->track_path);
	}
	pimax_hands_pose a2g;
	pimax_hands_aim_to_grip(&a2g);
	ctx->aim_to_grip = pose_from_rust(&a2g);
	pimax_hands_tracked_aim_to_grip(&a2g);
	ctx->tracked_aim_to_grip = pose_from_rust(&a2g);
	U_LOG_IFL_I(log_level,
	            "Pimax controllers: PIMAX_TOUCH_GRIP tracked grip = aim * (%.4f %.4f %.4f, quat %.4f %.4f %.4f %.4f) "
	            "(Quest 2 Touch openxr_aim -> openxr_grip, same aim as SteamVR)",
	            ctx->tracked_aim_to_grip.position.x, ctx->tracked_aim_to_grip.position.y,
	            ctx->tracked_aim_to_grip.position.z, ctx->tracked_aim_to_grip.orientation.x,
	            ctx->tracked_aim_to_grip.orientation.y, ctx->tracked_aim_to_grip.orientation.z,
	            ctx->tracked_aim_to_grip.orientation.w);
	char placement[256];
	if (pimax_hands_controller_describe(placement, sizeof(placement)) != 0) {
		U_LOG_IFL_W(log_level, "Pimax controllers: %s", placement);
	} else {
		U_LOG_IFL_I(log_level, "Pimax controllers: %s", placement);
	}
	if (ctx->hands && pimax_hands_abi_version() != PIMAX_HANDS_ABI_VERSION) {
		U_LOG_IFL_E(log_level, "pimax-hands ABI %u != header %u: stale libpimax_capi.a, hand tracking off",
		            pimax_hands_abi_version(), PIMAX_HANDS_ABI_VERSION);
		ctx->hands = ctx->hands_demo = false;
	}

	for (int i = 0; i < 2; i++) {
		ctx->ctl[i] = ctl_create(ctx, i);
		if (ctx->ctl[i] == NULL) {
			// Not worth unwinding in detail; this only fails on OOM.
			return NULL;
		}
		out_devs[i] = &ctx->ctl[i]->base;
	}
	if (ctx->track) {
		// body model: the hands see each other (never closer than 8 cm)
		pimax_ctrlfuse_link(ctx->ctl[0]->fuse, ctx->ctl[1]->fuse);
		atomic_store(&ctx->track_stop, false);
		if (pthread_create(&ctx->track_thread, NULL, track_thread, ctx) == 0) {
			ctx->track_thread_running = true;
		} else {
			U_LOG_IFL_E(log_level, "Pimax controllers: could not start the optical pose thread");
		}
	}
	U_LOG_IFL_I(log_level, "Pimax controllers: pose mode %s%s, hand tracking %s", ctx->imu_mode ? "imu" : "fixed",
	            ctx->imu_mode ? " (gyro scale is a guess)" : "",
	            ctx->hands ? (ctx->hands_demo ? "on (DEMO pattern)"
	                                          : (ctx->hands_eager ? "on (data source: controller, feed per report)"
	                                                              : "on (data source: controller, lazy feed)"))
	                       : "off");
	return ctx;
}
