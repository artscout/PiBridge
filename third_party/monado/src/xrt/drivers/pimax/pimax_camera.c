// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air tracking cameras -> xrt_frame sinks -> Mercury. Thin glue: capture, tile split,
 *         metadata, timestamps and calibration parsing live in the Rust crate pimax-cam (pimax_cam.h).
 *
 * Graph (one capture thread):
 *   pimax-cam (V4L2 or .pxcam replay) -> tracking-exposure frames (25 Hz; LED frames only reach the debug sinks)
 *     PIMAX_HT_CAMS tiles -> pimax_ht_push (camera hands, pimax_ht.c: own low-priority thread, newest pair only)
 *     PIMAX_SLAM_CAMS tiles (IMU-timed only) -> SLAM tracker (t_slam / VIT, Basalt) and/or EuRoC recorder
 *   HMD IMU thread -> pimax_camera_push_imu -> SLAM tracker IMU sink and/or EuRoC recorder
 *   camera hands -> world-locked with the head pose at the frame time (6DoF when SLAM tracks) -> two hand
 *     devices: XR_EXT_hand_tracking joints + XR_EXT_hand_interaction (pinch, poke, aim, grasp).
 *   HMD get_tracked_pose -> pimax_camera_head_relation: tracker pose checked by the Rust supervisor
 *     (pimax_cam_track_*: acquire/lose/reset, continuity) or the 3DoF orientation at the held position.
 *
 * @ingroup drv_pimax
 */

#include "pimax_camera.h"

#ifndef PIMAX_HAVE_CAM

// Built without -DPIMAX_CAM=ON: no camera support, the HMD works as before.
#include "util/u_debug.h"
#include "util/u_logging.h"

DEBUG_GET_ONCE_OPTION(pimax_hands_stub, "PIMAX_HANDS", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_cam_stub, "PIMAX_CAM", false)

DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam_stub, "PIMAX_SLAM", false)
DEBUG_GET_ONCE_OPTION(pimax_slam_record_stub, "PIMAX_SLAM_RECORD", NULL)

struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user,
                    pimax_camera_orient_fn orient,
                    void *orient_user,
                    pimax_camera_send_fn send,
                    void *send_user)
{
	(void)hmd;
	(void)clock;
	(void)clock_user;
	(void)orient;
	(void)orient_user;
	(void)send;
	(void)send_user;
	const char *hands = debug_get_option_pimax_hands_stub();
	if (hands != NULL || debug_get_bool_option_pimax_cam_stub() || debug_get_bool_option_pimax_slam_stub() ||
	    debug_get_option_pimax_slam_record_stub() != NULL) {
		U_LOG_IFL_W(log_level,
		            "PIMAX_HANDS/PIMAX_CAM/PIMAX_SLAM set, but Monado was built without -DPIMAX_CAM=ON");
	}
	return NULL;
}

bool
pimax_camera_wants_imu(const struct pimax_camera *pc)
{
	(void)pc;
	return false;
}

void
pimax_camera_push_imu(struct pimax_camera *pc, int64_t ts_ns, const struct xrt_vec3 *accel_m_s2,
                      const struct xrt_vec3 *gyro_rad_s)
{
	(void)pc;
	(void)ts_ns;
	(void)accel_m_s2;
	(void)gyro_rad_s;
}

bool
pimax_camera_head_relation(struct pimax_camera *pc,
                           int64_t at_ns,
                           const struct xrt_space_relation *rel3dof,
                           struct xrt_space_relation *out)
{
	(void)pc;
	(void)at_ns;
	(void)rel3dof;
	(void)out;
	return false;
}

void
pimax_camera_take_hand_devices(struct pimax_camera *pc, struct xrt_device *out[2])
{
	(void)pc;
	out[0] = out[1] = NULL;
}

void
pimax_camera_destroy(struct pimax_camera **pc_ptr)
{
	(void)pc_ptr;
}

#else // PIMAX_HAVE_CAM

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"

#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_sink.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "tracking/t_tracking.h"
#include "tracking/t_hand_tracking.h"
#include "tracking/t_euroc_recorder.h"

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
#include "pimax_ht.h"
#include "pimax_ht_calib.h"
#include "pimax_interface.h"
#endif

#include "pimax_cam.h"
#include "pimax_winbridge.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

DEBUG_GET_ONCE_OPTION(pimax_hands, "PIMAX_HANDS", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_cam, "PIMAX_CAM", false)
DEBUG_GET_ONCE_OPTION(pimax_cam_source, "PIMAX_CAM_SOURCE", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_record, "PIMAX_CAM_RECORD", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_calib, "PIMAX_CAM_CALIB", NULL)
DEBUG_GET_ONCE_OPTION(pimax_ht_cams, "PIMAX_HT_CAMS", "0,1")
DEBUG_GET_ONCE_OPTION(pimax_ht_orient, "PIMAX_HT_ORIENT", NULL)
DEBUG_GET_ONCE_OPTION(pimax_ht_head_offset, "PIMAX_HT_HEAD_OFFSET", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ht_stats, "PIMAX_HT_STATS", false)
DEBUG_GET_ONCE_OPTION(pimax_ht_roles, "PIMAX_HT_ROLES", "tracking")
DEBUG_GET_ONCE_NUM_OPTION(pimax_ht_nice, "PIMAX_HT_NICE", 10)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ht_hz, "PIMAX_HT_HZ", 25.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ht_duty, "PIMAX_HT_DUTY", 0.5f)
DEBUG_GET_ONCE_NUM_OPTION(pimax_ht_stale_ms, "PIMAX_HT_STALE_MS", 150)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ht_predict_ms, "PIMAX_HT_PREDICT_MS", 0.0f)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam, "PIMAX_SLAM", false)
DEBUG_GET_ONCE_OPTION(pimax_slam_cams, "PIMAX_SLAM_CAMS", "0,1")
DEBUG_GET_ONCE_OPTION(pimax_slam_record, "PIMAX_SLAM_RECORD", NULL)
DEBUG_GET_ONCE_NUM_OPTION(pimax_slam_cam_offset_us, "PIMAX_SLAM_CAM_OFFSET_US", 0)
DEBUG_GET_ONCE_OPTION(pimax_slam_imu, "PIMAX_SLAM_IMU", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_ae, "PIMAX_CAM_AE", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam_exposure_mid, "PIMAX_SLAM_EXPOSURE_MID", true)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam_passthrough, "PIMAX_SLAM_PASSTHROUGH", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_engine_leds, "PIMAX_CTRL_ENGINE", false)

//! Head origin in the IMU frame of the running SLAM setup (pimax_winbridge.h: the controllers use it).
static double g_head_in_imu[3];

void
pimax_camera_head_in_imu(double out[3])
{
	out[0] = g_head_in_imu[0];
	out[1] = g_head_in_imu[1];
	out[2] = g_head_in_imu[2];
}

enum ae_mode
{
	AE_OFF = 0,
	AE_DRY = 1,  //!< controller runs, nothing is sent
	AE_SEND = 2, //!< exposure push sent through pc->send
};

//! Basalt's world is z-up (gravity -z); OpenXR is y-up: rotate -90 deg about X (same fix as the Rift S driver).
static const struct xrt_quat SLAM_TO_YUP = {0.70710678f, 0.0f, 0.0f, -0.70710678f};

#define CAM_TRACE(pc, ...) U_LOG_IFL_T(pc->log_level, __VA_ARGS__)
#define CAM_DEBUG(pc, ...) U_LOG_IFL_D(pc->log_level, __VA_ARGS__)
#define CAM_INFO(pc, ...) U_LOG_IFL_I(pc->log_level, __VA_ARGS__)
#define CAM_WARN(pc, ...) U_LOG_IFL_W(pc->log_level, __VA_ARGS__)
#define CAM_ERROR(pc, ...) U_LOG_IFL_E(pc->log_level, __VA_ARGS__)

static_assert(sizeof(struct pimax_camera_clock_ref) == sizeof(pimax_cam_imu_clock_ref), "clock ref layout");

struct pimax_camera
{
	enum u_logging_level log_level;
	struct xrt_device *hmd;
	pimax_camera_clock_fn clock;
	void *clock_user;

	pimax_cam *cam;
	pimax_cam_calib calib;
	char source_desc[256];

	struct xrt_frame_context xfctx;
	struct os_thread_helper thread;
	bool thread_started;

	//! Camera hands (pimax_ht.c).
	bool hands;
	uint32_t ht_cams[2];
	struct pimax_ht *ht;
	bool stats_ht;

	//! Debug GUI: the four tiles of every frame.
	struct u_sink_debug debug[PIMAX_CAM_TILES];

	//! Stats (capture thread writes; u_var reads, races are harmless).
	uint64_t frames, tracking_frames, pairs_pushed, imu_timed, host_timed, errors, timeouts;
	float fps, tracking_fps;
	int64_t win_start_ns;
	uint64_t win_frames, win_tracking;
	int64_t last_frame_ts;

	//! 6DoF (PIMAX_SLAM) and EuRoC recording (PIMAX_SLAM_RECORD).
	pimax_camera_orient_fn orient;
	void *orient_user;
	bool slam_on;          //!< tracker running
	bool record_on;        //!< recorder running
	uint32_t slam_cams[4]; //!< factory camera ids in tracker order
	uint32_t slam_n;
	int64_t slam_cam_offset_ns;
	bool exposure_mid; //!< tracker frame time = stamp + exposure/2 (the stamp marks the exposure start)
	struct t_slam_calibration slam_calib;
	struct xrt_tracked_slam *slam;
	struct xrt_slam_sinks *slam_sinks;
	struct xrt_slam_sinks *rec_sinks;
	char rec_path[512];
	struct os_mutex slam_lock; //!< supervisor + tracker queries (compositor and camera threads)
	bool slam_lock_init;
	pimax_cam_track *track;
	pimax_cam_track_out tout;
	double head_in_imu[3];
	int64_t last_poll_ns;
	uint64_t slam_frames, slam_untimed, imu_pushed;
	//! IMU gate for the tracker (slam_lock): samples handed to it since the last frame set, held samples.
	uint32_t slam_imu_since_frame;
	bool slam_imu_frame_seen, slam_imu_holding;
	uint64_t slam_imu_held, slam_imu_gaps;
	char slam_status[128];
	//! PIMAX_SLAM_PASSTHROUGH: the tracker's pose as it comes (Pimax's own engine through libpibridge_vit_winbridge.so,
	//! used as Pimax Play uses it): no supervisor, still gate, tilt correction or drift hold, world = the tracker's
	//! world until a recenter. slam_lock.
	bool passthrough;
	bool pt_tracking;      //!< a fresh tracker pose was there at the last query
	bool pt_zeroed;        //!< the engine's large saved-map offset was removed at the first pose (start near origin)
	bool pt_recenter;      //!< recenter at the next head query
	struct xrt_quat pt_wq; //!< world_from_slam rotation (yaw)
	struct xrt_vec3 pt_wt; //!< world_from_slam translation
	struct xrt_quat pt_fix; //!< 6DoF orientation * 3DoF orientation^-1 at the last tracked query (kept while lost)
	struct xrt_vec3 pt_held; //!< head position at the last tracked query (held while lost)
	uint64_t pt_acquired, pt_lost;
	//! PIMAX_CTRL_ENGINE: the controller LED frames go to Pimax's LED engine (pimax_winbridge.h), camera thread only.
	bool leds_on;
	//! The Motion-JPEG profile of this firmware was seen on a frame (logged once).
	bool jpeg_seen;
	struct pimax_wb wb;
	uint8_t *led_buf; //!< 4 tiles 640x480
	uint64_t leds_pushed;
	struct u_var_button recenter_btn;
	int64_t last_info_ns;

	//! Auto-exposure (PIMAX_CAM_AE): Rust controller, the HMD sends.
	pimax_camera_send_fn send;
	void *send_user;
	enum ae_mode ae_mode;
	pimax_cam_ae *ae;
	pimax_cam_ae_out ae_out;
	uint64_t ae_sent, ae_send_errors;
	char ae_status[160];
};

static bool
parse_u32_pair(const char *s, uint32_t out[2])
{
	unsigned a = 0, b = 0;
	if (s == NULL || sscanf(s, "%u,%u", &a, &b) != 2) {
		return false;
	}
	out[0] = a;
	out[1] = b;
	return true;
}

static uint64_t
frame_time(struct pimax_camera *pc, const pimax_cam_frame_info *fi, bool *out_timed)
{
	*out_timed = false;
	// The Motion-JPEG stream of a Crystal Super carries no device timestamp (its metadata block does not survive
	// JPEG), so there is nothing to map: the host time of the frame is already on the monotonic timeline the IMU
	// samples are mapped onto. Frame-to-IMU alignment then carries the USB delivery latency instead of the
	// device-clock sync, which is what every other UVC camera driver lives with.
	if ((fi->flags & PIMAX_CAM_FLAG_NO_DEV_TIME) != 0) {
		pc->host_timed++;
		*out_timed = true;
		return fi->mono_ts_ns;
	}
	// A replay's device timestamps come from another session: use its rebased monotonic time instead.
	if ((fi->flags & PIMAX_CAM_FLAG_REPLAY) == 0 && pc->clock != NULL) {
		struct pimax_camera_clock_ref ref;
		if (pc->clock(pc->clock_user, &ref) && ref.valid) {
			pimax_cam_imu_clock_ref r;
			memcpy(&r, &ref, sizeof(r));
			uint64_t t = pimax_cam_dev_to_mono_ns(fi->dev_ts_us, &r);
			if (t != 0) {
				pc->imu_timed++;
				*out_timed = true;
				return t;
			}
		}
	}
	return fi->mono_ts_ns;
}

static struct xrt_frame *
tile_frame(struct pimax_camera *pc, uint32_t tile, uint64_t ts, const pimax_cam_frame_info *fi)
{
	struct xrt_frame *xf = NULL;
	u_frame_create_one_off(XRT_FORMAT_L8, PIMAX_CAM_TILE_W, PIMAX_CAM_TILE_H, &xf);
	if (xf == NULL) {
		return NULL;
	}
	if (pimax_cam_copy_tile(pc->cam, tile, xf->data, (uint32_t)xf->stride) != 0) {
		CAM_ERROR(pc, "copy tile %u: %s", tile, pimax_cam_last_error());
		xrt_frame_reference(&xf, NULL);
		return NULL;
	}
	xf->timestamp = (int64_t)ts;
	xf->source_timestamp = (int64_t)(fi->dev_ts_us * 1000);
	xf->source_sequence = fi->counter;
	xf->source_id = tile;
	return xf;
}

//! One tracking frame through the auto-exposure controller; sends only in AE_SEND mode.
static void
ae_step(struct pimax_camera *pc, int64_t now)
{
	if (pc->ae == NULL) {
		return;
	}
	pimax_cam_ae_out *o = &pc->ae_out;
	int r = pimax_cam_ae_update(pc->ae, pc->cam, (uint64_t)now, o);
	if (r < 0) {
		return;
	}
	if (o->changed) {
		CAM_DEBUG(pc, "AE: target exposure %u/%u %u/%u gain %u/%u %u/%u (frame %u/%u, mean %.0f/%.0f, sat %.3f/%.3f)",
		          o->exposure[0], o->exposure[2], o->exposure[1], o->exposure[3], o->gain[0], o->gain[2], o->gain[1],
		          o->gain[3], o->frame_exposure[0], o->frame_gain[0], o->mean[0], o->mean[1], o->sat[0], o->sat[1]);
	}
	if (r == 1 && pc->ae_mode == AE_SEND && pc->send != NULL) {
		if (pc->send(pc->send_user, PIMAX_CAM_AE_CMD, PIMAX_CAM_AE_OPT, o->payload, PIMAX_CAM_AE_PAYLOAD_LEN) == 0) {
			pc->ae_sent++;
		} else if (pc->ae_send_errors++ % 50 == 0) {
			CAM_WARN(pc, "AE: exposure push failed (%" PRIu64 " errors)", pc->ae_send_errors);
		}
	}
	snprintf(pc->ae_status, sizeof(pc->ae_status),
	         "%s: lower %u/%u mean %.0f, upper %u/%u mean %.0f; frame %u/%u; %" PRIu64 " changes, %" PRIu64 " sent",
	         pc->ae_mode == AE_SEND ? "on" : "dry", o->exposure[0], o->gain[0], o->mean[0], o->exposure[2],
	         o->gain[2], o->mean[1], o->frame_exposure[0], o->frame_gain[0], o->changes, pc->ae_sent);
}

static void
update_stats(struct pimax_camera *pc, int64_t now)
{
	if (pc->win_start_ns == 0) {
		pc->win_start_ns = now;
		return;
	}
	double dt = (double)(now - pc->win_start_ns) / 1e9;
	if (dt < 5.0) {
		return;
	}
	pc->fps = (float)(pc->win_frames / dt);
	pc->tracking_fps = (float)(pc->win_tracking / dt);
	pimax_cam_stats st = {0};
	pimax_cam_get_stats(pc->cam, &st);
	// Only the Motion-JPEG profile counts here; the note stays out of the line otherwise.
	char host_note[48] = "";
	if (pc->host_timed != 0) {
		snprintf(host_note, sizeof host_note, ", %" PRIu64 " host-timed", pc->host_timed);
	}
	CAM_INFO(pc,
	         "camera: %.2f fps (tracking %.2f), total %" PRIu64 " frames, lost %" PRIu64 ", corrupt %" PRIu64
	         ", IMU-timed %" PRIu64 ", Mercury pairs pushed %" PRIu64 "%s",
	         pc->fps, pc->tracking_fps, st.frames, st.dropped, st.corrupt, pc->imu_timed, pc->pairs_pushed, host_note);
	if (pc->slam_on || pc->record_on) {
		CAM_INFO(pc,
		         "slam: %" PRIu64 " frame sets, %" PRIu64 " IMU samples (%" PRIu64 " held from the tracker in %" PRIu64
		         " camera gaps), %" PRIu64 " untimed frames skipped%s%s",
		         pc->slam_frames, pc->imu_pushed, pc->slam_imu_held, pc->slam_imu_gaps, pc->slam_untimed,
		         pc->slam_on ? "; " : "", pc->slam_on ? pc->slam_status : "");
	}
	if (pc->ae != NULL) {
		CAM_INFO(pc, "AE %s", pc->ae_status);
	}
	if (pc->slam_on && pc->track != NULL && !pc->passthrough) {
		pimax_cam_track_info ti;
		os_mutex_lock(&pc->slam_lock);
		int ok = pimax_cam_track_get_info(pc->track, &ti);
		os_mutex_unlock(&pc->slam_lock);
		if (ok == 0) {
			CAM_INFO(pc,
			         "6DoF output: gyro bias (%.3f %.3f %.3f) deg/s, correction %.1f mm / %.2f deg (max %.1f mm / "
			         "%.2f deg), %" PRIu64 " blended, %" PRIu64 " snapped",
			         ti.gyro_bias[0] * 57.2958, ti.gyro_bias[1] * 57.2958, ti.gyro_bias[2] * 57.2958,
			         sqrt(ti.corr_pos[0] * ti.corr_pos[0] + ti.corr_pos[1] * ti.corr_pos[1] +
			              ti.corr_pos[2] * ti.corr_pos[2]) * 1000.0,
			         sqrt(ti.corr_rot[0] * ti.corr_rot[0] + ti.corr_rot[1] * ti.corr_rot[1] +
			              ti.corr_rot[2] * ti.corr_rot[2]) * 57.2958,
			         ti.max_corr_pos * 1000.0, ti.max_corr_rot * 57.2958, ti.blended, ti.snapped);
		}
		// Every output correction step of >= 20 mm (PIMAX_SLAM_BIG_CORR_MM) since the last stats: when, which
		// axis, how fast the head moved (corrections hide in motion; at rest they show as sliding).
		int logged = 0, more = 0;
		for (;;) {
			pimax_cam_track_corr c;
			os_mutex_lock(&pc->slam_lock);
			int got = pimax_cam_track_take_big_corr(pc->track, &c);
			os_mutex_unlock(&pc->slam_lock);
			if (got != 1) {
				break;
			}
			if (logged >= 8) {
				more++;
				continue;
			}
			logged++;
			double h = sqrt(c.step[0] * c.step[0] + c.step[2] * c.step[2]);
			double v = fabs(c.step[1]);
			double off = sqrt(c.offset[0] * c.offset[0] + c.offset[1] * c.offset[1] + c.offset[2] * c.offset[2]);
			CAM_INFO(pc,
			         "6DoF correction: %.0f mm (%s: vertical %.0f, horizontal %.0f), pending %.0f mm / %.2f deg, "
			         "head %.2f m/s %.0f deg/s, %s, t %.3f s",
			         sqrt(h * h + v * v) * 1000.0, v > h ? "vertical" : "horizontal", c.step[1] * 1000.0,
			         h * 1000.0, off * 1000.0, c.rot * 57.2958, c.speed, c.ang_speed * 57.2958,
			         c.pose_only ? "tracker pose" : "supervisor", (double)c.ts_ns / 1e9);
		}
		if (more > 0) {
			CAM_INFO(pc, "6DoF correction: %d more >= 20 mm not shown", more);
		}
	}
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	pimax_ht_log_stats(pc->ht);
#endif
	pc->win_start_ns = now;
	pc->win_frames = pc->win_tracking = 0;
}

/*
 *
 * 6DoF: SLAM tracker, supervisor, recorder.
 *
 */

//! Tracker relation (z-up world) -> y-up world, in place.
static void
slam_rel_to_yup(struct xrt_space_relation *r)
{
	math_quat_rotate(&SLAM_TO_YUP, &r->pose.orientation, &r->pose.orientation);
	math_quat_rotate_vec3(&SLAM_TO_YUP, &r->pose.position, &r->pose.position);
	math_quat_rotate_vec3(&SLAM_TO_YUP, &r->linear_velocity, &r->linear_velocity);
	math_quat_rotate_vec3(&SLAM_TO_YUP, &r->angular_velocity, &r->angular_velocity);
}

static const char *
track_state_name(uint32_t s)
{
	switch (s) {
	case PIMAX_CAM_TRACK_TRACKING: return "6DoF tracking";
	case PIMAX_CAM_TRACK_LOST: return "lost (3DoF, position held)";
	default: return "waiting for the tracker (3DoF)";
	}
}

//! Feed the newest tracker pose to the supervisor. Caller holds slam_lock.
static void
slam_poll_locked(struct pimax_camera *pc, int64_t now_ns)
{
	if (pc->passthrough) {
		return; // no supervisor: its loss and reset rules are for Basalt; the engine handles loss itself
	}
	pimax_cam_track_vio smp;
	const pimax_cam_track_vio *sp = NULL;
	int64_t ts = 0;
	struct xrt_space_relation r = XRT_SPACE_RELATION_ZERO;
	struct xrt_quat q3;
	if (t_slam_get_latest_pose(pc->slam, &ts, &r) && pc->orient != NULL && pc->orient(pc->orient_user, ts, &q3)) {
		slam_rel_to_yup(&r);
		smp.ts_ns = (uint64_t)ts;
		smp.pos[0] = r.pose.position.x;
		smp.pos[1] = r.pose.position.y;
		smp.pos[2] = r.pose.position.z;
		smp.q[0] = r.pose.orientation.x;
		smp.q[1] = r.pose.orientation.y;
		smp.q[2] = r.pose.orientation.z;
		smp.q[3] = r.pose.orientation.w;
		smp.vel[0] = r.linear_velocity.x;
		smp.vel[1] = r.linear_velocity.y;
		smp.vel[2] = r.linear_velocity.z;
		smp.q3dof[0] = q3.x;
		smp.q3dof[1] = q3.y;
		smp.q3dof[2] = q3.z;
		smp.q3dof[3] = q3.w;
		sp = &smp;
	}
	pimax_cam_track_out o;
	if (pimax_cam_track_update_vio(pc->track, (uint64_t)now_ns, sp, &o) != 0) {
		return;
	}
	pc->tout = o;
	pc->last_poll_ns = now_ns;
	if (o.events & PIMAX_CAM_TRACK_EV_ACQUIRED) {
		CAM_INFO(pc, "6DoF: %s (acquisition %" PRIu64 ")", o.reason, o.acquisitions);
	}
	if (o.events & PIMAX_CAM_TRACK_EV_LOST) {
		CAM_WARN(pc, "6DoF: lost (%s); 3DoF at the last position", o.reason);
	}
	if (o.events & PIMAX_CAM_TRACK_EV_RECENTERED) {
		CAM_INFO(pc, "6DoF: recentered");
	}
	if (o.events & PIMAX_CAM_TRACK_EV_RESET_WANTED) {
		// Raised after a long loss, or when the tracker drifts while the IMU says the headset lies still
		// (no landmarks: Basalt integrates the IMU alone; pimax-cam StillParams, PIMAX_SLAM_STILL=0 disables).
		// The still case is a re-zero: output held, no 6DoF loss (PIMAX_SLAM_STILL_REZERO=0: old drop to 3DoF).
		if (o.events & PIMAX_CAM_TRACK_EV_REZEROED) {
			CAM_INFO(pc, "6DoF: tracker re-zeroed while still (position held, no dropout; reset %" PRIu64 ")",
			         o.resets);
		} else {
			CAM_WARN(pc, "6DoF: resetting the tracker (%s; reset %" PRIu64 ")", o.reason, o.resets);
		}
		t_slam_reset(pc->slam);
	}
	// A still re-zero is held output in a fresh tracker world, not a dropout (the state is Lost only internally).
	const bool rezeroing = o.state == PIMAX_CAM_TRACK_LOST && strncmp(o.reason, "re-zeroed", 9) == 0;
	snprintf(pc->slam_status, sizeof(pc->slam_status), "%s; %" PRIu64 " acquired, %" PRIu64 " lost, %s",
	         rezeroing ? "6DoF tracking (re-zeroing while still, position held)" : track_state_name(o.state),
	         o.acquisitions, o.losses, o.reason);
}

bool
pimax_camera_wants_imu(const struct pimax_camera *pc)
{
	return pc != NULL && (pc->slam_on || pc->record_on);
}

/*!
 * IMU gate for the SLAM tracker. Basalt (frame_to_frame flow + sqrt VIO) has a blocking IMU queue of 300
 * samples that its VIO thread drains only when a frame arrives. A camera gap longer than 300 ms at 1 kHz (the
 * first auto-exposure push re-inits the sensors: ~0.3-0.5 s without frames, as on Windows) fills it; the push
 * then blocks the libpimax session thread (IMU, keepalive, controllers all stop), and the next frame deadlocks
 * the tracker: the optical flow waits for an IMU sample newer than the frame, which never comes because the
 * session thread is stuck on the VIO queue, whose thread waits for the optical flow. The image queue (10) then
 * fills and blocks the camera thread too. analysis/ae-stall.md in pimax-re.
 *
 * So hand the tracker at most SLAM_IMU_FULL samples per frame set at full rate, then every SLAM_IMU_DECIM-th
 * sample (a coarse IMU across the gap) up to SLAM_IMU_SPARSE more, then nothing until the next frame set.
 * Nothing before the first frame set either (the tracker does not need it). The predictor and the recorder
 * still get every sample.
 */
#define SLAM_IMU_FULL 200u
#define SLAM_IMU_DECIM 10u
#define SLAM_IMU_SPARSE 60u

//! Caller holds slam_lock. True = hand this sample to the tracker.
static bool
slam_imu_gate_locked(struct pimax_camera *pc)
{
	if (!pc->slam_imu_frame_seen) {
		return false;
	}
	uint32_t n = pc->slam_imu_since_frame++;
	if (n < SLAM_IMU_FULL) {
		return true;
	}
	n -= SLAM_IMU_FULL;
	return n < SLAM_IMU_SPARSE * SLAM_IMU_DECIM && n % SLAM_IMU_DECIM == 0;
}

void
pimax_camera_push_imu(struct pimax_camera *pc, int64_t ts_ns, const struct xrt_vec3 *accel_m_s2,
                      const struct xrt_vec3 *gyro_rad_s)
{
	if (!pimax_camera_wants_imu(pc)) {
		return;
	}
	struct xrt_imu_sample s = {
	    .timestamp_ns = ts_ns,
	    .accel_m_s2 = {accel_m_s2->x, accel_m_s2->y, accel_m_s2->z},
	    .gyro_rad_secs = {gyro_rad_s->x, gyro_rad_s->y, gyro_rad_s->z},
	};
	if (pc->slam_on) {
		// Same sample into the Rust predictor (rotation at IMU rate between tracker poses).
		os_mutex_lock(&pc->slam_lock);
		const double a3[3] = {accel_m_s2->x, accel_m_s2->y, accel_m_s2->z};
		const double g3[3] = {gyro_rad_s->x, gyro_rad_s->y, gyro_rad_s->z};
		pimax_cam_track_push_imu(pc->track, ts_ns, a3, g3);
		bool feed = slam_imu_gate_locked(pc);
		bool started = !feed && !pc->slam_imu_holding && pc->slam_imu_frame_seen;
		if (!feed) {
			pc->slam_imu_held++;
			if (started) {
				pc->slam_imu_gaps++;
			}
		}
		pc->slam_imu_holding = !feed && pc->slam_imu_frame_seen;
		uint64_t gaps = pc->slam_imu_gaps;
		os_mutex_unlock(&pc->slam_lock);
		if (started && (gaps <= 3 || gaps % 100 == 0)) {
			CAM_WARN(pc,
			         "camera gap: holding IMU back from the tracker until the next frame set (gap %" PRIu64
			         "; Basalt's 300-sample IMU queue would otherwise block the HID thread)",
			         gaps);
		}
		// Basalt's push blocks when its queue is full; the gate keeps it below capacity (slam_imu_gate_locked).
		if (feed) {
			xrt_sink_push_imu(pc->slam_sinks->imus[0], &s);
		}
	}
	if (pc->record_on) {
		xrt_sink_push_imu(pc->rec_sinks->imus[0], &s);
	}
	pc->imu_pushed++;
}

static void
rel_to_c(const struct xrt_space_relation *r, pimax_cam_rel *c)
{
	memset(c, 0, sizeof(*c));
	c->pos[0] = r->pose.position.x;
	c->pos[1] = r->pose.position.y;
	c->pos[2] = r->pose.position.z;
	c->q[0] = r->pose.orientation.x;
	c->q[1] = r->pose.orientation.y;
	c->q[2] = r->pose.orientation.z;
	c->q[3] = r->pose.orientation.w;
	c->lin_vel[0] = r->linear_velocity.x;
	c->lin_vel[1] = r->linear_velocity.y;
	c->lin_vel[2] = r->linear_velocity.z;
	c->ang_vel[0] = r->angular_velocity.x;
	c->ang_vel[1] = r->angular_velocity.y;
	c->ang_vel[2] = r->angular_velocity.z;
	c->flags = (uint32_t)r->relation_flags;
}

static void
rel_from_c(const pimax_cam_rel *c, struct xrt_space_relation *r)
{
	r->pose.position = (struct xrt_vec3){(float)c->pos[0], (float)c->pos[1], (float)c->pos[2]};
	r->pose.orientation = (struct xrt_quat){(float)c->q[0], (float)c->q[1], (float)c->q[2], (float)c->q[3]};
	r->linear_velocity = (struct xrt_vec3){(float)c->lin_vel[0], (float)c->lin_vel[1], (float)c->lin_vel[2]};
	r->angular_velocity = (struct xrt_vec3){(float)c->ang_vel[0], (float)c->ang_vel[1], (float)c->ang_vel[2]};
	r->relation_flags = (enum xrt_space_relation_flags)c->flags;
}

//! A tracker pose older than this is no pose (the supervisor's default stale time).
#define PT_STALE_NS (350 * U_TIME_1MS_IN_NS)
//! Extrapolate at most this far: Pimax Play's predictor caps requests at 25 ms (frida pass 5: the lead is a ns
//! timestamp in the PosePredict record; head ~16 ms, app/controllers clamp at exactly 25 ms).
#define PT_MAX_PREDICT_S 0.025f

static struct xrt_vec3
pt_add(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static struct xrt_vec3
pt_rot(const struct xrt_quat *q, struct xrt_vec3 v)
{
	struct xrt_vec3 o;
	math_quat_rotate_vec3(q, &v, &o);
	return o;
}

/*!
 * PIMAX_SLAM_PASSTHROUGH head pose, as Pimax Play uses its engine: the newest tracker pose (IMU frame in the tracker
 * world, y-up), moved on to @p at_ns with the tracker's linear velocity and the gyro (the 3DoF relation's angular
 * velocity, taken into the IMU frame), then the head origin and world_from_slam (identity until a recenter) on top.
 * Pimax's PosePredict does the same constant-velocity step. Without a fresh pose: the 3DoF orientation, continuous
 * with the last tracked one, at the last head position.
 */
static bool
passthrough_head(struct pimax_camera *pc,
                 int64_t at_ns,
                 const struct xrt_space_relation *rel3dof,
                 struct xrt_space_relation *out)
{
	int64_t ts = 0;
	struct xrt_space_relation r = XRT_SPACE_RELATION_ZERO;
	os_mutex_lock(&pc->slam_lock);
	const int64_t now = os_monotonic_get_ns();
	const bool fresh = t_slam_get_latest_pose(pc->slam, &ts, &r) && now - ts < PT_STALE_NS;
	*out = *rel3dof;
	if (!fresh) {
		if (pc->pt_tracking) {
			pc->pt_tracking = false;
			pc->pt_lost++;
			CAM_WARN(pc, "6DoF: no tracker pose for %d ms; 3DoF at the last position", (int)(PT_STALE_NS / 1000000));
			snprintf(pc->slam_status, sizeof(pc->slam_status),
			         "lost (3DoF, position held); %" PRIu64 " acquired, %" PRIu64 " lost (engine pose as is)",
			         pc->pt_acquired, pc->pt_lost);
		}
		math_quat_rotate(&pc->pt_fix, &rel3dof->pose.orientation, &out->pose.orientation);
		math_quat_normalize(&out->pose.orientation);
		out->pose.position = pc->pt_held;
		out->angular_velocity = pt_rot(&pc->pt_fix, rel3dof->angular_velocity);
		out->linear_velocity = (struct xrt_vec3){0, 0, 0};
		out->relation_flags = (enum xrt_space_relation_flags)(
		    (rel3dof->relation_flags & ~(XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
		                                 XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT)) |
		    XRT_SPACE_RELATION_POSITION_VALID_BIT);
		os_mutex_unlock(&pc->slam_lock);
		return true;
	}
	slam_rel_to_yup(&r);

	// gyro in the IMU frame: the 3DoF relation's world angular velocity rotated back by its orientation
	struct xrt_quat q3inv;
	math_quat_invert(&rel3dof->pose.orientation, &q3inv);
	const struct xrt_vec3 w_body = pt_rot(&q3inv, rel3dof->angular_velocity);
	float dt = (float)((double)(at_ns - ts) / 1e9);
	if (dt > PT_MAX_PREDICT_S) {
		dt = PT_MAX_PREDICT_S;
	}
	struct xrt_quat q = r.pose.orientation;
	math_quat_normalize(&q);
	struct xrt_quat qp;
	math_quat_integrate_velocity(&q, &w_body, dt, &qp);
	math_quat_normalize(&qp);
	const struct xrt_vec3 v = r.linear_velocity;
	const struct xrt_vec3 p = pt_add(r.pose.position, (struct xrt_vec3){v.x * dt, v.y * dt, v.z * dt});

	const struct xrt_vec3 h = {(float)pc->head_in_imu[0], (float)pc->head_in_imu[1], (float)pc->head_in_imu[2]};
	const struct xrt_vec3 hr = pt_rot(&qp, h);
	const struct xrt_vec3 head_slam = pt_add(p, hr);
	const struct xrt_vec3 w_slam = pt_rot(&qp, w_body);

	if (pc->pt_recenter) {
		// yaw of the head's forward (-z) to 0 and its floor position to the origin; the height stays
		pc->pt_recenter = false;
		struct xrt_quat qw;
		math_quat_rotate(&pc->pt_wq, &qp, &qw);
		const struct xrt_vec3 fwd = pt_rot(&qw, (struct xrt_vec3){0, 0, -1});
		const float yaw = atan2f(-fwd.x, -fwd.z);
		struct xrt_quat undo;
		math_quat_from_angle_vector(-yaw, &(struct xrt_vec3){0, 1, 0}, &undo);
		struct xrt_quat wq;
		math_quat_rotate(&undo, &pc->pt_wq, &wq);
		pc->pt_wq = wq;
		pc->pt_wt = pt_rot(&undo, pc->pt_wt);
		const struct xrt_vec3 hw = pt_add(pt_rot(&pc->pt_wq, head_slam), pc->pt_wt);
		pc->pt_wt.x -= hw.x;
		pc->pt_wt.z -= hw.z;
		CAM_INFO(pc, "6DoF: recentered (engine pose as is)");
	}

	if (!pc->pt_zeroed) {
		// The engine tracks in its saved-map frame, whose origin is tens of metres away (head positions ~95 m from
		// 0). Basalt starts at the origin; so remove that constant offset once, at the first pose, keeping the height.
		// The room map and the anchor expect poses near the origin (pibridge-roommap rejects an alignment > 20 m).
		pc->pt_zeroed = true;
		const struct xrt_vec3 hw = pt_rot(&pc->pt_wq, head_slam);
		pc->pt_wt.x = -hw.x;
		pc->pt_wt.z = -hw.z;
		CAM_INFO(pc, "6DoF: start near the origin (removed the engine's %.1f m saved-map offset)",
		         sqrt(hw.x * hw.x + hw.z * hw.z));
	}

	out->pose.position = pt_add(pt_rot(&pc->pt_wq, head_slam), pc->pt_wt);
	math_quat_rotate(&pc->pt_wq, &qp, &out->pose.orientation);
	math_quat_normalize(&out->pose.orientation);
	const struct xrt_vec3 wxr = {w_slam.y * hr.z - w_slam.z * hr.y, w_slam.z * hr.x - w_slam.x * hr.z,
	                             w_slam.x * hr.y - w_slam.y * hr.x};
	out->linear_velocity = pt_rot(&pc->pt_wq, pt_add(v, wxr));
	out->angular_velocity = pt_rot(&pc->pt_wq, w_slam);
	out->relation_flags = (enum xrt_space_relation_flags)XRT_SPACE_RELATION_BITMASK_ALL;

	// for a later loss: keep the orientation continuous and the position held
	math_quat_rotate(&out->pose.orientation, &q3inv, &pc->pt_fix);
	math_quat_normalize(&pc->pt_fix);
	pc->pt_held = out->pose.position;
	if (!pc->pt_tracking) {
		pc->pt_tracking = true;
		pc->pt_acquired++;
		CAM_INFO(pc, "6DoF: tracking (engine pose as is, acquisition %" PRIu64 ")", pc->pt_acquired);
		snprintf(pc->slam_status, sizeof(pc->slam_status),
		         "6DoF tracking; %" PRIu64 " acquired, %" PRIu64 " lost (engine pose as is)", pc->pt_acquired,
		         pc->pt_lost);
	}
	os_mutex_unlock(&pc->slam_lock);
	return true;
}

bool
pimax_camera_head_relation(struct pimax_camera *pc,
                           int64_t at_ns,
                           const struct xrt_space_relation *rel3dof,
                           struct xrt_space_relation *out)
{
	if (pc == NULL || !pc->slam_on) {
		return false;
	}
	if (pc->passthrough) {
		return passthrough_head(pc, at_ns, rel3dof, out);
	}
	// Rust (pimax_cam_track_head): IMU-rate prediction from the newest tracker pose with blended corrections while
	// tracking; 3DoF orientation (heading kept continuous) at the held position otherwise.
	pimax_cam_rel r3, ro;
	rel_to_c(rel3dof, &r3);
	os_mutex_lock(&pc->slam_lock);
	int64_t now = os_monotonic_get_ns();
	if (now - pc->last_poll_ns > U_TIME_1MS_IN_NS * 4) {
		slam_poll_locked(pc, now);
	}
	int st = pimax_cam_track_head(pc->track, (uint64_t)now, at_ns, &r3, &ro);
	os_mutex_unlock(&pc->slam_lock);
	if (st < 0) {
		return false;
	}
	*out = *rel3dof;
	rel_from_c(&ro, out);
	math_quat_normalize(&out->pose.orientation);
	return true;
}

static void
recenter_cb(void *ptr)
{
	struct pimax_camera *pc = (struct pimax_camera *)ptr;
	if (pc->passthrough) {
		os_mutex_lock(&pc->slam_lock);
		pc->pt_recenter = true; // applied at the next head query (it needs the head pose)
		os_mutex_unlock(&pc->slam_lock);
	} else if (pc->track != NULL) {
		os_mutex_lock(&pc->slam_lock);
		pimax_cam_track_recenter(pc->track);
		os_mutex_unlock(&pc->slam_lock);
	}
}

//! Push one IMU-timed tracking frame (tiles in tracker order) to the tracker and/or the recorder. @p exp_us =
//! exposure of every tile of the frame. One timestamp per frame set (Basalt asserts that all cameras of a set carry
//! the same one): the mean mid-exposure of the tracker cameras. A pair always has equal exposures, so with one pair
//! (0,1) this is that pair's own mid-exposure; with both pairs (PIMAX_SLAM_CAMS=0,1,2,3, the upper pair has its own
//! auto exposure) each pair is off by a quarter of the pairs' exposure difference instead of one pair by half of it.
static void
push_slam_frames(struct pimax_camera *pc, struct xrt_frame *tiles[PIMAX_CAM_TILES], int64_t ts,
                 const uint16_t exp_us[PIMAX_CAM_TILES])
{
	// Mid-exposure time: the frame stamp marks the exposure start (image rows start exposure/38.8 us later and
	// Pimax's factory delta 1.638 ms = half its typical 3.3 ms exposure; analysis/slam-exposure.md).
	uint64_t exp_sum = 0;
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		if (tiles[pc->slam_cams[i]] == NULL) {
			return; // all or nothing: the tracker wants one frame per camera per timestamp
		}
		exp_sum += exp_us[pc->slam_cams[i]];
	}
	// mean exposure (us) * 1000 / 2 = half of it in ns; slam_n >= 1 (setup_slam)
	int64_t adj = pc->slam_cam_offset_ns + (pc->exposure_mid ? (int64_t)(exp_sum * 500 / pc->slam_n) : 0);
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		// Shared frames: set the tracker timestamp on a private copy only when an offset is configured.
		struct xrt_frame *xf = tiles[pc->slam_cams[i]];
		struct xrt_frame *own = NULL;
		if (adj != 0) {
			u_frame_clone(xf, &own);
			own->timestamp = ts + adj;
			xf = own;
		}
		if (pc->slam_on) {
			xrt_sink_push_frame(pc->slam_sinks->cams[i], xf);
		}
		if (pc->record_on) {
			xrt_sink_push_frame(pc->rec_sinks->cams[i], xf);
		}
		xrt_frame_reference(&own, NULL);
	}
	pc->slam_frames++;
	if (pc->slam_on) {
		os_mutex_lock(&pc->slam_lock);
		pc->slam_imu_frame_seen = true;
		pc->slam_imu_since_frame = 0;
		slam_poll_locked(pc, os_monotonic_get_ns());
		os_mutex_unlock(&pc->slam_lock);
	}
}

static void
fill_slam_calibration(struct pimax_camera *pc, const pimax_cam_slam_imu *imu)
{
	struct t_slam_calibration *c = &pc->slam_calib;
	memset(c, 0, sizeof(*c));
	c->cam_count = (int)pc->slam_n;
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		const pimax_cam_kb4 *k = &pc->calib.cameras[pc->slam_cams[i]].intr;
		struct t_slam_camera_calibration *sc = &c->cams[i];
		struct t_camera_calibration *tc = &sc->base;
		tc->image_size_pixels.w = k->width;
		tc->image_size_pixels.h = k->height;
		t_camera_calibration_fill_intrinsics(tc, k->fx, k->fy, k->cx, k->cy);
		tc->distortion_model = T_DISTORTION_FISHEYE_KB4;
		tc->kb4.k1 = k->k[0];
		tc->kb4.k2 = k->k[1];
		tc->kb4.k3 = k->k[2];
		tc->kb4.k4 = k->k[3];
		double m[16];
		pimax_cam_slam_t_imu_cam(&pc->calib, pc->slam_cams[i], m); // checked in setup_slam
		for (int j = 0; j < 16; j++) {
			sc->T_imu_cam.v[j] = (float)m[j];
		}
		sc->frequency = 25.0;
	}
	struct t_imu_calibration *ic = &c->imu.base;
	for (int a = 0; a < 3; a++) {
		ic->accel.transform[a][a] = 1.0;
		ic->gyro.transform[a][a] = 1.0;
		ic->accel.noise_std[a] = imu->accel_noise_std[a];
		ic->accel.bias_std[a] = imu->accel_bias_std[a];
		ic->gyro.noise_std[a] = imu->gyro_noise_std[a];
		ic->gyro.bias_std[a] = imu->gyro_bias_std[a];
	}
	c->imu.frequency = imu->rate_hz;
}

static void
default_record_path(char *out, size_t len)
{
	const char *home = getenv("HOME");
	time_t now = time(NULL);
	struct tm tm;
	localtime_r(&now, &tm);
	char stamp[32];
	strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);
	snprintf(out, len, "%s/.local/share/omarchyvr/slam-recordings/%s", home != NULL ? home : ".", stamp);
}

/*!
 * The number of cameras in the Basalt calibration that $SLAM_CONFIG names (its cam-calib line; one "camera_type" per
 * camera), or -1 when there is no SLAM_CONFIG, no cam-calib line or the file cannot be read. Basalt aborts the whole
 * process when that count differs from the cameras it is fed (vit_tracker.cpp ASSERT_), so setup_slam checks it.
 */
static int
slam_config_cam_count(void)
{
	const char *toml = getenv("SLAM_CONFIG");
	if (toml == NULL || toml[0] == '\0') {
		return -1;
	}
	FILE *f = fopen(toml, "r");
	if (f == NULL) {
		return -1;
	}
	char line[1200];
	char calib[1100] = {0};
	while (fgets(line, sizeof(line), f) != NULL) {
		const char *l = line + strspn(line, " \t");
		if (strncmp(l, "cam-calib", 9) != 0) {
			continue;
		}
		const char *q = strchr(l, '"');
		const char *e = q != NULL ? strchr(q + 1, '"') : NULL;
		if (q != NULL && e != NULL && (size_t)(e - q - 1) < sizeof(calib)) {
			memcpy(calib, q + 1, (size_t)(e - q - 1));
		}
		break;
	}
	fclose(f);
	if (calib[0] == '\0') {
		return -1;
	}
	char path[1200];
	if (calib[0] == '/') {
		snprintf(path, sizeof(path), "%s", calib);
	} else {
		// relative: next to the toml (as Monado's t_slam resolves it from the working directory, the usual case is
		// an absolute path; PiBridge and the export write absolute ones)
		const char *slash = strrchr(toml, '/');
		int dl = slash != NULL ? (int)(slash - toml) : 0;
		snprintf(path, sizeof(path), "%.*s%s%s", dl, toml, dl > 0 ? "/" : "", calib);
	}
	f = fopen(path, "r");
	if (f == NULL) {
		return -1;
	}
	int n = 0;
	const char *key = "\"camera_type\"";
	size_t m = 0;
	int c;
	while ((c = fgetc(f)) != EOF) {
		m = (c == key[m]) ? m + 1 : (c == key[0] ? 1 : 0);
		if (key[m] == '\0') {
			n++;
			m = 0;
		}
	}
	fclose(f);
	return n;
}

//! PIMAX_SLAM / PIMAX_SLAM_RECORD. Returns false only on a configuration error.
static bool
setup_slam(struct pimax_camera *pc, bool want_slam, const char *record)
{
	int n = pimax_cam_slam_parse_cams(debug_get_option_pimax_slam_cams(), pc->calib.camera_count, pc->slam_cams,
	                                  4);
	if (n <= 0) {
		CAM_ERROR(pc, "PIMAX_SLAM_CAMS: %s", pimax_cam_last_error());
		return false;
	}
	pc->slam_n = (uint32_t)n;
	// Basalt aborts the process when the calibration of SLAM_CONFIG has another camera count than the tracker's
	// feed: fewer cameras in the file -> use the first of PIMAX_SLAM_CAMS only; more -> no tracker (3DoF, VR runs on)
	int file_cams = want_slam ? slam_config_cam_count() : -1;
	if (file_cams > 0 && (uint32_t)file_cams < pc->slam_n) {
		CAM_ERROR(pc, "PIMAX_SLAM_CAMS=%s names %u cameras but the SLAM_CONFIG calibration has %d: using the first %d",
		          debug_get_option_pimax_slam_cams(), pc->slam_n, file_cams, file_cams);
		pc->slam_n = (uint32_t)file_cams;
	} else if (file_cams > 0 && (uint32_t)file_cams > pc->slam_n) {
		CAM_ERROR(pc, "the SLAM_CONFIG calibration has %d cameras but PIMAX_SLAM_CAMS=%s names %u: no SLAM tracker "
		          "(3DoF)", file_cams, debug_get_option_pimax_slam_cams(), pc->slam_n);
		want_slam = false;
	}
	double m[16];
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		if (pimax_cam_slam_t_imu_cam(&pc->calib, pc->slam_cams[i], m) != 0) {
			CAM_ERROR(pc, "SLAM camera %u: %s", pc->slam_cams[i], pimax_cam_last_error());
			return false;
		}
	}
	pc->slam_cam_offset_ns = (int64_t)debug_get_num_option_pimax_slam_cam_offset_us() * 1000;
	pc->exposure_mid = debug_get_bool_option_pimax_slam_exposure_mid();

	pimax_cam_slam_imu imu;
	int ir = pimax_cam_slam_imu_load(debug_get_option_pimax_slam_imu(), &imu);
	if (ir < 0) {
		CAM_WARN(pc, "IMU noise file: %s (using defaults)", pimax_cam_last_error());
	}
	fill_slam_calibration(pc, &imu);
	if (pimax_cam_slam_default_head(&pc->calib, pc->head_in_imu) != 0) {
		pc->head_in_imu[0] = pc->head_in_imu[1] = pc->head_in_imu[2] = 0.0;
	}
	const char *ho = debug_get_option_pimax_ht_head_offset();
	if (ho != NULL) {
		double x, y, z;
		if (sscanf(ho, "%lf,%lf,%lf", &x, &y, &z) == 3) {
			pc->head_in_imu[0] = x;
			pc->head_in_imu[1] = y;
			pc->head_in_imu[2] = z;
		}
	}
	char names[160] = {0};
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		size_t l = strlen(names);
		snprintf(names + l, sizeof(names) - l, "%s%u (%s)", i ? ", " : "", pc->slam_cams[i],
		         pc->calib.cameras[pc->slam_cams[i]].name);
	}
	CAM_INFO(pc,
	         "SLAM sensors: cameras %s at 25 Hz, IMU 1 kHz, noise from %s, camera offset %" PRId64
	         " us, frame time at mid-exposure %s",
	         names, imu.source, pc->slam_cam_offset_ns / 1000, pc->exposure_mid ? "on" : "off");

	if (record != NULL && record[0] != '\0' && strcmp(record, "0") != 0) {
		if (strcmp(record, "1") == 0) {
			default_record_path(pc->rec_path, sizeof(pc->rec_path));
		} else {
			snprintf(pc->rec_path, sizeof(pc->rec_path), "%s", record);
		}
		char note[256];
		snprintf(note, sizeof(note),
		         "Monado pimax driver EuRoC recording; camera offset %" PRId64
		         " us applied; frame time = stamp + exposure/2: %s",
		         pc->slam_cam_offset_ns / 1000, pc->exposure_mid ? "yes" : "no");
		if (pimax_cam_slam_write_basalt(&pc->calib, pc->slam_cams, pc->slam_n, 25.0, 0, &imu, pc->rec_path,
		                                note) != 0) {
			CAM_ERROR(pc, "recording directory %s: %s", pc->rec_path, pimax_cam_last_error());
		} else {
			// The recorder appends _<datetime> to its prefix: <dir>/euroc_<datetime>/mav0/...
			char prefix[600];
			snprintf(prefix, sizeof(prefix), "%s/euroc", pc->rec_path);
			pc->rec_sinks = euroc_recorder_create(&pc->xfctx, prefix, (int)pc->slam_n, 1, true);
			pc->record_on = pc->rec_sinks != NULL;
			CAM_INFO(pc, "EuRoC recording %s: %s/euroc_<datetime> (calibration in %s/pimax-basalt-calib.json)",
			         pc->record_on ? "to" : "FAILED", pc->rec_path, pc->rec_path);
		}
	}

	if (!want_slam) {
		return true;
	}
#ifdef XRT_FEATURE_SLAM
	struct t_slam_tracker_config config = {0};
	t_slam_fill_default_config(&config);
	config.cam_count = (int)pc->slam_n;
	config.slam_calib = &pc->slam_calib;
	config.submit_from_start = true;
	struct xrt_slam_sinks *sinks = NULL;
	if (t_slam_create(&pc->xfctx, &config, &pc->slam, &sinks) != 0 || sinks == NULL) {
		CAM_ERROR(pc, "SLAM tracker could not be created (VIT library '%s' missing? set VIT_SYSTEM_LIBRARY_PATH); "
		          "staying 3DoF", config.vit_system_library_path);
		pc->slam = NULL;
		return true;
	}
	if (t_slam_start(pc->slam) != 0) {
		CAM_ERROR(pc, "SLAM tracker did not start; staying 3DoF");
		return true;
	}
	pc->slam_sinks = sinks;
	pc->track = pimax_cam_track_create(NULL, pc->head_in_imu);
	if (pc->track == NULL) {
		CAM_ERROR(pc, "6DoF supervisor: %s", pimax_cam_last_error());
		return true;
	}
	snprintf(pc->slam_status, sizeof(pc->slam_status), "%s", track_state_name(PIMAX_CAM_TRACK_INIT));
	pc->passthrough = debug_get_bool_option_pimax_slam_passthrough();
	pc->pt_wq = (struct xrt_quat)XRT_QUAT_IDENTITY;
	pc->pt_fix = (struct xrt_quat)XRT_QUAT_IDENTITY;
	pc->slam_on = true;
	g_head_in_imu[0] = pc->head_in_imu[0];
	g_head_in_imu[1] = pc->head_in_imu[1];
	g_head_in_imu[2] = pc->head_in_imu[2];
	if (debug_get_bool_option_pimax_ctrl_engine_leds()) {
		pc->leds_on = pimax_wb_load(&pc->wb);
		if (pc->leds_on) {
			pc->led_buf = calloc(PIMAX_CAM_TILES, (size_t)PIMAX_CAM_TILE_W * PIMAX_CAM_TILE_H);
			pc->leds_on = pc->led_buf != NULL;
		}
		if (pc->leds_on) {
			CAM_INFO(pc, "PIMAX_CTRL_ENGINE: controller LED frames go to Pimax's LED engine (winbridge)");
		} else {
			CAM_ERROR(pc, "PIMAX_CTRL_ENGINE: the VIT library (%s) has no winbridge controller extension; the LED "
			              "frames are not used",
			          config.vit_system_library_path);
		}
	}
	CAM_INFO(pc, "6DoF: SLAM tracker up, head origin (%.3f %.3f %.3f) m in the IMU frame; %s",
	         pc->head_in_imu[0], pc->head_in_imu[1], pc->head_in_imu[2],
	         pc->passthrough ? "the tracker's pose as it comes (PIMAX_SLAM_PASSTHROUGH)" : "3DoF until it is trusted");
#else
	CAM_ERROR(pc, "PIMAX_SLAM=1 but Monado was built without XRT_FEATURE_SLAM; staying 3DoF");
#endif
	return true;
}

static void *
capture_thread(void *ptr)
{
	struct pimax_camera *pc = (struct pimax_camera *)ptr;
	U_TRACE_SET_THREAD_NAME("Pimax camera");
	os_thread_name_self("pimax-camera");

	uint32_t consecutive_errors = 0;
	while (os_thread_helper_is_running(&pc->thread)) {
		pimax_cam_frame_info fi;
		int r = pimax_cam_next(pc->cam, 200, &fi);
		int64_t now = os_monotonic_get_ns();
		update_stats(pc, now);
		if (r == 0) {
			pc->timeouts++;
			continue;
		}
		if (r < 0) {
			pc->errors++;
			if (consecutive_errors++ % 50 == 0) {
				CAM_ERROR(pc, "camera: %s", pimax_cam_last_error());
			}
			os_nanosleep(U_TIME_1MS_IN_NS * 100);
			continue;
		}
		consecutive_errors = 0;
		pc->frames++;
		pc->win_frames++;
		if ((fi.flags & PIMAX_CAM_FLAG_NO_DEV_TIME) != 0 && !pc->jpeg_seen) {
			pc->jpeg_seen = true;
			CAM_INFO(pc, "camera: Motion-JPEG profile (a 480 Mbps USB link): tiles 0 and 1 only, no controller-LED "
			           "exposure, frames timed on the host clock");
			if (pc->ae != NULL) {
				// The controller steers on the exposure and gain reported by the metadata; this profile has neither.
				CAM_INFO(pc, "camera: auto-exposure off, the Motion-JPEG profile reports no exposure or gain");
				pimax_cam_ae_destroy(pc->ae);
				pc->ae = NULL;
			}
		}
		bool timed = false;
		uint64_t ts = frame_time(pc, &fi, &timed);
		pc->last_frame_ts = (int64_t)ts;

		// Debug GUI: every tile of every frame (tracking and LED exposure).
		for (uint32_t t = 0; t < PIMAX_CAM_TILES; t++) {
			if (u_sink_debug_is_active(&pc->debug[t])) {
				struct xrt_frame *xf = tile_frame(pc, t, ts, &fi);
				if (xf != NULL) {
					u_sink_debug_push_frame(&pc->debug[t], xf);
					xrt_frame_reference(&xf, NULL);
				}
			}
		}

		if (fi.kind == PIMAX_CAM_KIND_LED && pc->leds_on) {
			// PIMAX_CTRL_ENGINE: all four tiles of the LED exposure, as Pimax Play hands them (40 us)
			const uint8_t *tp[PIMAX_CAM_TILES];
			uint32_t strides[PIMAX_CAM_TILES];
			bool ok = true;
			for (uint32_t t = 0; t < PIMAX_CAM_TILES && ok; t++) {
				uint8_t *dst = pc->led_buf + (size_t)t * PIMAX_CAM_TILE_W * PIMAX_CAM_TILE_H;
				ok = pimax_cam_copy_tile(pc->cam, t, dst, PIMAX_CAM_TILE_W) == 0;
				tp[t] = dst;
				strides[t] = PIMAX_CAM_TILE_W;
			}
			if (ok && pc->wb.push_led((int64_t)ts, 40000, PIMAX_CAM_TILES, tp, strides) == 0) {
				pc->leds_pushed++;
			}
		}
		if (fi.kind != PIMAX_CAM_KIND_TRACKING) {
			continue;
		}
		pc->tracking_frames++;
		pc->win_tracking++;
		ae_step(pc, now);
		bool want_slam = pc->slam_on || pc->record_on;
		bool want_ht = pc->ht != NULL;
		if (!want_slam && !want_ht) {
			continue;
		}
		// One frame per needed tile, shared (read-only) by Mercury, the tracker and the recorder.
		struct xrt_frame *tiles[PIMAX_CAM_TILES] = {NULL};
		bool need[PIMAX_CAM_TILES] = {false};
		if (want_ht) {
			need[pc->ht_cams[0]] = need[pc->ht_cams[1]] = true;
		}
		if (want_slam && timed) {
			for (uint32_t i = 0; i < pc->slam_n; i++) {
				need[pc->slam_cams[i]] = true;
			}
		} else if (want_slam) {
			pc->slam_untimed++; // the tracker needs the IMU timeline (no replays, no IMU yet)
		}
		for (uint32_t t = 0; t < PIMAX_CAM_TILES; t++) {
			if (need[t]) {
				tiles[t] = tile_frame(pc, t, ts, &fi);
			}
		}
		if (want_ht && tiles[pc->ht_cams[0]] != NULL && tiles[pc->ht_cams[1]] != NULL) {
			// Never blocks: the hands thread takes the newest pair when it is free (and within its rate/duty).
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
			pimax_ht_push(pc->ht, tiles[pc->ht_cams[0]], tiles[pc->ht_cams[1]]);
#endif
			pc->pairs_pushed++;
		}
		if (want_slam && timed) {
			push_slam_frames(pc, tiles, (int64_t)ts, fi.exposure);
		}
		for (uint32_t t = 0; t < PIMAX_CAM_TILES; t++) {
			xrt_frame_reference(&tiles[t], NULL);
		}
	}
	return NULL;
}

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
static void
head_at(struct xrt_device *hmd, int64_t at_ns, struct xrt_space_relation *out)
{
	pimax_hmd_head_relation_at(hmd, at_ns, out);
}
#endif

/*!
 * PIMAX_HANDS=mercury: camera hands (pimax_ht.c). Any failure only disables the hands; the HMD keeps running.
 */
static bool
setup_mercury(struct pimax_camera *pc)
{
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (!parse_u32_pair(debug_get_option_pimax_ht_cams(), pc->ht_cams) ||
	    pc->ht_cams[0] >= pc->calib.camera_count || pc->ht_cams[1] >= pc->calib.camera_count ||
	    pc->ht_cams[0] == pc->ht_cams[1]) {
		CAM_ERROR(pc, "PIMAX_HT_CAMS=%s: want two different camera ids < %u", debug_get_option_pimax_ht_cams(),
		          pc->calib.camera_count);
		return false;
	}
	struct pimax_ht_config cfg = {0};
	cfg.cams[0] = pc->ht_cams[0];
	cfg.cams[1] = pc->ht_cams[1];
	cfg.orient[0] = cfg.orient[1] = -1;
	uint32_t orient_env[2];
	if (parse_u32_pair(debug_get_option_pimax_ht_orient(), orient_env)) {
		cfg.orient[0] = (int)orient_env[0];
		cfg.orient[1] = (int)orient_env[1];
	}
	// Same head origin as the 6DoF tracker (PIMAX_HT_HEAD_OFFSET overrides both), so hands and head agree.
	double head[3];
	const double *headp = NULL;
	const char *ho = debug_get_option_pimax_ht_head_offset();
	if (ho != NULL) {
		if (sscanf(ho, "%lf,%lf,%lf", &head[0], &head[1], &head[2]) == 3) {
			headp = head;
		} else {
			CAM_WARN(pc, "PIMAX_HT_HEAD_OFFSET=%s ignored (want x,y,z in metres)", ho);
		}
	}
	if (!pimax_ht_head_from_cam(&pc->calib, pc->ht_cams[0], headp, &cfg.head_from_cam)) {
		CAM_WARN(pc, "calibration has no camera-IMU extrinsic; hands placed with an identity camera pose");
	}
	const char *roles = debug_get_option_pimax_ht_roles();
	cfg.hand_roles = roles != NULL && strcmp(roles, "hands") == 0;
	if (roles != NULL && !cfg.hand_roles && strcmp(roles, "tracking") != 0) {
		CAM_WARN(pc, "PIMAX_HT_ROLES=%s unknown (tracking or hands); using tracking", roles);
	}
	long nice_v = debug_get_num_option_pimax_ht_nice();
	cfg.nice = nice_v < 0 ? 0 : (nice_v > 19 ? 19 : (int)nice_v);
	cfg.max_hz = debug_get_float_option_pimax_ht_hz();
	cfg.duty = debug_get_float_option_pimax_ht_duty();
	cfg.stale_ms = (int)debug_get_num_option_pimax_ht_stale_ms();
	cfg.predict_ms = debug_get_float_option_pimax_ht_predict_ms();
	cfg.stats = pc->stats_ht;
	cfg.log_level = pc->log_level;
	pc->ht = pimax_ht_create(pc->hmd, head_at, &pc->calib, &cfg);
	return pc->ht != NULL;
#else
	CAM_ERROR(pc, "PIMAX_HANDS=mercury but Monado was built without Mercury (needs ONNX Runtime + OpenCV)");
	(void)pc;
	return false;
#endif
}

struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user,
                    pimax_camera_orient_fn orient,
                    void *orient_user,
                    pimax_camera_send_fn send,
                    void *send_user)
{
	const char *hands = debug_get_option_pimax_hands();
	bool want_hands = hands != NULL && strcmp(hands, "mercury") == 0;
	if (hands != NULL && !want_hands && strcmp(hands, "0") != 0 && strcmp(hands, "off") != 0) {
		U_LOG_IFL_W(log_level, "PIMAX_HANDS=%s unknown (only \"mercury\")", hands);
	}
	bool want_slam = debug_get_bool_option_pimax_slam();
	const char *record = debug_get_option_pimax_slam_record();
	bool want_record = record != NULL && record[0] != '\0' && strcmp(record, "0") != 0;
	const char *ae = debug_get_option_pimax_cam_ae();
	enum ae_mode ae_mode = AE_OFF;
	if (ae != NULL && strcmp(ae, "dry") == 0) {
		ae_mode = AE_DRY;
	} else if (ae != NULL && strcmp(ae, "1") == 0) {
		ae_mode = AE_SEND;
	} else if (ae != NULL && ae[0] != '\0' && strcmp(ae, "0") != 0) {
		U_LOG_IFL_W(log_level, "PIMAX_CAM_AE=%s unknown (0, dry or 1); auto-exposure off", ae);
	}
	if (!want_hands && !want_slam && !want_record && ae_mode == AE_OFF && !debug_get_bool_option_pimax_cam()) {
		return NULL;
	}

	struct pimax_camera *pc = U_TYPED_CALLOC(struct pimax_camera);
	pc->log_level = log_level;
	pc->hmd = hmd;
	pc->clock = clock;
	pc->clock_user = clock_user;
	pc->orient = orient;
	pc->orient_user = orient_user;
	pc->send = send;
	pc->send_user = send_user;
	if (os_mutex_init(&pc->slam_lock) == 0) {
		pc->slam_lock_init = true;
	}
	pc->stats_ht = debug_get_bool_option_pimax_ht_stats();
	for (int t = 0; t < PIMAX_CAM_TILES; t++) {
		u_sink_debug_init(&pc->debug[t]);
	}
	os_thread_helper_init(&pc->thread);

	char used[512] = {0};
	if (pimax_cam_calib_load(debug_get_option_pimax_cam_calib(), &pc->calib, used, sizeof(used)) != 0) {
		CAM_ERROR(pc, "camera calibration: %s", pimax_cam_last_error());
		pimax_camera_destroy(&pc);
		return NULL;
	}
	CAM_INFO(pc, "camera calibration %s (%u cameras, %s)", used, pc->calib.camera_count, pc->calib.device_uid);

	const char *src = debug_get_option_pimax_cam_source();
	pimax_cam_open_params params = {0};
	if (src != NULL && src[0] != '\0' && strncmp(src, "/dev/", 5) != 0) {
		params.replay_flags = PIMAX_CAM_REPLAY_LOOP | PIMAX_CAM_REPLAY_REALTIME;
	}
	pc->cam = pimax_cam_open(src, &params);
	if (pc->cam == NULL) {
		CAM_ERROR(pc, "camera: %s", pimax_cam_last_error());
		pimax_camera_destroy(&pc);
		return NULL;
	}
	pimax_cam_describe(pc->cam, pc->source_desc, sizeof(pc->source_desc));
	CAM_INFO(pc, "camera source: %s (pimax-cam %s)", pc->source_desc, pimax_cam_version());

	const char *rec = debug_get_option_pimax_cam_record();
	if (rec != NULL && rec[0] != '\0') {
		if (pimax_cam_record_start(pc->cam, rec, 0) == 0) {
			CAM_INFO(pc, "recording raw camera frames to %s", rec);
		} else {
			CAM_WARN(pc, "recording: %s", pimax_cam_last_error());
		}
	}

	if (want_hands && !setup_mercury(pc)) {
		CAM_WARN(pc, "continuing without hand tracking");
	}
	pc->hands = pc->ht != NULL;

	if ((want_slam || want_record) && pc->slam_lock_init && !setup_slam(pc, want_slam, record)) {
		CAM_WARN(pc, "continuing without 6DoF/recording");
	}

	if (ae_mode != AE_OFF) {
		pc->ae = pimax_cam_ae_create();
		if (pc->ae == NULL) {
			CAM_ERROR(pc, "auto-exposure: %s", pimax_cam_last_error());
		} else if (ae_mode == AE_SEND && send == NULL) {
			CAM_WARN(pc, "PIMAX_CAM_AE=1 but no headset command channel (no device?): dry run only");
			pc->ae_mode = AE_DRY;
		} else {
			pc->ae_mode = ae_mode;
		}
		if (pc->ae != NULL) {
			snprintf(pc->ae_status, sizeof(pc->ae_status), "%s, waiting for tracking frames",
			         pc->ae_mode == AE_SEND ? "on" : "dry");
			CAM_WARN(pc, "auto-exposure %s: %s", pc->ae_mode == AE_SEND ? "ON" : "DRY RUN",
			         pc->ae_mode == AE_SEND ? "sending the exposure push (HID 0x51) to the headset"
			                                : "computing only, nothing is sent (PIMAX_CAM_AE=1 sends)");
		}
	}

	u_var_add_root(pc, "Pimax cameras", true);
	u_var_add_log_level(pc, &pc->log_level, "log_level");
	u_var_add_ro_text(pc, pc->source_desc, "source");
	u_var_add_ro_f32(pc, &pc->fps, "fps");
	u_var_add_ro_f32(pc, &pc->tracking_fps, "tracking fps");
	u_var_add_ro_u64(pc, &pc->frames, "frames");
	u_var_add_ro_u64(pc, &pc->pairs_pushed, "hand pairs offered");
	if (pc->slam_on || pc->record_on) {
		u_var_add_gui_header(pc, NULL, "6DoF / SLAM");
		u_var_add_ro_text(pc, pc->slam_status, "6DoF state");
		u_var_add_ro_text(pc, pc->rec_path, "EuRoC recording");
		u_var_add_ro_u64(pc, &pc->slam_frames, "SLAM frame sets");
		u_var_add_ro_u64(pc, &pc->imu_pushed, "SLAM IMU samples");
		pc->recenter_btn.cb = recenter_cb;
		pc->recenter_btn.ptr = pc;
		u_var_add_button(pc, &pc->recenter_btn, "Recenter 6DoF (head to origin)");
	}
	if (pc->ae != NULL) {
		u_var_add_gui_header(pc, NULL, "Auto-exposure");
		u_var_add_ro_text(pc, pc->ae_status, "AE");
	}
	for (int t = 0; t < PIMAX_CAM_TILES; t++) {
		char name[48];
		snprintf(name, sizeof(name), "tile %d (%s)", t, pc->calib.cameras[t].name);
		u_var_add_sink_debug(pc, &pc->debug[t], name);
	}

	if (os_thread_helper_start(&pc->thread, capture_thread, pc) != 0) {
		CAM_ERROR(pc, "could not start the capture thread");
		pimax_camera_destroy(&pc);
		return NULL;
	}
	pc->thread_started = true;
	CAM_INFO(pc, "camera pipeline up%s%s%s", pc->hands ? " with camera hands (Mercury)" : "",
	         pc->slam_on ? " with 6DoF SLAM" : "", pc->record_on ? " with EuRoC recording" : "");
	return pc;
}

void
pimax_camera_take_hand_devices(struct pimax_camera *pc, struct xrt_device *out[2])
{
	out[0] = out[1] = NULL;
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (pc != NULL) {
		pimax_ht_take_devices(pc->ht, out);
	}
#else
	(void)pc;
#endif
}

void
pimax_camera_destroy(struct pimax_camera **pc_ptr)
{
	struct pimax_camera *pc = *pc_ptr;
	if (pc == NULL) {
		return;
	}
	*pc_ptr = NULL;

	if (pc->thread_started) {
		os_thread_helper_stop_and_wait(&pc->thread);
	}
	os_thread_helper_destroy(&pc->thread);

	if (pc->cam != NULL) {
		int64_t n = pimax_cam_record_stop(pc->cam);
		if (n > 0) {
			CAM_INFO(pc, "recorded %" PRId64 " frames", n);
		}
	}

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	// Stops the hands thread before the HMD goes away (the devices then only report inactive hands).
	pimax_ht_destroy(&pc->ht);
#endif
	xrt_frame_context_destroy_nodes(&pc->xfctx);
	if (pc->ae != NULL) {
		CAM_INFO(pc, "AE %s", pc->ae_status);
		pimax_cam_ae_destroy(pc->ae);
		pc->ae = NULL;
	}
	pimax_cam_track_destroy(pc->track);
	pc->track = NULL;
	if (pc->led_buf != NULL) {
		CAM_INFO(pc, "PIMAX_CTRL_ENGINE: %" PRIu64 " LED frame sets handed to Pimax's LED engine", pc->leds_pushed);
		free(pc->led_buf);
		pc->led_buf = NULL;
	}
	if (pc->slam_lock_init) {
		os_mutex_destroy(&pc->slam_lock);
	}
	if (pc->record_on) {
		CAM_INFO(pc, "EuRoC recording finished: %s (%" PRIu64 " frame sets, %" PRIu64 " IMU samples)",
		         pc->rec_path, pc->slam_frames, pc->imu_pushed);
	}

	if (pc->cam != NULL) {
		pimax_cam_close(pc->cam);
		pc->cam = NULL;
	}
	u_var_remove_root(pc);
	for (int t = 0; t < PIMAX_CAM_TILES; t++) {
		u_sink_debug_destroy(&pc->debug[t]);
	}
	free(pc);
}

#endif // PIMAX_HAVE_CAM
