// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air HMD: 3DoF from the libpimax IMU stream.
 *
 * Based on the sample driver. Startup: the gyro is averaged for PIMAX_BIAS_SECONDS (default 2 s) while the
 * headset is still (window restarts on motion), the mean is used as gyro bias (the factory wBias is wrong,
 * see analysis/imu-orientation.md) and the initial tilt is seeded from the mean accelerometer.
 * If no still window comes within PIMAX_BIAS_TIMEOUT_MS (default 3000, e.g. started while worn), tracking starts
 * with the bias cached by the last good calibration (~/.cache/omarchyvr/gyro-bias, old ~/.cache/pimax, else zero) and the window keeps
 * running: every later window that is a REAL rest (pimax_bias.h: every axis as quiet as the bare sensor on a desk, and
 * the mean near the bias in use) refines the bias for the whole session and is cached; a worn head never qualifies
 * (2026-10-18: a worn window had been cached as the bias). With no bias at all (no cache), the first window that is
 * quiet under a looser rule is used as a provisional bias (never cached) until a real rest replaces it.
 *
 * @ingroup drv_pimax
 */

#include "os/os_time.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_relation_history.h"
#include "math/m_space.h"
#include "math/m_imu_3dof.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "math/m_matrix_2x2.h"
#include "math/m_mathinclude.h"

#include "util/u_recenter.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"

#include "pimax.h"
#include "pimax_interface.h"
#include "pimax_lens.h"
#include "pimax_ctrl.h"
#include "pimax_camera.h"
#include "pimax_eyeglue.h"
#include "pimax_anchor.h"
#include "pimax_bias.h"

#include "os/os_threading.h"

#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <unistd.h>

DEBUG_GET_ONCE_LOG_OPTION(pimax_log, "PIMAX_LOG", U_LOGGING_INFO)
//! TODO: the panels are mounted rotated by 90 deg (direction unknown). 0, 90, 180 or 270.
DEBUG_GET_ONCE_NUM_OPTION(pimax_panel_rotation, "PIMAX_PANEL_ROTATION", 0) // legacy: both eyes
DEBUG_GET_ONCE_NUM_OPTION(pimax_rot_left, "PIMAX_ROT_LEFT", 90)
DEBUG_GET_ONCE_NUM_OPTION(pimax_rot_right, "PIMAX_ROT_RIGHT", 270)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_mirror_left, "PIMAX_MIRROR_LEFT", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_mirror_right, "PIMAX_MIRROR_RIGHT", false)
//! 0: sensor axes == OpenXR axes (+X right, +Y up, -Z fwd). 1: 180 deg about Y (X and Z flipped).
//! Debug only: divide screen/view sizes so the windowed (XCB/Wayland) window fits a normal monitor.
DEBUG_GET_ONCE_NUM_OPTION(pimax_debug_div, "PIMAX_DEBUG_DIV", 1)
DEBUG_GET_ONCE_NUM_OPTION(pimax_axis_mode, "PIMAX_AXIS_MODE", 1)
//! Debug: do not open the headset (no USB access, no tracking): lens/compositor tests without hardware.
DEBUG_GET_ONCE_BOOL_OPTION(pimax_debug_ptrace, "PIMAX_DEBUG_PTRACE", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_no_device, "PIMAX_NO_DEVICE", false)
DEBUG_GET_ONCE_NUM_OPTION(pimax_bias_ms, "PIMAX_BIAS_MS", 2000)
DEBUG_GET_ONCE_NUM_OPTION(pimax_bias_timeout_ms, "PIMAX_BIAS_TIMEOUT_MS", 3000)

#define PIMAX_EYE_W 3552
#define PIMAX_EYE_H 3840
//! Refresh rate: 72 (default) or 90. Must match the mode Monado picks (XRT_COMPOSITOR_DESIRED_MODE).
DEBUG_GET_ONCE_NUM_OPTION(pimax_fps, "PIMAX_FPS", 72)
#define PIMAX_PUSH_DIV 4 // push every 4th IMU sample (250 Hz) into the relation history

struct pimax_hmd
{
	struct xrt_device base;
	enum u_logging_level log_level;

	pimax_t *lib;
	struct m_relation_history *relation_hist;
	struct m_imu_3dof fusion;

	int axis_mode;
	struct pimax_lens lens;

	struct pimax_ctrl_ctx *ctrl;
	struct xrt_device *ctrl_devs[2];
	bool ctrl_handed_out;

	// Device clock -> monotonic clock.
	bool have_clock;
	uint64_t dev_ts_ext_us; // unwrapped device time
	uint32_t last_dev_ts;
	int64_t clock_offset_ns;
	uint64_t last_ts_ns;

	// Gyro bias: the startup window, then refined on every real rest (pimax_bias.h).
	bool bias_done;
	uint32_t bias_target;
	struct pimax_bias_rule bias_rule;
	struct pimax_bias_state bias;
	uint32_t total_n;   // samples since start, for the timeout
	uint32_t timeout_n; // 1 kHz IMU: ms == samples
	uint32_t bias_refines; // windows accepted after the start
	uint32_t bias_rejects; // completed windows that were not a real rest
	uint64_t bias_log_ns;  // last rejection / refinement log line
	uint64_t bias_save_ns; // last cache write
	float bias_saved[3];   // the value last written to the cache
	bool bias_have_saved;

	uint32_t count;
	float bias_dps[3];

	//! IMU clock snapshot for the cameras (written by the IMU callback).
	struct os_mutex clk_lock;
	struct pimax_camera_clock_ref clk;
	//! Tracking cameras + Mercury + 6DoF (NULL unless PIMAX_HANDS / PIMAX_CAM / PIMAX_SLAM / PIMAX_SLAM_RECORD).
	//! The IMU thread reads it under cam_lock (it is set after the session thread is running).
	struct pimax_camera *camera;
	//! Eye gaze (XR_EXT_eye_gaze_interaction); NULL unless PIMAX_EYE=1.
	struct pimax_eyeglue *eye;
	struct os_mutex cam_lock;

	//! Lighthouse (pimax_hmd_set_head_source): the head pose comes from this device (the Lighthouse faceplate's HMD of
	//! steamvr_lh), placed by head_source_offset; NULL = the headset's own 3DoF / 6DoF.
	struct xrt_device *head_source;
	struct xrt_pose head_source_offset;

	//! HID link state from libpimax's on_state (session thread writes, any thread reads).
	atomic_bool hid_connected;
	//! Set after the first CONNECTED: later ones are reconnects.
	bool hid_ever_connected;
	uint32_t hid_losses;
	int64_t hid_lost_at_ns;
};

static inline struct pimax_hmd *
pimax_hmd(struct xrt_device *xdev)
{
	return (struct pimax_hmd *)xdev;
}

#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_WARN(hmd, ...) U_LOG_XDEV_IFL_W(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

static void
map_axes(const struct pimax_hmd *hmd, const float in[3], struct xrt_vec3 *out)
{
	if (hmd->axis_mode == 1) {
		out->x = -in[0];
		out->y = in[1];
		out->z = -in[2];
	} else {
		out->x = in[0];
		out->y = in[1];
		out->z = in[2];
	}
}

static uint64_t
sample_time_ns(struct pimax_hmd *hmd, uint32_t dev_ts_us)
{
	uint64_t now = os_monotonic_get_ns();
	if (!hmd->have_clock) {
		hmd->have_clock = true;
		hmd->dev_ts_ext_us = dev_ts_us;
		hmd->last_dev_ts = dev_ts_us;
		hmd->clock_offset_ns = (int64_t)now - (int64_t)(hmd->dev_ts_ext_us * 1000);
	} else {
		hmd->dev_ts_ext_us += (uint32_t)(dev_ts_us - hmd->last_dev_ts); // handles 2^32 wrap
		hmd->last_dev_ts = dev_ts_us;
	}
	int64_t cand = (int64_t)now - (int64_t)(hmd->dev_ts_ext_us * 1000);
	// Track the minimum (least delayed) offset; relax slowly to follow clock drift.
	if (cand < hmd->clock_offset_ns) {
		hmd->clock_offset_ns = cand;
	} else {
		hmd->clock_offset_ns += (cand - hmd->clock_offset_ns) >> 14;
	}
	uint64_t t = (uint64_t)((int64_t)(hmd->dev_ts_ext_us * 1000) + hmd->clock_offset_ns);
	if (t <= hmd->last_ts_ns) {
		t = hmd->last_ts_ns + 1;
	}
	hmd->last_ts_ns = t;

	os_mutex_lock(&hmd->clk_lock);
	hmd->clk.dev_ts_u32 = dev_ts_us;
	hmd->clk.dev_ext_us = hmd->dev_ts_ext_us;
	hmd->clk.offset_ns = hmd->clock_offset_ns;
	hmd->clk.valid = 1;
	os_mutex_unlock(&hmd->clk_lock);
	return t;
}

//! 6DoF supervisor: the 3DoF orientation history.
static bool
pimax_hmd_orient_at(void *user, int64_t ts_ns, struct xrt_quat *out)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	if (!hmd->bias_done) {
		return false;
	}
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	m_relation_history_get(hmd->relation_hist, ts_ns, &rel);
	if ((rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return false;
	}
	*out = rel.pose.orientation;
	math_quat_normalize(out);
	return true;
}

//! Camera thread: host command to the headset (exposure push, PIMAX_CAM_AE=1 only).
static int
pimax_hmd_send_cmd(void *user, uint8_t cmd, uint8_t opt, const uint8_t *data, size_t len)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	if (hmd->lib == NULL) {
		return -1;
	}
	return pimax_send(hmd->lib, cmd, opt, data, len) < 0 ? -1 : 0;
}

//! Camera thread: the IMU device-clock -> monotonic mapping, so frames share the IMU timeline.
static bool
pimax_hmd_clock_ref(void *user, struct pimax_camera_clock_ref *out)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	os_mutex_lock(&hmd->clk_lock);
	*out = hmd->clk;
	os_mutex_unlock(&hmd->clk_lock);
	return out->valid != 0;
}

void
pimax_hmd_take_hand_devices(struct xrt_device *xdev, struct xrt_device *out[2])
{
	pimax_camera_take_hand_devices(pimax_hmd(xdev)->camera, out);
}

static void
bias_restart(struct pimax_hmd *hmd)
{
	pimax_bias_restart(&hmd->bias);
}

//! The gyro bias cache: <cache>/omarchyvr/gyro-bias (the app's cache dir); `legacy` gives the old
//! <cache>/pimax/gyro-bias, still read when the new file does not exist yet.
static void
bias_cache_path_dir(char *out, size_t n, const char *dir)
{
	const char *xdg = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	if (xdg != NULL && xdg[0] != '\0') {
		snprintf(out, n, "%s/%s/gyro-bias", xdg, dir);
	} else {
		snprintf(out, n, "%s/.cache/%s/gyro-bias", home != NULL ? home : "/tmp", dir);
	}
}

static void
bias_cache_path(char *out, size_t n)
{
	bias_cache_path_dir(out, n, "omarchyvr");
}

static bool
bias_cache_load(float g[3])
{
	char path[512];
	bias_cache_path(path, sizeof(path));
	FILE *f = fopen(path, "r");
	if (f == NULL) {
		bias_cache_path_dir(path, sizeof(path), "pimax");
		f = fopen(path, "r");
	}
	if (f == NULL) {
		return false;
	}
	bool ok = fscanf(f, "%f %f %f", &g[0], &g[1], &g[2]) == 3;
	fclose(f);
	// A sane bias is ~1 dps; anything large is a bad file.
	for (int i = 0; ok && i < 3; i++) {
		ok = isfinite(g[i]) && fabsf(g[i]) < 5.0f;
	}
	return ok;
}

static void
bias_cache_save(const float g[3])
{
	char path[512];
	bias_cache_path(path, sizeof(path));
	char *slash = strrchr(path, '/');
	if (slash != NULL) {
		*slash = '\0';
		mkdir(path, 0755); // parent ~/.cache exists on any desktop
		*slash = '/';
	}
	FILE *f = fopen(path, "w");
	if (f != NULL) {
		fprintf(f, "%.4f %.4f %.4f\n", g[0], g[1], g[2]);
		fclose(f);
	}
}

static void
bias_set(struct pimax_hmd *hmd, const float g[3])
{
	struct xrt_vec3 gb;
	map_axes(hmd, g, &gb);
	hmd->fusion.gyro_bias.value = m_vec3_mul_scalar(gb, (float)(M_PI / 180.0));
	for (int i = 0; i < 3; i++) {
		hmd->bias_dps[i] = g[i];
	}
}

static void
seed_tilt(struct pimax_hmd *hmd, const float a[3])
{
	// Specific force points up, rotate it onto +Y.
	struct xrt_vec3 ac, up = {0, 1, 0};
	map_axes(hmd, a, &ac);
	struct xrt_vec3 ac_n = m_vec3_normalize(ac);
	math_quat_from_vec_a_to_vec_b(&ac_n, &up, &hmd->fusion.rot);
	math_quat_normalize(&hmd->fusion.rot);
}

//! The cache gets a new value only when it moved (> 0.01 deg/s on an axis) or 5 minutes passed: a headset resting on
//! the desk completes a window every 2 s.
static void
bias_maybe_save(struct pimax_hmd *hmd, const float g[3], uint64_t now)
{
	float dmax = 0;
	for (int i = 0; i < 3; i++) {
		dmax = fmaxf(dmax, fabsf(g[i] - hmd->bias_saved[i]));
	}
	if (hmd->bias_have_saved && dmax <= 0.01f && now - hmd->bias_save_ns < 300ULL * 1000000000ULL) {
		return;
	}
	bias_cache_save(g);
	memcpy(hmd->bias_saved, g, sizeof(hmd->bias_saved));
	hmd->bias_have_saved = true;
	hmd->bias_save_ns = now;
}

//! A real rest completed: the first time it also seeds the tilt and starts tracking; later it refines the bias.
static void
bias_finish(struct pimax_hmd *hmd, const float g[3], const float a[3])
{
	uint64_t now = os_monotonic_get_ns();
	float dmax = 0;
	for (int i = 0; i < 3; i++) {
		dmax = fmaxf(dmax, fabsf(g[i] - hmd->bias_dps[i]));
	}
	bias_set(hmd, g);
	bias_maybe_save(hmd, g, now);

	if (hmd->bias_done) {
		hmd->bias_refines++;
		if (dmax > 0.02f || now - hmd->bias_log_ns > 60ULL * 1000000000ULL) {
			hmd->bias_log_ns = now;
			HMD_INFO(hmd,
			         "gyro bias refined at rest: %.3f %.3f %.3f dps (moved %.3f; %u rests this session, %u windows "
			         "rejected as not a real rest)",
			         g[0], g[1], g[2], dmax, hmd->bias_refines, hmd->bias_rejects);
		}
		return;
	}
	seed_tilt(hmd, a);
	hmd->bias_done = true;
	HMD_INFO(hmd, "gyro bias done: %.3f %.3f %.3f dps over %u samples (%u restarts); accel %.3f %.3f %.3f g",
	         g[0], g[1], g[2], hmd->bias_target, hmd->bias.restarts, a[0], a[1], a[2]);
}

//! No still window in time (headset worn at start): track now with the cached bias.
static void
bias_timeout(struct pimax_hmd *hmd, const float accel_g[3])
{
	float g[3] = {0, 0, 0};
	bool cached = bias_cache_load(g);
	bias_set(hmd, g);
	seed_tilt(hmd, accel_g);
	hmd->bias_done = true;
	bias_restart(hmd);
	HMD_WARN(hmd,
	         "headset not still for %u ms (%u restarts): tracking with the %s gyro bias %.3f %.3f %.3f dps; "
	         "it is refined every time the headset rests for %u ms%s",
	         hmd->timeout_n, hmd->bias.restarts, cached ? "cached" : "zero (no cache)", g[0], g[1], g[2],
	         hmd->bias_target, cached ? "" : " (put the headset still on the desk for 2 s to calibrate the gyro)");
}

//! No bias yet (no cache) and a window quiet enough for a provisional one: use it (never cached); a real rest
//! replaces it. The first time it also seeds the tilt and starts tracking.
static void
bias_provisional(struct pimax_hmd *hmd, const float g[3], const float a[3])
{
	bias_set(hmd, g);
	const struct pimax_bias_state *b = &hmd->bias;
	HMD_WARN(hmd,
	         "gyro bias provisional: %.3f %.3f %.3f dps (gyro sd %.3f %.3f %.3f dps: not a real rest, not cached); "
	         "put the headset still on the desk for 2 s to calibrate the gyro",
	         g[0], g[1], g[2], b->last_gsd[0], b->last_gsd[1], b->last_gsd[2]);
	if (!hmd->bias_done) {
		seed_tilt(hmd, a);
		hmd->bias_done = true;
	}
}

//! Feed one sample into the rest window. Returns true when a window was accepted (bias_finish was called).
static bool
bias_feed(struct pimax_hmd *hmd, const pimax_imu_sample *s)
{
	float g[3], a[3];
	enum pimax_bias_result r = pimax_bias_feed(&hmd->bias, &hmd->bias_rule, s->gyro_dps, s->accel_g, g, a);
	if (r == PIMAX_BIAS_ACCEPT) {
		bias_finish(hmd, g, a);
		return true;
	}
	if (r == PIMAX_BIAS_PROVISIONAL) {
		bias_provisional(hmd, g, a);
		return true;
	}
	if (r == PIMAX_BIAS_NOT_REST || r == PIMAX_BIAS_FAR || r == PIMAX_BIAS_DRIFTING) {
		hmd->bias_rejects++;
		uint64_t now = os_monotonic_get_ns();
		if (r != PIMAX_BIAS_DRIFTING && now - hmd->bias_log_ns > 60ULL * 1000000000ULL) {
			hmd->bias_log_ns = now;
			const struct pimax_bias_state *b = &hmd->bias;
			HMD_INFO(hmd,
			         "gyro bias window not used (%s): mean %.3f %.3f %.3f dps, gyro sd %.3f %.3f %.3f dps, accel sd "
			         "%.4f %.4f %.4f m/s^2 (a rest needs < %.2f dps and < %.3f m/s^2); bias stays %.3f %.3f %.3f",
			         pimax_bias_result_str(r), b->last_mean[0], b->last_mean[1], b->last_mean[2], b->last_gsd[0],
			         b->last_gsd[1], b->last_gsd[2], b->last_asd[0] * 9.80665, b->last_asd[1] * 9.80665,
			         b->last_asd[2] * 9.80665, hmd->bias_rule.gyro_sd_max_dps, hmd->bias_rule.accel_sd_max_g * 9.80665,
			         hmd->bias_dps[0], hmd->bias_dps[1], hmd->bias_dps[2]);
		}
	}
	return false;
}

//! Runs on the libpimax session thread.
static void
pimax_on_controller(void *user, const pimax_controller_report *r)
{
	pimax_ctrl_on_report(((struct pimax_hmd *)user)->ctrl, r);
}

static void
pimax_on_controller_status(void *user, const pimax_controller_status *s)
{
	pimax_ctrl_on_status(((struct pimax_hmd *)user)->ctrl, s);
}

/*!
 * `$XDG_RUNTIME_DIR/pimax-hmd.state` (one line, replaced atomically on every HID link change), for pimaxd and the
 * GUI: `state=connected|disconnected losses=N since_ns=<CLOCK_MONOTONIC> pid=<monado pid>`.
 */
static void
hid_state_file_write(struct pimax_hmd *hmd, bool connected, int64_t now_ns)
{
	const char *xdg = getenv("XDG_RUNTIME_DIR");
	if (xdg == NULL || xdg[0] == '\0') {
		return;
	}
	char path[512], tmp[540];
	snprintf(path, sizeof(path), "%s/pimax-hmd.state", xdg);
	snprintf(tmp, sizeof(tmp), "%s.%d.tmp", path, (int)getpid());
	FILE *f = fopen(tmp, "w");
	if (f == NULL) {
		return;
	}
	fprintf(f, "state=%s losses=%u since_ns=%lld pid=%d\n", connected ? "connected" : "disconnected",
	        hmd->hid_losses, (long long)now_ns, (int)getpid());
	if (fclose(f) != 0 || rename(tmp, path) != 0) {
		unlink(tmp);
	}
}

/*!
 * Runs on the libpimax session thread (the same thread as pimax_on_imu, so the clock and fusion fields need no lock).
 *
 * libpimax reconnects the HID by itself after a re-enumeration and re-sends its wake; the keepalive resumes with the
 * rate set by pimax_video_prepare. Nothing new is sent here. On loss the pose is held (not extrapolated, flagged not
 * tracked) instead of freezing silently; on return the device clock mapping restarts (the MCU may have rebooted)
 * and the fusion skips the gap instead of integrating the gyro over it.
 */
/*!
 * The headset's volume buttons arrive as firmware log lines on IF3 ("<info> app: vol up 1" pressed, "... 0"
 * released; "vol down"). While Monado owns the headset, pibridge-keepalived (paused) cannot read them, so the state is
 * handed to it in $XDG_RUNTIME_DIR/pibridge/volume-button ("<seq> <dir +1|-1> <pressed 1|0>", written atomically);
 * keepalived changes the headset volume, with the repeat while held. Runs on the libpimax session thread.
 */
static void
pimax_on_log(void *user, const char *line)
{
	(void)user;
	int dir = 0;
	int pressed = 0;
	if (strstr(line, "app: vol up 1") != NULL) {
		dir = 1;
		pressed = 1;
	} else if (strstr(line, "app: vol up 0") != NULL) {
		dir = 1;
	} else if (strstr(line, "app: vol down 1") != NULL) {
		dir = -1;
		pressed = 1;
	} else if (strstr(line, "app: vol down 0") != NULL) {
		dir = -1;
	} else {
		return;
	}
	static unsigned seq = 0;
	seq++;
	const char *rt = getenv("XDG_RUNTIME_DIR");
	char dir_path[512];
	snprintf(dir_path, sizeof(dir_path), "%s/pibridge", rt != NULL && rt[0] != '\0' ? rt : "/tmp");
	(void)mkdir(dir_path, 0700);
	char path[600];
	char tmp[620];
	snprintf(path, sizeof(path), "%s/volume-button", dir_path);
	snprintf(tmp, sizeof(tmp), "%s.tmp", path);
	FILE *f = fopen(tmp, "w");
	if (f == NULL) {
		return;
	}
	fprintf(f, "%u %d %d\n", seq, dir, pressed);
	fclose(f);
	(void)rename(tmp, path);
}

static void
pimax_on_state(void *user, pimax_state state)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	pimax_ctrl_on_state(hmd->ctrl, state);

	int64_t now = os_monotonic_get_ns();
	bool connected = state == PIMAX_CONNECTED;
	bool was = atomic_load(&hmd->hid_connected);
	if (connected == was && (connected || hmd->hid_ever_connected)) {
		return;
	}
	if (connected) {
		if (hmd->hid_ever_connected) {
			HMD_WARN(hmd, "headset HID back after %.1f s (loss #%u): pose tracking resumes",
			         (double)(now - hmd->hid_lost_at_ns) / 1e9, hmd->hid_losses);
			hmd->have_clock = false;
			hmd->fusion.state = M_IMU_3DOF_STATE_START;
		}
		hmd->hid_ever_connected = true;
	} else {
		hmd->hid_losses++;
		hmd->hid_lost_at_ns = now;
		HMD_ERROR(hmd,
		          "headset HID lost (loss #%u): holding the last head pose (not tracked) until it returns; "
		          "libpimax reconnects by itself",
		          hmd->hid_losses);
	}
	atomic_store(&hmd->hid_connected, connected);
	hid_state_file_write(hmd, connected, now);
}

static void
pimax_on_imu(void *user, const pimax_imu_sample *s)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	uint64_t ts_ns = sample_time_ns(hmd, s->ts_us);

	float a[3] = {s->accel_g[0] * 9.80665f, s->accel_g[1] * 9.80665f, s->accel_g[2] * 9.80665f};
	float g[3] = {s->gyro_dps[0] * (float)(M_PI / 180.0), s->gyro_dps[1] * (float)(M_PI / 180.0),
	              s->gyro_dps[2] * (float)(M_PI / 180.0)};
	struct xrt_vec3 accel, gyro;
	map_axes(hmd, a, &accel);
	map_axes(hmd, g, &gyro);

	// 6DoF tracker / EuRoC recorder: every sample, raw (the tracker estimates its own biases).
	os_mutex_lock(&hmd->cam_lock);
	if (pimax_camera_wants_imu(hmd->camera)) {
		pimax_camera_push_imu(hmd->camera, (int64_t)ts_ns, &accel, &gyro);
	}
	os_mutex_unlock(&hmd->cam_lock);

	if (!hmd->bias_done) {
		if (!bias_feed(hmd, s) && ++hmd->total_n >= hmd->timeout_n) {
			bias_timeout(hmd, s->accel_g);
		}
		return;
	}
	// Every later real rest refines the bias (not only the first one, and never a worn window).
	bias_feed(hmd, s);

	m_imu_3dof_update(&hmd->fusion, ts_ns, &accel, &gyro);

	if ((++hmd->count % PIMAX_PUSH_DIV) != 0) {
		return;
	}

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                     XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	rel.pose.orientation = hmd->fusion.rot;
	// Angular velocity in world space (bias corrected) so the history can predict.
	struct xrt_vec3 gb = m_vec3_sub(gyro, hmd->fusion.gyro_bias.value);
	math_quat_rotate_vec3(&hmd->fusion.rot, &gb, &rel.angular_velocity);
	m_relation_history_push(hmd->relation_hist, &rel, ts_ns);
}

static void
pimax_hmd_destroy(struct xrt_device *xdev)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);

	// Eye gaze: stops the Rust session (unsubscribe, release the EyeChip) before anything else goes away.
	pimax_eyeglue_destroy(&hmd->eye);

	// Cameras first: their thread reads the IMU clock snapshot and pushes into Mercury. Unpublish the pointer
	// under cam_lock so an IMU callback in flight finishes before the pipeline goes away.
	os_mutex_lock(&hmd->cam_lock);
	struct pimax_camera *camera = hmd->camera;
	hmd->camera = NULL;
	os_mutex_unlock(&hmd->cam_lock);
	pimax_camera_destroy(&camera);

	// Stop the controller attach thread, then the session thread (and with it the callbacks) before freeing anything.
	if (hmd->ctrl != NULL) {
		pimax_ctrl_stop(hmd->ctrl);
		if (!hmd->ctrl_handed_out) {
			// Creation failed after the controllers were made: nobody else will destroy them.
			xrt_device_destroy(&hmd->ctrl_devs[0]);
			xrt_device_destroy(&hmd->ctrl_devs[1]);
		}
	}
	if (hmd->lib != NULL) {
		pimax_close(hmd->lib);
		hmd->lib = NULL;
	}

	u_var_remove_root(hmd);
	pimax_lens_destroy(&hmd->lens);
	m_imu_3dof_close(&hmd->fusion);
	m_relation_history_destroy(&hmd->relation_hist);
	os_mutex_destroy(&hmd->clk_lock);
	os_mutex_destroy(&hmd->cam_lock);
	u_device_free(&hmd->base);
}

static xrt_result_t
pimax_hmd_update_inputs(struct xrt_device *xdev)
{
	return XRT_SUCCESS;
}

/*
 * Recenter (pimax_hmd_request_recenter): one more world transform after the room anchor, applied to every head
 * relation, so the controllers (placed relative to the head) follow. Yaw and the horizontal position only: the
 * floor height and gravity stay. Logged as "PIMAX_RECENTER". Then u_recenter_announce(): the space overseer moves
 * the LOCAL spaces (each app's own too) onto the head, so seated apps face where you look as well.
 */
static pthread_mutex_t g_recenter_lock = PTHREAD_MUTEX_INITIALIZER;
static struct xrt_pose g_recenter = XRT_POSE_IDENTITY;
static atomic_bool g_recenter_request;

void
pimax_hmd_request_recenter(void)
{
	atomic_store(&g_recenter_request, true);
}

static void
pimax_hmd_apply_recenter(bool compositor_query, struct xrt_space_relation *rel)
{
	const bool valid = (rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
	// a new room offset (the room was recognised or set up) replaces any earlier recenter: it would shift the room
	static uint32_t seen_generation;
	pthread_mutex_lock(&g_pimax_anchor.lock);
	const uint32_t gen = g_pimax_anchor.generation;
	pthread_mutex_unlock(&g_pimax_anchor.lock);
	pthread_mutex_lock(&g_recenter_lock);
	if (gen != seen_generation) {
		seen_generation = gen;
		const struct xrt_pose id = XRT_POSE_IDENTITY;
		if (memcmp(&g_recenter, &id, sizeof(id)) != 0) {
			U_LOG_W("PIMAX_RECENTER: cleared (a new room offset was placed)");
		}
		g_recenter = id;
	}
	struct xrt_pose c = g_recenter;
	pthread_mutex_unlock(&g_recenter_lock);
	struct xrt_pose w = rel->pose;
	if (valid) {
		math_pose_transform(&c, &rel->pose, &w);
	}
	if (compositor_query && valid && atomic_exchange(&g_recenter_request, false)) {
		// the head now: its forward (-Z) turned onto -Z, its position moved over the origin
		const struct xrt_vec3 minus_z = {0, 0, -1};
		struct xrt_vec3 fwd;
		math_quat_rotate_vec3(&w.orientation, &minus_z, &fwd);
		const float yaw = atan2f(-fwd.x, -fwd.z);
		struct xrt_pose t = XRT_POSE_IDENTITY;
		t.orientation = (struct xrt_quat){0, sinf(-yaw / 2), 0, cosf(-yaw / 2)};
		struct xrt_vec3 back = {-w.position.x, 0, -w.position.z};
		math_quat_rotate_vec3(&t.orientation, &back, &t.position);
		struct xrt_pose nc;
		math_pose_transform(&t, &c, &nc);
		math_quat_normalize(&nc.orientation);
		pthread_mutex_lock(&g_recenter_lock);
		g_recenter = nc;
		pthread_mutex_unlock(&g_recenter_lock);
		// the LOCAL spaces (seated apps; each app's own was set where the head was when it started) follow
		u_recenter_announce();
		U_LOG_W("PIMAX_RECENTER: turned %.1f deg, moved %.3f m (floor kept)", yaw * 180.0f / (float)M_PI,
		        sqrtf(w.position.x * w.position.x + w.position.z * w.position.z));
		c = nc;
		math_pose_transform(&c, &rel->pose, &w);
	}
	if (!valid) {
		return;
	}
	if ((rel->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) {
		rel->pose = w;
	} else {
		rel->pose.orientation = w.orientation;
	}
	if (rel->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) {
		const struct xrt_vec3 v = rel->linear_velocity;
		math_quat_rotate_vec3(&c.orientation, &v, &rel->linear_velocity);
	}
	if (rel->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) {
		const struct xrt_vec3 v = rel->angular_velocity;
		math_quat_rotate_vec3(&c.orientation, &v, &rel->angular_velocity);
	}
}

//! Head relation at @p at_timestamp_ns (3DoF/6DoF, HID loss, room anchor). @p record_anchor: false for past-time
//! queries from the camera hands, so the anchor's pose ring only sees the compositor's queries.
static void
pimax_hmd_head_relation(struct pimax_hmd *hmd,
                        int64_t at_timestamp_ns,
                        bool record_anchor,
                        struct xrt_space_relation *out_relation)
{
	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	if (hmd->head_source != NULL) {
		// Lighthouse: the faceplate's tracked pose (already the head pose of the headset in the Lighthouse driver's
		// own config, in PiBridge's stage: steamvr_lh LH_STAGE_FILE), then the fixed offset. The room anchor only
		// records it into the head ring (PIMAX_ANCHOR=1; the room setup reads the head there) and applies no
		// offset (nobody writes one in this mode); the manual recenter still applies.
		struct xrt_space_relation src;
		if (xrt_device_get_tracked_pose(hmd->head_source, XRT_INPUT_GENERIC_HEAD_POSE, at_timestamp_ns, &src) !=
		    XRT_SUCCESS) {
			src = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		}
		struct xrt_relation_chain chain = {0};
		m_relation_chain_push_pose_if_not_identity(&chain, &hmd->head_source_offset);
		m_relation_chain_push_relation(&chain, &src);
		m_relation_chain_resolve(&chain, &relation);
		pimax_anchor_process_ex(at_timestamp_ns, &relation, record_anchor);
		pimax_hmd_apply_recenter(record_anchor, &relation);
		*out_relation = relation;
		return;
	}
	const bool hid_lost = hmd->lib != NULL && hmd->hid_ever_connected && !atomic_load(&hmd->hid_connected);
	if (hid_lost) {
		// No IMU: hold the newest pose instead of extrapolating its angular velocity into a spin.
		int64_t latest_ns = 0;
		if (m_relation_history_get_latest(hmd->relation_hist, &latest_ns, &relation)) {
			relation.angular_velocity = (struct xrt_vec3){0, 0, 0};
			relation.linear_velocity = (struct xrt_vec3){0, 0, 0};
		}
	} else {
		m_relation_history_get(hmd->relation_hist, at_timestamp_ns, &relation);
	}

	if ((relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		math_quat_normalize(&relation.pose.orientation);
	}

	// 6DoF (PIMAX_SLAM=1): the tracker's pose while it is trusted, else 3DoF at the held position.
	struct xrt_space_relation rel6;
	if (pimax_camera_head_relation(hmd->camera, at_timestamp_ns, &relation, &rel6)) {
		relation = rel6;
	}

	if (hid_lost) {
		// Valid but not tracked: clients can tell a held pose from a live one.
		relation.relation_flags = (enum xrt_space_relation_flags)(
		    relation.relation_flags &
		    ~(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
		      XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT));
	}

	// Room anchor (PIMAX_ANCHOR=1): record the raw SLAM pose, then move the world by the offset from `pimax-anchor apply`.
	pimax_anchor_process_ex(at_timestamp_ns, &relation, record_anchor);
	pimax_hmd_apply_recenter(record_anchor, &relation);
	*out_relation = relation;
}

void
pimax_hmd_set_head_source(struct xrt_device *xdev, struct xrt_device *source, const struct xrt_pose *offset)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);
	hmd->head_source = source;
	hmd->head_source_offset = offset != NULL ? *offset : (struct xrt_pose)XRT_POSE_IDENTITY;
	if (source != NULL) {
		// one space with the Lighthouse controllers and trackers
		hmd->base.tracking_origin = source->tracking_origin;
		HMD_INFO(hmd, "head pose from %s (%s), offset p(%.3f %.3f %.3f) q(%.4f %.4f %.4f %.4f)", source->str,
		         source->serial, hmd->head_source_offset.position.x, hmd->head_source_offset.position.y,
		         hmd->head_source_offset.position.z, hmd->head_source_offset.orientation.x,
		         hmd->head_source_offset.orientation.y, hmd->head_source_offset.orientation.z,
		         hmd->head_source_offset.orientation.w);
	}
}

void
pimax_hmd_head_relation_at(struct xrt_device *xdev, int64_t at_timestamp_ns, struct xrt_space_relation *out_relation)
{
	pimax_hmd_head_relation(pimax_hmd(xdev), at_timestamp_ns, false, out_relation);
}

static xrt_result_t
pimax_hmd_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE && !(name == XRT_INPUT_GENERIC_EYE_GAZE_POSE && hmd->eye != NULL)) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_space_relation relation;
	pimax_hmd_head_relation(hmd, at_timestamp_ns, true, &relation);

	if (name == XRT_INPUT_GENERIC_EYE_GAZE_POSE) {
		// Gaze ray in head space (Rust session), composed with the head pose at the requested time.
		struct xrt_pose gaze;
		if (!pimax_eyeglue_get_gaze(hmd->eye, at_timestamp_ns, &gaze)) {
			*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
			return XRT_SUCCESS;
		}
		struct xrt_relation_chain chain = {0};
		m_relation_chain_push_pose(&chain, &gaze);
		m_relation_chain_push_relation(&chain, &relation);
		m_relation_chain_resolve(&chain, out_relation);
		return XRT_SUCCESS;
	}

	*out_relation = relation;
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_begin_feature(struct xrt_device *xdev, enum xrt_device_feature_type type)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);
	if (type != XRT_DEVICE_FEATURE_EYE_TRACKING || hmd->eye == NULL) {
		return XRT_ERROR_FEATURE_NOT_SUPPORTED;
	}
	pimax_eyeglue_begin(hmd->eye);
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_end_feature(struct xrt_device *xdev, enum xrt_device_feature_type type)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);
	if (type != XRT_DEVICE_FEATURE_EYE_TRACKING || hmd->eye == NULL) {
		return XRT_ERROR_FEATURE_NOT_SUPPORTED;
	}
	pimax_eyeglue_end(hmd->eye);
	return XRT_SUCCESS;
}

static struct xrt_binding_input_pair eye_gaze_inputs_pimax[] = {
    {XRT_INPUT_GENERIC_EYE_GAZE_POSE, XRT_INPUT_GENERIC_EYE_GAZE_POSE},
};

static struct xrt_binding_profile pimax_hmd_binding_profiles[] = {
    {
        .name = XRT_DEVICE_EYE_GAZE_INTERACTION,
        .inputs = eye_gaze_inputs_pimax,
        .input_count = ARRAY_SIZE(eye_gaze_inputs_pimax),
    },
};

static xrt_result_t
pimax_hmd_get_view_poses(struct xrt_device *xdev,
                         const struct xrt_vec3 *default_eye_relation,
                         int64_t at_timestamp_ns,
                         enum xrt_view_type view_type,
                         uint32_t view_count,
                         struct xrt_space_relation *out_head_relation,
                         struct xrt_fov *out_fovs,
                         struct xrt_pose *out_poses)
{
	xrt_result_t xret = u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_type, view_count,
	                                            out_head_relation, out_fovs, out_poses);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// Display cant (lens.conf cant_deg): the panels point outward, so the left view is yawed to the left (+Y rotation
	// is counter-clockwise seen from above, -Z is forward), the right view to the right. Positions stay as they are.
	const struct pimax_hmd *hmd = (const struct pimax_hmd *)xdev;
	const float cant_rad = DEG_TO_RAD(pimax_lens_get_cant_deg(&hmd->lens));
	if (cant_rad != 0.0f && view_type == XRT_VIEW_TYPE_STEREO) {
		const struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
		for (uint32_t i = 0; i < view_count && i < 2; i++) {
			struct xrt_quat yaw;
			math_quat_from_angle_vector(i == 0 ? cant_rad : -cant_rad, &up, &yaw);
			// view orientation = existing orientation, then the extra yaw in the view's own frame
			struct xrt_quat q;
			math_quat_rotate(&out_poses[i].orientation, &yaw, &q);
			math_quat_normalize(&q);
			out_poses[i].orientation = q;
		}
	}
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_get_visibility_mask(struct xrt_device *xdev,
                              enum xrt_visibility_mask_type type,
                              uint32_t view_index,
                              struct xrt_visibility_mask **out_mask)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);
	// A configured edge black-out / mask (lens.conf) gets its own mesh, otherwise the generic FOV mask.
	struct xrt_visibility_mask *custom = pimax_lens_visibility_mask(&hmd->lens, type, view_index);
	if (custom != NULL) {
		*out_mask = custom;
		return XRT_SUCCESS;
	}
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)xdev;
	return pimax_lens_compute(&hmd->lens, view, u, v, result);
}

struct xrt_device *
pimax_hmd_create(const char *name, struct xrt_device *out_ctrls[2])
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	// PIMAX_EYE=1 adds the eye gaze pose as a second input of the HMD (like the PSVR2 driver).
	const bool want_eye = pimax_eyeglue_enabled();
	struct pimax_hmd *hmd = U_DEVICE_ALLOCATE(struct pimax_hmd, flags, want_eye ? 2 : 1, 0);
	if (hmd == NULL) {
		return NULL;
	}

	hmd->log_level = debug_get_log_option_pimax_log();
	if (debug_get_bool_option_pimax_debug_ptrace()) {
		// yama ptrace_scope=1: lets a same-user gdb/eu-stack attach to a hung service
		prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
		HMD_INFO(hmd, "PIMAX_DEBUG_PTRACE=1: same-user debuggers may attach to this process");
	}
	os_mutex_init(&hmd->clk_lock);
	os_mutex_init(&hmd->cam_lock);
	hmd->axis_mode = (int)debug_get_num_option_pimax_axis_mode();
	long bias_ms = debug_get_num_option_pimax_bias_ms();
	hmd->bias_target = (uint32_t)(bias_ms < 200 ? 200 : bias_ms); // 1 kHz IMU: ms == samples
	long timeout_ms = debug_get_num_option_pimax_bias_timeout_ms();
	hmd->timeout_n = (uint32_t)(timeout_ms < 200 ? 200 : timeout_ms);
	pimax_bias_rule_default(&hmd->bias_rule, hmd->bias_target);
	{
		// Windows are compared with the cached bias from the start (a far one needs a second that agrees).
		float cached[3];
		if (bias_cache_load(cached)) {
			pimax_bias_set_prev(&hmd->bias, cached);
		}
	}

	hmd->base.update_inputs = pimax_hmd_update_inputs;
	hmd->base.get_tracked_pose = pimax_hmd_get_tracked_pose;
	hmd->base.get_view_poses = pimax_hmd_get_view_poses;
	hmd->base.get_visibility_mask = pimax_hmd_get_visibility_mask;
	hmd->base.destroy = pimax_hmd_destroy;

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "%s", name);
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%s", name);

	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = false;
	if (want_eye) {
		hmd->eye = pimax_eyeglue_create(hmd->log_level);
	}
	if (hmd->eye == NULL) {
		hmd->base.input_count = 1; // PIMAX_EYE off, or the Rust library does not match this driver
	} else {
		hmd->base.inputs[1].name = XRT_INPUT_GENERIC_EYE_GAZE_POSE;
		hmd->base.supported.eye_gaze = true;
		hmd->base.begin_feature = pimax_hmd_begin_feature;
		hmd->base.end_feature = pimax_hmd_end_feature;
		hmd->base.binding_profiles = pimax_hmd_binding_profiles;
		hmd->base.binding_profile_count = ARRAY_SIZE(pimax_hmd_binding_profiles);
		HMD_INFO(hmd, "PIMAX_EYE=1: eye gaze pose exposed (XR_EXT_eye_gaze_interaction)");
	}

	hmd->base.hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = 1;

	// Display: one 7104x3840 @ 72 Hz output, left half = left eye, right half = right eye.
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / (float)debug_get_num_option_pimax_fps());
	long div = debug_get_num_option_pimax_debug_div();
	if (div < 1) {
		div = 1;
	}
	const uint32_t eye_w = PIMAX_EYE_W / div, eye_h = PIMAX_EYE_H / div;
	hmd->base.hmd->screens[0].w_pixels = eye_w * 2;
	hmd->base.hmd->screens[0].h_pixels = eye_h;

	// Lens model + FOV from ~/.config/pibridge/lens.conf (or PIMAX_LENS_CONF), see analysis/lens-distortion.md.
	pimax_lens_load(&hmd->lens, (float)PIMAX_EYE_H / (float)PIMAX_EYE_W);
	hmd->base.hmd->distortion.fov[0] = pimax_lens_get_fov(&hmd->lens, 0);
	hmd->base.hmd->distortion.fov[1] = pimax_lens_get_fov(&hmd->lens, 1);

	/*
	 * Panels are mounted rotated by 90 deg and mirror-symmetric: the left panel shows its half of the frame
	 * rotated 90 deg counter-clockwise, the right panel 90 deg clockwise. The compositor pre-rotates each eye the
	 * opposite way (u_device_rotation_*: "right" = 90 cw, "left" = 90 ccw as applied to the image).
	 * PIMAX_ROT_LEFT / PIMAX_ROT_RIGHT select 0/90/180/270 (degrees of pre-rotation, clockwise), optional
	 * PIMAX_MIRROR_LEFT / PIMAX_MIRROR_RIGHT flip the pre-rotated image horizontally.
	 * With 90/270 the user-facing display is eye_h x eye_w (3840x3552) while the framebuffer viewport stays
	 * eye_w x eye_h (3552x3840).
	 */
	long rots[2] = {debug_get_num_option_pimax_rot_left(), debug_get_num_option_pimax_rot_right()};
	long legacy = debug_get_num_option_pimax_panel_rotation();
	if (legacy != 0) {
		rots[0] = rots[1] = legacy;
	}
	bool mirror[2] = {debug_get_bool_option_pimax_mirror_left(), debug_get_bool_option_pimax_mirror_right()};
	for (uint8_t eye = 0; eye < 2; ++eye) {
		const struct xrt_matrix_2x2 *rotm = &u_device_rotation_ident;
		long r = ((rots[eye] % 360) + 360) % 360;
		switch (r) {
		case 90: rotm = &u_device_rotation_right; break;
		case 180: rotm = &u_device_rotation_180; break;
		case 270: rotm = &u_device_rotation_left; break;
		default: r = 0; break;
		}
		bool swap = (r == 90 || r == 270);
		hmd->base.hmd->views[eye].display.w_pixels = swap ? eye_h : eye_w;
		hmd->base.hmd->views[eye].display.h_pixels = swap ? eye_w : eye_h;
		hmd->base.hmd->views[eye].viewport.x_pixels = eye * eye_w;
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = eye_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = eye_h;
		struct xrt_matrix_2x2 m = *rotm;
		if (mirror[eye]) {
			const struct xrt_matrix_2x2 flip = {{.vecs = {{-1, 0}, {0, 1}}}};
			m_mat2x2_multiply(&m, &flip, &m);
		}
		hmd->base.hmd->views[eye].rot = m;
		HMD_INFO(hmd, "eye %u: pre-rotate %ld deg%s, display %ux%u, viewport %ux%u", eye, r,
		         mirror[eye] ? " + mirror" : "", hmd->base.hmd->views[eye].display.w_pixels,
		         hmd->base.hmd->views[eye].display.h_pixels, eye_w, eye_h);
	}

	// Distortion mesh from our own lens model (u, v arrive already in the pre-rotated, user-facing display space).
	hmd->base.compute_distortion = pimax_hmd_compute_distortion;
	hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	u_distortion_mesh_fill_in_compute(&hmd->base);

	m_relation_history_create(&hmd->relation_hist);
	m_imu_3dof_init(&hmd->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Initial identity so queries before the bias window finishes have something sane (not tracked).
	struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	identity.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	m_relation_history_push(hmd->relation_hist, &identity, os_monotonic_get_ns());

	if (debug_get_bool_option_pimax_no_device()) {
		HMD_WARN(hmd, "PIMAX_NO_DEVICE: headset not opened, identity pose");
		u_var_add_root(hmd, "Pimax Dream Air (no device)", true);
		// Test aid for the controller hand tracking: with PIMAX_CTRL_HANDS_DEMO=1 the controllers exist even without
		// a headset (no attach thread, no reports; the scripted demo input drives the hands).
		if (pimax_ctrl_demo_enabled()) {
			hmd->ctrl = pimax_ctrl_create(&hmd->base, hmd->log_level, hmd->ctrl_devs);
			if (hmd->ctrl != NULL) {
				out_ctrls[0] = hmd->ctrl_devs[0];
				out_ctrls[1] = hmd->ctrl_devs[1];
				hmd->ctrl_handed_out = true;
			}
		}
		// Cameras still work (live UVC or a PIMAX_CAM_SOURCE replay), timed without the IMU clock.
		hmd->camera = pimax_camera_create(&hmd->base, hmd->log_level, NULL, NULL, NULL, NULL, NULL, NULL);
		return &hmd->base;
	}

	bias_restart(hmd);

	hmd->ctrl = pimax_ctrl_create(&hmd->base, hmd->log_level, hmd->ctrl_devs);
	if (hmd->ctrl == NULL) {
		HMD_WARN(hmd, "controllers not created");
	}

	pimax_callbacks cb = {.user = hmd,
	                      .on_imu = pimax_on_imu,
	                      .on_state = pimax_on_state,
	                      .on_controller = pimax_on_controller,
	                      .on_controller_status = pimax_on_controller_status,
	                      .on_log = pimax_on_log};
	hmd->lib = pimax_open(&cb);
	if (hmd->lib == NULL) {
		HMD_ERROR(hmd, "pimax_open failed");
		pimax_hmd_destroy(&hmd->base);
		return NULL;
	}
	if (pimax_wait_connected(hmd->lib, 5000) != 0) {
		HMD_ERROR(hmd, "Pimax headset did not connect within 5 s");
		pimax_hmd_destroy(&hmd->base);
		return NULL;
	}

	// Firmware only lights the panels after the video-prepare sequence and ~5.5 s of stable 72 Hz video.
	// Also tell the keepalive service (pimax-keepalived) which rate to carry.
	long fps = debug_get_num_option_pimax_fps();
	int prep = pimax_video_prepare(hmd->lib, (uint8_t)fps);
	HMD_INFO(hmd, "pimax_video_prepare(%ld) -> %d", fps, prep);
	const char *xdg = getenv("XDG_RUNTIME_DIR");
	if (xdg != NULL) {
		char path[512];
		snprintf(path, sizeof(path), "%s/pimax-rate", xdg);
		FILE *f = fopen(path, "w");
		if (f != NULL) {
			fprintf(f, "%ld\n", fps);
			fclose(f);
			HMD_INFO(hmd, "wrote %ld to %s", fps, path);
		} else {
			HMD_WARN(hmd, "could not write %s", path);
		}
	}

	if (hmd->ctrl != NULL) {
		pimax_ctrl_start(hmd->ctrl, hmd->lib);
		out_ctrls[0] = hmd->ctrl_devs[0];
		out_ctrls[1] = hmd->ctrl_devs[1];
		hmd->ctrl_handed_out = true;
	}

	u_var_add_root(hmd, name, true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	u_var_add_bool(hmd, &hmd->bias_done, "gyro bias done");
	u_var_add_f32(hmd, &hmd->bias_dps[0], "bias x dps");
	u_var_add_f32(hmd, &hmd->bias_dps[1], "bias y dps");
	u_var_add_f32(hmd, &hmd->bias_dps[2], "bias z dps");

	// Tracking cameras and Mercury hands (PIMAX_HANDS=mercury or PIMAX_CAM=1); NULL otherwise.
	struct pimax_camera *camera =
	    pimax_camera_create(&hmd->base, hmd->log_level, pimax_hmd_clock_ref, hmd, pimax_hmd_orient_at, hmd,
	                        pimax_hmd_send_cmd, hmd);
	os_mutex_lock(&hmd->cam_lock);
	hmd->camera = camera;
	os_mutex_unlock(&hmd->cam_lock);

	HMD_INFO(hmd, "%s up: keep the headset still for %u ms for gyro calibration (axis mode %d)", name,
	         hmd->bias_target, hmd->axis_mode);
	return &hmd->base;
}
