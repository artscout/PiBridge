// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax factory camera calibration -> Mercury inputs. See pimax_ht_calib.h.
 * @ingroup drv_pimax
 */

#include "pimax_ht_calib.h"

#include "math/m_api.h"
#include "util/u_file.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

struct t_stereo_camera_calibration *
pimax_ht_make_stereo(const pimax_cam_calib *cal, uint32_t a, uint32_t b, double *out_baseline_m)
{
	if (a >= cal->camera_count || b >= cal->camera_count || a == b) {
		return NULL;
	}
	double r[9], t[3];
	if (pimax_cam_calib_stereo(cal, a, b, r, t) != 0) {
		return NULL;
	}
	struct t_stereo_camera_calibration *sc = NULL;
	t_stereo_camera_calibration_alloc(&sc, T_DISTORTION_FISHEYE_KB4);
	const uint32_t ids[2] = {a, b};
	for (int v = 0; v < 2; v++) {
		const pimax_cam_kb4 *k = &cal->cameras[ids[v]].intr;
		struct t_camera_calibration *tc = &sc->view[v];
		// The factory calibration is for the full 640x480 sensor image, and the tiles are 640x480 too (all 480
		// rows arrive; the "466 rows" of the early Windows capture was a pcap artefact, analysis/overnight/cam.md).
		tc->image_size_pixels.w = k->width;
		tc->image_size_pixels.h = k->height;
		t_camera_calibration_fill_intrinsics(tc, k->fx, k->fy, k->cx, k->cy);
		tc->distortion_model = T_DISTORTION_FISHEYE_KB4;
		tc->kb4.k1 = k->k[0];
		tc->kb4.k2 = k->k[1];
		tc->kb4.k3 = k->k[2];
		tc->kb4.k4 = k->k[3];
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			sc->camera_rotation[i][j] = r[i * 3 + j];
		}
		sc->camera_translation[i] = t[i];
	}
	if (out_baseline_m != NULL) {
		*out_baseline_m = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
	}
	return sc;
}

enum t_camera_orientation
pimax_ht_orientation_of(const pimax_cam_calib *cal, uint32_t cam, bool *out_rolled)
{
	if (out_rolled != NULL) {
		*out_rolled = false;
	}
	double r[9], t[3];
	if (pimax_cam_calib_body_from_cam(cal, cam, r, t) != 0) {
		return CAMERA_ORIENTATION_0;
	}
	double down_y = r[4]; // column 1 (image +y) in the body frame, y component
	if (down_y > 0.5) {
		return CAMERA_ORIENTATION_180;
	}
	if (down_y < -0.5) {
		return CAMERA_ORIENTATION_0;
	}
	if (out_rolled != NULL) {
		*out_rolled = true;
	}
	return CAMERA_ORIENTATION_0;
}

static void
mat9_to_quat(const double r[9], struct xrt_quat *out)
{
	struct xrt_matrix_3x3 m;
	for (int i = 0; i < 9; i++) {
		m.v[i] = (float)r[i];
	}
	math_quat_from_matrix_3x3(&m, out);
	math_quat_normalize(out);
}

bool
pimax_ht_head_from_cam(const pimax_cam_calib *cal, uint32_t cam, const double *head_in_imu, struct xrt_pose *out)
{
	double r[9], t[3];
	if (pimax_cam_calib_body_from_cam(cal, cam, r, t) != 0) {
		*out = (struct xrt_pose)XRT_POSE_IDENTITY;
		return false;
	}
	double head[3] = {0, 0, 0};
	if (head_in_imu != NULL) {
		memcpy(head, head_in_imu, sizeof(head));
	} else if (pimax_cam_slam_default_head(cal, head) != 0) {
		head[0] = head[1] = head[2] = 0.0;
	}
	// head_from_camCV: rotation body_from_cam, position relative to the head origin (body axes = head axes).
	struct xrt_pose head_from_camcv;
	mat9_to_quat(r, &head_from_camcv.orientation);
	head_from_camcv.position =
	    (struct xrt_vec3){(float)(t[0] - head[0]), (float)(t[1] - head[1]), (float)(t[2] - head[2])};
	// Mercury answers with OpenXR camera axes (-Z forward, +Y up) = OpenCV axes rotated 180 deg about X.
	const struct xrt_pose rx180 = {.orientation = {.x = 1.0f, .y = 0.0f, .z = 0.0f, .w = 0.0f},
	                               .position = {0, 0, 0}};
	math_pose_transform(&head_from_camcv, &rx180, out);
	return true;
}

static bool
file_ok(const char *dir, const char *name, char *why, size_t why_len)
{
	char p[1200];
	snprintf(p, sizeof(p), "%s/%s", dir, name);
	struct stat st;
	if (stat(p, &st) != 0 || !S_ISREG(st.st_mode)) {
		snprintf(why, why_len, "%s missing", p);
		return false;
	}
	// A git-lfs pointer (not pulled) is ~130 bytes; the real models are megabytes.
	if (st.st_size < 100000) {
		snprintf(why, why_len, "%s is only %lld bytes (git-lfs pointer? run git lfs pull)", p,
		         (long long)st.st_size);
		return false;
	}
	return true;
}

bool
pimax_ht_models_check(char *out_dir, size_t dir_len, char *why, size_t why_len)
{
	if (u_file_get_hand_tracking_models_dir(out_dir, dir_len) < 0) {
		snprintf(why, why_len, "no monado/hand-tracking-models directory (scripts/get-ht-models.sh)");
		return false;
	}
	return file_ok(out_dir, "grayscale_detection_160x160.onnx", why, why_len) &&
	       file_ok(out_dir, "grayscale_keypoint_jan18.onnx", why, why_len);
}
