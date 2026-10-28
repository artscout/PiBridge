// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax factory camera calibration -> Mercury inputs (stereo calibration, view orientation, head offset).
 *         Shared by the driver (pimax_camera.c / pimax_ht.c) and the offline bench (pimax_ht_bench.c).
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "tracking/t_tracking.h"
#include "tracking/t_hand_tracking.h"

#include "pimax_cam.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Stereo calibration for Mercury: view 0 = camera @p a, view 1 = camera @p b, KB4 intrinsics on the full 640x480
 * tile (the camera sends all 480 rows; each tile has its own start row, handled by pimax-cam), extrinsic
 * X_b = R * X_a + t (OpenCV axes). NULL on error (pimax_cam_last_error()). @p out_baseline_m may be NULL.
 */
struct t_stereo_camera_calibration *
pimax_ht_make_stereo(const pimax_cam_calib *cal, uint32_t a, uint32_t b, double *out_baseline_m);

/*!
 * Mercury view orientation from the calibration: image "down" pointing up in the body frame = mounted upside down
 * = CAMERA_ORIENTATION_180. @p out_rolled (may be NULL) is set when the camera is rolled ~90 deg (not handled).
 */
enum t_camera_orientation
pimax_ht_orientation_of(const pimax_cam_calib *cal, uint32_t cam, bool *out_rolled);

/*!
 * Pose of Mercury's output frame (camera @p cam with OpenXR axes: -Z forward, +Y up) in the head frame. The head
 * origin is @p head_in_imu (metres, IMU/body frame); NULL = the SLAM tracker's default head origin, so hands and
 * the 6DoF head pose use the same point. False when the calibration has no camera-IMU extrinsic.
 */
bool
pimax_ht_head_from_cam(const pimax_cam_calib *cal, uint32_t cam, const double *head_in_imu, struct xrt_pose *out);

/*!
 * Find Mercury's model folder (same search as Monado: $XDG_DATA_HOME, ~/.local/share, /usr/local/share, /usr/share
 * + /monado/hand-tracking-models) and check that both greyscale models are there and non-empty. False with a
 * reason in @p why when they are missing (the caller then runs without hands).
 */
bool
pimax_ht_models_check(char *out_dir, size_t dir_len, char *why, size_t why_len);

#ifdef __cplusplus
}
#endif
