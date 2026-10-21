// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PIMAX_CTRL_ENGINE: Pimax's own controller LED engine (LedObjectPoseEstimator.dll of Pimax Play, under Wine
 *         behind PiBridge's winbridge.exe) reached through the extension of the VIT library Monado loaded for the head
 *         (VIT_SYSTEM_LIBRARY_PATH = libpibridge_vit_winbridge.so; pimax_winbridge_ext.h). Header only.
 *
 * The camera thread hands it every controller LED frame set (pimax_camera.c), the controllers every IMU report and
 * read the engine's poses back (pimax_ctrl.c), as Pimax Play's service feeds the engine.
 * @ingroup drv_pimax
 */
#pragma once

#include "pimax_winbridge_ext.h"

#include <dlfcn.h>
#include <stdbool.h>
#include <stdlib.h>

struct pimax_wb
{
	pibridge_wb_push_led_fn push_led;
	pibridge_wb_push_ctrl_imu_fn push_ctrl_imu;
	pibridge_wb_ctrl_pose_fn ctrl_pose;
};

/*!
 * Fill @p wb from the VIT library. dlopen of the same file returns the handle Monado already holds (its globals, so
 * the running tracker). false when VIT_SYSTEM_LIBRARY_PATH is not PiBridge's winbridge library of this extension version.
 */
static inline bool
pimax_wb_load(struct pimax_wb *wb)
{
	const char *path = getenv("VIT_SYSTEM_LIBRARY_PATH");
	if (path == NULL || path[0] == '\0') {
		return false;
	}
	void *h = dlopen(path, RTLD_NOW | RTLD_LOCAL); // kept open for the process lifetime
	if (h == NULL) {
		return false;
	}
	pibridge_wb_ext_version_fn ver = (pibridge_wb_ext_version_fn)dlsym(h, "pibridge_wb_ext_version");
	wb->push_led = (pibridge_wb_push_led_fn)dlsym(h, "pibridge_wb_push_led");
	wb->push_ctrl_imu = (pibridge_wb_push_ctrl_imu_fn)dlsym(h, "pibridge_wb_push_ctrl_imu");
	wb->ctrl_pose = (pibridge_wb_ctrl_pose_fn)dlsym(h, "pibridge_wb_ctrl_pose");
	if (ver == NULL || ver() != PIBRIDGE_WB_EXT_VERSION || wb->push_led == NULL || wb->push_ctrl_imu == NULL ||
	    wb->ctrl_pose == NULL) {
		dlclose(h);
		return false;
	}
	return true;
}

/*!
 * Head origin in the head IMU frame (m), as the camera's SLAM setup uses it (pimax_camera.c; 0 before it ran): the
 * controllers put the engine's head-relative pose under the HMD's head pose with it.
 */
void
pimax_camera_head_in_imu(double out[3]);
