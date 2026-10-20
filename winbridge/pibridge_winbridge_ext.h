// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: MIT
/*
 * Extension of libpibridge_vit_winbridge.so for the Monado Pimax driver: Pimax's controller LED engine
 * (LedObjectPoseEstimator.dll under Wine, behind winbridge.exe) next to the head SLAM that VIT carries.
 *
 * The driver finds these functions in the VIT library Monado already loaded:
 *   void *h = dlopen(getenv("VIT_SYSTEM_LIBRARY_PATH"), RTLD_NOW | RTLD_NOLOAD);  dlsym(h, "pibridge_wb_...")
 * and checks pibridge_wb_ext_version() == PIBRIDGE_WB_EXT_VERSION. All calls return at once (queued to the library's
 * worker thread); with no running tracker or no winbridge they drop the data.
 *
 * Copies: pibridge winbridge/pibridge_winbridge_ext.h (this file) and the PiBridge Monado tree's
 * src/xrt/drivers/pimax/pimax_winbridge_ext.h. Keep them identical.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIBRIDGE_WB_EXT_VERSION 1

//! One controller pose from the engine: the controller body (frame of its LED model, controller_params.json)
//! in the head SLAM engine's world (z up, the frame of the VIT head poses), at the controller IMU sample time.
struct pibridge_wb_ctrl_pose
{
	int64_t ts_ns;
	float q[4]; //!< x y z w
	float p[3];
	float ang_vel[3];
	float lin_vel[3];
	//! The head IMU pose (same world) the engine placed the controller with: the controller relative to the head is
	//! inverse(head) * pose, which the driver composes with its own head pose (so recenter and room anchor apply).
	float head_q[4];
	float head_p[3];
	//! Rotation of the controller IMU into the body frame (ImuToDevice of controller_params.json; constant).
	float dev_from_imu[4];
};

typedef int (*pibridge_wb_ext_version_fn)(void);
//! A set of controller LED frames (the short-exposure frames between the tracking frames): cameras 0..ncam-1,
//! 640x480 Y8 each, own row stride. 0 = queued, -1 = no tracker running.
typedef int (*pibridge_wb_push_led_fn)(int64_t ts_ns, uint32_t shutter_ns, uint32_t ncam, const uint8_t *const *tiles,
                                    const uint32_t *strides);
//! One controller IMU sample in the controller's raw IMU axes (as Pimax receives them): m/s^2 and rad/s.
//! hand 0 = left, 1 = right. 0 = queued, -1 = no tracker running.
typedef int (*pibridge_wb_push_ctrl_imu_fn)(uint32_t hand, int64_t ts_ns, const float accel[3], const float gyro[3]);
//! The newest engine pose of a hand. 1 = tracked (out filled), 0 = the engine does not track it now.
typedef int (*pibridge_wb_ctrl_pose_fn)(uint32_t hand, struct pibridge_wb_ctrl_pose *out);

#ifdef __cplusplus
}
#endif
