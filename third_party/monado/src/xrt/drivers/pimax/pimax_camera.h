// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air tracking cameras: capture (Rust pimax-cam) into xrt_frame sinks, Mercury hand tracking.
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_device.h"
#include "util/u_logging.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * IMU clock reference published by the HMD: monotonic_ns = dev_ext_us * 1000 + offset_ns for the IMU sample whose
 * u32 device timestamp is dev_ts_u32. Same layout as pimax_cam_imu_clock_ref.
 */
struct pimax_camera_clock_ref
{
	uint32_t dev_ts_u32;
	uint32_t valid;
	uint64_t dev_ext_us;
	int64_t offset_ns;
};

typedef bool (*pimax_camera_clock_fn)(void *user, struct pimax_camera_clock_ref *out);

/*!
 * 3DoF orientation of the IMU (OpenXR axes) at @p ts_ns from the HMD's own filter, for the 6DoF supervisor.
 */
typedef bool (*pimax_camera_orient_fn)(void *user, int64_t ts_ns, struct xrt_quat *out);

/*!
 * Send a host command `58 LL cmd opt data..` to the headset (the HMD owns the libpimax handle). Returns 0 on success.
 * Used only for the tracking-camera exposure push when PIMAX_CAM_AE=1.
 */
typedef int (*pimax_camera_send_fn)(void *user, uint8_t cmd, uint8_t opt, const uint8_t *data, size_t len);

struct pimax_camera;

/*!
 * Create the camera pipeline when enabled by the environment, else return NULL:
 *  - PIMAX_HANDS=mercury: Mercury hand tracking on calibration cameras PIMAX_HT_CAMS (default "0,1" = trackingA,
 *    trackingB), exposed as a hand-tracking device (XR_EXT_hand_tracking) attached to @p hmd.
 *  - PIMAX_CAM=1: capture only (debug sinks for the debug GUI, rate log).
 *  - PIMAX_CAM_SOURCE: "" = find the camera (default), /dev/videoN, or a .pxcam recording (replayed in real time,
 *    looping) for tests without the headset.
 *  - PIMAX_CAM_RECORD=<file.pxcam>: also record every raw frame.
 *  - PIMAX_CAM_CALIB=<xml>: factory calibration (default search, see pimax_cam.h).
 *  - PIMAX_HT_HEAD_OFFSET="x,y,z": head origin in the IMU body frame in metres (default: between the two tracking
 *    cameras, 5 cm behind them).
 *  - PIMAX_HT_STATS=1: log Mercury's processed-pair rate (needs HTA_PREDICTION_DISABLE=1; test aid).
 *  - PIMAX_SLAM=1: 6DoF head tracking with Monado's SLAM tracker (VIT library, default libbasalt.so, see
 *    VIT_SYSTEM_LIBRARY_PATH) fed by the cameras PIMAX_SLAM_CAMS (default "0,1") and the IMU; falls back to 3DoF
 *    while the tracker is not trustworthy. SLAM_CONFIG (optional) overrides the calibration we hand over.
 *  - PIMAX_SLAM_RECORD=<dir> or 1: record a EuRoC dataset (cameras PIMAX_SLAM_CAMS + IMU, same clock) for offline
 *    replay into <dir>/euroc_<datetime>/mav0, with <dir>/pimax-basalt-calib.json + pimax-slam-info.json. Works
 *    without PIMAX_SLAM and without Basalt. "1" = ~/.local/share/omarchyvr/slam-recordings/<date>-<time>.
 *  - PIMAX_SLAM_CAM_OFFSET_US: added to every camera timestamp sent to the tracker/recorder (camera-IMU time offset
 *    test knob, default 0).
 *  - PIMAX_SLAM_IMU=<json>: IMU noise model (default ~/.config/pibridge/slam-imu.json, else built-in defaults).
 *  - PIMAX_SLAM_EXPOSURE_MID (default 1): tracker/recorder frame time = metadata stamp + exposure/2 (the stamp
 *    marks the exposure start; 5.2 ms at the power-on exposure 10410 us). 0 = raw stamp (phase-1 recordings).
 *  - PIMAX_SLAM_SMOOTH=0 / PIMAX_SLAM_TAU_MS / PIMAX_SLAM_RESNAP=0: head pose output tuning (pimax_cam.h).
 *  - PIMAX_CAM_AE: tracking-camera auto-exposure. Unset/0 = off (cameras stay at the power-on 10410/449);
 *    "dry" = run the controller on the live frames and log what it would send; "1" = also send the exposure
 *    push (HID 0x51, the frame Windows sends at 12.5 Hz) through @p send. Tuning: PIMAX_CAM_AE_* (pimax_cam.h).
 *
 * @p clock (may be NULL) gives the HMD's IMU clock so frames share the IMU timeline. @p orient (may be NULL)
 * gives the 3DoF orientation history (needed for 6DoF). @p send (may be NULL) sends host commands (exposure).
 */
struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user,
                    pimax_camera_orient_fn orient,
                    void *orient_user,
                    pimax_camera_send_fn send,
                    void *send_user);

/*!
 * True when the camera pipeline wants IMU samples (SLAM tracking or recording active).
 */
bool
pimax_camera_wants_imu(const struct pimax_camera *pc);

/*!
 * One IMU sample (1 kHz, IMU clock in CLOCK_MONOTONIC ns, OpenXR head axes = PIMAX_AXIS_MODE 1, not bias
 * corrected). Called from the HMD's IMU thread.
 */
void
pimax_camera_push_imu(struct pimax_camera *pc, int64_t ts_ns, const struct xrt_vec3 *accel_m_s2,
                      const struct xrt_vec3 *gyro_rad_s);

/*!
 * Head relation with 6DoF: given the HMD's 3DoF relation at @p at_ns, write the relation to report. Returns false
 * when 6DoF is not active (report @p rel3dof unchanged). While the tracker is trusted the result is the SLAM pose
 * (predicted to @p at_ns); otherwise the 3DoF orientation (heading kept continuous) at the last good position.
 */
bool
pimax_camera_head_relation(struct pimax_camera *pc,
                           int64_t at_ns,
                           const struct xrt_space_relation *rel3dof,
                           struct xrt_space_relation *out);

/*!
 * The left and right camera hand devices (world-locked through the HMD pose), NULL each when hands are off.
 * Ownership moves to the caller (the system destroys them); call at most once.
 */
void
pimax_camera_take_hand_devices(struct pimax_camera *pc, struct xrt_device *out[2]);

/*! Stop capture and free everything (frame graph included). The HMD must outlive this call. */
void
pimax_camera_destroy(struct pimax_camera **pc_ptr);

#ifdef __cplusplus
}
#endif
