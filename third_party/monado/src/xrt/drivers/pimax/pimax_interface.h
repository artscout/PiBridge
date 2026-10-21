// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to the Pimax Dream Air driver.
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_prober.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_pimax Pimax Dream Air driver
 * @ingroup drv
 *
 * @brief 3DoF driver for the Pimax Dream Air on top of libpimax (hidraw wake/keepalive + 1 kHz IMU).
 *
 * Cameras / hands (pimax_camera.h): PIMAX_HANDS=mercury, PIMAX_CAM=1, PIMAX_CAM_SOURCE, PIMAX_CAM_RECORD,
 * PIMAX_CAM_CALIB, PIMAX_HT_CAMS, PIMAX_HT_ORIENT, PIMAX_HT_HEAD_OFFSET, PIMAX_HT_STATS.
 *
 * Env vars: PIMAX_LOG, PIMAX_PANEL_ROTATION (0/90/180/270, default 0, TODO),
 * PIMAX_AXIS_MODE (0 = identity, 1 = 180 deg about Y, default: verified by nose-down tilt test 2026-10-13), PIMAX_BIAS_SECONDS (default 2).
 */

#define PIMAX_DRV_VID 0x34a4
#define PIMAX_DRV_PID 0x0044
//! Pimax Crystal Super: the same protocol as the Dream Air (omarchyvr analysis/crystal-super-capture-20261017.md).
#define PIMAX_DRV_PID_CRYSTAL_SUPER 0x0040

/*! Prober "found" function, matches USB VID/PID. */
int
pimax_found(struct xrt_prober *xp,
            struct xrt_prober_device **devices,
            size_t device_count,
            size_t index,
            cJSON *attached_data,
            struct xrt_device **out_xdev);

/*! Auto prober (test aid): creates the HMD without a headset on USB when PIMAX_NO_DEVICE=1, otherwise nothing. */
struct xrt_auto_prober *
pimax_create_auto_prober(void);

/*! Create the HMD (opens libpimax session) named @p name. out_ctrls[2] receives left/right controllers (NULL if none). */
struct xrt_device *
pimax_hmd_create(const char *name, struct xrt_device *out_ctrls[2]);

/*! Left and right camera hand devices (PIMAX_HANDS=mercury), NULL each if none. Call once; the system owns them. */
void
pimax_hmd_take_hand_devices(struct xrt_device *hmd, struct xrt_device *out[2]);

/*! Head relation at a (usually past) time, as get_tracked_pose reports it, without recording it for the room
 * anchor. Thread-safe; used by the camera hands to place a hand seen at the frame time in the world. */
void
pimax_hmd_head_relation_at(struct xrt_device *hmd, int64_t at_timestamp_ns, struct xrt_space_relation *out_relation);

/*! Recenter (a long press of a controller's menu / Pimax button, pimax_ctrl.c): at the next compositor head query
 * the world turns and shifts so the head faces forward (-Z) above the origin; the floor height stays. Thread-safe. */
void
pimax_hmd_request_recenter(void);

#ifdef __cplusplus
}
#endif
