// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax camera hands: Mercury on the tracking cameras, world-locked through the HMD's tracked head pose,
 *         exposed as two OpenXR hand devices (XR_EXT_hand_tracking joints + XR_EXT_hand_interaction pinch/poke/aim).
 *
 * Reliability (an optional leaf): Mercury runs on its own low-priority thread (nice, rate and duty-cycle limits);
 * the camera thread only hands it the newest pair and never waits. Missing models, a failed start or a slow or
 * stalled tracker only make the hands inactive; the HMD, the compositor and SLAM are not affected.
 *
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_device.h"
#include "xrt/xrt_frame.h"
#include "util/u_logging.h"

#include "pimax_cam.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pimax_ht;

//! Head relation at a (past) time: the HMD's tracked head pose in its tracking space.
typedef void (*pimax_ht_head_fn)(struct xrt_device *hmd, int64_t at_ns, struct xrt_space_relation *out);

struct pimax_ht_config
{
	uint32_t cams[2];              //!< factory camera ids, view 0 / view 1
	int orient[2];                 //!< Mercury view orientation in degrees, -1 = from the calibration
	struct xrt_pose head_from_cam; //!< Mercury output frame (view-0 camera, OpenXR axes) in the head frame
	bool hand_roles;               //!< devices are left/right hand controllers (take the controller roles)
	int nice;                      //!< nice value of the tracker threads (0 = unchanged)
	float max_hz;                  //!< max processed pairs per second
	float duty;                    //!< max busy fraction of the tracker thread (0..1]
	int stale_ms;                  //!< newest result older than this = hands inactive
	float predict_ms;              //!< max world-space extrapolation of the wrist (0 = hold the last pose)
	bool stats;                    //!< log rates every 5 s
	enum u_logging_level log_level;
};

/*!
 * Start the camera hands. Returns NULL (and logs why) when the models are missing or the calibration is unusable;
 * Mercury itself is created on the tracker thread, and a failure there only leaves the hands inactive.
 */
struct pimax_ht *
pimax_ht_create(struct xrt_device *hmd,
                pimax_ht_head_fn head_fn,
                const pimax_cam_calib *cal,
                const struct pimax_ht_config *cfg);

//! Camera thread: offer one stereo pair (view 0, view 1). Never blocks; dropped when the tracker is busy.
void
pimax_ht_push(struct pimax_ht *ht, struct xrt_frame *view0, struct xrt_frame *view1);

//! Left and right devices; call once, the system owns them afterwards.
void
pimax_ht_take_devices(struct pimax_ht *ht, struct xrt_device *out[2]);

//! Stats line(s) for the camera's 5 s log (no-op unless cfg.stats).
void
pimax_ht_log_stats(struct pimax_ht *ht);

//! Stop the tracker thread (before the HMD goes away) and drop the creator's reference. Devices still owned by
//! the system keep the shared state alive until they are destroyed; they then report inactive hands.
void
pimax_ht_destroy(struct pimax_ht **ht_ptr);

#ifdef __cplusplus
}
#endif
