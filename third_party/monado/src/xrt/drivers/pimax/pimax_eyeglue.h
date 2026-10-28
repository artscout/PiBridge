// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Eye gaze (XR_EXT_eye_gaze_interaction) of the Pimax Dream Air: thin glue to the Rust gaze session
 *         (pimax_eye.h, libpimax_capi.a). Opt-in with PIMAX_EYE=1.
 * @ingroup drv_pimax
 *
 * PIMAX_EYE=1       expose XRT_INPUT_GENERIC_EYE_GAZE_POSE on the HMD; the EyeChip is opened when an application
 *                   first uses the gaze space (xrt_device::begin_feature) and released when the last one is gone.
 * PIMAX_EYE_ALWAYS=1  open it when the service starts instead.
 * PIMAX_EYE_CAL=1   also send the captured calibration blob. Frame conversion: PIMAX_EYE_FLIP_X/Y/Z, _OFFSET_MM,
 *                   _SWAP (rust/pimax-eye/src/track.rs).
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_logging.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pimax_eyeglue;

/*! True when PIMAX_EYE=1. */
bool
pimax_eyeglue_enabled(void);

/*! Create the glue (no USB access yet). NULL when disabled or the Rust library has another ABI version. */
struct pimax_eyeglue *
pimax_eyeglue_create(enum u_logging_level log_level);

/*! An application started using the gaze pose: start the session (idempotent, counted). */
void
pimax_eyeglue_begin(struct pimax_eyeglue *g);

/*! The matching end: the last one stops the session (unsubscribe, release the interface). */
void
pimax_eyeglue_end(struct pimax_eyeglue *g);

/*! Stop everything and free. Sets *g to NULL. */
void
pimax_eyeglue_destroy(struct pimax_eyeglue **g);

/*!
 * Gaze pose in head space for host time @p at_ns: position = combined origin, -Z axis = gaze direction.
 * @return false while there is no valid gaze (not started, connecting, eyes lost, data older than 300 ms).
 */
bool
pimax_eyeglue_get_gaze(struct pimax_eyeglue *g, int64_t at_ns, struct xrt_pose *out_pose);

#ifdef __cplusplus
}
#endif
