// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air controllers (Oculus Touch-like) fed from the libpimax controller callbacks.
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_device.h"
#include "util/u_logging.h"
#include "pimax.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pimax_ctrl_ctx;

/*! Create the shared context + left/right controller devices. @p head is only used for its head pose (yaw).
 *  Refcount: HMD + 2 controllers; the HMD calls pimax_ctrl_stop() once. */
struct pimax_ctrl_ctx *
pimax_ctrl_create(struct xrt_device *head, enum u_logging_level log_level, struct xrt_device *out_devs[2]);

/*! True if PIMAX_CTRL_HANDS_DEMO=1 (and hand tracking is on): scripted hand input for tests without a human. */
bool
pimax_ctrl_demo_enabled(void);

/*! Start the attach thread (burst on start, re-send on a 0x37 heartbeat without reports). Uses the HMD's session. */
void
pimax_ctrl_start(struct pimax_ctrl_ctx *ctx, pimax_t *lib);

/*! Stop the attach thread and drop the HMD's reference. Call BEFORE pimax_close(). */
void
pimax_ctrl_stop(struct pimax_ctrl_ctx *ctx);

// libpimax callbacks (session thread).
void
pimax_ctrl_on_report(struct pimax_ctrl_ctx *ctx, const pimax_controller_report *r);
void
pimax_ctrl_on_status(struct pimax_ctrl_ctx *ctx, const pimax_controller_status *s);
void
pimax_ctrl_on_state(struct pimax_ctrl_ctx *ctx, pimax_state state);

#ifdef __cplusplus
}
#endif
