// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PiBridge builder for the original Pimax Crystal (P3B) tracking itself (its own inside-out).
 *
 * The original Crystal tracks its head and controllers on the headset. Its HID belongs to pibridge-keepalived, which
 * streams the poses to shared memory (drivers/pimax/pimax_poseshm.h); this builder makes the Pimax HMD of it
 * (display, lens profile, eye gaze: PIMAX_MODEL=crystal-og with PIMAX_NO_DEVICE=1, the HID left alone) posed by the
 * head of that file, and the two Crystal controllers as Touch controllers. Only by name (XRT_BUILDER=pibridge_og,
 * from the PiBridge app); its Lighthouse mode is the pibridge_lh builder.
 *
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_results.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "target_builder_interface.h"
#include "target_builder_helpers.h"

#include "pimax/pimax_interface.h"
#include "pimax/pimax_poseshm.h"

#ifndef XRT_BUILD_DRIVER_PIMAX
#error "PiBridge original Crystal builder needs the pimax driver"
#endif

static const char *driver_list[] = {
    "pimax",
};

static xrt_result_t
pibridge_og_estimate_system(struct xrt_builder *xb, cJSON *config, struct xrt_prober *xp,
                            struct xrt_builder_estimate *estimate)
{
	// only when asked for by name (XRT_BUILDER)
	U_ZERO(estimate);
	return XRT_SUCCESS;
}

static xrt_result_t
pibridge_og_open_system_impl(struct xrt_builder *xb, cJSON *config, struct xrt_prober *xp,
                             struct xrt_tracking_origin *origin, struct xrt_system_devices *xsysd,
                             struct xrt_frame_context *xfctx, struct t_builder_options *tbo)
{
	if (xsysd->static_xdev_count + 4 > XRT_SYSTEM_MAX_DEVICES) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	struct xrt_device *head = NULL;
	struct xrt_device *ctrls[2] = {NULL, NULL};
	if (!pimax_poseshm_create(&head, ctrls)) {
		U_LOG_E("PiBridge: the original Crystal's devices could not be created");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	struct xrt_device *pimax_ctrls[2] = {NULL, NULL}; // none without the HID (PIMAX_NO_DEVICE)
	struct xrt_device *hmd = pimax_hmd_create("Pimax Crystal", pimax_ctrls);
	if (hmd == NULL) {
		U_LOG_E("PiBridge: the Pimax HMD could not be created (PIMAX_MODEL=crystal-og PIMAX_NO_DEVICE=1?)");
		xrt_device_destroy(&head);
		xrt_device_destroy(&ctrls[0]);
		xrt_device_destroy(&ctrls[1]);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	// the source first: destroyed after the HMD that reads it
	xsysd->static_xdevs[xsysd->static_xdev_count++] = head;
	pimax_hmd_set_head_source(hmd, head, NULL);
	xsysd->static_xdevs[xsysd->static_xdev_count++] = hmd;
	xsysd->static_xdevs[xsysd->static_xdev_count++] = ctrls[0];
	xsysd->static_xdevs[xsysd->static_xdev_count++] = ctrls[1];
	tbo->head = hmd;
	tbo->left = ctrls[0];
	tbo->right = ctrls[1];
	// the headset's own tracking space: y up, the floor at y = 0 (as Pimax's runtime has it)
	tbo->T_stage_local = (struct xrt_pose)XRT_POSE_IDENTITY;
	U_LOG_I("PiBridge: the original Crystal, its own inside-out (head and controllers from keepalived)");
	return XRT_SUCCESS;
}

static void
pibridge_og_destroy(struct xrt_builder *xb)
{
	free(xb);
}

struct xrt_builder *
t_builder_pibridge_og_create(void)
{
	struct t_builder *tb = U_TYPED_CALLOC(struct t_builder);
	tb->base.estimate_system = pibridge_og_estimate_system;
	tb->base.open_system = t_builder_open_system_static_roles;
	tb->base.destroy = pibridge_og_destroy;
	tb->base.identifier = "pibridge_og";
	tb->base.name = "PiBridge: the original Pimax Crystal tracking itself";
	tb->base.driver_identifiers = driver_list;
	tb->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	tb->base.exclude_from_automatic_discovery = true;
	tb->open_system_static_roles = pibridge_og_open_system_impl;
	return &tb->base;
}
