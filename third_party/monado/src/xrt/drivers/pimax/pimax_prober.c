// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air prober (USB VID/PID 34a4:0044), plus a test-aid auto prober for PIMAX_NO_DEVICE=1.
 * @ingroup drv_pimax
 */

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "xrt/xrt_device.h"
#include "pimax_interface.h"

#include <stdlib.h>

DEBUG_GET_ONCE_BOOL_OPTION(pimax_no_device_probe, "PIMAX_NO_DEVICE", false)

// The device can show up more than once (one entry per USB interface), and the auto prober below may run too: only
// create one HMD.
static bool created = false;

//! Create the HMD and its controllers (and the Mercury hand device, if any) into out_xdev; returns the count.
static int
create_devices(const char *name, struct xrt_device **out_xdev)
{
	struct xrt_device *ctrls[2] = {NULL, NULL};
	struct xrt_device *xdev = pimax_hmd_create(name, ctrls);
	if (xdev == NULL) {
		return -1;
	}
	created = true;
	out_xdev[0] = xdev;
	int n = 1;
	// Camera hands (PIMAX_HANDS=mercury). With PIMAX_HT_ROLES=hands they are left/right hand controllers and come
	// before the Pimax controllers, so the role assignment (first of each type wins) gives them the hand roles.
	struct xrt_device *hands[2] = {NULL, NULL};
	pimax_hmd_take_hand_devices(xdev, hands);
	bool hands_first = hands[0] != NULL && hands[0]->device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	if (hands_first) {
		for (int i = 0; i < 2; i++) {
			if (hands[i] != NULL) {
				out_xdev[n++] = hands[i];
			}
		}
	}
	for (int i = 0; i < 2; i++) {
		if (ctrls[i] != NULL) {
			out_xdev[n++] = ctrls[i];
		}
	}
	if (!hands_first) {
		for (int i = 0; i < 2; i++) {
			if (hands[i] != NULL) {
				out_xdev[n++] = hands[i];
			}
		}
	}
	return n;
}

int
pimax_found(struct xrt_prober *xp,
            struct xrt_prober_device **devices,
            size_t device_count,
            size_t index,
            cJSON *attached_data,
            struct xrt_device **out_xdev)
{
	if (created) {
		return 0;
	}
	const bool crystal = devices[index]->product_id == PIMAX_DRV_PID_CRYSTAL_SUPER;
	return create_devices(crystal ? "Pimax Crystal Super" : "Pimax Dream Air", out_xdev);
}

/*
 * Test aid: with PIMAX_NO_DEVICE=1 the driver never opens the headset, so it does not need it on USB either. This auto
 * prober creates the Pimax HMD (identity pose, the real view/FOV/lens geometry, PIMAX_CTRL_HANDS_DEMO controllers)
 * when no headset was found, so offline captures (analysis/tools/run_hands_composite.sh, lens captures) work with
 * the headset unplugged. Without PIMAX_NO_DEVICE it does nothing.
 */

static int
pimax_no_device_autoprobe(
    struct xrt_auto_prober *xap, cJSON *attached_data, bool no_hmds, struct xrt_prober *xp, struct xrt_device **out_xdevs)
{
	if (no_hmds || created || !debug_get_bool_option_pimax_no_device_probe()) {
		return 0;
	}
	U_LOG_W("PIMAX_NO_DEVICE=1 and no headset on USB: creating the Pimax HMD without a device (test aid)");
	int n = create_devices("Pimax Dream Air", out_xdevs);
	return n < 0 ? 0 : n;
}

static void
pimax_no_device_destroy(struct xrt_auto_prober *xap)
{
	free(xap);
}

struct xrt_auto_prober *
pimax_create_auto_prober(void)
{
	struct xrt_auto_prober *xap = U_TYPED_CALLOC(struct xrt_auto_prober);
	if (xap == NULL) {
		return NULL;
	}
	xap->name = "Pimax Dream Air (PIMAX_NO_DEVICE)";
	xap->lelo_dallas_autoprobe = pimax_no_device_autoprobe;
	xap->destroy = pimax_no_device_destroy;
	return xap;
}
