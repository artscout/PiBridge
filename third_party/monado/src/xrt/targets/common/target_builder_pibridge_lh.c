// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PiBridge Lighthouse builder: the Pimax Crystal Super with its Lighthouse faceplate.
 *
 * The headset is the Pimax HMD (display, lens profile, eye gaze: drivers/pimax); its head pose comes from the
 * faceplate, which Valve's driver_lighthouse (through steamvr_lh) brings up as a Lighthouse HMD of its own. The
 * Lighthouse controllers and trackers come straight from steamvr_lh. Selected only when PiBridge asks for it
 * (PIBRIDGE_LH=1, set by its Monado backend in the Lighthouse mode), so the inside-out session (the legacy builder's
 * Pimax HMD with its own tracking) is unchanged.
 *
 * PIBRIDGE_LH_HEAD_OFFSET="px py pz qx qy qz qw": the Pimax head relative to the faceplate's pose (default identity:
 * driver_lighthouse already reports the head pose of the headset it knows).
 * LH_STAGE_FILE (steamvr_lh): PiBridge's stage of this backend ("px py pz qx qy qz qw" = stage_from_raw), re-read
 * while running; SteamVR's chaperone file is never read in this mode. PIMAX_ANCHOR=1 records the head into the
 * Pimax driver's head ring (the room setup reads it there).
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
#include "util/u_system_helpers.h"

#include "math/m_api.h"

#include "target_builder_interface.h"
#include "target_builder_helpers.h"

#include "pimax/pimax_interface.h"
#include "steamvr_lh/steamvr_lh_interface.h"

#include <stdio.h>
#include <string.h>

#if !defined(XRT_BUILD_DRIVER_PIMAX) || !defined(XRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE)
#error "PiBridge Lighthouse builder needs the pimax and steamvr_lh drivers"
#endif

DEBUG_GET_ONCE_BOOL_OPTION(pibridge_lh, "PIBRIDGE_LH", false)
DEBUG_GET_ONCE_OPTION(pibridge_lh_head_offset, "PIBRIDGE_LH_HEAD_OFFSET", NULL)
DEBUG_GET_ONCE_LOG_OPTION(pibridge_lh_log, "PIBRIDGE_LH_LOG", U_LOGGING_INFO)

#define PLH_INFO(...) U_LOG_IFL_I(debug_get_log_option_pibridge_lh_log(), __VA_ARGS__)
#define PLH_WARN(...) U_LOG_IFL_W(debug_get_log_option_pibridge_lh_log(), __VA_ARGS__)
#define PLH_ERROR(...) U_LOG_IFL_E(debug_get_log_option_pibridge_lh_log(), __VA_ARGS__)

static const char *driver_list[] = {
    "pimax",
    "steamvr_lh",
};

static bool
pimax_on_usb(struct xrt_prober *xp)
{
	struct xrt_prober_device **devs = NULL;
	size_t count = 0;
	bool found = false;
	if (xrt_prober_lock_list(xp, &devs, &count) != XRT_SUCCESS) {
		return false;
	}
	for (size_t i = 0; i < count; i++) {
		if (devs[i]->vendor_id == PIMAX_DRV_VID &&
		    (devs[i]->product_id == PIMAX_DRV_PID_CRYSTAL_SUPER || devs[i]->product_id == PIMAX_DRV_PID)) {
			found = true;
			break;
		}
	}
	xrt_prober_unlock_list(xp, &devs);
	return found;
}

//! PIBRIDGE_LH_HEAD_OFFSET, else identity; a malformed value is reported and ignored.
static struct xrt_pose
head_offset(void)
{
	struct xrt_pose p = XRT_POSE_IDENTITY;
	const char *s = debug_get_option_pibridge_lh_head_offset();
	if (s == NULL || *s == '\0') {
		return p;
	}
	float v[7];
	if (sscanf(s, "%f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) != 7) {
		PLH_WARN("PIBRIDGE_LH_HEAD_OFFSET \"%s\" is not \"px py pz qx qy qz qw\": identity", s);
		return p;
	}
	p.position = (struct xrt_vec3){v[0], v[1], v[2]};
	p.orientation = (struct xrt_quat){v[3], v[4], v[5], v[6]};
	math_quat_normalize(&p.orientation);
	return p;
}

static xrt_result_t
pibridge_lh_estimate_system(struct xrt_builder *xb, cJSON *config, struct xrt_prober *xp,
                            struct xrt_builder_estimate *estimate)
{
	U_ZERO(estimate);
	if (!debug_get_bool_option_pibridge_lh() || !pimax_on_usb(xp)) {
		return XRT_SUCCESS;
	}
	estimate->certain.head = true;
	estimate->certain.dof6 = true;
	estimate->maybe.left = true;
	estimate->maybe.right = true;
	estimate->priority = 10; // over the steamvr and lighthouse builders, which would take the faceplate as the HMD
	return XRT_SUCCESS;
}

static xrt_result_t
pibridge_lh_open_system_impl(struct xrt_builder *xb, cJSON *config, struct xrt_prober *xp,
                             struct xrt_tracking_origin *origin, struct xrt_system_devices *xsysd,
                             struct xrt_frame_context *xfctx, struct t_builder_options *tbo)
{
	// 1. Lighthouse: the faceplate (an HMD to steamvr_lh) and the controllers / trackers.
	xrt_result_t r = steamvr_lh_create_devices(xp, xsysd);
	if (r != XRT_SUCCESS) {
		PLH_ERROR("steamvr_lh could not start (SteamVR installed? base stations on?)");
		return r;
	}
	struct xrt_device *face = NULL;
	for (size_t i = 0; i < xsysd->static_xdev_count; i++) {
		if (xsysd->static_xdevs[i]->device_type == XRT_DEVICE_TYPE_HMD) {
			face = xsysd->static_xdevs[i];
			break;
		}
	}
	if (face == NULL) {
		PLH_ERROR("no Lighthouse HMD: the faceplate is not seen by driver_lighthouse (USB, power, paired?)");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	// 2. The Pimax HMD: display, lens, eye gaze. Its own tracking stays off (no PIMAX_SLAM in this mode).
	if (xsysd->static_xdev_count + 3 > XRT_SYSTEM_MAX_DEVICES) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	struct xrt_device *ctrls[2] = {NULL, NULL};
	struct xrt_device *hmd = pimax_hmd_create("Pimax Crystal Super", ctrls);
	if (hmd == NULL) {
		PLH_ERROR("the Pimax HMD could not be created");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	const struct xrt_pose off = head_offset();
	pimax_hmd_set_head_source(hmd, face, &off);
	xsysd->static_xdevs[xsysd->static_xdev_count++] = hmd;
	PLH_INFO("head: Pimax HMD posed by the Lighthouse faceplate %s (%s)", face->str, face->serial);

	// 3. Roles: the Pimax HMD is the head; the hands are the Lighthouse controllers, else the Pimax ones.
	tbo->head = hmd;
	for (size_t i = 0; i < xsysd->static_xdev_count; i++) {
		struct xrt_device *d = xsysd->static_xdevs[i];
		if (d->device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER && tbo->left == NULL) {
			tbo->left = d;
		} else if (d->device_type == XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER && tbo->right == NULL) {
			tbo->right = d;
		}
	}
	for (int i = 0; i < 2; i++) {
		if (ctrls[i] == NULL) {
			continue;
		}
		if (xsysd->static_xdev_count < XRT_SYSTEM_MAX_DEVICES) {
			xsysd->static_xdevs[xsysd->static_xdev_count++] = ctrls[i];
		}
		if (i == 0 && tbo->left == NULL) {
			tbo->left = ctrls[i];
		} else if (i == 1 && tbo->right == NULL) {
			tbo->right = ctrls[i];
		}
	}
	PLH_INFO("left: %s, right: %s", tbo->left != NULL ? tbo->left->str : "<none>",
	         tbo->right != NULL ? tbo->right->str : "<none>");

	// steamvr_lh poses are in the chaperone's standing space: the floor is y = 0
	tbo->T_stage_local = (struct xrt_pose)XRT_POSE_IDENTITY;
	return XRT_SUCCESS;
}

static void
pibridge_lh_destroy(struct xrt_builder *xb)
{
	free(xb);
}

struct xrt_builder *
t_builder_pibridge_lh_create(void)
{
	struct t_builder *tb = U_TYPED_CALLOC(struct t_builder);
	tb->base.estimate_system = pibridge_lh_estimate_system;
	tb->base.open_system = t_builder_open_system_static_roles;
	tb->base.destroy = pibridge_lh_destroy;
	tb->base.identifier = "pibridge_lh";
	tb->base.name = "PiBridge: Pimax Crystal Super with the Lighthouse faceplate";
	tb->base.driver_identifiers = driver_list;
	tb->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	tb->open_system_static_roles = pibridge_lh_open_system_impl;
	return &tb->base;
}
