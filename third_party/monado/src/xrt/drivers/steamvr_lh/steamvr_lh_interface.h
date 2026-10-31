// Copyright 2023, Shawn Wallace
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SteamVR driver device interface.
 * @author Shawn Wallace <yungwallace@live.com>
 * @ingroup drv_steamvr_lh
 */

#include <xrt/xrt_results.h>
#include <xrt/xrt_defines.h>

#include <stdbool.h>

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_steamvr_lh Wrapper for the SteamVR Lighthouse driver.
 * @ingroup drv
 *
 * @brief Wrapper driver around the SteamVR Lighthouse driver.
 */

/*!
 * @dir drivers/steamvr_lh
 *
 * @brief @ref drv_steamvr_lh files.
 */

/*!
 * Creates the steamvr system devices.
 *
 * @ingroup drv_steamvr_lh
 */
enum xrt_result
steamvr_lh_create_devices(struct xrt_prober *xp, struct xrt_system_devices *xsysd);

/*!
 * PiBridge (its Lighthouse builder): what the Pimax driver adds to the Lighthouse devices.
 *
 * @p world_offset: one more transform (the manual recenter) applied to the pose of every controller and tracker and
 * to the base stations, never to the Lighthouse HMD (the Pimax HMD applies it to the head itself). NULL: none.
 * @p right_system_button: the right Valve Index controller's system button (its only menu-like button) goes here and
 * never reaches the apps (the long-press recenter); the left one stays the menu. NULL: both reach the apps.
 *
 * Set before steamvr_lh_create_devices.
 *
 * @ingroup drv_steamvr_lh
 */
struct steamvr_lh_pibridge_hooks
{
	void (*world_offset)(struct xrt_pose *out);
	void (*right_system_button)(bool down);
};

void
steamvr_lh_set_pibridge_hooks(const struct steamvr_lh_pibridge_hooks *hooks);


#ifdef __cplusplus
}
#endif
