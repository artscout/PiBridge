// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-wide "GPU or display is wedged" flag.
 *
 * Set by the compositor when GPU work, or the flip of a presented image, does not complete and it gave up waiting
 * for it (XRT_COMPOSITOR_GPU_WAIT_TIMEOUT_S). From then on no code may wait for the GPU of the compositor's device:
 * vkDeviceWaitIdle, vkQueueWaitIdle, destroying images or memory that pending work uses and destroying the device
 * all block forever inside the driver. Teardown checks the flag and leaks those objects instead, and the service
 * leaves with _exit() so that no library destructor touches the driver either. The kernel frees everything when the
 * process exits; that exit must not be a SIGKILL while a DRM lease is held (analysis/nvidia-flip-hang.md).
 *
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Mark the GPU as wedged. The first reason is kept, later calls only keep the flag set.
 */
void
u_gpu_wedge_set(const char *reason);

/*!
 * Clear the flag again, for the case that the stuck work completed after all.
 */
void
u_gpu_wedge_clear(void);

/*!
 * True while the GPU is wedged.
 */
bool
u_gpu_wedge_is_set(void);

/*!
 * The reason given to the first u_gpu_wedge_set() call since the last clear, never NULL.
 */
const char *
u_gpu_wedge_reason(void);

#ifdef __cplusplus
}
#endif
