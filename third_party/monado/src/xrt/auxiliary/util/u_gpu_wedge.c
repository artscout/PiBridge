// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-wide "GPU or display is wedged" flag.
 * @ingroup aux_util
 */

#include "util/u_gpu_wedge.h"
#include "util/u_logging.h"

#include <stdio.h>

static xrt_atomic_s32_t wedged = 0;
static char wedge_reason[128] = "";

void
u_gpu_wedge_set(const char *reason)
{
	if (xrt_atomic_s32_cmpxchg(&wedged, 0, 1) == 0) {
		snprintf(wedge_reason, sizeof(wedge_reason), "%s", reason != NULL ? reason : "unknown");
		U_LOG_E("GPU wedged (%s): from now on nothing waits for the GPU, teardown leaks the Vulkan objects.",
		        wedge_reason);
	}
}

void
u_gpu_wedge_clear(void)
{
	if (xrt_atomic_s32_cmpxchg(&wedged, 1, 0) == 1) {
		U_LOG_W("GPU no longer wedged (%s).", wedge_reason);
	}
}

bool
u_gpu_wedge_is_set(void)
{
	return xrt_atomic_s32_load(&wedged) != 0;
}

const char *
u_gpu_wedge_reason(void)
{
	return u_gpu_wedge_is_set() ? wedge_reason : "";
}
