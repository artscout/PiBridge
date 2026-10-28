// Copyright 2026, PiBridge.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A recenter of the tracking origin, announced to the space overseer.
 * @ingroup aux_util
 */

#include "util/u_recenter.h"

#include <stdatomic.h>

static atomic_uint_fast32_t g_generation;

void
u_recenter_announce(void)
{
	atomic_fetch_add(&g_generation, 1);
}

uint32_t
u_recenter_generation(void)
{
	return (uint32_t)atomic_load(&g_generation);
}
