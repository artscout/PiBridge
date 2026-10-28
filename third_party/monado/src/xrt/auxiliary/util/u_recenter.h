// Copyright 2026, PiBridge.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A recenter of the tracking origin, announced to the space overseer.
 *
 * A driver that recenters its own tracking (PiBridge's Pimax driver: the Pimax button long-press, the dashboard's
 * Recenter) calls u_recenter_announce() once its new origin is in place. The space overseer then moves the LOCAL and
 * LOCAL_FLOOR spaces (the shared ones and every app's own) onto the head, so seated apps that use LOCAL face where you
 * look too, not only the ones that use STAGE.
 *
 * @ingroup aux_util
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! A recenter happened: the space overseer recenters the local spaces on its next locate.
void
u_recenter_announce(void);

//! Counts the announcements (starts at 0).
uint32_t
u_recenter_generation(void);

#ifdef __cplusplus
}
#endif
