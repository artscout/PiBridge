// Copyright 2026, pimax-re contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Recorder tap: hands the squashed eye images to `pimax-record` through shared memory, never stalling.
 *
 * The compositor thread only records one small compute blit plus a copy into a host-visible buffer and submits it
 * with a fence (no waits, no syscalls apart from the submit); a low-priority worker thread waits for the fence,
 * copies the pixels into the shared-memory ring and frees the slot. If every readback slot is busy, or the ring is
 * full, the frame is dropped. Nothing is allocated or attached until a consumer creates the shared-memory file.
 * `PIMAX_RECORD=0` disables the tap (no worker thread).
 * @ingroup comp_main
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct comp_compositor;
struct chl_scratch_state;
struct comp_recorder;

/*!
 * Create the tap (and its idle worker thread). Returns NULL if disabled.
 */
struct comp_recorder *
comp_recorder_create(struct comp_compositor *c);

/*!
 * True while a consumer is attached: the compositor then always squashes layers into the scratch images (the fast
 * path skips them), so there is something to record.
 */
bool
comp_recorder_wants_layer_squash(void);

/*!
 * Called on the compositor thread after the frame's GPU work went idle. Never blocks.
 */
void
comp_recorder_capture(struct comp_recorder *rec,
                      const struct chl_scratch_state *scratch_state,
                      uint64_t frame_id,
                      uint64_t predicted_display_time_ns);

/*!
 * Stop the worker, wait (bounded) for pending readbacks, release everything.
 */
void
comp_recorder_destroy(struct comp_recorder **rec_ptr);

#ifdef __cplusplus
}
#endif
