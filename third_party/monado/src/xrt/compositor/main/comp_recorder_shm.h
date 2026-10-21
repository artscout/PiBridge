// Copyright 2026, pimax-re contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared-memory layout between the compositor's recorder tap and `pimax-record` (the consumer).
 *
 * Mirrored byte for byte by `rust/pimax-record/src/shm.rs` (pimax-re). Version 1.
 *
 * The consumer creates the file (`$XDG_RUNTIME_DIR/pimax-record.shm`, or `$PIMAX_RECORD_SHM`) under a temporary
 * name, fills the request fields and renames it into place; the compositor polls for it, maps it and produces
 * frames into free slots. All fields marked "live" are accessed with atomics. Frames are RGBA8, rows tightly
 * packed (stride = width * 4), sRGB-encoded bytes.
 *
 * Slot states: 0 free, 1 producer writing, 2 ready, 3 consumer reading.
 * @ingroup comp_main
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#define PIMAX_REC_MAGIC 0x3143455258414d50ull // "PMAXREC1"
#define PIMAX_REC_VERSION 1u
#define PIMAX_REC_MAX_SLOTS 8u
#define PIMAX_REC_HEADER_SIZE 4096u

enum pimax_rec_slot_state
{
	PIMAX_REC_SLOT_FREE = 0,
	PIMAX_REC_SLOT_WRITING = 1,
	PIMAX_REC_SLOT_READY = 2,
	PIMAX_REC_SLOT_READING = 3,
};

struct pimax_rec_slot
{
	uint32_t state; //!< live
	uint32_t width;
	uint32_t height;
	uint32_t _pad;
	uint64_t seq;
	uint64_t timestamp_ns; //!< predicted display time, CLOCK_MONOTONIC
};

struct pimax_rec_header
{
	uint64_t magic;       // 0
	uint32_t version;     // 8
	uint32_t header_size; // 12

	// Request, written by the consumer before the rename.
	uint32_t eyes;           // 16: 1 = left eye, 2 = both side by side
	uint32_t fps;            // 20
	uint32_t max_height;     // 24
	uint32_t scale_permille; // 28: 0 = fit max_height, else a fraction of the eye's height
	uint32_t slot_count;     // 32
	uint32_t _pad0;          // 36
	uint64_t slot_size;      // 40

	// Consumer, live.
	uint64_t consumer_heartbeat_ns; // 48
	uint32_t stop;                  // 56

	// Producer, live.
	uint32_t producer_pid;          // 60
	uint64_t producer_heartbeat_ns; // 64
	uint32_t width;                 // 72
	uint32_t height;                // 76
	uint32_t src_width;             // 80
	uint32_t src_height;            // 84
	uint64_t produced;              // 88
	uint64_t dropped_busy;          // 96: GPU readback still busy
	uint64_t dropped_full;          // 104: every slot full (consumer behind)
	uint64_t skipped;               // 112: no squashed eye image this frame
	char producer_note[64];         // 120

	struct pimax_rec_slot slots[PIMAX_REC_MAX_SLOTS]; // 184
};

_Static_assert(offsetof(struct pimax_rec_header, consumer_heartbeat_ns) == 48, "layout");
_Static_assert(offsetof(struct pimax_rec_header, producer_heartbeat_ns) == 64, "layout");
_Static_assert(offsetof(struct pimax_rec_header, producer_note) == 120, "layout");
_Static_assert(offsetof(struct pimax_rec_header, slots) == 184, "layout");
_Static_assert(sizeof(struct pimax_rec_slot) == 32, "layout");
_Static_assert(sizeof(struct pimax_rec_header) <= PIMAX_REC_HEADER_SIZE, "layout");
