// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Reader of the optical controller poses written by `pimax-ctrltrack live` (pimax-re rust/pimax-ctrltrack,
 *         src/shm.rs: keep the layout in sync). Header only, no dependencies beyond libc.
 *
 * File (default $XDG_RUNTIME_DIR/pimax-ctrltrack.pose), 256 bytes, little endian, seqlock (seq odd while writing):
 *   0 u32 magic 0x54435850  4 u32 version 1  8 u32 seq  12 u32 writer pid
 *   16 + 96*hand (0 left, 1 right): u64 frame_ns (CLOCK_MONOTONIC of the LED frame, 0 = never), u64 publish_ns,
 *   f32 pos[3] (head/body frame, metres), f32 quat[4] body_from_aim (x y z w), f32 rms_px, u32 leds, u32 cams,
 *   f32 aim_from_imu[4] (x y z w).
 *   208: heartbeat (same version; 0 from older writers): u64 beat_ns (writer loop, CLOCK_MONOTONIC, at least every
 *   0.5 s while alive), u64 led_frames, u32 source_up (0 = broker/camera reopening), u32 reopens. Not read here;
 *   `pimax-ctrltrack health` prints it for pimaxd.
 * Body frame = OpenXR head axes (x right, y up, z back), the frame of the Pimax HMD pose.
 * @ingroup drv_pimax
 */

#pragma once

#include <fcntl.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define PIMAX_CTRLTRACK_MAGIC 0x54435850u
#define PIMAX_CTRLTRACK_SIZE 256
#define PIMAX_CTRLTRACK_HAND_OFF 16
#define PIMAX_CTRLTRACK_HAND_STRIDE 96

struct pimax_ctrltrack_hand
{
	uint64_t frame_ns;
	uint64_t publish_ns;
	float pos[3];
	float quat[4];
	float rms_px;
	uint32_t leds;
	uint32_t cams;
	float aim_from_imu[4];
};

struct pimax_ctrltrack_reader
{
	const uint8_t *map;
	ino_t ino;
};

static inline void
pimax_ctrltrack_default_path(char *out, size_t n)
{
	const char *env = getenv("PIMAX_CTRL_TRACK_FILE");
	if (env != NULL && env[0] != '\0') {
		snprintf(out, n, "%s", env);
		return;
	}
	const char *d = getenv("XDG_RUNTIME_DIR");
	snprintf(out, n, "%s/pimax-ctrltrack.pose", (d != NULL && d[0] != '\0') ? d : "/tmp");
}

static inline void
pimax_ctrltrack_close(struct pimax_ctrltrack_reader *r)
{
	if (r->map != NULL) {
		munmap((void *)r->map, PIMAX_CTRLTRACK_SIZE);
		r->map = NULL;
	}
}

//! (Re)map the file if it is not mapped or was replaced (new writer). Returns true when mapped.
static inline bool
pimax_ctrltrack_open(struct pimax_ctrltrack_reader *r, const char *path)
{
	struct stat st;
	if (stat(path, &st) != 0 || st.st_size < PIMAX_CTRLTRACK_SIZE) {
		pimax_ctrltrack_close(r);
		return false;
	}
	if (r->map != NULL && st.st_ino == r->ino) {
		return true;
	}
	pimax_ctrltrack_close(r);
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		return false;
	}
	void *p = mmap(NULL, PIMAX_CTRLTRACK_SIZE, PROT_READ, MAP_SHARED, fd, 0);
	close(fd);
	if (p == MAP_FAILED) {
		return false;
	}
	r->map = (const uint8_t *)p;
	r->ino = st.st_ino;
	return true;
}

//! Consistent snapshot of both hands. False if not mapped, wrong magic or the writer never settled.
static inline bool
pimax_ctrltrack_read(const struct pimax_ctrltrack_reader *r, struct pimax_ctrltrack_hand out[2])
{
	if (r->map == NULL) {
		return false;
	}
	const _Atomic uint32_t *seq = (const _Atomic uint32_t *)(const void *)(r->map + 8);
	for (int tries = 0; tries < 1000; tries++) {
		uint32_t s1 = atomic_load_explicit(seq, memory_order_acquire);
		if ((s1 & 1u) != 0) {
			continue;
		}
		uint32_t magic;
		memcpy(&magic, r->map, 4);
		if (magic != PIMAX_CTRLTRACK_MAGIC) {
			return false;
		}
		for (int h = 0; h < 2; h++) {
			const uint8_t *b = r->map + PIMAX_CTRLTRACK_HAND_OFF + h * PIMAX_CTRLTRACK_HAND_STRIDE;
			memcpy(&out[h].frame_ns, b + 0, 8);
			memcpy(&out[h].publish_ns, b + 8, 8);
			memcpy(out[h].pos, b + 16, 12);
			memcpy(out[h].quat, b + 28, 16);
			memcpy(&out[h].rms_px, b + 44, 4);
			memcpy(&out[h].leds, b + 48, 4);
			memcpy(&out[h].cams, b + 52, 4);
			memcpy(out[h].aim_from_imu, b + 56, 16);
		}
		atomic_thread_fence(memory_order_acquire);
		if (atomic_load_explicit(seq, memory_order_relaxed) == s1) {
			return true;
		}
	}
	return false;
}
