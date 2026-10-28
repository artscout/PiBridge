// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Room anchor hook (pimax_anchor.h): the head ring has one owner per session and its writer takes it back
 *         after it was replaced (live failure 2026-10-16: SteamVR's Monado driver recreated it and exited).
 */

#include "../src/xrt/drivers/pimax/pimax_anchor.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>

#define CHECK(x)                                                                                                       \
	do {                                                                                                           \
		if (!(x)) {                                                                                            \
			fprintf(stderr, "FAILED line %d: %s\n", __LINE__, #x);                                         \
			return 1;                                                                                      \
		}                                                                                                      \
	} while (0)

static char dir[256], ring[300], off[300];

static uint32_t
ring_pid(void)
{
	uint32_t hdr[4] = {0};
	FILE *f = fopen(ring, "rb");
	if (f == NULL) {
		return 0;
	}
	size_t n = fread(hdr, 4, 4, f);
	fclose(f);
	return n == 4 && hdr[0] == PIMAX_ANCHOR_MAGIC ? hdr[3] : 0;
}

//! A ring by another writer: @p pid, newest sample @p age_s old.
static void
write_foreign(uint32_t pid, double age_s)
{
	uint8_t b[PIMAX_ANCHOR_SIZE] = {0};
	uint32_t hdr[4] = {PIMAX_ANCHOR_MAGIC, 1, 0, pid};
	memcpy(b, hdr, sizeof(hdr));
	uint64_t ts = (uint64_t)((int64_t)os_monotonic_get_ns() - (int64_t)(age_s * 1e9));
	memcpy(b + PIMAX_ANCHOR_HDR, &ts, 8);
	char tmp[320];
	snprintf(tmp, sizeof(tmp), "%s.x", ring);
	FILE *f = fopen(tmp, "wb");
	fwrite(b, 1, sizeof(b), f);
	fclose(f);
	rename(tmp, ring);
}

static void
feed(void)
{
	struct xrt_space_relation rel = {0};
	rel.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT;
	rel.pose.orientation.w = 1.0f;
	rel.pose.position.y = 1.6f;
	pimax_anchor_process((int64_t)os_monotonic_get_ns(), &rel);
}

static void
reset_state(void)
{
	if (g_pimax_anchor.ring != NULL) {
		munmap(g_pimax_anchor.ring, PIMAX_ANCHOR_SIZE);
	}
	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
	memset(&g_pimax_anchor, 0, sizeof(g_pimax_anchor));
	g_pimax_anchor.lock = lock;
}

static uint32_t
dead_pid(void)
{
	pid_t c = fork();
	if (c == 0) {
		_exit(0);
	}
	waitpid(c, NULL, 0);
	return (uint32_t)c;
}

int
main(void)
{
	snprintf(dir, sizeof(dir), "/tmp/tests_pimax_anchor.%d", (int)getpid());
	mkdir(dir, 0700);
	snprintf(ring, sizeof(ring), "%s/pimax-anchor.head", dir);
	snprintf(off, sizeof(off), "%s/pimax-anchor.offset", dir);
	setenv("PIMAX_ANCHOR", "1", 1);
	setenv("PIMAX_ANCHOR_HEAD_FILE", ring, 1);
	setenv("PIMAX_ANCHOR_FILE", off, 1);
	uint32_t me = (uint32_t)getpid();

	// 1. start: a ring of a dead earlier session is replaced, ours is written
	write_foreign(dead_pid(), 0.0);
	feed();
	CHECK(g_pimax_anchor.enabled);
	CHECK(ring_pid() == me);
	printf("ok: start takes over a dead session's ring\n");

	// 2. another process (SteamVR's driver) replaced the ring and exited: taken back within the next check
	write_foreign(dead_pid(), 0.0);
	g_pimax_anchor.last_ring_check_ns = 0;
	feed();
	CHECK(ring_pid() == me);
	printf("ok: replaced ring with a dead writer taken back\n");

	// 3. removed: recreated
	unlink(ring);
	g_pimax_anchor.last_ring_check_ns = 0;
	feed();
	CHECK(ring_pid() == me);
	printf("ok: removed ring recreated\n");

	// 4. a live other writer keeps it fresh: left alone; once its samples are stale, taken back
	uint32_t parent = (uint32_t)getppid();
	write_foreign(parent, 0.0);
	g_pimax_anchor.last_ring_check_ns = 0;
	feed();
	CHECK(ring_pid() == parent);
	write_foreign(parent, 30.0);
	g_pimax_anchor.last_ring_check_ns = 0;
	feed();
	CHECK(ring_pid() == me);
	printf("ok: live writer respected, stale one replaced\n");

	// 5. a second process starting while the session's Monado owns a fresh ring: ring and offset untouched
	reset_state();
	write_foreign(parent, 0.0);
	FILE *f = fopen(off, "w");
	fprintf(f, "pimax-anchor-offset 1\npid %u\npose 0 0 0 0 0 0 1\n", parent);
	fclose(f);
	feed();
	CHECK(!g_pimax_anchor.enabled);
	CHECK(ring_pid() == parent);
	CHECK(access(off, F_OK) == 0);
	printf("ok: second process leaves the session's ring and offset alone\n");

	unlink(ring);
	unlink(off);
	rmdir(dir);
	return 0;
}
