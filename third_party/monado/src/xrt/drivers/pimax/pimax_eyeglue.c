// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Glue between the Rust gaze session (pimax_eye.h) and the Pimax HMD xrt_device.
 * @ingroup drv_pimax
 */

#include "pimax_eyeglue.h"

#include "os/os_threading.h"
#include "math/m_api.h"
#include "util/u_debug.h"
#include "util/u_misc.h"

#ifndef PIMAX_NO_RUST_EYE
#include "pimax_eye.h"
#endif

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "os/os_time.h"

#ifdef PIMAX_NO_RUST_EYE
// Built with the C libpimax: no gaze session available, PIMAX_EYE is ignored.
bool
pimax_eyeglue_enabled(void)
{
	return false;
}
struct pimax_eyeglue *
pimax_eyeglue_create(enum u_logging_level log_level)
{
	(void)log_level;
	return NULL;
}
void
pimax_eyeglue_begin(struct pimax_eyeglue *g)
{
	(void)g;
}
void
pimax_eyeglue_end(struct pimax_eyeglue *g)
{
	(void)g;
}
void
pimax_eyeglue_destroy(struct pimax_eyeglue **g)
{
	(void)g;
}
bool
pimax_eyeglue_get_gaze(struct pimax_eyeglue *g, int64_t at_ns, struct xrt_pose *out_pose)
{
	(void)g;
	(void)at_ns;
	(void)out_pose;
	return false;
}
#else

DEBUG_GET_ONCE_BOOL_OPTION(pimax_eye, "PIMAX_EYE", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_eye_always, "PIMAX_EYE_ALWAYS", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_eye_cal, "PIMAX_EYE_CAL", false)
// PiBridge: the gaze from pibridge-eyed's shared memory instead of opening the eye chip (the daemon keeps it):
// PIMAX_EYE_SHM=<file> (PiBridge libpibridge-eye shm.h layout, "OVEG" v1, 136 bytes, head frame of OpenXR).
DEBUG_GET_ONCE_OPTION(pimax_eye_shm, "PIMAX_EYE_SHM", NULL)

#define EYE_SHM_SIZE 136
#define EYE_SHM_MAGIC 0x4745564fu
#define EYE_SHM_COMBINED 4u
#define EYE_SHM_STALE_NS (200LL * 1000 * 1000)

struct pimax_eyeglue
{
	enum u_logging_level log_level;
	//! PIMAX_EYE_SHM: the file (opened lazily: the daemon may start after Monado), else -1.
	bool shm;
	int shm_fd;
	struct os_mutex lock; //!< guards eye, users
	pimax_eye *eye;
	int users;
	//! Diagnostics window (lock): get_gaze outcomes since stat_start_ns, logged every 5 s.
	uint64_t stat_start_ns;
	uint32_t st_calls, st_ok, st_none, st_stale, st_invalid;
	uint64_t st_first_seq, st_last_seq;
	bool st_have_seq;
};

//! Lock held: count one get_gaze outcome and log the window every 5 s while the pose is being asked for.
static void
stats_note_locked(struct pimax_eyeglue *g, int r, uint32_t flags, uint64_t seq)
{
	uint64_t now = os_monotonic_get_ns();
	if (g->stat_start_ns == 0) {
		g->stat_start_ns = now;
	}
	g->st_calls++;
	if (r == PIMAX_EYE_NO_SAMPLE || r == PIMAX_EYE_BAD_ARG) {
		g->st_none++;
	} else {
		if (!g->st_have_seq) {
			g->st_first_seq = seq;
			g->st_have_seq = true;
		}
		g->st_last_seq = seq;
		if (r == PIMAX_EYE_STALE) {
			g->st_stale++;
		} else if ((flags & PIMAX_EYE_GAZE_VALID) == 0) {
			g->st_invalid++;
		} else {
			g->st_ok++;
		}
	}
	double secs = (double)(now - g->stat_start_ns) / 1e9;
	if (secs < 5.0) {
		return;
	}
	U_LOG_IFL_I(g->log_level,
	            "Pimax eye: state %d, %u gaze queries in %.1f s: %u valid, %u invalid (eyes lost), %u stale, %u no sample; "
	            "chip samples %.0f/s",
	            g->eye != NULL ? pimax_eye_state(g->eye) : -1, g->st_calls, secs, g->st_ok, g->st_invalid,
	            g->st_stale, g->st_none, g->st_have_seq ? (double)(g->st_last_seq - g->st_first_seq) / secs : 0.0);
	g->stat_start_ns = now;
	g->st_calls = g->st_ok = g->st_none = g->st_stale = g->st_invalid = 0;
	g->st_have_seq = false;
}

static const char *
shm_path(void)
{
	const char *p = debug_get_option_pimax_eye_shm();
	return (p != NULL && p[0] != '\0') ? p : NULL;
}

bool
pimax_eyeglue_enabled(void)
{
	return debug_get_bool_option_pimax_eye() || shm_path() != NULL;
}

//! PIMAX_EYE_SHM: the newest combined gaze of the daemon's file (seqlock: seq_end, body, seq_begin, equal and even).
static bool
shm_gaze(struct pimax_eyeglue *g, struct xrt_pose *out_pose)
{
	if (g->shm_fd < 0) {
		g->shm_fd = open(shm_path(), O_RDONLY | O_CLOEXEC);
		if (g->shm_fd < 0) {
			return false;
		}
	}
	uint8_t b[EYE_SHM_SIZE];
	for (int tries = 0; tries < 4; tries++) {
		uint64_t end = 0, begin = 0;
		if (pread(g->shm_fd, &end, 8, 128) != 8 || pread(g->shm_fd, b, EYE_SHM_SIZE, 0) != EYE_SHM_SIZE ||
		    pread(g->shm_fd, &begin, 8, 0) != 8) {
			close(g->shm_fd); // the daemon recreated it, or it is gone
			g->shm_fd = -1;
			return false;
		}
		if (begin != end || (begin & 1u) != 0) {
			continue;
		}
		uint32_t magic, version, flags;
		int64_t published;
		float o[3], d[3];
		memcpy(&magic, b + 8, 4);
		memcpy(&version, b + 12, 4);
		memcpy(&published, b + 24, 8);
		memcpy(&flags, b + 32, 4);
		memcpy(o, b + 88, 12);
		memcpy(d, b + 100, 12);
		if (magic != EYE_SHM_MAGIC || version != 1 || (flags & EYE_SHM_COMBINED) == 0 ||
		    (int64_t)os_monotonic_get_ns() - published > EYE_SHM_STALE_NS) {
			return false;
		}
		const struct xrt_vec3 forward = {0.0f, 0.0f, -1.0f};
		const struct xrt_vec3 dir = {d[0], d[1], d[2]};
		struct xrt_pose pose = XRT_POSE_IDENTITY;
		math_quat_from_vec_a_to_vec_b(&forward, &dir, &pose.orientation);
		math_quat_normalize(&pose.orientation);
		pose.position = (struct xrt_vec3){o[0], o[1], o[2]};
		*out_pose = pose;
		return true;
	}
	return false;
}

static void
start_locked(struct pimax_eyeglue *g)
{
	if (g->shm || g->eye != NULL) {
		return;
	}
	g->eye = pimax_eye_start(debug_get_bool_option_pimax_eye_cal() ? PIMAX_EYE_START_WITH_CAL : 0);
	U_LOG_IFL_I(g->log_level, "Pimax eye tracking: %s", g->eye != NULL ? "session started" : "start failed");
}

static void
stop_locked(struct pimax_eyeglue *g)
{
	if (g->eye == NULL) {
		return;
	}
	pimax_eye_stop(g->eye); // unsubscribes and releases the interface before returning
	g->eye = NULL;
	U_LOG_IFL_I(g->log_level, "Pimax eye tracking: session stopped");
}

struct pimax_eyeglue *
pimax_eyeglue_create(enum u_logging_level log_level)
{
	if (!pimax_eyeglue_enabled()) {
		return NULL;
	}
	if (shm_path() != NULL) {
		struct pimax_eyeglue *gs = U_TYPED_CALLOC(struct pimax_eyeglue);
		if (gs == NULL) {
			return NULL;
		}
		gs->log_level = log_level;
		gs->shm = true;
		gs->shm_fd = -1;
		os_mutex_init(&gs->lock);
		U_LOG_IFL_I(log_level, "Pimax eye: the gaze from %s (pibridge-eyed keeps the chip)", shm_path());
		return gs;
	}
	if (pimax_eye_abi_version() != PIMAX_EYE_ABI_VERSION) {
		U_LOG_IFL_E(log_level, "libpimax_capi eye ABI %u, driver built for %u: eye tracking off",
		            (unsigned)pimax_eye_abi_version(), (unsigned)PIMAX_EYE_ABI_VERSION);
		return NULL;
	}
	struct pimax_eyeglue *g = U_TYPED_CALLOC(struct pimax_eyeglue);
	if (g == NULL) {
		return NULL;
	}
	g->log_level = log_level;
	g->shm_fd = -1;
	os_mutex_init(&g->lock);
	if (debug_get_bool_option_pimax_eye_always()) {
		os_mutex_lock(&g->lock);
		start_locked(g);
		os_mutex_unlock(&g->lock);
	}
	return g;
}

void
pimax_eyeglue_begin(struct pimax_eyeglue *g)
{
	if (g == NULL) {
		return;
	}
	os_mutex_lock(&g->lock);
	g->users++;
	start_locked(g);
	os_mutex_unlock(&g->lock);
}

void
pimax_eyeglue_end(struct pimax_eyeglue *g)
{
	if (g == NULL) {
		return;
	}
	os_mutex_lock(&g->lock);
	if (g->users > 0) {
		g->users--;
	}
	if (g->users == 0 && !debug_get_bool_option_pimax_eye_always()) {
		stop_locked(g);
	}
	os_mutex_unlock(&g->lock);
}

void
pimax_eyeglue_destroy(struct pimax_eyeglue **g_ptr)
{
	struct pimax_eyeglue *g = *g_ptr;
	if (g == NULL) {
		return;
	}
	os_mutex_lock(&g->lock);
	stop_locked(g);
	if (g->shm_fd >= 0) {
		close(g->shm_fd);
	}
	os_mutex_unlock(&g->lock);
	os_mutex_destroy(&g->lock);
	free(g);
	*g_ptr = NULL;
}

bool
pimax_eyeglue_get_gaze(struct pimax_eyeglue *g, int64_t at_ns, struct xrt_pose *out_pose)
{
	if (g == NULL) {
		return false;
	}
	if (g->shm) {
		(void)at_ns; // the daemon publishes latency-corrected samples; the newest one is the gaze now
		os_mutex_lock(&g->lock);
		const bool ok = shm_gaze(g, out_pose);
		os_mutex_unlock(&g->lock);
		return ok;
	}
	pimax_eye_sample s;
	int r;
	os_mutex_lock(&g->lock);
	s.flags = 0;
	s.seq = 0;
	r = pimax_eye_sample_at(g->eye, at_ns, &s); // NULL handle -> PIMAX_EYE_BAD_ARG
	stats_note_locked(g, r, s.flags, s.seq);
	os_mutex_unlock(&g->lock);
	if (r != PIMAX_EYE_OK || (s.flags & PIMAX_EYE_GAZE_VALID) == 0) {
		return false;
	}

	// OpenXR: the gaze pose's -Z axis points where the user looks.
	const struct xrt_vec3 forward = {0.0f, 0.0f, -1.0f};
	const struct xrt_vec3 dir = {s.direction[0], s.direction[1], s.direction[2]};
	struct xrt_pose pose = XRT_POSE_IDENTITY;
	math_quat_from_vec_a_to_vec_b(&forward, &dir, &pose.orientation);
	math_quat_normalize(&pose.orientation);
	pose.position.x = s.origin[0];
	pose.position.y = s.origin[1];
	pose.position.z = s.origin[2];
	*out_pose = pose;
	return true;
}

#endif // PIMAX_NO_RUST_EYE
