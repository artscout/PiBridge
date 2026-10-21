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

struct pimax_eyeglue
{
	enum u_logging_level log_level;
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

bool
pimax_eyeglue_enabled(void)
{
	return debug_get_bool_option_pimax_eye();
}

static void
start_locked(struct pimax_eyeglue *g)
{
	if (g->eye != NULL) {
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
