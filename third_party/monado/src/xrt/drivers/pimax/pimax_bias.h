// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air gyro bias: the rule that decides whether a window of IMU samples is a real rest.
 *
 * Header only and free of Monado types, so the same code runs in the driver (pimax_hmd.c) and in the test that
 * replays a session's head-imu.csv (pimax_bias_test.c).
 *
 * 2026-10-18: the old rule (every sample |gx|+|gy|+|gz| < 11 deg/s, first and second half means within 0.3 deg/s)
 * took a WORN window (head tilted 23 deg, gyro sd 0.5 deg/s) at 11:59:47 and saved 0.185 0.892 0.418 deg/s against
 * the true rest bias 0.118 0.775 0.370 (desk, 12:06:28-12:07:00). A worn head never gets as quiet as the bare sensor
 * on a desk (2 s windows: worn gyro sd 0.3-1.1 deg/s and accel sd 0.02-0.14 m/s^2 per axis; desk 0.06-0.15 deg/s
 * and 0.009-0.017 m/s^2), so a window is a rest only when every axis is that quiet, and its mean must be near the
 * bias in use (a slow steady turn is quiet too) unless two windows in a row agree on a new value.
 *
 * Provisional bias (2026-10-18, release review): with NO bias at all (no cache: a fresh machine), a noisier IMU or a
 * user who never rests the headset would keep a zero bias all session. So while there is no bias in use, the first
 * window under a looser rule (gyro sd < 0.3 deg/s, accel sd < 0.06 m/s^2 per axis, half means agreeing) is used as a
 * PROVISIONAL bias: applied, never cached. The next strict window replaces it (no "near" check against a provisional
 * value) and is cached as before. A worn window like 11:59:47 (sd 0.5 deg/s) is not even provisional.
 *
 * @ingroup drv_pimax
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Thresholds.
struct pimax_bias_rule
{
	uint32_t target;        //!< samples per window (1 kHz: ms)
	double sum_max_dps;     //!< any sample with |gx|+|gy|+|gz| above this restarts the window (moving)
	double half_tol_dps;    //!< first and second half means must agree within this (per axis)
	double gyro_sd_max_dps; //!< per-axis gyro standard deviation must be below this ...
	double accel_sd_max_g;  //!< ... and the per-axis accelerometer one below this
	double mean_tol_dps;    //!< the mean must be within this of the bias in use (per axis) ...
	double confirm_tol_dps; //!< ... unless the previous far window agreed with it within this (per axis)
	double prov_gyro_sd_max_dps; //!< with no bias in use: the looser gyro sd for a provisional bias ...
	double prov_accel_sd_max_g;  //!< ... and the looser accelerometer one
};

//! What a sample did.
enum pimax_bias_result
{
	PIMAX_BIAS_MORE = 0,     //!< window still filling
	PIMAX_BIAS_ACCEPT = 1,   //!< a real rest: the mean is the new bias
	PIMAX_BIAS_MOVING = 2,   //!< a moving sample restarted the window
	PIMAX_BIAS_DRIFTING = 3, //!< the two half means disagree
	PIMAX_BIAS_NOT_REST = 4, //!< too much spread for a headset at rest (worn)
	PIMAX_BIAS_FAR = 5,      //!< a rest, but far from the bias in use (kept to confirm)
	PIMAX_BIAS_PROVISIONAL = 6, //!< no bias in use yet, a quiet-enough window: use it, do not cache it
};

//! Window state.
struct pimax_bias_state
{
	uint32_t n;
	double g[3], g2[3], a[3], a2[3], half[3];
	bool have_prev; //!< `prev` = the bias in use (cache or an accepted window)
	bool prev_provisional; //!< `prev` came from a provisional window (not a real rest)
	float prev[3];
	bool have_far; //!< `far` = the last rest window rejected as far
	float far[3];
	uint32_t restarts;
	//! The last completed window's statistics (for the log).
	double last_mean[3], last_gsd[3], last_asd[3];
};

static inline void
pimax_bias_rule_default(struct pimax_bias_rule *r, uint32_t target)
{
	r->target = target;
	r->sum_max_dps = 11.0;
	r->half_tol_dps = 0.3;
	r->gyro_sd_max_dps = 0.15;
	r->accel_sd_max_g = 0.03 / 9.80665;
	r->mean_tol_dps = 0.3;
	r->confirm_tol_dps = 0.05;
	r->prov_gyro_sd_max_dps = 0.3;
	r->prov_accel_sd_max_g = 0.06 / 9.80665;
}

static inline void
pimax_bias_restart(struct pimax_bias_state *s)
{
	s->n = 0;
	for (int i = 0; i < 3; i++) {
		s->g[i] = s->g2[i] = s->a[i] = s->a2[i] = s->half[i] = 0;
	}
}

//! The bias in use (the cache at start, or a value set otherwise): later windows are compared with it.
static inline void
pimax_bias_set_prev(struct pimax_bias_state *s, const float g[3])
{
	for (int i = 0; i < 3; i++) {
		s->prev[i] = g[i];
	}
	s->have_prev = true;
	s->prev_provisional = false;
}

/*!
 * One raw sample (deg/s, g). On PIMAX_BIAS_ACCEPT `out_g` / `out_a` get the window means (gyro deg/s, accel g)
 * and the value becomes the bias in use (cache it). On PIMAX_BIAS_PROVISIONAL the same, but it is not a real rest
 * (use it, do not cache it). Every completed window (accepted or not) starts a new one.
 */
static inline enum pimax_bias_result
pimax_bias_feed(struct pimax_bias_state *s,
                const struct pimax_bias_rule *r,
                const float gyro_dps[3],
                const float accel_g[3],
                float out_g[3],
                float out_a[3])
{
	double gm = fabs(gyro_dps[0]) + fabs(gyro_dps[1]) + fabs(gyro_dps[2]);
	if (gm > r->sum_max_dps) {
		bool had = s->n > 0;
		if (had) {
			s->restarts++;
		}
		pimax_bias_restart(s);
		return had ? PIMAX_BIAS_MOVING : PIMAX_BIAS_MORE;
	}
	for (int i = 0; i < 3; i++) {
		s->g[i] += gyro_dps[i];
		s->g2[i] += (double)gyro_dps[i] * gyro_dps[i];
		s->a[i] += accel_g[i];
		s->a2[i] += (double)accel_g[i] * accel_g[i];
	}
	s->n++;
	if (s->n == r->target / 2) {
		for (int i = 0; i < 3; i++) {
			s->half[i] = s->g[i];
		}
	}
	if (s->n < r->target) {
		return PIMAX_BIAS_MORE;
	}
	double n = s->n, h = r->target / 2;
	double dmax = 0;
	bool quiet = true, loose = true;
	for (int i = 0; i < 3; i++) {
		double m = s->g[i] / n;
		double ma = s->a[i] / n;
		s->last_mean[i] = m;
		s->last_gsd[i] = sqrt(fmax(0.0, s->g2[i] / n - m * m));
		s->last_asd[i] = sqrt(fmax(0.0, s->a2[i] / n - ma * ma));
		dmax = fmax(dmax, fabs(s->half[i] / h - (s->g[i] - s->half[i]) / (n - h)));
		quiet = quiet && s->last_gsd[i] < r->gyro_sd_max_dps && s->last_asd[i] < r->accel_sd_max_g;
		loose = loose && s->last_gsd[i] < r->prov_gyro_sd_max_dps && s->last_asd[i] < r->prov_accel_sd_max_g;
		out_g[i] = (float)m;
		out_a[i] = (float)ma;
	}
	pimax_bias_restart(s);
	if (dmax > r->half_tol_dps) {
		s->restarts++;
		return PIMAX_BIAS_DRIFTING;
	}
	if (!quiet) {
		if (loose && !s->have_prev) {
			pimax_bias_set_prev(s, out_g);
			s->prev_provisional = true;
			return PIMAX_BIAS_PROVISIONAL;
		}
		return PIMAX_BIAS_NOT_REST;
	}
	bool near = true, confirmed = s->have_far;
	for (int i = 0; i < 3; i++) {
		near = near && (!s->have_prev || s->prev_provisional || fabs(out_g[i] - s->prev[i]) <= r->mean_tol_dps);
		confirmed = confirmed && fabs(out_g[i] - s->far[i]) <= r->confirm_tol_dps;
	}
	if (!near && !confirmed) {
		for (int i = 0; i < 3; i++) {
			s->far[i] = out_g[i];
		}
		s->have_far = true;
		return PIMAX_BIAS_FAR;
	}
	s->have_far = false;
	pimax_bias_set_prev(s, out_g);
	return PIMAX_BIAS_ACCEPT;
}

static inline const char *
pimax_bias_result_str(enum pimax_bias_result r)
{
	switch (r) {
	case PIMAX_BIAS_MORE: return "more";
	case PIMAX_BIAS_ACCEPT: return "accepted";
	case PIMAX_BIAS_MOVING: return "moving";
	case PIMAX_BIAS_DRIFTING: return "drifting";
	case PIMAX_BIAS_NOT_REST: return "not a rest (worn or handled)";
	case PIMAX_BIAS_FAR: return "far from the bias in use";
	case PIMAX_BIAS_PROVISIONAL: return "provisional (no bias yet, not a real rest)";
	}
	return "?";
}

#ifdef __cplusplus
}
#endif
