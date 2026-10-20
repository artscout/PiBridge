/* Controller pose fusion for the optical (LED) controller tracking (Rust: pimax-hands src/fuse.rs, fuse_ffi.rs).
 * Out of view: short extrapolation, then an arm-model hold relative to the head (yaw + position), rest pose after
 * PIMAX_CTRL_TRACK_LOST_S (default 4 s). Orientation between fixes from the controller gyro once calibrated; the gyro
 * calibration (scale, axes, bias, time offset) is learned online from the optical poses and kept in
 * ~/.config/omarchyvr/ctrl-imu.json. Reacquire blends over 150 ms from the first display request; optical outliers vs the prediction are dropped.
 * Body model: shoulders from the head (position + yaw), reach 0.75 m, hands never closer than 8 cm, a hand at most
 * 0.3 m across the head centre; fixes that violate it are always dropped, predictions are kept inside it.
 * Env: PIMAX_CTRL_TRACK_IMU (0 off, 1 force, unset = when calibrated), PIMAX_CTRL_TRACK_LOST_S,
 * PIMAX_CTRL_TRACK_EXTRAP_MS (80), PIMAX_CTRL_TRACK_BLEND_MS (150), PIMAX_CTRL_IMU_FIT=0 (no online fit),
 * PIMAX_CTRL_IMU_CALIB (file), PIMAX_CTRL_IMU_LOG=<prefix> (log <prefix>-left.csv / -right.csv for
 * `pimax-ctrltrack imufit`), PIMAX_CTRL_IMU_LOG_MIN (10).
 * Code lives in libpimax_capi.a / libpimax_hands.a with the pimax_hands_* functions. All functions accept NULL. */
#ifndef PIMAX_CTRLFUSE_H
#define PIMAX_CTRLFUSE_H

#include <stddef.h>
#include <stdint.h>

#include "pimax_hands.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PimaxCtrlFuse pimax_ctrlfuse; /* opaque, one per hand */

#define PIMAX_CTRLFUSE_NONE 0    /* no pose: show the rest pose */
#define PIMAX_CTRLFUSE_OPTICAL 1 /* fresh optical fix (+ short extrapolation) */
#define PIMAX_CTRLFUSE_HOLD 2    /* out of view: held relative to the head */

typedef struct pimax_ctrlfuse_info
{
	uint32_t state; /* PIMAX_CTRLFUSE_* */
	uint32_t gyro;  /* orientation from the gyro between fixes */
} pimax_ctrlfuse_info;

/* hand: 0 left, 1 right. */
pimax_ctrlfuse *pimax_ctrlfuse_create(int hand);
/* Let the two hands see each other (body model: never closer than 8 cm). Call once after creating both. */
void pimax_ctrlfuse_link(pimax_ctrlfuse *left, pimax_ctrlfuse *right);
/* Saves an improved calibration, closes the log. */
void pimax_ctrlfuse_destroy(pimax_ctrlfuse *f);
/* Every controller report (64 raw bytes) with its host CLOCK_MONOTONIC arrival time. */
void pimax_ctrlfuse_report(pimax_ctrlfuse *f, uint64_t host_ns, const uint8_t *raw64);
/* Every new optical fix: world_from_aim and world_from_head at the LED frame time. */
void pimax_ctrlfuse_optical(pimax_ctrlfuse *f,
                            uint64_t frame_ns,
                            const pimax_hands_pose *world_from_aim,
                            const pimax_hands_pose *world_from_head,
                            float rms_px,
                            uint32_t leds);
/* Aim pose at at_ns (head pose at at_ns given). Returns PIMAX_CTRLFUSE_*; NONE leaves *out unspecified. */
int pimax_ctrlfuse_get(pimax_ctrlfuse *f,
                       uint64_t at_ns,
                       const pimax_hands_pose *world_from_head,
                       pimax_hands_pose *out,
                       pimax_ctrlfuse_info *info);
/* One-line status into buf (NUL terminated); returns the length. */
size_t pimax_ctrlfuse_describe(pimax_ctrlfuse *f, char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif
