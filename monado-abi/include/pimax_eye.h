/* Eye gaze of the Pimax Dream Air (Tobii EyeChip 2104:0561) for the Monado driver. Rust crate pimax-eye
 * (live::Tracker), exported from libpimax_capi.a (pimax-capi re-exports it: Monado links ONE Rust staticlib).
 *
 * A background thread opens the chip over usbfs, sends the captured Windows init, subscribes to the gaze stream and
 * keeps the newest samples; on stop it unsubscribes and releases the interface; if the chip re-enumerates it starts
 * over by itself. All functions accept NULL.
 *
 * Frame: OpenXR head space, metres: x right, y up, -z forward. Eye order is [left, right]. The device-to-head
 * conversion is configurable through PIMAX_EYE_FLIP_X / _FLIP_Y / _FLIP_Z / _OFFSET_MM / _SWAP (see
 * rust/pimax-eye/src/track.rs); other environment: PIMAX_EYE_CAL, PIMAX_EYE_SKIP_AUTH, PIMAX_EYE_LATENCY_MS,
 * PIMAX_EYE_USB, PIMAX_EYE_CALIBRATION (per-eye calibration file, default ~/.config/omarchyvr/eye-calibration.toml;
 * "none" = raw; its frame conversion replaces the FLIP variables), PIMAX_EYE_FILTER (none|one-euro),
 * PIMAX_EYE_USB_RESET=0, PIMAX_EYE_POWER_ON=0. host_ts_ns is CLOCK_MONOTONIC (os_monotonic_get_ns). */
#ifndef PIMAX_EYE_H
#define PIMAX_EYE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bumped on every incompatible change of the structs below. Compare with pimax_eye_abi_version() at start-up. */
#define PIMAX_EYE_ABI_VERSION 1u

/* pimax_eye_start flags */
#define PIMAX_EYE_START_WITH_CAL (1u << 0) /* also send the captured calibration blob (0x456) */

/* pimax_eye_sample.flags */
#define PIMAX_EYE_GAZE_VALID (1u << 0)   /* origin / direction (combined ray) are valid */
#define PIMAX_EYE_LEFT_VALID (1u << 1)   /* eye_origin[0] / eye_direction[0] valid (the left eye is tracked) */
#define PIMAX_EYE_RIGHT_VALID (1u << 2)

/* pimax_eye_state() */
#define PIMAX_EYE_STATE_SEARCHING 0   /* no chip on the bus yet */
#define PIMAX_EYE_STATE_CONNECTING 1
#define PIMAX_EYE_STATE_STREAMING 2
#define PIMAX_EYE_STATE_RECONNECTING 3
#define PIMAX_EYE_STATE_STOPPED 4

/* Return codes of pimax_eye_latest / pimax_eye_sample_at */
#define PIMAX_EYE_OK 0
#define PIMAX_EYE_NO_SAMPLE (-1) /* nothing received yet */
#define PIMAX_EYE_BAD_ARG (-2)
#define PIMAX_EYE_STALE (-3)     /* newest sample older than 300 ms: tracker is (re)connecting; `out` is still filled */

typedef struct pimax_eye pimax_eye; /* opaque */

typedef struct pimax_eye_sample
{
	uint64_t seq;          /* running sample number since start */
	int64_t device_ts_us;  /* chip timestamp */
	int64_t host_ts_ns;    /* CLOCK_MONOTONIC time of the sample (device clock mapped to the host clock) */
	uint32_t flags;        /* PIMAX_EYE_*_VALID */
	uint32_t _pad;
	float origin[3];       /* combined gaze origin, metres, head space (midpoint of the contributing eyes) */
	float direction[3];    /* combined gaze direction, unit, head space */
	float eye_origin[2][3];
	float eye_direction[2][3];
	float pupil_mm[2];     /* negative when unknown */
} pimax_eye_sample;

uint32_t pimax_eye_abi_version(void);

/* Start the session thread (reads the PIMAX_EYE_* environment once). Returns NULL on failure. Never touches the
 * USB device before the thread has found it; a missing chip is not an error (state SEARCHING). */
pimax_eye *pimax_eye_start(uint32_t flags);
/* Stop the thread (unsubscribe, release the interface) and free the handle. */
void pimax_eye_stop(pimax_eye *h);

int pimax_eye_state(const pimax_eye *h);
/* Newest sample. */
int pimax_eye_latest(const pimax_eye *h, pimax_eye_sample *out);
/* Sample for host time at_ns (CLOCK_MONOTONIC): the combined ray is interpolated between neighbours and clamped to
 * the newest/oldest sample (no extrapolation: a prediction time in the future gets the newest sample). */
int pimax_eye_sample_at(const pimax_eye *h, int64_t at_ns, pimax_eye_sample *out);

/* Drift-correction hook (needs a per-eye calibration, ~/.config/omarchyvr/eye-calibration.toml): the wearer was looking
 * along dir (head space, unit, from the head origin) at host time at_ns, e.g. a confirmed click on a UI element.
 * Each calibrated eye's residual nudges a running per-eye offset (bounded, outliers ignored). Returns the number of
 * eyes that used it (0..2), PIMAX_EYE_BAD_ARG, or PIMAX_EYE_NO_SAMPLE (no calibration / no sample at that time). */
int pimax_eye_observe_target(const pimax_eye *h, int64_t at_ns, const float dir[3]);

#ifdef __cplusplus
}
#endif

#endif
