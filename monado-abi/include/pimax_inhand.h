/* In-hand detector for the Pimax Dream Air controllers (Rust: pimax-hands src/inhand.rs, inhand_ffi.rs;
 * docs/CONTROLLERS-INHAND.md). From the controller's IMU and inputs: "held" (moved > 3.7 deg/s for 120 ms, tilted
 * > 10 deg, or an input changed) or "down" (still for 5 s, or no report for 1.5 s). The Monado driver reports no
 * pose and inactive inputs for a controller that is down (mode auto), so a controller lying on the desk is neither
 * drawn nor jumping around.
 * Mode: PIMAX_CTRL_INHAND = auto (when picked up) | always | off; at run time $XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode.
 * Status: $XDG_RUNTIME_DIR/pimax-ctrl-inhand (pid, mode, left/right held|down|shown|off and why).
 * Env: PIMAX_CTRL_INHAND_DOWN_S (seconds still before "down", default 5).
 * Code lives in libpimax_capi.a / libpimax_hands.a with the pimax_hands_* functions. All functions accept NULL. */
#ifndef PIMAX_INHAND_H
#define PIMAX_INHAND_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PimaxInHand pimax_inhand; /* opaque, one per controller */

#define PIMAX_INHAND_MODE_AUTO 0   /* shown only while held (Work) */
#define PIMAX_INHAND_MODE_ALWAYS 1 /* shown whenever reported (Play) */
#define PIMAX_INHAND_MODE_OFF 2    /* never shown, inputs inactive */

/* PIMAX_CTRL_INHAND, or default_mode when unset / invalid. */
int pimax_inhand_mode_from_env(int default_mode);
/* "auto" / "always" / "off" (static string). */
const char *pimax_inhand_mode_name(int mode);
/* hand: 0 left, 1 right. */
pimax_inhand *pimax_inhand_create(int hand);
void pimax_inhand_destroy(pimax_inhand *h);
/* Every controller report: 64 raw bytes (IMU) + decoded inputs (trigger/grip 0..255, stick -128..127, PIMAX_BTN_*
 * buttons, raw touch byte 49), host CLOCK_MONOTONIC arrival time. */
void pimax_inhand_report(pimax_inhand *h, uint64_t host_ns, const uint8_t *raw64, uint8_t trigger, uint8_t grip,
                         int stick_x, int stick_y, uint32_t buttons, uint8_t touch);
/* 1 = in the hand, 0 = lying somewhere / silent. */
int pimax_inhand_held(pimax_inhand *h, uint64_t now_ns);
/* Write the status file if it changed; 1 when written. */
int pimax_inhand_publish(int mode, pimax_inhand *left, pimax_inhand *right, uint64_t now_ns);
/* A run-time mode request (PIMAX_INHAND_MODE_*), consumed; -1 when there is none. */
int pimax_inhand_poll_mode(void);
/* Delete a stale run-time request (once at start). */
void pimax_inhand_clear_request(void);
/* "held (motion)" / "down (still)" into buf (NUL terminated); returns the length. */
size_t pimax_inhand_describe(pimax_inhand *h, uint64_t now_ns, char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif
