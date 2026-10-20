/* Controller-driven hand tracking for the Pimax Dream Air (Rust crate pimax-hands).
 * Trigger / grip / touch -> finger curls (One Euro filter + thumb state machine) and the grip->wrist offset.
 * The Monado driver feeds the result to u_hand_sim_simulate_generic(). All functions accept NULL. */
#ifndef PIMAX_HANDS_H
#define PIMAX_HANDS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bumped on every incompatible change of the structs below. Compare with pimax_hands_abi_version() at start-up. */
#define PIMAX_HANDS_ABI_VERSION 1u

/* pimax_hands_input.flags */
#define PIMAX_HANDS_TOUCH_BTN1 (1u << 0) /* thumb touches A (right) / X (left) */
#define PIMAX_HANDS_TOUCH_BTN2 (1u << 1) /* thumb touches B (right) / Y (left) */
#define PIMAX_HANDS_TOUCH_STICK (1u << 2)
#define PIMAX_HANDS_CLICK_BTN1 (1u << 3) /* A / X pressed */
#define PIMAX_HANDS_CLICK_BTN2 (1u << 4) /* B / Y pressed */
#define PIMAX_HANDS_CLICK_STICK (1u << 5)
#define PIMAX_HANDS_CLICK_MENU (1u << 6) /* menu (left) / system (right) pressed */

/* pimax_hands_out.thumb_state */
#define PIMAX_HANDS_THUMB_REST 0u
#define PIMAX_HANDS_THUMB_BTN1 1u
#define PIMAX_HANDS_THUMB_BTN2 2u
#define PIMAX_HANDS_THUMB_STICK 3u
#define PIMAX_HANDS_THUMB_MENU 4u

typedef struct pimax_hands pimax_hands; /* opaque, one per hand */

/* One decoded controller report. trigger and grip 0..1, stick -1..1. */
typedef struct pimax_hands_input
{
	float trigger;
	float grip;
	float stick_x;
	float stick_y;
	uint32_t flags;
} pimax_hands_input;

/* Same layout as Monado's struct u_hand_tracking_finger_value without joint_count.
 * joint_curls: [metacarpal, proximal, intermediate, distal] flexion in radians. */
typedef struct pimax_hands_finger
{
	float splay;
	float joint_curls[4];
} pimax_hands_finger;

typedef struct pimax_hands_out
{
	/* Order of struct u_hand_tracking_values. */
	pimax_hands_finger little, ring, middle, index, thumb;
	/* Normalised curl 0..1: little, ring, middle, index, thumb. */
	float curl[5];
	uint32_t thumb_state;
} pimax_hands_out;

/* Rigid transform; orientation is x, y, z, w. */
typedef struct pimax_hands_pose
{
	float orientation[4];
	float position[3];
} pimax_hands_pose;

uint32_t pimax_hands_abi_version(void);

/* New hand in the relaxed rest pose, NULL on allocation failure. */
pimax_hands *pimax_hands_create(void);
void pimax_hands_destroy(pimax_hands *h);
/* Forget all history (controller reconnected). */
void pimax_hands_reset(pimax_hands *h);
/* Feed one report. now_ns: any monotonic clock, nanoseconds. A gap > 0.5 s restarts the filters from this report. */
void pimax_hands_update(pimax_hands *h, uint64_t now_ns, const pimax_hands_input *input);
/* Feed one controller report straight from the fields of libpimax's pimax_controller_report: trigger and grip 0..255,
 * stick_x / stick_y -128..127, buttons (PIMAX_BTN_* bits of pimax.h), touch (raw report byte 49). The bit meanings are
 * decoded inside the library. This is what the Monado driver calls for real reports. */
void pimax_hands_update_report(pimax_hands *h, uint64_t now_ns, uint8_t trigger, uint8_t grip, int stick_x, int stick_y,
                               uint32_t buttons, uint8_t touch);
/* Curls of the latest update. */
void pimax_hands_get(const pimax_hands *h, pimax_hands_out *out);

/* Pose of the hand-simulation root (the wrist) in the controller's grip frame: wrist = grip_pose * *out. */
void pimax_hands_grip_to_wrist(int is_right, pimax_hands_pose *out);

/* Scripted test input for the monotonic time now_ns (a pure function, see demo.rs): lets the driver and a test client
 * agree on what the "controllers" did without a human. Right hand runs 2.5 s later than the left. */
void pimax_hands_demo_input(uint64_t now_ns, int is_right, pimax_hands_input *out);

/* Synthesized pose of an untracked controller (the Dream Air controllers have no positional tracking): aim and grip
 * pose for the head orientation head_orientation[4] (x, y, z, w; NULL = identity), in the tracking-origin frame with
 * the head at the origin. The placement follows head yaw; it comes from PIMAX_CTRL_REST (desk | legacy |
 * side,down,fwd[,grip_pitch_deg]; default desk: in the lower part of the view) and PIMAX_CTRL_AIM (level | head |
 * head:<m>; head = the aim ray points at the spot that far ahead of the view, look to aim), read once. Outputs may
 * be NULL. Added after ABI version 1 without changing it (new functions only). */
void pimax_hands_controller_poses(int is_right, const float *head_orientation, pimax_hands_pose *aim,
                                  pimax_hands_pose *grip);

/* Grip relative to aim (the fixed handle pitch): grip = aim * *out, for other orientation sources (IMU mode). */
void pimax_hands_aim_to_grip(pimax_hands_pose *out);

/* Grip relative to aim of a TRACKED controller (optical / IMU): grip = aim * *out, rotation AND translation, from the
 * Oculus Touch (Quest 2) openxr_aim / openxr_grip components SteamVR uses (60 deg pitch, grip 3.0 cm below and
 * 9.5 cm behind the aim origin), so SteamVR's derived aim equals the driver's aim. Added without an ABI bump. */
void pimax_hands_tracked_aim_to_grip(pimax_hands_pose *out);

/* One NUL-terminated line describing the placement in use (truncated to len). Returns 1 when PIMAX_CTRL_REST or
 * PIMAX_CTRL_AIM was invalid (defaults used), else 0. */
int pimax_hands_controller_describe(char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* PIMAX_HANDS_H */
