/* libpimax - Pimax Dream Air (34a4:0044) host library over Linux hidraw.
 * Protocol: analysis/init-sequence.md, analysis/controllers-settings.md.
 * All callbacks run on the library's session thread: keep them short, never call
 * pimax_request() from inside a callback (it would wait for the thread it is running on). */
#ifndef PIMAX_H
#define PIMAX_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIMAX_VID 0x34a4
#define PIMAX_PID 0x0044               /* Dream Air */
#define PIMAX_PID_CRYSTAL_SUPER 0x0040 /* same protocol (analysis/crystal-super-capture-20261017.md) */
#define PIMAX_NUM_IF 4         /* USB interfaces 0..3 */
#define PIMAX_PATH_MAX 64
#define PIMAX_REPORT_SIZE 64
#define PIMAX_MAX_DATA 60      /* payload bytes of one command (64 - 4 header) */

#define PIMAX_SLOT_LEFT 2
#define PIMAX_SLOT_RIGHT 3

/* controller buttons (pimax_controller_report.buttons) */
#define PIMAX_BTN_B 0x01
#define PIMAX_BTN_A 0x02
#define PIMAX_BTN_STICK 0x08
#define PIMAX_BTN_MENU 0x10    /* byte 48 bit 0, remapped */
/* controller touch bits (pimax_controller_report.touch, raw byte 49) */
#define PIMAX_TOUCH_STICK 0x08

typedef struct pimax pimax_t;

typedef struct {
    uint32_t ts_us;            /* device clock; 2nd sample of a report is ts + 1000 (approximate) */
    int16_t  raw_accel[3];
    int16_t  raw_gyro[3];
    float    accel_g[3];       /* raw / 2048 */
    float    gyro_dps[3];      /* raw / 16.4 */
    float    ipd_mm;           /* live lens IPD (report bytes 45-46, u16 LE / 100) */
    int      sample_in_report; /* 0 or 1 */
} pimax_imu_sample;

typedef struct {
    uint8_t  slot;             /* PIMAX_SLOT_LEFT / PIMAX_SLOT_RIGHT */
    uint32_t ts_us;            /* device uptime clock */
    int      stick_x, stick_y; /* -127..127, right/up positive */
    uint8_t  trigger, grip;    /* 0..255 */
    uint8_t  battery;          /* percent (medium confidence) */
    uint32_t buttons;          /* PIMAX_BTN_* */
    uint8_t  touch;            /* raw byte 49 */
    int16_t  raw_accel[3];     /* ~1024 LSB/g, big-endian on the wire */
    int16_t  raw_gyro[3];      /* unit unknown */
    uint8_t  raw[PIMAX_REPORT_SIZE];
} pimax_controller_report;

typedef struct {               /* unsolicited cmd 0x37, every 2 s per slot */
    uint8_t slot;
    uint8_t battery;
    uint8_t fw[2];
    uint8_t raw[PIMAX_REPORT_SIZE];
} pimax_controller_status;

typedef enum { PIMAX_DISCONNECTED = 0, PIMAX_CONNECTED = 1 } pimax_state;

typedef struct {
    void *user;
    void (*on_state)(void *user, pimax_state state);
    void (*on_imu)(void *user, const pimax_imu_sample *s);
    void (*on_controller)(void *user, const pimax_controller_report *r);
    void (*on_controller_status)(void *user, const pimax_controller_status *s);
    void (*on_log)(void *user, const char *line);       /* IF3 firmware log, one line, no newline */
    void (*on_reply)(void *user, const uint8_t report[PIMAX_REPORT_SIZE]); /* every IF0 report; may be NULL */
} pimax_callbacks;

/* Discovery: fill paths[n] with "/dev/hidrawN" for USB interface n. Returns bitmask of found interfaces. */
int pimax_find_interfaces(char paths[PIMAX_NUM_IF][PIMAX_PATH_MAX]);

/* Start the session thread (discovery, wake, keepalive, drain, auto-reconnect). cb may be NULL. */
pimax_t *pimax_open(const pimax_callbacks *cb);
void pimax_close(pimax_t *h);
int  pimax_is_connected(pimax_t *h);
/* Block until connected; 0 on success, -ETIMEDOUT otherwise. */
int  pimax_wait_connected(pimax_t *h, int timeout_ms);

/* Keepalive carries the current video rate (0 = none). */
void pimax_set_video_rate(pimax_t *h, uint8_t fps);

/* Fire and forget: builds `58 LL cmd opt data` (LL = len + 3), zero padded to 64 B. 0 or -errno. */
int pimax_send(pimax_t *h, uint8_t cmd, uint8_t opt, const void *data, size_t len);
/* Send and wait for the `4c LL cmd opt ...` reply on IF0. Reply data (without the trailing check
 * byte) is copied to reply (up to reply_cap). Returns reply data length, or -ETIMEDOUT / -ENOTCONN. */
int pimax_request(pimax_t *h, uint8_t cmd, uint8_t opt, const void *data, size_t len,
                  uint8_t *reply, size_t reply_cap, int timeout_ms);

/* Convenience */
int pimax_read_serial(pimax_t *h, uint8_t opt, char *out, size_t cap, int timeout_ms);  /* cmd 0x16 */
int pimax_read_version(pimax_t *h, uint8_t opt, uint8_t *out, size_t cap, int timeout_ms); /* cmd 0x03 */
/* Per-slot attach burst (0x16, 0x50, 0x0a, 0x03) that starts the EP 0x82 controller reports. */
int pimax_controller_attach(pimax_t *h, uint8_t slot);

/* Video helpers. NOT used by the test suite / pimax-cli: they switch the panel.
 * Sends 5b(02 02), 29(01), 1a(02 FPS), 1a(05 00) and sets the keepalive rate to fps. */
int pimax_video_prepare(pimax_t *h, uint8_t fps);

/* Controller vibration (Rust libpimax only; cmd 0x07 from the Windows capture 2026-10-15:
 * 58 09 07 SS AMP DUR_lo DUR_hi, AMP 1..100 %, DUR ms, no reply). OpenXR-style input: amplitude 0..1, duration in ns
 * (<= 0 = minimum, 12 ms like games), frequency is not supported by the hardware. Queued for the session thread, at
 * most one command per 8 ms per hand, newest wins. 0 = queued, 1 = stop (nothing sent), -EINVAL = bad slot. */
#define PIMAX_HAPTICS 1
int pimax_haptic(pimax_t *h, uint8_t slot, float amplitude, int64_t duration_ns);
/* Test pulse request file $XDG_RUNTIME_DIR/pimax-haptic-request ("SLOT AMP MS"): taken and sent if present.
 * Returns the slot sent, 0 if none, -EINVAL if malformed. */
int pimax_haptic_poll_request(pimax_t *h);
void pimax_haptic_counts(pimax_t *h, uint64_t *sent, uint64_t *replaced);

/* Pure decoders (exposed for tests). Return 0 if the buffer is not that kind of report. */
int pimax_decode_imu(const uint8_t r[PIMAX_REPORT_SIZE], pimax_imu_sample out[2]);
int pimax_decode_controller(const uint8_t r[PIMAX_REPORT_SIZE], pimax_controller_report *out);

#ifdef __cplusplus
}
#endif
#endif
