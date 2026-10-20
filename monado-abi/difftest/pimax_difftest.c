/* SPDX-License-Identifier: MIT
 * Difftest of the libpimax ABI (pimax.h) and the eye ABI (pimax_eye.h), M8: this one program is compiled against
 * omarchyvr's libpimax_capi.a (the Rust oracle) and against PiBridge's libpibridge_pimax.a; run_difftest.sh compares the
 * standard output byte for byte. A fake headset over SOCK_SEQPACKET socketpairs (pimax_open_with_fds, the extension
 * both libraries export) stands in for the hidraw nodes: no hardware is touched. Floats as %a.
 *
 *   pimax_difftest layout                     struct sizes / offsets / constants of pimax.h and pimax_eye.h
 *   pimax_difftest decode FILE.rec.gz...      pimax_decode_imu / _controller over recorded reports + 200000 noise reports
 *   pimax_difftest session WORKDIR            a whole session: wake, rate, IMU, controller, status, log, requests,
 *                                             attach burst, video prepare, haptics, the haptic request file, unplug
 *   pimax_difftest replay FILE.rec.gz IFACE   a recorded report stream into interface IFACE through the session:
 *                                             every callback
 *   pimax_difftest status37 GOLDEN            the 0x37 packets of status37_golden.txt into IF0: the status callbacks,
 *                                             checked against the golden's "slot:battery" lists
 *   pimax_difftest mahony FILE.rec.gz CALIB GOLDEN   p83 through the session, a Mahony filter on the delivered
 *                                             samples (omarchyvr-tools orient.rs), every 1000th quaternion, and the
 *                                             check against mahony_golden.txt (|d| < 1e-9 like the Rust test)
 *   pimax_difftest find SYSROOT DEVROOT       pimax_find_interfaces over a fake sysfs tree
 *   pimax_difftest nodevice SYSROOT           pimax_open on an empty sysfs tree (nothing found, nothing opened)
 *   pimax_difftest close                      pimax_close while the caller holds a lock the state callback needs
 *   pimax_difftest eye WORKDIR                the eye ABI without a chip (PIMAX_EYE_USB = a missing node)
 *   pimax_difftest commands WORKDIR           every command the session wrote, in full (allowlist gate check input)
 */
#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

#include "pimax.h"
#include "pimax_eye.h"

pimax_t *pimax_open_with_fds(const pimax_callbacks *cb, const int read_fds[4], int ctl_write_fd); /* extension */

#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fflush(stdout); exit(1); } } while (0)
#define A(x) __atomic_load_n(&(x), __ATOMIC_ACQUIRE)
#define INC(x) __atomic_add_fetch(&(x), 1, __ATOMIC_RELEASE)

static void msleep(int ms)
{
	struct timespec t = {ms / 1000, (ms % 1000) * 1000000L};
	nanosleep(&t, NULL);
}

static int64_t now_ms(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

/* ---------- callback log: lines collected on the session thread, printed by the main thread ---------- */

static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
static char *g_buf;
static size_t g_len, g_cap;
static int n_state, last_state = -1, n_imu, n_ctrl, n_status, n_log, n_reply;
static int g_quiet_imu; /* mahony: the samples go to the filter, not to the log */
static int g_tag;
static int g_sync; /* sync markers seen (replays): everything sent before them on that interface was handled */
#define SYNC_TS 0xfeedf00du

static void logf_(const char *fmt, ...)
{
	char tmp[2048];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(tmp, sizeof tmp, fmt, ap);
	va_end(ap);
	if (n < 0) return;
	if ((size_t)n >= sizeof tmp) n = sizeof tmp - 1;
	pthread_mutex_lock(&g_mu);
	if (g_len + (size_t)n + 1 > g_cap) {
		g_cap = (g_len + (size_t)n + 1) * 2;
		g_buf = realloc(g_buf, g_cap);
	}
	memcpy(g_buf + g_len, tmp, (size_t)n);
	g_len += (size_t)n;
	g_buf[g_len] = 0;
	pthread_mutex_unlock(&g_mu);
}

static void flush_log(void)
{
	pthread_mutex_lock(&g_mu);
	if (g_len) fwrite(g_buf, 1, g_len, stdout);
	g_len = 0;
	pthread_mutex_unlock(&g_mu);
	fflush(stdout);
}

static void hex(char *out, const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++) sprintf(out + 2 * i, "%02x", p[i]);
	out[2 * n] = 0;
}

/* the Mahony mode's filter input */
static void (*g_imu_sink)(const pimax_imu_sample *s);

static void on_state(void *u, pimax_state s)
{
	CHECK(u == &g_tag);
	logf_("state %d\n", (int)s);
	__atomic_store_n(&last_state, (int)s, __ATOMIC_RELEASE);
	INC(n_state);
}
static void on_imu(void *u, const pimax_imu_sample *s)
{
	CHECK(u == &g_tag);
	if (s->ts_us == SYNC_TS || s->ts_us == SYNC_TS + 1000u) {
		if (s->sample_in_report == 1) INC(g_sync);
		return;
	}
	if (g_imu_sink) g_imu_sink(s);
	if (!g_quiet_imu)
		logf_("imu %u %d a %d,%d,%d g %d,%d,%d %a,%a,%a %a,%a,%a ipd %a\n", s->ts_us, s->sample_in_report,
		      s->raw_accel[0], s->raw_accel[1], s->raw_accel[2], s->raw_gyro[0], s->raw_gyro[1], s->raw_gyro[2],
		      s->accel_g[0], s->accel_g[1], s->accel_g[2], s->gyro_dps[0], s->gyro_dps[1], s->gyro_dps[2], s->ipd_mm);
	INC(n_imu);
}
static void on_ctrl(void *u, const pimax_controller_report *r)
{
	CHECK(u == &g_tag);
	char h[129];
	hex(h, r->raw, 64);
	logf_("ctrl slot %u ts %u stick %d,%d trig %u grip %u bat %u btn %#x touch %#x a %d,%d,%d g %d,%d,%d raw %s\n",
	      r->slot, r->ts_us, r->stick_x, r->stick_y, r->trigger, r->grip, r->battery, r->buttons, r->touch,
	      r->raw_accel[0], r->raw_accel[1], r->raw_accel[2], r->raw_gyro[0], r->raw_gyro[1], r->raw_gyro[2], h);
	INC(n_ctrl);
}
static void on_status(void *u, const pimax_controller_status *s)
{
	CHECK(u == &g_tag);
	char h[129];
	hex(h, s->raw, 64);
	logf_("status slot %u bat %u fw %02x%02x raw %s\n", s->slot, s->battery, s->fw[0], s->fw[1], h);
	INC(n_status);
}
static void on_log(void *u, const char *l)
{
	CHECK(u == &g_tag);
	char h[1100];
	size_t n = strlen(l);
	CHECK(n <= 511);
	hex(h, (const uint8_t *)l, n);
	logf_("log %zu %s\n", n, h);
	INC(n_log);
}
static void on_reply(void *u, const uint8_t r[PIMAX_REPORT_SIZE])
{
	CHECK(u == &g_tag);
	if (r[0] == 0x4c && r[1] == 0x04 && r[2] == 0xee && r[3] == 0xee) {
		INC(g_sync);
		return;
	}
	char h[129];
	hex(h, r, 64);
	logf_("reply %s\n", h);
	INC(n_reply);
}

/* ---------- the fake headset ---------- */

static int dev_if[4], host_if[4], ctl_dev, ctl_host;

static pimax_t *start(int with_cb)
{
	for (int i = 0; i < 4; i++) {
		int sv[2];
		CHECK(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) == 0);
		host_if[i] = sv[0];
		dev_if[i] = sv[1];
	}
	int cv[2];
	CHECK(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, cv) == 0);
	ctl_host = cv[0];
	ctl_dev = cv[1];
	pimax_callbacks cb = {.user = &g_tag, .on_state = on_state, .on_imu = on_imu, .on_controller = on_ctrl,
	                      .on_controller_status = on_status, .on_log = on_log, .on_reply = on_reply};
	pimax_t *h = pimax_open_with_fds(with_cb ? &cb : NULL, host_if, ctl_host);
	CHECK(h);
	int rc = pimax_wait_connected(h, 3000);
	printf("wait_connected %d is_connected %d\n", rc, pimax_is_connected(h));
	return h;
}

static int recv_cmd(uint8_t out[64], int timeout_ms)
{
	struct pollfd p = {.fd = ctl_dev, .events = POLLIN};
	if (poll(&p, 1, timeout_ms) <= 0) return 0;
	uint8_t b[66];
	ssize_t n = recv(ctl_dev, b, sizeof b, MSG_DONTWAIT);
	if (n != 65) {
		printf("command of %zd bytes\n", n);
		return n > 0 ? -1 : 0;
	}
	CHECK(b[0] == 0); /* hidraw report id */
	memcpy(out, b + 1, 64);
	return 1;
}

static const uint8_t WAKE[7] = {0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00};

static int is_wake(const uint8_t *c)
{
	if (memcmp(c, WAKE, 7)) return 0;
	for (int i = 7; i < 64; i++)
		if (c[i]) return 0;
	return 1;
}

static int g_all_commands; /* commands mode: print keepalives and wakes too */
static int g_last_rate = -1;

static void print_cmd(const char *tag, const uint8_t *c)
{
	int n = 64;
	while (n > 4 && c[n - 1] == 0) n--;
	char h[129];
	hex(h, c, (size_t)n);
	printf("%s %s\n", tag, h);
}

/* Every command the session writes within `ms`, keepalives (only their rate when it changes) and wakes left out. */
static void drain_cmds(const char *tag, int ms)
{
	int64_t end = now_ms() + ms;
	for (;;) {
		int64_t left = end - now_ms();
		if (left <= 0) break;
		uint8_t c[64];
		int r = recv_cmd(c, (int)left);
		if (r <= 0) continue;
		if (g_all_commands) {
			print_cmd("cmd", c);
			continue;
		}
		if (c[0] == 0x58 && c[1] == 0x04 && c[2] == 0x39) {
			if (c[4] != g_last_rate) {
				printf("%s keepalive rate %u\n", tag, c[4]);
				g_last_rate = c[4];
			}
			continue;
		}
		if (is_wake(c)) continue;
		print_cmd(tag, c);
	}
	fflush(stdout);
}

#define wait_for(vp, want)                                                                                    \
	do {                                                                                                          \
		for (int i_ = 0; i_ < 5000 && A(*(vp)) < (want); i_++) msleep(1);                                         \
		CHECK(A(*(vp)) >= (want));                                                                                \
	} while (0)

static void send_dev(int iface, const uint8_t *r)
{
	CHECK(send(dev_if[iface], r, 64, 0) == 64);
}

/* ---------- layout ---------- */

#define F(s, f) printf(" %s=%zu", #f, offsetof(s, f))
static int mode_layout(void)
{
	printf("imu size=%zu align=%zu", sizeof(pimax_imu_sample), _Alignof(pimax_imu_sample));
	F(pimax_imu_sample, ts_us); F(pimax_imu_sample, raw_accel); F(pimax_imu_sample, raw_gyro); F(pimax_imu_sample, accel_g);
	F(pimax_imu_sample, gyro_dps); F(pimax_imu_sample, ipd_mm); F(pimax_imu_sample, sample_in_report); printf("\n");
	printf("ctrl size=%zu align=%zu", sizeof(pimax_controller_report), _Alignof(pimax_controller_report));
	F(pimax_controller_report, slot); F(pimax_controller_report, ts_us); F(pimax_controller_report, stick_x);
	F(pimax_controller_report, stick_y); F(pimax_controller_report, trigger); F(pimax_controller_report, grip);
	F(pimax_controller_report, battery); F(pimax_controller_report, buttons); F(pimax_controller_report, touch);
	F(pimax_controller_report, raw_accel); F(pimax_controller_report, raw_gyro); F(pimax_controller_report, raw); printf("\n");
	printf("status size=%zu align=%zu", sizeof(pimax_controller_status), _Alignof(pimax_controller_status));
	F(pimax_controller_status, slot); F(pimax_controller_status, battery); F(pimax_controller_status, fw); F(pimax_controller_status, raw); printf("\n");
	printf("callbacks size=%zu align=%zu", sizeof(pimax_callbacks), _Alignof(pimax_callbacks));
	F(pimax_callbacks, user); F(pimax_callbacks, on_state); F(pimax_callbacks, on_imu); F(pimax_callbacks, on_controller);
	F(pimax_callbacks, on_controller_status); F(pimax_callbacks, on_log); F(pimax_callbacks, on_reply); printf("\n");
	printf("state size=%zu DISCONNECTED=%d CONNECTED=%d\n", sizeof(pimax_state), PIMAX_DISCONNECTED, PIMAX_CONNECTED);
	printf("consts VID=%#x PID=%#x CS=%#x NUM_IF=%d PATH_MAX=%d REPORT=%d MAXDATA=%d L=%d R=%d B=%#x A=%#x STICK=%#x MENU=%#x "
	       "TSTICK=%#x HAPTICS=%d\n",
	       PIMAX_VID, PIMAX_PID, PIMAX_PID_CRYSTAL_SUPER, PIMAX_NUM_IF, PIMAX_PATH_MAX, PIMAX_REPORT_SIZE, PIMAX_MAX_DATA,
	       PIMAX_SLOT_LEFT, PIMAX_SLOT_RIGHT, PIMAX_BTN_B, PIMAX_BTN_A, PIMAX_BTN_STICK, PIMAX_BTN_MENU, PIMAX_TOUCH_STICK,
	       PIMAX_HAPTICS);
	printf("eye_sample size=%zu align=%zu", sizeof(pimax_eye_sample), _Alignof(pimax_eye_sample));
	F(pimax_eye_sample, seq); F(pimax_eye_sample, device_ts_us); F(pimax_eye_sample, host_ts_ns); F(pimax_eye_sample, flags);
	F(pimax_eye_sample, _pad); F(pimax_eye_sample, origin); F(pimax_eye_sample, direction); F(pimax_eye_sample, eye_origin);
	F(pimax_eye_sample, eye_direction); F(pimax_eye_sample, pupil_mm); printf("\n");
	printf("eye abi %u (header %u)\n", pimax_eye_abi_version(), PIMAX_EYE_ABI_VERSION);
	return 0;
}

/* ---------- decode ---------- */

static uint8_t *load_rec(const char *path, size_t *n_out)
{
	gzFile f = gzopen(path, "rb");
	if (!f) {
		printf("cannot open %s\n", path);
		exit(77);
	}
	size_t cap = 1 << 20, len = 0;
	uint8_t *d = malloc(cap);
	for (;;) {
		if (len + 65536 > cap) d = realloc(d, cap *= 2);
		int n = gzread(f, d + len, 65536);
		if (n <= 0) break;
		len += (size_t)n;
	}
	gzclose(f);
	size_t recs = len / 72;
	uint8_t *out = malloc(recs * 64 + 1);
	for (size_t i = 0; i < recs; i++) memcpy(out + 64 * i, d + 72 * i + 8, 64);
	free(d);
	*n_out = recs;
	return out;
}

static void decode_one(unsigned long n, const uint8_t *r)
{
	pimax_imu_sample s[2];
	pimax_controller_report c;
	memset(s, 0, sizeof s);
	memset(&c, 0, sizeof c);
	int i = pimax_decode_imu(r, s);
	int k = pimax_decode_controller(r, &c);
	if (i)
		for (int j = 0; j < 2; j++)
			printf("%lu I %u %d %d,%d,%d %d,%d,%d %a,%a,%a %a,%a,%a %a\n", n, s[j].ts_us, s[j].sample_in_report,
			       s[j].raw_accel[0], s[j].raw_accel[1], s[j].raw_accel[2], s[j].raw_gyro[0], s[j].raw_gyro[1],
			       s[j].raw_gyro[2], s[j].accel_g[0], s[j].accel_g[1], s[j].accel_g[2], s[j].gyro_dps[0],
			       s[j].gyro_dps[1], s[j].gyro_dps[2], s[j].ipd_mm);
	if (k)
		printf("%lu C %u %u %d %d %u %u %u %u %u %d,%d,%d %d,%d,%d rawok=%d\n", n, c.slot, c.ts_us, c.stick_x, c.stick_y,
		       c.trigger, c.grip, c.battery, c.buttons, c.touch, c.raw_accel[0], c.raw_accel[1], c.raw_accel[2],
		       c.raw_gyro[0], c.raw_gyro[1], c.raw_gyro[2], memcmp(c.raw, r, 64) == 0);
	if (!i && !k) printf("%lu -\n", n);
}

static int mode_decode(int argc, char **argv)
{
	unsigned long n = 0;
	for (int a = 0; a < argc; a++) {
		size_t recs;
		uint8_t *d = load_rec(argv[a], &recs);
		for (size_t i = 0; i < recs; i++) decode_one(n++, d + 64 * i);
		free(d);
	}
	/* deterministic pseudo-random noise, half of it with valid magics (the Rust abi.rs test's generator) */
	uint64_t x = 0x2545F4914F6CDD1Dull;
	for (int i = 0; i < 200000; i++) {
		uint8_t r[64];
		for (int b = 0; b < 64; b++) {
			x ^= x << 13;
			x ^= x >> 7;
			x ^= x << 17;
			r[b] = (uint8_t)x;
		}
		if (i % 4 == 0) {
			memcpy(r, "\x53\x31\x08\x00", 4);
		} else if (i % 4 == 1) {
			memcpy(r, "\x53\x32\x08", 3);
			r[3] = (uint8_t)(2 + (r[3] & 1));
		}
		decode_one(n++, r);
	}
	printf("%lu reports\n", n);
	return 0;
}

/* ---------- session ---------- */

static int stop_responder;

static void *responder(void *arg)
{
	(void)arg;
	while (!A(stop_responder)) {
		uint8_t c[64], r[64] = {0};
		if (recv_cmd(c, 20) <= 0) continue;
		if (c[0] == 0x58 && c[2] == 0x16 && c[3] == 0) {
			/* noise first: a 0x37 and an unrelated reply must not satisfy the request */
			uint8_t st[64] = {0x4c, 0x0f, 0x37, 0x02, 0x48, 1, 2, 3, 4, 0x49, 0, 2, 1, 0, 8, 0x2d};
			send_dev(0, st);
			uint8_t other[64] = {0x4c, 0x04, 0x51, 0x00};
			send_dev(0, other);
			memcpy(r, "\x4c\x1a\x16\x00\x05H1234", 10);
			send_dev(0, r);
		} else if (c[0] == 0x58 && c[2] == 0x16 && c[3] == 2) {
			/* a reply that claims more bytes than it has */
			memcpy(r, "\x4c\x05\x16\x02\x3fXY", 7);
			send_dev(0, r);
		} else if (c[0] == 0x58 && c[2] == 0x03) {
			memcpy(r, "\x4c\x07\x03\x00\x00\x08\x2d\x6b", 8);
			r[3] = c[3];
			send_dev(0, r);
		}
	}
	return NULL;
}

static void write_file(const char *path, const char *s, size_t n)
{
	FILE *f = fopen(path, "wb");
	CHECK(f);
	fwrite(s, 1, n, f);
	fclose(f);
}

static int mode_session(const char *work)
{
	pimax_t *h = start(1);
	flush_log();
	uint8_t c[64];
	CHECK(recv_cmd(c, 1000) == 1);
	print_cmd("first", c);
	pimax_set_video_rate(h, 72);
	int saw72 = 0;
	for (int i = 0; i < 12 && !saw72; i++)
		if (recv_cmd(c, 400) == 1 && c[2] == 0x39 && c[4] == 72) saw72 = 1;
	printf("rate 72 keepalive seen %d\n", saw72);
	g_last_rate = 72;

	/* IMU: ts=5_000_000, sample 0: acc (2048,-1024,0) gyro (164,0,-164), ipd 65.90; sample 1 odd values */
	uint8_t r[64] = {0x53, 0x31, 0x08, 0x00};
	uint32_t ts = 5000000;
	memcpy(r + 4, &ts, 4);
	r[14] = 0x08; r[15] = 0x00; r[16] = 0xfc; r[17] = 0x00; r[20] = 0x00; r[21] = 0xa4; r[24] = 0xff; r[25] = 0x5c;
	r[30] = 0x80; r[31] = 0x00; r[32] = 0x7f; r[33] = 0xff; r[40] = 0x12; r[41] = 0x34;
	r[45] = 6590 & 255; r[46] = 6590 >> 8;
	send_dev(2, r);
	wait_for(&n_imu, 2);
	/* the same report on IF1 (both read interfaces deliver IMU), and a wrapped clock */
	ts = 0xffffff00u;
	memcpy(r + 4, &ts, 4);
	send_dev(1, r);
	wait_for(&n_imu, 4);
	flush_log();

	/* controllers on IF1: right and left, charging bit, odd sticks, every button */
	uint8_t cr[64] = {0x53, 0x32, 0x08, 0x03, 0x43, 255, 1, 200};
	cr[44] = 99; cr[46] = 0xda; cr[47] = 0x0a; cr[48] = 1; cr[49] = 0x08;
	send_dev(1, cr);
	wait_for(&n_ctrl, 1);
	uint8_t cl[64] = {0x53, 0x32, 0x08, 0x02, 0x10, 0x80, 0x7f, 0x00, 0x11, 0x22, 0x33, 0x44};
	for (int i = 12; i < 44; i++) cl[i] = (uint8_t)(i * 37);
	cl[44] = 0; cl[45] = 0xff; cl[46] = 0x63; cl[47] = 0xff; cl[48] = 0xff; cl[49] = 0xff;
	send_dev(2, cl);
	wait_for(&n_ctrl, 2);
	flush_log();

	/* status heartbeats on IF0: one message, two messages */
	uint8_t st[64] = {0x4c, 0x0f, 0x37, 0x02, 0x48, 1, 2, 3, 4, 0x49, 0, 2, 1, 0, 0x08, 0x2d};
	send_dev(0, st);
	wait_for(&n_status, 1);
	uint8_t st2[64] = {0x4c, 0x0f, 0x37, 0x02, 0x48, 0xe8, 0xb6, 0x91, 0x02, 0x49, 0x00, 0x02, 0x01, 0x00, 0x08, 0x2d,
	                   0xc7, 0x0f, 0x37, 0x03, 0x48, 0x46, 0xa7, 0x91, 0x02, 0x63, 0x00, 0x02, 0x01, 0x00, 0x08, 0x2e, 0x38};
	send_dev(0, st2);
	wait_for(&n_status, 3);
	flush_log();

	/* firmware log on IF3: a line, two lines in one report, a line over two reports, invalid UTF-8, a long line */
	uint8_t lg[64] = "hello fw\r\n";
	send_dev(3, lg);
	wait_for(&n_log, 1);
	uint8_t lg2[64] = "one\ntwo\npart";
	send_dev(3, lg2);
	wait_for(&n_log, 3);
	uint8_t lg3[64] = " of three\n\xff\xfe bad \xc3\n";
	send_dev(3, lg3);
	wait_for(&n_log, 5);
	for (int k = 0; k < 10; k++) {
		uint8_t lp[64];
		memset(lp, 'a' + k, 64);
		if (k == 9) lp[63] = '\n';
		send_dev(3, lp);
	}
	wait_for(&n_log, 6);
	msleep(30);
	flush_log();

	/* requests */
	pthread_t th;
	pthread_create(&th, NULL, responder, NULL);
	char ser[32];
	int n = pimax_read_serial(h, 0, ser, sizeof ser, 1000);
	printf("serial %d '%s'\n", n, n >= 0 ? ser : "");
	memset(ser, 'x', sizeof ser);
	n = pimax_read_serial(h, 0, ser, 4, 1000);
	printf("serial cap4 %d '%s'\n", n, n >= 0 ? ser : "");
	n = pimax_read_serial(h, 0, ser, 1, 1000);
	printf("serial cap1 %d '%s'\n", n, n >= 0 ? ser : "");
	n = pimax_read_serial(h, 0, NULL, 0, 1000);
	printf("serial cap0 %d\n", n);
	n = pimax_read_serial(h, 2, ser, sizeof ser, 1000);
	printf("serial slot2 %d '%s'\n", n, n >= 0 ? ser : "");
	uint8_t ver[64];
	for (int slot = 0; slot < 4; slot += 2) {
		memset(ver, 0, sizeof ver);
		n = pimax_read_version(h, (uint8_t)slot, ver, sizeof ver, 1000);
		printf("version %d: %d %02x %02x %02x %02x\n", slot, n, ver[0], ver[1], ver[2], ver[3]);
	}
	n = pimax_read_version(h, 3, ver, 2, 1000);
	printf("version cap2 %d %02x %02x %02x\n", n, ver[0], ver[1], ver[2]);
	uint8_t rep[64];
	memset(rep, 0, sizeof rep);
	n = pimax_request(h, 0x16, 0, (const uint8_t[]){0x1e}, 1, rep, sizeof rep, 1000);
	char hx[129];
	hex(hx, rep, 24);
	printf("request %d %s\n", n, hx);
	n = pimax_request(h, 0x16, 0, (const uint8_t[]){0x1e}, 1, NULL, 0, 1000);
	printf("request no buffer %d\n", n);
	n = pimax_request(h, 0x50, 0, NULL, 0, rep, sizeof rep, 120); /* allowlisted, never answered */
	printf("request unanswered %d\n", n);
	n = pimax_request(h, 0x50, 0, NULL, 0, rep, sizeof rep, -5);
	printf("request negative timeout %d\n", n);
	uint8_t big[61] = {0};
	printf("send 61 bytes %d\n", pimax_send(h, 0x16, 0, big, sizeof big));
	printf("request 61 bytes %d\n", pimax_request(h, 0x16, 0, big, sizeof big, rep, sizeof rep, 100));
	__atomic_store_n(&stop_responder, 1, __ATOMIC_RELEASE);
	pthread_join(th, NULL);
	/* after the responder: it would read these commands itself */
	printf("send read_info %d\n", pimax_send(h, 0x50, 3, NULL, 0));
	printf("send read_info with NULL data len 4 %d\n", pimax_send(h, 0x50, 2, NULL, 4));
	flush_log();
	drain_cmds("after requests", 60);

	/* attach burst and video prepare wire forms */
	printf("attach left %d\n", pimax_controller_attach(h, PIMAX_SLOT_LEFT));
	drain_cmds("attach left", 120);
	printf("attach right %d\n", pimax_controller_attach(h, PIMAX_SLOT_RIGHT));
	drain_cmds("attach right", 120);
	printf("video_prepare 90 %d\n", pimax_video_prepare(h, 90));
	drain_cmds("video 90", 500);
	printf("video_prepare 72 %d\n", pimax_video_prepare(h, 72));
	drain_cmds("video 72", 500);

	/* haptics: OpenXR values to commands, one pulse at a time (more than 8 ms apart: nothing coalesces) */
	struct {
		int slot;
		float amp;
		int64_t ns;
	} hp[] = {
		{3, 0.25f, 300000000}, {2, 1.0f, -1}, {2, 0.004f, 1}, {3, 0.005f, 999999}, {3, 2.0f, 1000000000000LL},
		{2, NAN, 5000000}, {2, -0.5f, 5000000}, {2, 0.0f, 5000000}, {4, 0.5f, 5000000}, {1, 0.0f, 0},
		{3, 0.999f, 1000001}, {3, 0.5f, INT64_MAX}, {3, 1e-9f, 12000000}, {2, INFINITY, 12000000},
		{2, -INFINITY, 12000000}, {0, 0.5f, 1}, {3, 0.125f, 65535000000LL}, {3, 0.995f, 65536000000LL},
		{2, 0.635f, 11999999}, {255, 1.0f, 1},
	};
	for (size_t i = 0; i < sizeof hp / sizeof hp[0]; i++) {
		int rc = pimax_haptic(h, (uint8_t)hp[i].slot, hp[i].amp, hp[i].ns);
		printf("haptic %zu: slot %d amp %a ns %lld -> %d\n", i, hp[i].slot, (double)hp[i].amp, (long long)hp[i].ns, rc);
		drain_cmds("  sent", 25);
	}
	uint64_t sent = 99, replaced = 99;
	pimax_haptic_counts(h, &sent, &replaced);
	printf("haptic counts %llu %llu\n", (unsigned long long)sent, (unsigned long long)replaced);
	pimax_haptic_counts(h, NULL, NULL);
	/* within the 8 ms gap after a pulse the next one waits: a newer one replaces it, a stop drops it */
	int r1 = pimax_haptic(h, 3, 0.5f, 12000000);
	msleep(2);
	int r2 = pimax_haptic(h, 3, 0.6f, 12000000);
	int r3 = pimax_haptic(h, 3, 0.7f, 12000000);
	int r4 = pimax_haptic(h, 3, 0.0f, 0);
	printf("haptic, wait, replace, stop %d %d %d %d\n", r1, r2, r3, r4);
	drain_cmds("  sent", 40);

	/* the test pulse request file ($XDG_RUNTIME_DIR = WORKDIR) */
	char req[600];
	snprintf(req, sizeof req, "%s/pimax-haptic-request", work);
	printf("poll none %d\n", pimax_haptic_poll_request(h));
	const char *reqs[] = {"3 25 300\n", "2 40 200", "junk", "1 25 300", "3 25 300 x", "+2 +5 +9\n", "\xe3\x80\x80" "3\xc2\xa0" "7 8",
	                      "", "3 101 300", "3 25 0", "2 100 65535", "3 25 65536", "2\t1\t1\r\n", "3 25 300\n4"};
	for (size_t i = 0; i < sizeof reqs / sizeof reqs[0]; i++) {
		write_file(req, reqs[i], strlen(reqs[i]));
		int rc = pimax_haptic_poll_request(h);
		printf("poll %zu -> %d, file left %d\n", i, rc, access(req, F_OK) == 0);
		unlink(req);
		drain_cmds("  sent", 25);
	}
	write_file(req, "3 25 \xff\n", 7);
	printf("poll not utf-8 -> %d, file left %d\n", pimax_haptic_poll_request(h), access(req, F_OK) == 0);
	unlink(req);
	CHECK(mkdir(req, 0700) == 0);
	printf("poll directory -> %d, left %d\n", pimax_haptic_poll_request(h), access(req, F_OK) == 0);
	rmdir(req);
	drain_cmds("  sent", 25);
	pimax_haptic_counts(h, &sent, &replaced);
	printf("haptic counts %llu %llu\n", (unsigned long long)sent, (unsigned long long)replaced);
	flush_log();

	/* unplug: all device ends closed */
	for (int i = 0; i < 4; i++) close(dev_if[i]);
	close(ctl_dev);
	for (int i = 0; i < 3000 && A(last_state) == PIMAX_CONNECTED; i++) msleep(1);
	msleep(20);
	flush_log();
	printf("after unplug: connected %d\n", pimax_is_connected(h));
	printf("send %d\n", pimax_send(h, 3, 0, NULL, 0));
	printf("request %d\n", pimax_request(h, 3, 0, NULL, 0, rep, sizeof rep, 100));
	printf("serial %d\n", pimax_read_serial(h, 0, ser, sizeof ser, 100));
	printf("attach %d\n", pimax_controller_attach(h, 2));
	printf("video_prepare %d\n", pimax_video_prepare(h, 90));
	printf("haptic %d\n", pimax_haptic(h, 2, 0.5f, 1000000));
	printf("wait_connected %d\n", pimax_wait_connected(h, 50));
	pimax_close(h);
	flush_log();
	printf("closed\n");
	return 0;
}

/* ---------- replays ---------- */

static void sync_iface(int iface)
{
	int before = A(g_sync);
	uint8_t m[64] = {0};
	if (iface == 0) {
		memcpy(m, "\x4c\x04\xee\xee", 4);
	} else {
		uint32_t ts = SYNC_TS;
		memcpy(m, "\x53\x31\x08\x00", 4);
		memcpy(m + 4, &ts, 4);
	}
	send_dev(iface, m);
	wait_for(&g_sync, before + 1);
}

static int mode_replay(const char *file, int iface)
{
	size_t recs;
	uint8_t *d = load_rec(file, &recs);
	pimax_t *h = start(1);
	flush_log();
	int imu0 = A(n_imu), ctrl0 = A(n_ctrl), st0 = A(n_status), log0 = A(n_log), rep0 = A(n_reply);
	for (size_t i = 0; i < recs; i++) {
		send_dev(iface, d + 64 * i); /* blocking: the session drains the socket as the hidraw node would */
		if (i % 4096 == 4095) flush_log();
	}
	if (iface == 3) {
		/* the log interface has no marker: wait until no line arrives for 300 ms */
		int last = -1;
		for (int k = 0; k < 100 && A(n_log) != last; k++) {
			last = A(n_log);
			msleep(300);
		}
	} else {
		sync_iface(iface);
	}
	flush_log();
	printf("records %zu imu %d ctrl %d status %d log %d reply %d\n", recs, A(n_imu) - imu0, A(n_ctrl) - ctrl0,
	       A(n_status) - st0, A(n_log) - log0, A(n_reply) - rep0);
	pimax_close(h);
	free(d);
	return 0;
}

static int mode_status37(const char *golden)
{
	FILE *f = fopen(golden, "r");
	if (!f) {
		printf("skipped: %s missing\n", golden);
		return 77;
	}
	pimax_t *h = start(1);
	flush_log();
	char line[512];
	int packets = 0, ok = 0;
	while (fgets(line, sizeof line, f)) {
		char *sp = strchr(line, ' ');
		if (!sp || sp - line < 128) continue;
		uint8_t p[64];
		for (int i = 0; i < 64; i++) {
			unsigned v;
			sscanf(line + 2 * i, "%2x", &v);
			p[i] = (uint8_t)v;
		}
		char want[64];
		snprintf(want, sizeof want, "%s", sp + 1);
		want[strcspn(want, "\r\n")] = 0;
		int before = A(n_status);
		send_dev(0, p);
		sync_iface(0);
		int got = A(n_status) - before;
		/* the golden lists one "slot:battery" per message */
		int expect = 1;
		for (char *q = want; *q; q++) expect += *q == ';';
		ok += got == expect;
		packets++;
		printf("packet %d: %d status callbacks, golden %s\n", packets, got, want);
		flush_log();
	}
	fclose(f);
	printf("%d packets, %d with the golden's message count\n", packets, ok);
	pimax_close(h);
	return 0;
}

/* omarchyvr-tools/src/orient.rs (Mahony + factory IMU terms), on the samples the session delivers */
typedef struct {
	double a_bias[3], w_bias[3], ka[3], kg[3];
} calib_t;
static calib_t g_cal;
static double g_q[4] = {1, 0, 0, 0};
static int g_init, g_count;
static uint64_t g_prev, g_wraps;
static int64_t g_last_ts = -1;
static double g_gold[256][8];
static int g_ngold, g_checked;
static double g_worst;
static const double D2R = 3.14159265358979323846 / 180.0;
static const double G = 9.80665;

static int attr3(const char *xml, const char *key, double out[3])
{
	char pat[32];
	snprintf(pat, sizeof pat, "%s=\"", key);
	const char *p = strstr(xml, pat);
	if (!p) return 0;
	p += strlen(pat);
	return sscanf(p, "%lf %lf %lf", &out[0], &out[1], &out[2]) == 3;
}

static void seed(const double ac[3])
{
	double n = sqrt(ac[0] * ac[0] + ac[1] * ac[1] + ac[2] * ac[2]);
	if (n == 0.0) n = 1.0;
	double ux = ac[0] / n, uy = ac[1] / n, uz = ac[2] / n;
	double w = 1.0 + uy, qx = -uz, qz = ux;
	double m = sqrt(w * w + qx * qx + qz * qz);
	if (m == 0.0) m = 1.0;
	g_q[0] = w / m; g_q[1] = qx / m; g_q[2] = 0.0; g_q[3] = qz / m;
}

static void update(const double g[3], const double a[3], double dt)
{
	double q0 = g_q[0], q1 = g_q[1], q2 = g_q[2], q3 = g_q[3];
	double n = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	double ex = 0, ey = 0, ez = 0;
	if (0.7 < n && n < 1.3 && fabs(g[0]) + fabs(g[1]) + fabs(g[2]) < 150.0) {
		double ax = a[0] / n, ay = a[1] / n, az = a[2] / n;
		double vx = 2.0 * (q1 * q2 + q0 * q3);
		double vy = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
		double vz = 2.0 * (q2 * q3 - q0 * q1);
		ex = ay * vz - az * vy;
		ey = az * vx - ax * vz;
		ez = ax * vy - ay * vx;
	}
	double gx = g[0] * D2R + 1.0 * ex, gy = g[1] * D2R + 1.0 * ey, gz = g[2] * D2R + 1.0 * ez;
	double hh = 0.5 * dt;
	double q[4] = {q0 + hh * (-q1 * gx - q2 * gy - q3 * gz), q1 + hh * (q0 * gx + q2 * gz - q3 * gy),
	               q2 + hh * (q0 * gy - q1 * gz + q3 * gx), q3 + hh * (q0 * gz + q1 * gy - q2 * gx)};
	double m = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int i = 0; i < 4; i++) g_q[i] = q[i] / m;
}

static void euler(double *yaw, double *pitch, double *roll)
{
	double w = g_q[0], x = g_q[1], y = g_q[2], z = g_q[3];
	double r1 = 2.0 * (x * y + w * z);
	double u1 = 1.0 - 2.0 * (x * x + z * z);
	double b[3] = {2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)};
	double f[3] = {-b[0], -b[1], -b[2]};
	*yaw = atan2(-f[0], -f[2]) * (180.0 / 3.14159265358979323846);
	double fy = f[1] < -1.0 ? -1.0 : f[1] > 1.0 ? 1.0 : f[1];
	*pitch = asin(fy) * (180.0 / 3.14159265358979323846);
	*roll = atan2(-r1, u1) * (180.0 / 3.14159265358979323846);
}

static void mahony_sink(const pimax_imu_sample *s)
{
	/* orient.rs Unwrap: the 32-bit clock unwrapped; the second sample of a report is ts + 1000 */
	if (s->sample_in_report == 0) {
		if (g_last_ts >= 0 && (int64_t)s->ts_us < g_last_ts - (1LL << 31)) g_wraps++;
		g_last_ts = s->ts_us;
	}
	uint64_t t = (uint64_t)(uint32_t)g_last_ts + (g_wraps << 32) + 1000u * (uint64_t)s->sample_in_report;
	double acc[3], gyr[3], a[3], w[3];
	for (int i = 0; i < 3; i++) {
		acc[i] = (double)s->raw_accel[i] / 2048.0;
		gyr[i] = (double)s->raw_gyro[i] / 16.4;
		a[i] = (acc[i] * G - g_cal.a_bias[i]) / (1.0 + g_cal.ka[i]) / G;
		w[i] = (gyr[i] * D2R - g_cal.w_bias[i]) / (1.0 + g_cal.kg[i]) / D2R;
	}
	if (!g_init) {
		seed(a);
		g_init = 1;
		g_prev = t;
		return;
	}
	double dt = (double)(t - g_prev) * 1e-6;
	g_prev = t;
	if (!(0.0 < dt && dt < 0.05)) return;
	update(w, a, dt);
	g_count++;
	if (g_count % 1000 == 0) {
		double yw, pt, rl;
		euler(&yw, &pt, &rl);
		logf_("%d %a %a %a %a\n", g_count, g_q[0], g_q[1], g_q[2], g_q[3]);
		int k = g_count / 1000 - 1;
		if (k < g_ngold && (int)g_gold[k][0] == g_count) {
			for (int i = 0; i < 4; i++) {
				double dd = fabs(g_q[i] - g_gold[k][1 + i]);
				if (dd > g_worst) g_worst = dd;
			}
			double e[3] = {yw, pt, rl};
			for (int i = 0; i < 3; i++) {
				double dd = fabs(e[i] - g_gold[k][5 + i]) * 1e-3;
				if (dd > g_worst) g_worst = dd;
			}
			g_checked++;
		}
	}
}

static int mode_mahony(const char *file, const char *calib, const char *golden)
{
	FILE *f = fopen(calib, "r");
	if (!f) {
		printf("skipped: %s missing\n", calib);
		return 77;
	}
	static char xml[1 << 20];
	size_t n = fread(xml, 1, sizeof xml - 1, f);
	xml[n] = 0;
	fclose(f);
	CHECK(attr3(xml, "aBias", g_cal.a_bias) && attr3(xml, "wBias", g_cal.w_bias) && attr3(xml, "ka", g_cal.ka) &&
	      attr3(xml, "kg", g_cal.kg));
	FILE *gf = fopen(golden, "r");
	if (gf) {
		char line[512];
		while (g_ngold < 256 && fgets(line, sizeof line, gf)) {
			double *v = g_gold[g_ngold];
			if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) == 8)
				g_ngold++;
		}
		fclose(gf);
	}
	g_quiet_imu = 1;
	g_imu_sink = mahony_sink;
	int rc = mode_replay(file, 2);
	printf("golden: %d of %d lines checked, max deviation %s 1e-9\n", g_checked, g_ngold, g_worst < 1e-9 ? "<" : ">=");
	return rc;
}

/* ---------- discovery, no device, close ---------- */

static int mode_find(const char *sys, const char *dev)
{
	setenv("PIMAX_SYSFS_ROOT", sys, 1);
	setenv("PIBRIDGE_SYSFS_ROOT", sys, 1);
	setenv("PIMAX_DEV_ROOT", dev, 1);
	setenv("PIBRIDGE_DEV_ROOT", dev, 1);
	char paths[PIMAX_NUM_IF][PIMAX_PATH_MAX];
	memset(paths, 'x', sizeof paths);
	int m = pimax_find_interfaces(paths);
	printf("mask %#x\n", m);
	for (int i = 0; i < PIMAX_NUM_IF; i++)
		if (m & (1 << i)) printf("if%d %zu '%s'\n", i, strlen(paths[i]), paths[i]);
	printf("mask without buffer %#x\n", pimax_find_interfaces(NULL));
	return 0;
}

static int mode_nodevice(const char *sys)
{
	/* never the real /sys: the session would open the user's headset */
	setenv("PIMAX_SYSFS_ROOT", sys, 1);
	setenv("PIBRIDGE_SYSFS_ROOT", sys, 1);
	setenv("PIMAX_DEV_ROOT", sys, 1);
	setenv("PIBRIDGE_DEV_ROOT", sys, 1);
	pimax_t *h = pimax_open(NULL);
	CHECK(h);
	printf("connected %d\n", pimax_is_connected(h));
	printf("wait %d\n", pimax_wait_connected(h, 200));
	printf("send %d\n", pimax_send(h, 3, 0, NULL, 0));
	uint8_t rep[8];
	printf("request %d\n", pimax_request(h, 3, 0, NULL, 0, rep, sizeof rep, 50));
	printf("haptic %d\n", pimax_haptic(h, 3, 0.5f, 1000));
	printf("poll %d\n", pimax_haptic_poll_request(h));
	int64_t t = now_ms();
	pimax_close(h);
	printf("close prompt %d\n", now_ms() - t < 500);
	return 0;
}

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static void on_state_locking(void *u, pimax_state s)
{
	(void)u;
	if (s == PIMAX_DISCONNECTED) {
		pthread_mutex_lock(&g_lock); /* a disconnect during close */
		pthread_mutex_unlock(&g_lock);
	}
}

static int mode_close(void)
{
	int sv[4][2];
	for (int i = 0; i < 4; i++) CHECK(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv[i]) == 0);
	int reads[4] = {sv[0][0], sv[1][0], sv[2][0], sv[3][0]};
	pimax_callbacks cb = {.on_state = on_state_locking};
	pimax_t *h = pimax_open_with_fds(&cb, reads, sv[0][1]);
	CHECK(h);
	printf("wait %d\n", pimax_wait_connected(h, 2000));
	pthread_mutex_lock(&g_lock);
	int64_t t = now_ms();
	pimax_close(h);
	int prompt = now_ms() - t < 500;
	pthread_mutex_unlock(&g_lock);
	printf("close prompt %d\n", prompt);
	/* NULL callbacks: a session without any */
	int sw[4][2];
	for (int i = 0; i < 4; i++) CHECK(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sw[i]) == 0);
	int r2[4] = {sw[0][0], -1, sw[2][0], -1};
	h = pimax_open_with_fds(NULL, r2, sw[1][1]);
	CHECK(h);
	printf("no callbacks: wait %d\n", pimax_wait_connected(h, 2000));
	uint8_t imu[64] = {0x53, 0x31, 0x08, 0x00};
	CHECK(send(sw[2][1], imu, 64, 0) == 64);
	msleep(20);
	pimax_close(h);
	pimax_close(NULL);
	printf("closed\n");
	return 0;
}

/* ---------- eye ---------- */

static void print_eye(const char *tag, int rc, const pimax_eye_sample *s)
{
	printf("%s %d seq %llu flags %u\n", tag, rc, (unsigned long long)s->seq, s->flags);
}

static int mode_eye(const char *work)
{
	/* no chip: the tracker is pointed at a node that does not exist (nothing on USB is opened), no power-on */
	char node[600];
	snprintf(node, sizeof node, "%s/no-such-usb-node", work);
	setenv("PIMAX_EYE_USB", node, 1);
	setenv("PIBRIDGE_EYE_USB", node, 1);
	setenv("PIMAX_EYE_POWER_ON", "0", 1);
	setenv("PIBRIDGE_EYE_POWER_ON", "0", 1);
	setenv("PIMAX_EYE_USB_RESET", "0", 1);
	setenv("PIBRIDGE_EYE_USB_RESET", "0", 1);
	printf("abi %u\n", pimax_eye_abi_version());
	pimax_eye_sample s;
	memset(&s, 0, sizeof s);
	const float dir[3] = {0, 0, -1};
	printf("NULL: latest %d sample_at %d state %d observe %d %d\n", pimax_eye_latest(NULL, &s),
	       pimax_eye_sample_at(NULL, 0, &s), pimax_eye_state(NULL), pimax_eye_observe_target(NULL, 0, dir),
	       pimax_eye_observe_target(NULL, 0, NULL));
	pimax_eye_stop(NULL);
	for (uint32_t flags = 0; flags < 2; flags++) {
		pimax_eye *e = pimax_eye_start(flags);
		CHECK(e);
		msleep(100);
		int st = pimax_eye_state(e);
		printf("start %u: state searching/connecting/reconnecting %d\n", flags, st == 0 || st == 1 || st == 3);
		print_eye("latest", pimax_eye_latest(e, &s), &s);
		print_eye("sample_at", pimax_eye_sample_at(e, 123456789, &s), &s);
		printf("latest NULL out %d, sample_at NULL out %d\n", pimax_eye_latest(e, NULL), pimax_eye_sample_at(e, 0, NULL));
		printf("observe %d, NULL dir %d\n", pimax_eye_observe_target(e, 0, dir), pimax_eye_observe_target(e, 0, NULL));
		pimax_eye_stop(e);
		printf("stopped\n");
	}
	return 0;
}

/* ---------- commands: the full wire of a session (input of the allowlist gate check) ---------- */

static int mode_commands(const char *work)
{
	g_all_commands = 1;
	pimax_t *h = start(0);
	drain_cmds("", 50);
	pimax_set_video_rate(h, 90);
	drain_cmds("", 250);
	pimax_controller_attach(h, 2);
	pimax_controller_attach(h, 3);
	pimax_video_prepare(h, 90);
	pimax_video_prepare(h, 72);
	pimax_set_video_rate(h, 0);
	drain_cmds("", 250);
	for (int slot = 2; slot <= 3; slot++) {
		pimax_haptic(h, (uint8_t)slot, 0.25f, 300000000);
		drain_cmds("", 20);
		pimax_haptic(h, (uint8_t)slot, 1.0f, -1);
		drain_cmds("", 20);
	}
	char req[600];
	snprintf(req, sizeof req, "%s/pimax-haptic-request", work);
	write_file(req, "2 40 200\n", 9);
	pimax_haptic_poll_request(h);
	drain_cmds("", 20);
	pimax_close(h);
	return 0;
}

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IOFBF, 1 << 16);
	if (argc < 2) {
		fprintf(stderr, "usage: pimax_difftest MODE ...\n");
		return 2;
	}
	const char *m = argv[1];
	if (!strcmp(m, "layout")) return mode_layout();
	if (!strcmp(m, "decode")) return mode_decode(argc - 2, argv + 2);
	if (!strcmp(m, "session") && argc > 2) return mode_session(argv[2]);
	if (!strcmp(m, "replay") && argc > 3) return mode_replay(argv[2], atoi(argv[3]));
	if (!strcmp(m, "status37") && argc > 2) return mode_status37(argv[2]);
	if (!strcmp(m, "mahony") && argc > 4) return mode_mahony(argv[2], argv[3], argv[4]);
	if (!strcmp(m, "find") && argc > 3) return mode_find(argv[2], argv[3]);
	if (!strcmp(m, "nodevice") && argc > 2) return mode_nodevice(argv[2]);
	if (!strcmp(m, "close")) return mode_close();
	if (!strcmp(m, "eye") && argc > 2) return mode_eye(argv[2]);
	if (!strcmp(m, "commands") && argc > 2) return mode_commands(argv[2]);
	fprintf(stderr, "bad mode %s\n", m);
	return 2;
}
