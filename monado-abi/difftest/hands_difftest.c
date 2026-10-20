/* SPDX-License-Identifier: MIT
 * Difftest of the controller ABIs (pimax_hands.h, pimax_ctrlfuse.h, pimax_inhand.h, M6): this one program is compiled
 * against omarchyvr's libpimax_hands.a (the Rust oracle) and against PiBridge's libpibridge_pimax.a; run_difftest.sh
 * compares the standard output byte for byte. Every float is printed exactly (%a), long sequences as FNV-1a hashes of
 * the raw output structs plus a full line every so often.
 *
 *   hands_difftest layout
 *   hands_difftest hands SEED N                 synthetic + garbage input through every pimax_hands_* entry point
 *   hands_difftest poses                        controller_poses / aim_to_grip / describe (env PIMAX_CTRL_REST/AIM)
 *   hands_difftest replay FILE.rec.gz           a recorded report stream ([f64 time][64 B] records) through hands,
 *                                               ctrlfuse (report) and inhand (report)
 *   hands_difftest inhand_csv FILE.csv.gz MODE  an in-hand IMU fixture (dt_us,gx,gy,gz,ax,ay,az) through
 *                                               pimax_inhand_report; MODE: free | picked | half | typing
 *   hands_difftest inhand_synth SEED WORKDIR    synthetic in-hand session, publish (status file), poll_mode, describe
 *   hands_difftest fuse SCENARIO SEED SECONDS   synthetic optical + IMU session through pimax_ctrlfuse_*
 *   hands_difftest fuse_nulls                   NULL handling and odd arguments of pimax_ctrlfuse_*
 *   hands_difftest session                      tests/c/session.c of pimax-hands (prints PASS)
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <zlib.h>

#include "pimax_ctrlfuse.h"
#include "pimax_hands.h"
#include "pimax_inhand.h"

static uint64_t g_rng = 1;
static uint64_t rnd(void)
{
	g_rng = g_rng * 6364136223846793005ull + 1442695040888963407ull;
	return g_rng >> 11;
}
/* uniform in [-1, 1] */
static double rndf(void)
{
	return (double)rnd() / (double)(1ull << 53) * 2.0 - 1.0;
}
static uint64_t rndn(uint64_t n)
{
	return n ? rnd() % n : 0;
}

static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
	const unsigned char *b = p;
	for (size_t i = 0; i < n; i++) {
		h ^= b[i];
		h *= 1099511628211ull;
	}
	return h;
}
#define FNV0 1469598103934665603ull

static void print_out(const char *tag, const pimax_hands_out *o)
{
	const pimax_hands_finger *f[5] = {&o->little, &o->ring, &o->middle, &o->index, &o->thumb};
	printf("%s", tag);
	for (int i = 0; i < 5; i++) {
		printf(" [%a", f[i]->splay);
		for (int k = 0; k < 4; k++) printf(" %a", f[i]->joint_curls[k]);
		printf("]");
	}
	printf(" curl");
	for (int k = 0; k < 5; k++) printf(" %a", o->curl[k]);
	printf(" thumb %u\n", o->thumb_state);
}

static void print_pose(const char *tag, const pimax_hands_pose *p)
{
	printf("%s q %a %a %a %a p %a %a %a\n", tag, p->orientation[0], p->orientation[1], p->orientation[2],
	       p->orientation[3], p->position[0], p->position[1], p->position[2]);
}

/* ---------- layout ---------- */

#define F(s, f) printf(" %s=%zu", #f, offsetof(s, f))
static int mode_layout(void)
{
	printf("input size=%zu align=%zu", sizeof(pimax_hands_input), _Alignof(pimax_hands_input));
	F(pimax_hands_input, trigger); F(pimax_hands_input, grip); F(pimax_hands_input, stick_x);
	F(pimax_hands_input, stick_y); F(pimax_hands_input, flags); printf("\n");
	printf("finger size=%zu align=%zu", sizeof(pimax_hands_finger), _Alignof(pimax_hands_finger));
	F(pimax_hands_finger, splay); F(pimax_hands_finger, joint_curls); printf("\n");
	printf("out size=%zu align=%zu", sizeof(pimax_hands_out), _Alignof(pimax_hands_out));
	F(pimax_hands_out, little); F(pimax_hands_out, ring); F(pimax_hands_out, middle); F(pimax_hands_out, index);
	F(pimax_hands_out, thumb); F(pimax_hands_out, curl); F(pimax_hands_out, thumb_state); printf("\n");
	printf("pose size=%zu align=%zu", sizeof(pimax_hands_pose), _Alignof(pimax_hands_pose));
	F(pimax_hands_pose, orientation); F(pimax_hands_pose, position); printf("\n");
	printf("info size=%zu", sizeof(pimax_ctrlfuse_info));
	F(pimax_ctrlfuse_info, state); F(pimax_ctrlfuse_info, gyro); printf("\n");
	printf("abi %u (header %u)\n", pimax_hands_abi_version(), PIMAX_HANDS_ABI_VERSION);
	for (int m = -2; m < 5; m++) printf("mode %d %s\n", m, pimax_inhand_mode_name(m));
	return 0;
}

/* ---------- hands ---------- */

static float nasty(void)
{
	switch (rndn(12)) {
	case 0: return NAN;
	case 1: return INFINITY;
	case 2: return -INFINITY;
	case 3: return 3.4028235e38f;
	case 4: return -3.4028235e38f;
	case 5: return 1e-40f;
	case 6: return -0.0f;
	default: return (float)(rndf() * 1.6);
	}
}

static int mode_hands(uint64_t seed, int n)
{
	g_rng = seed;
	pimax_hands *h[2] = {pimax_hands_create(), pimax_hands_create()};
	uint64_t t[2] = {seed * 7919u, 1000000000ull};
	uint64_t hash = FNV0;
	pimax_hands_input in = {0};
	int phase_len = 0, phase = 0;
	for (int i = 0; i < n; i++) {
		const int k = (int)rndn(2);
		if (phase_len-- <= 0) {
			phase = (int)rndn(8);
			phase_len = 20 + (int)rndn(400);
			in.trigger = (float)(rndf() * 0.5 + 0.5);
			in.grip = (float)(rndf() * 0.5 + 0.5);
			in.stick_x = (float)rndf();
			in.stick_y = (float)rndf();
			in.flags = (uint32_t)rndn(128);
		}
		/* time step: mostly 4 ms, sometimes jitter, a long gap, a repeat or a step back */
		switch (rndn(40)) {
		case 0: t[k] += 600000000ull + rndn(3000000000ull); break;
		case 1: break;
		case 2: t[k] -= rndn(50000000ull); break;
		case 3: t[k] += rndn(200000000ull); break;
		default: t[k] += 3000000ull + rndn(2000000ull); break;
		}
		switch (phase) {
		case 0: /* steady clean input with small jitter */
		{
			pimax_hands_input j = in;
			j.trigger += (float)(rndf() * 0.01);
			j.grip += (float)(rndf() * 0.01);
			pimax_hands_update(h[k], t[k], &j);
			break;
		}
		case 1: /* flicker of the touch bits */
		{
			pimax_hands_input j = in;
			j.flags = (uint32_t)(rndn(3) == 0 ? rndn(8) : (in.flags & 7));
			pimax_hands_update(h[k], t[k], &j);
			break;
		}
		case 2: /* garbage */
		{
			pimax_hands_input j = {nasty(), nasty(), nasty(), nasty(), (uint32_t)rnd()};
			pimax_hands_update(h[k], t[k], &j);
			break;
		}
		case 3: /* raw report path */
		case 4:
			pimax_hands_update_report(h[k], t[k], (uint8_t)rndn(256), (uint8_t)rndn(256), (int)rndn(256) - 128,
			                          (int)rndn(256) - 128, (uint32_t)rnd(), (uint8_t)rndn(256));
			break;
		case 5: /* report path with odd stick values */
			pimax_hands_update_report(h[k], t[k], 255, 0, (int)(rnd() & 0xffffffff), -(int)rndn(100000), 0x1f,
			                          (uint8_t)rndn(16));
			break;
		case 6: /* sweep */
		{
			pimax_hands_input j = in;
			j.trigger = (float)(0.5 + 0.5 * sin((double)i * 0.01));
			j.grip = (float)(0.5 + 0.5 * cos((double)i * 0.013));
			j.stick_x = (float)sin((double)i * 0.02);
			pimax_hands_update(h[k], t[k], &j);
			break;
		}
		default:
			if (rndn(50) == 0) {
				pimax_hands_reset(h[k]);
			} else {
				pimax_hands_update(h[k], t[k], &in);
			}
			break;
		}
		pimax_hands_out o;
		memset(&o, 0, sizeof o);
		pimax_hands_get(h[k], &o);
		hash = fnv(hash, &o, sizeof o);
		if (i % 97 == 0) {
			char tag[64];
			snprintf(tag, sizeof tag, "%d h%d t=%llu", i, k, (unsigned long long)t[k]);
			print_out(tag, &o);
		}
	}
	printf("hash %016llx\n", (unsigned long long)hash);
	/* demo input over two periods, every 7 ms, plus big and odd times */
	hash = FNV0;
	for (uint64_t ns = 0; ns < 25000000000ull; ns += 7000000ull) {
		for (int r = 0; r < 2; r++) {
			pimax_hands_input d;
			pimax_hands_demo_input(ns, r, &d);
			hash = fnv(hash, &d, sizeof d);
		}
	}
	printf("demo hash %016llx\n", (unsigned long long)hash);
	const uint64_t odd[] = {0, 1, 999999999, 1000000000, 2999999999ull, 5000000000ull, 6500000000ull,
	                        7000000000ull, 18446744073709551615ull, 9223372036854775808ull, 123456789012345678ull};
	for (size_t i = 0; i < sizeof odd / sizeof odd[0]; i++) {
		for (int r = -1; r < 2; r++) {
			pimax_hands_input d;
			pimax_hands_demo_input(odd[i], r, &d);
			printf("demo %llu %d: %a %a %a %a %u\n", (unsigned long long)odd[i], r, d.trigger, d.grip, d.stick_x,
			       d.stick_y, d.flags);
		}
	}
	for (int r = -1; r < 3; r++) {
		pimax_hands_pose p;
		pimax_hands_grip_to_wrist(r, &p);
		char tag[32];
		snprintf(tag, sizeof tag, "grip_to_wrist %d", r);
		print_pose(tag, &p);
	}
	pimax_hands_pose tp;
	pimax_hands_tracked_aim_to_grip(&tp);
	print_pose("tracked_aim_to_grip", &tp);
	/* NULLs */
	pimax_hands_update_report(NULL, 1, 0, 0, 0, 0, 0, 0);
	pimax_hands_update(NULL, 1, &in);
	pimax_hands_update(h[0], 1, NULL);
	pimax_hands_out o;
	memset(&o, 0x5a, sizeof o);
	pimax_hands_get(NULL, &o);
	pimax_hands_get(h[0], NULL);
	pimax_hands_grip_to_wrist(0, NULL);
	pimax_hands_demo_input(0, 0, NULL);
	pimax_hands_tracked_aim_to_grip(NULL);
	pimax_hands_reset(NULL);
	pimax_hands_destroy(NULL);
	printf("untouched %d\n", o.thumb_state == 0x5a5a5a5au);
	pimax_hands_destroy(h[0]);
	pimax_hands_destroy(h[1]);
	return 0;
}

/* ---------- poses ---------- */

static int mode_poses(void)
{
	char buf[512];
	int bad = pimax_hands_controller_describe(buf, sizeof buf);
	printf("describe %d: %s\n", bad, buf);
	for (size_t len = 0; len < 12; len++) {
		char b[16];
		memset(b, 'x', sizeof b);
		int r = pimax_hands_controller_describe(len ? b : NULL, len);
		printf("describe len %zu -> %d [%.*s]\n", len, r, (int)(len ? strlen(b) < 16 ? strlen(b) : 16 : 0), b);
	}
	printf("describe NULL,5 -> %d\n", pimax_hands_controller_describe(NULL, 5));
	pimax_hands_pose g;
	pimax_hands_aim_to_grip(&g);
	print_pose("aim_to_grip", &g);
	pimax_hands_aim_to_grip(NULL);
	g_rng = 99;
	for (int i = 0; i < 400; i++) {
		float q[4];
		const float *qp = q;
		switch (i) {
		case 0: qp = NULL; break;
		case 1: q[0] = q[1] = q[2] = q[3] = 0; break;
		case 2: q[0] = NAN; q[1] = q[2] = 0; q[3] = 1; break;
		case 3: q[0] = INFINITY; q[1] = q[2] = 0; q[3] = 1; break;
		case 4: q[0] = 0; q[1] = 0.70710678f; q[2] = 0; q[3] = 0.70710678f; break;
		case 5: q[0] = 0.70710678f; q[1] = 0; q[2] = 0; q[3] = 0.70710678f; break; /* looking straight up */
		case 6: q[0] = -0.70710678f; q[1] = 0; q[2] = 0; q[3] = 0.70710678f; break;
		case 7: q[0] = 0; q[1] = 0; q[2] = 0; q[3] = -3.0f; break;
		case 8: q[0] = 1e-7f; q[1] = 0; q[2] = 0; q[3] = 0; break;
		case 9: q[0] = 3e38f; q[1] = 3e38f; q[2] = 0; q[3] = 0; break;
		default:
			for (int k = 0; k < 4; k++) q[k] = (float)rndf();
			if (i % 5 == 0) q[3] *= 0.01f;
			break;
		}
		for (int r = 0; r < 2; r++) {
			pimax_hands_pose a, gp;
			memset(&a, 0, sizeof a);
			memset(&gp, 0, sizeof gp);
			pimax_hands_controller_poses(r ? (i % 3) + 1 : 0, qp, &a, &gp);
			char tag[32];
			snprintf(tag, sizeof tag, "%d %d aim", i, r);
			print_pose(tag, &a);
			snprintf(tag, sizeof tag, "%d %d grip", i, r);
			print_pose(tag, &gp);
		}
		pimax_hands_controller_poses(1, qp, NULL, &g);
		pimax_hands_controller_poses(1, qp, &g, NULL);
		pimax_hands_controller_poses(1, qp, NULL, NULL);
	}
	return 0;
}

/* ---------- recorded reports ---------- */

typedef struct {
	uint8_t slot, trigger, grip, touch;
	int stick_x, stick_y;
	uint32_t buttons;
} decoded;

/* pimax-proto decode_controller (the fields the driver hands over). 0 when the record is no controller report. */
static int decode(const uint8_t *r, decoded *d)
{
	if (r[0] != 0x53 || r[1] != 0x32 || r[2] != 0x08) return 0;
	if (r[3] != 2 && r[3] != 3) return 0;
	d->slot = r[3];
	d->stick_x = (int)r[5] - 128;
	d->stick_y = (int)r[6] - 128;
	d->trigger = r[7];
	d->grip = r[44];
	d->buttons = (uint32_t)r[47] & (0x01 | 0x02 | 0x08);
	if (r[48] & 1) d->buttons |= 0x10;
	d->touch = r[49];
	return 1;
}

static unsigned char *gz_read_all(const char *path, size_t *len)
{
	gzFile f = gzopen(path, "rb");
	if (!f) return NULL;
	size_t cap = 1 << 20, n = 0;
	unsigned char *b = malloc(cap);
	for (;;) {
		if (n + 65536 > cap) {
			cap *= 2;
			b = realloc(b, cap);
		}
		int r = gzread(f, b + n, 65536);
		if (r <= 0) break;
		n += (size_t)r;
	}
	gzclose(f);
	*len = n;
	return b;
}

static int mode_replay(const char *path)
{
	size_t len = 0;
	unsigned char *d = gz_read_all(path, &len);
	if (!d) {
		printf("cannot read %s\n", path);
		return 1;
	}
	const size_t nrec = len / 72;
	double t0;
	memcpy(&t0, d, 8);
	pimax_hands *h[2] = {pimax_hands_create(), pimax_hands_create()};
	pimax_inhand *ih[2] = {pimax_inhand_create(0), pimax_inhand_create(1)};
	pimax_ctrlfuse *cf[2] = {pimax_ctrlfuse_create(0), pimax_ctrlfuse_create(1)};
	pimax_ctrlfuse_link(cf[0], cf[1]);
	uint64_t hash[2] = {FNV0, FNV0};
	size_t count[2] = {0, 0};
	int held_prev[2] = {-1, -1};
	uint64_t fuse_hash = FNV0;
	for (size_t i = 0; i < nrec; i++) {
		double t;
		memcpy(&t, d + i * 72, 8);
		const uint8_t *r = d + i * 72 + 8;
		decoded c;
		if (!decode(r, &c)) continue;
		const int k = c.slot == 2 ? 0 : 1;
		/* the Rust replay test's clock: ((t - t0) * 1e9) as u64, and a host clock 1 s later for the rest */
		const double dt = (t - t0) * 1e9;
		const uint64_t ns = dt <= 0 ? 0 : (uint64_t)dt;
		pimax_hands_update_report(h[k], ns, c.trigger, c.grip, c.stick_x, c.stick_y, c.buttons, c.touch);
		pimax_hands_out o;
		pimax_hands_get(h[k], &o);
		hash[k] = fnv(hash[k], &o, sizeof o);
		if (count[k] % 211 == 0) {
			char tag[64];
			snprintf(tag, sizeof tag, "s%u #%zu ns=%llu", c.slot, count[k], (unsigned long long)ns);
			print_out(tag, &o);
		}
		count[k]++;
		const uint64_t host = 1000000000ull + ns;
		pimax_inhand_report(ih[k], host, r, c.trigger, c.grip, c.stick_x, c.stick_y, c.buttons, c.touch);
		const int held = pimax_inhand_held(ih[k], host);
		if (held != held_prev[k]) {
			char b[64];
			pimax_inhand_describe(ih[k], host, b, sizeof b);
			printf("inhand s%u #%zu %d %s\n", c.slot, count[k], held, b);
			held_prev[k] = held;
		}
		pimax_ctrlfuse_report(cf[k], host, r);
		if (count[k] % 50 == 0) {
			pimax_hands_pose head = {{0, 0, 0, 1}, {0, 0, 0}}, out;
			pimax_ctrlfuse_info info;
			int s = pimax_ctrlfuse_get(cf[k], host, &head, &out, &info);
			fuse_hash = fnv(fuse_hash, &s, sizeof s);
			fuse_hash = fnv(fuse_hash, &out, sizeof out);
			fuse_hash = fnv(fuse_hash, &info, sizeof info);
		}
	}
	for (int k = 0; k < 2; k++) {
		char b[512];
		pimax_ctrlfuse_describe(cf[k], b, sizeof b);
		printf("hand %d: %zu reports, hash %016llx\nfuse %d: %s\n", k, count[k], (unsigned long long)hash[k], k, b);
		pimax_hands_destroy(h[k]);
		pimax_inhand_destroy(ih[k]);
	}
	printf("fuse hash %016llx\n", (unsigned long long)fuse_hash);
	pimax_ctrlfuse_destroy(cf[0]);
	pimax_ctrlfuse_destroy(cf[1]);
	free(d);
	return 0;
}

/* ---------- in-hand ---------- */

static void put_be16(uint8_t *r, int o, int v)
{
	r[o] = (uint8_t)((unsigned)v >> 8);
	r[o + 1] = (uint8_t)v;
}

/* A 64-byte controller report with one or two IMU samples and the given inputs. */
static void make_report(uint8_t *r, int slot, uint32_t ts, const int16_t *g0, const int16_t *a0, const int16_t *g1,
                        const int16_t *a1, uint8_t trigger, uint8_t grip, int sx, int sy, uint32_t buttons,
                        uint8_t touch)
{
	memset(r, 0, 64);
	r[0] = 0x53; r[1] = 0x32; r[2] = 0x08; r[3] = (uint8_t)slot;
	r[5] = (uint8_t)(sx + 128); r[6] = (uint8_t)(sy + 128); r[7] = trigger;
	r[8] = (uint8_t)ts; r[9] = (uint8_t)(ts >> 8); r[10] = (uint8_t)(ts >> 16); r[11] = (uint8_t)(ts >> 24);
	for (int k = 0; k < 3; k++) {
		put_be16(r, 16 + 2 * k, a0[k]);
		put_be16(r, 22 + 2 * k, g0[k]);
		if (g1) {
			put_be16(r, 30 + 2 * k, a1[k]);
			put_be16(r, 36 + 2 * k, g1[k]);
		}
	}
	r[44] = grip;
	r[47] = (uint8_t)(buttons & 0x0b);
	r[48] = (buttons & 0x10) ? 1 : 0;
	r[49] = touch;
}

static int mode_inhand_csv(const char *path, const char *mode)
{
	size_t len = 0;
	unsigned char *d = gz_read_all(path, &len);
	if (!d) {
		printf("cannot read %s\n", path);
		return 1;
	}
	char *text = malloc(len + 1);
	memcpy(text, d, len);
	text[len] = 0;
	free(d);
	/* parse the samples */
	size_t cap = 1024, n = 0;
	int64_t (*rows)[7] = malloc(cap * sizeof *rows);
	for (char *line = strtok(text, "\n"); line; line = strtok(NULL, "\n")) {
		if (n == cap) {
			cap *= 2;
			rows = realloc(rows, cap * sizeof *rows);
		}
		char *p = line;
		for (int k = 0; k < 7; k++) {
			rows[n][k] = strtoll(p, &p, 10);
			if (*p == ',') p++;
		}
		n++;
	}
	free(text);
	pimax_inhand *h = pimax_inhand_create(0);
	uint64_t t = 1000000000ull;
	uint64_t t0 = t + (uint64_t)rows[0][0] * 1000;
	const int picked = strcmp(mode, "picked") == 0 || strcmp(mode, "half") == 0;
	const size_t end = strcmp(mode, "half") == 0 ? n / 2 : n;
	uint8_t r[64];
	const int16_t z[3] = {0, 0, 0};
	if (picked) {
		/* pick up first with a button: the reference report, then A pressed (the IMU of these two is zero) */
		make_report(r, 2, 0, z, z, NULL, NULL, 0, 0, 0, 0, 0, 0);
		pimax_inhand_report(h, t0 - 2, r, 0, 0, 0, 0, 0, 0);
		pimax_inhand_report(h, t0 - 1, r, 0, 0, 0, 0, 0x02, 0);
		printf("picked %d\n", pimax_inhand_held(h, t0 - 1));
	}
	int held = pimax_inhand_held(h, t0);
	uint64_t seed = 12345;
	uint64_t next_key = t0 + 100000000ull, key_end = 0;
	uint64_t last = t;
	for (size_t i = 0; i < end; i++) {
		t += (uint64_t)rows[i][0] * 1000;
		int16_t g[3] = {(int16_t)rows[i][1], (int16_t)rows[i][2], (int16_t)rows[i][3]};
		int16_t a[3] = {(int16_t)rows[i][4], (int16_t)rows[i][5], (int16_t)rows[i][6]};
		if (strcmp(mode, "typing") == 0) {
			/* keystroke knocks (inhand_fixtures.rs typing_on_the_desk_stays_down) */
			if (t >= next_key) {
				key_end = t + 10000000ull;
				seed = seed * 6364136223846793005ull + 1442695040888963407ull;
				next_key = t + 120000000ull + (uint64_t)(((int32_t)(seed >> 33) % 81) - 40 + 40) * 1000000ull;
			}
			if (t < key_end) {
				for (int k = 0; k < 3; k++) {
					seed = seed * 6364136223846793005ull + 1442695040888963407ull;
					g[k] = (int16_t)(g[k] + (((int32_t)(seed >> 33) % 31) - 15));
					seed = seed * 6364136223846793005ull + 1442695040888963407ull;
					a[k] = (int16_t)(a[k] + (((int32_t)(seed >> 33) % 81) - 40));
				}
			}
		}
		make_report(r, 2, (uint32_t)(t / 1000), g, a, NULL, NULL, 0, 0, 0, 0, picked ? 0x02 : 0, 0);
		pimax_inhand_report(h, t, r, 0, 0, 0, 0, picked ? 0x02 : 0, 0);
		const int now = pimax_inhand_held(h, t);
		if (now != held) {
			char b[64];
			pimax_inhand_describe(h, t, b, sizeof b);
			printf("%.9f %d %s\n", (double)(t - t0) * 1e-9, now, b);
			held = now;
		}
		last = t;
	}
	if (strcmp(mode, "half") == 0) {
		printf("+1 s %d\n", pimax_inhand_held(h, last + 1000000000ull));
		printf("+2 s %d\n", pimax_inhand_held(h, last + 2000000000ull));
		char b[64];
		pimax_inhand_describe(h, last + 2000000000ull, b, sizeof b);
		printf("%s\n", b);
	}
	printf("end %d after %zu samples\n", held, end);
	pimax_inhand_destroy(h);
	free(rows);
	return 0;
}

static void dump_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		printf("[%s: none]\n", strrchr(path, '/') + 1);
		return;
	}
	char b[512];
	printf("[%s]\n", strrchr(path, '/') + 1);
	while (fgets(b, sizeof b, f)) {
		if (strncmp(b, "pid=", 4) == 0) {
			printf("pid=<pid>\n"); /* differs between any two runs */
			continue;
		}
		fputs(b, stdout);
	}
	fclose(f);
}

static void write_text(const char *path, const char *s)
{
	FILE *f = fopen(path, "wb");
	if (f) {
		fputs(s, f);
		fclose(f);
	}
}

static int mode_inhand_synth(uint64_t seed, const char *work)
{
	g_rng = seed;
	char status[1024], req[1024];
	snprintf(status, sizeof status, "%s/pimax-ctrl-inhand", work);
	snprintf(req, sizeof req, "%s/pimax-ctrl-inhand-mode", work);
	const int dm[] = {-1, 0, 1, 2, 7};
	for (size_t i = 0; i < sizeof dm / sizeof dm[0]; i++) printf("mode_from_env(%d) = %d\n", dm[i], pimax_inhand_mode_from_env(dm[i]));
	/* run-time requests */
	const char *reqs[] = {"always", " Off\n", "WHEN-HELD", "picked-up", "on", "0", "nope", "", "\xff\xfe", "auto\n\n"};
	for (size_t i = 0; i < sizeof reqs / sizeof reqs[0]; i++) {
		write_text(req, reqs[i]);
		int m = pimax_inhand_poll_mode();
		printf("request %zu -> %d, file left %d\n", i, m, access(req, F_OK) == 0);
		unlink(req);
	}
	printf("no request -> %d\n", pimax_inhand_poll_mode());
	write_text(req, "always");
	pimax_inhand_clear_request();
	printf("cleared -> %d\n", pimax_inhand_poll_mode());
	pimax_inhand_clear_request();

	pimax_inhand *h[2] = {pimax_inhand_create(0), pimax_inhand_create(1)};
	uint64_t t = 5000000000ull;
	uint32_t dev = 0;
	double ang[2] = {0, 0};
	int phase[2] = {0, 0}, left[2] = {0, 0};
	uint8_t trig[2] = {0, 0};
	int held_prev[2] = {-1, -1};
	int pub_mode = 0;
	const int N = 100000;
	for (int i = 0; i < N; i++) {
		t += 2000000ull + rndn(200000);
		dev += 2000;
		const int k = i & 1;
		if (left[k]-- <= 0) {
			phase[k] = (int)rndn(7);
			left[k] = 200 + (int)rndn(6000);
		}
		int16_t g[3], a[3], g1[3], a1[3];
		uint8_t tr = trig[k], gr = 0;
		uint32_t btn = 0;
		int sx = 0;
		int silent = 0;
		switch (phase[k]) {
		case 0: /* lying still with sensor noise */
			for (int j = 0; j < 3; j++) {
				g[j] = (int16_t)(-5 + (int)rndn(3) - 1);
				a[j] = (int16_t)((j == 1 ? 669 : (j == 2 ? -726 : -17)) + (int)rndn(3) - 1);
			}
			break;
		case 1: /* waved in the hand */
			ang[k] += 0.004;
			for (int j = 0; j < 3; j++) {
				g[j] = (int16_t)(200.0 * sin(ang[k] * (j + 1)) + rndf() * 20);
				a[j] = (int16_t)(700.0 * cos(ang[k] + j) + rndf() * 30);
			}
			break;
		case 2: /* quiet in the hand: tremor */
			for (int j = 0; j < 3; j++) {
				g[j] = (int16_t)(rndf() * 15);
				a[j] = (int16_t)((j == 1 ? 900 : 100) + rndf() * 10);
			}
			break;
		case 3: /* tilted slowly on the desk */
			ang[k] += 0.0004;
			for (int j = 0; j < 3; j++) {
				g[j] = (int16_t)(rndn(3));
				a[j] = (int16_t)(1000.0 * (j == 1 ? cos(ang[k]) : (j == 2 ? sin(ang[k]) : 0.0)));
			}
			break;
		case 4: /* buttons and analog drift */
			for (int j = 0; j < 3; j++) {
				g[j] = (int16_t)(rndn(5));
				a[j] = (int16_t)(j == 1 ? 1000 : 0);
			}
			if (rndn(500) == 0) btn = 0x02;
			if (rndn(3) == 0 && tr < 250) tr++;
			gr = (uint8_t)rndn(20);
			sx = (int)rndn(30) - 15;
			break;
		case 5: /* silent: no report */
			silent = 1;
			for (int j = 0; j < 3; j++) g[j] = a[j] = 0;
			break;
		default: /* knocks */
			for (int j = 0; j < 3; j++) {
				g[j] = (int16_t)(rndn(40) == 0 ? rndf() * 300 : rndn(3));
				a[j] = (int16_t)((j == 1 ? 1000 : 0) + (rndn(40) == 0 ? rndf() * 200 : 0));
			}
			break;
		}
		trig[k] = tr;
		if (!silent) {
			for (int j = 0; j < 3; j++) {
				g1[j] = (int16_t)(g[j] + (int)rndn(3) - 1);
				a1[j] = (int16_t)(a[j] + (int)rndn(3) - 1);
			}
			uint8_t r[64];
			const int two = rndn(4) != 0;
			make_report(r, 2 + k, dev, g, a, two ? g1 : NULL, two ? a1 : NULL, tr, gr, sx, 0, btn, (uint8_t)rndn(2));
			pimax_inhand_report(h[k], t, r, tr, gr, sx, 0, btn, (uint8_t)(rndn(200) == 0 ? rndn(256) : 0));
		}
		const int held = pimax_inhand_held(h[k], t);
		if (held != held_prev[k]) {
			char b[64];
			size_t n = pimax_inhand_describe(h[k], t, b, sizeof b);
			printf("%d h%d %d %zu %s\n", i, k, held, n, b);
			held_prev[k] = held;
		}
		if (i % 997 == 0) {
			pub_mode = (int)rndn(4);
			int w = pimax_inhand_publish(pub_mode, rndn(9) ? h[0] : NULL, h[1], t);
			printf("%d publish %d -> %d\n", i, pub_mode, w);
			if (w) dump_file(status);
			printf("%d publish again -> %d\n", i, pimax_inhand_publish(pub_mode, h[0], h[1], t));
		}
	}
	char b[8];
	printf("describe small %zu [%s]\n", pimax_inhand_describe(h[0], t, b, sizeof b), b);
	printf("describe NULL h %zu [%s]\n", pimax_inhand_describe(NULL, t, b, sizeof b), b);
	printf("describe 0 len %zu\n", pimax_inhand_describe(h[0], t, b, 0));
	printf("describe NULL buf %zu\n", pimax_inhand_describe(h[0], t, NULL, 9));
	printf("held NULL %d\n", pimax_inhand_held(NULL, t));
	pimax_inhand_report(NULL, t, NULL, 0, 0, 0, 0, 0, 0);
	pimax_inhand_report(h[0], t, NULL, 0, 0, 0, 0, 0, 0);
	printf("publish off -> %d\n", pimax_inhand_publish(2, h[0], h[1], t));
	dump_file(status);
	printf("publish NULLs -> %d\n", pimax_inhand_publish(1, NULL, NULL, t));
	dump_file(status);
	pimax_inhand_destroy(h[0]);
	pimax_inhand_destroy(h[1]);
	pimax_inhand_destroy(NULL);
	return 0;
}

/* ---------- ctrlfuse ---------- */

typedef double V3[3];
typedef double Q4[4];

static void qmul(const double *a, const double *b, double *o)
{
	double r[4] = {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
	               a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
	memcpy(o, r, sizeof r);
}
static void qnorm(double *q)
{
	double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int k = 0; k < 4; k++) q[k] /= n;
}
static void qexp(const double *w, double *o)
{
	double a = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	if (a < 1e-12) {
		o[0] = w[0] / 2; o[1] = w[1] / 2; o[2] = w[2] / 2; o[3] = 1;
		qnorm(o);
		return;
	}
	double s = sin(a / 2) / a;
	o[0] = w[0] * s; o[1] = w[1] * s; o[2] = w[2] * s; o[3] = cos(a / 2);
}
static void qrot(const double *q, const double *v, double *o)
{
	double p[4] = {v[0], v[1], v[2], 0}, c[4] = {-q[0], -q[1], -q[2], q[3]}, t[4];
	qmul(q, p, t);
	qmul(t, c, t);
	o[0] = t[0]; o[1] = t[1]; o[2] = t[2];
}

static void to_pose(const double *q, const double *p, pimax_hands_pose *o)
{
	for (int k = 0; k < 4; k++) o->orientation[k] = (float)q[k];
	for (int k = 0; k < 3; k++) o->position[k] = (float)p[k];
}

/* Synthetic session. scenario: 0 = both hands, smooth motion, fixes at 25 Hz with bad solves, gaps out of view,
 * head turns and walks; 1 = a desk session (controllers mostly still, put down, picked up); 2 = no IMU reports at
 * all (optical only); 3 = gyro saturating and reordered device clocks, wild optical fixes (NaN free), extreme times;
 * 4 = like 0 with a host clock that jitters and reports bunched; 5 = like 0, the hands not linked; 6 = clean data
 * (many LEDs, no bad solves, always in view) so the online gyro fit replaces the built-in calibration and is saved. */
static int mode_fuse(int scenario, uint64_t seed, double seconds, const char *dumpdir)
{
	g_rng = seed;
	pimax_ctrlfuse *f[2] = {pimax_ctrlfuse_create(0), pimax_ctrlfuse_create(1)};
	if (scenario != 5) pimax_ctrlfuse_link(f[0], f[1]);
	char b[1024];
	for (int k = 0; k < 2; k++) {
		pimax_ctrlfuse_describe(f[k], b, sizeof b);
		printf("start %d: %s\n", k, b);
	}
	/* true raw -> aim mapping of each hand (the built-in calibration's axes, a slightly different scale) */
	const double s = 0.122 * M_PI / 180.0 * (1.0 + 0.02 * rndf());
	const double bias[2][3] = {{12, -7, 3}, {-4, 9, 15}};
	Q4 q[2] = {{0, 0, 0, 1}, {0, 0, 0, 1}};
	V3 pos[2] = {{-0.2, -0.3, -0.35}, {0.2, -0.3, -0.35}};
	double head_yaw = 0, head_pitch = 0;
	V3 head_p = {0, 0, 0};
	const uint64_t T0 = 20000000000ull + rndn(1000000000ull);
	const double dt = 0.002;
	const long n = (long)(seconds / dt);
	uint32_t dev[2] = {(uint32_t)rnd(), (uint32_t)rnd()};
	int16_t pend[2][3], penda[2][3];
	int have_pend[2] = {0, 0};
	int out_of_view[2] = {0, 0};
	int ov_left[2] = {0, 0};
	int still[2] = {0, 0};
	uint64_t gethash = FNV0;
	long nget = 0;
	int last_state[2] = {-1, -1};
	for (long i = 0; i < n; i++) {
		const double ts = (double)i * dt;
		uint64_t host = T0 + (uint64_t)(ts * 1e9);
		/* head motion */
		head_yaw = 0.8 * sin(ts * 0.21) + (scenario == 0 ? 0.6 * sin(ts * 0.05) : 0.0);
		head_pitch = 0.3 * sin(ts * 0.17);
		head_p[0] = 0.2 * sin(ts * 0.07);
		head_p[2] = scenario == 0 ? 0.5 * sin(ts * 0.03) : 0.0;
		Q4 hy = {0, sin(head_yaw / 2), 0, cos(head_yaw / 2)}, hp = {sin(head_pitch / 2), 0, 0, cos(head_pitch / 2)}, hq;
		qmul(hy, hp, hq);
		for (int k = 0; k < 2; k++) {
			if ((i + k * 371) % 4000 == 0) still[k] = scenario == 1 ? rndn(3) != 0 : rndn(6) == 0;
			V3 w = {0, 0, 0};
			if (!still[k]) {
				w[0] = 2.0 * sin(1.3 * ts + k) + 0.7 * cos(3.1 * ts);
				w[1] = 1.5 * sin(0.7 * ts + 1.0 + k);
				w[2] = 1.2 * cos(2.3 * ts - k);
				/* bounded motion around the hand's home in front of the body */
				pos[k][0] = (k ? 0.2 : -0.2) + 0.12 * sin(0.9 * ts + k);
				pos[k][1] = -0.3 + 0.1 * sin(1.1 * ts);
				pos[k][2] = -0.35 + 0.1 * cos(0.6 * ts + 2 * k);
			}
			Q4 dq, nq;
			V3 wd = {w[0] * dt, w[1] * dt, w[2] * dt};
			qexp(wd, dq);
			qmul(q[k], dq, nq);
			qnorm(nq);
			memcpy(q[k], nq, sizeof nq);
			/* gyro: aim = C raw, C = scale * axis permutation (left: aim x = -raw z, y = -raw x, z = raw y) */
			double raw[3];
			if (k == 0) {
				raw[2] = -w[0] / s; raw[0] = -w[1] / s; raw[1] = w[2] / s;
			} else {
				raw[2] = w[0] / s; raw[0] = w[1] / s; raw[1] = w[2] / s;
			}
			int16_t g[3], a[3];
			for (int j = 0; j < 3; j++) {
				double v = raw[j] + bias[k][j] + rndf() * 2.0;
				if (scenario == 3 && rndn(200) == 0) v = rndf() * 40000.0;
				g[j] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : lrint(v)));
				a[j] = (int16_t)(j == 1 ? 1024 + rndf() * 5 : rndf() * 5);
			}
			if (scenario == 2) continue;
			if (!have_pend[k]) {
				memcpy(pend[k], g, sizeof g);
				memcpy(penda[k], a, sizeof a);
				have_pend[k] = 1;
			} else {
				have_pend[k] = 0;
				uint8_t r[64];
				uint32_t d0 = dev[k];
				if (scenario == 3 && rndn(300) == 0) d0 -= (uint32_t)rndn(100000); /* reordered */
				const int two = rndn(10) != 0;
				make_report(r, 2 + k, d0, pend[k], penda[k], two ? g : NULL, two ? a : NULL, 0, 0, 0, 0, 0, 0);
				dev[k] += 4000; /* a report without its second sample still covers 4 ms */
				uint64_t at = host + 16000000ull + 1000000ull + rndn(800000);
				if (scenario == 4) at += rndn(6000000);
				pimax_ctrlfuse_report(f[k], at, r);
			}
		}
		/* optical fixes at 25 Hz per hand, staggered */
		if (i % 20 == 0 || i % 20 == 10) {
			const int k = (i % 20) ? 1 : 0;
			if (ov_left[k]-- <= 0) {
				out_of_view[k] = scenario != 6 && rndn(5) == 0;
				ov_left[k] = out_of_view[k] ? 5 + (int)rndn(60) : 10 + (int)rndn(150);
				if (out_of_view[k] && rndn(4) == 0) ov_left[k] = 1 + (int)rndn(4); /* short dropouts (re-entry) */
			}
			if (!out_of_view[k]) {
				V3 noise = {rndf() * 0.01, rndf() * 0.01, rndf() * 0.01};
				Q4 nq, dq;
				qexp(noise, dq);
				qmul(q[k], dq, nq);
				qnorm(nq);
				uint32_t leds = scenario == 6 ? 10 + (uint32_t)rndn(5) : 5 + (uint32_t)rndn(10);
				float rms = (float)(0.3 + 2.5 * fabs(rndf()) * fabs(rndf()));
				if (scenario != 6 && (i / 20) % 9 == 4) {
					/* a bad solve: flipped orientation, few LEDs, or a jump */
					V3 flip = {rndf() * 2.5, rndf() * 2.5, rndf() * 2.5};
					qexp(flip, dq);
					qmul(nq, dq, nq);
					qnorm(nq);
					if (rndn(2)) leds = 5 + (uint32_t)rndn(3);
				}
				V3 p = {pos[k][0], pos[k][1], pos[k][2]};
				if (scenario != 6 && (i / 20) % 23 == 7) p[0] += (rndn(2) ? 0.3 : -0.3); /* jump */
				if ((i / 20) % 41 == 9) {
					p[0] = pos[1 - k][0]; /* on top of the other hand */
					p[1] = pos[1 - k][1];
					p[2] = pos[1 - k][2];
				}
				if ((i / 20) % 53 == 11) p[0] = k == 0 ? 0.6 : -0.6; /* wrong side */
				if ((i / 20) % 67 == 13) p[2] = -2.0;                 /* out of reach */
				if (scenario == 3 && rndn(20) == 0) {
					p[0] = rndf() * 3; p[1] = rndf() * 3; p[2] = rndf() * 3;
				}
				/* world_from_aim = head * (head_from_aim): the optical tracker sees it relative to the head */
				V3 wp;
				qrot(hq, p, wp);
				for (int j = 0; j < 3; j++) wp[j] += head_p[j];
				Q4 wq;
				qmul(hq, nq, wq);
				pimax_hands_pose aim, head;
				to_pose(wq, wp, &aim);
				to_pose(hq, head_p, &head);
				uint64_t frame = host - 30000000ull - rndn(5000000);
				if (scenario == 3 && rndn(50) == 0) frame -= 3000000000ull; /* stale */
				if (scenario == 3 && rndn(80) == 0) frame = 0;
				pimax_ctrlfuse_optical(f[k], frame, &aim, &head, rms, leds);
			}
		}
		/* display queries at 90 Hz for both hands */
		if (i % 5 == 0 || i % 6 == 0) {
			for (int k = 0; k < 2; k++) {
				pimax_hands_pose head, out;
				to_pose(hq, head_p, &head);
				pimax_ctrlfuse_info info;
				memset(&info, 0x77, sizeof info);
				memset(&out, 0, sizeof out);
				const uint64_t at = host + 12000000ull;
				const int st = pimax_ctrlfuse_get(f[k], at, &head, &out, (i % 7) ? &info : NULL);
				gethash = fnv(gethash, &st, sizeof st);
				gethash = fnv(gethash, &out, sizeof out);
				gethash = fnv(gethash, &info, sizeof info);
				nget++;
				if (st != last_state[k] || nget % 61 == 0) {
					char tag[96];
					snprintf(tag, sizeof tag, "%ld h%d st %d info %u %u", i, k, st, info.state, info.gyro);
					print_pose(tag, &out);
					last_state[k] = st;
				}
			}
		}
		if (i % 5000 == 0) {
			for (int k = 0; k < 2; k++) {
				pimax_ctrlfuse_describe(f[k], b, sizeof b);
				printf("%ld %d: %s\n", i, k, b);
			}
		}
	}
	printf("gets %ld hash %016llx\n", nget, (unsigned long long)gethash);
	for (int k = 0; k < 2; k++) {
		pimax_ctrlfuse_describe(f[k], b, sizeof b);
		printf("end %d: %s\n", k, b);
	}
	/* small buffers */
	for (size_t len = 1; len < 6; len++) {
		char sb[8];
		size_t r = pimax_ctrlfuse_describe(f[0], sb, len);
		printf("describe len %zu -> %zu [%s]\n", len, r, sb);
	}
	/* the left hand goes first; the right one still sees it */
	pimax_ctrlfuse_destroy(f[0]);
	pimax_hands_pose head = {{0, 0, 0, 1}, {0, 0, 0}}, out;
	int st = pimax_ctrlfuse_get(f[1], T0 + (uint64_t)(seconds * 1e9), &head, &out, NULL);
	printf("after destroy %d\n", st);
	print_pose("right", &out);
	pimax_ctrlfuse_destroy(f[1]);
	(void)dumpdir;
	return 0;
}

static int mode_fuse_nulls(void)
{
	pimax_hands_pose p = {{0, 0, 0, 1}, {0, 0, -0.4f}}, out;
	pimax_ctrlfuse_info info;
	uint8_t r[64] = {0};
	pimax_ctrlfuse_destroy(NULL);
	pimax_ctrlfuse_report(NULL, 1, r);
	pimax_ctrlfuse_optical(NULL, 1, &p, &p, 1, 10);
	pimax_ctrlfuse_link(NULL, NULL);
	printf("get NULL %d\n", pimax_ctrlfuse_get(NULL, 1, &p, &out, &info));
	char b[64];
	printf("describe NULL %zu\n", pimax_ctrlfuse_describe(NULL, b, sizeof b));
	for (int hand = -1; hand < 4; hand++) {
		pimax_ctrlfuse *f = pimax_ctrlfuse_create(hand);
		pimax_ctrlfuse_link(f, f);
		pimax_ctrlfuse_link(f, NULL);
		pimax_ctrlfuse_report(f, 1, NULL);
		pimax_ctrlfuse_optical(f, 1, NULL, &p, 1, 10);
		pimax_ctrlfuse_optical(f, 1, &p, NULL, 1, 10);
		printf("get before %d\n", pimax_ctrlfuse_get(f, 5, &p, &out, &info));
		print_pose("  out", &out);
		printf("  info %u %u\n", info.state, info.gyro);
		printf("get NULL head %d NULL out %d\n", pimax_ctrlfuse_get(f, 5, NULL, &out, &info),
		       pimax_ctrlfuse_get(f, 5, &p, NULL, &info));
		/* a hand: left at x -0.2, right at +0.2; a zero quaternion and a NaN-free odd one */
		pimax_hands_pose a = {{0, 0, 0, 0}, {hand == 0 ? -0.2f : 0.2f, -0.2f, -0.4f}};
		for (int i = 0; i < 30; i++) {
			a.orientation[3] = (float)i * 0.1f;
			pimax_ctrlfuse_optical(f, 1000000000ull + (uint64_t)i * 40000000ull, &a, &p, (float)i * 0.2f, (uint32_t)i);
			const int st = pimax_ctrlfuse_get(f, 1010000000ull + (uint64_t)i * 40000000ull, &p, &out, &info);
			printf("  %d st %d info %u %u", i, st, info.state, info.gyro);
			print_pose("", &out);
		}
		for (uint64_t t = 2000000000ull; t < 9000000000ull; t += 700000000ull) {
			const int st = pimax_ctrlfuse_get(f, t, &p, &out, &info);
			printf("  t %llu st %d", (unsigned long long)t, st);
			print_pose("", &out);
		}
		/* extreme times */
		const uint64_t odd[] = {0, 1, 18446744073709551615ull, 9223372036854775808ull, 9223372036854775807ull};
		for (int i = 0; i < 5; i++) {
			const int st = pimax_ctrlfuse_get(f, odd[i], &p, &out, &info);
			printf("  odd %d st %d", i, st);
			print_pose("", &out);
			pimax_ctrlfuse_optical(f, odd[i], &a, &p, 0.5f, 12);
			make_report(r, 2, (uint32_t)odd[i], (int16_t[]){1, 2, 3}, (int16_t[]){0, 1024, 0}, NULL, NULL, 0, 0, 0, 0, 0, 0);
			pimax_ctrlfuse_report(f, odd[i], r);
		}
		size_t n = pimax_ctrlfuse_describe(f, b, sizeof b);
		printf("describe %zu %s\n", n, b);
		printf("describe len0 %zu\n", pimax_ctrlfuse_describe(f, b, 0));
		pimax_ctrlfuse_destroy(f);
	}
	return 0;
}

/* ---------- pimax-hands tests/c/session.c ---------- */

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static int mode_session(void)
{
	CHECK(pimax_hands_abi_version() == PIMAX_HANDS_ABI_VERSION, "abi %u", pimax_hands_abi_version());
	pimax_hands *h = pimax_hands_create();
	CHECK(h != NULL, "create");
	pimax_hands_out out;
	memset(&out, 0xff, sizeof out);
	pimax_hands_get(h, &out);
	CHECK(out.thumb_state == PIMAX_HANDS_THUMB_REST, "rest state %u", out.thumb_state);
	CHECK(out.curl[3] < 0.2f && out.curl[2] < 0.4f, "rest curls %f %f", out.curl[3], out.curl[2]);
	pimax_hands_input in = {.trigger = 1.0f, .grip = 1.0f, .stick_x = 0.0f, .stick_y = 0.0f, .flags = PIMAX_HANDS_TOUCH_STICK};
	uint64_t t = 5000ull * 1000 * 1000;
	for (int i = 0; i < 300; i++) {
		t += 4ull * 1000 * 1000;
		pimax_hands_update(h, t, &in);
	}
	pimax_hands_get(h, &out);
	CHECK(out.thumb_state == PIMAX_HANDS_THUMB_STICK, "thumb state %u", out.thumb_state);
	CHECK(fabsf(out.curl[2] - 1.0f) < 0.01f, "middle %f", out.curl[2]);
	CHECK(out.curl[0] == out.curl[1] && out.curl[1] == out.curl[2], "lower fingers differ");
	CHECK(out.curl[3] > 0.7f && out.curl[3] < 0.9f, "index %f", out.curl[3]);
	CHECK(out.curl[4] > 0.4f && out.curl[4] < 0.7f, "thumb %f", out.curl[4]);
	CHECK(fabsf(out.index.joint_curls[1] - 1.43f * out.curl[3]) < 1e-3f, "index proximal %f", out.index.joint_curls[1]);
	CHECK(fabsf(out.middle.joint_curls[1] - 1.859f * out.curl[2]) < 1e-3f, "middle proximal %f", out.middle.joint_curls[1]);
	CHECK(out.thumb.splay == -0.35f, "thumb splay %f", out.thumb.splay);
	pimax_hands_reset(h);
	for (int i = 0; i < 300; i++) {
		t += 4ull * 1000 * 1000;
		pimax_hands_update_report(h, t, 255, 255, 0, 0, 0x02, 0x01);
	}
	pimax_hands_get(h, &out);
	CHECK(out.thumb_state == PIMAX_HANDS_THUMB_BTN1, "report path thumb state %u", out.thumb_state);
	CHECK(fabsf(out.curl[2] - 1.0f) < 0.01f && out.curl[3] > 0.8f, "report path curls %f %f", out.curl[2], out.curl[3]);
	CHECK(out.curl[4] > 0.55f, "A pressed: thumb deeper than a touch (%f)", out.curl[4]);
	pimax_hands_reset(h);
	pimax_hands_get(h, &out);
	CHECK(out.thumb_state == PIMAX_HANDS_THUMB_REST && out.curl[2] < 0.4f, "after reset");
	pimax_hands_pose l, r;
	pimax_hands_grip_to_wrist(0, &l);
	pimax_hands_grip_to_wrist(1, &r);
	CHECK(l.position[1] > 0.05f && l.position[1] < 0.08f, "left y %f", l.position[1]);
	CHECK(fabsf(l.position[0] + r.position[0]) < 1e-6f && fabsf(l.position[1] - r.position[1]) < 1e-6f, "mirror");
	CHECK(fabsf(l.orientation[0] * l.orientation[0] + l.orientation[1] * l.orientation[1] + l.orientation[2] * l.orientation[2] +
	            l.orientation[3] * l.orientation[3] - 1.0f) < 1e-5f, "unit quaternion");
	pimax_hands_input d;
	pimax_hands_demo_input(2000ull * 1000 * 1000, 0, &d);
	CHECK(d.grip > 0.99f && d.trigger == 0.0f, "demo at 2 s: grip %f trigger %f", d.grip, d.trigger);
	pimax_hands_update_report(NULL, 1, 0, 0, 0, 0, 0, 0);
	pimax_hands_update(NULL, 1, &in);
	pimax_hands_update(h, 1, NULL);
	pimax_hands_get(NULL, &out);
	pimax_hands_get(h, NULL);
	pimax_hands_grip_to_wrist(0, NULL);
	pimax_hands_demo_input(0, 0, NULL);
	pimax_hands_reset(NULL);
	pimax_hands_destroy(NULL);
	pimax_hands_destroy(h);
	if (fails == 0) printf("PASS\n");
	return fails != 0;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: hands_difftest MODE ...\n");
		return 2;
	}
	const char *m = argv[1];
	if (!strcmp(m, "layout")) return mode_layout();
	if (!strcmp(m, "hands") && argc >= 4) return mode_hands(strtoull(argv[2], NULL, 10), atoi(argv[3]));
	if (!strcmp(m, "poses")) return mode_poses();
	if (!strcmp(m, "replay") && argc >= 3) return mode_replay(argv[2]);
	if (!strcmp(m, "inhand_csv") && argc >= 4) return mode_inhand_csv(argv[2], argv[3]);
	if (!strcmp(m, "inhand_synth") && argc >= 4) return mode_inhand_synth(strtoull(argv[2], NULL, 10), argv[3]);
	if (!strcmp(m, "fuse") && argc >= 5)
		return mode_fuse(atoi(argv[2]), strtoull(argv[3], NULL, 10), atof(argv[4]), argc > 5 ? argv[5] : NULL);
	if (!strcmp(m, "fuse_nulls")) return mode_fuse_nulls();
	if (!strcmp(m, "session")) return mode_session();
	fprintf(stderr, "bad mode %s\n", m);
	return 2;
}
