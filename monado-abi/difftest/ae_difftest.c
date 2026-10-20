// SPDX-License-Identifier: MIT
// Difftest driver for the auto-exposure part of pimax_cam.h (docs/research/rust-port-plan.md, M3). The same C program
// is compiled twice, against omarchyvr's libpimax_cam.a (the oracle) and against PiBridge's libpibridge_pimax.a;
// run_difftest.sh compares the two outputs byte for byte. stderr is folded into stdout (the PIMAX_AE_MODEL /
// PIMAX_AE_EXTENDED / PIMAX_AE_LIGHT_LOW lines are part of the ABI), floats as exact hex (%a).
//
// pimax_cam_ae_update only acts on live frames (a replay is flagged and skipped), so the program runs its own tiny
// camera broker (PXCB v1, the protocol of pimax-cam broker, without FLAG_REPLAY) in a scratch directory and opens it
// through pimax_cam_open("broker:SOCKET"): every frame the library sees is a "live" frame, published in lock step.
//
//   ae_difftest layout
//   ae_difftest create                     pimax_cam_ae_create / _create_model with names, env and error texts
//   ae_difftest errors WORKDIR             NULL handles and outputs, a camera without a frame, replay frames, LED frames
//   ae_difftest pxcam WORKDIR MODEL FILE [N]   the frames of a recording (open loop: their own metadata), now = the
//                                          recorded host time
//   ae_difftest synth WORKDIR MODEL SEED SECONDS   closed loop on a simulated sensor: random rooms (dark to sunlit,
//                                          windows, light blobs, an upper pair of its own), 25 Hz tracking + LED
//                                          frames, every command applied two tracking frames later as on Windows
//   ae_difftest scene WORKDIR              the head tracker's brightness input (PIMAX_FUSE_AUTO): the same walk once
//                                          with bright and once with dark AE frames
//
// MODEL: crystal-super, dream-air, "" (detect) or "-" (pimax_cam_ae_create). For the Crystal Super every payload that
// the library hands out for sending must lie inside Windows' pair ranges (lower 2200..10410 us / gain 13..449, upper
// 2200..8941 / 13..439, LED 40/62): the run counts violations and exits 3 when there is one (both libraries).
#define _GNU_SOURCE
#include "pimax_cam.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <zlib.h>

#define O(t, f) (unsigned long)offsetof(t, f)
#define FW 2560
#define FH 832
#define FBYTES (FW * FH)
#define PAGE 4096
#define SLOTS 8
#define SLOT_BYTES (((64 + FBYTES) + PAGE - 1) / PAGE * PAGE)

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}
static void put32(uint8_t *p, uint32_t v)
{
	for (int i = 0; i < 4; i++)
		p[i] = (uint8_t)(v >> (8 * i));
}
static void put64(uint8_t *p, uint64_t v)
{
	for (int i = 0; i < 8; i++)
		p[i] = (uint8_t)(v >> (8 * i));
}
static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static uint64_t get64(const uint8_t *p)
{
	return (uint64_t)get32(p) | (uint64_t)get32(p + 4) << 32;
}

static const char *err(void)
{
	const char *e = pimax_cam_last_error();
	return e ? e : "(null)";
}

/* ---- the mini broker ---- */

struct broker {
	int lsock, csock, memfd;
	uint8_t *ring;
	size_t ring_len;
	uint32_t flags;
	uint64_t k;
	char path[200];
};

static void *accept_one(void *arg)
{
	struct broker *b = arg;
	b->csock = accept4(b->lsock, NULL, NULL, SOCK_CLOEXEC);
	if (b->csock < 0)
		return NULL;
	uint8_t h[4 + 6 * 4 + 16];
	memcpy(h, "PXCB", 4);
	const char *desc = "ae_difftest";
	put32(h + 4, 1);
	put32(h + 8, SLOTS);
	put32(h + 12, SLOT_BYTES);
	put32(h + 16, FBYTES);
	put32(h + 20, b->flags);
	put32(h + 24, (uint32_t)strlen(desc));
	memcpy(h + 28, desc, strlen(desc));
	struct iovec iov = {h, 28 + strlen(desc)};
	char cbuf[CMSG_SPACE(sizeof(int))];
	memset(cbuf, 0, sizeof cbuf);
	struct msghdr m = {0};
	m.msg_iov = &iov;
	m.msg_iovlen = 1;
	m.msg_control = cbuf;
	m.msg_controllen = sizeof cbuf;
	struct cmsghdr *c = CMSG_FIRSTHDR(&m);
	c->cmsg_level = SOL_SOCKET;
	c->cmsg_type = SCM_RIGHTS;
	c->cmsg_len = CMSG_LEN(sizeof(int));
	memcpy(CMSG_DATA(c), &b->memfd, sizeof(int));
	if (sendmsg(b->csock, &m, 0) < 0)
		perror("sendmsg");
	return NULL;
}

/* Start the broker in dir and connect the library's camera to it. */
static pimax_cam *broker_open(struct broker *b, const char *dir, uint32_t flags)
{
	memset(b, 0, sizeof *b);
	b->flags = flags;
	b->csock = -1;
	snprintf(b->path, sizeof b->path, "%s/s", dir);
	unlink(b->path);
	b->lsock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	struct sockaddr_un a = {0};
	a.sun_family = AF_UNIX;
	snprintf(a.sun_path, sizeof a.sun_path, "%s", b->path);
	if (bind(b->lsock, (struct sockaddr *)&a, sizeof a) != 0 || listen(b->lsock, 4) != 0) {
		perror("broker socket");
		exit(2);
	}
	b->ring_len = PAGE + (size_t)SLOTS * SLOT_BYTES;
	b->memfd = memfd_create("ae_difftest", MFD_CLOEXEC);
	if (b->memfd < 0 || ftruncate(b->memfd, (off_t)b->ring_len) != 0) {
		perror("memfd");
		exit(2);
	}
	b->ring = mmap(NULL, b->ring_len, PROT_READ | PROT_WRITE, MAP_SHARED, b->memfd, 0);
	if (b->ring == MAP_FAILED) {
		perror("mmap");
		exit(2);
	}
	memcpy(b->ring, "PXCB", 4);
	put32(b->ring + 4, 1);
	__atomic_store_n((uint64_t *)(b->ring + 88), 1, __ATOMIC_RELEASE); /* camera streaming */
	pthread_t th;
	pthread_create(&th, NULL, accept_one, b);
	char spec[220];
	snprintf(spec, sizeof spec, "broker:%s", b->path);
	pimax_cam *cam = pimax_cam_open(spec, NULL);
	pthread_join(th, NULL);
	if (!cam) {
		printf("open %s: %s\n", spec, err());
		exit(2);
	}
	return cam;
}

/* Publish one raw frame and let the library take it (lock step). Returns pimax_cam_next's result. */
static int broker_push(struct broker *b, pimax_cam *cam, const uint8_t *frame, uint64_t host_ns, uint32_t seq,
                       pimax_cam_frame_info *info)
{
	const uint64_t k = b->k++;
	uint8_t *s = b->ring + PAGE + (size_t)(k % SLOTS) * SLOT_BYTES;
	__atomic_store_n((uint64_t *)s, 2 * k + 1, __ATOMIC_RELAXED);
	__atomic_thread_fence(__ATOMIC_RELEASE);
	put64(s + 8, host_ns);
	put32(s + 16, seq);
	put32(s + 20, FBYTES);
	put32(s + 24, FW);
	put64(s + 32, k);
	memcpy(s + 64, frame, FBYTES);
	__atomic_store_n((uint64_t *)s, 2 * k + 2, __ATOMIC_RELEASE);
	__atomic_store_n((uint64_t *)(b->ring + 64), k + 1, __ATOMIC_RELEASE);
	__atomic_fetch_add((uint64_t *)(b->ring + 72), 1, __ATOMIC_ACQ_REL);
	__atomic_store_n((uint64_t *)(b->ring + 96), host_ns, __ATOMIC_RELEASE);
	uint8_t n[8];
	put64(n, k);
	if (send(b->csock, n, 8, MSG_DONTWAIT) != 8) {
		perror("notice");
		exit(2);
	}
	return pimax_cam_next(cam, 5000, info);
}

static void broker_close(struct broker *b, pimax_cam *cam)
{
	pimax_cam_close(cam);
	if (b->csock >= 0)
		close(b->csock);
	close(b->lsock);
	munmap(b->ring, b->ring_len);
	close(b->memfd);
	unlink(b->path);
}

/* ---- synthetic frames ---- */

/* Metadata row (frame.rs layout): type 0x86 tracking / 0x06 LED, device us, counter, exposure, gain, tile rows. */
static void meta(uint8_t *f, int tracking, uint64_t ts_us, uint32_t counter, const uint16_t e[4], const uint16_t g[4],
                 uint16_t yoff)
{
	memset(f, 0, 96);
	for (int i = 0; i < 12; i++)
		f[i] = (uint8_t)((counter >> i) & 1);
	f[32] = tracking ? 0x86 : 0x06;
	put64(f + 33, ts_us);
	put32(f + 41, counter);
	for (int i = 0; i < 4; i++) {
		put16(f + 45 + 2 * i, e[i]);
		put16(f + 53 + 2 * i, g[i]);
		put16(f + 61 + 2 * i, yoff);
	}
	f[92] = 0x3c;
	f[93] = 0x3d;
	f[94] = 0x3e;
	f[95] = 0x3f;
}

static double tex[480][640];

static void init_tex(void)
{
	for (int y = 0; y < 480; y++)
		for (int x = 0; x < 640; x++)
			tex[y][x] = 0.6 + 0.4 * (double)((x / 16 + y / 16) % 2) + 0.1 * sin((double)x * 0.1);
}

struct room {
	double radiance; /* lower pair */
	double upper;    /* upper pair factor */
	double window;   /* fraction of the width that is a 20x brighter window */
	int blob;        /* a light blob */
};

static uint8_t clamp8(double v)
{
	v = round(v);
	if (!(v > 0.0))
		return 0;
	if (v > 255.0)
		return 255;
	return (uint8_t)v;
}

/* One tile: pixel = clamp(radiance * texture * e * g / (2000 * 13)), black lens corners, optional window and blob. */
static void render(uint8_t *f, int tile, uint16_t yoff, double rad, const struct room *r, uint16_t e, uint16_t g)
{
	const double c = (double)e * (double)g / (2000.0 * 13.0);
	const int x0 = (int)(640.0 * (1.0 - r->window));
	for (int y = 0; y < 480; y++) {
		uint8_t *row = f + (size_t)(yoff + y) * FW + (size_t)tile * 640;
		for (int x = 0; x < 640; x++) {
			const double dx = ((double)x - 319.5) / 320.0, dy = ((double)y - 239.5) / 240.0;
			double v = rad * tex[y][x] * c;
			if (dx * dx + dy * dy > 1.0)
				v = 0.0;
			else if (x >= x0)
				v = 20.0 * rad * 0.8 * c;
			if (r->blob && (x - 500) * (x - 500) + (y - 100) * (y - 100) < 900)
				v = 1e4;
			row[x] = clamp8(v);
		}
	}
}

static uint64_t lcg_state;
static double rnd(void)
{
	lcg_state = lcg_state * 6364136223846793005ull + 1442695040888963407ull;
	return (double)(lcg_state >> 11) / 9007199254740992.0;
}

/* ---- output ---- */

static unsigned long violations, sent, targets_out;
/* The run is a Crystal Super run (by name or PIMAX_CAM_AE_MODEL): out-of-range payloads fail it. */
static int crystal;

static int in_ranges(const uint8_t *p)
{
	uint16_t w[10];
	for (int i = 0; i < 10; i++)
		w[i] = (uint16_t)(p[2 + 2 * i] | p[3 + 2 * i] << 8);
	if (p[0] != 0x01 || p[1] != 0x32)
		return 0;
	for (int c = 0; c < 4; c++) {
		const uint16_t emax = c < 2 ? 10410 : 8941, gmax = c < 2 ? 449 : 439;
		if (w[c] < 2200 || w[c] > emax || w[4 + c] < 13 || w[4 + c] > gmax)
			return 0;
	}
	return w[8] == 40 && w[9] == 62;
}

static void print_out(const char *tag, int r, const pimax_cam_ae_out *o)
{
	printf("%s r=%d send=%u changed=%u n=%" PRIu64 "/%" PRIu64 "/%" PRIu64 " p=", tag, r, o->send, o->changed,
	       o->frames, o->sends, o->changes);
	for (int i = 0; i < PIMAX_CAM_AE_PAYLOAD_LEN; i++)
		printf("%02x", o->payload[i]);
	printf(" pad=%02x%02x t=%u,%u,%u,%u/%u,%u,%u,%u led=%u/%u f=%u,%u,%u,%u/%u,%u,%u,%u m=%a,%a s=%a,%a e=%a,%a l=%u\n",
	       o->pad[0], o->pad[1], o->exposure[0], o->exposure[1], o->exposure[2], o->exposure[3], o->gain[0], o->gain[1],
	       o->gain[2], o->gain[3], o->led_exposure, o->led_gain, o->frame_exposure[0], o->frame_exposure[1],
	       o->frame_exposure[2], o->frame_exposure[3], o->frame_gain[0], o->frame_gain[1], o->frame_gain[2],
	       o->frame_gain[3], (double)o->mean[0], (double)o->mean[1], (double)o->sat[0], (double)o->sat[1],
	       (double)o->err[0], (double)o->err[1], o->light);
	if (r == 1) {
		sent++;
		if (!in_ranges(o->payload)) {
			violations++;
			if (crystal)
				printf("VIOLATION: payload outside the Crystal pair ranges\n");
		}
	}
	if (!in_ranges(o->payload))
		targets_out++;
}

static pimax_cam_ae *make(const char *model)
{
	pimax_cam_ae *a = strcmp(model, "-") == 0 ? pimax_cam_ae_create() : pimax_cam_ae_create_model(model);
	if (!a) {
		printf("create %s: NULL (%s)\n", model, err());
		exit(4);
	}
	const char *env = getenv("PIMAX_CAM_AE_MODEL");
	crystal = strstr(model, "crystal") != NULL || (env && strstr(env, "crystal") != NULL);
	return a;
}

static int finish(void)
{
	printf("sent %lu, outside the Crystal ranges: %lu sent, %lu targets%s\n", sent, violations, targets_out,
	       crystal ? " (Crystal Super run: must be 0 sent)" : "");
	return violations && crystal ? 3 : 0;
}

/* ---- modes ---- */

static int mode_layout(void)
{
	printf("ae_out %lu: send %lu changed %lu frames %lu sends %lu changes %lu payload %lu pad %lu exposure %lu gain %lu "
	       "led_exposure %lu led_gain %lu frame_exposure %lu frame_gain %lu mean %lu sat %lu err %lu light %lu\n",
	       (unsigned long)sizeof(pimax_cam_ae_out), O(pimax_cam_ae_out, send), O(pimax_cam_ae_out, changed),
	       O(pimax_cam_ae_out, frames), O(pimax_cam_ae_out, sends), O(pimax_cam_ae_out, changes),
	       O(pimax_cam_ae_out, payload), O(pimax_cam_ae_out, pad), O(pimax_cam_ae_out, exposure),
	       O(pimax_cam_ae_out, gain), O(pimax_cam_ae_out, led_exposure), O(pimax_cam_ae_out, led_gain),
	       O(pimax_cam_ae_out, frame_exposure), O(pimax_cam_ae_out, frame_gain), O(pimax_cam_ae_out, mean),
	       O(pimax_cam_ae_out, sat), O(pimax_cam_ae_out, err), O(pimax_cam_ae_out, light));
	printf("cmd %#x opt %#x len %d version %s\n", PIMAX_CAM_AE_CMD, PIMAX_CAM_AE_OPT, PIMAX_CAM_AE_PAYLOAD_LEN,
	       pimax_cam_version());
	return 0;
}

static int mode_create(void)
{
	static const char *names[] = {"crystal-super", "dream-air", "", " ", "Crystal_Super", " crystal super ", "CRYSTAL",
	                              "0x0040", "0040", "dreamair", "0x0044", "8kx", "crystal-super-x", "\xff\xfe"};
	for (size_t i = 0; i < sizeof names / sizeof names[0]; i++) {
		pimax_cam_ae *a = pimax_cam_ae_create_model(names[i]);
		printf("create_model \"%s\": %s %s\n", names[i], a ? "ok" : "NULL", a ? "" : err());
		pimax_cam_ae_destroy(a);
	}
	pimax_cam_ae *a = pimax_cam_ae_create_model(NULL);
	printf("create_model NULL: %s\n", a ? "ok" : "NULL");
	pimax_cam_ae_destroy(a);
	a = pimax_cam_ae_create();
	printf("create: %s %s\n", a ? "ok" : "NULL", a ? "" : err());
	pimax_cam_ae_destroy(a);
	pimax_cam_ae_destroy(NULL);
	return 0;
}

static uint8_t frame[FBYTES];

static int mode_errors(const char *dir)
{
	pimax_cam_ae_out o;
	memset(&o, 0x5a, sizeof o);
	pimax_cam_ae *a = pimax_cam_ae_create_model("crystal-super");
	/* (the result first: pimax_cam_last_error's pointer is only valid until the next error) */
	int u = pimax_cam_ae_update(NULL, NULL, 0, &o);
	printf("null ae: %d %s\n", u, err());
	u = pimax_cam_ae_update(a, NULL, 0, &o);
	printf("null cam: %d %s\n", u, err());
	struct broker b;
	pimax_cam *cam = broker_open(&b, dir, 0);
	u = pimax_cam_ae_update(a, cam, 0, NULL);
	printf("null out: %d %s\n", u, err());
	printf("no frame yet: %d\n", pimax_cam_ae_update(a, cam, 0, &o));
	pimax_cam_frame_info fi;
	const uint16_t e[4] = {3500, 3500, 3500, 3500}, g[4] = {60, 60, 60, 60};
	/* an LED frame, a frame without metadata, then a tracking frame */
	memset(frame, 0, sizeof frame);
	meta(frame, 0, 1000, 1, e, g, 10);
	int r = broker_push(&b, cam, frame, 1000000000ull, 1, &fi);
	printf("led: next %d kind %u -> %d\n", r, fi.kind, pimax_cam_ae_update(a, cam, 1000000000ull, &o));
	memset(frame, 0, sizeof frame);
	r = broker_push(&b, cam, frame, 1020000000ull, 2, &fi);
	printf("no meta: next %d kind %u flags %u -> %d\n", r, fi.kind, fi.flags, pimax_cam_ae_update(a, cam, 1020000000ull, &o));
	init_tex();
	struct room rm = {30.0, 1.0, 0.0, 0};
	meta(frame, 1, 41000, 3, e, g, 278);
	for (int t = 0; t < 4; t++)
		render(frame, t, 278, rm.radiance, &rm, e[t], g[t]);
	r = broker_push(&b, cam, frame, 1040000000ull, 3, &fi);
	memset(&o, 0x5a, sizeof o);
	u = pimax_cam_ae_update(a, cam, 1040000000ull, &o);
	print_out("tracking", u, &o);
	broker_close(&b, cam);
	/* a replay broker: frames are flagged and never drive the loop */
	cam = broker_open(&b, dir, 1);
	r = broker_push(&b, cam, frame, 2000000000ull, 4, &fi);
	printf("replay: next %d flags %u -> %d\n", r, fi.flags, pimax_cam_ae_update(a, cam, 2000000000ull, &o));
	broker_close(&b, cam);
	pimax_cam_ae_destroy(a);
	return 0;
}

static int mode_pxcam(const char *dir, const char *model, const char *path, long max)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		printf("open %s failed\n", path);
		return 2;
	}
	uint8_t hdr[32];
	if (fread(hdr, 1, 32, f) != 32 || memcmp(hdr, "PXCAMRAW", 8) != 0) {
		printf("not a .pxcam\n");
		return 2;
	}
	pimax_cam_ae *a = make(model);
	struct broker b;
	pimax_cam *cam = broker_open(&b, dir, 0);
	static uint8_t zbuf[FBYTES + 65536];
	long n = 0;
	uint8_t rh[32];
	while (fread(rh, 1, 32, f) == 32) {
		const uint32_t flags = get32(rh + 4), stored = get32(rh + 8), raw = get32(rh + 12);
		const uint64_t host = get64(rh + 16);
		const uint32_t seq = get32(rh + 24);
		if (get32(rh) != 0x4d415246 || raw != FBYTES || stored > sizeof zbuf)
			break;
		if (flags & 1) {
			if (fread(zbuf, 1, stored, f) != stored)
				break;
			uLongf out = FBYTES;
			if (uncompress(frame, &out, zbuf, stored) != Z_OK || out != FBYTES)
				break;
		} else if (fread(frame, 1, stored, f) != stored) {
			break;
		}
		pimax_cam_frame_info fi;
		const int r = broker_push(&b, cam, frame, host, seq, &fi);
		pimax_cam_ae_out o;
		memset(&o, 0, sizeof o);
		const int u = pimax_cam_ae_update(a, cam, host, &o);
		if (u != 0 || fi.kind == 1) {
			char tag[64];
			snprintf(tag, sizeof tag, "%ld next=%d kind=%u", n, r, fi.kind);
			print_out(tag, u, &o);
		}
		if (++n == max)
			break;
	}
	fclose(f);
	broker_close(&b, cam);
	pimax_cam_ae_destroy(a);
	printf("%ld frames\n", n);
	return finish();
}

static int mode_synth(const char *dir, const char *model, uint64_t seed, int seconds)
{
	init_tex();
	lcg_state = seed * 0x9e3779b97f4a7c15ull + 1;
	pimax_cam_ae *a = make(model);
	struct broker b;
	pimax_cam *cam = broker_open(&b, dir, 0);
	/* the cameras start at a firmware default (Crystal 3500/60, Dream Air 10410/449, or a random Windows value) */
	uint16_t e[4], g[4];
	const double s0 = rnd();
	const uint16_t e0 = s0 < 0.4 ? 3500 : s0 < 0.7 ? 10410 : (uint16_t)(1000 + rnd() * 9000);
	const uint16_t g0 = s0 < 0.4 ? 60 : s0 < 0.7 ? 449 : (uint16_t)(1 + rnd() * 500);
	for (int i = 0; i < 4; i++) {
		e[i] = e0;
		g[i] = g0;
	}
	struct pend {
		long at;
		uint16_t e[4], g[4];
	} pend[64];
	int npend = 0;
	struct room rm = {30.0, 1.0, 0.0, 0};
	long seg_end = 0;
	const long frames = (long)seconds * 50;
	uint64_t host = 5000000000ull + seed * 1000;
	long tracking = 0;
	for (long i = 0; i < frames; i++) {
		/* host clock steps of 20 ms with jitter, sometimes a gap (camera restart: > 2 s) */
		host += 20000000ull + (uint64_t)(rnd() * 2000000.0);
		if (rnd() < 0.0005)
			host += 2500000000ull;
		const int is_tracking = (i % 2) == 0;
		memset(frame, 0, sizeof frame);
		if (is_tracking) {
			if (tracking >= seg_end) {
				/* a new room: log-uniform radiance 0.03..3000, sometimes a window or a blob */
				rm.radiance = exp(log(0.03) + rnd() * (log(3000.0) - log(0.03)));
				rm.upper = 0.3 + rnd() * 1.2;
				rm.window = rnd() < 0.3 ? rnd() * 0.6 : 0.0;
				rm.blob = rnd() < 0.3;
				seg_end = tracking + 25 * (2 + (long)(rnd() * 14.0));
				printf("room at %ld: radiance %a upper %a window %a blob %d\n", tracking, rm.radiance, rm.upper,
				       rm.window, rm.blob);
			}
			for (int p = 0; p < npend;) {
				if (pend[p].at <= tracking) {
					memcpy(e, pend[p].e, sizeof e);
					memcpy(g, pend[p].g, sizeof g);
					memmove(&pend[p], &pend[p + 1], sizeof pend[0] * (size_t)(npend - p - 1));
					npend--;
				} else {
					p++;
				}
			}
			meta(frame, 1, host / 1000, (uint32_t)i, e, g, 278);
			render(frame, 0, 278, rm.radiance, &rm, e[0], g[0]);
			for (int y = 0; y < 480; y++)
				memcpy(frame + (size_t)(278 + y) * FW + 640, frame + (size_t)(278 + y) * FW, 640);
			struct room up = rm;
			up.blob = 0;
			up.window = rm.window * 0.5;
			render(frame, 2, 278, rm.radiance * rm.upper, &up, e[2], g[2]);
			for (int y = 0; y < 480; y++)
				memcpy(frame + (size_t)(278 + y) * FW + 1920, frame + (size_t)(278 + y) * FW + 1280, 640);
		} else {
			const uint16_t le[4] = {40, 40, 40, 40}, lg[4] = {62, 62, 62, 62};
			meta(frame, 0, host / 1000, (uint32_t)i, le, lg, 10);
			for (int t = 0; t < 4; t++)
				frame[(size_t)(10 + 200) * FW + (size_t)t * 640 + 300] = 255;
		}
		pimax_cam_frame_info fi;
		const int r = broker_push(&b, cam, frame, host, (uint32_t)i, &fi);
		pimax_cam_ae_out o;
		memset(&o, 0, sizeof o);
		const int u = pimax_cam_ae_update(a, cam, host, &o);
		if (r != 1 || (!is_tracking && u != 0))
			printf("frame %ld: next %d update %d (%s)\n", i, r, u, err());
		if (is_tracking) {
			char tag[48];
			snprintf(tag, sizeof tag, "%ld", tracking);
			print_out(tag, u, &o);
			if (u == 1 && npend < 64) {
				pend[npend].at = tracking + 2;
				memcpy(pend[npend].e, o.exposure, sizeof e);
				memcpy(pend[npend].g, o.gain, sizeof g);
				npend++;
			}
			tracking++;
		}
	}
	broker_close(&b, cam);
	pimax_cam_ae_destroy(a);
	return finish();
}

static int mode_scene(const char *dir)
{
	init_tex();
	pimax_cam_ae *a = pimax_cam_ae_create_model("crystal-super");
	struct broker b;
	pimax_cam *cam = broker_open(&b, dir, 0);
	double first[400][3];
	for (int run = 0; run < 2; run++) {
		pimax_cam_track *t = pimax_cam_track_create(NULL, NULL);
		struct room rm = {run == 0 ? 60.0 : 0.05, 1.0, 0.0, 0};
		const uint16_t e[4] = {3500, 3500, 3500, 3500}, g[4] = {60, 60, 60, 60};
		lcg_state = 7;
		int differ = 0;
		for (int i = 0; i < 400; i++) {
			const uint64_t now = 1000000000ull + (uint64_t)i * 40000000ull;
			memset(frame, 0, sizeof frame);
			meta(frame, 1, now / 1000, (uint32_t)i, e, g, 278);
			for (int k = 0; k < 4; k++)
				render(frame, k, 278, rm.radiance, &rm, e[k], g[k]);
			pimax_cam_frame_info fi;
			broker_push(&b, cam, frame, now, (uint32_t)(run * 1000 + i), &fi);
			pimax_cam_ae_out o;
			pimax_cam_ae_update(a, cam, now, &o);
			/* the head sways along x (0.5 m, 1 rad/s) and turns its yaw (0.3 rad, 0.8 rad/s): IMU at 1 kHz */
			for (int s = 0; s < 40; s++) {
				const double ts = (double)(now - 40000000ull + (uint64_t)s * 1000000ull) * 1e-9;
				const double ax = -0.5 * sin(ts), yaw = 0.3 * sin(0.8 * ts);
				const double acc[3] = {cos(yaw) * ax + 0.02 * (rnd() - 0.5), 9.80665 + 0.02 * (rnd() - 0.5),
				                       sin(yaw) * ax + 0.02 * (rnd() - 0.5)};
				const double gyr[3] = {0.002 * (rnd() - 0.5), 0.24 * cos(0.8 * ts) + 0.002 * (rnd() - 0.5),
				                       0.002 * (rnd() - 0.5)};
				pimax_cam_track_push_imu(t, (int64_t)(now - 40000000ull + (uint64_t)s * 1000000ull), acc, gyr);
			}
			/* the tracker pose of 25 ms ago, with noise */
			const double tv = (double)(now - 25000000ull) * 1e-9, yaw = 0.3 * sin(0.8 * tv);
			pimax_cam_track_vio v;
			memset(&v, 0, sizeof v);
			v.ts_ns = now - 25000000ull;
			v.pos[0] = 0.5 * sin(tv) + 0.005 * (rnd() - 0.5);
			v.pos[1] = 1.6 + 0.005 * (rnd() - 0.5);
			v.pos[2] = 0.005 * (rnd() - 0.5);
			v.q[1] = sin(yaw / 2.0);
			v.q[3] = cos(yaw / 2.0);
			v.vel[0] = 0.5 * cos(tv) + 0.2 * (rnd() - 0.5);
			v.vel[1] = 0.2 * (rnd() - 0.5);
			v.vel[2] = 0.2 * (rnd() - 0.5);
			v.q3dof[1] = v.q[1];
			v.q3dof[3] = v.q[3];
			pimax_cam_track_out to;
			pimax_cam_track_update_vio(t, now, &v, &to);
			pimax_cam_rel id;
			memset(&id, 0, sizeof id);
			id.q[3] = 1.0;
			id.flags = 3;
			pimax_cam_rel h;
			const int st = pimax_cam_track_head(t, now, (int64_t)now, &id, &h);
			printf("run %d %d mean %a state %d/%u %s acq %" PRIu64 " rej %" PRIu64 " pos %a %a %a\n", run, i,
			       (double)o.mean[0], st, to.state, to.reason, to.acquisitions, to.rejects, h.pos[0], h.pos[1], h.pos[2]);
			if (run == 0)
				memcpy(first[i], h.pos, sizeof first[i]);
			else if (memcmp(first[i], h.pos, sizeof first[i]) != 0)
				differ = 1;
		}
		if (run == 1)
			printf("dark frames change the head output: %s\n", differ ? "yes" : "no");
		pimax_cam_track_destroy(t);
	}
	broker_close(&b, cam);
	pimax_cam_ae_destroy(a);
	return 0;
}

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IOLBF, 0);
	dup2(1, 2);
	setenv("PIMAX_CAM_RECONNECT", "0", 1);
	if (argc >= 2 && strcmp(argv[1], "layout") == 0)
		return mode_layout();
	if (argc >= 2 && strcmp(argv[1], "create") == 0)
		return mode_create();
	if (argc >= 3 && strcmp(argv[1], "errors") == 0)
		return mode_errors(argv[2]);
	if (argc >= 5 && strcmp(argv[1], "pxcam") == 0)
		return mode_pxcam(argv[2], argv[3], argv[4], argc > 5 ? atol(argv[5]) : -1);
	if (argc >= 6 && strcmp(argv[1], "synth") == 0)
		return mode_synth(argv[2], argv[3], strtoull(argv[4], NULL, 10), atoi(argv[5]));
	if (argc >= 3 && strcmp(argv[1], "scene") == 0)
		return mode_scene(argv[2]);
	fprintf(stderr, "usage: ae_difftest layout|create|errors DIR|pxcam DIR MODEL FILE [N]|synth DIR MODEL SEED S|scene DIR\n");
	return 2;
}
