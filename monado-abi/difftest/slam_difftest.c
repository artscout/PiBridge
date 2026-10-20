// SPDX-License-Identifier: MIT
// Difftest driver for the "6DoF / SLAM" part of pimax_cam.h (docs/research/rust-port-plan.md, M4). Compiled twice,
// against omarchyvr's libpimax_cam.a (the oracle) and against PiBridge's libpibridge_pimax.a; run_difftest.sh compares the
// two outputs byte for byte. Doubles are printed as exact hex (%a). stderr is sent to stdout (the library's log lines:
// the PIMAX_HEAD_TILT describe line, the "6DoF drift:" reports) so they are compared too.
//
//   slam_difftest [-e K=V | -u K]... MODE ARGS
//   layout                         struct sizes and offsets, default parameters
//   slam CALIB WORKDIR [private]   t_imu_cam, default_head, parse_cams, write_basalt (files dumped; "private" prints
//                                  hashes instead of values, for a headset's own calibration)
//   imu WORKDIR                    pimax_cam_slam_imu_load: default search, files, broken files
//   synth                          synthetic head motion through the pimax_cam_track_* ABI (walks, losses, re-snap,
//                                  new worlds, still desk with a drifting tracker, recenter, jumps, NaN, phase 1)
//   replay EUROC TRACKING [N]      a recorded EuRoC IMU (mav0/imu0/data.csv) and a recorded tracker trajectory
//                                  (monado-cli slambatch tracking.csv, z-up) driven as the Monado driver does it:
//                                  y-up, 25 ms late, 3DoF from a Mahony filter, tracker resets emulated; queries at
//                                  90 Hz; N = stop after N IMU samples
//   imulog WORKDIR                 PIMAX_HEAD_IMU_LOG: the file written, the kept earlier logs
//   errors                         NULL handles and arguments
//
// -e K=V sets and -u K unsets an environment variable before the mode runs (HeadParams switches, HOME, ...).
#define _GNU_SOURCE
#include "pimax_cam.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define O(t, f) (unsigned long)offsetof(t, f)

static uint64_t fnv(const void *p, size_t n, uint64_t h)
{
	const uint8_t *b = p;
	for (size_t i = 0; i < n; i++) {
		h ^= b[i];
		h *= 0x100000001b3ull;
	}
	return h;
}
#define FNV0 0xcbf29ce484222325ull

static int g_private;

static char *read_all(const char *path, size_t *len)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		return NULL;
	}
	size_t cap = 1 << 16, n = 0;
	char *b = malloc(cap + 1);
	size_t k;
	while ((k = fread(b + n, 1, cap - n, f)) > 0) {
		n += k;
		if (n == cap) {
			cap *= 2;
			b = realloc(b, cap + 1);
		}
	}
	fclose(f);
	b[n] = 0;
	*len = n;
	return b;
}

static void write_text(const char *path, const char *text)
{
	FILE *f = fopen(path, "wb");
	if (!f) {
		printf("cannot write %s\n", path);
		exit(2);
	}
	fwrite(text, 1, strlen(text), f);
	fclose(f);
}

static void mkdirs(const char *path)
{
	char tmp[1024];
	snprintf(tmp, sizeof tmp, "%s", path);
	for (char *p = tmp + 1; *p; p++) {
		if (*p == '/') {
			*p = 0;
			mkdir(tmp, 0755);
			*p = '/';
		}
	}
	mkdir(tmp, 0755);
}

static void pd(const char *label, const double *v, int n)
{
	printf("%s", label);
	for (int i = 0; i < n; i++) {
		printf(" %a", v[i]);
	}
	printf("\n");
}

/* Values of a headset's own calibration are not printed (private lab data): a hash instead. */
static void pdp(const char *label, const double *v, int n)
{
	if (g_private) {
		printf("%s #%016" PRIx64 "\n", label, fnv(v, sizeof(double) * (size_t)n, FNV0));
	} else {
		pd(label, v, n);
	}
}

static void dump_file(const char *label, const char *path)
{
	size_t n = 0;
	char *b = read_all(path, &n);
	if (!b) {
		printf("%s: (missing)\n", label);
		return;
	}
	if (g_private) {
		printf("%s: %zu bytes #%016" PRIx64 "\n", label, n, fnv(b, n, FNV0));
	} else {
		printf("%s: %zu bytes\n%s<<end\n", label, n, b);
	}
	free(b);
}

static const char *err_text(void)
{
	return pimax_cam_last_error();
}

/* The call first, then the error text (the Rust library's pointer is only valid until the next error). */
#define ERRLINE(label, call)                                                                                           \
	do {                                                                                                           \
		int rc_ = (call);                                                                                      \
		printf("%s %d %s\n", label, rc_, err_text());                                                          \
	} while (0)

/* ---- quaternion helpers (the driver's own math; identical in both builds) ---------------------------------- */

typedef struct
{
	double x, y, z, w;
} quat;

static quat qm(quat a, quat b)
{
	quat r = {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
	return r;
}

static quat qc(quat q)
{
	quat r = {-q.x, -q.y, -q.z, q.w};
	return r;
}

static quat qn(quat q)
{
	double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	if (n < 1e-12) {
		quat i = {0, 0, 0, 1};
		return i;
	}
	quat r = {q.x / n, q.y / n, q.z / n, q.w / n};
	return r;
}

static void qr(quat q, const double v[3], double o[3])
{
	quat p = {v[0], v[1], v[2], 0};
	quat r = qm(qm(q, p), qc(q));
	o[0] = r.x;
	o[1] = r.y;
	o[2] = r.z;
}

static quat qyaw(double rad)
{
	quat r = {0, sin(rad / 2), 0, cos(rad / 2)};
	return r;
}

static quat qx(double rad)
{
	quat r = {sin(rad / 2), 0, 0, cos(rad / 2)};
	return r;
}

static void qarr(quat q, double o[4])
{
	o[0] = q.x;
	o[1] = q.y;
	o[2] = q.z;
	o[3] = q.w;
}

/* Deterministic standard normal (Irwin-Hall from an LCG). */
static double gauss(uint64_t *seed)
{
	double s = 0;
	for (int i = 0; i < 12; i++) {
		*seed = *seed * 6364136223846793005ull + 1442695040888963407ull;
		s += (double)(*seed >> 11) / (double)(1ull << 53);
	}
	return s - 6.0;
}

/* ---- output of the ABI ------------------------------------------------------------------------------------- */

static uint64_t g_upd_hash;
static uint64_t g_upd_count;

static void print_out(const char *tag, const pimax_cam_track_out *o)
{
	printf("%s st %u ev %u wt %a %a %a wq %a %a %a %a fix %a %a %a %a held %a %a %a n %" PRIu64 " %" PRIu64
	       " %" PRIu64 " %" PRIu64 " \"%s\"\n",
	       tag, o->state, o->events, o->world_from_slam_pos[0], o->world_from_slam_pos[1], o->world_from_slam_pos[2],
	       o->world_from_slam_q[0], o->world_from_slam_q[1], o->world_from_slam_q[2], o->world_from_slam_q[3],
	       o->fix3dof_q[0], o->fix3dof_q[1], o->fix3dof_q[2], o->fix3dof_q[3], o->held_pos[0], o->held_pos[1],
	       o->held_pos[2], o->acquisitions, o->losses, o->resets, o->rejects, o->reason);
}

/* Every update goes into a running hash; updates with a pose or with events are printed. */
static void note_out(const char *tag, const pimax_cam_track_out *o, int with_pose, int rc)
{
	g_upd_hash = fnv(o, sizeof *o, g_upd_hash);
	g_upd_hash = fnv(&rc, sizeof rc, g_upd_hash);
	g_upd_count++;
	if (rc != 0) {
		printf("%s rc %d %s\n", tag, rc, err_text());
	}
	if (with_pose || o->events != 0) {
		print_out(tag, o);
	}
}

static void print_rel(const char *tag, int st, const pimax_cam_rel *r)
{
	printf("%s %d p %a %a %a q %a %a %a %a v %a %a %a w %a %a %a f %u %u\n", tag, st, r->pos[0], r->pos[1], r->pos[2],
	       r->q[0], r->q[1], r->q[2], r->q[3], r->lin_vel[0], r->lin_vel[1], r->lin_vel[2], r->ang_vel[0],
	       r->ang_vel[1], r->ang_vel[2], r->flags, r->reserved);
}

static void print_info(pimax_cam_track *t)
{
	pimax_cam_track_info i;
	memset(&i, 0x5a, sizeof i);
	int rc = pimax_cam_track_get_info(t, &i);
	printf("info %d gb %a %a %a cp %a %a %a cr %a %a %a max %a %a n %" PRIu64 " %" PRIu64 " last %" PRId64 "\n", rc,
	       i.gyro_bias[0], i.gyro_bias[1], i.gyro_bias[2], i.corr_pos[0], i.corr_pos[1], i.corr_pos[2], i.corr_rot[0],
	       i.corr_rot[1], i.corr_rot[2], i.max_corr_pos, i.max_corr_rot, i.blended, i.snapped, i.last_imu_ns);
}

static void drain_big(pimax_cam_track *t)
{
	pimax_cam_track_corr c;
	for (;;) {
		memset(&c, 0x5a, sizeof c);
		int rc = pimax_cam_track_take_big_corr(t, &c);
		if (rc != 1) {
			if (rc != 0) {
				printf("big rc %d %s\n", rc, err_text());
			}
			return;
		}
		printf("big %" PRId64 " step %a %a %a off %a %a %a rot %a sp %a %a po %u %u\n", c.ts_ns, c.step[0], c.step[1],
		       c.step[2], c.offset[0], c.offset[1], c.offset[2], c.rot, c.speed, c.ang_speed, c.pose_only,
		       c.reserved);
	}
}

/* ---- layout ------------------------------------------------------------------------------------------------- */

static int cmd_layout(void)
{
	printf("slam_imu %zu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_slam_imu), O(pimax_cam_slam_imu, gyro_noise_std),
	       O(pimax_cam_slam_imu, accel_bias_std), O(pimax_cam_slam_imu, gyro_bias_std), O(pimax_cam_slam_imu, rate_hz),
	       O(pimax_cam_slam_imu, source));
	printf("params %zu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_track_params),
	       O(pimax_cam_track_params, max_rot_mismatch_deg), O(pimax_cam_track_params, acquire_poses),
	       O(pimax_cam_track_params, reserved), O(pimax_cam_track_params, stale_ns),
	       O(pimax_cam_track_params, reset_after_ns), O(pimax_cam_track_params, reset_every_ns));
	printf("sample %zu %lu %lu %lu\n", sizeof(pimax_cam_track_sample), O(pimax_cam_track_sample, pos),
	       O(pimax_cam_track_sample, q), O(pimax_cam_track_sample, q3dof));
	printf("out %zu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_track_out),
	       O(pimax_cam_track_out, events), O(pimax_cam_track_out, world_from_slam_pos),
	       O(pimax_cam_track_out, world_from_slam_q), O(pimax_cam_track_out, fix3dof_q),
	       O(pimax_cam_track_out, held_pos), O(pimax_cam_track_out, acquisitions), O(pimax_cam_track_out, losses),
	       O(pimax_cam_track_out, resets), O(pimax_cam_track_out, rejects), O(pimax_cam_track_out, reason));
	printf("vio %zu %lu %lu %lu %lu\n", sizeof(pimax_cam_track_vio), O(pimax_cam_track_vio, pos),
	       O(pimax_cam_track_vio, q), O(pimax_cam_track_vio, vel), O(pimax_cam_track_vio, q3dof));
	printf("rel %zu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_rel), O(pimax_cam_rel, q), O(pimax_cam_rel, lin_vel),
	       O(pimax_cam_rel, ang_vel), O(pimax_cam_rel, flags), O(pimax_cam_rel, reserved));
	printf("info %zu %lu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_track_info), O(pimax_cam_track_info, corr_pos),
	       O(pimax_cam_track_info, corr_rot), O(pimax_cam_track_info, max_corr_pos),
	       O(pimax_cam_track_info, max_corr_rot), O(pimax_cam_track_info, blended), O(pimax_cam_track_info, snapped),
	       O(pimax_cam_track_info, last_imu_ns));
	printf("corr %zu %lu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_track_corr), O(pimax_cam_track_corr, step),
	       O(pimax_cam_track_corr, offset), O(pimax_cam_track_corr, rot), O(pimax_cam_track_corr, speed),
	       O(pimax_cam_track_corr, ang_speed), O(pimax_cam_track_corr, pose_only), O(pimax_cam_track_corr, reserved));
	pimax_cam_track_params p;
	memset(&p, 0x5a, sizeof p);
	pimax_cam_track_default_params(&p);
	printf("defaults %a %a %u %u %" PRIu64 " %" PRIu64 " %" PRIu64 "\n", p.max_speed_m_s, p.max_rot_mismatch_deg,
	       p.acquire_poses, p.reserved, p.stale_ns, p.reset_after_ns, p.reset_every_ns);
	pimax_cam_track_default_params(NULL);
	return 0;
}

/* ---- slam: calibration export -------------------------------------------------------------------------------- */

static void show_imu(const char *label, int rc, const pimax_cam_slam_imu *m)
{
	printf("%s rc %d\n", label, rc);
	if (rc < 0) {
		printf("  err %s\n", err_text());
	}
	pd("  an", m->accel_noise_std, 3);
	pd("  gn", m->gyro_noise_std, 3);
	pd("  ab", m->accel_bias_std, 3);
	pd("  gb", m->gyro_bias_std, 3);
	pd("  rate", &m->rate_hz, 1);
	printf("  source \"%.*s\" #%016" PRIx64 "\n", 256, m->source, fnv(m->source, sizeof m->source, FNV0));
}

static void write_case(const char *label, const pimax_cam_calib *c, const uint32_t *cams, uint32_t n, double hz,
                       int64_t off, const pimax_cam_slam_imu *imu, const char *dir, const char *note)
{
	int rc = pimax_cam_slam_write_basalt(c, cams, n, hz, off, imu, dir, note);
	printf("write %s rc %d\n", label, rc);
	if (rc != 0) {
		printf("  err %s\n", err_text());
		return;
	}
	char p[1200];
	snprintf(p, sizeof p, "%s/pimax-basalt-calib.json", dir);
	dump_file("  calib", p);
	snprintf(p, sizeof p, "%s/pimax-slam-info.json", dir);
	/* the info file names the cameras and the source; the reference's values are public */
	dump_file("  info", p);
}

static int cmd_slam(const char *calib, const char *work)
{
	pimax_cam_calib c;
	memset(&c, 0, sizeof c);
	char used[512];
	int rc = pimax_cam_calib_load(calib, &c, used, sizeof used);
	printf("load %d\n", rc);
	if (rc != 0) {
		printf("err %s\n", err_text());
		return 1;
	}
	double m[16];
	for (uint32_t i = 0; i < 8; i++) {
		memset(m, 0, sizeof m);
		rc = pimax_cam_slam_t_imu_cam(&c, i, m);
		printf("t_imu_cam %u rc %d", i, rc);
		if (rc != 0) {
			printf(" %s\n", err_text());
		} else {
			printf("\n");
			pdp("  m", m, 16);
		}
	}
	ERRLINE("t_imu_cam null cal", pimax_cam_slam_t_imu_cam(NULL, 0, m));
	ERRLINE("t_imu_cam null out", pimax_cam_slam_t_imu_cam(&c, 0, NULL));
	double h[3] = {0};
	rc = pimax_cam_slam_default_head(&c, h);
	printf("default_head %d\n", rc);
	pdp("  h", h, 3);
	ERRLINE("default_head null", pimax_cam_slam_default_head(NULL, h));
	ERRLINE("default_head null out", pimax_cam_slam_default_head(&c, NULL));
	pimax_cam_calib noimu = c;
	noimu.has_imu = 0;
	rc = pimax_cam_slam_default_head(&noimu, h);
	printf("default_head no imu %d\n", rc);
	pd("  h", h, 3);
	ERRLINE("t_imu_cam no imu", pimax_cam_slam_t_imu_cam(&noimu, 0, m));
	pimax_cam_calib one = c;
	one.camera_count = 1;
	rc = pimax_cam_slam_default_head(&one, h);
	printf("default_head one cam %d\n", rc);
	pd("  h", h, 3);
	pimax_cam_calib many = c;
	many.camera_count = 9; /* clamped to 4 by from_c */
	ERRLINE("t_imu_cam count 9 cam 5:", pimax_cam_slam_t_imu_cam(&many, 5, m));

	static const char *const lists[] = {"0,1", " 0, 1,2,3 ", "0,0", "4", "", ",", "1,,2", "+1", "-1", "a",
	                                    "0,1,2,3,4", "18446744073709551616", "3 ,2", "\t1\n", "1\xc2\xa0",
	                                    "\xc2\xa0" "2", "1\xe3\x80\x80", "0x1", "1 2", "\xff", "3,2,1", "00,01"};
	for (size_t k = 0; k < sizeof lists / sizeof lists[0]; k++) {
		static const uint32_t counts[] = {4, 2, 0};
		static const uint32_t caps[] = {4, 2, 0};
		for (int ci = 0; ci < 3; ci++) {
			for (int pi = 0; pi < 3; pi++) {
				uint32_t ids[8] = {0};
				rc = pimax_cam_slam_parse_cams(lists[k], counts[ci], ids, caps[pi]);
				printf("cams [%s] count %u cap %u: %d", lists[k], counts[ci], caps[pi], rc);
				if (rc < 0) {
					printf(" %s\n", err_text());
				} else {
					for (int i = 0; i < rc; i++) {
						printf(" %u", ids[i]);
					}
					printf("\n");
				}
			}
		}
		rc = pimax_cam_slam_parse_cams(lists[k], 4, NULL, 4);
		printf("cams [%s] null out: %d %s\n", lists[k], rc, rc < 0 ? err_text() : "");
	}
	ERRLINE("cams null:", pimax_cam_slam_parse_cams(NULL, 4, NULL, 4));

	pimax_cam_slam_imu imu;
	char p[1024];
	snprintf(p, sizeof p, "%s/nonexistent/slam-imu.json", work);
	show_imu("imu missing", pimax_cam_slam_imu_load(p, &imu), &imu);
	/* the gyro bias cache: XDG_CACHE_HOME, else HOME/.cache */
	char cache[1024], dir[1100];
	snprintf(cache, sizeof cache, "%s/cache", work);
	setenv("XDG_CACHE_HOME", cache, 1);
	static const char *const caches[] = {NULL, "0.2315 0.7274 0.4080\n", "1 2", "9 0 0", "nan 0 0", "-4.9 4.9 0",
	                                     "1e0 +2 -0.5 ", " 1\t2\n3\n", "0 0 5", "0 0 5.0000001", "1,2,3"};
	uint32_t c01[2] = {0, 1};
	for (size_t k = 0; k < sizeof caches / sizeof caches[0]; k++) {
		char cp[1100];
		snprintf(cp, sizeof cp, "%s/omarchyvr", cache);
		mkdirs(cp);
		snprintf(cp, sizeof cp, "%s/omarchyvr/gyro-bias", cache);
		unlink(cp);
		if (caches[k]) {
			write_text(cp, caches[k]);
		}
		snprintf(dir, sizeof dir, "%s/wb-cache%zu", work, k);
		char label[64];
		snprintf(label, sizeof label, "cache%zu", k);
		write_case(label, &c, c01, 2, 25.0, 0, &imu, dir, NULL);
	}
	unsetenv("XDG_CACHE_HOME");
	char home[1024];
	snprintf(home, sizeof home, "%s/home", work);
	setenv("HOME", home, 1);
	snprintf(p, sizeof p, "%s/.cache/omarchyvr", home);
	mkdirs(p);
	snprintf(p, sizeof p, "%s/.cache/omarchyvr/gyro-bias", home);
	write_text(p, "-0.5 0.25 1.5\n");
	snprintf(dir, sizeof dir, "%s/wb-home", work);
	write_case("home cache", &c, c01, 2, 25.0, 0, &imu, dir, "note \"quoted\" \\ \n\t\x01 \xc3\xa9 \xff");
	setenv("XDG_CACHE_HOME", "", 1);
	snprintf(dir, sizeof dir, "%s/wb-empty-xdg", work);
	write_case("empty xdg", &c, c01, 2, 25.0, 0, &imu, dir, "");
	unsetenv("XDG_CACHE_HOME");
	unsetenv("HOME");
	snprintf(dir, sizeof dir, "%s/wb-nohome", work);
	write_case("no home", &c, c01, 2, 25.0, 0, NULL, dir, NULL);

	pimax_cam_slam_imu custom = imu;
	for (int i = 0; i < 3; i++) {
		custom.accel_noise_std[i] = 0.01 * (i + 1);
		custom.gyro_noise_std[i] = 1e-4 / (i + 1);
		custom.accel_bias_std[i] = -1.5e-3;
		custom.gyro_bias_std[i] = 3.0e-7 * (i + 2);
	}
	custom.rate_hz = 1024.5;
	memset(custom.source, 0, sizeof custom.source);
	snprintf(custom.source, sizeof custom.source, "src \"q\" \\ \n \xc3\xa9 \xe2\x82");
	uint32_t c3210[4] = {3, 2, 1, 0};
	snprintf(dir, sizeof dir, "%s/wb-four", work);
	write_case("four", &c, c3210, 4, 30.5, -1234567, &custom, dir, "x");
	memset(custom.source, 'a', sizeof custom.source); /* no NUL: the whole 256 bytes */
	snprintf(dir, sizeof dir, "%s/wb-long", work);
	write_case("long source", &c, c3210 + 2, 2, 1e-300, INT64_MIN, &custom, dir, NULL);
	custom.rate_hz = NAN;
	custom.accel_noise_std[1] = INFINITY;
	custom.source[0] = 0;
	snprintf(dir, sizeof dir, "%s/wb-nan", work);
	write_case("nan", &c, c01, 2, -0.0, INT64_MAX, &custom, dir, NULL);
	uint32_t c7[1] = {7}, c1[1] = {1}, c55[2] = {1, 1};
	snprintf(dir, sizeof dir, "%s/wb-err", work);
	write_case("bad cam", &c, c7, 1, 25.0, 0, &imu, dir, NULL);
	write_case("dup cam", &c, c55, 2, 25.0, 0, &imu, dir, NULL);
	write_case("one cam", &c, c1, 1, 25.0, 0, &imu, dir, NULL);
	write_case("no imu", &noimu, c01, 2, 25.0, 0, &imu, dir, NULL);
	write_case("null cal", NULL, c01, 2, 25.0, 0, &imu, dir, NULL);
	write_case("null cams", &c, NULL, 2, 25.0, 0, &imu, dir, NULL);
	write_case("n 0", &c, c01, 0, 25.0, 0, &imu, dir, NULL);
	write_case("null dir", &c, c01, 2, 25.0, 0, &imu, NULL, NULL);
	snprintf(dir, sizeof dir, "%s/a/b/c/d", work);
	write_case("nested", &c, c01, 2, 25.0, 0, &imu, dir, NULL);
	snprintf(p, sizeof p, "%s/file", work);
	write_text(p, "x");
	write_case("dir is a file", &c, c01, 2, 25.0, 0, &imu, p, NULL);
	snprintf(dir, sizeof dir, "%s/file/sub", work);
	write_case("parent is a file", &c, c01, 2, 25.0, 0, &imu, dir, NULL);
	write_case("empty dir", &c, c01, 2, 25.0, 0, &imu, "", NULL);
	snprintf(dir, sizeof dir, "%s/ro", work);
	mkdirs(dir);
	snprintf(p, sizeof p, "%s/pimax-basalt-calib.json", dir);
	mkdirs(p); /* the calib file name is taken by a directory */
	write_case("calib is a dir", &c, c01, 2, 25.0, 0, &imu, dir, NULL);
	snprintf(dir, sizeof dir, "%s/ro2", work);
	mkdirs(dir);
	snprintf(p, sizeof p, "%s/pimax-slam-info.json", dir);
	mkdirs(p);
	write_case("info is a dir", &c, c01, 2, 25.0, 0, &imu, dir, NULL);
	return 0;
}

/* ---- imu: the noise file ------------------------------------------------------------------------------------- */

static int cmd_imu(const char *work)
{
	pimax_cam_slam_imu m;
	char home[1024], p[1200];
	snprintf(home, sizeof home, "%s/home", work);
	mkdirs(home);
	unsetenv("PIMAX_SLAM_IMU");
	setenv("HOME", home, 1);
	memset(&m, 0x5a, sizeof m);
	show_imu("no file", pimax_cam_slam_imu_load(NULL, &m), &m);
	snprintf(p, sizeof p, "%s/.config/omarchyvr", home);
	mkdirs(p);
	snprintf(p, sizeof p, "%s/.config/omarchyvr/slam-imu.json", home);
	write_text(p, "{\n  \"comment\": \"x\",\n  \"imu_update_rate\": 1000.0,\n  \"accel_noise_std\": [0.01, 0.02, 0.03],\n"
	              "  \"gyro_noise_std\": [0.0001, 0.0002, 0.0003],\n  \"accel_bias_std\": [0.001, 0.001, 0.001],\n"
	              "  \"gyro_bias_std\": [0.0001, 0.0001, 0.0001]\n}\n");
	show_imu("home file", pimax_cam_slam_imu_load(NULL, &m), &m);
	setenv("PIMAX_SLAM_IMU", "", 1);
	show_imu("empty env", pimax_cam_slam_imu_load(NULL, &m), &m);
	snprintf(p, sizeof p, "%s/env-missing.json", work);
	setenv("PIMAX_SLAM_IMU", p, 1);
	show_imu("env missing", pimax_cam_slam_imu_load(NULL, &m), &m);
	show_imu("explicit missing", pimax_cam_slam_imu_load(p, &m), &m);
	show_imu("explicit dir", pimax_cam_slam_imu_load(work, &m), &m);
	show_imu("empty path", pimax_cam_slam_imu_load("", &m), &m);
	static const char *const docs[] = {
		"{\"accel_noise_std\": 0.01}",
		"{\"gyro_noise_std\": [1e-4, 2e-4, 3e-4], \"imu_update_rate\": 1024}",
		"{\"gyro_bias_std\": 0}",
		"{\"gyro_bias_std\": -1}",
		"{\"accel_bias_std\": [1, 2]}",
		"{\"accel_bias_std\": [1, 2, \"x\"]}",
		"{\"accel_bias_std\": \"0.1\"}",
		"{\"imu_update_rate\": 0}",
		"{\"imu_update_rate\": -5}",
		"{\"imu_update_rate\": \"fast\"}",
		"{\"imu_update_rate\": 1e308, \"accel_noise_std\": 1e-308}",
		"{\"accel_noise_std\": 1e999}",
		"[]",
		"[1, 2]",
		"1",
		"null",
		"{}",
		"",
		"{",
		"{\"a\": 1} x",
		"{\"accel_noise_std\": [0.1, 0.2, 0.3], \"accel_noise_std\": 5}",
		"{\"gyro_noise_std\": true}",
		"\xef\xbb\xbf{}",
		"{\"x\": \"\xff\"}",
		"  {\"gyro_noise_std\" : 2e-3 }  \n",
	};
	for (size_t k = 0; k < sizeof docs / sizeof docs[0]; k++) {
		snprintf(p, sizeof p, "%s/doc%zu.json", work, k);
		write_text(p, docs[k]);
		char label[64];
		snprintf(label, sizeof label, "doc%zu", k);
		memset(&m, 0x5a, sizeof m);
		show_imu(label, pimax_cam_slam_imu_load(p, &m), &m);
		if (k == 2) {
			setenv("PIMAX_SLAM_IMU", p, 1);
			show_imu("env bad doc", pimax_cam_slam_imu_load(NULL, &m), &m);
		}
	}
	printf("null out %d\n", pimax_cam_slam_imu_load(NULL, NULL));
	unsetenv("PIMAX_SLAM_IMU");
	unsetenv("HOME");
	show_imu("no home", pimax_cam_slam_imu_load(NULL, &m), &m);
	return 0;
}

/* ---- synth: synthetic head motion --------------------------------------------------------------------------- */

/* Smooth head motion (headpose.rs tests truth()): slow walk on a circle with a head bob, yaw/pitch oscillation. */
static void truth(double t, double p[3], quat *q, double v[3], double w[3], double a[3])
{
	const double r = 0.8, wc = 0.35, ws = 2.0 * M_PI * 1.8, bob = 0.015;
	p[0] = r * sin(wc * t);
	p[1] = 1.6 + 0.03 * sin(1.3 * t) + bob * sin(ws * t);
	p[2] = r * cos(wc * t) - r;
	v[0] = r * wc * cos(wc * t);
	v[1] = 0.03 * 1.3 * cos(1.3 * t) + bob * ws * cos(ws * t);
	v[2] = -r * wc * sin(wc * t);
	a[0] = -r * wc * wc * sin(wc * t);
	a[1] = -0.03 * 1.69 * sin(1.3 * t) - bob * ws * ws * sin(ws * t);
	a[2] = -r * wc * wc * cos(wc * t);
	double yaw = 0.6 * sin(0.9 * t) + 0.2 * t;
	double pitch = 0.25 * sin(1.7 * t);
	quat qy = qyaw(yaw);
	*q = qm(qy, qx(pitch));
	double yr = 0.54 * cos(0.9 * t) + 0.2;
	double pr = 0.425 * cos(1.7 * t);
	double px[3] = {pr, 0, 0}, rx[3];
	qr(qy, px, rx);
	w[0] = rx[0];
	w[1] = yr + rx[1];
	w[2] = rx[2];
}

typedef struct
{
	const char *name;
	double secs;
	int still;            /* headset lying still (desk) instead of the walk */
	double gap_from, gap_to; /* tracker silent */
	double new_yaw, new_off[3]; /* after the gap: another tracker world (new_yaw != 0 or offset) */
	int new_world;
	double drift_acc;     /* tracker runs away while still: 0.5 a t^2 along x after drift_from */
	double drift_from;
	double jump_at;       /* one position jump of 1 m */
	double nan_at;        /* one NaN pose */
	double spin_from, spin_to; /* tracker rotates 20 deg per pose against the gyro */
	double recenter_at[2];
	double noise_m;       /* tracker position noise */
	double vel_noise;     /* tracker velocity noise */
	int phase1;           /* use pimax_cam_track_update (no velocity) */
	double slide;         /* tracker slides sideways at this speed (m/s), the drift governor's case */
	double pickup_at;     /* still desk: picked up (0.5 rad/s) from here */
	double bias[3];       /* gyro bias */
	double big_step_at;   /* tracker corrects itself by 6 cm */
	int acquire_poses;
	int seated;           /* a worn, seated head: breathing sway, small yaw, tremor (never desk-still) */
} scenario;

static void run_scenario(const scenario *sc)
{
	printf("== scenario %s\n", sc->name);
	pimax_cam_track_params prm;
	pimax_cam_track_default_params(&prm);
	prm.acquire_poses = (uint32_t)sc->acquire_poses;
	double head[3] = {0.0, -0.02, 0.07};
	pimax_cam_track *t = pimax_cam_track_create(&prm, head);
	if (!t) {
		printf("create failed %s\n", err_text());
		return;
	}
	g_upd_hash = FNV0;
	g_upd_count = 0;
	uint64_t seed = 0x1234;
	uint64_t qhash = FNV0;
	int reset_seen = 0;
	double next_vio = 0.04, next_q = 0.0;
	/* pending tracker poses (delivered 25 ms late) */
	pimax_cam_track_vio pend[8];
	double due[8];
	int npend = 0;
	int recentered[2] = {0, 0};
	long n = (long)(sc->secs * 1000.0);
	for (long i = 0; i < n; i++) {
		double tt = (double)i * 0.001;
		int64_t now = i * 1000000;
		double p[3], v[3], w[3], a[3];
		quat q;
		if (sc->still) {
			p[0] = 0.3;
			p[1] = 0.8;
			p[2] = -0.4;
			v[0] = v[1] = v[2] = 0;
			a[0] = a[1] = a[2] = 0;
			q = qm(qyaw(0.4), qx(-0.3));
			w[0] = w[1] = w[2] = 0;
			if (sc->pickup_at > 0 && tt >= sc->pickup_at) {
				double ang = 0.5 * (tt - sc->pickup_at);
				q = qm(qyaw(0.4 + ang), qx(-0.3));
				w[1] = 0.5;
			}
		} else if (sc->seated) {
			const double wb = 2.0 * M_PI * 0.25;
			p[0] = 0.004 * sin(wb * tt);
			p[1] = 1.2;
			p[2] = 0.0;
			v[0] = 0.004 * wb * cos(wb * tt);
			v[1] = v[2] = 0.0;
			a[0] = -0.004 * wb * wb * sin(wb * tt);
			a[1] = a[2] = 0.0;
			q = qm(qyaw(0.3 + 0.02 * sin(0.7 * tt)), qx(-0.15));
			w[0] = w[2] = 0.0;
			w[1] = 0.014 * cos(0.7 * tt);
		} else {
			truth(tt, p, &q, v, w, a);
		}
		/* IMU: body frame, raw */
		double gw[3], gb[3], ab[3], aw[3] = {a[0], a[1] + 9.80665, a[2]};
		qr(qc(q), w, gb);
		qr(qc(q), aw, ab);
		double ng = sc->still ? 0.0004 : 0.002, na = sc->still ? 0.005 : 0.02;
		for (int k = 0; k < 3; k++) {
			gw[k] = gb[k] + sc->bias[k] + ng * gauss(&seed);
			ab[k] += na * gauss(&seed);
		}
		if (sc->seated) {
			gw[0] += 0.008 * sin(9.0 * tt);
			gw[1] += 0.007 * sin(6.3 * tt + 1.0);
			gw[2] += 0.006 * sin(11.0 * tt);
		}
		pimax_cam_track_push_imu(t, now, ab, gw);
		/* a tracker pose every 40 ms */
		if (tt >= next_vio) {
			next_vio += 0.04;
			int silent = tt >= sc->gap_from && tt < sc->gap_to;
			if (!silent) {
				quat wq = qyaw(0.7);
				double off[3] = {1.0, 0.0, -2.0};
				if (sc->new_world && tt >= sc->gap_to) {
					wq = qyaw(sc->new_yaw);
					off[0] = sc->new_off[0];
					off[1] = sc->new_off[1];
					off[2] = sc->new_off[2];
				}
				if (reset_seen && sc->still) {
					wq = qyaw(-0.9);
					off[0] = 5.0;
					off[1] = 0.0;
					off[2] = 5.0;
				}
				pimax_cam_track_vio s;
				memset(&s, 0, sizeof s);
				s.ts_ns = (uint64_t)now;
				double pw[3], vw[3];
				qr(wq, p, pw);
				qr(wq, v, vw);
				for (int k = 0; k < 3; k++) {
					s.pos[k] = pw[k] + off[k] + sc->noise_m * gauss(&seed);
					s.vel[k] = vw[k] + sc->vel_noise * gauss(&seed);
				}
				if (sc->drift_acc > 0 && tt > sc->drift_from && !reset_seen) {
					double d = tt - sc->drift_from;
					s.pos[0] += 0.5 * sc->drift_acc * d * d;
					s.vel[0] += sc->drift_acc * d;
				}
				if (sc->slide > 0 && tt > 3.0) {
					s.pos[0] += sc->slide * (tt - 3.0);
					s.vel[0] += sc->slide;
				}
				if (sc->big_step_at > 0 && tt >= sc->big_step_at) {
					s.pos[0] += 0.06;
				}
				if (sc->jump_at > 0 && fabs(tt - sc->jump_at) < 0.02) {
					s.pos[0] += 1.0;
				}
				quat qs = qm(wq, q);
				if (sc->spin_to > 0 && tt >= sc->spin_from && tt < sc->spin_to) {
					qs = qm(qyaw(0.35 * (double)(i / 40)), qs);
				}
				qarr(qs, s.q);
				qarr(q, s.q3dof);
				if (sc->nan_at > 0 && fabs(tt - sc->nan_at) < 0.02) {
					s.pos[1] = NAN;
				}
				if (npend < 8) {
					pend[npend] = s;
					due[npend] = tt + 0.025;
					npend++;
				}
			}
		}
		/* deliver one due pose per ms, else an update without one */
		pimax_cam_track_out o;
		memset(&o, 0x5a, sizeof o);
		int rc;
		int delivered = 0;
		for (int k = 0; k < npend; k++) {
			if (due[k] <= tt) {
				pimax_cam_track_vio s = pend[k];
				memmove(&pend[k], &pend[k + 1], sizeof pend[0] * (size_t)(npend - k - 1));
				memmove(&due[k], &due[k + 1], sizeof due[0] * (size_t)(npend - k - 1));
				npend--;
				if (sc->phase1) {
					pimax_cam_track_sample s1;
					s1.ts_ns = s.ts_ns;
					memcpy(s1.pos, s.pos, sizeof s1.pos);
					memcpy(s1.q, s.q, sizeof s1.q);
					memcpy(s1.q3dof, s.q3dof, sizeof s1.q3dof);
					rc = pimax_cam_track_update(t, (uint64_t)now, &s1, &o);
				} else {
					rc = pimax_cam_track_update_vio(t, (uint64_t)now, &s, &o);
				}
				delivered = 1;
				break;
			}
		}
		if (!delivered) {
			rc = sc->phase1 ? pimax_cam_track_update(t, (uint64_t)now, NULL, &o)
			                : pimax_cam_track_update_vio(t, (uint64_t)now, NULL, (i % 7 == 0) ? NULL : &o);
			if (!sc->phase1 && i % 7 == 0) {
				memset(&o, 0, sizeof o);
			}
		}
		if (o.events & PIMAX_CAM_TRACK_EV_RESET_WANTED) {
			reset_seen = 1;
		}
		char tag[64];
		snprintf(tag, sizeof tag, "u %ld", i);
		note_out(tag, &o, delivered, rc);
		for (int k = 0; k < 2; k++) {
			if (sc->recenter_at[k] > 0 && tt >= sc->recenter_at[k] && !recentered[k]) {
				recentered[k] = 1;
				pimax_cam_track_recenter(t);
				printf("recenter at %ld\n", i);
			}
		}
		/* queries at 90 Hz for display time (now + 20 ms) */
		if (tt >= next_q) {
			next_q += 1.0 / 90.0;
			pimax_cam_rel r3, out;
			memset(&r3, 0, sizeof r3);
			double at = tt + 0.020;
			double p2[3], v2[3], w2[3], a2[3];
			quat q2 = q;
			if (!sc->still) {
				truth(at, p2, &q2, v2, w2, a2);
				memcpy(r3.ang_vel, w2, sizeof r3.ang_vel);
			} else {
				memcpy(r3.ang_vel, w, sizeof r3.ang_vel);
			}
			qarr(q2, r3.q);
			r3.flags = 1 | 16 | ((i % 3) ? 8 : 0);
			memset(&out, 0x5a, sizeof out);
			int64_t at_ns = (int64_t)llround(at * 1e9);
			if (i % 50 == 25) {
				at_ns -= 60000000; /* a past query (before the newest tracker pose) */
			}
			int st = pimax_cam_track_head(t, (uint64_t)now, at_ns, &r3, &out);
			char qt[64];
			snprintf(qt, sizeof qt, "q %ld", i);
			print_rel(qt, st, &out);
			qhash = fnv(&out, sizeof out, qhash);
		}
		if (i % 500 == 0) {
			print_info(t);
			drain_big(t);
		}
	}
	print_info(t);
	drain_big(t);
	printf("end %s updates %" PRIu64 " #%016" PRIx64 " queries #%016" PRIx64 "\n", sc->name, g_upd_count, g_upd_hash,
	       qhash);
	pimax_cam_track_destroy(t);
}

static int cmd_synth(void)
{
	/* the capi.rs test track_through_c */
	pimax_cam_track_params p;
	pimax_cam_track_default_params(&p);
	p.acquire_poses = 2;
	pimax_cam_track *t = pimax_cam_track_create(&p, NULL);
	pimax_cam_track_out out;
	for (uint64_t i = 0; i < 3; i++) {
		pimax_cam_track_sample s = {1000000000ull + i * 40000000ull, {1, 2, 3}, {0, 0, 0, 1}, {0, 0, 0, 1}};
		memset(&out, 0, sizeof out);
		int rc = pimax_cam_track_update(t, s.ts_ns, &s, &out);
		printf("phase1 %d ", rc);
		print_out("o", &out);
	}
	pimax_cam_track_destroy(t);
	t = pimax_cam_track_create(&p, NULL);
	double g[3] = {0, 0, 0}, a[3] = {0, 9.80665, 0};
	for (int64_t i = 0; i < 200; i++) {
		pimax_cam_track_push_imu(t, 1000000000 + i * 1000000, a, g);
	}
	for (uint64_t i = 0; i < 3; i++) {
		pimax_cam_track_vio s = {1000000000ull + i * 40000000ull, {1, 2, 3}, {0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0, 1}};
		int rc = pimax_cam_track_update_vio(t, s.ts_ns, &s, &out);
		printf("vio %d ", rc);
		print_out("o", &out);
	}
	pimax_cam_rel r3 = {{0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0}, 17, 0}, r;
	int st = pimax_cam_track_head(t, 1150000000, 1170000000, &r3, &r);
	print_rel("head", st, &r);
	print_info(t);
	pimax_cam_track_destroy(t);
	pimax_cam_track_destroy(NULL);

	scenario base;
	memset(&base, 0, sizeof base);
	base.secs = 8.0;
	base.noise_m = 0.002;
	base.acquire_poses = 3;
	base.gap_from = base.gap_to = -1;
	scenario s;

	s = base;
	s.name = "walk";
	run_scenario(&s);

	s = base;
	s.name = "walk vel noise";
	s.vel_noise = 0.3;
	s.secs = 6;
	run_scenario(&s);

	s = base;
	s.name = "loss same world";
	s.noise_m = 0;
	s.secs = 9;
	s.gap_from = 3.0;
	s.gap_to = 4.5;
	run_scenario(&s);

	s = base;
	s.name = "loss new world";
	s.noise_m = 0;
	s.secs = 7;
	s.gap_from = 3.0;
	s.gap_to = 4.0;
	s.new_world = 1;
	s.new_yaw = 1.2;
	s.new_off[0] = 3.0;
	s.new_off[2] = 1.0;
	run_scenario(&s);

	s = base;
	s.name = "long loss resets";
	s.secs = 16;
	s.gap_from = 2.0;
	s.gap_to = 13.0;
	s.new_world = 1;
	s.new_yaw = -0.4;
	run_scenario(&s);

	s = base;
	s.name = "correction step";
	s.noise_m = 0;
	s.secs = 6;
	s.big_step_at = 3.0;
	run_scenario(&s);

	s = base;
	s.name = "jump nan spin";
	s.secs = 9;
	s.jump_at = 2.5;
	s.nan_at = 4.0;
	s.spin_from = 5.0;
	s.spin_to = 6.0;
	run_scenario(&s);

	s = base;
	s.name = "recenter";
	s.secs = 7;
	s.recenter_at[0] = 0.2;
	s.recenter_at[1] = 4.0;
	run_scenario(&s);

	s = base;
	s.name = "phase1";
	s.secs = 5;
	s.phase1 = 1;
	s.gap_from = 3.0;
	s.gap_to = 3.6;
	run_scenario(&s);

	s = base;
	s.name = "desk drifting tracker";
	s.still = 1;
	s.secs = 14;
	s.noise_m = 0.0002;
	s.drift_acc = 0.07;
	s.drift_from = 8.0;
	s.bias[0] = -0.0043;
	s.bias[1] = 0.0138;
	s.bias[2] = -0.0059;
	run_scenario(&s);

	s = base;
	s.name = "desk pickup";
	s.still = 1;
	s.secs = 5;
	s.noise_m = 0.0002;
	s.pickup_at = 3.5;
	run_scenario(&s);

	s = base;
	s.name = "sliding tracker";
	s.secs = 26;
	s.noise_m = 0.001;
	s.slide = 0.026;
	run_scenario(&s);

	s = base;
	s.name = "seated, sliding tracker";
	s.seated = 1;
	s.secs = 26;
	s.noise_m = 0.001;
	s.slide = 0.026;
	run_scenario(&s);

	s = base;
	s.name = "acquire 12";
	s.acquire_poses = 12;
	s.secs = 3;
	run_scenario(&s);
	return 0;
}

/* ---- replay: recorded IMU + recorded tracker trajectory ------------------------------------------------------ */

typedef struct
{
	int64_t ts;
	double g[3], a[3];
} imu_row;

typedef struct
{
	int64_t ts;
	double p[3];
	quat q;
} pose_row;

static size_t load_imu(const char *dir, imu_row **out)
{
	char p[1200];
	snprintf(p, sizeof p, "%s/mav0/imu0/data.csv", dir);
	FILE *f = fopen(p, "r");
	if (!f) {
		return 0;
	}
	size_t cap = 1 << 18, n = 0;
	imu_row *v = malloc(sizeof *v * cap);
	char line[512];
	while (fgets(line, sizeof line, f)) {
		if (line[0] == '#') {
			continue;
		}
		imu_row r;
		if (sscanf(line, "%" SCNd64 ",%lf,%lf,%lf,%lf,%lf,%lf", &r.ts, &r.g[0], &r.g[1], &r.g[2], &r.a[0], &r.a[1],
		           &r.a[2]) == 7) {
			if (n == cap) {
				cap *= 2;
				v = realloc(v, sizeof *v * cap);
			}
			v[n++] = r;
		}
	}
	fclose(f);
	*out = v;
	return n;
}

static size_t load_traj(const char *path, pose_row **out)
{
	FILE *f = fopen(path, "r");
	if (!f) {
		return 0;
	}
	size_t cap = 1 << 14, n = 0;
	pose_row *v = malloc(sizeof *v * cap);
	char line[512];
	while (fgets(line, sizeof line, f)) {
		if (line[0] == '#') {
			continue;
		}
		pose_row r;
		/* EuRoC: ts, px py pz, qw qx qy qz */
		if (sscanf(line, "%" SCNd64 ",%lf,%lf,%lf,%lf,%lf,%lf,%lf", &r.ts, &r.p[0], &r.p[1], &r.p[2], &r.q.w, &r.q.x,
		           &r.q.y, &r.q.z) == 8) {
			if (n == cap) {
				cap *= 2;
				v = realloc(v, sizeof *v * cap);
			}
			v[n++] = r;
		}
	}
	fclose(f);
	*out = v;
	return n;
}

static int cmd_replay(const char *euroc, const char *traj, long max_imu)
{
	imu_row *imu = NULL;
	pose_row *vio = NULL;
	size_t ni = load_imu(euroc, &imu);
	size_t nv = load_traj(traj, &vio);
	printf("replay: %zu imu, %zu poses\n", ni, nv);
	if (ni < 1000 || nv < 50) {
		return 1;
	}
	if (max_imu > 0 && (size_t)max_imu < ni) {
		ni = (size_t)max_imu;
	}
	/* Basalt world (z up) -> driver world (y up): pimax_camera.c SLAM_TO_YUP, a -90 deg turn about x. */
	const quat to_yup = {-M_SQRT1_2, 0, 0, M_SQRT1_2};
	pimax_cam_track_params prm;
	pimax_cam_track_default_params(&prm);
	double head[3] = {0.049, -0.015, 0.030};
	pimax_cam_track *t = pimax_cam_track_create(&prm, head);
	if (!t) {
		printf("create failed %s\n", err_text());
		return 1;
	}
	g_upd_hash = FNV0;
	g_upd_count = 0;
	/* 3DoF: Mahony (proportional) from the gyro and gravity, bias from the first quiet second */
	quat fus = {0, 0, 0, 1};
	int fus_init = 0;
	double bias[3] = {0, 0, 0};
	{
		size_t n = 0;
		double m[3] = {0, 0, 0};
		while (n < ni && imu[n].ts - imu[0].ts < 1000000000) {
			for (int k = 0; k < 3; k++) {
				m[k] += imu[n].g[k];
			}
			n++;
		}
		if (n >= 100) {
			int quiet = 1;
			for (int k = 0; k < 3; k++) {
				m[k] /= (double)n;
			}
			for (size_t i = 0; i < n; i++) {
				double d[3] = {imu[i].g[0] - m[0], imu[i].g[1] - m[1], imu[i].g[2] - m[2]};
				if (sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) >= 0.05) {
					quiet = 0;
				}
			}
			if (quiet) {
				memcpy(bias, m, sizeof bias);
			}
		}
	}
	pd("startup bias", bias, 3);
	/* q3dof history ring */
	enum { QH = 2048 };
	static int64_t qh_t[QH];
	static quat qh_q[QH];
	size_t qh_n = 0;
	size_t k = 0;
	int pending_reset = 0;
	int rebased = 0;
	int64_t rb_t = 0;
	double rb_p[3] = {0}, rb_v[3] = {0};
	quat rb_r = {0, 0, 0, 1};
	quat w0 = {0, 0, 0, 1};
	int have_w0 = 0;
	int skip = 0;
	int64_t next_q = imu[0].ts + 1000000000;
	uint64_t qhash = FNV0;
	double prev_y[3][3];
	int64_t prev_t[3] = {0};
	int nprev = 0;
	for (size_t i = 0; i < ni; i++) {
		const imu_row *s = &imu[i];
		double dt = i == 0 ? 0.001 : (double)(s->ts - imu[i - 1].ts) / 1e9;
		if (dt < 1e-4) {
			dt = 1e-4;
		}
		if (dt > 0.01) {
			dt = 0.01;
		}
		double an = sqrt(s->a[0] * s->a[0] + s->a[1] * s->a[1] + s->a[2] * s->a[2]);
		if (!fus_init) {
			if (an >= 1.0) {
				double u[3] = {s->a[0] / an, s->a[1] / an, s->a[2] / an};
				/* shortest rotation taking the measured up (body) to world +y */
				/* cross(u, y) = (-u_z, 0, u_x) */
				quat q0 = {-u[2], 0.0, u[0], 1.0 + u[1]};
				fus = qn(q0);
				fus_init = 1;
			}
		} else {
			double w[3] = {s->g[0] - bias[0], s->g[1] - bias[1], s->g[2] - bias[2]};
			if (fabs(an - 9.81) < 1.0) {
				double y[3] = {0, 1, 0}, up[3];
				qr(qc(fus), y, up);
				double e[3] = {s->a[1] / an * up[2] - s->a[2] / an * up[1], s->a[2] / an * up[0] - s->a[0] / an * up[2],
				               s->a[0] / an * up[1] - s->a[1] / an * up[0]};
				for (int j = 0; j < 3; j++) {
					w[j] += 0.5 * e[j];
				}
			}
			quat h = {w[0] * 0.5 * dt, w[1] * 0.5 * dt, w[2] * 0.5 * dt, 1.0};
			fus = qn(qm(fus, h));
		}
		qh_t[qh_n % QH] = s->ts;
		qh_q[qh_n % QH] = fus;
		qh_n++;
		pimax_cam_track_push_imu(t, s->ts, s->a, s->g);
		/* deliver tracker poses whose time + 25 ms has come */
		while (k < nv && vio[k].ts + 25000000 <= s->ts) {
			double raw_p[3];
			qr(to_yup, vio[k].p, raw_p);
			quat raw_q = qn(qm(to_yup, qn(vio[k].q)));
			int64_t ts = vio[k].ts;
			/* causal velocity: second-order backward difference */
			double raw_v[3] = {0, 0, 0};
			if (nprev >= 2) {
				double d2 = (double)(ts - prev_t[1]) / 2e9;
				if (d2 > 0) {
					for (int j = 0; j < 3; j++) {
						raw_v[j] = (3.0 * raw_p[j] - 4.0 * prev_y[0][j] + prev_y[1][j]) * 0.5 / d2;
					}
				}
			}
			memcpy(prev_y[1], prev_y[0], sizeof prev_y[0]);
			prev_t[1] = prev_t[0];
			memcpy(prev_y[0], raw_p, sizeof raw_p);
			prev_t[0] = ts;
			if (nprev < 2) {
				nprev++;
			}
			if (pending_reset) {
				/* a fresh tracker world: starts at the current pose with zero velocity, gravity aligned like the
				 * first one, another yaw (stillgate.rs) */
				pending_reset = 0;
				quat q3r = qh_q[(qh_n - 1) % QH];
				quat d = qn(qm(qm(have_w0 ? w0 : (quat){0, 0, 0, 1}, q3r), qc(raw_q)));
				double fwd[3] = {0, 0, -1}, f2[3];
				qr(q3r, fwd, f2);
				quat y = qyaw(atan2(-f2[0], -f2[2]) + 0.6);
				rb_r = qn(qm(qc(y), d));
				rb_t = ts;
				memcpy(rb_p, raw_p, sizeof rb_p);
				memcpy(rb_v, raw_v, sizeof rb_v);
				rebased = 1;
				skip = 2;
				printf("tracker reset emulated at %" PRId64 "\n", ts);
			}
			k++;
			if (skip > 0) {
				skip--;
				continue;
			}
			pimax_cam_track_vio sv;
			memset(&sv, 0, sizeof sv);
			sv.ts_ns = (uint64_t)ts;
			if (rebased) {
				double tr = (double)(ts - rb_t) / 1e9;
				double rel[3], dv[3];
				for (int j = 0; j < 3; j++) {
					rel[j] = raw_p[j] - rb_p[j] - rb_v[j] * tr;
					dv[j] = raw_v[j] - rb_v[j];
				}
				qr(rb_r, rel, sv.pos);
				qarr(qn(qm(rb_r, raw_q)), sv.q);
				qr(rb_r, dv, sv.vel);
			} else {
				memcpy(sv.pos, raw_p, sizeof sv.pos);
				qarr(raw_q, sv.q);
				memcpy(sv.vel, raw_v, sizeof sv.vel);
			}
			/* q3dof at the pose time */
			quat q3 = fus;
			for (size_t j = qh_n; j-- > 0 && qh_n - j <= QH;) {
				if (qh_t[j % QH] <= ts) {
					q3 = qh_q[j % QH];
					break;
				}
			}
			qarr(q3, sv.q3dof);
			if (!have_w0 && fus_init) {
				quat qq = {sv.q[0], sv.q[1], sv.q[2], sv.q[3]};
				w0 = qn(qm(qq, qc(q3)));
				have_w0 = 1;
			}
			pimax_cam_track_out o;
			memset(&o, 0x5a, sizeof o);
			int rc = pimax_cam_track_update_vio(t, (uint64_t)s->ts, &sv, &o);
			char tag[64];
			snprintf(tag, sizeof tag, "v %zu", i);
			note_out(tag, &o, 1, rc);
			if (o.events & PIMAX_CAM_TRACK_EV_RESET_WANTED) {
				pending_reset = 1;
			}
		}
		if (s->ts < next_q) {
			continue;
		}
		next_q += 11111111;
		pimax_cam_track_out o;
		memset(&o, 0x5a, sizeof o);
		int rc = pimax_cam_track_update_vio(t, (uint64_t)s->ts, NULL, &o);
		char tag[64];
		snprintf(tag, sizeof tag, "u %zu", i);
		note_out(tag, &o, 0, rc);
		if (o.events & PIMAX_CAM_TRACK_EV_RESET_WANTED) {
			pending_reset = 1;
		}
		pimax_cam_rel r3, out;
		memset(&r3, 0, sizeof r3);
		qarr(fus, r3.q);
		double wb[3] = {s->g[0] - bias[0], s->g[1] - bias[1], s->g[2] - bias[2]};
		qr(fus, wb, r3.ang_vel);
		r3.flags = 1 | 16 | 8;
		memset(&out, 0x5a, sizeof out);
		int st = pimax_cam_track_head(t, (uint64_t)s->ts, s->ts + 20000000, &r3, &out);
		snprintf(tag, sizeof tag, "q %zu", i);
		print_rel(tag, st, &out);
		qhash = fnv(&out, sizeof out, qhash);
		if (i % 2000 == 0) {
			print_info(t);
		}
		drain_big(t);
	}
	print_info(t);
	drain_big(t);
	printf("end updates %" PRIu64 " #%016" PRIx64 " queries #%016" PRIx64 "\n", g_upd_count, g_upd_hash, qhash);
	pimax_cam_track_destroy(t);
	free(imu);
	free(vio);
	return 0;
}

/* ---- imulog --------------------------------------------------------------------------------------------------- */

static int name_cmp(const void *a, const void *b)
{
	return strcmp(*(const char *const *)a, *(const char *const *)b);
}

static void list_dir(const char *label, const char *dir)
{
	DIR *d = opendir(dir);
	if (!d) {
		printf("%s: (no dir)\n", label);
		return;
	}
	char *names[64];
	int n = 0;
	struct dirent *e;
	while ((e = readdir(d)) && n < 64) {
		if (strcmp(e->d_name, ".") && strcmp(e->d_name, "..")) {
			names[n++] = strdup(e->d_name);
		}
	}
	closedir(d);
	qsort(names, (size_t)n, sizeof names[0], name_cmp);
	printf("%s:", label);
	for (int i = 0; i < n; i++) {
		char p[1200], l[1200];
		snprintf(p, sizeof p, "%s/%s", dir, names[i]);
		ssize_t r = readlink(p, l, sizeof l - 1);
		if (r >= 0) {
			l[r] = 0;
			/* a link into the work dir: print it relative */
			const char *rel = strstr(l, "/state/");
			printf(" %s->%s", names[i], rel ? rel : l);
		} else {
			printf(" %s", names[i]);
		}
		free(names[i]);
	}
	printf("\n");
}

/* Wait until the writer and keeper threads (detached in both libraries) are done: the file stops changing and no
 * .keeping file is left. */
static void settle(const char *path, const char *dir)
{
	off_t last = -1;
	int stable = 0;
	for (int i = 0; i < 400 && stable < 5; i++) {
		struct stat st;
		off_t sz = stat(path, &st) == 0 ? st.st_size : -2;
		int keeping = 0;
		DIR *d = opendir(dir);
		if (d) {
			struct dirent *e;
			while ((e = readdir(d))) {
				if (strstr(e->d_name, ".keeping")) {
					keeping = 1;
				}
			}
			closedir(d);
		}
		stable = (sz == last && !keeping) ? stable + 1 : 0;
		last = sz;
		struct timespec ts = {0, 20000000};
		nanosleep(&ts, NULL);
	}
}

static void log_session(const char *work, const char *label, int n_imu, int vio_every)
{
	printf("-- %s\n", label);
	char path[1100];
	snprintf(path, sizeof path, "%s/session/head-imu.csv", work);
	pimax_cam_track *t = pimax_cam_track_create(NULL, NULL);
	if (!t) {
		printf("create failed %s\n", err_text());
		return;
	}
	uint64_t seed = 99;
	for (int i = 0; i < n_imu; i++) {
		double a[3] = {0.02 * gauss(&seed), 9.80665 + 0.02 * gauss(&seed), -1e-7 * i};
		double g[3] = {1e-3 * gauss(&seed), -0.5 + 1e-3 * i, 123456.123456789};
		int64_t ts = 5000000000ll + (int64_t)i * 1000000;
		pimax_cam_track_push_imu(t, ts, a, g);
		if (i % vio_every == 0) {
			pimax_cam_track_vio s = {(uint64_t)ts - 3000000, {0.1 * i, -2.5e-6, 1e6}, {0.5, -0.5, 0.5, 0.5},
			                         {0, 0, 0},              {0, 0, 0, 1}};
			pimax_cam_track_out o;
			pimax_cam_track_update_vio(t, (uint64_t)ts, &s, &o);
			pimax_cam_track_update_vio(t, (uint64_t)ts, &s, &o); /* the same pose again: logged once */
			pimax_cam_track_update(t, (uint64_t)ts, NULL, &o);
		}
	}
	pimax_cam_track_destroy(t);
	char sdir[1100];
	snprintf(sdir, sizeof sdir, "%s/session", work);
	settle(path, sdir);
	g_private = 0;
	dump_file("log", path);
	list_dir("session", sdir);
	char kdir[1100];
	snprintf(kdir, sizeof kdir, "%s/state/omarchyvr/head-imu", work);
	list_dir("kept", kdir);
	char prev[1200];
	snprintf(prev, sizeof prev, "%s.prev", path);
	size_t n = 0;
	char *b = read_all(prev, &n);
	printf("prev: %s\n", b ? b : "(none)");
	free(b);
}

static int cmd_imulog(const char *work)
{
	char p[1100];
	snprintf(p, sizeof p, "%s/session", work);
	mkdirs(p);
	snprintf(p, sizeof p, "%s/session/head-imu.csv", work);
	setenv("PIMAX_HEAD_IMU_LOG", p, 1);
	char st[1100];
	snprintf(st, sizeof st, "%s/state", work);
	setenv("XDG_STATE_HOME", st, 1);
	setenv("TZ", "UTC", 1);
	tzset();
	char rt[1100];
	snprintf(rt, sizeof rt, "%s/run", work);
	mkdirs(rt);
	setenv("XDG_RUNTIME_DIR", rt, 1);
	log_session(work, "first", 3000, 40);
	/* the earlier log is kept: give it a fixed write time so its stamp is the same in both runs */
	struct timespec tt[2] = {{1760000000, 0}, {1760000000, 0}};
	utimensat(AT_FDCWD, p, tt, 0);
	setenv("PIMAX_HEAD_IMU_LOG_MIN", " 0.0001 ", 1); /* 6 ms, then only with the flag file */
	log_session(work, "short, no flag", 500, 40);
	utimensat(AT_FDCWD, p, tt, 0);
	char flag[1200];
	snprintf(flag, sizeof flag, "%s/omarchyvr-imu-log.on", rt);
	write_text(flag, "");
	log_session(work, "short, flag", 500, 40);
	unlink(flag);
	struct timespec t2[2] = {{1760000100, 0}, {1760000100, 0}};
	utimensat(AT_FDCWD, p, t2, 0);
	setenv("PIMAX_HEAD_IMU_LOG_MIN", "bad", 1);
	log_session(work, "bad minutes", 200, 50);
	utimensat(AT_FDCWD, p, t2, 0);
	setenv("PIMAX_HEAD_IMU_LOG_MIN", "-3", 1);
	log_session(work, "negative minutes", 200, 50);
	utimensat(AT_FDCWD, p, t2, 0);
	unsetenv("PIMAX_HEAD_IMU_LOG_MIN");
	log_session(work, "same stamp again", 100, 50);
	snprintf(p, sizeof p, "%s/nodir/head-imu.csv", work);
	setenv("PIMAX_HEAD_IMU_LOG", p, 1);
	pimax_cam_track *t = pimax_cam_track_create(NULL, NULL);
	printf("log in a missing dir: create %s\n", t ? "ok" : "failed");
	pimax_cam_track_destroy(t);
	setenv("PIMAX_HEAD_IMU_LOG", "", 1);
	t = pimax_cam_track_create(NULL, NULL);
	printf("empty log name: create %s\n", t ? "ok" : "failed");
	pimax_cam_track_destroy(t);
	return 0;
}

/* ---- errors ---------------------------------------------------------------------------------------------------- */

static int cmd_errors(void)
{
	pimax_cam_track_out o;
	pimax_cam_rel r = {{0}, {0, 0, 0, 1}, {0}, {0}, 0, 0}, out;
	pimax_cam_track_info info;
	pimax_cam_track_corr c;
	ERRLINE("update null", pimax_cam_track_update(NULL, 0, NULL, &o));
	ERRLINE("update_vio null", pimax_cam_track_update_vio(NULL, 0, NULL, &o));
	ERRLINE("head null", pimax_cam_track_head(NULL, 0, 0, &r, &out));
	ERRLINE("info null", pimax_cam_track_get_info(NULL, &info));
	ERRLINE("big null", pimax_cam_track_take_big_corr(NULL, &c));
	pimax_cam_track_push_imu(NULL, 0, NULL, NULL);
	pimax_cam_track_recenter(NULL);
	pimax_cam_track *t = pimax_cam_track_create(NULL, NULL);
	double a[3] = {0, 9.8, 0};
	pimax_cam_track_push_imu(t, 5, NULL, a);
	pimax_cam_track_push_imu(t, 5, a, NULL);
	ERRLINE("head null rel", pimax_cam_track_head(t, 0, 0, NULL, &out));
	ERRLINE("head null out", pimax_cam_track_head(t, 0, 0, &r, NULL));
	ERRLINE("info null out", pimax_cam_track_get_info(t, NULL));
	ERRLINE("big null out", pimax_cam_track_take_big_corr(t, NULL));
	printf("big none %d\n", pimax_cam_track_take_big_corr(t, &c));
	printf("update null out %d\n", pimax_cam_track_update(t, 0, NULL, NULL));
	memset(&out, 0x5a, sizeof out);
	int st = pimax_cam_track_head(t, 0, 0, &r, &out);
	print_rel("head fresh", st, &out);
	print_info(t);
	/* extreme times: wrapping in both */
	pimax_cam_track_push_imu(t, INT64_MIN, a, a);
	pimax_cam_track_push_imu(t, INT64_MAX, a, a);
	pimax_cam_track_vio s = {UINT64_MAX, {0}, {0, 0, 0, 1}, {0}, {0, 0, 0, 1}};
	memset(&o, 0, sizeof o);
	printf("update huge ts %d ", pimax_cam_track_update_vio(t, UINT64_MAX, &s, &o));
	print_out("o", &o);
	st = pimax_cam_track_head(t, UINT64_MAX, INT64_MIN, &r, &out);
	print_rel("head extreme", st, &out);
	print_info(t);
	pimax_cam_track_destroy(t);
	pimax_cam_track_params p;
	pimax_cam_track_default_params(&p);
	p.acquire_poses = 0;
	p.max_speed_m_s = NAN;
	p.stale_ns = 0;
	p.reset_after_ns = 1;
	p.reset_every_ns = 0;
	t = pimax_cam_track_create(&p, a);
	for (int i = 0; i < 6; i++) {
		pimax_cam_track_vio v = {(uint64_t)(1000 + i * 40000000), {0.01 * i, 0, 0}, {0, 0, 0, 1}, {0}, {0, 0, 0, 1}};
		memset(&o, 0, sizeof o);
		int rc = pimax_cam_track_update_vio(t, v.ts_ns + 500000000ull * (uint64_t)(i % 2), (i == 3) ? NULL : &v, &o);
		printf("odd params %d %d ", i, rc);
		print_out("o", &o);
	}
	pimax_cam_track_destroy(t);
	return 0;
}

int main(int argc, char **argv)
{
	/* the library's stderr lines go into the compared output, in order */
	setvbuf(stdout, NULL, _IONBF, 0);
	dup2(STDOUT_FILENO, STDERR_FILENO);
	/* a clean environment for the switches the library reads; -e sets them per test */
	static const char *const clean[] = {"PIMAX_SLAM_SMOOTH", "PIMAX_SLAM_TAU_MS", "PIMAX_SLAM_RESNAP", "PIMAX_SLAM_FUSE",
	                                    "PIMAX_FUSE_AUTO", "PIMAX_HEAD_TILT", "PIMAX_SLAM_TAU_REST_MS",
	                                    "PIMAX_SLAM_BIG_CORR_MM", "PIMAX_SLAM_DRIFT_HOLD", "PIMAX_SLAM_FREEZE",
	                                    "PIMAX_SLAM_STILL_REZERO", "PIMAX_SLAM_STILL", "PIMAX_HEAD_IMU_LOG",
	                                    "PIMAX_HEAD_IMU_LOG_MIN", "PIMAX_SLAM_IMU", "XDG_CACHE_HOME", "XDG_STATE_HOME"};
	for (size_t k = 0; k < sizeof clean / sizeof clean[0]; k++) {
		unsetenv(clean[k]);
	}
	int i = 1;
	while (i + 1 < argc && (!strcmp(argv[i], "-e") || !strcmp(argv[i], "-u"))) {
		if (!strcmp(argv[i], "-e")) {
			char *kv = argv[i + 1];
			char *eq = strchr(kv, '=');
			if (eq) {
				*eq = 0;
				setenv(kv, eq + 1, 1);
				*eq = '=';
			}
		} else {
			unsetenv(argv[i + 1]);
		}
		i += 2;
	}
	if (i >= argc) {
		fprintf(stderr, "usage: slam_difftest [-e K=V | -u K]... layout|slam|imu|synth|replay|imulog|errors ...\n");
		return 2;
	}
	const char *mode = argv[i];
	char **a = argv + i + 1;
	int n = argc - i - 1;
	if (!strcmp(mode, "layout")) {
		return cmd_layout();
	}
	if (!strcmp(mode, "slam") && n >= 2) {
		g_private = n >= 3 && !strcmp(a[2], "private");
		return cmd_slam(a[0], a[1]);
	}
	if (!strcmp(mode, "imu") && n >= 1) {
		return cmd_imu(a[0]);
	}
	if (!strcmp(mode, "synth")) {
		return cmd_synth();
	}
	if (!strcmp(mode, "replay") && n >= 2) {
		return cmd_replay(a[0], a[1], n >= 3 ? atol(a[2]) : 0);
	}
	if (!strcmp(mode, "imulog") && n >= 1) {
		return cmd_imulog(a[0]);
	}
	if (!strcmp(mode, "errors")) {
		return cmd_errors();
	}
	fprintf(stderr, "bad mode %s\n", mode);
	return 2;
}
