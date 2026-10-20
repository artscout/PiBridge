// SPDX-License-Identifier: MIT
// Difftest driver for the pimax_cam.h ABI (docs/research/rust-port-plan.md, M0-M2). The same C program is compiled
// twice, against omarchyvr's libpimax_cam.a (the oracle) and against PiBridge's libpibridge_pimax.a; run_difftest.sh
// compares the two outputs byte for byte. Everything printed is deterministic: doubles as exact hex (%a), replay
// times relative to the first frame (a replay rebases them onto the current clock), tiles as FNV-1a hashes.
//
//   cam_difftest layout
//   cam_difftest calib FILE...          parse, mutations, stereo / body poses of each calibration XML
//   cam_difftest load WORKDIR REFXML    default search (HOME, PIMAX_CAM_CALIB, ...) in a scratch home
//   cam_difftest clock                  pimax_cam_dev_to_mono_ns over a grid
//   cam_difftest replay FILE [LOOPFRAMES]
//   cam_difftest record SRC WORKDIR N   record through the ABI, read the file back
//   cam_difftest errors WORKDIR         NULL handles, broken files, error texts
//   cam_difftest client SOCKET REF [SLOW_MS]   frames from a running broker checked against a replay of REF
//   cam_difftest hello SOCKET           the raw PXCB hello and ring header of a running broker (no library)
//   cam_difftest pxcam FILE             the structure of a .pxcam file, times relative to the first record
//   cam_difftest extract FILE N OUT     the first N raw frames of an uncompressed .pxcam (input for `import`)
//
// Nothing here opens a camera: sources are recordings, sockets in a temp dir and a nonexistent /dev path.
#define _GNU_SOURCE
#include "pimax_cam.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
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

static void write_file(const char *path, const void *data, size_t n)
{
	FILE *f = fopen(path, "wb");
	if (!f) {
		printf("cannot write %s\n", path);
		exit(2);
	}
	fwrite(data, 1, n, f);
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

/* ---- layout ------------------------------------------------------------------------------------------------- */

static int cmd_layout(void)
{
	printf("frame_info %zu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n",
	       sizeof(pimax_cam_frame_info), O(pimax_cam_frame_info, host_ts_ns), O(pimax_cam_frame_info, dev_ts_us),
	       O(pimax_cam_frame_info, mono_ts_ns), O(pimax_cam_frame_info, counter), O(pimax_cam_frame_info, sequence),
	       O(pimax_cam_frame_info, dropped), O(pimax_cam_frame_info, kind), O(pimax_cam_frame_info, frame_type),
	       O(pimax_cam_frame_info, stream_index), O(pimax_cam_frame_info, exposure), O(pimax_cam_frame_info, gain),
	       O(pimax_cam_frame_info, y_offset), O(pimax_cam_frame_info, tile_width),
	       O(pimax_cam_frame_info, tile_height), O(pimax_cam_frame_info, flags), O(pimax_cam_frame_info, reserved));
	printf("stats %zu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_stats), O(pimax_cam_stats, tracking),
	       O(pimax_cam_stats, led), O(pimax_cam_stats, unknown), O(pimax_cam_stats, dropped),
	       O(pimax_cam_stats, corrupt), O(pimax_cam_stats, scanned_rows), O(pimax_cam_stats, row_mismatch),
	       O(pimax_cam_stats, kind_repeats), O(pimax_cam_stats, clock_offset_ns));
	printf("clockref %zu %lu %lu %lu\n", sizeof(pimax_cam_imu_clock_ref), O(pimax_cam_imu_clock_ref, valid),
	       O(pimax_cam_imu_clock_ref, dev_ext_us), O(pimax_cam_imu_clock_ref, offset_ns));
	printf("kb4 %zu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_kb4), O(pimax_cam_kb4, height), O(pimax_cam_kb4, fx),
	       O(pimax_cam_kb4, fy), O(pimax_cam_kb4, cy), O(pimax_cam_kb4, k), O(pimax_cam_kb4, extra));
	printf("camera %zu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_camera), O(pimax_cam_camera, id),
	       O(pimax_cam_camera, reserved), O(pimax_cam_camera, intr), O(pimax_cam_camera, rig_rotation),
	       O(pimax_cam_camera, rig_translation), O(pimax_cam_camera, time_delta_s));
	printf("calib %zu %lu %lu %lu %lu %lu %lu\n", sizeof(pimax_cam_calib), O(pimax_cam_calib, has_imu),
	       O(pimax_cam_calib, cameras), O(pimax_cam_calib, ombc), O(pimax_cam_calib, tbc),
	       O(pimax_cam_calib, imu_cam_delta_s), O(pimax_cam_calib, device_uid));
	printf("open_params %zu %lu %lu %lu\n", sizeof(pimax_cam_open_params), O(pimax_cam_open_params, replay_flags),
	       O(pimax_cam_open_params, verify_rows), O(pimax_cam_open_params, reserved));
	printf("version %s\n", pimax_cam_version());
	printf("initial error \"%s\"\n", pimax_cam_last_error());
	return 0;
}

/* ---- calibration -------------------------------------------------------------------------------------------- */

static void dump_calib(FILE *o, const pimax_cam_calib *c)
{
	fprintf(o, "count %u has_imu %u uid_hash %016" PRIx64 "\n", c->camera_count, c->has_imu,
	        fnv(c->device_uid, sizeof c->device_uid, FNV0));
	for (int i = 0; i < 4; i++) {
		const pimax_cam_camera *m = &c->cameras[i];
		fprintf(o, " cam %d name \"%.32s\" id %d res %u %ux%u f %a %a c %a %a k %a %a %a %a x %a %a\n", i, m->name,
		        m->id, m->reserved, m->intr.width, m->intr.height, m->intr.fx, m->intr.fy, m->intr.cx, m->intr.cy,
		        m->intr.k[0], m->intr.k[1], m->intr.k[2], m->intr.k[3], m->intr.extra[0], m->intr.extra[1]);
		fprintf(o, "  R");
		for (int j = 0; j < 9; j++) {
			fprintf(o, " %a", m->rig_rotation[j]);
		}
		fprintf(o, " t %a %a %a dt %a\n", m->rig_translation[0], m->rig_translation[1], m->rig_translation[2],
		        m->time_delta_s);
	}
	fprintf(o, " ombc %a %a %a tbc %a %a %a delta %a\n", c->ombc[0], c->ombc[1], c->ombc[2], c->tbc[0], c->tbc[1],
	        c->tbc[2], c->imu_cam_delta_s);
}

static void dump_poses(FILE *o, const pimax_cam_calib *c)
{
	double r[9], t[3];
	for (uint32_t a = 0; a < 6; a++) {
		for (uint32_t b = 0; b < 6; b++) {
			int rc = pimax_cam_calib_stereo(c, a, b, r, t);
			if (rc != 0) {
				if (a < 5 && b < 5) {
					fprintf(o, " stereo %u %u -> %d %s\n", a, b, rc, pimax_cam_last_error());
				}
				continue;
			}
			fprintf(o, " stereo %u %u R", a, b);
			for (int j = 0; j < 9; j++) {
				fprintf(o, " %a", r[j]);
			}
			fprintf(o, " t %a %a %a\n", t[0], t[1], t[2]);
		}
	}
	for (uint32_t i = 0; i < 6; i++) {
		int rc = pimax_cam_calib_body_from_cam(c, i, r, t);
		if (rc != 0) {
			fprintf(o, " body %u -> %d %s\n", i, rc, pimax_cam_last_error());
			continue;
		}
		fprintf(o, " body %u R", i);
		for (int j = 0; j < 9; j++) {
			fprintf(o, " %a", r[j]);
		}
		fprintf(o, " t %a %a %a\n", t[0], t[1], t[2]);
	}
	int n1 = pimax_cam_calib_stereo(NULL, 0, 1, r, t);
	fprintf(o, " null calib: %d %s\n", n1, pimax_cam_last_error());
	n1 = pimax_cam_calib_stereo(c, 0, 1, NULL, t);
	fprintf(o, " null r: %d %s\n", n1, pimax_cam_last_error());
	n1 = pimax_cam_calib_body_from_cam(c, 0, r, NULL);
	fprintf(o, " null t: %d %s\n", n1, pimax_cam_last_error());
}

/* parse one document; full dump when verbose, else one line with a hash of the dump */
static void parse_one(const char *label, const char *doc, size_t len, int verbose)
{
	pimax_cam_calib c;
	memset(&c, 0x5a, sizeof c);
	int rc = pimax_cam_calib_parse(doc, len, &c);
	if (rc != 0) {
		printf("%s -> %d %s\n", label, rc, pimax_cam_last_error());
		return;
	}
	if (verbose) {
		printf("%s -> ok\n", label);
		dump_calib(stdout, &c);
		dump_poses(stdout, &c);
		return;
	}
	char *buf = NULL;
	size_t blen = 0;
	FILE *m = open_memstream(&buf, &blen);
	dump_calib(m, &c);
	dump_poses(m, &c);
	fclose(m);
	printf("%s -> ok %016" PRIx64 "\n", label, fnv(buf, blen, FNV0));
	free(buf);
}

static const char *const NUM_VARIANTS[] = {"abc", "nan", "inf", "-inf", "+1.5", "1e999", "-0", "1e-400", "0x10",
                                           ".5",  "5.",  "1_0", "+-1",  "1e",   "E5",    "",   "1.0e+2", "\xc2\xa0" "7"};

static int cmd_calib(int argc, char **argv)
{
	/* synthetic documents */
	static const char *const SYN[] = {
	    "",
	    "<x/>",
	    "<DeviceConfiguration/>",
	    "<DeviceConfiguration",
	    "<DeviceConfiguration deviceUID='a&amp;b&#65;&#x42;&#X43;&lt;'><Camera/></DeviceConfiguration>",
	    "<DeviceConfiguration><Camera id='x'/></DeviceConfiguration>",
	    "<DeviceConfiguration><Camera id=' 0 '/></DeviceConfiguration>",
	    "<DeviceConfiguration><Camera id='+0'><Calibration/></Camera></DeviceConfiguration>",
	    "<DeviceConfiguration><Camera id='-0'><Calibration/></Camera></DeviceConfiguration>",
	    "<DeviceConfiguration><Camera id='0'><Calibration/><Rig/></Camera></DeviceConfiguration>",
	    "<A x='&bogus;'/>",
	    "<A x='&#xd800;'/>",
	    "<A x='&#1114112;'/>",
	    "<A x='&#+65;'/>",
	    "<A/><B/>",
	    "<A></B>",
	    "<A x=1/>",
	    "<A x=\"unterminated/>",
	    "<!-- c --><!DOCTYPE x><?pi?><A><![CDATA[ <x/> ]]></A>",
	    "<!-- unterminated",
	    "<A><!-- x --></A >",
	    "<A></A",
	    "< A/>",
	    "<A b c/>",
	    "<A\x0b/>",
	    "<\xc3\xa9/>",
	    "<\xff\xfe/>",
	};
	char label[64];
	for (size_t i = 0; i < sizeof SYN / sizeof SYN[0]; i++) {
		snprintf(label, sizeof label, "syn %zu", i);
		parse_one(label, SYN[i], strlen(SYN[i]), 0);
	}
	/* deep nesting: 64 levels pass the XML limit, 66 do not */
	for (int depth = 64; depth <= 67; depth++) {
		char d[2048] = "";
		for (int k = 0; k < depth; k++) {
			strcat(d, "<a>");
		}
		for (int k = 0; k < depth; k++) {
			strcat(d, "</a>");
		}
		snprintf(label, sizeof label, "deep %d", depth);
		parse_one(label, d, strlen(d), 0);
	}
	for (int f = 0; f < argc; f++) {
		size_t len;
		char *doc = read_all(argv[f], &len);
		if (!doc) {
			printf("file %d: cannot read\n", f);
			return 1;
		}
		snprintf(label, sizeof label, "file %d", f);
		parse_one(label, doc, len, 1);
		/* every prefix length in steps of 7 */
		for (size_t n = 0; n < len; n += 7) {
			snprintf(label, sizeof label, "file %d prefix %zu", f, n);
			parse_one(label, doc, n, 0);
		}
		/* structural bytes and invalid UTF-8 at every 13th position */
		static const char FLIP[] = {'<', '>', '"', '&', ' ', '\0', '/', '=', (char)0xff};
		char *m = malloc(len + 1);
		for (size_t p = 0; p < len; p += 13) {
			for (size_t v = 0; v < sizeof FLIP; v++) {
				memcpy(m, doc, len);
				m[p] = FLIP[v];
				snprintf(label, sizeof label, "file %d flip %zu %zu", f, p, v);
				parse_one(label, m, len, 0);
			}
		}
		free(m);
		/* number tokens replaced by the variants (first 60 tokens) */
		size_t tok = 0;
		for (size_t p = 0; p < len && tok < 60; p++) {
			int starts = (doc[p] >= '0' && doc[p] <= '9') || doc[p] == '-';
			if (!starts || (p > 0 && (doc[p - 1] != ' ' && doc[p - 1] != '"'))) {
				continue;
			}
			size_t e = p;
			while (e < len && doc[e] != ' ' && doc[e] != '"') {
				e++;
			}
			for (size_t v = 0; v < sizeof NUM_VARIANTS / sizeof NUM_VARIANTS[0]; v++) {
				const char *rep = NUM_VARIANTS[v];
				size_t rl = strlen(rep);
				char *d2 = malloc(len - (e - p) + rl + 1);
				memcpy(d2, doc, p);
				memcpy(d2 + p, rep, rl);
				memcpy(d2 + p + rl, doc + e, len - e);
				snprintf(label, sizeof label, "file %d tok %zu v %zu", f, tok, v);
				parse_one(label, d2, len - (e - p) + rl, 0);
				free(d2);
			}
			tok++;
			p = e;
		}
		free(doc);
	}
	/* NULL arguments */
	pimax_cam_calib c;
	{
		long long rc_ = (long long)(pimax_cam_calib_parse(NULL, 0, &c));
		printf("null parse %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_calib_parse("<x/>", 4, NULL));
		printf("null out %lld %s\n", rc_, pimax_cam_last_error());
	}
	/* a calibration with more than 4 cameras in the struct (camera_count is trusted by the index check) */
	memset(&c, 0, sizeof c);
	c.camera_count = 8;
	double r[9], t[3];
	{
		long long rc_ = (long long)(pimax_cam_calib_stereo(&c, 0, 5, r, t));
		printf("count 8: stereo 0 5 -> %lld %s\n", rc_, pimax_cam_last_error());
	}
	return 0;
}

/* ---- default search ----------------------------------------------------------------------------------------- */

static char W[900];

static void show_load(const char *label, const char *path, size_t used_len)
{
	pimax_cam_calib c;
	char used[1024];
	memset(used, '#', sizeof used);
	int rc = pimax_cam_calib_load(path, &c, used_len ? used : NULL, used_len);
	if (rc != 0) {
		printf("%s -> %d %s\n", label, rc, pimax_cam_last_error());
		return;
	}
	printf("%s -> ok used \"%s\" uid \"%.32s\" count %u fx %a\n", label, used_len ? used : "-", c.device_uid,
	       c.camera_count, c.cameras[0].intr.fx);
}

static void put_xml(const char *rel, const char *ref, size_t len, const char *uid)
{
	char p[1200];
	snprintf(p, sizeof p, "%s/%s", W, rel);
	char dir[1200];
	snprintf(dir, sizeof dir, "%s", p);
	*strrchr(dir, '/') = 0;
	mkdirs(dir);
	/* replace deviceUID="reference" by the given uid */
	const char *at = strstr(ref, "deviceUID=\"reference\"");
	FILE *f = fopen(p, "wb");
	if (at) {
		fwrite(ref, 1, (size_t)(at - ref), f);
		fprintf(f, "deviceUID=\"%s\"", uid);
		const char *rest = at + strlen("deviceUID=\"reference\"");
		fwrite(rest, 1, len - (size_t)(rest - ref), f);
	} else {
		fwrite(ref, 1, len, f);
	}
	fclose(f);
}

static int cmd_load(const char *work, const char *refxml)
{
	snprintf(W, sizeof W, "%s", work);
	size_t len;
	char *ref = read_all(refxml, &len);
	if (!ref) {
		printf("cannot read %s\n", refxml);
		return 1;
	}
	unsetenv("PIMAX_CAM_CALIB");
	unsetenv("PIMAX_CALIB");
	unsetenv("PIMAX_REPO");
	unsetenv("PIMAX_NO_REFERENCE");
	unsetenv("PIBRIDGE_CAM_CALIB");
	unsetenv("PIBRIDGE_CALIB");
	unsetenv("PIBRIDGE_REPO");
	char home[1200], p[1200];
	snprintf(home, sizeof home, "%s/h1", W);
	mkdirs(home);
	setenv("HOME", home, 1);
	show_load("empty home", NULL, 512);
	setenv("PIMAX_NO_REFERENCE", "1", 1);
	show_load("no reference", NULL, 512);
	setenv("PIMAX_NO_REFERENCE", "0", 1);
	show_load("no reference=0", NULL, 512);
	setenv("PIMAX_NO_REFERENCE", "", 1);
	show_load("no reference empty", NULL, 512);
	unsetenv("PIMAX_NO_REFERENCE");
	put_xml("h1/.local/share/omarchyvr/data/rust/pimax-device/devices/dream-air/reference/headset-calibration.xml",
	        ref, len, "reference");
	show_load("installed reference", NULL, 512);
	put_xml("h1/omarchyvr/analysis/headset-calibration.xml", ref, len, "home-repo");
	show_load("home repo", NULL, 512);
	put_xml("h1/.local/share/omarchyvr/data/analysis/headset-calibration.xml", ref, len, "data-copy-no-marker");
	show_load("data copy without LESSONS", NULL, 512);
	put_xml("h1/.local/share/omarchyvr/data/docs/LESSONS.md", "x", 1, "");
	show_load("data copy", NULL, 512);
	put_xml("repo/analysis/headset-calibration.xml", ref, len, "pimax-repo");
	snprintf(p, sizeof p, "%s/repo", W);
	setenv("PIMAX_REPO", p, 1);
	show_load("PIMAX_REPO", NULL, 512);
	put_xml("h1/.config/omarchyvr/headset-calibration.xml", ref, len, "own");
	show_load("own config", NULL, 512);
	snprintf(p, sizeof p, "%s/missing.xml", W);
	setenv("PIMAX_CAM_CALIB", p, 1);
	show_load("PIMAX_CAM_CALIB missing", NULL, 512);
	put_xml("calib.xml", ref, len, "pimax-calib");
	snprintf(p, sizeof p, "%s/calib.xml", W);
	setenv("PIMAX_CALIB", p, 1);
	show_load("PIMAX_CALIB", NULL, 512);
	put_xml("cam-calib.xml", ref, len, "pimax-cam-calib");
	snprintf(p, sizeof p, "%s/cam-calib.xml", W);
	setenv("PIMAX_CAM_CALIB", p, 1);
	show_load("PIMAX_CAM_CALIB", NULL, 512);
	show_load("PIMAX_CAM_CALIB short buffer", NULL, 6);
	show_load("PIMAX_CAM_CALIB len 1", NULL, 1);
	show_load("PIMAX_CAM_CALIB no buffer", NULL, 0);
	setenv("PIMAX_CAM_CALIB", "", 1);
	show_load("PIMAX_CAM_CALIB empty", NULL, 512);
	snprintf(p, sizeof p, "%s/cam-calib.xml", W);
	show_load("explicit", p, 512);
	show_load("explicit empty string", "", 512);
	snprintf(p, sizeof p, "%s/nothing.xml", W);
	show_load("explicit missing", p, 512);
	show_load("explicit directory", W, 512);
	snprintf(p, sizeof p, "%s/bad-utf8.xml", W);
	write_file(p, "<DeviceConfiguration deviceUID='\xff'/>", 35);
	show_load("explicit invalid utf-8", p, 512);
	snprintf(p, sizeof p, "%s/bad.xml", W);
	write_file(p, "<Foo/>", 6);
	show_load("explicit not a calibration", p, 512);
	snprintf(p, sizeof p, "%s/cam-calib.xml", W);
	{
		long long rc_ = (long long)(pimax_cam_calib_load(p, NULL, NULL, 0));
		printf("null out %lld %s\n", rc_, pimax_cam_last_error());
	}
	unsetenv("PIMAX_CAM_CALIB");
	unsetenv("PIMAX_CALIB");
	unsetenv("PIMAX_REPO");
	unsetenv("HOME");
	show_load("no HOME", NULL, 512);
	setenv("PIMAX_NO_REFERENCE", "1", 1);
	show_load("no HOME no reference", NULL, 512);
	setenv("HOME", "", 1);
	show_load("HOME empty no reference", NULL, 512);
	free(ref);
	return 0;
}

/* ---- clock -------------------------------------------------------------------------------------------------- */

static int cmd_clock(void)
{
	static const uint32_t TS[] = {0, 1, 1000, 0x7fffffffu, 0x80000000u, 0xffffff00u, 0xffffffffu};
	static const uint64_t EXT[] = {0, 1000, 5000000000000ull, 0x7fffffffffffffffull, 0xffffffffffffffffull};
	static const int64_t OFF[] = {0, 5, -5, 1000000000, -5000000000000000ll, INT64_MAX, INT64_MIN};
	static const uint64_t DEV[] = {0, 3000, 0x100, (7ull << 32) | 0x100, 0xffffff00u - 20000, 0xffffffffffffffffull,
	                               0x80000000ull, 0x17fffffffull};
	uint64_t h = FNV0;
	unsigned n = 0;
	for (size_t a = 0; a < sizeof TS / sizeof TS[0]; a++) {
		for (size_t b = 0; b < sizeof EXT / sizeof EXT[0]; b++) {
			for (size_t c = 0; c < sizeof OFF / sizeof OFF[0]; c++) {
				pimax_cam_imu_clock_ref r = {TS[a], 1, EXT[b], OFF[c]};
				for (size_t d = 0; d < sizeof DEV / sizeof DEV[0]; d++) {
					uint64_t v = pimax_cam_dev_to_mono_ns(DEV[d], &r);
					if (n < 40) {
						printf("ts %u ext %" PRIu64 " off %" PRId64 " dev %" PRIu64 " -> %" PRIu64 "\n", TS[a],
						       EXT[b], OFF[c], DEV[d], v);
					}
					h = fnv(&v, sizeof v, h);
					n++;
				}
			}
		}
	}
	pimax_cam_imu_clock_ref inv = {1000, 0, 1000, 5};
	printf("grid %u hash %016" PRIx64 " invalid %" PRIu64 " null %" PRIu64 "\n", n, h, pimax_cam_dev_to_mono_ns(3000, &inv),
	       pimax_cam_dev_to_mono_ns(3000, NULL));
	return 0;
}

/* ---- replay ------------------------------------------------------------------------------------------------- */

static uint8_t *g_tile;

/* one frame line: everything deterministic of the frame info and the four tile hashes */
static void frame_line(char *out, size_t cap, pimax_cam *c, const pimax_cam_frame_info *fi, uint64_t host0)
{
	uint64_t th[4];
	for (uint32_t t = 0; t < 4; t++) {
		if (pimax_cam_copy_tile(c, t, g_tile, 640) != 0) {
			th[t] = 0;
			continue;
		}
		th[t] = fnv(g_tile, 640 * 480, FNV0);
	}
	snprintf(out, cap,
	         "dev %" PRIu64 " host+%" PRIu64 " mono%+" PRId64 " ctr %u seq %u drop %u kind %u type %#x idx %u exp %u %u %u %u gain %u "
	         "%u %u %u rows %u %u %u %u tile %ux%u flags %#x res %u tiles %016" PRIx64 " %016" PRIx64 " %016" PRIx64 " %016" PRIx64,
	         fi->dev_ts_us, fi->host_ts_ns - host0, (int64_t)(fi->mono_ts_ns - host0), fi->counter, fi->sequence,
	         fi->dropped, fi->kind, fi->frame_type, fi->stream_index, fi->exposure[0], fi->exposure[1], fi->exposure[2],
	         fi->exposure[3], fi->gain[0], fi->gain[1], fi->gain[2], fi->gain[3], fi->y_offset[0], fi->y_offset[1],
	         fi->y_offset[2], fi->y_offset[3], fi->tile_width, fi->tile_height, fi->flags, fi->reserved, th[0], th[1],
	         th[2], th[3]);
}

static void print_stats(pimax_cam *c, uint64_t host0)
{
	pimax_cam_stats st;
	int rc = pimax_cam_get_stats(c, &st);
	printf("stats rc %d frames %" PRIu64 " tracking %" PRIu64 " led %" PRIu64 " unknown %" PRIu64 " dropped %" PRIu64
	       " corrupt %" PRIu64 " scanned %" PRIu64 " mismatch %" PRIu64 " repeats %" PRIu64 " offset-host0 %" PRId64 "\n",
	       rc, st.frames, st.tracking, st.led, st.unknown, st.dropped, st.corrupt, st.scanned_rows, st.row_mismatch,
	       st.kind_repeats, st.clock_offset_ns - (int64_t)host0);
}

static int replay_dump(const char *path, uint32_t flags, long max_frames, int verify)
{
	pimax_cam_open_params p = {0};
	p.replay_flags = flags;
	p.verify_rows = (uint32_t)verify;
	pimax_cam *c = pimax_cam_open(path, &p);
	if (!c) {
		printf("open error: %s\n", pimax_cam_last_error());
		return 1;
	}
	char desc[512];
	size_t dl = pimax_cam_describe(c, desc, sizeof desc);
	printf("describe %zu \"%s\"\n", dl, desc);
	pimax_cam_frame_info fi;
	uint64_t host0 = 0;
	long n = 0;
	int rc;
	char line[1024];
	uint8_t *wide = malloc(700 * 480);
	while ((max_frames < 0 || n < max_frames) && (rc = pimax_cam_next(c, 0, &fi)) == 1) {
		if (n == 0) {
			host0 = fi.host_ts_ns;
		}
		frame_line(line, sizeof line, c, &fi, host0);
		/* stride 700: only the 640 tile bytes of each row are written */
		memset(wide, 0xee, 700 * 480);
		pimax_cam_copy_tile(c, 3, wide, 700);
		printf("frame %ld %s wide3 %016" PRIx64 "\n", n, line, fnv(wide, 700 * 480, FNV0));
		n++;
	}
	if (max_frames >= 0 && n >= max_frames) {
		rc = 1;
	}
	printf("end rc %d after %ld frames: %s\n", rc, n, rc == 1 ? "" : pimax_cam_last_error());
	print_stats(c, host0);
	/* after the end: the last frame is still the current one */
	{
		long long rc_ = (long long)(pimax_cam_copy_tile(c, 9, g_tile, 640));
		printf("copy tile 9 %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_copy_tile(c, 0, NULL, 640));
		printf("copy null %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_copy_tile(c, 0, wide, 639));
		printf("copy stride 639 %lld %s\n", rc_, pimax_cam_last_error());
	}
	int after = pimax_cam_copy_tile(c, 1, g_tile, 640);
	printf("copy after end %d %016" PRIx64 "\n", after, after == 0 ? fnv(g_tile, 640 * 480, FNV0) : 0);
	{
		long long rc_ = (long long)(pimax_cam_next(c, 0, &fi));
		printf("next after end %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_get_stats(c, NULL));
		printf("stats null out %lld %s\n", rc_, pimax_cam_last_error());
	}
	printf("record stop idle %" PRId64 "\n", pimax_cam_record_stop(c));
	free(wide);
	pimax_cam_close(c);
	return 0;
}

static int cmd_replay(const char *path, long loop_frames)
{
	if (loop_frames > 0) {
		return replay_dump(path, PIMAX_CAM_REPLAY_LOOP, loop_frames, 1);
	}
	return replay_dump(path, 0, -1, 1);
}

/* ---- record ------------------------------------------------------------------------------------------------- */

static void dump_pxcam(const char *path, int payload_hash)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		printf("cannot open %s\n", path);
		return;
	}
	uint8_t h[32];
	if (fread(h, 1, 32, f) != 32) {
		printf("short file\n");
		fclose(f);
		return;
	}
	printf("file header");
	for (int i = 0; i < 32; i++) {
		printf(" %02x", h[i]);
	}
	printf("\n");
	uint8_t r[32];
	uint64_t ts0 = 0;
	int n = 0;
	uint8_t *buf = malloc(4 << 20);
	while (fread(r, 1, 32, f) == 32) {
		uint32_t stored, raw, flags, seq, res;
		uint64_t ts;
		memcpy(&flags, r + 4, 4);
		memcpy(&stored, r + 8, 4);
		memcpy(&raw, r + 12, 4);
		memcpy(&ts, r + 16, 8);
		memcpy(&seq, r + 24, 4);
		memcpy(&res, r + 28, 4);
		if (n == 0) {
			ts0 = ts;
		}
		if (stored > (4u << 20) || fread(buf, 1, stored, f) != stored) {
			printf("record %d truncated\n", n);
			break;
		}
		printf("record %d magic %02x%02x%02x%02x flags %u raw %u seq %u res %u ts+%" PRIu64, n, r[0], r[1], r[2], r[3],
		       flags, raw, seq, res, ts - ts0);
		if (payload_hash) {
			printf(" stored %u payload %016" PRIx64, stored, fnv(buf, stored, FNV0));
		}
		printf("\n");
		n++;
	}
	free(buf);
	fclose(f);
}

static int cmd_record(const char *src, const char *work, long n)
{
	for (uint32_t level = 0; level <= 6; level += 6) {
		char out[1200];
		snprintf(out, sizeof out, "%s/rec%u.pxcam", work, level);
		pimax_cam *c = pimax_cam_open(src, NULL);
		if (!c) {
			printf("open error: %s\n", pimax_cam_last_error());
			return 1;
		}
		printf("level %u start %d\n", level, pimax_cam_record_start(c, out, level));
		pimax_cam_frame_info fi;
		long k = 0;
		while (k < n && pimax_cam_next(c, 0, &fi) == 1) {
			k++;
			/* slow enough that the writer thread never drops a frame (the queue holds 64) */
			struct timespec ts = {0, (level == 0 ? 5 : 60) * 1000 * 1000};
			nanosleep(&ts, NULL);
		}
		int64_t w = pimax_cam_record_stop(c);
		printf("level %u read %ld written %" PRId64 " stop again %" PRId64 "\n", level, k, w, pimax_cam_record_stop(c));
		pimax_cam_close(c);
		dump_pxcam(out, level == 0);
		replay_dump(out, 0, -1, 1);
	}
	/* recording to a directory that does not exist, and with a NULL path */
	pimax_cam *c = pimax_cam_open(src, NULL);
	char bad[1200];
	snprintf(bad, sizeof bad, "%s/no/such/dir/x.pxcam", work);
	{
		long long rc_ = (long long)(pimax_cam_record_start(c, bad, 0));
		printf("bad dir %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_record_start(c, NULL, 0));
		printf("null path %lld %s\n", rc_, pimax_cam_last_error());
	}
	/* close while recording flushes the file */
	char out[1200];
	snprintf(out, sizeof out, "%s/close.pxcam", work);
	printf("close-rec start %d\n", pimax_cam_record_start(c, out, 0));
	pimax_cam_frame_info fi;
	for (int k = 0; k < 3; k++) {
		pimax_cam_next(c, 0, &fi);
	}
	pimax_cam_close(c);
	dump_pxcam(out, 1);
	return 0;
}

/* ---- errors ------------------------------------------------------------------------------------------------- */

static int g_mask_zlib;

static const char *err_text(void)
{
	const char *e = pimax_cam_last_error();
	if (g_mask_zlib && strncmp(e, "zlib: ", 6) == 0) {
		return "zlib: <decoder detail>";
	}
	return e;
}

static void open_show(const char *label, const char *src)
{
	pimax_cam *c = pimax_cam_open(src, NULL);
	if (!c) {
		printf("%s: NULL %s\n", label, err_text());
		return;
	}
	char d[256];
	size_t n = pimax_cam_describe(c, d, sizeof d);
	pimax_cam_frame_info fi;
	int rc = pimax_cam_next(c, 0, &fi);
	printf("%s: open ok (%zu) next %d %s\n", label, n, rc, rc == 1 ? "" : err_text());
	rc = pimax_cam_next(c, 0, &fi);
	printf("%s: next %d %s\n", label, rc, rc == 1 ? "" : err_text());
	pimax_cam_close(c);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	memcpy(p, &v, 4);
}

static int cmd_errors(const char *work)
{
	pimax_cam_frame_info fi;
	pimax_cam_stats st;
	char buf[64];
	{
		long long rc_ = (long long)(pimax_cam_next(NULL, 0, &fi));
		printf("null next %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_copy_tile(NULL, 0, (uint8_t *)buf, 640));
		printf("null copy %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_get_stats(NULL, &st));
		printf("null stats %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_describe(NULL, buf, sizeof buf));
		printf("null describe %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_record_start(NULL, "x", 0));
		printf("null record start %lld %s\n", rc_, pimax_cam_last_error());
	}
	{
		long long rc_ = (long long)(pimax_cam_record_stop(NULL));
		printf("null record stop %lld %s\n", rc_, pimax_cam_last_error());
	}
	pimax_cam_close(NULL);
	open_show("nonexistent", "/nonexistent/clip.pxcam");
	open_show("dev missing", "/dev/does-not-exist-pimax");
	open_show("quoted name", "/nonexistent/a \"b\"\\c\td\x01.pxcam");
	open_show("invalid utf-8 name", "/nonexistent/\xff\xfe.pxcam");
	char sock[1200];
	snprintf(sock, sizeof sock, "broker:%s/no-broker.sock", work);
	open_show("broker missing", sock);
	snprintf(sock, sizeof sock, "  broker::%s/no-broker.sock  ", work);
	open_show("broker spaces colons", sock);
	open_show("broker path too long",
	          "broker:/tmp/xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx.sock");
	open_show("brokerx", "brokerx");
	char p[1200];
	snprintf(p, sizeof p, "%s/short.pxcam", work);
	write_file(p, "PXCAMRAWgarbage", 15);
	open_show("short", p);
	snprintf(p, sizeof p, "%s/nope.pxcam", work);
	write_file(p, "nope", 4);
	open_show("nope", p);
	open_show("directory", work);
	/* headers: good, bad version, bad size, longer header */
	uint8_t h[64];
	memset(h, 0, sizeof h);
	memcpy(h, "PXCAMRAW", 8);
	h[8] = 1;
	h[10] = 32;
	h[12] = 0x00;
	h[13] = 0x0a; /* 2560 */
	h[14] = 0x40;
	h[15] = 0x03; /* 832 */
	put_le32(h + 16, 2560);
	snprintf(p, sizeof p, "%s/empty.pxcam", work);
	write_file(p, h, 32);
	open_show("empty", p);
	h[8] = 2;
	snprintf(p, sizeof p, "%s/v2.pxcam", work);
	write_file(p, h, 32);
	open_show("version 2", p);
	h[8] = 1;
	put_le32(h + 16, 100);
	snprintf(p, sizeof p, "%s/stride.pxcam", work);
	write_file(p, h, 32);
	open_show("stride 100", p);
	put_le32(h + 16, 2560);
	h[10] = 48;
	snprintf(p, sizeof p, "%s/hdr48.pxcam", work);
	write_file(p, h, 48);
	open_show("header 48", p);
	h[10] = 32;
	/* records: bad magic, bad lengths, truncated header, truncated payload, a short zlib stream */
	uint8_t f[96];
	memcpy(f, h, 32);
	memset(f + 32, 0, 64);
	memcpy(f + 32, "JUNK", 4);
	snprintf(p, sizeof p, "%s/badmagic.pxcam", work);
	write_file(p, f, 64);
	open_show("bad record magic", p);
	put_le32(f + 32, 0x4d415246);
	put_le32(f + 40, 10);
	put_le32(f + 44, 999);
	snprintf(p, sizeof p, "%s/badlen.pxcam", work);
	write_file(p, f, 64);
	open_show("bad record lengths", p);
	snprintf(p, sizeof p, "%s/trunchdr.pxcam", work);
	write_file(p, f, 50);
	open_show("truncated record header", p);
	put_le32(f + 44, 2560 * 832);
	put_le32(f + 40, 2560 * 832);
	snprintf(p, sizeof p, "%s/truncpay.pxcam", work);
	write_file(p, f, 96);
	open_show("truncated payload", p);
	put_le32(f + 36, 1);
	put_le32(f + 40, 20);
	/* zlib stream of 16 zero bytes: decompresses to fewer bytes than a frame */
	static const uint8_t z16[] = {0x78, 0x9c, 0x63, 0x60, 0x40, 0x05, 0x00, 0x00, 0x10, 0x00, 0x01};
	put_le32(f + 40, sizeof z16);
	memset(f + 64, 0, 32);
	memcpy(f + 64, z16, sizeof z16);
	snprintf(p, sizeof p, "%s/zshort.pxcam", work);
	write_file(p, f, 64 + sizeof z16);
	open_show("zlib short", p);
	/* a corrupt stream: both refuse it, the detail after "zlib: " is the decoder's own (miniz_oxide's Debug of its
	   error with the partial output vs zlib's message) and is not compared */
	f[64 + 9] ^= 0x55;
	snprintf(p, sizeof p, "%s/zbad.pxcam", work);
	write_file(p, f, 64 + sizeof z16);
	g_mask_zlib = 1;
	open_show("zlib corrupt", p);
	g_mask_zlib = 0;
	/* describe into small buffers */
	snprintf(p, sizeof p, "%s/empty.pxcam", work);
	pimax_cam *c = pimax_cam_open(p, NULL);
	if (c) {
		size_t sizes[] = {0, 1, 5, 64};
		for (size_t i = 0; i < 4; i++) {
			memset(buf, '#', sizeof buf);
			size_t n = pimax_cam_describe(c, sizes[i] ? buf : NULL, sizes[i]);
			buf[63] = 0;
			printf("describe len %zu -> %zu \"%s\"\n", sizes[i], n, sizes[i] > 1 ? buf : "");
		}
		memset(buf, '#', sizeof buf);
		printf("describe buf len 0 -> %zu first '%c'\n", pimax_cam_describe(c, buf, 0), buf[0]);
		{
			long long rc_ = (long long)(pimax_cam_copy_tile(c, 0, g_tile, 640));
			printf("copy before any frame %lld %s\n", rc_, pimax_cam_last_error());
		}
		pimax_cam_close(c);
	}
	return 0;
}

/* ---- broker client ------------------------------------------------------------------------------------------ */

struct ref_frame {
	uint32_t counter;
	uint64_t host_rel;
	char line[640];
};

/* the frame line without the time fields (they are compared separately) */
static void ref_line(char *out, size_t cap, pimax_cam *c, const pimax_cam_frame_info *fi)
{
	uint64_t th[4];
	for (uint32_t t = 0; t < 4; t++) {
		th[t] = pimax_cam_copy_tile(c, t, g_tile, 640) == 0 ? fnv(g_tile, 640 * 480, FNV0) : 0;
	}
	snprintf(out, cap,
	         "dev %" PRIu64 " ctr %u seq %u kind %u type %#x exp %u %u %u %u gain %u %u %u %u rows %u %u %u %u flags %#x "
	         "tiles %016" PRIx64 " %016" PRIx64 " %016" PRIx64 " %016" PRIx64,
	         fi->dev_ts_us, fi->counter, fi->sequence, fi->kind, fi->frame_type, fi->exposure[0], fi->exposure[1],
	         fi->exposure[2], fi->exposure[3], fi->gain[0], fi->gain[1], fi->gain[2], fi->gain[3], fi->y_offset[0],
	         fi->y_offset[1], fi->y_offset[2], fi->y_offset[3], fi->flags, th[0], th[1], th[2], th[3]);
}

static int cmd_client(const char *sock, const char *ref, long slow_ms)
{
	/* reference: a direct replay of the file the broker serves */
	pimax_cam *r = pimax_cam_open(ref, NULL);
	if (!r) {
		printf("ref open error: %s\n", pimax_cam_last_error());
		return 1;
	}
	size_t cap = 4096, nref = 0;
	struct ref_frame *refs = malloc(cap * sizeof *refs);
	pimax_cam_frame_info fi;
	uint64_t h0 = 0;
	while (pimax_cam_next(r, 0, &fi) == 1) {
		if (nref == cap) {
			cap *= 2;
			refs = realloc(refs, cap * sizeof *refs);
		}
		if (nref == 0) {
			h0 = fi.host_ts_ns;
		}
		refs[nref].counter = fi.counter;
		refs[nref].host_rel = fi.host_ts_ns - h0;
		ref_line(refs[nref].line, sizeof refs[nref].line, r, &fi);
		nref++;
	}
	pimax_cam_close(r);

	char spec[1200];
	snprintf(spec, sizeof spec, "broker:%s", sock);
	pimax_cam *c = NULL;
	for (int i = 0; i < 300 && !c; i++) {
		c = pimax_cam_open(spec, NULL);
		if (!c) {
			struct timespec ts = {0, 10 * 1000 * 1000};
			nanosleep(&ts, NULL);
		}
	}
	if (!c) {
		printf("connect error: %s\n", pimax_cam_last_error());
		return 1;
	}
	char desc[512];
	pimax_cam_describe(c, desc, sizeof desc);
	printf("describe \"%s\"\n", desc);
	size_t got = 0, mismatch = 0, order = 0, timing = 0, pos = 0, first_ref = 0, gaps = 0;
	uint64_t host_first = 0;
	int rc;
	char line[640];
	uint32_t replay_flags = 0;
	while ((rc = pimax_cam_next(c, 3000, &fi)) == 1) {
		replay_flags |= fi.flags;
		ref_line(line, sizeof line, c, &fi);
		/* find the frame by counter at or after the last position (frames come in order) */
		size_t k = pos;
		while (k < nref && refs[k].counter != fi.counter) {
			k++;
		}
		if (k == nref) {
			order++;
			continue;
		}
		if (got == 0) {
			host_first = fi.host_ts_ns;
			first_ref = k;
		} else if (k != pos) {
			gaps += k - pos;
		}
		if (strcmp(line, refs[k].line) != 0) {
			if (mismatch < 3) {
				printf("mismatch at ctr %u:\n  broker %s\n  replay %s\n", fi.counter, line, refs[k].line);
			}
			mismatch++;
		}
		/* the broker passes its host times through: spacing as recorded (the server's replay may add 1 ns to keep
		   them strictly increasing) */
		uint64_t want = refs[k].host_rel - refs[first_ref].host_rel;
		uint64_t have = fi.host_ts_ns - host_first;
		if (have + 1000 < want || have > want + 1000) {
			timing++;
		}
		pos = k + 1;
		got++;
		if (slow_ms > 0) {
			struct timespec ts = {0, slow_ms * 1000 * 1000};
			nanosleep(&ts, NULL);
		}
	}
	const char *e = pimax_cam_last_error();
	/* the error text names the socket path, which differs between runs: print its shape only */
	printf("end rc %d: %s\n", rc, strstr(e, "stopped") ? "camera broker <socket> stopped" : e);
	pimax_cam_stats st;
	pimax_cam_get_stats(c, &st);
	pimax_cam_close(c);
	free(refs);
	int ok = mismatch == 0 && order == 0 && timing == 0 && (replay_flags & PIMAX_CAM_FLAG_REPLAY) != 0;
	if (slow_ms > 0) {
		/* a slow client skips frames (but in order, never blocking the server) */
		ok = ok && got < nref / 2 && st.dropped > 0;
		printf("slow client: frames %s, in order %s, identical %s, skipped some %s\n", got > 0 ? "yes" : "no",
		       order == 0 ? "yes" : "no", mismatch == 0 ? "yes" : "no", st.dropped > 0 ? "yes" : "no");
	} else {
		/* the frames published before the client connected are gone; every later one must arrive */
		ok = ok && gaps == 0 && got * 2 >= nref && first_ref + got == nref;
		printf("client: every frame after connecting %s, in order %s, identical %s, spacing as recorded %s\n",
		       gaps == 0 && first_ref + got == nref ? "yes" : "no", order == 0 ? "yes" : "no",
		       mismatch == 0 ? "yes" : "no", timing == 0 ? "yes" : "no");
	}
	fprintf(stderr,
	        "client: %zu of %zu frames (first at %zu), %zu missed later, %zu mismatches, %zu out of order, %zu timing\n",
	        got, nref, first_ref, gaps, mismatch, order, timing);
	return ok ? 0 : 1;
}

/* ---- raw hello ---------------------------------------------------------------------------------------------- */

static int cmd_hello(const char *path)
{
	struct sockaddr_un a;
	memset(&a, 0, sizeof a);
	a.sun_family = AF_UNIX;
	snprintf(a.sun_path, sizeof a.sun_path, "%s", path);
	int s = -1;
	for (int i = 0; i < 300; i++) {
		s = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
		if (connect(s, (struct sockaddr *)&a, sizeof a) == 0) {
			break;
		}
		close(s);
		s = -1;
		struct timespec ts = {0, 10 * 1000 * 1000};
		nanosleep(&ts, NULL);
	}
	if (s < 0) {
		printf("connect failed\n");
		return 1;
	}
	uint8_t b[512];
	union {
		struct cmsghdr h;
		char c[64];
	} cb;
	struct iovec iov = {b, sizeof b};
	struct msghdr m;
	memset(&m, 0, sizeof m);
	m.msg_iov = &iov;
	m.msg_iovlen = 1;
	m.msg_control = cb.c;
	m.msg_controllen = sizeof cb.c;
	ssize_t n = recvmsg(s, &m, MSG_CMSG_CLOEXEC);
	struct cmsghdr *cm = CMSG_FIRSTHDR(&m);
	int fd = -1;
	if (cm && cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) {
		memcpy(&fd, CMSG_DATA(cm), sizeof fd);
	}
	printf("hello %zd bytes, fd %s, controllen %zu, flags %#x\n", n, fd >= 0 ? "yes" : "no", (size_t)m.msg_controllen,
	       m.msg_flags);
	/* the fixed part, then the description */
	printf("fixed");
	for (ssize_t i = 0; i < 28 && i < n; i++) {
		printf(" %02x", b[i]);
	}
	printf("\ndesc \"%.*s\"\n", n > 28 ? (int)(n - 28) : 0, (const char *)b + 28);
	if (fd < 0) {
		return 1;
	}
	struct stat st;
	fstat(fd, &st);
	uint32_t slots, slot_bytes;
	memcpy(&slots, b + 8, 4);
	memcpy(&slot_bytes, b + 12, 4);
	printf("ring size %lld = 4096 + %u x %u: %s\n", (long long)st.st_size, slots, slot_bytes,
	       (uint64_t)st.st_size == 4096ull + (uint64_t)slots * slot_bytes ? "yes" : "no");
	char name[256] = "";
	char link[64];
	snprintf(link, sizeof link, "/proc/self/fd/%d", fd);
	ssize_t ln = readlink(link, name, sizeof name - 1);
	if (ln > 0) {
		name[ln] = 0;
	}
	printf("memfd %s\n", name);
	/* a read-only mapping works; a writable one is refused only if the server sealed it (it does not) */
	uint8_t *p = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED) {
		printf("mmap failed\n");
		return 1;
	}
	printf("ring header");
	for (int i = 0; i < 8; i++) {
		printf(" %02x", p[i]);
	}
	printf(" zero 8..64 %s\n", memchr(p + 8, 1, 56) == NULL && memchr(p + 8, 0xff, 56) == NULL ? "yes" : "no");
	/* three notices: k is u64 LE, the slot k % slots carries lock 2k+2 and its own index at 32 */
	for (int i = 0; i < 3; i++) {
		struct pollfd pf = {s, POLLIN, 0};
		if (poll(&pf, 1, 3000) != 1) {
			printf("no notice\n");
			break;
		}
		uint8_t nb[16];
		ssize_t k = recv(s, nb, sizeof nb, 0);
		uint64_t idx;
		memcpy(&idx, nb, 8);
		const uint8_t *slot = p + 4096 + (idx % slots) * slot_bytes;
		uint64_t lock, kk;
		uint32_t len, stride, seq;
		memcpy(&lock, slot, 8);
		memcpy(&seq, slot + 16, 4);
		memcpy(&len, slot + 20, 4);
		memcpy(&stride, slot + 24, 4);
		memcpy(&kk, slot + 32, 8);
		int lock_ok = lock == 2 * idx + 2 || lock > 2 * idx + 2; /* may already be overwritten */
		printf("notice %zd bytes: lock ok %s, k at 32 %s, len %u stride %u, pad 28..32 %02x%02x%02x%02x\n", k,
		       lock_ok ? "yes" : "no", kk == idx || lock != 2 * idx + 2 ? "yes" : "no", len, stride, slot[28],
		       slot[29], slot[30], slot[31]);
		(void)seq;
	}
	uint64_t state;
	memcpy(&state, p + 88, 8);
	printf("camera state %llu\n", (unsigned long long)state);
	close(s);
	return 0;
}

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IOFBF, 1 << 16);
	g_tile = malloc(640 * 480);
	if (argc < 2) {
		fprintf(stderr, "usage: see the header of cam_difftest.c\n");
		return 2;
	}
	const char *cmd = argv[1];
	int rc = 2;
	if (strcmp(cmd, "layout") == 0) {
		rc = cmd_layout();
	} else if (strcmp(cmd, "calib") == 0) {
		rc = cmd_calib(argc - 2, argv + 2);
	} else if (strcmp(cmd, "load") == 0 && argc == 4) {
		rc = cmd_load(argv[2], argv[3]);
	} else if (strcmp(cmd, "clock") == 0) {
		rc = cmd_clock();
	} else if (strcmp(cmd, "replay") == 0 && argc >= 3) {
		rc = cmd_replay(argv[2], argc > 3 ? atol(argv[3]) : 0);
	} else if (strcmp(cmd, "record") == 0 && argc == 5) {
		rc = cmd_record(argv[2], argv[3], atol(argv[4]));
	} else if (strcmp(cmd, "errors") == 0 && argc == 3) {
		rc = cmd_errors(argv[2]);
	} else if (strcmp(cmd, "client") == 0 && argc >= 4) {
		rc = cmd_client(argv[2], argv[3], argc > 4 ? atol(argv[4]) : 0);
	} else if (strcmp(cmd, "extract") == 0 && argc == 5) {
		FILE *in = fopen(argv[2], "rb");
		FILE *o = fopen(argv[4], "wb");
		long want = atol(argv[3]);
		uint8_t h[32];
		uint8_t *buf = malloc(4 << 20);
		rc = in && o && fread(h, 1, 32, in) == 32 ? 0 : 1;
		for (long k = 0; rc == 0 && k < want && fread(h, 1, 32, in) == 32; k++) {
			uint32_t stored;
			memcpy(&stored, h + 8, 4);
			if (stored > (4u << 20) || fread(buf, 1, stored, in) != stored) {
				break;
			}
			fwrite(buf, 1, stored, o);
		}
		free(buf);
		if (in) {
			fclose(in);
		}
		if (o) {
			fclose(o);
		}
	} else if (strcmp(cmd, "pxcam") == 0 && argc == 3) {
		dump_pxcam(argv[2], 1);
		rc = 0;
	} else if (strcmp(cmd, "hello") == 0 && argc == 3) {
		rc = cmd_hello(argv[2]);
	} else {
		fprintf(stderr, "bad arguments\n");
	}
	fflush(stdout);
	free(g_tile);
	return rc;
}
