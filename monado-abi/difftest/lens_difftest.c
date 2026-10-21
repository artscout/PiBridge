// SPDX-License-Identifier: MIT
// Difftest driver for the pimax_lens_rs.h ABI (docs/research/rust-port-plan.md, M5). The same C program is compiled
// twice, against omarchyvr's libpimax_lens.a (the oracle) and against PiBridge's libpibridge_pimax.a; run_difftest.sh
// compares the two outputs byte for byte. Floats are printed as exact hex (%a); dense grids as FNV-1a hashes of the
// float bits per row plus every 16th point in full; the debug grid PNGs by their inflated scanlines (filter bytes
// and pixels: the deflate bytes of zlib and miniz_oxide differ, the pictures must not).
//
//   lens_difftest synth WORKDIR          built-in scenarios: reference profile, code defaults, every key, bad lines,
//                                        environment overrides, synthetic and corrupt LUT files, the debug grid
//   lens_difftest conf WORKDIR CONF N    one lens.conf (e.g. the Crystal's crystal.conf + its LUT): an N x N grid per
//                                        eye, masks, and the debug grid pictures at 1/16 size
#define _GNU_SOURCE
#include "pimax_lens_rs.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zlib.h>

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

static uint32_t crc32_png(const uint8_t *d, size_t n)
{
	uint32_t c = 0xffffffffu;
	for (size_t i = 0; i < n; i++) {
		c ^= d[i];
		for (int k = 0; k < 8; k++) {
			c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
		}
	}
	return c ^ 0xffffffffu;
}

static uint8_t *read_all(const char *path, size_t *len)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		return NULL;
	}
	size_t cap = 1 << 16, n = 0;
	uint8_t *b = malloc(cap + 1);
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

static void write_text(const char *path, const char *text)
{
	write_file(path, text, strlen(text));
}

static const char *const ENV_KEYS[] = {
    "PIMAX_LENS_CONF",  "PIMAX_LENS_OFF",     "PIMAX_LENS_DEBUG_GRID", "PIMAX_LENS_DEBUG_GRID_DIV",
    "PIMAX_CANT_DEG",   "PIMAX_NO_REFERENCE", "XDG_CONFIG_HOME",       "HOME",
};

static void clear_env(void)
{
	for (size_t i = 0; i < sizeof ENV_KEYS / sizeof ENV_KEYS[0]; i++) {
		unsetenv(ENV_KEYS[i]);
	}
}

static void log_cb(void *user, int level, const char *msg)
{
	printf("  log %s %d %s\n", (const char *)user, level, msg);
}

// ---------------------------------------------------------------------------------------------------------------
// Probing a handle
// ---------------------------------------------------------------------------------------------------------------

static void print_f(const float *v, int n)
{
	for (int i = 0; i < n; i++) {
		printf(" %a", (double)v[i]);
	}
}

static void probe_points(const pimax_lens_rs *l, uint32_t view)
{
	const float pts[] = {0.0f, 1.0f, 0.5f, -0.0f, 0.25f, 0.75f, 1e-30f, -1e-30f, 1.5f, -0.5f, 1e30f, -1e30f, 1e-45f,
	                     0.999999f, 0.000001f, INFINITY, -INFINITY, NAN};
	const int n = (int)(sizeof pts / sizeof pts[0]);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			float o[6] = {0};
			const int r = pimax_lens_rs_compute(l, view, pts[i], pts[j], o);
			printf("  pt %u %a %a: %d", view, (double)pts[i], (double)pts[j], r);
			print_f(o, 6);
			printf("\n");
		}
	}
}

static void probe_grid(const pimax_lens_rs *l, uint32_t view, int n, float lo, float hi)
{
	uint64_t all = FNV0;
	for (int j = 0; j < n; j++) {
		uint64_t h = FNV0;
		const float v = lo + (hi - lo) * (float)j / (float)(n - 1);
		for (int i = 0; i < n; i++) {
			const float u = lo + (hi - lo) * (float)i / (float)(n - 1);
			float o[6] = {0};
			const int r = pimax_lens_rs_compute(l, view, u, v, o);
			h = fnv(&r, sizeof r, h);
			h = fnv(o, sizeof o, h);
			if (j % 16 == 0 && i % 16 == 0) {
				printf("  g %u %d %d:", view, i, j);
				print_f(o, 6);
				printf("\n");
			}
		}
		printf("  row %u %d %016" PRIx64 "\n", view, j, h);
		all = fnv(&h, sizeof h, all);
	}
	printf("  grid %u %dx%d %016" PRIx64 "\n", view, n, n, all);
}

static void probe_mask(const pimax_lens_rs *l, uint32_t view)
{
	for (int kind = -1; kind <= 3; kind++) {
		uint32_t counts[2] = {0xdeadbeef, 0xdeadbeef};
		int r = pimax_lens_rs_visibility_mask(l, view, kind, NULL, 0, NULL, 0, counts);
		printf("  mask %u kind %d: %d counts %u %u\n", view, kind, r, counts[0], counts[1]);
		if (r != -2) {
			continue;
		}
		const uint32_t nv = counts[0], ni = counts[1];
		float *v = calloc((size_t)nv * 2 + 2, sizeof(float));
		uint32_t *idx = calloc((size_t)ni + 1, sizeof(uint32_t));
		// one too small each, then NULL counts
		r = pimax_lens_rs_visibility_mask(l, view, kind, v, nv - 1, idx, ni, counts);
		printf("  mask %u kind %d small v: %d\n", view, kind, r);
		r = pimax_lens_rs_visibility_mask(l, view, kind, v, nv, idx, ni - 1, counts);
		printf("  mask %u kind %d small i: %d\n", view, kind, r);
		r = pimax_lens_rs_visibility_mask(l, view, kind, v, nv, idx, ni, NULL);
		printf("  mask %u kind %d null counts: %d\n", view, kind, r);
		r = pimax_lens_rs_visibility_mask(l, view, kind, v, nv, idx, ni, counts);
		printf("  mask %u kind %d: %d counts %u %u\n   v", view, kind, r, counts[0], counts[1]);
		print_f(v, (int)nv * 2);
		printf("\n   i");
		for (uint32_t k = 0; k < ni; k++) {
			printf(" %u", idx[k]);
		}
		printf("\n");
		free(v);
		free(idx);
	}
}

// The PNG structure (chunk kinds, lengths and CRC validity), IHDR, and the inflated scanlines.
static void probe_png(const char *path)
{
	size_t n = 0;
	uint8_t *b = read_all(path, &n);
	if (!b) {
		printf("  png %s: missing\n", path);
		return;
	}
	printf("  png %s: signature %s\n", path, n >= 8 && memcmp(b, "\x89PNG\r\n\x1a\n", 8) == 0 ? "ok" : "BAD");
	size_t pos = 8, idat_len = 0;
	uint8_t *idat = malloc(n);
	uint32_t w = 0, h = 0;
	while (pos + 12 <= n) {
		const uint32_t len = (uint32_t)b[pos] << 24 | (uint32_t)b[pos + 1] << 16 | (uint32_t)b[pos + 2] << 8 |
		                     b[pos + 3];
		if (pos + 12 + len > n) {
			printf("  truncated\n");
			break;
		}
		const uint32_t crc = (uint32_t)b[pos + 8 + len] << 24 | (uint32_t)b[pos + 9 + len] << 16 |
		                     (uint32_t)b[pos + 10 + len] << 8 | b[pos + 11 + len];
		const int ok = crc == crc32_png(b + pos + 4, len + 4);
		if (memcmp(b + pos + 4, "IDAT", 4) == 0) {
			memcpy(idat + idat_len, b + pos + 8, len);
			idat_len += len;
			printf("  chunk IDAT crc %s\n", ok ? "ok" : "BAD");
		} else {
			printf("  chunk %.4s %u crc %s", (const char *)b + pos + 4, len, ok ? "ok" : "BAD");
			for (uint32_t k = 0; k < len; k++) {
				printf(" %02x", b[pos + 8 + k]);
			}
			printf("\n");
			if (memcmp(b + pos + 4, "IHDR", 4) == 0 && len >= 8) {
				w = (uint32_t)b[pos + 8] << 24 | (uint32_t)b[pos + 9] << 16 | (uint32_t)b[pos + 10] << 8 |
				    b[pos + 11];
				h = (uint32_t)b[pos + 12] << 24 | (uint32_t)b[pos + 13] << 16 | (uint32_t)b[pos + 14] << 8 |
				    b[pos + 15];
			}
		}
		pos += 12 + len;
	}
	const size_t want = ((size_t)w * 3 + 1) * h;
	uint8_t *raw = malloc(want + 16);
	uLongf got = (uLongf)(want + 16);
	const int zr = uncompress(raw, &got, idat, (uLong)idat_len);
	printf("  inflate %d: %lu bytes (want %zu) %016" PRIx64 "\n", zr, (unsigned long)got, want,
	       fnv(raw, (size_t)got, FNV0));
	// per scanline: filter byte and hash
	if (zr == Z_OK && got == want) {
		uint64_t filters = FNV0;
		for (uint32_t y = 0; y < h; y++) {
			filters = fnv(raw + (size_t)y * (w * 3 + 1), 1, filters);
		}
		printf("  filters %016" PRIx64 "\n", filters);
	}
	free(raw);
	free(idat);
	free(b);
}

static void probe_text(const char *path)
{
	size_t n = 0;
	uint8_t *b = read_all(path, &n);
	if (!b) {
		printf("  text %s: missing\n", path);
		return;
	}
	printf("  text %s: %zu bytes\n%s\n  end of text\n", path, n, (const char *)b);
	free(b);
}

static void run(const char *name, int grid_n, const char *grid_dir)
{
	printf("scenario %s\n", name);
	pimax_lens_rs *l = pimax_lens_rs_create(3840.0f / 3552.0f, log_cb, (void *)name);
	printf("  handle %s\n", l ? "ok" : "NULL");
	if (!l) {
		return;
	}
	printf("  cant %a\n", (double)pimax_lens_rs_get_cant_deg(l));
	for (uint32_t view = 0; view < 3; view++) {
		float f[4] = {0};
		const int r = pimax_lens_rs_get_fov(l, view, f);
		printf("  fov %u: %d", view, r);
		print_f(f, 4);
		printf(" mask %d\n", pimax_lens_rs_has_mask(l, view));
	}
	printf("  fov null out %d\n", pimax_lens_rs_get_fov(l, 0, NULL));
	{
		float o[6] = {0};
		printf("  compute view 2: %d, null out %d\n", pimax_lens_rs_compute(l, 2, 0.5f, 0.5f, o),
		       pimax_lens_rs_compute(l, 0, 0.5f, 0.5f, NULL));
	}
	for (uint32_t view = 0; view < 2; view++) {
		probe_grid(l, view, grid_n, -0.0625f, 1.0625f);
		probe_mask(l, view);
	}
	probe_mask(l, 2);
	if (grid_n <= 65) {
		probe_points(l, 0);
		probe_points(l, 1);
	}
	if (grid_dir) {
		char p[4096];
		snprintf(p, sizeof p, "%s/pimax_lens_grid_left.png", grid_dir);
		probe_png(p);
		snprintf(p, sizeof p, "%s/pimax_lens_grid_right.png", grid_dir);
		probe_png(p);
		snprintf(p, sizeof p, "%s/pimax_lens_summary.txt", grid_dir);
		probe_text(p);
	}
	pimax_lens_rs_destroy(l);
}

// ---------------------------------------------------------------------------------------------------------------
// Synthetic LUT files
// ---------------------------------------------------------------------------------------------------------------

static void put_u32(uint8_t *b, uint32_t v)
{
	b[0] = (uint8_t)v;
	b[1] = (uint8_t)(v >> 8);
	b[2] = (uint8_t)(v >> 16);
	b[3] = (uint8_t)(v >> 24);
}

static void put_f32(uint8_t *b, float f)
{
	uint32_t v;
	memcpy(&v, &f, 4);
	put_u32(b, v);
}

static void fix_crc(uint8_t *b, size_t len)
{
	put_u32(b + len - 4, crc32_png(b, len - 4));
}

// A LUT of grid size n: a smooth barrel-like map of the panel to ray tangents, different per eye and channel.
static uint8_t *make_lut(uint32_t n, size_t *len)
{
	const size_t header = 56 + 2 * 36 + 16;
	*len = header + 2 * 3 * (size_t)n * n * 8 + 4;
	uint8_t *b = calloc(*len, 1);
	memcpy(b, "PIMAXLUT", 8);
	put_u32(b + 8, 1);
	put_u32(b + 12, n);
	put_u32(b + 16, 3);
	put_u32(b + 20, 2);
	memcpy(b + 24, "SYNTHETIC-LUT", 13);
	for (int e = 0; e < 2; e++) {
		uint8_t *h = b + 56 + e * 36;
		put_u32(h, e == 0 ? 3 : 1);
		put_f32(h + 4, 3840.0f);
		put_f32(h + 8, 3552.0f);
		put_f32(h + 12, 1.0f + 0.01f * (float)e);
		put_f32(h + 16, 1.05f);
		put_f32(h + 20, 1.2f - 0.1f * (float)e);
		put_f32(h + 24, 1.1f + 0.1f * (float)e);
	}
	size_t off = header;
	for (int e = 0; e < 2; e++) {
		for (int c = 0; c < 3; c++) {
			for (uint32_t j = 0; j < n; j++) {
				for (uint32_t i = 0; i < n; i++) {
					const float x = ((float)i / (float)(n - 1) - 0.5f) * 2.0f;
					const float y = ((float)j / (float)(n - 1) - 0.5f) * 2.0f;
					const float r2 = x * x + y * y;
					const float k = (1.1f + 0.002f * (float)c - 0.003f * (float)e) * (1.0f + 0.18f * r2);
					put_f32(b + off, x * k + 0.01f * (float)e);
					put_f32(b + off + 4, -y * k * 0.97f);
					off += 8;
				}
			}
		}
	}
	fix_crc(b, *len);
	return b;
}

static void scenario_conf(const char *work, const char *name, const char *text, const char *grid)
{
	char conf[4096];
	snprintf(conf, sizeof conf, "%s/%s.conf", work, name);
	write_text(conf, text);
	clear_env();
	setenv("PIMAX_LENS_CONF", conf, 1);
	run(name, 33, grid);
}

static int cmd_synth(const char *work)
{
	char p[4096], home[4096];
	snprintf(home, sizeof home, "%s/home", work);
	mkdir(home, 0755);

	// no file at all: the shipped Dream Air reference; PIMAX_NO_REFERENCE; an explicit missing file
	clear_env();
	setenv("HOME", home, 1);
	run("reference", 65, NULL);
	clear_env();
	setenv("HOME", home, 1);
	setenv("PIMAX_NO_REFERENCE", "1", 1);
	run("no-reference", 33, NULL);
	clear_env();
	setenv("HOME", home, 1);
	setenv("PIMAX_NO_REFERENCE", "0", 1);
	run("no-reference-0", 9, NULL);
	clear_env();
	setenv("PIMAX_LENS_CONF", "/nonexistent/lens.conf", 1);
	run("missing", 33, NULL);
	clear_env();
	run("no-home", 9, NULL);
	clear_env();
	setenv("HOME", home, 1);
	setenv("PIMAX_LENS_CONF", "", 1);
	run("empty-conf-var", 9, NULL);
	// a directory as the file
	clear_env();
	setenv("PIMAX_LENS_CONF", work, 1);
	run("directory", 9, NULL);

	// every key
	scenario_conf(work, "all-keys",
	              "# every key\ncant_deg = 1.75\n[common]\nfov = 51 49.5 45 44\ncenter = 0.02, -0.01\nbase = 0.66\n"
	              "scale = 1.02\nk = 0.6 -0.7 0.55 0.02 -0.01\nk_r = 0.61 -0.72 0.58 0.01\nbase_b = 0.655\n"
	              "ca_r = 1.004\nca_b = 0.996\nshift_r = 0.002 -0.001\nshift_b = -0.003 0.0015\nfit_range = 1.3\n"
	              "blackout = 0.01 0.02 0.03 0.04\nmask = ellipse 1.05 0.95\nmask_feather = 0.04\n"
	              "[left]\ncenter = 0.03 0.01\n[right]\ncenter = -0.03 0.01\nmask = superellipse 1.1 1.0 3.5\n",
	              NULL);
	scenario_conf(work, "mask-rect", "center = 0 0\nmask = rect 0.9 0.8\nmask_feather = 0\n", NULL);
	scenario_conf(work, "mask-box", "mask = box 1.1\nmask_feather = 0.2\n", NULL);
	scenario_conf(work, "mask-circle", "mask = circle 0.95\n", NULL);
	scenario_conf(work, "mask-round", "mask = round 1.0 0.9 6\n[right]\nmask = none\n", NULL);
	scenario_conf(work, "blackout-only", "blackout = 0.15\nmask_feather = 0\n", NULL);
	scenario_conf(work, "hidden-axis", "blackout = 5\n", NULL);
	scenario_conf(work, "off-with-mask", "distortion = no\nmask = ellipse 1 1\n", NULL);
	scenario_conf(work, "folding", "k = 0 -3 0\nfit_range = 0.9\n", NULL);
	scenario_conf(work, "fov-edges", "fov = 0.5 89.5 0.5 89.5\nscale = 0.05\n[right]\nscale = 20\nbase = 10\n", NULL);
	// bad lines, odd numbers, CRLF, comments, invalid UTF-8, a NUL byte
	{
		static const char bad[] =
		    "[nonsense]\r\njust words\r\nfov = 1 2\nunknown = 3\nk = 1 2\nk_x = 1 2 3\nca_g = 0\n"
		    "cant_deg = 99\ncant_deg = x\ncant_deg = -20\nmask = blob 1\nmask = ellipse -1\nmask = superellipse 1 1 0.5\n"
		    "blackout = 1 2\nblackout = -1\nfit_range = -0.1\nmask_feather = -1\nlut =\nlut_margin = 11\n"
		    "lut_interp = spline\nlut_interp = LINEAR\nlut_interp = cubic\n"
		    "base = 1e999\nbase = nan\nscale = .5x\nscale = +3\nscale = 5.\nscale = 1e\nscale = 2e+\n"
		    "center = 1,2  \ncenter = 0.01 0.02 0.03 0.04 junk\nshift_g = 1e-50 -1e-46\n"
		    "base_r = 0.7 # comment ; more\n  ; whole\n# whole\nKEY_WITH_CAPS = 1\nFOV = 50 50 44 44\n"
		    "distortion = maybe\n[left] trailing\nscale = 1.1\n[both]\n[common]x\n"
		    "lut = \"none\"\nlut = 'off'\nmask = 0\n"
		    "fov = 50 50 44 \xff\xfe\nfov = 50 50 44 44\xe2\x80\x83\nk_g = 0.1\xc2\xa0 0.2 0.3\n=\n = 5\nscale\n"
		    "scale = 1\x00 trailing\nlast = line without newline";
		char conf[4096];
		snprintf(conf, sizeof conf, "%s/bad-lines.conf", work);
		write_file(conf, bad, sizeof bad - 1);
		clear_env();
		setenv("PIMAX_LENS_CONF", conf, 1);
		run("bad-lines", 9, NULL);
	}
	// environment overrides
	{
		static const char *const cants[] = {"2.95", "abc", " 50 ", "-20", "  ", "1,5", "\xc2\xa0" "3\xc2\xa0", "+7.25e0"};
		for (size_t i = 0; i < sizeof cants / sizeof cants[0]; i++) {
			snprintf(p, sizeof p, "%s/cant.conf", work);
			write_text(p, "cant_deg = 1\n");
			clear_env();
			setenv("PIMAX_LENS_CONF", p, 1);
			setenv("PIMAX_CANT_DEG", cants[i], 1);
			char nm[64];
			snprintf(nm, sizeof nm, "cant-%zu", i);
			run(nm, 9, NULL);
		}
		static const char *const offs[] = {"1", "yes", "0", "", "0x"};
		for (size_t i = 0; i < sizeof offs / sizeof offs[0]; i++) {
			clear_env();
			setenv("HOME", home, 1);
			setenv("PIMAX_LENS_OFF", offs[i], 1);
			char nm[64];
			snprintf(nm, sizeof nm, "off-%zu", i);
			run(nm, 9, NULL);
		}
	}
	// XDG_CONFIG_HOME and the user's own partial file on the reference
	{
		char xdg[4096];
		snprintf(xdg, sizeof xdg, "%s/xdg", work);
		mkdir(xdg, 0755);
		snprintf(p, sizeof p, "%s/xdg/pibridge", work);
		mkdir(p, 0755);
		snprintf(p, sizeof p, "%s/xdg/pibridge/lens.conf", work);
		write_text(p, "fov = 40 41 42 43\n");
		clear_env();
		setenv("HOME", home, 1);
		setenv("XDG_CONFIG_HOME", xdg, 1);
		run("xdg", 33, NULL);
		clear_env();
		setenv("HOME", home, 1);
		setenv("XDG_CONFIG_HOME", xdg, 1);
		setenv("PIMAX_NO_REFERENCE", "1", 1);
		run("xdg-no-reference", 9, NULL);
	}

	// LUT files: valid ones, every corruption, relative / ~ / shared paths
	size_t len = 0;
	uint8_t *lut = make_lut(9, &len);
	snprintf(p, sizeof p, "%s/lut9.bin", work);
	write_file(p, lut, len);
	snprintf(p, sizeof p, "%s/lut9.bin", home);
	write_file(p, lut, len);
	scenario_conf(work, "lut-relative", "cant_deg = 2\nfov = 40 40 40 40\ncenter = 0.3 0.3\nlut = lut9.bin\n", NULL);
	scenario_conf(work, "lut-bilinear-margin", "lut = \"lut9.bin\"\nlut_margin = 0.5\nlut_interp = bilinear\n"
	                                           "[right]\nlut = none\nmask = ellipse 1 1\n",
	              NULL);
	scenario_conf(work, "lut-mask", "lut = lut9.bin\nblackout = 0.1\nmask_feather = 0.05\n", NULL);
	scenario_conf(work, "lut-off", "distortion = off\nlut = lut9.bin\n", NULL);
	scenario_conf(work, "lut-dotted", "[left]\nlut = ./lut9.bin\n[right]\nlut = .//lut9.bin\n", NULL);
	{
		clear_env();
		snprintf(p, sizeof p, "%s/tilde.conf", work);
		write_text(p, "lut = ~/lut9.bin\nlut_margin = 10\n");
		setenv("PIMAX_LENS_CONF", p, 1);
		setenv("HOME", home, 1);
		run("lut-tilde", 33, NULL);
	}
	{
		snprintf(p, sizeof p, "%s/abs.conf", work);
		char text[4096];
		snprintf(text, sizeof text, "lut = %s/lut9.bin\n", work);
		write_text(p, text);
		clear_env();
		setenv("PIMAX_LENS_CONF", p, 1);
		run("lut-absolute", 9, NULL);
	}
	{
		size_t l2 = 0;
		uint8_t *big = make_lut(129, &l2);
		snprintf(p, sizeof p, "%s/lut129.bin", work);
		write_file(p, big, l2);
		free(big);
		scenario_conf(work, "lut-129", "lut = lut129.bin\n", NULL);
		size_t l3 = 0;
		uint8_t *two = make_lut(2, &l3);
		snprintf(p, sizeof p, "%s/lut2.bin", work);
		write_file(p, two, l3);
		free(two);
		scenario_conf(work, "lut-2", "lut = lut2.bin\n[right]\nlut_interp = bilinear\n", NULL);
	}
	{
		struct {
			const char *name;
			int what;
		} bad[] = {{"trunc", 0}, {"crc", 1},     {"magic", 2},  {"version", 3}, {"channels", 4}, {"orient", 5},
		           {"nan", 6},   {"fov0", 7},    {"fovinf", 8}, {"n1", 9},      {"short", 10},  {"empty", 11},
		           {"extra", 12}, {"eyes", 13}};
		for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
			uint8_t *b = malloc(len + 16);
			memcpy(b, lut, len);
			size_t n = len;
			switch (bad[i].what) {
			case 0:
				n = len - 1;
				break;
			case 1:
				b[200] ^= 0x40;
				break;
			case 2:
				b[0] = 'X';
				break;
			case 3:
				b[8] = 9;
				break;
			case 4:
				put_u32(b + 16, 4);
				fix_crc(b, n);
				break;
			case 5:
				put_u32(b + 56 + 36, 8);
				fix_crc(b, n);
				break;
			case 6:
				put_f32(b + 56 + 2 * 36 + 16 + 8 * 7, NAN);
				fix_crc(b, n);
				break;
			case 7:
				put_f32(b + 56 + 12, 0.0f);
				fix_crc(b, n);
				break;
			case 8:
				put_f32(b + 56 + 36 + 24, INFINITY);
				fix_crc(b, n);
				break;
			case 9:
				put_u32(b + 12, 1);
				fix_crc(b, n);
				break;
			case 10:
				n = 100;
				break;
			case 11:
				n = 0;
				break;
			case 12:
				memset(b + len, 0, 4);
				n = len + 4;
				fix_crc(b, n);
				break;
			case 13:
				put_u32(b + 20, 3);
				fix_crc(b, n);
				break;
			}
			char lp[4096];
			snprintf(lp, sizeof lp, "%s/bad-%s.bin", work, bad[i].name);
			write_file(lp, b, n);
			free(b);
			char text[4096], nm[64];
			snprintf(text, sizeof text, "lut = bad-%s.bin\nfov = 45 45 45 45\n", bad[i].name);
			snprintf(nm, sizeof nm, "lut-bad-%s", bad[i].name);
			scenario_conf(work, nm, text, NULL);
		}
		scenario_conf(work, "lut-missing", "lut = nope.bin\n", NULL);
		scenario_conf(work, "lut-dir", "lut = .\n", NULL);
		scenario_conf(work, "lut-one-bad", "[left]\nlut = lut9.bin\n[right]\nlut = bad-crc.bin\n", NULL);
	}
	free(lut);

	// the debug grid
	{
		static const char *const divs[] = {"32", "0", "65", "abc", " 40 ", "+48", NULL};
		for (size_t i = 0; i < 7; i++) {
			char gdir[4096], nm[64];
			snprintf(gdir, sizeof gdir, "%s/grid%zu/sub", work, i);
			clear_env();
			setenv("HOME", home, 1);
			setenv("PIMAX_LENS_DEBUG_GRID", gdir, 1);
			if (divs[i]) {
				setenv("PIMAX_LENS_DEBUG_GRID_DIV", divs[i], 1);
			}
			snprintf(nm, sizeof nm, "grid-%zu", i);
			// "0", "65", "abc" fall back to 4: 960x888 pictures (slow but fine)
			run(nm, 5, gdir);
		}
		char gdir[4096];
		snprintf(gdir, sizeof gdir, "%s/grid-mask", work);
		snprintf(p, sizeof p, "%s/gm.conf", work);
		write_text(p, "mask = ellipse 0.9 0.8\nblackout = 0.02 0 0.05 0\nfit_range = 1.4\n[right]\nk = 0 -3 0\n");
		clear_env();
		setenv("PIMAX_LENS_CONF", p, 1);
		setenv("PIMAX_LENS_DEBUG_GRID", gdir, 1);
		setenv("PIMAX_LENS_DEBUG_GRID_DIV", "20", 1);
		run("grid-mask", 5, gdir);
		snprintf(gdir, sizeof gdir, "%s/grid-lut", work);
		snprintf(p, sizeof p, "%s/gl.conf", work);
		write_text(p, "lut = lut9.bin\n[right]\nlut_interp = bilinear\n");
		clear_env();
		setenv("PIMAX_LENS_CONF", p, 1);
		setenv("PIMAX_LENS_DEBUG_GRID", gdir, 1);
		setenv("PIMAX_LENS_DEBUG_GRID_DIV", "24", 1);
		run("grid-lut", 5, gdir);
		snprintf(gdir, sizeof gdir, "%s/grid-off", work);
		clear_env();
		setenv("HOME", home, 1);
		setenv("PIMAX_LENS_OFF", "1", 1);
		setenv("PIMAX_LENS_DEBUG_GRID", gdir, 1);
		setenv("PIMAX_LENS_DEBUG_GRID_DIV", "30", 1);
		run("grid-off", 5, gdir);
		// unwritable places: a warning, the handle is still made
		clear_env();
		setenv("HOME", home, 1);
		setenv("PIMAX_LENS_DEBUG_GRID", "/proc/definitely/not/here", 1);
		setenv("PIMAX_LENS_DEBUG_GRID_DIV", "32", 1);
		run("grid-proc", 5, NULL);
		snprintf(p, sizeof p, "%s/a-file", work);
		write_text(p, "x");
		snprintf(gdir, sizeof gdir, "%s/a-file/sub", work);
		clear_env();
		setenv("HOME", home, 1);
		setenv("PIMAX_LENS_DEBUG_GRID", gdir, 1);
		setenv("PIMAX_LENS_DEBUG_GRID_DIV", "32", 1);
		run("grid-under-file", 5, NULL);
		clear_env();
		setenv("HOME", home, 1);
		setenv("PIMAX_LENS_DEBUG_GRID", "", 1);
		run("grid-empty", 5, NULL);
	}

	// NULL handles and the version
	printf("abi %u\n", pimax_lens_rs_abi_version());
	{
		float o[6] = {0};
		uint32_t counts[2] = {0};
		printf("null: fov %d cant %a compute %d mask %d vis %d\n", pimax_lens_rs_get_fov(NULL, 0, o),
		       (double)pimax_lens_rs_get_cant_deg(NULL), pimax_lens_rs_compute(NULL, 0, 0.5f, 0.5f, o),
		       pimax_lens_rs_has_mask(NULL, 0),
		       pimax_lens_rs_visibility_mask(NULL, 0, 0, NULL, 0, NULL, 0, counts));
		pimax_lens_rs_destroy(NULL);
		clear_env();
		pimax_lens_rs *l = pimax_lens_rs_create(1.0f, NULL, NULL);
		printf("no log callback: %s\n", l ? "ok" : "NULL");
		pimax_lens_rs_destroy(l);
		static const float aspects[] = {0.5f, 2.0f, 0.0f, -1.0f, NAN, INFINITY};
		for (size_t i = 0; i < sizeof aspects / sizeof aspects[0]; i++) {
			clear_env();
			setenv("HOME", home, 1);
			l = pimax_lens_rs_create(aspects[i], NULL, NULL);
			printf("aspect %a:\n", (double)aspects[i]);
			for (uint32_t view = 0; view < 2; view++) {
				const float pts[][2] = {{0.5f, 0.5f}, {0.0f, 0.0f}, {1.0f, 0.3f}, {0.2f, 0.9f}};
				for (size_t k = 0; k < 4; k++) {
					pimax_lens_rs_compute(l, view, pts[k][0], pts[k][1], o);
					printf(" ");
					print_f(o, 6);
					printf("\n");
				}
				probe_mask(l, view);
			}
			pimax_lens_rs_destroy(l);
		}
	}
	return 0;
}

static int cmd_conf(const char *work, const char *conf, int n)
{
	char home[4096], gdir[4096];
	snprintf(home, sizeof home, "%s/home", work);
	mkdir(home, 0755);
	clear_env();
	setenv("HOME", home, 1);
	setenv("PIMAX_LENS_CONF", conf, 1);
	run("conf", n, NULL);
	snprintf(gdir, sizeof gdir, "%s/grid", work);
	clear_env();
	setenv("HOME", home, 1);
	setenv("PIMAX_LENS_CONF", conf, 1);
	setenv("PIMAX_LENS_DEBUG_GRID", gdir, 1);
	setenv("PIMAX_LENS_DEBUG_GRID_DIV", "16", 1);
	run("conf-grid", 5, gdir);
	clear_env();
	setenv("HOME", home, 1);
	setenv("PIMAX_LENS_CONF", conf, 1);
	setenv("PIMAX_CANT_DEG", "0", 1);
	run("conf-cant0", 65, NULL);
	return 0;
}

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IOFBF, 1 << 20);
	if (argc >= 3 && strcmp(argv[1], "synth") == 0) {
		return cmd_synth(argv[2]);
	}
	if (argc >= 5 && strcmp(argv[1], "conf") == 0) {
		return cmd_conf(argv[2], argv[3], atoi(argv[4]));
	}
	fprintf(stderr, "usage: lens_difftest synth WORKDIR | conf WORKDIR CONF N\n");
	return 2;
}
