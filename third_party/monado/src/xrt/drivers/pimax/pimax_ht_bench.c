// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Offline Mercury bench on a .pxcam recording: no headset, no IPC, no prober. Feeds the tracking-exposure
 *         stereo pairs (the same tiles and calibration the driver uses) to Mercury's synchronous tracker and
 *         measures wall time, CPU time and the detection rate per pair.
 *
 * Usage: pimax_ht_bench SRC.pxcam|EUROC/mav0 [--cams A,B] [--orient auto|O0,O1] [--skip N] [--max N] [--every K]
 *                       [--calib XML] [--tsv OUT.tsv] [--nice N] [--dump DIR [--dump-every K]]
 *                       [--kind tracking|led|any]
 *
 * --kind picks the .pxcam frames fed to Mercury: tracking (default, what the driver feeds), led (the short
 * controller-LED exposure, for comparison only) or any (both, alternating as the camera delivers them).
 *
 * Example: nice -n 19 pimax_ht_bench ~/ctrl-test.pxcam --tsv /tmp/ht.tsv
 *
 * @ingroup drv_pimax
 */

#include "pimax_ht_calib.h"

#include "tracking/t_hand_tracking.h"
#include "hg_interface.h"
#include "util/u_frame.h"
#include "util/u_logging.h"

#include "pimax_cam.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>


int
pimax_ht_bench_load_grey(const char *path, uint8_t *dst, uint32_t w, uint32_t h, uint32_t stride);

//! EuRoC input: frames present in both camera CSVs (same timestamps, the recorder writes them together).
struct euroc
{
	char dir[1024];
	const char *camdir[2];
	uint64_t *ts;
	size_t count, next;
};

static bool
euroc_open(struct euroc *e, const char *dir, const unsigned cams[2])
{
	snprintf(e->dir, sizeof(e->dir), "%s", dir);
	// Recorder cam0/cam1 = PIMAX_SLAM_CAMS order (default 0,1). --cams 1,0 swaps the views.
	if (cams[0] > 1 || cams[1] > 1 || cams[0] == cams[1]) {
		fprintf(stderr, "EuRoC input holds factory cameras 0 and 1 only (--cams 0,1 or 1,0)\n");
		return false;
	}
	e->camdir[0] = cams[0] == 0 ? "cam0" : "cam1";
	e->camdir[1] = cams[1] == 0 ? "cam0" : "cam1";
	char p[1200];
	snprintf(p, sizeof(p), "%s/cam0/data.csv", dir);
	FILE *f = fopen(p, "r");
	if (f == NULL) {
		fprintf(stderr, "%s: cannot open (want a .../mav0 directory)\n", p);
		return false;
	}
	size_t cap = 4096;
	e->ts = calloc(cap, sizeof(uint64_t));
	char line[256];
	while (fgets(line, sizeof(line), f) != NULL) {
		uint64_t t;
		if (line[0] == '#' || sscanf(line, "%" SCNu64 ",", &t) != 1) {
			continue;
		}
		if (e->count == cap) {
			cap *= 2;
			e->ts = realloc(e->ts, cap * sizeof(uint64_t));
		}
		e->ts[e->count++] = t;
	}
	fclose(f);
	return e->count > 0;
}

static double
now_s(clockid_t c)
{
	struct timespec ts;
	clock_gettime(c, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int
cmp_d(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return (x > y) - (x < y);
}

static double
pct(double *v, size_t n, double p)
{
	if (n == 0) {
		return NAN;
	}
	size_t i = (size_t)(p * (double)(n - 1) + 0.5);
	return v[i];
}

static double
tile_mean(const struct xrt_frame *xf)
{
	uint64_t s = 0;
	for (uint32_t y = 0; y < xf->height; y += 4) {
		const uint8_t *row = xf->data + (size_t)y * xf->stride;
		for (uint32_t x = 0; x < xf->width; x += 4) {
			s += row[x];
		}
	}
	return (double)s / (double)((xf->height / 4) * (xf->width / 4));
}

int
main(int argc, char **argv)
{
	if (argc < 2 || argv[1][0] == '-') {
		fprintf(stderr,
		        "usage: %s SRC.pxcam [--cams A,B] [--orient auto|O0,O1] [--skip N] [--max N] [--every K] "
		        "[--calib XML] [--tsv OUT] [--nice N] [--dump DIR [--dump-every K]] [--kind tracking|led|any]\n",
		        argv[0]);
		return 2;
	}
	const char *src = argv[1];
	unsigned cams[2] = {0, 1};
	int orient[2] = {-1, -1};
	long skip = 0, max = -1, every = 1;
	int nice_v = 0;
	const char *calib_path = NULL, *tsv_path = NULL, *dump_dir = NULL;
	long dump_every = 25;
	int want_kind = PIMAX_CAM_KIND_TRACKING; // -1 = any
	for (int i = 2; i < argc; i++) {
		const char *a = argv[i];
		const char *v = i + 1 < argc ? argv[i + 1] : NULL;
		if (v == NULL) {
			fprintf(stderr, "%s needs a value\n", a);
			return 2;
		}
		if (strcmp(a, "--cams") == 0) {
			sscanf(v, "%u,%u", &cams[0], &cams[1]);
		} else if (strcmp(a, "--orient") == 0) {
			if (strcmp(v, "auto") != 0) {
				sscanf(v, "%d,%d", &orient[0], &orient[1]);
			}
		} else if (strcmp(a, "--skip") == 0) {
			skip = atol(v);
		} else if (strcmp(a, "--max") == 0) {
			max = atol(v);
		} else if (strcmp(a, "--every") == 0) {
			every = atol(v) < 1 ? 1 : atol(v);
		} else if (strcmp(a, "--calib") == 0) {
			calib_path = v;
		} else if (strcmp(a, "--tsv") == 0) {
			tsv_path = v;
		} else if (strcmp(a, "--dump") == 0) {
			dump_dir = v;
		} else if (strcmp(a, "--dump-every") == 0) {
			dump_every = atol(v) < 1 ? 1 : atol(v);
		} else if (strcmp(a, "--kind") == 0) {
			if (strcmp(v, "tracking") == 0) {
				want_kind = PIMAX_CAM_KIND_TRACKING;
			} else if (strcmp(v, "led") == 0) {
				want_kind = PIMAX_CAM_KIND_LED;
			} else if (strcmp(v, "any") == 0) {
				want_kind = -1;
			} else {
				fprintf(stderr, "--kind: tracking, led or any\n");
				return 2;
			}
		} else if (strcmp(a, "--nice") == 0) {
			nice_v = atoi(v);
		} else {
			fprintf(stderr, "unknown option %s\n", a);
			return 2;
		}
		i++;
	}
	if (nice_v != 0) {
		// Before Mercury creates its worker pool, so the pool threads inherit it.
		setpriority(PRIO_PROCESS, 0, nice_v);
	}

	pimax_cam_calib cal;
	char used[512] = {0};
	if (pimax_cam_calib_load(calib_path, &cal, used, sizeof(used)) != 0) {
		fprintf(stderr, "calibration: %s\n", pimax_cam_last_error());
		return 1;
	}
	double baseline = 0;
	struct t_stereo_camera_calibration *stereo = pimax_ht_make_stereo(&cal, cams[0], cams[1], &baseline);
	if (stereo == NULL) {
		fprintf(stderr, "stereo %u-%u: %s\n", cams[0], cams[1], pimax_cam_last_error());
		return 1;
	}
	struct t_camera_extra_info extra = {0};
	for (int v = 0; v < 2; v++) {
		extra.views[v].boundary_type = HT_IMAGE_BOUNDARY_NONE;
		extra.views[v].camera_orientation =
		    orient[v] >= 0 ? (enum t_camera_orientation)orient[v] : pimax_ht_orientation_of(&cal, cams[v], NULL);
	}
	char models[1024], why[1400];
	if (!pimax_ht_models_check(models, sizeof(models), why, sizeof(why))) {
		fprintf(stderr, "models: %s\n", why);
		return 1;
	}
	printf("# calibration %s; cams %u (%s) + %u (%s), baseline %.1f mm, orientation %d,%d; models %s\n", used,
	       cams[0], cal.cameras[cams[0]].name, cams[1], cal.cameras[cams[1]].name, baseline * 1000.0,
	       (int)extra.views[0].camera_orientation, (int)extra.views[1].camera_orientation, models);

	double t_create = now_s(CLOCK_MONOTONIC);
	struct t_hand_tracking_create_info ci = {.cams_info = extra, .masks_sink = NULL};
	struct t_hand_tracking_sync *sync = t_hand_tracking_sync_mercury_create(stereo, ci, models);
	if (sync == NULL) {
		fprintf(stderr, "Mercury create failed\n");
		return 1;
	}
	t_create = now_s(CLOCK_MONOTONIC) - t_create;

	struct euroc eu = {0};
	pimax_cam *cam = NULL;
	struct stat sst;
	if (stat(src, &sst) == 0 && S_ISDIR(sst.st_mode)) {
		// EuRoC recording (PIMAX_SLAM_RECORD): .../mav0 with cam0/cam1 = PIMAX_SLAM_CAMS (default factory 0,1).
		if (!euroc_open(&eu, src, cams)) {
			return 1;
		}
	} else {
		pimax_cam_open_params params = {0}; // replay: no loop, no real-time pacing (as fast as Mercury goes)
		cam = pimax_cam_open(src, &params);
		if (cam == NULL) {
			fprintf(stderr, "source: %s\n", pimax_cam_last_error());
			return 1;
		}
	}

	FILE *tsv = tsv_path != NULL ? fopen(tsv_path, "w") : NULL;
	if (tsv != NULL) {
		fprintf(tsv, "idx\tcounter\tdev_us\twall_ms\tcpu_ms\tmean0\tmean1\tleft\tright\tlx\tly\tlz\trx\try\trz\n");
	}

	size_t cap = 1 << 16, n = 0;
	double *wall = calloc(cap, sizeof(double)), *cpu = calloc(cap, sizeof(double));
	double *dist = calloc(cap * 2, sizeof(double));
	size_t ndist = 0;
	uint64_t act[2] = {0, 0}, any = 0, both = 0, tracking_idx = 0, dark = 0;
	double cpu_total0 = now_s(CLOCK_PROCESS_CPUTIME_ID), wall_total0 = now_s(CLOCK_MONOTONIC);
	double cpu_in_mercury = 0;

	while (max < 0 || (long)n < max) {
		pimax_cam_frame_info fi = {0};
		if (cam != NULL) {
			int r = pimax_cam_next(cam, 1000, &fi);
			if (r <= 0) {
				break;
			}
			if (want_kind >= 0 && (int)fi.kind != want_kind) {
				continue;
			}
		} else if (eu.next >= eu.count) {
			break;
		}
		uint64_t idx = tracking_idx++;
		if ((long)idx < skip || (idx - (uint64_t)skip) % (uint64_t)every != 0) {
			if (cam == NULL) {
				eu.next++;
			}
			continue;
		}
		struct xrt_frame *xf[2] = {NULL, NULL};
		bool ok = true;
		for (int v = 0; v < 2; v++) {
			u_frame_create_one_off(XRT_FORMAT_L8, PIMAX_CAM_TILE_W, PIMAX_CAM_TILE_H, &xf[v]);
			if (cam != NULL) {
				pimax_cam_copy_tile(cam, cams[v], xf[v]->data, (uint32_t)xf[v]->stride);
			} else {
				char path[1400];
				snprintf(path, sizeof(path), "%s/%s/data/%" PRIu64 ".png", eu.dir, eu.camdir[v],
				         eu.ts[eu.next]);
				ok = ok && pimax_ht_bench_load_grey(path, xf[v]->data, xf[v]->width, xf[v]->height,
				                                    (uint32_t)xf[v]->stride) == 0;
				fi.mono_ts_ns = eu.ts[eu.next];
				fi.dev_ts_us = eu.ts[eu.next] / 1000;
				fi.counter = (uint32_t)eu.next;
			}
			xf[v]->timestamp = (int64_t)fi.mono_ts_ns;
			xf[v]->source_timestamp = (int64_t)fi.dev_ts_us * 1000;
			xf[v]->source_sequence = fi.counter;
			xf[v]->source_id = cams[v];
		}
		if (cam == NULL) {
			eu.next++;
		}
		if (!ok) {
			xrt_frame_reference(&xf[0], NULL);
			xrt_frame_reference(&xf[1], NULL);
			continue;
		}
		double m0 = tile_mean(xf[0]), m1 = tile_mean(xf[1]);
		if (m0 < 15 && m1 < 15) {
			dark++;
		}
		struct xrt_hand_joint_set hands[2];
		int64_t ts = 0;
		double w0 = now_s(CLOCK_MONOTONIC), c0 = now_s(CLOCK_PROCESS_CPUTIME_ID);
		t_ht_sync_process(sync, xf[0], xf[1], &hands[0], &hands[1], &ts);
		double w1 = now_s(CLOCK_MONOTONIC), c1 = now_s(CLOCK_PROCESS_CPUTIME_ID);
		if (dump_dir != NULL && n % (size_t)dump_every == 0) {
			// Side-by-side PGM of the pair as Mercury got it (view 0 | view 1), detection in the name.
			char path[1400];
			snprintf(path, sizeof(path), "%s/pair%05" PRIu64 "_L%dR%d.pgm", dump_dir, idx, hands[0].is_active,
			         hands[1].is_active);
			FILE *f = fopen(path, "wb");
			if (f != NULL) {
				fprintf(f, "P5\n%u %u\n255\n", xf[0]->width * 2, xf[0]->height);
				for (uint32_t y = 0; y < xf[0]->height; y++) {
					fwrite(xf[0]->data + (size_t)y * xf[0]->stride, 1, xf[0]->width, f);
					fwrite(xf[1]->data + (size_t)y * xf[1]->stride, 1, xf[1]->width, f);
				}
				fclose(f);
			}
		}
		xrt_frame_reference(&xf[0], NULL);
		xrt_frame_reference(&xf[1], NULL);
		if (n == cap) {
			break;
		}
		wall[n] = (w1 - w0) * 1000.0;
		cpu[n] = (c1 - c0) * 1000.0;
		cpu_in_mercury += c1 - c0;
		struct xrt_vec3 p[2] = {{0, 0, 0}, {0, 0, 0}};
		for (int h = 0; h < 2; h++) {
			if (hands[h].is_active) {
				act[h]++;
				p[h] = hands[h].values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose.position;
				dist[ndist++] = sqrt(p[h].x * p[h].x + p[h].y * p[h].y + p[h].z * p[h].z);
			}
		}
		any += (hands[0].is_active || hands[1].is_active);
		both += (hands[0].is_active && hands[1].is_active);
		if (tsv != NULL) {
			fprintf(tsv, "%" PRIu64 "\t%u\t%" PRIu64 "\t%.2f\t%.2f\t%.1f\t%.1f\t%d\t%d\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
			        idx, fi.counter, fi.dev_ts_us, wall[n], cpu[n], m0, m1, hands[0].is_active,
			        hands[1].is_active, p[0].x, p[0].y, p[0].z, p[1].x, p[1].y, p[1].z);
		}
		n++;
	}
	double wall_total = now_s(CLOCK_MONOTONIC) - wall_total0;
	double cpu_total = now_s(CLOCK_PROCESS_CPUTIME_ID) - cpu_total0;
	if (tsv != NULL) {
		fclose(tsv);
	}

	double mean_w = 0, mean_c = 0;
	for (size_t i = 0; i < n; i++) {
		mean_w += wall[i];
		mean_c += cpu[i];
	}
	mean_w /= n ? (double)n : 1;
	mean_c /= n ? (double)n : 1;
	qsort(wall, n, sizeof(double), cmp_d);
	qsort(cpu, n, sizeof(double), cmp_d);
	qsort(dist, ndist, sizeof(double), cmp_d);
	printf("model load %.0f ms\n", t_create * 1000.0);
	printf("pairs %zu (%s frames seen %" PRIu64 ", skip %ld, every %ld), dark pairs %" PRIu64 "\n", n,
	       want_kind == PIMAX_CAM_KIND_TRACKING ? "tracking" : want_kind == PIMAX_CAM_KIND_LED ? "LED" : "all",
	       tracking_idx, skip, every, dark);
	printf("wall ms per pair: mean %.2f p50 %.2f p95 %.2f p99 %.2f max %.2f\n", mean_w, pct(wall, n, 0.5),
	       pct(wall, n, 0.95), pct(wall, n, 0.99), n ? wall[n - 1] : NAN);
	printf("cpu ms per pair (all threads): mean %.2f p50 %.2f p95 %.2f -> at 25 pairs/s %.1f %% of one core\n",
	       mean_c, pct(cpu, n, 0.5), pct(cpu, n, 0.95), mean_c * 25.0 / 10.0);
	printf("detection: left %.1f %%, right %.1f %%, any %.1f %%, both %.1f %%\n", 100.0 * act[0] / (n ? n : 1),
	       100.0 * act[1] / (n ? n : 1), 100.0 * any / (n ? n : 1), 100.0 * both / (n ? n : 1));
	printf("wrist distance from camera %u (m): p10 %.2f p50 %.2f p90 %.2f (n %zu)\n", cams[0], pct(dist, ndist, 0.1),
	       pct(dist, ndist, 0.5), pct(dist, ndist, 0.9), ndist);
	printf("process: %.1f s wall, %.1f s CPU (Mercury %.1f s, replay decode etc. %.1f s)\n", wall_total, cpu_total,
	       cpu_in_mercury, cpu_total - cpu_in_mercury);

	if (cam != NULL) {
		pimax_cam_close(cam);
	}
	free(eu.ts);
	t_ht_sync_destroy(&sync);
	t_stereo_camera_calibration_reference(&stereo, NULL);
	free(wall);
	free(cpu);
	free(dist);
	return 0;
}
