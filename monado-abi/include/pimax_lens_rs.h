/* C ABI of the Rust lens model (crate pimax-lens, libpimax_lens.a). Used by the thin C glue pimax_lens.c of the
 * Monado pimax driver. See rust/pimax-lens/src/capi.rs for the semantics. */
#ifndef PIMAX_LENS_RS_H
#define PIMAX_LENS_RS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Incremented when the ABI changes incompatibly. */
#define PIMAX_LENS_RS_ABI_VERSION 1

typedef struct pimax_lens_rs pimax_lens_rs; /* opaque */

/* level: 0 = info, 1 = warning. msg is a NUL terminated line without newline. */
typedef void (*pimax_lens_rs_log_fn)(void *user, int level, const char *msg);

enum pimax_lens_rs_mask_kind {
	PIMAX_LENS_RS_MASK_HIDDEN = 0,
	PIMAX_LENS_RS_MASK_VISIBLE = 1,
	PIMAX_LENS_RS_MASK_LINE_LOOP = 2,
};

uint32_t
pimax_lens_rs_abi_version(void);

/* Reads lens.conf (PIMAX_LENS_CONF, $XDG_CONFIG_HOME/omarchyvr/lens.conf, ~/.config/omarchyvr/lens.conf), honours
 * PIMAX_LENS_OFF and PIMAX_LENS_DEBUG_GRID=<dir>, logs the active configuration through `log`.
 * aspect = display width / height as the client sees it (3840/3552). Never returns NULL except on OOM. */
pimax_lens_rs *
pimax_lens_rs_create(float aspect, pimax_lens_rs_log_fn log, void *user);

void
pimax_lens_rs_destroy(pimax_lens_rs *lens);

/* FOV half-angles in degrees, all positive: left, right, up, down. Returns 0, or -1 on a bad argument. */
int
pimax_lens_rs_get_fov(const pimax_lens_rs *lens, uint32_t view, float out_deg[4]);

/* Outward cant of each display in degrees (cant_deg in lens.conf, PIMAX_CANT_DEG overrides; default 0).
 * The driver yaws the left view by +cant and the right view by -cant about +Y. */
float
pimax_lens_rs_get_cant_deg(const pimax_lens_rs *lens);

/* The distortion at display point (u, v) of `view` (0 left, 1 right): UVs to sample for R, G and B:
 * out_uv = r.x r.y g.x g.y b.x b.y. Returns 0, or -1 on a bad argument. */
int
pimax_lens_rs_compute(const pimax_lens_rs *lens, uint32_t view, float u, float v, float out_uv[6]);

/* 1 if the eye has a configured edge black-out / visibility mask, else 0. */
int
pimax_lens_rs_has_mask(const pimax_lens_rs *lens, uint32_t view);

/* Visibility mask mesh. counts[0] = vertex count, counts[1] = index count; vertices are x,y float pairs.
 * Call with NULL buffers and zero capacities first (returns -2 and fills counts), then with buffers.
 * Returns 0 ok, -1 bad argument, -2 buffers too small, -3 no mask configured (use the generic mask). */
int
pimax_lens_rs_visibility_mask(const pimax_lens_rs *lens,
                              uint32_t view,
                              int kind,
                              float *vertices_xy,
                              uint32_t vertex_capacity,
                              uint32_t *indices,
                              uint32_t index_capacity,
                              uint32_t counts[2]);

#ifdef __cplusplus
}
#endif

#endif /* PIMAX_LENS_RS_H */
