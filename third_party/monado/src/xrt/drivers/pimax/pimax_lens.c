// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air lens model: thin glue around the Rust crate pimax-lens. See pimax_lens.h.
 *
 * The model itself (for a point (u, v) of the user-facing display, after the compositor's panel pre-rotation: the
 * client image UV to sample for R, G and B) is documented in rust/pimax-lens/src/model.rs and
 * analysis/lens-distortion.md in the pimax-re tree. With an unchanged lens.conf the results are bit-identical to the
 * C implementation this file contained before (checked by rust/pimax-lens-difftest against the original source).
 *
 * @ingroup drv_pimax
 */

#include "pimax_lens.h"
#include "pimax_lens_rs.h"

#include "util/u_logging.h"
#include "util/u_misc.h"
#include "math/m_mathinclude.h"

#include <stdlib.h>

#define DEG2RAD(x) ((x) * (float)(M_PI / 180.0))

static void
lens_log(void *user, int level, const char *msg)
{
	(void)user;
	if (level == 1) {
		U_LOG_W("%s", msg);
	} else {
		U_LOG_I("%s", msg);
	}
}

void
pimax_lens_load(struct pimax_lens *lens, float aspect)
{
	if (lens->rs != NULL) {
		pimax_lens_rs_destroy(lens->rs);
	}
	if (pimax_lens_rs_abi_version() != PIMAX_LENS_RS_ABI_VERSION) {
		U_LOG_E("pimax lens: libpimax_lens.a has ABI %u, this driver was built for %u: rebuild rust/pimax-lens",
		        (unsigned)pimax_lens_rs_abi_version(), (unsigned)PIMAX_LENS_RS_ABI_VERSION);
		lens->rs = NULL;
		return;
	}
	lens->rs = pimax_lens_rs_create(aspect, lens_log, NULL);
	if (lens->rs == NULL) {
		U_LOG_E("pimax lens: out of memory");
	}
}

void
pimax_lens_destroy(struct pimax_lens *lens)
{
	if (lens->rs != NULL) {
		pimax_lens_rs_destroy(lens->rs);
		lens->rs = NULL;
	}
}

struct xrt_fov
pimax_lens_get_fov(const struct pimax_lens *lens, uint32_t view)
{
	float d[4] = {50.0f, 50.0f, 44.5f, 44.5f};
	if (lens->rs != NULL) {
		(void)pimax_lens_rs_get_fov(lens->rs, view, d);
	}
	struct xrt_fov fov = {
	    .angle_left = -DEG2RAD(d[0]),
	    .angle_right = DEG2RAD(d[1]),
	    .angle_up = DEG2RAD(d[2]),
	    .angle_down = -DEG2RAD(d[3]),
	};
	return fov;
}

float
pimax_lens_get_cant_deg(const struct pimax_lens *lens)
{
	return lens->rs != NULL ? pimax_lens_rs_get_cant_deg(lens->rs) : 0.0f;
}

xrt_result_t
pimax_lens_compute(const struct pimax_lens *lens, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	float uv[6];
	if (lens->rs == NULL || pimax_lens_rs_compute(lens->rs, view, u, v, uv) != 0) {
		// No model (should not happen): identity mesh instead of failing the compositor.
		result->r.x = result->g.x = result->b.x = u;
		result->r.y = result->g.y = result->b.y = v;
		return XRT_SUCCESS;
	}
	result->r.x = uv[0];
	result->r.y = uv[1];
	result->g.x = uv[2];
	result->g.y = uv[3];
	result->b.x = uv[4];
	result->b.y = uv[5];
	return XRT_SUCCESS;
}

struct xrt_visibility_mask *
pimax_lens_visibility_mask(const struct pimax_lens *lens, enum xrt_visibility_mask_type type, uint32_t view)
{
	if (lens->rs == NULL || pimax_lens_rs_has_mask(lens->rs, view) == 0) {
		return NULL;
	}
	int kind;
	switch (type) {
	case XRT_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH: kind = PIMAX_LENS_RS_MASK_HIDDEN; break;
	case XRT_VISIBILITY_MASK_TYPE_VISIBLE_TRIANGLE_MESH: kind = PIMAX_LENS_RS_MASK_VISIBLE; break;
	case XRT_VISIBILITY_MASK_TYPE_LINE_LOOP: kind = PIMAX_LENS_RS_MASK_LINE_LOOP; break;
	default: return NULL;
	}
	uint32_t counts[2] = {0, 0};
	if (pimax_lens_rs_visibility_mask(lens->rs, view, kind, NULL, 0, NULL, 0, counts) != -2 || counts[0] == 0 ||
	    counts[1] == 0) {
		return NULL;
	}
	const size_t size =
	    sizeof(struct xrt_visibility_mask) + sizeof(uint32_t) * counts[1] + sizeof(struct xrt_vec2) * counts[0];
	struct xrt_visibility_mask *mask = U_CALLOC_WITH_CAST(struct xrt_visibility_mask, size);
	if (mask == NULL) {
		return NULL;
	}
	mask->type = type;
	mask->index_count = counts[1];
	mask->vertex_count = counts[0];
	// struct xrt_vec2 is two floats: the Rust side writes x,y pairs straight into the mask.
	if (pimax_lens_rs_visibility_mask(lens->rs, view, kind, (float *)xrt_visibility_mask_get_vertices(mask), counts[0],
	                                  xrt_visibility_mask_get_indices(mask), counts[1], counts) != 0) {
		free(mask);
		return NULL;
	}
	return mask;
}
