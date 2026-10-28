// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air lens model: thin C glue around the Rust crate pimax-lens (rust/pimax-lens in pimax-re).
 *
 * The distortion math, the lens.conf parser, the visibility mask and the debug UV grid all live in Rust
 * (libpimax_lens.a, header pimax_lens_rs.h); this file only converts types for Monado.
 * Config: ~/.config/pibridge/lens.conf, see analysis/lens-distortion.md and analysis/tools/lens.conf.example.
 * Debug: PIMAX_LENS_DEBUG_GRID=<dir> writes the UV grid pictures of both eyes at start-up, PIMAX_LENS_OFF=1 turns
 * the distortion off, PIMAX_LENS_CONF=<file> picks the config file.
 *
 * @ingroup drv_pimax
 */
#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_visibility_mask.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pimax_lens_rs;

struct pimax_lens
{
	//! Handle of the Rust lens model, NULL before pimax_lens_load.
	struct pimax_lens_rs *rs;
};

//! Load the lens model: defaults, then the config file (PIMAX_LENS_CONF or ~/.config/pibridge/lens.conf).
//! @a aspect is the display width / height as the client sees it (3840/3552).
void
pimax_lens_load(struct pimax_lens *lens, float aspect);

void
pimax_lens_destroy(struct pimax_lens *lens);

//! Convert the eye FOV to an xrt_fov.
struct xrt_fov
pimax_lens_get_fov(const struct pimax_lens *lens, uint32_t view);

//! Outward cant of each display in degrees (lens.conf cant_deg / PIMAX_CANT_DEG, default 0).
float
pimax_lens_get_cant_deg(const struct pimax_lens *lens);

xrt_result_t
pimax_lens_compute(const struct pimax_lens *lens, uint32_t view, float u, float v, struct xrt_uv_triplet *result);

/*!
 * Visibility mask built from the configured edge black-out / mask, or NULL if none is configured for this eye (the
 * caller then uses the generic FOV mask). The caller owns the result (free()).
 */
struct xrt_visibility_mask *
pimax_lens_visibility_mask(const struct pimax_lens *lens, enum xrt_visibility_mask_type type, uint32_t view);

#ifdef __cplusplus
}
#endif
