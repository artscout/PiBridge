// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/capi.rs (MIT OR Apache-2.0).
//
// The C ABI of include/pimax_lens_rs.h (verbatim copy of the Rust crate's header) over libpibridge-lens, called by the
// thin C glue pimax_lens.c of the Monado Pimax driver. All functions are safe to call from several threads once
// pimax_lens_rs_create returned (the handle is read-only afterwards). `view` is 0 (left) or 1 (right). No entry
// point lets an exception out (a catch-all where one can occur): the Rust functions have no panic guard, so a
// failure there would abort the process; here create returns NULL (the header's "OOM" case), the mask -1.
#include "pimax_lens_rs.h"

#include "pibridge/cam/rustfmt.h"
#include "pibridge/lens/lens.h"
#include "pibridge/lens/mask.h"

#include <cstring>
#include <memory>
#include <new>
#include <string>

#define PIBRIDGE_ABI extern "C" __attribute__((visibility("default")))

// The handle (the header only declares it).
struct pimax_lens_rs {
    pibridge::lens::Lens lens;
};

namespace {

using pibridge::lens::Lens;

// Incremented when the C ABI changes incompatibly.
constexpr std::uint32_t ABI_VERSION = 1;
static_assert(ABI_VERSION == PIMAX_LENS_RS_ABI_VERSION);
// Number of boundary samples of the visibility mask meshes.
constexpr std::size_t MASK_SEGMENTS = 64;

static_assert(static_cast<int>(pibridge::lens::MaskKind::HiddenTriangleMesh) == PIMAX_LENS_RS_MASK_HIDDEN);
static_assert(static_cast<int>(pibridge::lens::MaskKind::VisibleTriangleMesh) == PIMAX_LENS_RS_MASK_VISIBLE);
static_assert(static_cast<int>(pibridge::lens::MaskKind::LineLoop) == PIMAX_LENS_RS_MASK_LINE_LOOP);

void emit(pimax_lens_rs_log_fn log, void* user, const std::vector<pibridge::lens::Message>& messages) {
    if (log == nullptr) {
        return;
    }
    for (const auto& m : messages) {
        std::string text;
        text.reserve(m.text.size());
        for (const char c : m.text) {
            if (c != '\0') {
                text.push_back(c);
            }
        }
        log(user, m.level == pibridge::lens::Level::Warn ? 1 : 0, text.c_str());
    }
}

const Lens* handle(const pimax_lens_rs* l, std::uint32_t view) {
    if (l == nullptr || view > 1) {
        return nullptr;
    }
    return &l->lens;
}

} // namespace

PIBRIDGE_ABI uint32_t pimax_lens_rs_abi_version(void) {
    return ABI_VERSION;
}

PIBRIDGE_ABI pimax_lens_rs* pimax_lens_rs_create(float aspect, pimax_lens_rs_log_fn log, void* user) {
    try {
        auto loaded = pibridge::lens::load(aspect, pibridge::lens::process_env);
        emit(log, user, loaded.messages);
        if (const auto dir = pibridge::lens::process_env("PIMAX_LENS_DEBUG_GRID"); dir && !dir->empty()) {
            std::uint32_t div = 4;
            if (const auto d = pibridge::lens::process_env("PIMAX_LENS_DEBUG_GRID_DIV")) {
                std::uint32_t v = 0;
                if (pibridge::cam::rfmt::parse_u32(pibridge::cam::rfmt::trim(*d), v) && v >= 1 && v <= 64) {
                    div = v;
                }
            }
            emit(log, user, pibridge::lens::write_debug_grid(loaded.lens, *dir, div));
        }
        return new pimax_lens_rs{std::move(loaded.lens)};
    } catch (...) {
        return nullptr;
    }
}

PIBRIDGE_ABI void pimax_lens_rs_destroy(pimax_lens_rs* lens) {
    delete lens;
}

PIBRIDGE_ABI int pimax_lens_rs_get_fov(const pimax_lens_rs* l, uint32_t view, float out_deg[4]) {
    const Lens* lens = handle(l, view);
    if (lens == nullptr || out_deg == nullptr) {
        return -1;
    }
    const auto f = lens->fov_deg(view);
    std::memcpy(out_deg, f.data(), sizeof(float) * 4);
    return 0;
}

PIBRIDGE_ABI float pimax_lens_rs_get_cant_deg(const pimax_lens_rs* l) {
    return l == nullptr ? 0.0f : l->lens.config().cant_deg;
}

PIBRIDGE_ABI int pimax_lens_rs_compute(const pimax_lens_rs* l, uint32_t view, float u, float v, float out_uv[6]) {
    const Lens* lens = handle(l, view);
    if (lens == nullptr || out_uv == nullptr) {
        return -1;
    }
    const auto uv = lens->compute(view, u, v);
    for (std::size_t c = 0; c < 3; c++) {
        out_uv[c * 2] = uv[c][0];
        out_uv[c * 2 + 1] = uv[c][1];
    }
    return 0;
}

PIBRIDGE_ABI int pimax_lens_rs_has_mask(const pimax_lens_rs* l, uint32_t view) {
    const Lens* lens = handle(l, view);
    if (lens == nullptr) {
        return 0;
    }
    return lens->config().enabled && lens->has_mask(view) ? 1 : 0;
}

PIBRIDGE_ABI int pimax_lens_rs_visibility_mask(const pimax_lens_rs* l, uint32_t view, int kind, float* vertices_xy,
                                            uint32_t vertex_capacity, uint32_t* indices, uint32_t index_capacity,
                                            uint32_t counts[2]) {
    try {
        const Lens* lens = handle(l, view);
        if (lens == nullptr) {
            return -1;
        }
        const auto k = pibridge::lens::mask_kind_from_i32(kind);
        if (!k || counts == nullptr) {
            return -1;
        }
        const auto mesh = pibridge::lens::visibility_mesh(*lens, view, *k, MASK_SEGMENTS);
        if (!mesh) {
            return -3;
        }
        counts[0] = static_cast<std::uint32_t>(mesh->vertices.size());
        counts[1] = static_cast<std::uint32_t>(mesh->indices.size());
        if (vertices_xy == nullptr || indices == nullptr ||
            static_cast<std::uint32_t>(mesh->vertices.size()) > vertex_capacity ||
            static_cast<std::uint32_t>(mesh->indices.size()) > index_capacity) {
            return -2;
        }
        for (std::size_t i = 0; i < mesh->vertices.size(); i++) {
            vertices_xy[i * 2] = mesh->vertices[i][0];
            vertices_xy[i * 2 + 1] = mesh->vertices[i][1];
        }
        std::memcpy(indices, mesh->indices.data(), mesh->indices.size() * sizeof(std::uint32_t));
        return 0;
    } catch (...) {
        return -1;
    }
}
