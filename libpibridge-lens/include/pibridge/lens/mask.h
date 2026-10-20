// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/mask.rs (MIT OR Apache-2.0).
//
// Visibility mask meshes (OpenXR `XR_KHR_visibility_mask`) derived from the lens model's visibility mask. The
// visible area of an eye is "inside the rendered FOV" and "not hidden by the mask"; it is traced with rays from the
// optical axis (bisection along each ray), mapped to the green channel's ray tangent plane, and turned into a line
// loop, a visible-area triangle fan and a hidden-area ring of triangles between the polygon and the FOV rectangle.
// Vertex convention (Monado's `u_visibility_mask_get_default`): x = tangent to the right, y = minus the upward one.
#pragma once

#include "pibridge/lens/model.h"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace pibridge::lens {

enum class MaskKind { HiddenTriangleMesh = 0, VisibleTriangleMesh = 1, LineLoop = 2 };
std::optional<MaskKind> mask_kind_from_i32(int v);

struct MaskMesh {
    std::vector<std::array<float, 2>> vertices;
    std::vector<std::uint32_t> indices;
    bool operator==(const MaskMesh&) const = default;
};

/// Polygon (tangent plane, x right, y up) of the visible area traced with `n` rays; nullopt if the optical axis
/// itself is not visible.
std::optional<std::vector<std::array<float, 2>>> outline_tangent(const Lens& lens, std::size_t view, std::size_t n);
/// The mask mesh of `kind` for `view` with `n` boundary samples; nullopt when the eye has no mask (use the generic
/// FOV mask then), when the optical axis is hidden, or n < 8.
std::optional<MaskMesh> visibility_mesh(const Lens& lens, std::size_t view, MaskKind kind, std::size_t n);
/// Area of the triangles of a mesh.
float mesh_area(const MaskMesh& m);

} // namespace pibridge::lens
