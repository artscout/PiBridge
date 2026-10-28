// SPDX-License-Identifier: MIT
// One triangle covering the composed image; uv (0,0) is the top-left pixel corner.
#version 450

layout(location = 0) out vec2 outUv;

void main() {
    const vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    outUv = uv;
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
