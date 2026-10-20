// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/shaders/marks.vert (MIT OR Apache-2.0).
#version 450
// Full-screen triangle, no vertex buffer.
void main() {
    vec2 p = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
