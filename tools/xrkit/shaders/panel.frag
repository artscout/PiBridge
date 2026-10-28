// SPDX-License-Identifier: MIT
#version 450
layout(location = 0) in vec2 vTc;
layout(location = 0) out vec4 outColor;
layout(push_constant) uniform Push {
    mat4 viewProj;
    vec4 flags; // x: 1 = sRGB swapchain, y: opacity, z: 1 = the picture's own alpha counts, w: 1 = a hole (transparent)
} pc;
layout(set = 0, binding = 0) uniform sampler2D pic;
void main() {
    if (pc.flags.w > 0.5) {
        outColor = vec4(0.0); // the hole pass: see-through, written as is (no blending in that pipeline)
        return;
    }
    vec4 t = texture(pic, vTc);
    vec3 c = t.rgb; // linear (sRGB image format)
    if (pc.flags.x < 0.5) {
        c = pow(c, vec3(1.0 / 2.2));
    }
    float a = pc.flags.y * (pc.flags.z > 0.5 ? t.a : 1.0);
    if (a <= 0.002) {
        discard; // fully transparent (outside the rounded corners): nothing written, not even depth
    }
    outColor = vec4(c, a);
}
