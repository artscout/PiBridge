// SPDX-License-Identifier: MIT
// pibridge-xrkit (pibridge-roomsetup, pibridge-home): unlit coloured triangles and lines in stage space.
#version 450
layout(location = 0) in vec3 pos;
layout(location = 1) in vec4 color;
layout(push_constant) uniform Push {
    mat4 viewProj;
    vec4 flags; // x: 1 when the swapchain is sRGB (the palette is sRGB, so it is linearised first)
} pc;
layout(location = 0) out vec4 vColor;
void main() {
    gl_Position = pc.viewProj * vec4(pos, 1.0);
    vec3 c = color.rgb;
    if (pc.flags.x > 0.5) {
        c = pow(c, vec3(2.2));
    }
    vColor = vec4(c, color.a);
}
