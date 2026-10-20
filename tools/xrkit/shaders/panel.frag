// SPDX-License-Identifier: MIT
#version 450
layout(location = 0) in vec2 vTc;
layout(location = 0) out vec4 outColor;
layout(push_constant) uniform Push {
    mat4 viewProj;
    vec4 flags; // x: 1 = sRGB swapchain, y: opacity
} pc;
layout(set = 0, binding = 0) uniform sampler2D pic;
void main() {
    vec3 c = texture(pic, vTc).rgb; // linear (sRGB image format)
    if (pc.flags.x < 0.5) {
        c = pow(c, vec3(1.0 / 2.2));
    }
    outColor = vec4(c, pc.flags.y);
}
