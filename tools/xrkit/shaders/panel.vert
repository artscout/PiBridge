// SPDX-License-Identifier: MIT
// pibridge-xrkit panel: a textured mesh in stage space (pibridge-home's curved PiBridge window).
#version 450
layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 tc;
layout(push_constant) uniform Push {
    mat4 viewProj;
    vec4 flags; // x: 1 = sRGB swapchain, y: opacity
} pc;
layout(location = 0) out vec2 vTc;
void main() {
    gl_Position = pc.viewProj * vec4(pos, 1.0);
    vTc = tc;
}
