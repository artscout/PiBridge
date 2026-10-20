// SPDX-License-Identifier: MIT
// pibridge-xrkit sky: a 360-degree equirectangular picture behind everything (pibridge-home's backgrounds). The eye ray
// (rotation only: the picture is infinitely far) -> longitude / latitude -> the picture (sRGB, linearised by the
// sampler). The CPU reference is pibridge-home's sky.h skyDir().
#version 450
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;
layout(push_constant) uniform Sky {
    mat4 stageFromEye; // rotation used only
    vec4 tanv;         // left, right, up, down
    vec4 misc;         // 1 = sRGB swapchain, yaw (rad), brightness, 0
} pc;
layout(set = 0, binding = 0) uniform sampler2D pic;
void main() {
    float tx = mix(pc.tanv.x, pc.tanv.y, uv.x);
    float ty = mix(pc.tanv.z, pc.tanv.w, uv.y);
    vec3 d = normalize(mat3(pc.stageFromEye) * vec3(tx, ty, -1.0));
    // longitude 0 = -z (forward at the start), growing to the right (+x); latitude + = up
    float lon = atan(d.x, -d.z) + pc.misc.y;
    float lat = asin(clamp(d.y, -1.0, 1.0));
    vec2 st = vec2(fract(lon / 6.28318530718 + 0.5), 0.5 - lat / 3.14159265359);
    vec3 c = textureLod(pic, st, 0.0).rgb * pc.misc.z; // linear (sRGB image format)
    if (pc.misc.x < 0.5) {
        c = pow(c, vec3(1.0 / 2.2)); // a UNORM swapchain takes sRGB-encoded values
    }
    outColor = vec4(c, 1.0);
}
