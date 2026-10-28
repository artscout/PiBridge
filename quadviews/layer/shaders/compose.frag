// SPDX-License-Identifier: MIT
// Compose one eye: the wide (peripheral) view upscaled over the whole image, the focus view on top in its window,
// blended in over a band at its edge. The two views share the eye pose, so the focus window is an axis-aligned
// rectangle in the image (computed on the CPU, in tangent space).
//
// The views are sampled straight from the game's swapchain images, inside their sub-image rectangles (no copies).
#version 450

layout(set = 0, binding = 0) uniform sampler2D peripheralTex;
layout(set = 0, binding = 1) uniform sampler2D focusTex;

layout(push_constant) uniform Params {
    vec4 focusRect;   // the focus window in the image uv: min.xy, max.xy
    vec4 periSrc;     // the peripheral sub-image in its texture (normalised): offset.xy, size.xy
    vec4 focusSrc;    // the focus sub-image in its texture
    vec4 periTexel;   // peripheral texture: 1/w, 1/h, w, h
    vec4 focusTexel;  // focus texture
    vec4 params;      // transition (share of the focus size), sharpen (0..1), bicubic (0/1), debug overlay (0/1)
    vec4 gaze;        // the gaze in the image uv (debug), x y; z = 1 when the focus follows a live gaze;
                      // w = edge opacity: share of the focus at the outer edge of the band
} pc;

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

vec4 tap(sampler2D t, vec2 uv, vec2 lo, vec2 hi) {
    return textureLod(t, clamp(uv, lo, hi), 0.0);
}

// Catmull-Rom from 9 bilinear taps (the inner 2x2 of the 4x4 kernel folded into one tap per axis).
vec4 catmullRom(sampler2D t, vec2 uv, vec2 size, vec2 lo, vec2 hi) {
    const vec2 pos = uv * size;
    const vec2 p1 = floor(pos - 0.5) + 0.5;
    const vec2 f = pos - p1;
    const vec2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
    const vec2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
    const vec2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
    const vec2 w3 = f * f * (-0.5 + 0.5 * f);
    const vec2 w12 = w1 + w2;
    const vec2 t0 = (p1 - 1.0) / size;
    const vec2 t12 = (p1 + w2 / w12) / size;
    const vec2 t3 = (p1 + 2.0) / size;
    vec4 c = tap(t, vec2(t0.x, t0.y), lo, hi) * w0.x * w0.y
           + tap(t, vec2(t12.x, t0.y), lo, hi) * w12.x * w0.y
           + tap(t, vec2(t3.x, t0.y), lo, hi) * w3.x * w0.y
           + tap(t, vec2(t0.x, t12.y), lo, hi) * w0.x * w12.y
           + tap(t, vec2(t12.x, t12.y), lo, hi) * w12.x * w12.y
           + tap(t, vec2(t3.x, t12.y), lo, hi) * w3.x * w12.y
           + tap(t, vec2(t0.x, t3.y), lo, hi) * w0.x * w3.y
           + tap(t, vec2(t12.x, t3.y), lo, hi) * w12.x * w3.y
           + tap(t, vec2(t3.x, t3.y), lo, hi) * w3.x * w3.y;
    return max(c, vec4(0.0));
}

// Contrast-adaptive sharpening (the CAS idea: sharpen less where the local range is already near the limits, so
// edges do not ring). `amount` 0..1 maps to CAS's sharpness.
vec4 sharpened(sampler2D t, vec2 uv, vec2 texel, vec2 lo, vec2 hi, float amount, float sharpness) {
    const vec4 e = tap(t, uv, lo, hi);
    if (amount <= 0.001) {
        return e;
    }
    const vec3 b = tap(t, uv + vec2(0.0, -texel.y), lo, hi).rgb;
    const vec3 d = tap(t, uv + vec2(-texel.x, 0.0), lo, hi).rgb;
    const vec3 f = tap(t, uv + vec2(texel.x, 0.0), lo, hi).rgb;
    const vec3 h = tap(t, uv + vec2(0.0, texel.y), lo, hi).rgb;
    const vec3 mn = min(min(min(d, e.rgb), min(f, b)), h);
    const vec3 mx = max(max(max(d, e.rgb), max(f, b)), h);
    if (any(greaterThan(mx, vec3(1.0)))) {
        return e; // HDR values: CAS's range test does not apply
    }
    const vec3 amp = sqrt(clamp(min(mn, 1.0 - mx) / max(mx, vec3(1e-5)), 0.0, 1.0));
    const vec3 w = amp * (-1.0 / mix(8.0, 5.0, sharpness));
    const vec3 r = clamp((e.rgb + (b + d + f + h) * w) / (1.0 + 4.0 * w), 0.0, 1.0);
    return vec4(mix(e.rgb, r, amount), e.a);
}

float ring(vec2 uv, vec2 c, float radiusPx, float thickPx) {
    const vec2 dPx = (uv - c) * pc.periTexel.zw / pc.periSrc.zw;
    return 1.0 - smoothstep(thickPx, thickPx + 1.0, abs(length(dPx) - radiusPx));
}

void main() {
    const vec2 uv = inUv;
    const float transition = pc.params.x;
    const float sharpen = pc.params.y;
    const bool bicubic = pc.params.z > 0.5;
    const bool debug = pc.params.w > 0.5;

    const vec2 pLo = pc.periSrc.xy + 0.5 * pc.periTexel.xy;
    const vec2 pHi = pc.periSrc.xy + pc.periSrc.zw - 0.5 * pc.periTexel.xy;
    const vec2 puv = pc.periSrc.xy + uv * pc.periSrc.zw;
    vec4 color = bicubic ? catmullRom(peripheralTex, puv, pc.periTexel.zw, pLo, pHi)
                         : tap(peripheralTex, puv, pLo, pHi);

    const vec2 fsize = pc.focusRect.zw - pc.focusRect.xy;
    const vec2 fuv = (uv - pc.focusRect.xy) / max(fsize, vec2(1e-6));
    const bool inside = all(greaterThanEqual(fuv, vec2(0.0))) && all(lessThanEqual(fuv, vec2(1.0)));
    float alpha = 0.0;
    if (inside) {
        const vec2 edge = min(fuv, 1.0 - fuv);
        alpha = transition > 0.0 ? mix(pc.gaze.w, 1.0, smoothstep(0.0, transition, min(edge.x, edge.y))) : 1.0;
        const vec2 fLo = pc.focusSrc.xy + 0.5 * pc.focusTexel.xy;
        const vec2 fHi = pc.focusSrc.xy + pc.focusSrc.zw - 0.5 * pc.focusTexel.xy;
        const vec4 focus = sharpened(focusTex, pc.focusSrc.xy + fuv * pc.focusSrc.zw, pc.focusTexel.xy, fLo, fHi,
                                     sharpen * alpha, sharpen);
        color = mix(color, focus, alpha);
    }

    if (debug) {
        if (inside) {
            const vec3 tint = alpha < 0.999 ? vec3(1.0, 0.55, 0.05) : vec3(0.1, 0.9, 0.3);
            color.rgb = mix(color.rgb, tint, alpha < 0.999 ? 0.25 : 0.06);
        }
        if (pc.gaze.z > 0.5) {
            color.rgb = mix(color.rgb, vec3(0.0, 0.95, 1.0), ring(uv, pc.gaze.xy, 10.0, 1.5));
        }
    }
    outColor = color;
}
