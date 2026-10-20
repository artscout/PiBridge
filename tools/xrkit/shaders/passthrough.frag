// SPDX-License-Identifier: MIT
// pibridge-xrkit passthrough (pibridge/xrkit/passthrough.h; pt::sampleFull is the same arithmetic on the CPU): the eye ray
// at the display time -> the room prism, the floor plane or a sphere around the eye -> each camera at its capture
// time -> KB4 -> the first camera that sees it, cross-faded with the best other one at its edge -> grey.
#version 450
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;
layout(set = 0, binding = 0, std140) uniform Eye {
    mat4 stageFromEye;
    vec4 tanv;      // left, right, up, down
    vec4 misc;      // floor y, radius, gain, sRGB
    vec4 info;      // cameras, feather px, vignette radius px, max distance
    vec4 order;     // camera slots, best first (-1: none)
    vec4 roomInfo;  // walls, ceiling y, room on, border px
    vec4 walls[16]; // x0 z0 x1 z1
    mat4 camFromStage[4];
    vec4 kbF[4];    // fx fy cx cy
    vec4 kbK[4];    // k1..k4
    vec4 kbL[4];    // theta limit, valid, gain, 0
} e;
layout(set = 0, binding = 1) uniform sampler2DArray cams;

// st (0..1) and the weight of slot c for stage point p; weight 0 = not seen
float project(int c, vec3 p, out vec2 st) {
    st = vec2(0.0);
    if (e.kbL[c].y < 0.5) {
        return 0.0;
    }
    vec3 q = (e.camFromStage[c] * vec4(p, 1.0)).xyz;
    float r = length(q.xy);
    float th = atan(r, q.z);
    if (th > e.kbL[c].x) {
        return 0.0;
    }
    vec4 k = e.kbK[c];
    float t2 = th * th;
    float td = th * (1.0 + t2 * (k.x + t2 * (k.y + t2 * (k.z + t2 * k.w))));
    vec2 dir = r > 1e-9 ? q.xy / r : vec2(0.0);
    vec2 px = vec2(e.kbF[c].x * td * dir.x + e.kbF[c].z, e.kbF[c].y * td * dir.y + e.kbF[c].w);
    st = (px + 0.5) / vec2(640.0, 480.0);
    float b = e.roomInfo.w;
    float edge = min(min(px.x - b, 639.0 - b - px.x), min(px.y - b, 479.0 - b - px.y));
    float vig = e.info.z - length(px - e.kbF[c].zw);
    return clamp(min(edge, vig) / max(1.0, e.info.y), 0.0, 1.0);
}

float distanceAlong(vec3 o, vec3 d) {
    const float big = 1e9;
    float tf = big;
    if (d.y < -1e-4) {
        float t = (e.misc.x - o.y) / d.y;
        if (t > 0.0) {
            tf = t;
        }
    }
    if (e.roomInfo.z > 0.5) {
        float t = tf;
        if (d.y > 1e-4) {
            float tc = (e.roomInfo.y - o.y) / d.y;
            if (tc > 0.0) {
                t = min(t, tc);
            }
        }
        int nw = int(e.roomInfo.x);
        for (int i = 0; i < nw; i++) {
            vec4 w = e.walls[i];
            vec2 ed = w.zw - w.xy;
            float den = d.x * ed.y - d.z * ed.x;
            if (abs(den) < 1e-9) {
                continue;
            }
            vec2 wo = w.xy - o.xz;
            float tw = (wo.x * ed.y - wo.y * ed.x) / den;
            float s = (wo.x * d.z - wo.y * d.x) / den;
            if (tw > 1e-4 && s >= 0.0 && s <= 1.0) {
                t = min(t, tw);
            }
        }
        return t < big ? min(t, e.info.w) : e.misc.y;
    }
    return tf < big ? min(tf, e.info.w) : e.misc.y;
}

void main() {
    float tx = mix(e.tanv.x, e.tanv.y, uv.x);
    float ty = mix(e.tanv.z, e.tanv.w, uv.y);
    vec3 d = mat3(e.stageFromEye) * normalize(vec3(tx, ty, -1.0));
    vec3 o = e.stageFromEye[3].xyz;
    vec3 p = o + d * distanceAlong(o, d);
    int n = int(e.info.x);
    int a = -1;
    int b = -1;
    float wa = 0.0;
    float wb = 0.0;
    vec2 sa = vec2(0.0);
    vec2 sb = vec2(0.0);
    for (int i = 0; i < 4; i++) {
        if (i >= n) {
            break;
        }
        int c = int(e.order[i]);
        if (c < 0) {
            continue;
        }
        vec2 st;
        float w = project(c, p, st);
        if (w <= 0.0) {
            continue;
        }
        if (a < 0) {
            a = c;
            wa = w;
            sa = st;
            if (w >= 1.0) {
                break;
            }
        } else if (b < 0 || w > wb) {
            b = c;
            wb = w;
            sb = st;
        }
    }
    const float none = 5.0 / 255.0;
    float g;
    if (a < 0) {
        g = none;
    } else {
        float ga = texture(cams, vec3(sa, float(a))).r * e.kbL[a].z;
        if (b >= 0) {
            float alpha = wa / (wa + (1.0 - wa) * wb);
            float gb = texture(cams, vec3(sb, float(b))).r * e.kbL[b].z;
            g = ga * alpha + gb * (1.0 - alpha);
        } else {
            g = ga * wa + none * (1.0 - wa);
        }
        g = clamp(g * e.misc.z, 0.0, 1.0);
    }
    if (e.misc.w > 0.5) {
        g = pow(g, 2.2);
    }
    outColor = vec4(vec3(g), 1.0);
}
