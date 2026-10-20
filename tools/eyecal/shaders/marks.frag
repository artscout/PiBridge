// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/shaders/marks.frag (MIT OR Apache-2.0).
// The check grid: ported from omarchyvr analysis/tools/xr_grid.c `draw_grid` (MIT OR Apache-2.0).
#version 450
// The eye-calibration scene, analytically per pixel (src/marks.cpp `block` fills the uniform block, `shade` there is the
// CPU twin used by the tests): a background colour and up to MAX_MARKS marks, each given by its direction in THIS
// EYE's view space (the CPU does the head-to-eye transform, so canted views need nothing here), an angular radius,
// a colour and a kind: 0 disc, 1 ring (width), 2 cross (two bars of the given width, half length = radius).
// Anti-aliased by the pixel's own angular size. Under the marks, when grid.x is set, xr_grid's head-locked check grid
// in this eye's view space (square grid, red centre cross, circles at 10..50 degrees, orange FOV border); over them the
// gaze dot (gaze.w = radius, 0 = none): a filled disc with a thin dark outline. The constants match src/marks.h. Output is the panel code converted with srgb_to_linear, which gives
// the right bytes for both *_SRGB (re-encoded by the hardware) and *_UNORM attachments.
layout(location = 0) out vec4 outColor;

const int MAX_MARKS = 32;
struct Mark {
    vec4 dir_r;      // unit direction (eye space: x right, y up, -z forward), radius (rad)
    vec4 color_kind; // sRGB colour, kind
    vec4 extra;      // width (rad), unused
};
layout(std140, set = 0, binding = 0) uniform Scene {
    vec4 tan_;   // tangents of the half-angles: left, right, up, down (positive)
    vec4 info;   // width, height, tangent per pixel, mark count
    vec4 bg;     // background sRGB
    vec4 grid;   // on, grid step (tangent units), FOV border (pixels), unused
    vec4 gaze;   // unit direction of the gaze dot (eye space), radius (rad; 0 = no dot)
    vec4 gaze_col; // its sRGB colour, outline width (rad)
    Mark marks[MAX_MARKS];
} u;

const vec3 RING_RGB[5] = vec3[5](vec3(1.0, 1.0, 1.0), vec3(1.0, 1.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, 1.0, 1.0),
                                 vec3(1.0, 0.0, 1.0));
const float RING_DEG[5] = float[5](10.0, 20.0, 30.0, 40.0, 50.0);
const vec3 LINE_RGB = vec3(144.0 / 255.0);
const vec3 MAJOR_RGB = vec3(224.0 / 255.0);
const vec3 CROSS_RGB = vec3(1.0, 0.0, 0.0);
const vec3 BORDER_RGB = vec3(1.0, 128.0 / 255.0, 0.0);
const vec3 OUTLINE_RGB = vec3(0.02, 0.02, 0.03);

float srgb_to_linear(float c) {
    return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4);
}

float cover(float d, float pix) { return clamp(d / pix + 0.5, 0.0, 1.0); }

// xr_grid's coverage of a line of half-width w at distance d
float line_cov(float d, float w, float pix) { return clamp((w - abs(d)) / pix + 0.5, 0.0, 1.0); }

vec3 grid(vec3 c, float tx, float ty, float tpp, float step) {
    float gx = tx / step, gy = ty / step;
    float fx = gx - floor(gx + 0.5), fy = gy - floor(gy + 0.5);
    int ix = int(floor(gx + 0.5)), iy = int(floor(gy + 0.5));
    bool mx = abs(ix) % 5 == 0, my = abs(iy) % 5 == 0;
    float ps = tpp / step;
    float ax = line_cov(fx, ps * (mx ? 2.0 : 1.0), ps), ay = line_cov(fy, ps * (my ? 2.0 : 1.0), ps);
    c = mix(c, LINE_RGB, ax);
    c = mix(c, LINE_RGB, ay);
    if (mx) c = mix(c, MAJOR_RGB, ax);
    if (my) c = mix(c, MAJOR_RGB, ay);
    float r = sqrt(tx * tx + ty * ty);
    for (int i = 0; i < 5; i++) c = mix(c, RING_RGB[i], line_cov(r - tan(radians(RING_DEG[i])), 2.0 * tpp, tpp));
    c = mix(c, CROSS_RGB, line_cov(tx, 3.0 * tpp, tpp));
    c = mix(c, CROSS_RGB, line_cov(ty, 3.0 * tpp, tpp));
    return c;
}

void main() {
    vec2 px = gl_FragCoord.xy;
    float w = u.info.x, h = u.info.y;
    float tl = u.tan_.x, tr = u.tan_.y, tu = u.tan_.z, td = u.tan_.w;
    float tx = -tl + px.x / w * (tl + tr);
    float ty = tu - px.y / h * (tu + td);
    float r2 = tx * tx + ty * ty;
    float n = sqrt(r2 + 1.0);
    vec3 v = vec3(tx / n, ty / n, -1.0 / n);
    float pix = u.info.z / (1.0 + r2);
    vec3 code = u.bg.rgb;
    if (u.grid.x > 0.5) {
        code = grid(code, tx, ty, u.info.z, u.grid.y);
        float e = u.grid.z;
        if (px.x < e || px.y < e || px.x >= w - e || px.y >= h - e) code = BORDER_RGB;
    }
    int count = min(int(u.info.w + 0.5), MAX_MARKS);
    for (int i = 0; i < count; i++) {
        vec3 d = u.marks[i].dir_r.xyz;
        float r = u.marks[i].dir_r.w;
        int kind = int(u.marks[i].color_kind.w + 0.5);
        float wd = u.marks[i].extra.x;
        float c = dot(d, v);
        // cheap reject: further than radius + width + 4 pixels
        if (c < cos(min(r + wd + 4.0 * pix, 3.0))) continue;
        float ang = atan(length(cross(d, v)), c);
        float cov = 0.0;
        if (kind == 0) {
            cov = cover(r - ang, pix);
        } else if (kind == 1) {
            cov = cover(0.5 * wd - abs(ang - r), pix);
        } else {
            vec3 e1 = normalize(cross(d, vec3(0.0, 1.0, 0.0)));
            vec3 e2 = cross(e1, d);
            float lx = dot(v, e1) / c;
            float ly = dot(v, e2) / c;
            float b1 = min(cover(0.5 * wd - abs(ly), pix), cover(r - abs(lx), pix));
            float b2 = min(cover(0.5 * wd - abs(lx), pix), cover(r - abs(ly), pix));
            cov = max(b1, b2);
        }
        code = mix(code, u.marks[i].color_kind.rgb, cov);
    }
    if (u.gaze.w > 0.0) {
        float r = u.gaze.w, ow = u.gaze_col.w;
        float c = dot(u.gaze.xyz, v);
        if (c >= cos(min(r + ow + 4.0 * pix, 3.0))) {
            float ang = atan(length(cross(u.gaze.xyz, v)), c);
            code = mix(code, OUTLINE_RGB, cover(r + ow - ang, pix));
            code = mix(code, u.gaze_col.rgb, cover(r - ang, pix));
        }
    }
    outColor = vec4(srgb_to_linear(code.r), srgb_to_linear(code.g), srgb_to_linear(code.b), 1.0);
}
