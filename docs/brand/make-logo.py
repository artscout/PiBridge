# SPDX-License-Identifier: MIT
# The PiBridge logo: an original cartoon fly (our own character) with big eyes whose irises are pibridgeed, a pilot's scarf
# and a wide VR headset pushed up on the forehead. Usage: python3 make-logo.py app/data/pibridge.svg
import math, sys
OUT, W = "#231f3a", 7
def hexes(cx, cy, r0, r):
    out = []
    w, h = math.sqrt(3) * r, 1.5 * r
    n = int(r0 / r) + 3
    for j in range(-n, n + 1):
        for i in range(-n, n + 1):
            x, y = cx + i * w + (w / 2 if j % 2 else 0), cy + j * h
            c = [(x + r * math.cos(math.radians(60 * k - 30)), y + r * math.sin(math.radians(60 * k - 30))) for k in range(6)]
            if all(math.hypot(px - cx, py - cy) <= r0 * 0.96 for px, py in c):
                out.append('<polygon points="%s"/>' % " ".join("%.1f,%.1f" % p for p in c))
    return "\n".join(out)
o = 'stroke="%s" stroke-width="%d" stroke-linejoin="round" stroke-linecap="round"' % (OUT, W)
s = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" width="512" height="512">', "<defs>",
     '<radialGradient id="skin" cx="0.4" cy="0.3" r="0.9"><stop offset="0" stop-color="#c9ffd9"/><stop offset="0.55" stop-color="#3fd28a"/><stop offset="1" stop-color="#127a5c"/></radialGradient>',
     '<radialGradient id="iris" cx="0.4" cy="0.35" r="0.8"><stop offset="0" stop-color="#ffb35c"/><stop offset="0.55" stop-color="#ff4d3a"/><stop offset="1" stop-color="#a8102b"/></radialGradient>',
     '<linearGradient id="hs" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#8b7bff"/><stop offset="1" stop-color="#4a36d6"/></linearGradient>',
     '<linearGradient id="lens" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#7ff3ff"/><stop offset="1" stop-color="#2a8cff"/></linearGradient>',
     '<linearGradient id="scarf" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="#ff5a7a"/><stop offset="1" stop-color="#e0245e"/></linearGradient>',
     '<linearGradient id="shell" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#ffffff"/><stop offset="1" stop-color="#cfd4e6"/></linearGradient>',
     '<linearGradient id="visor" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#3a3f5c"/><stop offset="0.5" stop-color="#11131f"/><stop offset="1" stop-color="#262a44"/></linearGradient>',
     "</defs>"]
# wings: four small round wings, a motion swoosh
for sx in (-1, 1):
    s.append('<ellipse cx="%d" cy="250" rx="70" ry="40" fill="#e3f7ff" fill-opacity="0.95" %s transform="rotate(%d %d 250)"/>' % (256 + sx * 150, o, sx * 25, 256 + sx * 150))
    s.append('<ellipse cx="%d" cy="300" rx="52" ry="30" fill="#e3f7ff" fill-opacity="0.95" %s transform="rotate(%d %d 300)"/>' % (256 + sx * 140, o, sx * 5, 256 + sx * 140))
    s.append('<path d="M%d 222 Q %d 236 %d 260" fill="none" stroke="#9ad7f0" stroke-width="5" stroke-linecap="round"/>' % (256 + sx * 118, 256 + sx * 150, 256 + sx * 186))
# body: a small round body with stripes, little arms waving
s.append('<ellipse cx="256" cy="392" rx="70" ry="62" fill="url(#skin)" %s/>' % o)
s.append('<path d="M196 392 Q256 414 316 392" fill="none" stroke="#0f6b4f" stroke-opacity="0.45" stroke-width="6" stroke-linecap="round"/>')
s.append('<path d="M206 420 Q256 438 306 420" fill="none" stroke="#0f6b4f" stroke-opacity="0.35" stroke-width="5" stroke-linecap="round"/>')
s.append('<path d="M196 372 Q160 360 146 330" fill="none" %s/><circle cx="144" cy="326" r="10" fill="url(#skin)" %s/>' % (o, o))
s.append('<path d="M316 372 Q350 384 372 360" fill="none" %s/><circle cx="374" cy="356" r="10" fill="url(#skin)" %s/>' % (o, o))
for x0, x1 in ((226, 210), (286, 302)):
    s.append('<path d="M%d 448 Q %d 472 %d 482" fill="none" stroke="%s" stroke-width="9" stroke-linecap="round"/>' % (x0, (x0 + x1) // 2, x1, OUT))
    s.append('<ellipse cx="%d" cy="486" rx="14" ry="8" fill="%s"/>' % (x1, OUT))
# antennae
for sx in (-1, 1):
    s.append('<path d="M%d 150 Q %d 92 %d 74" fill="none" %s/>' % (256 + sx * 36, 256 + sx * 54, 256 + sx * 90, o))
    s.append('<circle cx="%d" cy="74" r="13" fill="#ff5a7a" %s/>' % (256 + sx * 90, o))
# head
s.append('<circle cx="256" cy="236" r="112" fill="url(#skin)" %s/>' % o)
# big eyes: white, pibridgeed iris, pupil, highlights
for cx in (206, 306):
    s.append('<ellipse cx="%d" cy="244" rx="48" ry="56" fill="#ffffff" %s/>' % (cx, o))
    ix = cx + (8 if cx < 256 else -8)
    s.append('<circle cx="%d" cy="254" r="30" fill="url(#iris)" stroke="%s" stroke-width="4"/>' % (ix, OUT))
    s.append('<g fill="none" stroke="#7a0a20" stroke-opacity="0.5" stroke-width="1.8">%s</g>' % hexes(ix, 254, 28, 6))
    s.append('<circle cx="%d" cy="256" r="12" fill="%s"/>' % (ix, OUT))
    s.append('<circle cx="%d" cy="242" r="8" fill="#ffffff"/>' % (ix - 9))
    s.append('<circle cx="%d" cy="264" r="4" fill="#ffffff" fill-opacity="0.9"/>' % (ix + 8))
# smile and cheeks
s.append('<ellipse cx="178" cy="300" rx="16" ry="9" fill="#ff8fb1" fill-opacity="0.85"/><ellipse cx="334" cy="300" rx="16" ry="9" fill="#ff8fb1" fill-opacity="0.85"/>')
s.append('<path d="M226 306 Q256 338 286 306" fill="#7a1f3a" %s/>' % o)
s.append('<path d="M242 320 Q256 330 270 320" fill="#ff6f8f" stroke="none"/>')
# scarf: knot under the head, one tail flying
s.append('<path d="M180 330 Q256 360 332 330 L326 352 Q256 380 186 352 Z" fill="url(#scarf)" %s/>' % o)
s.append('<path d="M314 346 Q370 350 414 324 Q392 360 404 384 Q360 380 306 362 Z" fill="url(#scarf)" %s/>' % o)
# a wide VR headset in the style of a Pimax Crystal (white shell, glossy black front, tracking cameras), pushed up
# on the forehead like pilot goggles; no logo or brand on it
s.append('<path d="M140 200 Q256 104 372 200" fill="none" stroke="#e9ecf5" stroke-width="16" stroke-linecap="round"/>')
s.append('<path d="M140 200 Q256 104 372 200" fill="none" stroke="%s" stroke-width="4" stroke-linecap="round"/>' % OUT)
s.append('<path d="M150 150 Q150 112 192 110 L320 110 Q362 112 362 150 L360 178 Q356 204 326 206 L186 206 Q156 204 152 178 Z" fill="url(#shell)" %s/>' % o)
s.append('<path d="M170 148 Q172 126 200 125 L312 125 Q340 126 342 148 L340 172 Q336 190 314 191 L198 191 Q176 190 172 172 Z" fill="url(#visor)" stroke="%s" stroke-width="5"/>' % OUT)
s.append('<path d="M186 140 Q230 130 290 133" fill="none" stroke="#ffffff" stroke-opacity="0.55" stroke-width="7" stroke-linecap="round"/>')
for cx in (188, 324):
    s.append('<circle cx="%d" cy="180" r="7" fill="#0b0d16" stroke="#5a6080" stroke-width="2.5"/>' % cx)
    s.append('<circle cx="%d" cy="178" r="2" fill="#9fb4ff"/>' % (cx - 1))
s.append('<rect x="240" y="196" width="32" height="8" rx="4" fill="#c7cbe0" stroke="%s" stroke-width="3"/>' % OUT)
s.append("</svg>")
open(sys.argv[1], "w").write("<!-- SPDX-License-Identifier: MIT -->\n" + "\n".join(s))
