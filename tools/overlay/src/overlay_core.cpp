// SPDX-License-Identifier: MIT
#include "overlay_core.h"

#include "pibridge_status.h" // json_escape

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

namespace pibridge::overlay {

namespace {

struct Rect {
    std::uint32_t x0, y0, x1, y1; // top-left origin, [x0, x1) x [y0, y1)
    bool has(std::uint32_t x, std::uint32_t y) const { return x >= x0 && x < x1 && y >= y0 && y < y1; }
};

// Page layout, fixed for kPanelW x kPanelH: the tab's header, one big button, the status line under it.
constexpr std::uint32_t kWideX0 = 24, kWideX1 = kPanelW - 24;
constexpr Rect kHeaderRow{kWideX0, 14, kWideX1, 44};
constexpr Rect kButton{kWideX0, 54, kWideX1, 150};
constexpr Rect kStatusRow{kWideX0, 164, kWideX1, 204};

struct Rgba {
    std::uint8_t r, g, b, a;
};

constexpr Rgba kBg{24, 26, 32, 255};        // the dark card
constexpr Rgba kBtn{44, 48, 60, 255};       // a resting button
constexpr Rgba kBtnHot{58, 110, 196, 255};  // the hovered button (accent)
constexpr Rgba kBtnOff{34, 36, 44, 255};    // a disabled button
constexpr Rgba kBtnRec{150, 36, 40, 255};   // STOP RECORDING (a recording runs)
constexpr Rgba kBtnRecHot{200, 54, 60, 255};
constexpr Rgba kBorder{80, 86, 104, 255};   // a resting edge
constexpr Rgba kBorderHot{150, 190, 255, 255};
constexpr Rgba kBorderOff{56, 60, 72, 255};
constexpr Rgba kBorderRec{255, 120, 120, 255};
constexpr Rgba kText{236, 236, 242, 255};
constexpr Rgba kTextOff{120, 124, 138, 255};
constexpr Rgba kHeader{150, 156, 174, 255}; // a dim header / status line
constexpr Rgba kRed{255, 92, 92, 255};      // the recording indicator
constexpr Rgba kAmber{240, 170, 60, 255};   // a room setup runs
constexpr Rgba kIconInk{236, 236, 242, 255};
constexpr Rgba kIconRec{235, 40, 45, 255};
constexpr Rgba kIconRecIdle{150, 60, 64, 255};

// A 5x7 uppercase font (digits, the letters and the few signs the labels need; others draw blank). Each row is 5 bits, b4 = leftmost.
const std::array<std::uint8_t, 7>& glyph(char c) {
    static const std::array<std::uint8_t, 7> kSpace{0, 0, 0, 0, 0, 0, 0};
    static const std::array<std::uint8_t, 7> k0{0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> k1{0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const std::array<std::uint8_t, 7> k2{0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
    static const std::array<std::uint8_t, 7> k3{0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> k4{0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
    static const std::array<std::uint8_t, 7> k5{0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> k6{0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> k7{0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    static const std::array<std::uint8_t, 7> k8{0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> k9{0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C};
    static const std::array<std::uint8_t, 7> kA{0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kB{0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
    static const std::array<std::uint8_t, 7> kC{0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> kD{0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C};
    static const std::array<std::uint8_t, 7> kE{0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    static const std::array<std::uint8_t, 7> kF{0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
    static const std::array<std::uint8_t, 7> kG{0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F};
    static const std::array<std::uint8_t, 7> kH{0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kI{0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const std::array<std::uint8_t, 7> kJ{0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C};
    static const std::array<std::uint8_t, 7> kK{0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    static const std::array<std::uint8_t, 7> kL{0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
    static const std::array<std::uint8_t, 7> kM{0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kN{0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kO{0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> kP{0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
    static const std::array<std::uint8_t, 7> kQ{0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
    static const std::array<std::uint8_t, 7> kR{0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    static const std::array<std::uint8_t, 7> kS{0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
    static const std::array<std::uint8_t, 7> kT{0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const std::array<std::uint8_t, 7> kU{0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> kV{0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04};
    static const std::array<std::uint8_t, 7> kW{0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11};
    static const std::array<std::uint8_t, 7> kX{0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kY{0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
    static const std::array<std::uint8_t, 7> kDot{0, 0, 0, 0, 0, 0x0C, 0x0C};
    static const std::array<std::uint8_t, 7> kComma{0, 0, 0, 0, 0x0C, 0x04, 0x08};
    static const std::array<std::uint8_t, 7> kDash{0, 0, 0, 0x1F, 0, 0, 0};
    static const std::array<std::uint8_t, 7> kColon{0, 0x0C, 0x0C, 0, 0x0C, 0x0C, 0};
    static const std::array<std::uint8_t, 7> kZ{0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F};
    switch (c) {
    case '0': return k0;
    case '1': return k1;
    case '2': return k2;
    case '3': return k3;
    case '4': return k4;
    case '5': return k5;
    case '6': return k6;
    case '7': return k7;
    case '8': return k8;
    case '9': return k9;
    case 'A': return kA;
    case 'B': return kB;
    case 'C': return kC;
    case 'D': return kD;
    case 'E': return kE;
    case 'F': return kF;
    case 'G': return kG;
    case 'H': return kH;
    case 'I': return kI;
    case 'J': return kJ;
    case 'K': return kK;
    case 'L': return kL;
    case 'M': return kM;
    case 'N': return kN;
    case 'O': return kO;
    case 'P': return kP;
    case 'Q': return kQ;
    case 'R': return kR;
    case 'S': return kS;
    case 'T': return kT;
    case 'U': return kU;
    case 'V': return kV;
    case 'W': return kW;
    case 'X': return kX;
    case 'Y': return kY;
    case 'Z': return kZ;
    case '.': return kDot;
    case ',': return kComma;
    case '-': return kDash;
    case ':': return kColon;
    default: return kSpace;
    }
}

// An RGBA8 canvas, top row first. fill/border/text overwrite; disc/ring blend with 4x4 supersampled coverage (smooth
// edges on the icons, which are drawn on a transparent background).
class Canvas {
public:
    Canvas(std::uint32_t w, std::uint32_t h) : w_(w), h_(h), px_(static_cast<std::size_t>(w) * h * 4, 0) {}
    void fill(Rect r, Rgba c) {
        for (std::uint32_t y = r.y0; y < r.y1 && y < h_; y++) {
            for (std::uint32_t x = r.x0; x < r.x1 && x < w_; x++) {
                put(x, y, c);
            }
        }
    }
    void border(Rect r, Rgba c, std::uint32_t t) {
        fill({r.x0, r.y0, r.x1, r.y0 + t}, c);
        fill({r.x0, r.y1 - t, r.x1, r.y1}, c);
        fill({r.x0, r.y0, r.x0 + t, r.y1}, c);
        fill({r.x1 - t, r.y0, r.x1, r.y1}, c);
    }
    // The pixel width of `str` at `scale`.
    static std::uint32_t textWidth(const std::string& str, std::uint32_t scale) {
        return str.empty() ? 0 : static_cast<std::uint32_t>(str.size()) * 6 * scale - scale; // 5 wide + 1 gap
    }
    // `str` drawn centred in `r`, scaled `scale`x, in colour `c`.
    void text(Rect r, const std::string& str, std::uint32_t scale, Rgba c) {
        const std::uint32_t advance = 6 * scale;
        const std::uint32_t w = textWidth(str, scale);
        const std::uint32_t h = 7 * scale;
        const std::uint32_t ox = r.x0 + (r.x1 - r.x0 > w ? (r.x1 - r.x0 - w) / 2 : 0);
        const std::uint32_t oy = r.y0 + (r.y1 - r.y0 > h ? (r.y1 - r.y0 - h) / 2 : 0);
        std::uint32_t cx = ox;
        for (const char ch : str) {
            const std::array<std::uint8_t, 7>& g = glyph(ch);
            for (std::uint32_t row = 0; row < 7; row++) {
                for (std::uint32_t col = 0; col < 5; col++) {
                    if ((g[row] >> (4 - col)) & 1U) {
                        fill({cx + col * scale, oy + row * scale, cx + (col + 1) * scale, oy + (row + 1) * scale}, c);
                    }
                }
            }
            cx += advance;
        }
    }
    // A filled circle (rIn = 0) or a ring rIn..rOut around (cx, cy), blended over what is there.
    void ring(double cx, double cy, double rIn, double rOut, Rgba c) {
        const auto y0 = static_cast<std::uint32_t>(std::max(0.0, std::floor(cy - rOut)));
        const auto x0 = static_cast<std::uint32_t>(std::max(0.0, std::floor(cx - rOut)));
        for (std::uint32_t y = y0; y < h_ && y <= cy + rOut; y++) {
            for (std::uint32_t x = x0; x < w_ && x <= cx + rOut; x++) {
                int in = 0;
                for (int sy = 0; sy < 4; sy++) {
                    for (int sx = 0; sx < 4; sx++) {
                        const double dx = x + (sx + 0.5) / 4.0 - cx, dy = y + (sy + 0.5) / 4.0 - cy;
                        const double d2 = dx * dx + dy * dy;
                        in += (d2 <= rOut * rOut && d2 >= rIn * rIn) ? 1 : 0;
                    }
                }
                if (in > 0) {
                    blend(x, y, c, in / 16.0);
                }
            }
        }
    }
    void disc(double cx, double cy, double r, Rgba c) { ring(cx, cy, 0.0, r, c); }
    std::vector<std::uint8_t> take() { return std::move(px_); }

private:
    void put(std::uint32_t x, std::uint32_t y, Rgba c) {
        std::uint8_t* q = &px_[(static_cast<std::size_t>(y) * w_ + x) * 4];
        q[0] = c.r;
        q[1] = c.g;
        q[2] = c.b;
        q[3] = c.a;
    }
    // Straight-alpha "over": `c` at coverage `k` on top of the pixel.
    void blend(std::uint32_t x, std::uint32_t y, Rgba c, double k) {
        std::uint8_t* q = &px_[(static_cast<std::size_t>(y) * w_ + x) * 4];
        const double sa = c.a / 255.0 * k, da = q[3] / 255.0;
        const double oa = sa + da * (1.0 - sa);
        if (oa <= 0.0) {
            return;
        }
        const auto mix = [&](std::uint8_t s, std::uint8_t d) {
            return static_cast<std::uint8_t>(std::lround((s * sa + d * da * (1.0 - sa)) / oa));
        };
        q[0] = mix(c.r, q[0]);
        q[1] = mix(c.g, q[1]);
        q[2] = mix(c.b, q[2]);
        q[3] = static_cast<std::uint8_t>(std::lround(oa * 255.0));
    }
    std::uint32_t w_, h_;
    std::vector<std::uint8_t> px_;
};

bool isRoomTab(Tab t) { return t == Tab::Room || t == Tab::QuickRoom; }

std::string header(Tab t) {
    switch (t) {
    case Tab::Room: return "ROOM SETUP";
    case Tab::QuickRoom: return "QUICK ROOM SETUP";
    case Tab::Record: return "RECORD VIDEO";
    }
    return {};
}

} // namespace

const char* tabKey(Tab t) {
    switch (t) {
    case Tab::Room: return "pibridge.dashboard";
    case Tab::QuickRoom: return "pibridge.dashboard.quick";
    case Tab::Record: return "pibridge.dashboard.record";
    }
    return "pibridge.dashboard";
}

std::string tabTitle(Tab t, const TabState& s) {
    switch (t) {
    case Tab::Room: return "Room Setup";
    case Tab::QuickRoom: return "Quick Room Setup";
    case Tab::Record: return s.recording ? "Recording" : "Record Video";
    }
    return {};
}

bool buttonEnabled(Tab t, const TabState& s) {
    if (isRoomTab(t)) {
        return !s.roomSetupRunning && !s.roomPending; // never a second room setup
    }
    return !s.recordPending;
}

std::string buttonLabel(Tab t, const TabState& s) {
    if (isRoomTab(t)) {
        if (s.roomSetupRunning) {
            return "ROOM SETUP RUNNING";
        }
        if (s.roomPending) {
            return "STARTING...";
        }
        return t == Tab::Room ? "START ROOM SETUP" : "START QUICK SETUP";
    }
    if (s.recordPending) {
        return s.recording ? "STOPPING..." : "STARTING...";
    }
    return s.recording ? "STOP RECORDING" : "START RECORDING";
}

std::string statusLine(Tab t, const TabState& s) {
    if (isRoomTab(t)) {
        if (s.roomSetupRunning) {
            return "FINISH IT IN THE HEADSET FIRST";
        }
        if (s.roomPending) {
            return "IT OPENS IN A FEW SECONDS";
        }
        return t == Tab::Room ? "FULL OR QUICK, CHOSEN IN THE HEADSET" : "STRAIGHT TO THE QUICK CIRCLE SETUP";
    }
    return s.recording ? "RECORDING NOW" : "NOT RECORDING";
}

std::string requestFor(Tab t, const TabState& s) {
    if (!buttonEnabled(t, s)) {
        return {};
    }
    switch (t) {
    case Tab::Room: return kRequestRoomSetup;
    case Tab::QuickRoom: return kRequestQuickRoomSetup;
    case Tab::Record: return s.recording ? kRequestRecordStop : kRequestRecordStart;
    }
    return {};
}

bool hitButton(float x, float y) {
    if (x < 0 || y < 0 || x >= static_cast<float>(kPanelW) || y >= static_cast<float>(kPanelH)) {
        return false;
    }
    const auto px = static_cast<std::uint32_t>(x);
    const auto py = kPanelH - 1 - static_cast<std::uint32_t>(y); // GL bottom-left -> top-first row
    return kButton.has(px, py);
}

std::vector<std::uint8_t> drawPage(Tab t, const TabState& s, bool hot) {
    Canvas cv(kPanelW, kPanelH);
    cv.fill({0, 0, kPanelW, kPanelH}, kBg);
    cv.text(kHeaderRow, header(t), 3, kHeader);
    const bool on = buttonEnabled(t, s);
    const bool lit = hot && on;
    const bool rec = t == Tab::Record && s.recording;
    const Rgba fill = !on ? kBtnOff : rec ? (lit ? kBtnRecHot : kBtnRec) : (lit ? kBtnHot : kBtn);
    const Rgba edge = !on ? kBorderOff : rec ? kBorderRec : (lit ? kBorderHot : kBorder);
    cv.fill(kButton, fill);
    cv.border(kButton, edge, 3);
    const std::string label = buttonLabel(t, s);
    cv.text(kButton, label, Canvas::textWidth(label, 5) <= kButton.x1 - kButton.x0 - 64 ? 5 : 4, on ? kText : kTextOff);
    const std::string line = statusLine(t, s);
    Rgba ink = kHeader;
    if (rec) {
        ink = kRed;
    } else if (isRoomTab(t) && s.roomSetupRunning) {
        ink = kAmber;
    }
    if (rec) { // the recording indicator: a red dot left of the status words
        const std::uint32_t w = Canvas::textWidth(line, 2);
        const double cx = (kStatusRow.x0 + kStatusRow.x1) / 2.0 - w / 2.0 - 16.0;
        cv.disc(cx, (kStatusRow.y0 + kStatusRow.y1) / 2.0, 8.0, kRed);
    }
    cv.text(kStatusRow, line, 2, ink);
    return cv.take();
}

std::vector<std::uint8_t> drawIcon(Tab t, const TabState& s) {
    Canvas cv(kIconSize, kIconSize); // transparent
    const double c = kIconSize / 2.0;
    const Rgba ink = isRoomTab(t) && s.roomSetupRunning ? kAmber : kIconInk;
    switch (t) {
    case Tab::Room: // a room outline with you in the middle
        cv.border({18, 18, kIconSize - 18, kIconSize - 18}, ink, 10);
        cv.disc(c, c, 13.0, ink);
        break;
    case Tab::QuickRoom: // the quick circle around you
        cv.ring(c, c, 37.0, 47.0, ink);
        cv.disc(c, c, 13.0, ink);
        break;
    case Tab::Record:
        if (s.recording) { // a solid red dot: recording now
            cv.disc(c, c, 52.0, kIconRec);
            cv.ring(c, c, 52.0, 58.0, kIconInk);
        } else { // a white ring around a dim dot: ready to record
            cv.ring(c, c, 44.0, 53.0, kIconInk);
            cv.disc(c, c, 28.0, kIconRecIdle);
        }
        break;
    }
    return cv.take();
}

bool captureActive(const std::string& controlText) {
    std::istringstream in(controlText);
    std::string line;
    bool active = false;
    while (std::getline(in, line)) {
        std::istringstream is(line);
        std::string key;
        if ((is >> key) && key == "active") {
            int v = 0;
            if (is >> v) {
                active = v != 0;
            }
        }
    }
    return active;
}

bool isRoomSetupCmdline(const std::string& cmdline) {
    std::vector<std::string> argv;
    std::size_t at = 0;
    while (at < cmdline.size()) {
        const std::size_t end = cmdline.find('\0', at);
        argv.push_back(cmdline.substr(at, end == std::string::npos ? std::string::npos : end - at));
        if (end == std::string::npos) {
            break;
        }
        at = end + 1;
    }
    if (argv.empty()) {
        return false;
    }
    const std::size_t slash = argv[0].rfind('/');
    if ((slash == std::string::npos ? argv[0] : argv[0].substr(slash + 1)) != "pibridge-roomsetup") {
        return false;
    }
    for (std::size_t i = 1; i < argv.size(); i++) {
        if (argv[i] == "--map") {
            return true;
        }
    }
    return false;
}

int inlineTabsToSet(int current) {
    const int now = current > 0 ? current : 2; // unset = SteamVR's default of 2
    return now >= kInlineTabsWanted ? 0 : kInlineTabsWanted;
}

std::vector<std::uint8_t> drawRing(double progress) {
    std::vector<std::uint8_t> px(static_cast<std::size_t>(kRingSize) * kRingSize * 4, 0); // transparent background
    const double c = kRingSize / 2.0;
    const double rOut = kRingSize * 0.46, rIn = kRingSize * 0.30; // a thicker band reads denser
    const double p = progress < 0.0 ? 0.0 : (progress > 1.0 ? 1.0 : progress);
    constexpr double tau = 6.283185307179586;
    // 4x4 samples per pixel: the band's edges and the end of the arc are smooth (no stair steps); the colours are
    // mixed by coverage, the alpha is the covered share (straight alpha, as the overlay texture expects)
    constexpr int kSs = 4;
    constexpr double fill[4] = {77, 255, 191, 255};   // the filled arc: bright green-cyan, opaque
    constexpr double track[4] = {120, 170, 205, 140}; // the track: dimmer but solid
    for (std::uint32_t y = 0; y < kRingSize; y++) {
        for (std::uint32_t x = 0; x < kRingSize; x++) {
            double wf = 0.0, wt = 0.0; // covered samples of the arc / the track
            for (int sy = 0; sy < kSs; sy++) {
                for (int sx = 0; sx < kSs; sx++) {
                    const double dx = x + (sx + 0.5) / kSs - c, dy = y + (sy + 0.5) / kSs - c;
                    const double r = std::sqrt(dx * dx + dy * dy);
                    if (r < rIn || r > rOut) {
                        continue;
                    }
                    double a = std::atan2(dx, -dy); // 0 at the top, growing clockwise
                    if (a < 0.0) {
                        a += tau;
                    }
                    (a <= tau * p ? wf : wt) += 1.0;
                }
            }
            const double n = kSs * kSs;
            const double cov = (wf + wt) / n;
            if (cov <= 0.0) {
                continue;
            }
            std::uint8_t* q = &px[(static_cast<std::size_t>(y) * kRingSize + x) * 4];
            // premultiplied sum of both parts, then back to straight colour
            double a = (wf * fill[3] + wt * track[3]) / n;
            for (int k = 0; k < 3; k++) {
                const double pm = (wf * fill[k] * fill[3] + wt * track[k] * track[3]) / n;
                q[k] = static_cast<std::uint8_t>(std::lround(a > 0.0 ? std::min(255.0, pm / a) : 0.0));
            }
            q[3] = static_cast<std::uint8_t>(std::lround(std::min(255.0, a)));
        }
    }
    return px;
}

std::vector<std::uint8_t> drawRingSheet() {
    const std::uint32_t side = kRingSize * static_cast<std::uint32_t>(kRingGrid); // square sheet
    std::vector<std::uint8_t> sheet(static_cast<std::size_t>(side) * side * 4, 0);
    for (int i = 0; i < kRingFrames; i++) {
        const std::vector<std::uint8_t> fr = drawRing(static_cast<double>(i) / (kRingFrames - 1));
        const std::uint32_t x0 = kRingSize * static_cast<std::uint32_t>(i % kRingGrid);
        const std::uint32_t y0 = kRingSize * static_cast<std::uint32_t>(i / kRingGrid);
        for (std::uint32_t y = 0; y < kRingSize; y++) {
            std::memcpy(&sheet[(static_cast<std::size_t>(y0 + y) * side + x0) * 4],
                        &fr[static_cast<std::size_t>(y) * kRingSize * 4], static_cast<std::size_t>(kRingSize) * 4);
        }
    }
    return sheet;
}

std::string manifestJson(const std::string& appKey, const std::string& binaryPath, const std::string& iconPath,
                         const std::string& name) {
    const auto q = [](const std::string& s) { return "\"" + status::json_escape(s) + "\""; };
    std::string o = "{\n";
    o += "  \"source\": \"builtin\",\n";
    o += "  \"applications\": [\n";
    o += "    {\n";
    o += "      \"app_key\": " + q(appKey) + ",\n";
    o += "      \"launch_type\": \"binary\",\n";
    o += "      \"binary_path_linux\": " + q(binaryPath) + ",\n";
    o += "      \"arguments\": \"\",\n";
    o += "      \"is_dashboard_overlay\": true,\n";
    o += "      \"image_path\": " + q(iconPath) + ",\n";
    o += "      \"strings\": {\n";
    o += "        \"en_us\": {\n";
    o += "          \"name\": " + q(name) + ",\n";
    o += "          \"description\": \"PiBridge room setup and video recording buttons in the SteamVR dashboard.\"\n";
    o += "        }\n";
    o += "      }\n";
    o += "    }\n";
    o += "  ]\n";
    o += "}\n";
    return o;
}

} // namespace pibridge::overlay
