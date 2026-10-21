// SPDX-License-Identifier: MIT
#include "overlay_core.h"

#include "pibridge_status.h" // json_escape

#include <array>

namespace pibridge::overlay {

namespace {

struct Rect {
    std::uint32_t x0, y0, x1, y1; // top-left origin, [x0, x1) x [y0, y1)
    bool has(std::uint32_t x, std::uint32_t y) const { return x >= x0 && x < x1 && y >= y0 && y < y1; }
};

// Layout, fixed for kPanelW x kPanelH. One RECORDING button and a ROOM section (a header and two wide buttons),
// stacked with margins.
constexpr std::uint32_t kWideX0 = 24, kWideX1 = kPanelW - 24;

constexpr Rect kRecHeader{kWideX0, 16, kWideX1, 46};
constexpr Rect kRecordBtn{kWideX0, 56, kWideX1, 126};
constexpr Rect kRoomHeader{kWideX0, 150, kWideX1, 180};
constexpr Rect kRoomSetup{kWideX0, 190, kWideX1, 260};
constexpr Rect kQuickRoomSetup{kWideX0, 268, kWideX1, 338};

struct Rgba {
    std::uint8_t r, g, b, a;
};

constexpr Rgba kBg{24, 26, 32, 255};        // the dark card
constexpr Rgba kBtn{44, 48, 60, 255};       // a resting button
constexpr Rgba kBtnHot{58, 110, 196, 255};  // the hovered button (accent)
constexpr Rgba kBorder{80, 86, 104, 255};   // a resting edge
constexpr Rgba kBorderHot{150, 190, 255, 255};
constexpr Rgba kText{236, 236, 242, 255};
constexpr Rgba kHeader{150, 156, 174, 255}; // a dim section header

// A 5x7 uppercase font (digits and the letters the labels need; others draw blank). Each row is 5 bits, b4 = leftmost.
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
    case 'Z': return kZ;
    default: return kSpace;
    }
}

class Canvas {
public:
    Canvas() : px_(static_cast<std::size_t>(kPanelW) * kPanelH * 4) {}
    void fill(Rect r, Rgba c) {
        for (std::uint32_t y = r.y0; y < r.y1 && y < kPanelH; y++) {
            for (std::uint32_t x = r.x0; x < r.x1 && x < kPanelW; x++) {
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
    // `str` drawn centred in `r`, scaled `scale`x, in colour `c`.
    void text(Rect r, const std::string& str, std::uint32_t scale, Rgba c) {
        const std::uint32_t advance = 6 * scale; // 5 wide + 1 gap
        const std::uint32_t w = str.empty() ? 0 : static_cast<std::uint32_t>(str.size()) * advance - scale;
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
    std::vector<std::uint8_t> take() { return std::move(px_); }

private:
    void put(std::uint32_t x, std::uint32_t y, Rgba c) {
        std::uint8_t* q = &px_[(static_cast<std::size_t>(y) * kPanelW + x) * 4];
        q[0] = c.r;
        q[1] = c.g;
        q[2] = c.b;
        q[3] = c.a;
    }
    std::vector<std::uint8_t> px_;
};

void drawButton(Canvas& cv, Rect r, const std::string& label, bool hot) {
    cv.fill(r, hot ? kBtnHot : kBtn);
    cv.border(r, hot ? kBorderHot : kBorder, 2);
    cv.text(r, label, 4, kText);
}

} // namespace

std::string requestFor(Hit h, bool recording) {
    switch (h.kind) {
    case Hit::Kind::Room: return kRequestRoomSetup;
    case Hit::Kind::QuickRoom: return kRequestQuickRoomSetup;
    case Hit::Kind::Record: return recording ? kRequestRecordStop : kRequestRecordStart;
    case Hit::Kind::None: break;
    }
    return {};
}

Hit hitTest(float x, float y) {
    if (x < 0 || y < 0 || x >= static_cast<float>(kPanelW) || y >= static_cast<float>(kPanelH)) {
        return {};
    }
    const auto px = static_cast<std::uint32_t>(x);
    const auto py = kPanelH - 1 - static_cast<std::uint32_t>(y); // GL bottom-left -> top-first row
    if (kRecordBtn.has(px, py)) {
        return {Hit::Kind::Record};
    }
    if (kRoomSetup.has(px, py)) {
        return {Hit::Kind::Room};
    }
    if (kQuickRoomSetup.has(px, py)) {
        return {Hit::Kind::QuickRoom};
    }
    return {};
}

std::vector<std::uint8_t> drawPanel(bool recording, Hit hovered) {
    Canvas cv;
    cv.fill({0, 0, kPanelW, kPanelH}, kBg);
    cv.text(kRecHeader, "RECORDING", 3, kHeader);
    drawButton(cv, kRecordBtn, recording ? "STOP RECORDING" : "START RECORDING", hovered.kind == Hit::Kind::Record);
    cv.text(kRoomHeader, "ROOM", 3, kHeader);
    drawButton(cv, kRoomSetup, "ROOM SETUP", hovered.kind == Hit::Kind::Room);
    drawButton(cv, kQuickRoomSetup, "QUICK ROOM SETUP", hovered.kind == Hit::Kind::QuickRoom);
    return cv.take();
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
    o += "          \"description\": \"PiBridge recording and room setup in the SteamVR dashboard.\"\n";
    o += "        }\n";
    o += "      }\n";
    o += "    }\n";
    o += "  ]\n";
    o += "}\n";
    return o;
}

} // namespace pibridge::overlay
