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

// Three buttons, stacked, with a margin. Fixed for kPanelW x kPanelH. The third is the record toggle.
constexpr Rect kRoomSetup{24, 20, kPanelW - 24, 100};
constexpr Rect kQuickRoomSetup{24, 114, kPanelW - 24, 194};
constexpr Rect kRecord{24, 208, kPanelW - 24, 288};

struct Rgba {
    std::uint8_t r, g, b, a;
};

constexpr Rgba kBg{24, 26, 32, 255};       // the dark card
constexpr Rgba kBtn{44, 48, 60, 255};      // a resting button
constexpr Rgba kBtnHot{58, 110, 196, 255}; // the hovered button (accent)
constexpr Rgba kBorder{80, 86, 104, 255};  // a resting button's edge
constexpr Rgba kBorderHot{150, 190, 255, 255};
constexpr Rgba kText{236, 236, 242, 255};

// A 5x7 uppercase font, only the glyphs the two labels need (others draw blank). Each row is 5 bits, b4 = leftmost.
const std::array<std::uint8_t, 7>& glyph(char c) {
    static const std::array<std::uint8_t, 7> kSpace{0, 0, 0, 0, 0, 0, 0};
    static const std::array<std::uint8_t, 7> kA{0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kC{0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> kD{0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C};
    static const std::array<std::uint8_t, 7> kE{0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    static const std::array<std::uint8_t, 7> kG{0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F};
    static const std::array<std::uint8_t, 7> kI{0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const std::array<std::uint8_t, 7> kK{0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    static const std::array<std::uint8_t, 7> kM{0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kN{0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    static const std::array<std::uint8_t, 7> kO{0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const std::array<std::uint8_t, 7> kP{0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
    static const std::array<std::uint8_t, 7> kQ{0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
    static const std::array<std::uint8_t, 7> kR{0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    static const std::array<std::uint8_t, 7> kS{0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
    static const std::array<std::uint8_t, 7> kT{0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const std::array<std::uint8_t, 7> kU{0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    switch (c) {
    case 'A': return kA;
    case 'C': return kC;
    case 'D': return kD;
    case 'E': return kE;
    case 'G': return kG;
    case 'I': return kI;
    case 'K': return kK;
    case 'M': return kM;
    case 'N': return kN;
    case 'O': return kO;
    case 'P': return kP;
    case 'Q': return kQ;
    case 'R': return kR;
    case 'S': return kS;
    case 'T': return kT;
    case 'U': return kU;
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

std::string requestFor(Button b, bool recording) {
    switch (b) {
    case Button::RoomSetup: return kRequestRoomSetup;
    case Button::QuickRoomSetup: return kRequestQuickRoomSetup;
    case Button::Record: return recording ? kRequestRecordStop : kRequestRecordStart;
    case Button::None: break;
    }
    return {};
}

Button hitTest(float x, float y) {
    if (x < 0 || y < 0 || x >= static_cast<float>(kPanelW) || y >= static_cast<float>(kPanelH)) {
        return Button::None;
    }
    const auto px = static_cast<std::uint32_t>(x);
    const auto py = kPanelH - 1 - static_cast<std::uint32_t>(y); // GL bottom-left -> top-first row
    if (kRoomSetup.has(px, py)) {
        return Button::RoomSetup;
    }
    if (kQuickRoomSetup.has(px, py)) {
        return Button::QuickRoomSetup;
    }
    if (kRecord.has(px, py)) {
        return Button::Record;
    }
    return Button::None;
}

std::vector<std::uint8_t> drawPanel(Button hovered, bool recording) {
    Canvas cv;
    cv.fill({0, 0, kPanelW, kPanelH}, kBg);
    drawButton(cv, kRoomSetup, "ROOM SETUP", hovered == Button::RoomSetup);
    drawButton(cv, kQuickRoomSetup, "QUICK ROOM SETUP", hovered == Button::QuickRoomSetup);
    drawButton(cv, kRecord, recording ? "STOP RECORDING" : "START RECORDING", hovered == Button::Record);
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
    o += "          \"description\": \"Launch PiBridge room setup from the SteamVR dashboard.\"\n";
    o += "        }\n";
    o += "      }\n";
    o += "    }\n";
    o += "  ]\n";
    o += "}\n";
    return o;
}

} // namespace pibridge::overlay
