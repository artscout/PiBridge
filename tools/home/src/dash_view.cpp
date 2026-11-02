// SPDX-License-Identifier: MIT
#include "dash_view.h"

#include <algorithm>
#include <format>

namespace pibridge::home::dash {

namespace {

constexpr double kMargin = 24;
constexpr double kQuickW = 330; // a quick action's box
constexpr double kQuickGap = 18;
constexpr double kTopH = 110;
constexpr double kBottomH = 110;
constexpr double kNavW = 420;
constexpr double kGap = 20;
constexpr double kRowH = 150;
constexpr double kTextH = 100;
constexpr double kCtrlX = 1150;  // where the controls of a row begin
constexpr double kCtrlH = 96;

/// The quick actions, right-aligned to `right` (Desktop first when `desktop`, then Record video, Quick room setup,
/// Recenter, Close app); where the leftmost begins.
double quickActions(View& v, const Header& h, double right, double y, double bh, bool desktop) {
    struct Q {
        const char* id;
        std::string label;
        Icon icon;
        bool enabled;
        bool selected;
    };
    std::vector<Q> qs;
    if (desktop) {
        qs.push_back({"view:desktop", "Desktop", Icon::Desktop, h.desktopAvailable, false});
    }
    // the recording at hand: Record video / Stop recording (highlighted while it records)
    qs.push_back({"view:record", h.recording ? "Stop recording" : "Record video", h.recording ? Icon::Stop : Icon::Record,
                  h.recordAvailable, h.recording});
    qs.push_back({"view:roomquick", "Quick room setup", Icon::Room, h.roomQuickAvailable, false});
    qs.push_back({"view:recenter", "Recenter", Icon::Recenter, h.recenterAvailable, false});
    if (h.closeShown) {
        qs.push_back({"view:close", "Close app", Icon::Close, !h.closeName.empty(), false});
    }
    double x = right;
    for (auto it = qs.rbegin(); it != qs.rend(); ++it) {
        x -= kQuickW;
        v.boxes.push_back({it->id, {x, y, kQuickW, bh}, it->label, it->selected ? Box::Style::Selected : Box::Style::Bar,
                           it->enabled, it->icon});
        x -= kQuickGap;
    }
    return x;
}

} // namespace

View deskLayout(const Header& h, const DeskBar& d) {
    View v;
    v.w = kW;
    v.h = kBarH;
    const Rect words{kMargin, 14, kW - 2 * kMargin, 86};
    const Rect bar{kMargin, 110, kW - 2 * kMargin, kBarH - 124};
    v.cards = {bar};
    Rect wordsR = words;
    if (d.zoomShown) {
        // Zoom and Distance, right-aligned in the top row: a label, [-], the value, [+]
        const double sh = words.h - 8, sy = words.y + 4;
        double sx = words.x + words.w;
        const auto stepper = [&](const char* id, const char* name, double value) {
            sx -= 96;
            v.boxes.push_back({std::string(id) + ":+", {sx, sy, 96, sh}, "+", Box::Style::Normal, true});
            sx -= 160;
            v.labels.push_back({{sx, sy, 160, sh}, std::format("{:.2f}", value), Label::Kind::Value}); // never "1,25"
            sx -= 96;
            v.boxes.push_back({std::string(id) + ":-", {sx, sy, 96, sh}, "\xE2\x88\x92", Box::Style::Normal, true});
            sx -= 210;
            v.labels.push_back({{sx, sy, 200, sh}, name, Label::Kind::RowLabel});
            sx -= 30;
        };
        stepper("desk:dist", "Distance", d.distance);
        stepper("desk:zoom", "Zoom", d.zoom);
        wordsR.w = sx - words.x;
    }
    v.labels.push_back({wordsR, d.words.empty() ? h.message : d.words, Label::Kind::Status});
    const double bh = bar.h - 24, by = bar.y + 12;
    double x = bar.x + 20;
    if (d.toggleShown) {
        v.boxes.push_back({"desk:toggle", {x, by, 300, bh}, "Desktop", d.on ? Box::Style::Selected : Box::Style::Bar, true,
                           Icon::Desktop});
        x += 300 + 40;
    } else if (d.dashShown) {
        v.boxes.push_back({"desk:dash", {x, by, 300, bh}, "Dashboard", Box::Style::Bar, true, Icon::Dashboard});
        x += 300 + 40;
    }
    // the monitors: each alone, or all of them side by side
    for (std::uint32_t i = 0; i < d.monitors && i < 4 && d.on; i++) {
        const bool only = d.mask == (1U << i);
        v.boxes.push_back({"desk:mon:" + std::to_string(i), {x, by, 120, bh}, std::to_string(i + 1),
                           only ? Box::Style::Selected : Box::Style::Normal, true});
        x += 120 + kQuickGap;
    }
    if (d.monitors > 1 && d.on) {
        const bool all = d.mask == (1U << d.monitors) - 1;
        v.boxes.push_back({"desk:all", {x, by, 150, bh}, "All", all ? Box::Style::Selected : Box::Style::Normal, true});
    }
    quickActions(v, h, bar.x + bar.w - 20, by, bh, false);
    return v;
}

View layout(Page page, const std::vector<Item>& items, const Header& h) {
    View v;
    const Rect top{kMargin, 20, kW - 2 * kMargin, kTopH};
    const Rect bottom{kMargin, kH - 20 - kBottomH, kW - 2 * kMargin, kBottomH};
    const double midY = top.y + top.h + kGap;
    const double midH = bottom.y - kGap - midY;
    const Rect nav{kMargin, midY, kNavW, midH};
    const Rect body{nav.x + nav.w + kGap, midY, kW - kMargin - (nav.x + nav.w + kGap), midH};
    v.cards = {top, nav, body, bottom};

    // ---- the top bar
    v.labels.push_back({{top.x + 36, top.y, 420, top.h}, "PiBridge", Label::Kind::Title});
    std::string info = h.backend.empty() ? std::string() : h.backend;
    if (!h.runtime.empty() && h.runtime != h.backend) {
        info += (info.empty() ? "" : "  \xC2\xB7  ") + h.runtime;
    }
    info += (info.empty() ? "" : "  \xC2\xB7  ") + (h.room.empty() ? std::string("no saved room") : "room " + h.room);
    v.labels.push_back({{top.x + 480, top.y, 1400, top.h}, info, Label::Kind::Info});
    v.labels.push_back({{top.x + top.w - 336, top.y, 300, top.h}, h.clock, Label::Kind::Clock});

    // ---- the pages
    double y = nav.y + 30;
    for (std::size_t i = 0; i < std::size(kPages); i++) {
        const Page p = kPages[i];
        v.boxes.push_back({"page:" + std::to_string(i), {nav.x + 20, y, nav.w - 40, 110}, pageTitle(p),
                           p == page ? Box::Style::NavSelected : Box::Style::Nav, true});
        y += 110 + kGap;
    }

    // ---- the page's rows
    const double right = body.x + body.w - 40;
    const double labelX = body.x + 40;
    y = body.y + 40;
    // a page with more rows than fit (Recording: eight) gets lower rows, without the timing tags
    int rowCount = 0, textCount = 0;
    for (const Item& it : items) {
        (it.kind == Item::Kind::Text ? textCount : rowCount)++;
    }
    const double fit = rowCount > 0 ? (body.h - 60 - textCount * kTextH) / rowCount : kRowH;
    const double rowH = std::clamp(fit, kCtrlH + 4, kRowH);
    const bool compact = rowH < kRowH;
    for (const Item& it : items) {
        if (it.kind == Item::Kind::Text) {
            v.labels.push_back({{labelX, y, right - labelX, kTextH}, it.value, Label::Kind::Body});
            y += kTextH;
            continue;
        }
        v.labels.push_back({{labelX, compact ? y + (rowH - 70) / 2 : y + 4, kCtrlX - labelX - 30, 70}, it.label,
                            Label::Kind::RowLabel});
        // the timing tag under the label; a choice's own words (why it is off) instead when it has them
        const std::string tag = it.kind == Item::Kind::Choice && !it.value.empty() ? it.value : whenWords(it.when, it.pending);
        if (!tag.empty() && !compact) {
            v.labels.push_back({{labelX, y + 74, kCtrlX - labelX - 30, 50}, tag, it.pending ? Label::Kind::TagPending : Label::Kind::Tag});
        }
        const double cy = compact ? y + (rowH - kCtrlH) / 2 : y + (kRowH - kCtrlH) / 2 - 10;
        switch (it.kind) {
        case Item::Kind::Stepper:
            v.boxes.push_back({it.id + ":-", {kCtrlX, cy, 130, kCtrlH}, "\xE2\x88\x92", Box::Style::Normal, it.enabled});
            v.labels.push_back({{kCtrlX + 150, cy, 420, kCtrlH}, it.value, Label::Kind::Value});
            v.boxes.push_back({it.id + ":+", {kCtrlX + 590, cy, 130, kCtrlH}, "+", Box::Style::Normal, it.enabled});
            break;
        case Item::Kind::Choice: {
            const auto n = static_cast<double>(std::max<std::size_t>(1, it.choices.size()));
            const double w = (right - kCtrlX - kGap * (n - 1)) / n;
            for (std::size_t c = 0; c < it.choices.size(); c++) {
                const bool sel = static_cast<int>(c) == it.chosen;
                v.boxes.push_back({it.id + ":" + std::to_string(c), {kCtrlX + static_cast<double>(c) * (w + kGap), cy, w, kCtrlH},
                                   it.choices[c], sel ? Box::Style::Selected : Box::Style::Normal, it.enabled});
            }
            break;
        }
        case Item::Kind::Button:
            v.boxes.push_back({it.id, {kCtrlX, cy, 520, kCtrlH}, it.label, it.primary ? Box::Style::Primary : Box::Style::Normal,
                               it.enabled});
            v.labels.push_back({{kCtrlX + 550, cy, right - kCtrlX - 550, kCtrlH}, it.value, Label::Kind::Body});
            break;
        case Item::Kind::Text: break;
        }
        y += rowH;
    }

    // ---- the bottom bar
    if (h.overGame) {
        const double bh = bottom.h - 24, by = bottom.y + 12, barRight = bottom.x + bottom.w - 20;
        const double left = quickActions(v, h, barRight, by, bh, true);
        v.labels.push_back({{bottom.x + 36, bottom.y, left - bottom.x - 56, bottom.h}, h.message, Label::Kind::Status});
        return v;
    }
    v.labels.push_back({{bottom.x + 36, bottom.y, 1450, bottom.h}, h.message, Label::Kind::Status});
    v.boxes.push_back({"view:window", {bottom.x + bottom.w - 820, bottom.y + 12, 380, bottom.h - 24}, "PiBridge window",
                       Box::Style::Bar, h.windowAvailable});
    v.boxes.push_back({"view:hide", {bottom.x + bottom.w - 420, bottom.y + 12, 400, bottom.h - 24}, "Hide", Box::Style::Bar, true});
    return v;
}

std::string signature(const View& v) {
    std::string s = std::to_string(v.w) + 'x' + std::to_string(v.h) + '\x1d';
    for (const Box& b : v.boxes) {
        s += b.id + '\x1f' + b.label + '\x1f' + std::to_string(static_cast<int>(b.style)) + (b.enabled ? "1" : "0") + '\x1e';
    }
    for (const Label& l : v.labels) {
        s += l.text + '\x1f' + std::to_string(static_cast<int>(l.kind)) + '\x1f' + std::to_string(static_cast<int>(l.r.y)) + '\x1e';
    }
    return s + '\x1d' + v.hover + '\x1d' + v.pressed;
}

std::optional<std::string> hit(const View& v, double px, double py) {
    for (const Box& b : v.boxes) {
        if (b.enabled && b.r.contains(px, py)) {
            return b.id;
        }
    }
    return std::nullopt;
}

std::optional<Page> pageOf(const std::string& box) {
    if (box.rfind("page:", 0) != 0) {
        return std::nullopt;
    }
    const std::string n = box.substr(5);
    for (std::size_t i = 0; i < std::size(kPages); i++) {
        if (n == std::to_string(i)) {
            return kPages[i];
        }
    }
    return std::nullopt;
}

std::optional<std::string> Clicker::update(const std::optional<std::string>& over, bool pressed) {
    std::optional<std::string> click;
    if (pressed && !was_) {
        downOn_ = over;
    } else if (!pressed && was_) {
        if (downOn_ && over == downOn_) {
            click = downOn_;
        }
        downOn_.reset();
    }
    was_ = pressed;
    return click;
}

} // namespace pibridge::home::dash
