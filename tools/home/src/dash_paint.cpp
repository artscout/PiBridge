// SPDX-License-Identifier: MIT
#include "dash_paint.h"

#include <QColor>
#include <QFont>
#include <QFontMetricsF>
#include <QGuiApplication>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QRectF>
#include <QString>

#include <algorithm>
#include <cstring>
#include <memory>

namespace pibridge::home::dash {

namespace {

// PiBridge's VR palette: deep blue-grey cards, a cool blue accent, amber for a change waiting for a restart
const QColor kBack(10, 13, 20, 228);
const QColor kCard(22, 28, 40, 238);
const QColor kEdge(70, 92, 124, 150);
const QColor kAccent(76, 156, 236);
const QColor kAccentHi(120, 186, 250);
const QColor kAmber(236, 172, 70);
const QColor kText(234, 240, 248);
const QColor kSub(150, 164, 186);
const QColor kDim(92, 102, 120);
const QColor kChip(36, 44, 60, 245);
const QColor kChipHi(52, 64, 86, 250);

void ensureApp() {
    if (QGuiApplication::instance() != nullptr) {
        return;
    }
    qputenv("QT_QPA_PLATFORM", "offscreen");
    static int argc = 1;
    static char arg0[] = "pibridge-home";
    static char* argv[] = {arg0, nullptr};
    static std::unique_ptr<QGuiApplication> app = std::make_unique<QGuiApplication>(argc, argv);
    (void)app;
}

QFont sans(double px, bool bold) {
    QFont f(QStringLiteral("DejaVu Sans"));
    f.setPixelSize(std::max(1, static_cast<int>(px)));
    f.setBold(bold);
    return f;
}

QRectF qr(const Rect& r) { return {r.x, r.y, r.w, r.h}; }
QString qs(const std::string& s) { return QString::fromUtf8(s.data(), static_cast<qsizetype>(s.size())); }

// one line, shrunk to fit (down to 70 %), then elided
void line(QPainter& p, const QRectF& r, const QString& s, double px, bool bold, const QColor& c, Qt::Alignment al) {
    double fp = px;
    const double w = QFontMetricsF(sans(fp, bold)).horizontalAdvance(s);
    if (w > r.width() && w > 1) {
        fp = std::max(px * 0.7, fp * r.width() / w);
    }
    const QFont f = sans(fp, bold);
    p.setFont(f);
    p.setPen(c);
    p.drawText(r, al | Qt::AlignVCenter, QFontMetricsF(f).elidedText(s, Qt::ElideRight, r.width()));
}

void chip(QPainter& p, const Box& b, bool hover, bool pressed) {
    const QRectF r = qr(b.r);
    const double rad = std::min(r.height() * 0.3, 30.0);
    QPainterPath path;
    path.addRoundedRect(r, rad, rad);
    QColor fill = kChip, edge = kEdge, text = kText;
    switch (b.style) {
    case Box::Style::Primary:
        fill = hover ? kAccentHi : kAccent;
        edge = kAccentHi;
        break;
    case Box::Style::Selected:
        fill = QColor(40, 84, 132, 250);
        edge = kAccentHi;
        break;
    case Box::Style::Nav:
        fill = hover ? kChipHi : QColor(0, 0, 0, 0);
        edge = QColor(0, 0, 0, 0);
        text = kSub;
        break;
    case Box::Style::NavSelected:
        fill = QColor(40, 84, 132, 250);
        edge = QColor(0, 0, 0, 0);
        break;
    case Box::Style::Bar:
    case Box::Style::Normal:
        fill = hover ? kChipHi : kChip;
        edge = hover ? kAccent : kEdge;
        break;
    }
    if (!b.enabled) {
        fill = QColor(26, 31, 42, 200);
        edge = QColor(48, 56, 70, 160);
        text = kDim;
    } else if (pressed) {
        fill = fill.darker(130);
    }
    p.fillPath(path, fill);
    if (edge.alpha() > 0) {
        p.setPen(QPen(edge, hover && b.enabled ? 4.0 : 2.5));
        p.drawPath(path);
    }
    if (b.style == Box::Style::NavSelected) {
        // the accent strip of the page that is open
        p.fillRect(QRectF(r.x() + 6, r.y() + r.height() * 0.2, 8, r.height() * 0.6), kAccentHi);
    }
    const bool nav = b.style == Box::Style::Nav || b.style == Box::Style::NavSelected;
    const QRectF tr = nav ? r.adjusted(44, 0, -16, 0) : r.adjusted(16, 0, -16, 0);
    line(p, tr, qs(b.label), b.label.size() <= 3 ? 64 : 42, true, text, nav ? Qt::AlignLeft : Qt::AlignHCenter);
}

} // namespace

struct Painter::Impl {
    QImage img;
};

Painter::Painter() : d_(new Impl) { ensureApp(); }
Painter::~Painter() { delete d_; }

const std::uint8_t* Painter::paint(const View& v, int& stride) {
    ensureApp();
    if (d_->img.width() != kW || d_->img.height() != kH) {
        d_->img = QImage(kW, kH, QImage::Format_ARGB32); // B, G, R, A bytes on little-endian
    }
    d_->img.fill(Qt::transparent);
    QPainter p(&d_->img);
    p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

    // the whole panel, then the bars and columns
    QPainterPath all;
    all.addRoundedRect(QRectF(4, 4, kW - 8, kH - 8), 40, 40);
    p.fillPath(all, kBack);
    p.setPen(QPen(kEdge, 3));
    p.drawPath(all);
    for (const Rect& c : v.cards) {
        QPainterPath cp;
        cp.addRoundedRect(qr(c), 26, 26);
        p.fillPath(cp, kCard);
    }

    for (const Label& l : v.labels) {
        const QRectF r = qr(l.r);
        const QString s = qs(l.text);
        switch (l.kind) {
        case Label::Kind::Title: line(p, r, s, 60, true, kText, Qt::AlignLeft); break;
        case Label::Kind::Info: line(p, r, s, 38, false, kSub, Qt::AlignHCenter); break;
        case Label::Kind::Clock: line(p, r, s, 48, true, kText, Qt::AlignRight); break;
        case Label::Kind::RowLabel: line(p, r, s, 50, true, kText, Qt::AlignLeft); break;
        case Label::Kind::Value: line(p, r, s, 54, true, kText, Qt::AlignHCenter); break;
        case Label::Kind::Tag: line(p, r, s, 28, true, kSub, Qt::AlignLeft); break;
        case Label::Kind::TagPending: line(p, r, s, 28, true, kAmber, Qt::AlignLeft); break;
        case Label::Kind::Status: line(p, r, s, 36, false, kText, Qt::AlignLeft); break;
        case Label::Kind::Body: {
            // wrapped, shrunk until it fits the rect
            double fp = 38;
            for (int i = 0; i < 8; i++) {
                const QRectF need = QFontMetricsF(sans(fp, false)).boundingRect(r, Qt::TextWordWrap, s);
                if (need.height() <= r.height()) {
                    break;
                }
                fp *= 0.9;
            }
            p.setFont(sans(fp, false));
            p.setPen(kSub);
            p.drawText(r, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap, s);
            break;
        }
        }
    }
    for (const Box& b : v.boxes) {
        chip(p, b, b.id == v.hover, b.id == v.pressed);
    }
    p.end();
    stride = static_cast<int>(d_->img.bytesPerLine());
    return d_->img.constBits();
}

std::vector<std::uint8_t> Painter::picture(const View& v) {
    int stride = 0;
    const std::uint8_t* px = paint(v, stride);
    std::vector<std::uint8_t> out(static_cast<std::size_t>(kW) * kH * 4);
    for (int y = 0; y < kH; y++) {
        std::memcpy(&out[static_cast<std::size_t>(y) * kW * 4], px + static_cast<std::size_t>(y) * stride, static_cast<std::size_t>(kW) * 4);
    }
    return out;
}

} // namespace pibridge::home::dash
