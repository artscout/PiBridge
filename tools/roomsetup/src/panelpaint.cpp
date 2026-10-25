// SPDX-License-Identifier: MIT
#include "panelpaint.h"

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
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>

namespace pibridge::roomsetup::panel {

namespace {

// Pimax-ish palette (tunable). Green accent to sit next to PVRGuardian's own art when it is skinned on top.
const QColor kCardBg(18, 23, 27, 236);
const QColor kCardEdge(64, 196, 112, 120);
const QColor kAccent(62, 196, 112);
const QColor kAccentHi(104, 226, 142);
const QColor kText(236, 244, 240);
const QColor kSub(176, 190, 184);
const QColor kChipBg(38, 46, 50, 235);

// Ensure there is a QGuiApplication (offscreen) so the font database works in this headless process.
void ensureApp() {
    if (QGuiApplication::instance() != nullptr) {
        return;
    }
    qputenv("QT_QPA_PLATFORM", "offscreen");
    static int argc = 1;
    static char arg0[] = "pibridge-roomsetup";
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

// a rounded chip in the accent style (procedural fallback)
void drawChip(QPainter& p, const QRectF& r, const QString& label, bool hover, bool primary, double fontPx) {
    const double rad = r.height() * 0.28;
    QPainterPath path;
    path.addRoundedRect(r, rad, rad);
    if (primary) {
        p.fillPath(path, hover ? kAccentHi : kAccent);
        p.setPen(QPen(kText));
    } else {
        p.fillPath(path, hover ? QColor(54, 66, 70, 240) : kChipBg);
        p.setPen(QPen(hover ? kAccentHi : kAccent, std::max(1.0, r.height() * 0.045)));
        p.drawPath(path);
        p.setPen(QPen(hover ? kText : kSub));
    }
    double fp = fontPx;
    const double lw = QFontMetricsF(sans(fp, true)).horizontalAdvance(label);
    if (lw > r.width() * 0.86 && lw > 1) {
        fp *= r.width() * 0.86 / lw;
    }
    p.setFont(sans(fp, true));
    p.drawText(r, Qt::AlignCenter, label);
}

// a faint hex band (the "scanning" accent) across the top of the card
void drawScanHex(QPainter& p, const QRectF& card) {
    p.save();
    p.setClipRect(card);
    const double s = card.height() * 0.055; // hex radius
    const double dx = s * 1.8, dy = s * 1.56;
    QColor c = kAccent;
    c.setAlpha(46);
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    int row = 0;
    for (double y = card.top() + s; y < card.top() + card.height() * 0.42; y += dy, ++row) {
        const double x0 = card.left() + (row % 2 ? dx * 0.5 : 0) + s;
        for (double x = x0; x < card.right(); x += dx) {
            QPainterPath h;
            for (int i = 0; i < 6; ++i) {
                const double a = M_PI / 180.0 * (60 * i - 30);
                const QPointF pt(x + s * 0.82 * std::cos(a), y + s * 0.82 * std::sin(a));
                if (i == 0) {
                    h.moveTo(pt);
                } else {
                    h.lineTo(pt);
                }
            }
            h.closeSubpath();
            p.drawPath(h);
        }
    }
    p.restore();
}

} // namespace

struct Painter::Impl {
    QImage img;
    std::string assetDir;
    QImage btnNormal, btnHover, btnPrimary;
    bool skinTried = false;

    void loadSkin() {
        skinTried = true;
        if (assetDir.empty()) {
            return;
        }
        namespace fs = std::filesystem;
        auto load = [&](const char* name) {
            const auto p = fs::path(assetDir) / name;
            QImage px;
            if (fs::exists(p)) {
                px.load(QString::fromStdString(p.string()));
            }
            return px;
        };
        btnNormal = load("btn_normal.png");
        btnHover = load("btn_hover.png");
        btnPrimary = load("btn_normal.png"); // the green one is the primary; normal chips drawn procedurally
    }
};

Painter::Painter() : d_(new Impl) { ensureApp(); }
Painter::~Painter() { delete d_; }

void Painter::setAssetDir(const std::string& dir) {
    d_->assetDir = dir;
    d_->skinTried = false;
}

const std::uint8_t* Painter::paint(const Content& c, int pxW, int pxH, int& strideBytes) {
    ensureApp();
    if (pxW < 2 || pxH < 2) {
        strideBytes = 0;
        return nullptr;
    }
    if (d_->img.width() != pxW || d_->img.height() != pxH) {
        d_->img = QImage(pxW, pxH, QImage::Format_ARGB32);
    }
    if (!d_->skinTried) {
        d_->loadSkin();
    }
    d_->img.fill(Qt::transparent);

    QPainter p(&d_->img);
    p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);

    const double m = pxW * 0.015; // outer margin
    const QRectF card(m, m, pxW - 2 * m, pxH - 2 * m);
    const double rad = card.height() * 0.10;

    // the card
    QPainterPath cardPath;
    cardPath.addRoundedRect(card, rad, rad);
    p.fillPath(cardPath, kCardBg);
    if (c.scanning) {
        drawScanHex(p, card);
    }
    p.setPen(QPen(kCardEdge, std::max(1.0, pxH * 0.004)));
    p.drawPath(cardPath);

    const double pad = card.width() * 0.055;
    double y = card.top() + pad;

    const double avail = card.width() - 2 * pad;
    auto fitFont = [&](double px, bool bold, const QString& s) {
        if (s.isEmpty()) {
            return sans(px, bold);
        }
        const double w = QFontMetricsF(sans(px, bold)).horizontalAdvance(s);
        if (w > avail && w > 1) {
            px *= avail / w;
        }
        return sans(px, bold);
    };

    // header
    if (!c.title.empty()) {
        const double tpx = pxH * 0.095;
        p.setFont(fitFont(tpx, true, QString::fromStdString(c.title)));
        p.setPen(kText);
        const QRectF hr(card.left() + pad, y, card.width() - 2 * pad, tpx * 1.3);
        p.drawText(hr, Qt::AlignHCenter | Qt::AlignVCenter, QString::fromStdString(c.title));
        y = hr.bottom() + pxH * 0.012;
        // accent underline
        p.setPen(QPen(kAccent, std::max(1.0, pxH * 0.006)));
        const double uw = card.width() * 0.16;
        p.drawLine(QPointF(card.center().x() - uw, y), QPointF(card.center().x() + uw, y));
        y += pxH * 0.03;
    }

    // body lines: one font size, shrunk so the widest line fits the card width
    double lpx = pxH * 0.058;
    {
        double maxw = 0;
        for (const auto& line : c.lines) {
            maxw = std::max(maxw, QFontMetricsF(sans(lpx, false)).horizontalAdvance(QString::fromStdString(line)));
        }
        if (maxw > avail && maxw > 1) {
            lpx *= avail / maxw;
        }
    }
    p.setFont(sans(lpx, false));
    for (const auto& line : c.lines) {
        p.setPen(kText);
        const QRectF lr(card.left() + pad, y, card.width() - 2 * pad, lpx * 1.35);
        p.drawText(lr, Qt::AlignHCenter | Qt::AlignVCenter, QString::fromStdString(line));
        y = lr.bottom();
    }

    // buttons (UV rects 0..1, y down, over the whole image)
    const double chipFont = pxH * 0.05;
    for (const auto& b : c.buttons) {
        const QRectF r(b.x * pxW, b.y * pxH, b.w * pxW, b.h * pxH);
        const bool skin = !d_->btnNormal.isNull() || (b.primary && !d_->btnPrimary.isNull());
        if (skin) {
            const QImage& px = b.primary ? d_->btnPrimary : (b.hover && !d_->btnHover.isNull() ? d_->btnHover : d_->btnNormal);
            if (!px.isNull()) {
                p.setOpacity(b.hover ? 1.0 : 0.92);
                p.drawImage(r, px, px.rect());
                p.setOpacity(1.0);
                p.setFont(sans(chipFont, true));
                p.setPen(kText);
                p.drawText(r, Qt::AlignCenter, QString::fromStdString(b.label));
                continue;
            }
        }
        drawChip(p, r, QString::fromStdString(b.label), b.hover, b.primary, chipFont);
    }

    p.end();
    strideBytes = static_cast<int>(d_->img.bytesPerLine());
    return d_->img.constBits();
}

bool Painter::savePng(const Content& c, int pxW, int pxH, const std::string& path) {
    int stride = 0;
    if (paint(c, pxW, pxH, stride) == nullptr) {
        return false;
    }
    return d_->img.save(QString::fromStdString(path), "PNG");
}

} // namespace pibridge::roomsetup::panel
