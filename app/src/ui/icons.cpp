// SPDX-License-Identifier: MIT
#include "icons.h"

#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QTransform>

namespace pibridge::ui {

namespace {

// Paths in a 24 x 24 box.
QPainterPath path(Icon kind) {
    QPainterPath p;
    switch (kind) {
    case Icon::Dashboard: // a headset seen from the front
        p.moveTo(4, 9);
        p.cubicTo(4, 7, 5, 6, 7, 6);
        p.lineTo(17, 6);
        p.cubicTo(19, 6, 20, 7, 20, 9);
        p.lineTo(20, 15);
        p.cubicTo(20, 17, 19, 18, 17, 18);
        p.lineTo(15, 18);
        p.lineTo(13.5, 15.5);
        p.lineTo(10.5, 15.5);
        p.lineTo(9, 18);
        p.lineTo(7, 18);
        p.cubicTo(5, 18, 4, 17, 4, 15);
        p.closeSubpath();
        break;
    case Icon::Performance: // a bolt
        p.moveTo(13.5, 3);
        p.lineTo(6, 13.5);
        p.lineTo(11.5, 13.5);
        p.lineTo(10.5, 21);
        p.lineTo(18, 10.5);
        p.lineTo(12.5, 10.5);
        p.closeSubpath();
        break;
    case Icon::Logs: // lines of text in a page
        p.addRoundedRect(QRectF(5, 3.5, 14, 17), 2, 2);
        for (double y : {8.0, 12.0, 16.0}) {
            p.moveTo(8, y);
            p.lineTo(16, y);
        }
        break;
    case Icon::About: // i in a circle
        p.addEllipse(QPointF(12, 12), 8.5, 8.5);
        p.moveTo(12, 11);
        p.lineTo(12, 16.5);
        p.moveTo(12, 7.6);
        p.lineTo(12, 7.9);
        break;
    case Icon::Help: // ? in a circle
        p.addEllipse(QPointF(12, 12), 8.5, 8.5);
        p.moveTo(9.5, 9.5);
        p.cubicTo(9.5, 6.5, 14.5, 6.5, 14.5, 9.5);
        p.cubicTo(14.5, 11.5, 12, 11.5, 12, 13.5);
        p.moveTo(12, 16.3);
        p.lineTo(12, 16.6);
        break;
    case Icon::Play:
        p.moveTo(8, 5.5);
        p.lineTo(18.5, 12);
        p.lineTo(8, 18.5);
        p.closeSubpath();
        break;
    case Icon::Restart: // a circular arrow
        p.arcMoveTo(QRectF(4.5, 4.5, 15, 15), 60);
        p.arcTo(QRectF(4.5, 4.5, 15, 15), 60, 300);
        p.moveTo(15.8, 3.2);
        p.lineTo(15.8, 6.5);
        p.lineTo(12.5, 6.5);
        break;
    case Icon::Power:
        p.arcMoveTo(QRectF(4.5, 5, 15, 15), 60);
        p.arcTo(QRectF(4.5, 5, 15, 15), 60, -300);
        p.moveTo(12, 3);
        p.lineTo(12, 11);
        break;
    case Icon::Eye:
        p.moveTo(3, 12);
        p.cubicTo(6, 6.5, 18, 6.5, 21, 12);
        p.cubicTo(18, 17.5, 6, 17.5, 3, 12);
        p.addEllipse(QPointF(12, 12), 3, 3);
        break;
    case Icon::Sound:
        p.moveTo(4, 9.5);
        p.lineTo(7.5, 9.5);
        p.lineTo(12, 5.5);
        p.lineTo(12, 18.5);
        p.lineTo(7.5, 14.5);
        p.lineTo(4, 14.5);
        p.closeSubpath();
        p.arcMoveTo(QRectF(10, 7, 10, 10), -50);
        p.arcTo(QRectF(10, 7, 10, 10), -50, 100);
        break;
    case Icon::Tracking: // a base station sending waves
        p.addRoundedRect(QRectF(9, 9, 6, 6), 1.5, 1.5);
        p.arcMoveTo(QRectF(5, 5, 14, 14), 135);
        p.arcTo(QRectF(5, 5, 14, 14), 135, 90);
        p.arcMoveTo(QRectF(5, 5, 14, 14), -45);
        p.arcTo(QRectF(5, 5, 14, 14), -45, 90);
        break;
    case Icon::Controller: // a ring controller
        p.addEllipse(QPointF(12, 7.5), 6, 4);
        p.moveTo(10.5, 11.3);
        p.lineTo(10, 20.5);
        p.lineTo(13, 20.5);
        p.lineTo(13.5, 11.3);
        break;
    case Icon::Room: // a floor plan: an outline with a corner cut, and the play centre's arrow
        p.moveTo(4, 5);
        p.lineTo(20, 5);
        p.lineTo(20, 14);
        p.lineTo(15, 19);
        p.lineTo(4, 19);
        p.closeSubpath();
        p.moveTo(11, 15);
        p.lineTo(11, 9);
        p.moveTo(9, 11);
        p.lineTo(11, 9);
        p.lineTo(13, 11);
        break;
    case Icon::Link: // two chain links on a diagonal
    case Icon::Unlink: { // the same links pulled apart
        const double g = kind == Icon::Unlink ? 1.6 : 0.0;
        p.addRoundedRect(QRectF(3 - g, 10 + g, 10, 6), 3, 3);
        p.addRoundedRect(QRectF(11 + g, 8 - g, 10, 6), 3, 3);
        QTransform t;
        t.translate(12, 12);
        t.rotate(-45);
        t.translate(-12, -12);
        p = t.map(p);
        break;
    }
    case Icon::Mic: // a microphone on its stand
        p.addRoundedRect(QRectF(9, 3, 6, 11), 3, 3);
        p.moveTo(6, 11);
        p.cubicTo(6, 18, 18, 18, 18, 11);
        p.moveTo(12, 17);
        p.lineTo(12, 21);
        p.moveTo(9, 21);
        p.lineTo(15, 21);
        break;
    case Icon::Pulse: // a heartbeat line
        p.moveTo(3, 12);
        p.lineTo(7, 12);
        p.lineTo(9.5, 6);
        p.lineTo(13.5, 18);
        p.lineTo(16, 12);
        p.lineTo(21, 12);
        break;
    }
    return p;
}

QPixmap draw(Icon kind, const QColor& c) {
    const int s = 48;
    QPixmap pm(s, s);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.scale(s / 24.0, s / 24.0);
    QPen pen(c, 1.6);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawPath(path(kind));
    return pm;
}

} // namespace

QIcon lineIcon(Icon kind, const QColor& normal, const QColor& checked) {
    QIcon i;
    i.addPixmap(draw(kind, normal), QIcon::Normal, QIcon::Off);
    i.addPixmap(draw(kind, checked), QIcon::Normal, QIcon::On);
    i.addPixmap(draw(kind, normal.darker(170)), QIcon::Disabled, QIcon::Off);
    return i;
}

QIcon lineIcon(Icon kind, const QColor& c) {
    return lineIcon(kind, c, c);
}

namespace {

// The gem in a 24 x 24 box: table (top edge), girdle, the point below, and the facet lines.
QPainterPath gemPath() {
    QPainterPath p;
    p.moveTo(7, 4.5);
    p.lineTo(17, 4.5);
    p.lineTo(21, 9.5);
    p.lineTo(12, 20.5);
    p.lineTo(3, 9.5);
    p.closeSubpath();
    p.moveTo(3, 9.5);
    p.lineTo(21, 9.5);
    p.moveTo(7, 4.5);
    p.lineTo(9.5, 9.5);
    p.lineTo(12, 4.5);
    p.lineTo(14.5, 9.5);
    p.lineTo(17, 4.5);
    p.moveTo(9.5, 9.5);
    p.lineTo(12, 20.5);
    p.lineTo(14.5, 9.5);
    return p;
}

void drawGem(QPainter& p, double box, const QColor& c, double width) {
    p.save();
    p.scale(box / 24.0, box / 24.0);
    QPen pen(c, width);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawPath(gemPath());
    p.restore();
}

} // namespace

QPixmap controllerPixmap(int h, const QColor& c, bool right) {
    const qreal dpr = 2.0;
    const int w = static_cast<int>(h * 0.8);
    QPixmap pm(static_cast<int>(w * dpr), static_cast<int>(h * dpr));
    pm.setDevicePixelRatio(dpr);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    // a 40 x 50 box
    p.scale(w / 40.0, h / 50.0);
    if (right) {
        p.translate(40, 0);
        p.scale(-1, 1);
    }
    QPen pen(c, 1.3);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    // the tracking ring: its outer rim and the inner edge seen from above
    p.drawEllipse(QRectF(6, 3, 28, 17));
    QPainterPath inner;
    inner.moveTo(7.5, 13);
    inner.cubicTo(11, 8.5, 29, 8.5, 32.5, 13);
    p.drawPath(inner);
    // the face under the ring and the grip down to its rounded end
    QPainterPath body;
    body.moveTo(9, 15.5);
    body.cubicTo(10, 21, 13, 23, 14, 26);
    body.cubicTo(15.5, 32, 16, 40, 17.5, 45.5);
    body.cubicTo(18.5, 48, 22.5, 48, 23, 45);
    body.cubicTo(23.5, 38, 23, 30, 25, 24);
    body.cubicTo(27, 21, 30, 19.5, 31.5, 16.5);
    p.drawPath(body);
    // the face's front edge
    QPainterPath face;
    face.moveTo(10, 17.5);
    face.cubicTo(15, 21.5, 25, 21.5, 30.5, 17.5);
    p.drawPath(face);
    // the stick and two buttons
    p.drawEllipse(QPointF(16, 15.5), 2.2, 1.5);
    p.drawEllipse(QPointF(22, 16), 1.1, 0.8);
    p.drawEllipse(QPointF(25.5, 15), 1.1, 0.8);
    // the grip's seam and the trigger on its front
    QPainterPath seam;
    seam.moveTo(17.5, 24);
    seam.cubicTo(18, 32, 19, 40, 20.5, 47);
    p.drawPath(seam);
    p.setBrush(c);
    p.drawRoundedRect(QRectF(25.2, 27, 2.2, 5.5), 1, 1);
    return pm;
}

QPixmap gemPixmap(int px, const QColor& c) {
    const qreal dpr = 2.0;
    QPixmap pm(static_cast<int>(px * dpr), static_cast<int>(px * dpr));
    pm.setDevicePixelRatio(dpr);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    drawGem(p, px, c, 1.6);
    return pm;
}

// The app icon: the pibridgeed gem in indigo on a dark rounded square (app/data/pibridge-crystal.svg), drawn here so it is
// crisp at every size and needs no pre-rendered PNGs. Rendered at each icon size.
QIcon appIcon() {
    QIcon icon;
    for (const int s : {16, 24, 32, 48, 64, 128, 256, 512}) {
        const qreal dpr = 2.0;
        QPixmap pm(static_cast<int>(s * dpr), static_cast<int>(s * dpr));
        pm.setDevicePixelRatio(dpr);
        pm.fill(Qt::transparent);
        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing);
        p.scale(s / 64.0, s / 64.0); // the 64 x 64 viewBox of pibridge-crystal.svg
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0x14, 0x14, 0x17));
        p.drawRoundedRect(QRectF(2, 2, 60, 60), 14, 14);
        p.translate(8, 8);
        p.scale(2, 2); // the gem is the 24-box gemPath() placed as in pibridge-crystal.svg
        QPen pen(QColor(0x7a, 0x88, 0xff), 1.7);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        p.drawPath(gemPath());
        p.end();
        icon.addPixmap(pm);
    }
    return icon;
}

QPixmap logoPixmap(int px) {
    // the sidebar mark: the faceted gem outline in the app's indigo (same drawing as appIcon(), no PNG)
    return gemPixmap(px, QColor(0x7a, 0x88, 0xff));
}

} // namespace pibridge::ui
