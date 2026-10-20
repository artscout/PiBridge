// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux gui/src/preview.cpp (MIT); colours adapted to the app's theme.
#include "preview.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace qvgui {

namespace {
double tanDeg(double d) {
    return std::tan(d * std::numbers::pi / 180.0);
}
} // namespace

Preview::Preview(QWidget* parent) : QWidget(parent) {
    field_ = {float(-tanDeg(52)), float(tanDeg(43)), float(-tanDeg(46)), float(tanDeg(44))};
    setMinimumSize(300, 260);
    setMouseTracking(false);
    setToolTip(tr("Drag to move the gaze"));
}

void Preview::setSettings(const qv::Settings& s) {
    s_ = s;
    update();
}

QRectF Preview::fieldRect() const {
    const double fw = field_.right - field_.left, fh = field_.up - field_.down;
    const QRectF area = rect().adjusted(12, 12, -12, -34);
    const double k = std::min(area.width() / fw, area.height() / fh);
    const QSizeF sz(fw * k, fh * k);
    return QRectF(area.center() - QPointF(sz.width() / 2, sz.height() / 2), sz);
}

QPointF Preview::toWidget(double tx, double ty) const {
    const QRectF r = fieldRect();
    return {r.left() + (tx - field_.left) / (field_.right - field_.left) * r.width(),
            r.top() + (field_.up - ty) / (field_.up - field_.down) * r.height()};
}

QPointF Preview::toTangent(QPointF p) const {
    const QRectF r = fieldRect();
    const double tx = field_.left + (p.x() - r.left()) / r.width() * (field_.right - field_.left);
    const double ty = field_.up - (p.y() - r.top()) / r.height() * (field_.up - field_.down);
    return {std::clamp(tx, double(field_.left), double(field_.right)),
            std::clamp(ty, double(field_.down), double(field_.up))};
}

void Preview::drawFocus(QPainter& p, const qv::Tan& w, const QColor& c, const QString& label) const {
    const QRectF outer(toWidget(w.left, w.up), toWidget(w.right, w.down));
    // The blend band: from the edge to `transition` of the size inside.
    const double t = s_.transition;
    const QRectF inner = outer.adjusted(outer.width() * t, outer.height() * t, -outer.width() * t, -outer.height() * t);
    QColor fill = c;
    fill.setAlpha(55);
    p.fillRect(inner.isValid() ? inner : outer, fill);
    if (t > 0 && inner.isValid()) {
        QPainterPath band;
        band.addRect(outer);
        band.addRect(inner);
        QColor bf = c;
        bf.setAlpha(25);
        p.fillPath(band, bf);
        p.setPen(QPen(c, 1, Qt::DashLine));
        p.drawRect(inner);
    }
    p.setPen(QPen(c, 2));
    p.drawRect(outer);
    p.setPen(c);
    p.drawText(outer.topLeft() + QPointF(4, 14), label);
}

void Preview::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF f = fieldRect();
    const QPalette& pal = palette();
    p.fillRect(rect(), pal.window());

    // The wide view, with a grid as coarse as its pixels are (one line per 40 of its pixels at 1000 recommended).
    p.fillRect(f, QColor(22, 26, 52));
    const double step = std::max(4.0, f.width() / (1000.0 * s_.peripheralDensity / 40.0));
    p.setPen(QColor(40, 46, 92));
    for (double x = f.left() + step; x < f.right(); x += step) p.drawLine(QPointF(x, f.top()), QPointF(x, f.bottom()));
    for (double y = f.top() + step; y < f.bottom(); y += step) p.drawLine(QPointF(f.left(), y), QPointF(f.right(), y));
    p.setPen(QPen(QColor(90, 100, 170), 1));
    p.drawRect(f);
    // The centre of the field (straight ahead).
    const QPointF c0 = toWidget(0, 0);
    p.drawLine(c0 + QPointF(-6, 0), c0 + QPointF(6, 0));
    p.drawLine(c0 + QPointF(0, -6), c0 + QPointF(0, 6));

    const float ox = s_.offsetDeg[0], oy = s_.offsetDeg[1];
    const qv::Tan fixedWin = qv::focusWindow(field_, qv::addAngle(0.f, ox), qv::addAngle(0.f, oy), s_.fixedSize[0],
                                             s_.fixedSize[1]);
    drawFocus(p, fixedWin, QColor(80, 200, 130), tr("fixed"));
    if (s_.tracking == qv::Tracking::Eye) {
        const qv::Tan gazeWin = qv::focusWindow(field_, qv::addAngle(float(gaze_.x()), ox),
                                                qv::addAngle(float(gaze_.y()), oy), s_.gazeSize[0], s_.gazeSize[1]);
        drawFocus(p, gazeWin, QColor(122, 136, 255), tr("gaze"));
        const QPointF g = toWidget(gaze_.x(), gaze_.y());
        p.setBrush(QColor(240, 242, 255));
        p.setPen(Qt::NoPen);
        p.drawEllipse(g, 5, 5);
        p.setBrush(Qt::NoBrush);
    }
    p.setPen(pal.color(QPalette::WindowText));
    p.drawText(QRectF(0, height() - 26, width(), 22), Qt::AlignCenter,
               s_.tracking == qv::Tracking::Eye ? tr("Left eye. Drag to move the gaze (white dot).")
                                                : tr("Left eye. The focus stays fixed."));
}

void Preview::mousePressEvent(QMouseEvent* e) {
    gaze_ = toTangent(e->position());
    update();
}

void Preview::mouseMoveEvent(QMouseEvent* e) {
    if (e->buttons() & Qt::LeftButton) {
        gaze_ = toTangent(e->position());
        update();
    }
}

} // namespace qvgui
