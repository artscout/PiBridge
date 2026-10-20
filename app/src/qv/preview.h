// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux gui/src/preview.h (MIT).
// One eye's field with the focus areas drawn to scale (tangent space, as the layer computes them): the fixed area,
// the area that follows the gaze (drag the dot), and their blend bands.
#pragma once

#include <QPointF>
#include <QWidget>

#include "fovmath.h"
#include "settings.h"

namespace qvgui {

class Preview : public QWidget {
    Q_OBJECT
  public:
    explicit Preview(QWidget* parent = nullptr);
    void setSettings(const qv::Settings& s);
    QSize sizeHint() const override { return {420, 380}; }

  protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;

  private:
    QRectF fieldRect() const;
    QPointF toWidget(double tx, double ty) const;
    QPointF toTangent(QPointF p) const;
    void drawFocus(class QPainter& p, const qv::Tan& win, const QColor& c, const QString& label) const;

    qv::Settings s_;
    qv::Tan field_; // a typical left eye: 52 degrees out, 43 in, 44 up, 46 down
    QPointF gaze_{0.25, 0.1};
};

} // namespace qvgui
