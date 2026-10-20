// SPDX-License-Identifier: MIT
#include "widgets.h"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSpinBox>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QToolTip>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include "theme.h"

namespace pibridge::ui {

QLabel* capsLabel(const QString& text, int px, QWidget* parent) {
    auto* l = new QLabel(text, parent);
    l->setFont(labelFont(px));
    return l;
}

QLabel* dimLabel(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text, parent);
    l->setObjectName(QStringLiteral("dim"));
    l->setWordWrap(true);
    return l;
}

QPushButton* toolButton(const QString& text, QWidget* parent) {
    auto* b = new QPushButton(text, parent);
    b->setObjectName(QStringLiteral("tool"));
    b->setFont(labelFont(13));
    b->setCursor(Qt::PointingHandCursor);
    return b;
}

QPushButton* accentButton(const QString& text, QWidget* parent) {
    auto* b = toolButton(text, parent);
    b->setObjectName(QStringLiteral("accent"));
    return b;
}

QPushButton* linkButton(const QString& text, QWidget* parent) {
    auto* b = toolButton(text, parent);
    b->setObjectName(QStringLiteral("link"));
    return b;
}

QFrame* rule(QWidget* parent) {
    auto* f = new QFrame(parent);
    f->setObjectName(QStringLiteral("rule"));
    f->setFrameShape(QFrame::NoFrame);
    return f;
}

// ---- Toggle

Toggle::Toggle(QWidget* parent) : QAbstractButton(parent) {
    setCheckable(true);
    setCursor(Qt::PointingHandCursor);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void Toggle::setCheckedQuiet(bool on) {
    const bool was = blockSignals(true);
    setChecked(on);
    blockSignals(was);
}

void Toggle::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF track = QRectF(rect()).adjusted(1.5, 1.5, -1.5, -1.5);
    const double r = track.height() / 2;
    const bool on = isChecked();
    const bool enabled = isEnabled();
    p.setPen(QPen(on ? color::accent : color::borderStrong, 1.5));
    p.setBrush(on ? color::accentDark : QColor(0x10, 0x10, 0x13));
    p.drawRoundedRect(track, r, r);
    const double kr = r - 4;
    const QPointF c(on ? track.right() - r : track.left() + r, track.center().y());
    QColor knob = on ? color::accent : QColor(0x55, 0x55, 0x5f);
    if (!enabled) {
        knob = knob.darker(160);
    }
    if (on) {
        p.setPen(QPen(QColor(0xa8, 0xb2, 0xff), 1.5));
    } else {
        p.setPen(Qt::NoPen);
    }
    p.setBrush(knob);
    p.drawEllipse(c, kr, kr);
}

// ---- Chip

Chip::Chip(QWidget* parent) : QWidget(parent) {
    setFont(labelFont(12));
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void Chip::set(const QString& text, pibridge::app::ChipKind kind) {
    if (text == text_ && kind == kind_) {
        return;
    }
    text_ = text;
    kind_ = kind;
    setVisible(!text.isEmpty());
    updateGeometry();
    update();
}

QSize Chip::sizeHint() const {
    const QFontMetrics fm(font());
    return {fm.horizontalAdvance(text_) + 30, fm.height() + 8};
}

void Chip::paintEvent(QPaintEvent*) {
    QColor c;
    switch (kind_) {
    case pibridge::app::ChipKind::Info: c = color::accentText; break;
    case pibridge::app::ChipKind::Warn: c = color::amber; break;
    case pibridge::app::ChipKind::Error: c = color::red; break;
    case pibridge::app::ChipKind::Off: c = color::dim; break;
    }
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QPen(kind_ == pibridge::app::ChipKind::Info ? color::accent : c, 1));
    p.setBrush(kind_ == pibridge::app::ChipKind::Info ? QColor(0x1a, 0x1f, 0x48) : QColor(0, 0, 0, 0));
    p.drawRoundedRect(r, 2, 2);
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    p.drawEllipse(QPointF(11, r.center().y()), 2.5, 2.5);
    p.setPen(c);
    p.setFont(font());
    p.drawText(r.adjusted(19, 0, -4, 0), Qt::AlignVCenter | Qt::AlignLeft, text_);
}

// ---- RestartDot

RestartDot::RestartDot(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setToolTip(QStringLiteral("Restart the game for this setting to take effect."));
}

void RestartDot::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(color::yellowDot);
    p.drawEllipse(QRectF(rect()).center(), 3.5, 3.5);
}

// ---- HelpBadge

HelpBadge::HelpBadge(const QString& text, QWidget* parent) : QWidget(parent), text_(text) {
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setCursor(Qt::PointingHandCursor); // a plain pointer; the "?" WhatsThis cursor was mistaken for the hint itself
    // Wrap as rich text so a long hint word-wraps in the tooltip instead of one very wide line.
    setToolTip(QStringLiteral("<qt>") + text.toHtmlEscaped() + QStringLiteral("</qt>"));
}

void HelpBadge::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(1.5, 1.5, -1.5, -1.5);
    p.setPen(QPen(color::borderStrong, 1.2));
    p.setBrush(QColor(0x1c, 0x1c, 0x21));
    p.drawEllipse(r);
    QFont f = bodyFont(11);
    f.setWeight(QFont::DemiBold);
    p.setFont(f);
    p.setPen(color::accentText);
    p.drawText(rect(), Qt::AlignCenter, QStringLiteral("?"));
}

void HelpBadge::mousePressEvent(QMouseEvent* e) {
    // Clicking shows the hint too, so it also works read off a screen or in the headset (not only on hover).
    QToolTip::showText(e->globalPosition().toPoint(), toolTip(), this);
}

QWidget* helpBadge(const QString& text) {
    return new HelpBadge(text);
}

// ---- Card

Card::Card(const QString& title, QWidget* parent) : QFrame(parent) {
    setObjectName(QStringLiteral("card"));
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(16, 12, 16, 14);
    v->setSpacing(10);
    header_ = new QHBoxLayout;
    header_->setSpacing(10);
    title_ = new QLabel(title);
    title_->setFont(titleFont(23));
    chip_ = new Chip;
    chip_->hide();
    header_->addWidget(title_);
    header_->addStretch(1);
    header_->addWidget(chip_, 0, Qt::AlignTop);
    v->addLayout(header_);
    body_ = new QVBoxLayout;
    body_->setSpacing(8);
    v->addLayout(body_, 1);
}

// ---- Segmented

Segmented::Segmented(const QStringList& items, QWidget* parent) : QWidget(parent) {
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(0);
    group_ = new QButtonGroup(this);
    group_->setExclusive(false); // so -1 (none) is possible; exclusivity is done by hand
    for (int i = 0; i < items.size(); i++) {
        auto* b = new QPushButton(items[i]);
        b->setObjectName(QStringLiteral("seg"));
        b->setProperty("first", i == 0);
        b->setCheckable(true);
        b->setFont(labelFont(13));
        b->setCursor(Qt::PointingHandCursor);
        group_->addButton(b, i);
        buttons_.push_back(b);
        h->addWidget(b);
    }
    h->addStretch(1);
    connect(group_, &QButtonGroup::idClicked, this, [this](int id) {
        setCurrent(id);
        emit activated(id);
    });
}

void Segmented::setCurrent(int index) {
    for (size_t i = 0; i < buttons_.size(); i++) {
        buttons_[i]->setChecked(static_cast<int>(i) == index);
    }
}

int Segmented::current() const {
    for (size_t i = 0; i < buttons_.size(); i++) {
        if (buttons_[i]->isChecked()) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void Segmented::setItemEnabled(int index, bool on) {
    if (index >= 0 && index < static_cast<int>(buttons_.size())) {
        buttons_[static_cast<size_t>(index)]->setEnabled(on);
    }
}

// ---- TickSlider

TickSlider::TickSlider(QWidget* parent) : QSlider(Qt::Horizontal, parent) {
    setCursor(Qt::PointingHandCursor);
    setMinimumWidth(160);
}

void TickSlider::paintEvent(QPaintEvent*) {
    QPainter p(this);
    const QRectF r = QRectF(rect()).adjusted(4, 3, -4, -3);
    const int span = maximum() - minimum();
    const double frac = span > 0 ? double(value() - minimum()) / span : 0.0;
    const double xv = r.left() + frac * r.width();
    const double step = 3.0;
    const QColor on = isEnabled() ? color::accent : color::faint;
    for (double x = r.left(); x <= r.right(); x += step) {
        if (x <= xv) {
            p.setPen(QPen(on, 1));
            p.drawLine(QPointF(x + 0.5, r.top() + 2), QPointF(x + 0.5, r.bottom() - 2));
        } else {
            p.setPen(QPen(QColor(0x4a, 0x4a, 0x54), 1));
            p.drawLine(QPointF(x + 0.5, r.center().y() - 1), QPointF(x + 0.5, r.center().y() + 1));
        }
    }
    // the handle: a short white bar
    p.setPen(Qt::NoPen);
    p.setBrush(isEnabled() ? QColor(0xf2, 0xf2, 0xf6) : color::dim);
    p.drawRect(QRectF(xv - 1.5, r.top(), 3, r.height()));
}

// ---- SliderRow

SliderRow::SliderRow(const QString& label, int min, int max, const QString& unit, bool restartDot,
                     const QString& tip, QWidget* parent)
    : QWidget(parent) {
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(2);
    auto* top = new QHBoxLayout;
    top->setSpacing(6);
    top->addWidget(capsLabel(label, 13));
    if (restartDot) {
        top->addWidget(new RestartDot);
    }
    if (!tip.isEmpty()) {
        top->addWidget(new HelpBadge(tip)); // a visible "?" hint next to the label
    }
    top->addStretch(1);
    v->addLayout(top);
    auto* row = new QHBoxLayout;
    row->setSpacing(6);
    auto* minus = new QPushButton(QStringLiteral("−"));
    auto* plus = new QPushButton(QStringLiteral("+"));
    for (auto* b : {minus, plus}) {
        b->setObjectName(QStringLiteral("step"));
        b->setFont(bodyFont(16));
        b->setCursor(Qt::PointingHandCursor);
        b->setAutoRepeat(true);
    }
    slider_ = new TickSlider;
    slider_->setRange(min, max);
    box_ = new QSpinBox;
    box_->setObjectName(QStringLiteral("valuebox"));
    box_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    box_->setRange(min, max);
    box_->setAlignment(Qt::AlignCenter);
    box_->setFixedWidth(52);
    auto* u = new QLabel(unit);
    u->setFixedWidth(22);
    row->addWidget(minus);
    row->addWidget(slider_, 1);
    row->addWidget(plus);
    row->addWidget(box_);
    row->addWidget(u);
    v->addLayout(row);
    setToolTip(tip);

    connect(minus, &QPushButton::clicked, this, [this] { slider_->setValue(slider_->value() - 1); });
    connect(plus, &QPushButton::clicked, this, [this] { slider_->setValue(slider_->value() + 1); });
    connect(slider_, &QSlider::valueChanged, this, [this](int x) {
        const bool was = box_->blockSignals(true);
        box_->setValue(x);
        box_->blockSignals(was);
        if (!quiet_) {
            emit valueChanged(x);
        }
    });
    connect(box_, &QSpinBox::valueChanged, this, [this](int x) { slider_->setValue(x); });
}

int SliderRow::value() const {
    return slider_->value();
}

void SliderRow::setValueQuiet(int x) {
    quiet_ = true;
    slider_->setValue(x);
    const bool was = box_->blockSignals(true);
    box_->setValue(slider_->value());
    box_->blockSignals(was);
    quiet_ = false;
}

QWidget* settingRow(const QString& label, QWidget* right, QWidget* extra, bool restartDot, const QString& tip) {
    auto* w = new QWidget;
    auto* h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(8);
    h->addWidget(capsLabel(label, 13));
    if (restartDot) {
        h->addWidget(new RestartDot);
    }
    if (!tip.isEmpty()) {
        h->addWidget(new HelpBadge(tip)); // a visible "?" hint next to the label
    }
    h->addStretch(1);
    if (extra) {
        h->addWidget(extra);
    }
    if (right) {
        h->addWidget(right);
    }
    return w;
}

// ---- HeadsetArt

HeadsetArt::HeadsetArt(QWidget* parent) : QWidget(parent) {
    setMinimumSize(220, 150);
}

void HeadsetArt::setLit(bool lit) {
    lit_ = lit;
    update();
}

void HeadsetArt::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    // Drawn in a 300 x 200 box, scaled to fit.
    const double k = std::min(width() / 300.0, height() / 200.0);
    p.translate((width() - 300 * k) / 2, (height() - 200 * k) / 2);
    p.scale(k, k);
    const QColor line(0x6a, 0x72, 0xb8);
    QPen pen(line, 2.2);
    pen.setJoinStyle(Qt::RoundJoin);
    pen.setCapStyle(Qt::RoundCap);

    // soft glow behind the visor when the screen is on
    if (lit_) {
        p.save();
        p.translate(150, 112);
        p.scale(1.0, 0.62); // an elliptic glow that fades out before its edge
        QRadialGradient glow(QPointF(0, 0), 150);
        glow.setColorAt(0, QColor(74, 90, 232, 60));
        glow.setColorAt(1, QColor(74, 90, 232, 0));
        p.setPen(Qt::NoPen);
        p.setBrush(glow);
        p.drawEllipse(QPointF(0, 0), 150, 150);
        p.restore();
    }
    p.setBrush(Qt::NoBrush);
    p.setPen(pen);
    // head strap arcs
    QPainterPath strap;
    strap.moveTo(70, 70);
    strap.cubicTo(80, 20, 220, 20, 230, 70);
    p.drawPath(strap);
    p.drawRoundedRect(QRectF(132, 18, 36, 40), 4, 4);
    // the visor body
    QPainterPath body;
    body.moveTo(40, 80);
    body.cubicTo(40, 62, 60, 58, 80, 58);
    body.lineTo(220, 58);
    body.cubicTo(240, 58, 260, 62, 260, 80);
    body.lineTo(258, 140);
    body.cubicTo(256, 162, 240, 170, 220, 170);
    body.cubicTo(190, 170, 175, 150, 150, 150);
    body.cubicTo(125, 150, 110, 170, 80, 170);
    body.cubicTo(60, 170, 44, 162, 42, 140);
    body.closeSubpath();
    p.setBrush(QColor(0x14, 0x16, 0x26));
    p.drawPath(body);
    // the front plate
    QPainterPath plate;
    plate.moveTo(62, 82);
    plate.lineTo(238, 82);
    plate.lineTo(236, 130);
    plate.cubicTo(234, 146, 222, 152, 210, 150);
    plate.cubicTo(185, 146, 170, 132, 150, 132);
    plate.cubicTo(130, 132, 115, 146, 90, 150);
    plate.cubicTo(78, 152, 66, 146, 64, 130);
    plate.closeSubpath();
    p.setBrush(Qt::NoBrush);
    p.drawPath(plate);
    // side mounts
    p.drawRoundedRect(QRectF(28, 76, 14, 46), 4, 4);
    p.drawRoundedRect(QRectF(258, 76, 14, 46), 4, 4);
    // the accent V
    QPen v(lit_ ? QColor(0x6e, 0x7c, 0xff) : QColor(0x3c, 0x42, 0x7a), 3.5);
    v.setCapStyle(Qt::RoundCap);
    p.setPen(v);
    QPainterPath vee;
    vee.moveTo(84, 96);
    vee.lineTo(150, 118);
    vee.lineTo(216, 96);
    p.drawPath(vee);
    // camera dots
    p.setPen(QPen(line, 1.6));
    for (const QPointF c : {QPointF(74, 92), QPointF(226, 92), QPointF(120, 160), QPointF(180, 160)}) {
        p.drawEllipse(c, 3.5, 3.5);
    }
}

// ---- GazeDot

GazeDot::GazeDot(QWidget* parent) : QWidget(parent) {
    setFixedSize(120, 120);
    setToolTip(QStringLiteral("Where you look, live from the eye tracker (straight ahead is the centre)."));
}

void GazeDot::setGaze(std::optional<std::pair<float, float>> tangent) {
    if (tangent == g_) {
        return;
    }
    g_ = tangent;
    update();
}

void GazeDot::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(4, 4, -4, -4);
    p.setPen(QPen(color::border, 1));
    p.setBrush(QColor(0x0f, 0x0f, 0x12));
    p.drawRoundedRect(r, 3, 3);
    const QPointF c = r.center();
    const double rad = r.width() / 2 - 8;
    p.setPen(QPen(QColor(0x2e, 0x2e, 0x38), 1, Qt::DashLine));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(c, rad, rad);
    p.drawEllipse(c, rad / 2, rad / 2);
    p.setPen(QPen(QColor(0x2e, 0x2e, 0x38), 1));
    p.drawLine(QPointF(c.x() - rad, c.y()), QPointF(c.x() + rad, c.y()));
    p.drawLine(QPointF(c.x(), c.y() - rad), QPointF(c.x(), c.y() + rad));
    if (!g_) {
        const QString t = QStringLiteral("no gaze");
        p.setFont(labelFont(10));
        const QFontMetricsF fm(p.font());
        const QRectF box(c.x() - fm.horizontalAdvance(t) / 2 - 6, c.y() - fm.height() / 2 - 2,
                         fm.horizontalAdvance(t) + 12, fm.height() + 4);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0x0f, 0x0f, 0x12));
        p.drawRect(box);
        p.setPen(color::dim);
        p.drawText(box, Qt::AlignCenter, t);
        return;
    }
    // tangent 1 (45 degrees) at the outer circle
    const double x = std::clamp(double(g_->first), -1.0, 1.0), y = std::clamp(double(g_->second), -1.0, 1.0);
    const QPointF d(c.x() + x * rad, c.y() - y * rad);
    QRadialGradient glow(d, 12);
    glow.setColorAt(0, QColor(122, 136, 255, 160));
    glow.setColorAt(1, QColor(122, 136, 255, 0));
    p.setPen(Qt::NoPen);
    p.setBrush(glow);
    p.drawEllipse(d, 12, 12);
    p.setBrush(QColor(0xc8, 0xce, 0xff));
    p.drawEllipse(d, 4, 4);
}

// ---- Battery

Battery::Battery(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void Battery::set(int percent, bool charging) {
    if (percent == percent_ && charging == charging_) {
        return;
    }
    percent_ = percent;
    charging_ = charging;
    update();
}

void Battery::paintEvent(QPaintEvent*) {
    if (percent_ < 0) {
        return;
    }
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF body(1.5, 1.5, width() - 6.0, height() - 3.0);
    const QColor level = percent_ <= 10 ? color::red : percent_ <= 20 ? color::amber : color::accentText;
    p.setPen(QPen(color::dim, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(body, 2, 2);
    p.setPen(Qt::NoPen);
    p.setBrush(color::dim);
    p.drawRoundedRect(QRectF(body.right() + 1, body.center().y() - 2.5, 2.5, 5), 1, 1);
    const QRectF inner = body.adjusted(2, 2, -2, -2);
    p.setBrush(level);
    p.drawRect(QRectF(inner.left(), inner.top(), inner.width() * std::clamp(percent_, 0, 100) / 100.0, inner.height()));
    if (charging_) {
        QPainterPath bolt;
        const QPointF c = body.center();
        bolt.moveTo(c.x() + 2, c.y() - 6);
        bolt.lineTo(c.x() - 4, c.y() + 1);
        bolt.lineTo(c.x() - 0.3, c.y() + 1);
        bolt.lineTo(c.x() - 2, c.y() + 6);
        bolt.lineTo(c.x() + 4, c.y() - 1);
        bolt.lineTo(c.x() + 0.3, c.y() - 1);
        bolt.closeSubpath();
        p.setPen(QPen(color::panel, 1.5));
        p.setBrush(Qt::white);
        p.drawPath(bolt);
    }
}

} // namespace pibridge::ui
