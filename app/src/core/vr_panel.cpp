// SPDX-License-Identifier: MIT
#include "vr_panel.h"

#include "pibridge_vrpanel.h"

#include <QApplication>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <QWidget>
#include <unistd.h>

namespace pibridge::app {

QSize pictureSize(QSize w) {
    if (w.isEmpty()) {
        return {};
    }
    const double s = std::min({1.0, static_cast<double>(vrpanel::kMaxW) / w.width(), static_cast<double>(vrpanel::kMaxH) / w.height()});
    return {std::max(1, static_cast<int>(w.width() * s)), std::max(1, static_cast<int>(w.height() * s))};
}

QPoint pointerToWindow(std::uint32_t x, std::uint32_t y, QSize picture, QSize window) {
    if (picture.isEmpty() || window.isEmpty()) {
        return {};
    }
    const double sx = static_cast<double>(window.width()) / picture.width();
    const double sy = static_cast<double>(window.height()) / picture.height();
    return {std::clamp(static_cast<int>((x + 0.5) * sx), 0, window.width() - 1),
            std::clamp(static_cast<int>((y + 0.5) * sy), 0, window.height() - 1)};
}

VrPanelLink::VrPanelLink(QWidget* window, QObject* parent) : QObject(parent), window_(window), link_(std::make_unique<vrpanel::Link>()) {
    connect(&timer_, &QTimer::timeout, this, &VrPanelLink::tick);
}

VrPanelLink::~VrPanelLink() {
    if (link_ && link_->isOpen()) {
        link_->header()->appBeatNs.store(0);
    }
}

void VrPanelLink::start() {
    timer_.start(500);
    sinceFrame_.start();
    sincePointer_.start();
}

void VrPanelLink::tick() {
    if (!window_) {
        return;
    }
    if (!link_->isOpen()) {
        if (sinceOpen_.isValid() && sinceOpen_.elapsed() < 5000) {
            return;
        }
        sinceOpen_.start();
        if (!link_->open()) {
            return; // tried again in 5 s
        }
    }
    auto* h = link_->header();
    const std::uint64_t now = vrpanel::monoNs();
    h->appPid.store(static_cast<std::uint32_t>(::getpid()));
    h->appBeatNs.store(now);
    const bool shown = vrpanel::fresh(h->homeBeatNs.load(), now);
    if (shown != shown_) {
        shown_ = shown;
        haveWheel_ = false;
        timer_.setInterval(shown ? 33 : 500);
        emit shownChanged(shown);
        if (!shown && down_ && pressTarget_) {
            // the home went away mid-click: let go
            QMouseEvent up(QEvent::MouseButtonRelease, QPointF(), QPointF(), Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
            send(pressTarget_, &up);
            down_ = false;
        }
    }
    if (!shown) {
        return;
    }
    pointer();
    // 30 pictures a second while pointed at in the last 2 s, else 10
    const int every = sincePointer_.elapsed() < 2000 ? 30 : 95;
    if (sinceFrame_.elapsed() >= every) {
        sinceFrame_.start();
        render();
    }
}

void VrPanelLink::render() {
    const QSize ws = window_->size();
    picture_ = pictureSize(ws);
    if (picture_.isEmpty()) {
        return;
    }
    if (img_.size() != ws) {
        img_ = QImage(ws, QImage::Format_ARGB32_Premultiplied);
    }
    img_.fill(Qt::black);
    {
        QPainter p(&img_);
        window_->render(&p, QPoint(), QRegion(), QWidget::DrawWindowBackground | QWidget::DrawChildren);
    }
    const QImage out = picture_ == ws ? img_ : img_.scaled(picture_, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    // ARGB32 is B, G, R, A in memory on little-endian machines: the link's BGRA
    (void)link_->writeFrame(out.constBits(), static_cast<std::uint32_t>(out.width()), static_cast<std::uint32_t>(out.height()),
                            static_cast<std::size_t>(out.bytesPerLine()));
}

void VrPanelLink::send(QWidget* target, QEvent* e) { QCoreApplication::sendEvent(target, e); }

void VrPanelLink::pointer() {
    auto* h = link_->header();
    const std::uint64_t raw = h->pointer.load();
    const std::int64_t wheel = h->wheel.load();
    const vrpanel::Pointer p = vrpanel::unpackPointer(raw);
    if (!haveWheel_) {
        lastWheel_ = wheel;
        haveWheel_ = true;
    }
    if (raw == lastPointer_ && wheel == lastWheel_) {
        return;
    }
    sincePointer_.start();
    lastPointer_ = raw;
    const QPoint wp = pointerToWindow(p.x, p.y, picture_, window_->size());
    QWidget* under = p.on ? window_->childAt(wp) : nullptr;
    if (under == nullptr && p.on) {
        under = window_;
    }
    QWidget* target = down_ && pressTarget_ ? pressTarget_.data() : under;
    if (hoverTarget_ != under) {
        if (hoverTarget_) {
            QEvent leave(QEvent::Leave);
            send(hoverTarget_, &leave);
        }
        hoverTarget_ = under;
        if (under) {
            QEnterEvent enter(under->mapFrom(window_, QPointF(wp)), QPointF(wp), window_->mapToGlobal(QPointF(wp)));
            send(under, &enter);
        }
    }
    if (target) {
        const QPointF local = target->mapFrom(window_, QPointF(wp));
        const QPointF global = window_->mapToGlobal(QPointF(wp));
        const bool wantDown = (p.buttons & 1U) != 0;
        if (p.on || down_) {
            QMouseEvent mv(QEvent::MouseMove, local, local, global, Qt::NoButton, down_ ? Qt::LeftButton : Qt::NoButton,
                           Qt::NoModifier);
            send(target, &mv);
        }
        if (wantDown && !down_ && p.on) {
            down_ = true;
            pressTarget_ = under;
            QMouseEvent pr(QEvent::MouseButtonPress, under->mapFrom(window_, QPointF(wp)), under->mapFrom(window_, QPointF(wp)),
                           global, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
            send(under, &pr);
        } else if (!wantDown && down_) {
            down_ = false;
            QWidget* t = pressTarget_ ? pressTarget_.data() : target;
            const QPointF l2 = t->mapFrom(window_, QPointF(wp));
            QMouseEvent rl(QEvent::MouseButtonRelease, l2, l2, global, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
            send(t, &rl);
            pressTarget_ = nullptr;
        }
        if (wheel != lastWheel_ && under) {
            const int delta = static_cast<int>(std::clamp<std::int64_t>(wheel - lastWheel_, -1200, 1200));
            const QPointF l3 = under->mapFrom(window_, QPointF(wp));
            QWheelEvent we(l3, global, QPoint(), QPoint(0, delta), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
            send(under, &we);
        }
    }
    lastWheel_ = wheel;
}

} // namespace pibridge::app
