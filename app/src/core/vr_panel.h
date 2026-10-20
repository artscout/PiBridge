// SPDX-License-Identifier: MIT
// The app's side of the PiBridge window in the VR home (common/pibridge_vrpanel.h, docs/home.md): while pibridge-home shows
// the panel (its beat is under 2 s old) the main window is rendered offscreen (QWidget::render into an image) about
// 30 times a second (10 while nobody points at it) and written to the shared file; the controller pointer that
// pibridge-home writes back is turned into mouse events (move, left button, wheel) on the widget under it. Without
// pibridge-home it only checks the file twice a second. A leaf: if anything fails, the app and the home go on alone.
#pragma once

#include <QElapsedTimer>
#include <QImage>
#include <QObject>
#include <QPoint>
#include <QPointer>
#include <QTimer>
#include <memory>

namespace pibridge::vrpanel {
class Link;
}

class QWidget;

namespace pibridge::app {

/// Pure: the window position (logical pixels) of a pointer in picture pixels (the picture may be scaled down to fit
/// the link's 2048 px).
QPoint pointerToWindow(std::uint32_t x, std::uint32_t y, QSize picture, QSize window);
/// The picture size for a window (at most 2048 x 2048, the aspect kept).
QSize pictureSize(QSize window);

class VrPanelLink : public QObject {
    Q_OBJECT
public:
    explicit VrPanelLink(QWidget* window, QObject* parent = nullptr);
    ~VrPanelLink() override;
    void start();
    /// pibridge-home shows the window now.
    bool shown() const { return shown_; }

signals:
    void shownChanged(bool shown);

private:
    void tick();
    void render();
    void pointer();
    void send(QWidget* target, QEvent* e);

    QPointer<QWidget> window_;
    std::unique_ptr<vrpanel::Link> link_;
    QTimer timer_;
    QElapsedTimer sinceOpen_;
    QElapsedTimer sincePointer_;
    QElapsedTimer sinceFrame_;
    bool shown_ = false;
    QImage img_;
    QSize picture_;
    std::uint64_t lastPointer_ = 0;
    std::int64_t lastWheel_ = 0;
    bool haveWheel_ = false;
    QPointer<QWidget> pressTarget_;
    QPointer<QWidget> hoverTarget_;
    bool down_ = false;
};

} // namespace pibridge::app
