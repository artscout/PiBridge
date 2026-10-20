// SPDX-License-Identifier: MIT
// The VR home page (docs/home.md): what pibridge-home shows in the headset and its switches. The PiBridge window itself is
// the home's menu in the headset (core/vr_panel.h), so this page is also what you point at there: the room setup,
// recenter, passthrough on/off and the backgrounds (thumbnails; a picture is downloaded on first use).
// Settings: home_background, home_panel (pibridge.conf, docs/CONVENTIONS.md).
#pragma once

#include <QWidget>
#include <functional>
#include <vector>

class QGridLayout;
class QLabel;
class QPushButton;
class QToolButton;

namespace pibridge::ui {
class Toggle;
class Card;
}

namespace pibridge::app {

class BackgroundLibrary;
class Conf;
class Runner;
class StateSource;

class VrHomePage : public QWidget {
    Q_OBJECT
public:
    using ToolFinder = std::function<QString(const QString&)>;
    VrHomePage(StateSource& source, Conf& conf, Runner& runner, ToolFinder findTool, BackgroundLibrary& lib,
               QWidget* parent = nullptr);
    /// pibridge-home shows this window in the headset now (VrPanelLink::shownChanged).
    void setShownInHeadset(bool on);

protected:
    void showEvent(QShowEvent* e) override;

signals:
    /// "map" or "recenter": the Room page's own button (it has the checks and says why when it cannot).
    void roomAction(const QString& which);

private:
    void rebuildTiles();
    void choose(const QString& value);
    void updateView();

    StateSource& source_;
    Conf& conf_;
    Runner& runner_;
    ToolFinder findTool_;
    BackgroundLibrary& lib_;
    bool shownInHeadset_ = false;
    pibridge::ui::Card* homeCard_ = nullptr;
    QLabel* homeLine_ = nullptr;
    QLabel* homeNote_ = nullptr;
    pibridge::ui::Toggle* panel_ = nullptr;
    pibridge::ui::Toggle* passthrough_ = nullptr;
    QPushButton* steamvr_ = nullptr;
    QLabel* steamvrWhy_ = nullptr;
    QGridLayout* grid_ = nullptr;
    std::vector<QToolButton*> tiles_;
    QLabel* credit_ = nullptr;
    QLabel* downloadLine_ = nullptr;
};

} // namespace pibridge::app
