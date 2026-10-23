// SPDX-License-Identifier: MIT
// The window: navigation column on the left, the pages, and the status bar at the bottom.
#pragma once

#include <QMainWindow>
#include <vector>

#include "core/actions.h"
#include "core/conf.h"
#include "core/io_session.h"
#include "core/overlay_control.h"
#include <memory>
#include "core/state.h"

class QLabel;
class QPushButton;
class QStackedWidget;

namespace pibridge::app {

class VrPanelLink;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    // `fake`: fixed state, commands go to the injected runner (a FakeRunner) and nothing is read from the system.
    MainWindow(StateSource& source, Runner& runner, Actions::ToolFinder findTool, QWidget* parent = nullptr);

    // Tracking: the tracking mode and the inside-out settings (pages/tracking.h).
    enum Page { Dashboard, Apps, Performance, Tracking, Room, Logs, About, Display, Recording, PageCount };
    void showPage(int page);
    static QString pageSlug(int page);
    QWidget* page(int page) const;
    // START VR in inside-out, and "Monado directly" (no longer started from the UI).
    IoSessionControl& trackingSession() { return *io_; }
    IoSessionControl& directSession() { return *direct_; }

private:
    void applyState();
    // "Quad views profile..." on the Apps page: the Performance page, with the profile of that name selected when
    // it has one (else it stays on its current choice).
    void openQuadViewsProfile(const QString& name);

    StateSource& source_;
    Conf conf_;
    Actions actions_;
    // the one tracking session (START TRACKING), shared by the Dashboard, the Tracking page and the Apps page
    std::unique_ptr<IoSessionControl> io_;
    std::unique_ptr<IoSessionControl> direct_;
    // the SteamVR dashboard launcher (core/overlay_control.h): a leaf; its clicks launch room setup
    std::unique_ptr<OverlayControl> overlay_;
    QStackedWidget* stack_ = nullptr;
    std::vector<QPushButton*> nav_;
    QLabel* sbSteam_ = nullptr;
    QLabel* sbHeadset_ = nullptr;
    QLabel* sbEye_ = nullptr;
    // the link that shows this window in the headset (the SteamVR-panel embed)
    VrPanelLink* vrPanel_ = nullptr;
};

} // namespace pibridge::app
