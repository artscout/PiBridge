// SPDX-License-Identifier: MIT
// Room: the playspace setup. SETUP PLAYSPACE (inside-out) runs the camera-based mapper pibridge-roommap setup fast over
// the running session (no in-headset wizard): stand in the middle, look around, hold still looking forward; the floor
// comes from your height, the centre and forward from where you stood. Here on the desktop: the universes of SteamVR's
// chaperone file (the one SteamVR tracks in is marked), the saved rooms (load, rename, delete), and a floor plan of the
// selected one with a few edits (drag a corner, double-click a wall to add one, right-click a corner to delete it, a
// new 2 x 2 m room, fit the play area, centre it, turn it) and Apply to SteamVR / Revert / Restore original. The
// editing is libpibridge-room's Editor; the files and gates are core/roomsetup.h and core/roommap_link.h.
#pragma once

#include <QTimer>
#include <QWidget>
#include <memory>

#include "../core/actions.h"
#include "../core/conf.h"
#include "../core/roomsetup.h"
#include "../core/state.h"
#include "pibridge/room/editor.h"

class QLabel;
class QLineEdit;
class QPushButton;
class QVBoxLayout;

namespace pibridge::ui {
class Card;
} // namespace pibridge::ui

namespace pibridge::app {

/// The top-down floor plan of the editor's boundary, play area and centre (forward is up).
class FloorPlan : public QWidget {
    Q_OBJECT
public:
    explicit FloorPlan(QWidget* parent = nullptr);
    void setEditor(pibridge::room::Editor* ed, double warnDistance);
    QSize sizeHint() const override { return {640, 460}; }
    QSize minimumSizeHint() const override { return {320, 240}; }

signals:
    void edited();

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;

private:
    QPointF toScreen(pibridge::room::Pt2 p) const;
    pibridge::room::Pt2 toFloor(QPointF s) const;
    void fit();
    int vertexAt(QPointF s) const;

    pibridge::room::Editor* ed_ = nullptr;
    double warn_ = 0;
    double scale_ = 100; // px per metre
    QPointF origin_;     // screen position of stage (0, 0)
    int drag_ = -1;
    pibridge::room::Pt2 dragOff_;
};

class RoomPage : public QWidget {
    Q_OBJECT
public:
    // `conf`: the tracking mode; in the inside-out mode the page works on driver_monado's universe 2.
    RoomPage(StateSource& source, Runner& runner, Actions::ToolFinder findTool, Conf* conf = nullptr,
             QWidget* parent = nullptr);
    ~RoomPage() override;

    // Start the in-headset room setup from elsewhere (the Dashboard's ROOM SETUP button), without opening this page.
    void launchSetup();
    // Start directly in QUICK mode (the Dashboard's QUICK ROOM SETUP button): skip the in-VR Full/Quick mode menu.
    void launchQuickSetup();

protected:
    void showEvent(QShowEvent*) override;

private:
    void reload(const std::string& universe = {});
    void refreshLists();
    void updateView();
    void say(const QString& text, bool error = false);
    void selectUniverse(const QString& id);
    void loadRoom(const QString& name);
    void renameRoom(const QString& name);
    void deleteRoom(const QString& name);
    void saveAsRoom();
    void apply();
    bool steamvrRunning() const;
    bool insideOut() const;
    void createInsideOutUniverse();
    void mapRoom(bool quick = false);
    QString roommapTool() const;
    QString toolPath() const; // pibridge-roomsetup: the in-headset passthrough view of SETUP PLAYSPACE

    StateSource& source_;
    Runner& runner_;
    Conf* conf_ = nullptr;
    bool monadoMissing_ = false; // inside-out mode, and the chaperone file has no universe 2 yet
    bool loadedInsideOut_ = false; // the mode of the last load (a mode change picks the universe again)
    Actions::ToolFinder findTool_;
    bool fake_;
    pibridge::room::Env env_;
    std::unique_ptr<pibridge::room::Store> store_;
    std::unique_ptr<pibridge::room::FileZoneControl> zone_;
    std::unique_ptr<pibridge::room::Editor> ed_;
    QString loadError_;
    QString loadedRoom_; // the saved room shown, "" when it is the universe's own boundary
    QString lastActive_;

    pibridge::ui::Card* mapCard_ = nullptr;
    QLabel* mapSteps_ = nullptr;
    QPushButton* map_ = nullptr;
    QLabel* mapStatus_ = nullptr;
    QLabel* mapWhy_ = nullptr;
    bool mapRunning_ = false;
    // the floor from the user's height (fed to pibridge-roommap setup fast as --eye-height)
    QLineEdit* bodyH_ = nullptr;
    QLineEdit* eyeH_ = nullptr;
    QLabel* eyeNote_ = nullptr;
    void saveHeights();
    QLabel* ioNote_ = nullptr;
    QPushButton* createIo_ = nullptr;
    pibridge::ui::Card* planCard_ = nullptr;
    FloorPlan* plan_ = nullptr;
    QLabel* planInfo_ = nullptr;
    QLabel* message_ = nullptr;
    QPushButton* apply_ = nullptr;
    QPushButton* revert_ = nullptr;
    QPushButton* restore_ = nullptr;
    QPushButton* undo_ = nullptr;
    QLabel* applyWhy_ = nullptr;
    pibridge::ui::Card* uniCard_ = nullptr;
    QVBoxLayout* uniList_ = nullptr;
    pibridge::ui::Card* roomsCard_ = nullptr;
    QVBoxLayout* roomList_ = nullptr;
};

} // namespace pibridge::app
