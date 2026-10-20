// SPDX-License-Identifier: MIT
// Apps: the installed Steam games (found by reading Steam's files, core/apps_steam.h) and the user's own programs
// (~/.config/pibridge/apps.toml, core/apps_own.h) as cover tiles with PLAY, a state chip and the last played time; a
// search box and a "VR only" switch on top; "Add app..." with an editor (program, arguments, working directory,
// environment, icon, quad views on/off, the Performance profile). Own apps start once SteamVR is fully ready
// (core/steamvr_ready.h), their output goes to ~/.local/state/pibridge/apps/ (core/apps_launch.h). With the tracking
// mode inside-out and apps through "Monado directly" they start on the Tracking session's Monado instead
// (core/apps_runtime.h), and Steam games are off with the reason.
// In fake mode the library and the apps are made up and nothing is ever started.
#pragma once

#include <QHash>
#include <QIcon>
#include <QSet>
#include <QTimer>
#include <QWidget>
#include <memory>
#include <vector>

#include "../core/apps_launch.h"
#include "../core/apps_own.h"
#include "../core/apps_steam.h"
#include "../core/io_session.h"
#include "../core/steamvr_ready.h"

class QLabel;
class QLineEdit;
class QPushButton;
class QVBoxLayout;

namespace pibridge::ui {
class Chip;
class Toggle;
} // namespace pibridge::ui

namespace pibridge::app {

class StateSource;
class Runner;
class AppTile;
class FlowLayout;

// The nav icon of the page: four rounded squares.
QIcon appsNavIcon(const QColor& normal, const QColor& checked);

class AppsPage : public QWidget {
    Q_OBJECT
public:
    // `runner` starts Steam (FakeRunner in fake mode); own programs go through QtAppProcesses, or a
    // FakeAppProcesses in fake mode. `conf`, `io` ("Monado directly", Development page) and `tracking` (START
    // TRACKING, SteamVR + Monado) decide the runtime; without them every app runs on SteamVR.
    AppsPage(StateSource& source, Runner& runner, Conf* conf = nullptr, IoSessionControl* io = nullptr,
             IoSessionControl* tracking = nullptr, QWidget* parent = nullptr);
    ~AppsPage() override;

    const SteamVrReady& readiness() const { return *ready_; }

signals:
    // "Quad views profile..." of an own app: the profile name (core/apps_own.h qvProfileName()).
    void quadViewsProfileRequested(const QString& name);

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    void rescan();
    void rebuild();
    void refreshTiles();
    void refreshTile(AppTile* t);
    void applyFilter();
    void updateReadyLine();
    void say(const QString& text, bool error = false);
    void editApp(const QString& id); // "" = add
    void removeApp(const QString& id);
    bool saveStore();
    void openLog(const QString& id);
    void playOwn(const QString& id);
    void showFakeStates();

    StateSource& source_;
    Runner& runner_;
    bool fake_;
    std::unique_ptr<AppProcesses> procs_;
    std::unique_ptr<SteamVrReady> ready_;  // before launcher_: the launcher's waits point into it
    std::unique_ptr<TrackingRuntime> runtime_;
    IoSessionControl* io_ = nullptr;
    IoSessionControl* tracking_ = nullptr;
    AppTarget target_ = AppTarget::SteamVr; // as of the last updateRuntimeNow()
    MonadoReadiness monadoNow_;
    void updateRuntimeNow();
    std::unique_ptr<AppLauncher> launcher_;
    AppsStore store_;
    SteamLibrary steam_;
    QSet<QString> steamRunning_;
    qint64 lastScanMs_ = 0;
    QTimer runningTimer_;
    QTimer messageTimer_;

    QLineEdit* search_ = nullptr;
    pibridge::ui::Toggle* vrOnly_ = nullptr;
    QLabel* vrOnlyNote_ = nullptr;
    pibridge::ui::Chip* readyChip_ = nullptr;
    QLabel* readyWords_ = nullptr;
    QLabel* message_ = nullptr;
    QLabel* steamTitle_ = nullptr;
    QLabel* steamEmpty_ = nullptr;
    QLabel* ownTitle_ = nullptr;
    QLabel* ownEmpty_ = nullptr;
    QWidget* steamGrid_ = nullptr;
    QWidget* ownGrid_ = nullptr;
    FlowLayout* steamFlow_ = nullptr;
    FlowLayout* ownFlow_ = nullptr;
    std::vector<AppTile*> tiles_;
};

} // namespace pibridge::app
