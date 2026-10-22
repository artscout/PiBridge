// SPDX-License-Identifier: MIT
// The dashboard: toolbar (SteamVR, restart services/headset), the headset card, controllers, quick settings
// (eye tracking, calibration, import of the Windows calibration), tracking and sound.
#pragma once

#include <QTimer>
#include <QWidget>

#include "../core/actions.h"
#include "../core/conf.h"
#include "../core/io_session.h"
#include "../core/monado_devices.h"
#include "../core/state.h"
#include "../core/steamvr_reset.h"

#include <QElapsedTimer>
#include <memory>

class QLabel;
class QPushButton;

namespace pibridge::ui {
class Battery;
class Chip;
class GazeDot;
class HeadsetArt;
class Segmented;
class SliderRow;
class Toggle;
} // namespace pibridge::ui

namespace pibridge::app {

class RecordCard;
class SoundTestPanel;
class OverlayControl;

class DashboardPage : public QWidget {
    Q_OBJECT
public:
    // `io`: START TRACKING (SteamVR + Monado); `direct`: "Monado directly" (Development page), whose Monado the
    // Controllers card and the headset card read while it runs. `overlay`: the SteamVR dashboard launcher, toggled from
    // Quick settings (owned by the main window; a leaf).
    DashboardPage(StateSource& source, Actions& actions, Conf& conf, IoSessionControl& io, IoSessionControl& direct,
                  OverlayControl& overlay, QWidget* parent = nullptr);

    // The Record card (the screenshot mode shows it recording).
    RecordCard* recordCard() const { return record_; }

signals:
    void openTracking();
    void openRoom();
    void startRoomSetup();      // ROOM SETUP button: launch the in-headset setup directly (mode menu, no page nav)
    void startQuickRoomSetup(); // QUICK ROOM SETUP button: launch straight into the quick circle setup (no mode menu)

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    void applyState();
    void applyConf();
    void updateGaze();
    void updateVrStatus();
    void updateFixCaps(); // show the top-bar "Fix permissions" button only when monado-service lacks cap_sys_nice
    void openDriverSettings(); // CustomHeadsetOpenVR's configurator (render size, lens, ...), from the Tracking card
    void openTrackingSettings(); // the Tracking card's Settings: SLAM settings (inside-out) or driver settings (Lighthouse)
    void say(const QString& text, bool error = false);
    void setBusy(const QString& what);
    void toolDone(const QString& what, int code, const QString& out);
    void updateToolButtons();
    // RESET STEAMVR: scan, the confirm dialog, then close the listed PIDs or restart Steam (core/steamvr_reset.h).
    void resetSteamVr();
    bool confirmReset(const ResetDialogModel& m);
    void closeSteamVr(std::vector<SteamVrProc> targets);
    void restartSteam();
    // HEADSET DRIVER SETTINGS: shown only while CustomHeadsetOpenVR is the headset's SteamVR driver (core/chovr.h),
    // enabled when its configurator is found. Re-checked whenever the page is shown.
    void updateChovr();
    // The Tracking card's mode chip and the START TRACKING button (inside-out mode only).
    void updateMode();
    // The Quick settings SteamVR-dashboard toggle: reflects the saved choice and shows the launcher's plain-words status.
    void updateOverlay();

    StateSource& source_;
    Actions& actions_;
    Conf& conf_;
    IoSessionControl& io_;
    IoSessionControl& direct_;
    OverlayControl& overlay_; // the SteamVR dashboard launcher, toggled from Quick settings (owned by the main window)
    // the controllers from Monado while the Tracking session runs with apps through Monado directly
    bool monadoMode_ = false;
    MonadoStatus mndStatus_;
    QString mndService_; // the monado-service the library below was looked up for
    QString mndLibrary_;
    QLabel* runtime_ = nullptr;
    std::unique_ptr<MonadoWorker> mnd_; // last: joined first, before the widgets go
    pibridge::ui::Chip* trChip_ = nullptr;
    QLabel* trMode_ = nullptr;
    pibridge::ui::Segmented* trModeSel_ = nullptr; // head-tracking mode (Lighthouse / Inside-out), same as the Tracking page
    QPushButton* ioStart_ = nullptr;
    QPushButton* ioStop_ = nullptr;
    QPushButton* roomBtn_ = nullptr;
    QPushButton* quickRoomBtn_ = nullptr;
    QPushButton* fixCaps_ = nullptr; // top bar: "Fix permissions" (pkexec setcap cap_sys_nice on monado-service)
    QString fixCapsPath_;            // the monado-service the button fixes
    QLabel* vrStatus_ = nullptr;
    qint64 vrStartMs_ = 0; // when VR became running (app clock), for the "VR running Xm" line; 0 = not running
    QTimer gazeTimer_;
    QTimer messageTimer_;         // clears the message line (8 s; errors 20 s)
    bool waitingForSteamVr_ = false; // "Starting SteamVR..." is shown until vrserver runs
    QString busy_; // the eye tool running now, or empty
    // Reset SteamVR: one at a time, stepped by resetTimer_ with resetClock_ (monotonic ms).
    QPushButton* reset_ = nullptr;
    QString chovrGui_; // the configurator found, or empty
    QTimer resetTimer_;
    QElapsedTimer resetClock_;
    std::unique_ptr<SteamVrCloser> closer_;
    std::unique_ptr<SteamRestarter> restarter_;
    // The Runner for `steam -shutdown` / `steam`: a ProcessRunner of this page in real mode, a FakeRunner in fake mode
    // (Actions keeps its Runner to itself).
    std::unique_ptr<Runner> steamRunner_;

    QLabel* message_ = nullptr;
    pibridge::ui::Chip* hsChip_ = nullptr;
    QLabel* hsTitle_ = nullptr;
    QLabel* hsSub_ = nullptr;   // the tracking mode line under the title (inside-out / Lighthouse)
    QLabel* hsHint_ = nullptr;
    QLabel* hsError_ = nullptr;
    QLabel* hsState_ = nullptr;
    QLabel* hsSerial_ = nullptr; // the headset serial (real serial from SteamVR/keepalived)
    pibridge::ui::HeadsetArt* art_ = nullptr;
    pibridge::ui::Segmented* refresh_ = nullptr;
    pibridge::ui::Chip* ctrlChip_ = nullptr;
    QLabel* ctrlText_ = nullptr;
    struct ControllerWidgets {
        QLabel* icon = nullptr;
        QLabel* model = nullptr;
        QLabel* detail = nullptr;
        pibridge::ui::Chip* chip = nullptr;
        pibridge::ui::Battery* battery = nullptr;
        QLabel* batteryText = nullptr;
        QLabel* serial = nullptr;
        QPushButton* copy = nullptr;
        QString serialText;
    };
    ControllerWidgets ctrlRows_[2]; // left, right
    QLabel* ctrlExtra_ = nullptr;
    QLabel* trSummary_ = nullptr;
    QWidget* trStations_ = nullptr;
    QLabel* trText_ = nullptr;
    QString trKey_; // what the station rows show now (rebuilt only when it changes)
    pibridge::ui::Toggle* passthrough_ = nullptr;
    pibridge::ui::Toggle* dashOverlay_ = nullptr; // Quick settings: the SteamVR dashboard panel on/off
    QLabel* dashOverlayLine_ = nullptr;           // its plain-words status
    pibridge::ui::Toggle* eyeToggle_ = nullptr;
    QPushButton* calibrate_ = nullptr;
    QPushButton* recenter_ = nullptr;
    pibridge::ui::SliderRow* ipd_ = nullptr; // ipd_mm: the user's IPD in whole mm (lens motors + rendering)
    QTimer* ipdTimer_ = nullptr;             // sends the IPD 0.6 s after the slider stops
    bool ipdFromHeadset_ = false;            // the slider took the lens IPD the headset reported
    double lastLensIpdMm_ = 0;               // the lens IPD last taken from the headset
    QElapsedTimer ipdSent_;                  // since our last motor request (the lenses travel meanwhile)
    pibridge::ui::Chip* eyeChip_ = nullptr;
    QLabel* eyeLine_ = nullptr;
    QLabel* toolResult_ = nullptr;
    pibridge::ui::GazeDot* gaze_ = nullptr;
    pibridge::ui::Toggle* audio_ = nullptr;
    SoundTestPanel* soundPanel_ = nullptr; // the TEST SPEAKERS / TEST MIC card (gated on SteamVR running)
    RecordCard* record_ = nullptr;
};

} // namespace pibridge::app
