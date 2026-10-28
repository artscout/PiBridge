// SPDX-License-Identifier: MIT
// The dashboard: toolbar (SteamVR, restart services/headset), the headset card, controllers, quick settings
// (eye tracking, calibration, import of the Windows calibration), tracking and sound.
#pragma once

#include <QTimer>
#include <QWidget>

#include "../core/actions.h"
#include "../core/conf.h"
#include "../core/controllers.h"
#include "../core/io_session.h"
#include "../core/monado_devices.h"
#include "../core/state.h"
#include "../core/steamvr_reset.h"

#include <QElapsedTimer>
#include <memory>

class QLabel;
class QComboBox;
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
    // close exactly these processes; `startAfter`: start SteamVR again once they are all gone (Reset SteamVR)
    void closeSteamVr(std::vector<SteamVrProc> targets, bool startAfter = false);
    // START VR's start: the inside-out session (Monado) or SteamVR
    void startVr();
    void restartSteam();
    // HEADSET DRIVER SETTINGS: shown only while CustomHeadsetOpenVR is the headset's SteamVR driver (core/chovr.h),
    // enabled when its configurator is found. Re-checked whenever the page is shown.
    void updateChovr();
    // The Tracking card's mode chip and the START TRACKING button (inside-out mode only).
    void updateMode();
    /// The original Crystal is on: it tracks itself (Lighthouse faceplate or its own inside-out), VR is SteamVR alone.
    bool crystalOn() const;
    // the running Monado session, if any: the Monado backend's main one, else "Monado directly"
    const IoSessionControl& monadoControl() const;
    void updateRuntime();
    /// The tracking mode in effect: the Crystal's own while it is on, the saved setting otherwise.
    bool insideOutNow() const;
    /// What the tracking selector shows: the original Crystal's chosen mode (crystal_tracking), else insideOutNow().
    bool selectedInsideOut() const;
    /// START VR runs the Monado inside-out session (the saved inside-out mode, not the Crystal).
    bool monadoSession() const;
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
    QComboBox* runtime_ = nullptr; // the system's active OpenXR runtime (core/openxr_runtime.h)
    std::unique_ptr<MonadoWorker> mnd_; // last: joined first, before the widgets go
    pibridge::ui::Chip* trChip_ = nullptr;
    QLabel* trMode_ = nullptr;
    pibridge::ui::Segmented* backendSel_ = nullptr; // vr_backend (SteamVR / Monado), same as the Tracking page
    pibridge::ui::Segmented* trModeSel_ = nullptr; // head-tracking mode (Lighthouse / Inside-out), same as the Tracking page
    QPushButton* ioStart_ = nullptr;
    QPushButton* ioStop_ = nullptr;
    QPushButton* roomBtn_ = nullptr;
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
    QLabel* hsSerial_ = nullptr; // the headset serial (its own, read by keepalived; else SteamVR's)
    QLabel* hsOptics_ = nullptr; // the optical module's serial (Crystal Super)
    QWidget* hsOpticsLabel_ = nullptr;
    pibridge::ui::HeadsetArt* art_ = nullptr;
    pibridge::ui::Segmented* refresh_ = nullptr;
    pibridge::ui::Segmented* refreshCrystal_ = nullptr; // the original Crystal: 72 / 90 / 120 Hz, through CHOVR
    bool wasCrystalRefresh_ = false;                    // the Crystal selector was shown last time (read the file once)
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
        // Pair / Unpair / Test vibration (core/controllers.h), like Pimax Play's controller page
        QPushButton* pair = nullptr;
        QPushButton* unpair = nullptr;
        QPushButton* buzz = nullptr;
        QLabel* opText = nullptr;
    };
    ControllerWidgets ctrlRows_[2]; // left, right
    ControllerOps ctrlOps_;
    bool ctrlSeen_[2] = {false, false}; // the controller is shown by SteamVR now (left, right)
    QTimer* ctrlOpsTimer_ = nullptr;
    void controllerAction(int slot, int action);
    // PAIR: a small guide window with the steps, START PAIRING, the 35 s progress and the result.
    void showPairDialog(int slot);
    void applyControllerOps();
    QLabel* ctrlExtra_ = nullptr;
    QLabel* trSummary_ = nullptr;
    QWidget* trStations_ = nullptr;
    QWidget* trBaseHead_ = nullptr; // the base stations' summary row (Lighthouse only)
    QLabel* trText_ = nullptr;
    QString trKey_; // what the station rows show now (rebuilt only when it changes)
    pibridge::ui::Toggle* passthrough_ = nullptr;
    pibridge::ui::Toggle* dashOverlay_ = nullptr; // Quick settings: the SteamVR dashboard panel on/off
    QLabel* dashOverlayLine_ = nullptr;           // its plain-words status
    pibridge::ui::Toggle* eyeToggle_ = nullptr;
    QPushButton* calibrate_ = nullptr;
    QPushButton* importEye_ = nullptr; // calibration.setpm from Pimax Play / EVO (Windows)
    QPushButton* recenter_ = nullptr;
    pibridge::ui::SliderRow* ipd_ = nullptr; // ipd_mm: the user's IPD in whole mm (lens motors + rendering)
    QTimer* ipdTimer_ = nullptr;             // sends the IPD 0.6 s after the slider stops
    // the original Crystal (P3B): battery row in the headset card, switch-off button next to QUICK ROOM SETUP
    QLabel* hsBatteryLabel_ = nullptr;
    pibridge::ui::Battery* hsBattery_ = nullptr;
    QLabel* hsBatteryText_ = nullptr;
    QPushButton* crystalOffBtn_ = nullptr;
    bool crystalChovrChecked_ = false;
    bool startAfterClose_ = false;           // Reset SteamVR: start it again once closed       // CHOVR's crystalOG entry enabled once per app start
    bool ipdFromHeadset_ = false;            // the slider took the lens IPD the headset reported
    double lastLensIpdMm_ = 0;               // the lens IPD last taken from the headset
    QElapsedTimer ipdSent_;                  // since our last motor request (the lenses travel meanwhile)
    pibridge::ui::Chip* eyeChip_ = nullptr;
    QLabel* eyeLine_ = nullptr;
    QLabel* toolResult_ = nullptr;
    pibridge::ui::GazeDot* gaze_ = nullptr;
    pibridge::ui::Toggle* audio_ = nullptr;
    pibridge::ui::Toggle* mic_ = nullptr;     // mic_to_headset
    QLabel* inputNow_ = nullptr;              // the default input now: headset or another
    void updateInputNow();
    SoundTestPanel* soundPanel_ = nullptr; // the TEST SPEAKERS / TEST MIC card (gated on SteamVR running)
    RecordCard* record_ = nullptr;
};

} // namespace pibridge::app
