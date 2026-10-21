// SPDX-License-Identifier: MIT
// Tracking: the tracking mode (Lighthouse / Inside-out), START TRACKING (SteamVR + Monado: driver_monado.so in vrserver,
// the inside-out mode) and every setting of the Pimax driver (Basalt SLAM on the headset cameras, camera auto exposure,
// optical controller tracking), each with its one-line help, the recommended value marked, and the reason when it does
// not apply. Saved in pibridge.conf; used by START TRACKING and by Record tracking and "Monado directly" on the
// Development page (core/inside_out.h builds the one environment all use).
#pragma once

#include <QWidget>
#include <functional>
#include <vector>

#include "../core/conf.h"
#include "../core/io_session.h"
#include "../core/state.h"

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QVBoxLayout;

namespace pibridge::ui {
class Card;
class Segmented;
class Toggle;
} // namespace pibridge::ui

namespace pibridge::app {

class OptionRow;

class TrackingPage : public QWidget {
    Q_OBJECT
public:
    TrackingPage(StateSource& source, Conf& conf, IoSessionControl& control, QWidget* parent = nullptr);

protected:
    void showEvent(QShowEvent*) override;

private:
    void load();       // conf -> widgets, readiness, reasons, preview
    void save(std::string_view key, const QString& value);
    void say(const QString& text, bool error = false);
    void updateSession();
    void rebuildPaths();

    StateSource& source_;
    Conf& conf_;
    IoSessionControl& control_;
    bool loading_ = false;
    IoSettings io_;

    QLabel* message_ = nullptr;
    pibridge::ui::Segmented* mode_ = nullptr;
    QLabel* modeText_ = nullptr;
    pibridge::ui::Card* modeCard_ = nullptr;

    pibridge::ui::Card* sessionCard_ = nullptr;
    QComboBox* connector_ = nullptr;
    QLabel* drivers_ = nullptr;
    QPushButton* start_ = nullptr;
    QPushButton* stop_ = nullptr;
    QLabel* sessionMsg_ = nullptr;
    QLabel* reasons_ = nullptr;

    pibridge::ui::Toggle* slam_ = nullptr;
    pibridge::ui::Segmented* engine_ = nullptr; // io_slam_engine: Basalt | Pimax
    pibridge::ui::Segmented* grid_ = nullptr;
    pibridge::ui::Segmented* cams_ = nullptr;
    pibridge::ui::Segmented* accel_ = nullptr;
    pibridge::ui::Segmented* gyro_ = nullptr;
    pibridge::ui::Toggle* driftHold_ = nullptr;
    pibridge::ui::Toggle* stillHold_ = nullptr;
    pibridge::ui::Toggle* headTilt_ = nullptr;
    pibridge::ui::Toggle* fuseAuto_ = nullptr;
    pibridge::ui::Toggle* smooth_ = nullptr;
    pibridge::ui::Toggle* anchor_ = nullptr;
    pibridge::ui::Toggle* anchorGlide_ = nullptr;
    QLabel* slamPreview_ = nullptr;

    QComboBox* lens_ = nullptr; // lens_profile: Default (CHOVR) | Like Pimax | Pimax native (dropdown; 3 options are too wide as a segment)

    pibridge::ui::Segmented* camAe_ = nullptr;
    pibridge::ui::Segmented* camModel_ = nullptr;
    pibridge::ui::Toggle* camLaw_ = nullptr;

    pibridge::ui::Toggle* ctrlTrack_ = nullptr;
    QSpinBox* ctrlAngle_ = nullptr;
    QSpinBox* ctrlLost_ = nullptr;
    pibridge::ui::Toggle* ctrlSteady_ = nullptr;

    QLineEdit* extraEnv_ = nullptr;
    QVBoxLayout* paths_ = nullptr;
    QLabel* envPreview_ = nullptr;

    // every option row with its "is it the recommended value" test and its "why not applicable" ("" = applies)
    struct Row {
        OptionRow* row;
        std::function<bool(const IoSettings&)> isRecommended;
        std::function<QString(const IoSettings&, const RecSetup&)> notApplicable;
    };
    std::vector<Row> rows_;
};

} // namespace pibridge::app
