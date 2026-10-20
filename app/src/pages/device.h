// SPDX-License-Identifier: MIT
// The Device page (core/device_info.h): the headset's model, serial number and IPD, the controllers' and base
// stations' serial numbers, and how the app renders now. Serials are shown only here (with COPY), never logged.
#pragma once

#include <QWidget>
#include <array>

#include "../core/conf.h"
#include "../core/controllers.h"
#include "../core/device_info.h"
#include "../core/io_session.h"
#include "../core/state.h"

class QGridLayout;
class QLabel;
class QPushButton;
class QTimer;

namespace pibridge::ui {
class Card;
}

namespace pibridge::app {

class DevicePage : public QWidget {
    Q_OBJECT
public:
    DevicePage(StateSource& source, Conf& conf, IoSessionControl& io, IoSessionControl& direct,
               QWidget* parent = nullptr);

protected:
    void showEvent(QShowEvent*) override;

private:
    // Re-reads the files (CustomHeadsetOpenVR, steamvr.vrsettings, quad views, the Monado build): on show only.
    void readFiles();
    void apply();
    // The Pair / Unpair / Test vibration rows (core/controllers.h).
    void applyControllers();
    void controllerAction(int slot, int action);

    StateSource& source_;
    Conf& conf_;
    IoSessionControl& io_;
    IoSessionControl& direct_;
    DeviceInputs files_; // the part of the inputs that comes from files
    pibridge::ui::Card* headset_ = nullptr;
    QGridLayout* headsetRows_ = nullptr;
    QGridLayout* deviceRows_ = nullptr;
    QLabel* devicesNote_ = nullptr;
    QGridLayout* renderRows_ = nullptr;
    QString key_; // what the rows show now (rebuilt only when it changes)
    ControllerOps ctrl_;
    QTimer* ctrlTimer_ = nullptr;
    struct CtrlRow {
        QLabel* state = nullptr;
        QLabel* text = nullptr;
        QPushButton* pair = nullptr;
        QPushButton* unpair = nullptr;
        QPushButton* buzz = nullptr;
    };
    std::array<CtrlRow, 2> ctrlRows_{};
};

} // namespace pibridge::app
