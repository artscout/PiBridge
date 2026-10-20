// SPDX-License-Identifier: MIT
// Recording: the full recording page in the left menu. It reuses the Dashboard's Record card (record_card.h) and adds
// where to save and (later) the encoding settings.
#pragma once

#include <QWidget>

class QLineEdit;

namespace pibridge::app {

class StateSource;
class IoSessionControl;
class Conf;

class RecordingPage : public QWidget {
    Q_OBJECT
public:
    RecordingPage(StateSource& source, IoSessionControl& direct, Conf& conf, QWidget* parent = nullptr);

private:
    Conf& conf_;
    QLineEdit* dir_ = nullptr;
    void chooseDir();
    void saveDir();
    QString recordDir() const;
};

} // namespace pibridge::app
