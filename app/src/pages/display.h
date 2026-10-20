// SPDX-License-Identifier: MIT
// Display: the display/visual settings that CustomHeadsetOpenVR (CHOVR) applies through its shader, independent of the
// tracking backend (SLAM or Lighthouse). They are written to CHOVR's ~/.config/CustomHeadset/settings.json and take
// effect the next time the VR app starts. Render quality, centre rendering and the hidden-area mask live on the
// Performance page, not here.
#pragma once

#include <QWidget>

#include "../core/state.h"

namespace pibridge::ui {
class SliderRow;
} // namespace pibridge::ui

class QLabel;

namespace pibridge::app {

class DisplayPage : public QWidget {
    Q_OBJECT
public:
    explicit DisplayPage(StateSource& source, QWidget* parent = nullptr);

protected:
    void showEvent(QShowEvent*) override;

private:
    void load();                    // read the CHOVR settings into the sliders
    void save();                    // write the sliders back to the CHOVR settings file
    void refreshInfo();

    StateSource& source_;
    QLabel* info_ = nullptr;
    QLabel* note_ = nullptr;
    pibridge::ui::SliderRow* brightness_ = nullptr;
    pibridge::ui::SliderRow* contrast_ = nullptr;
    pibridge::ui::SliderRow* blackLevel_ = nullptr;
    pibridge::ui::SliderRow* gamma_ = nullptr;
    pibridge::ui::SliderRow* saturation_ = nullptr;
    pibridge::ui::SliderRow* ipd_ = nullptr;
    bool loading_ = false;
};

} // namespace pibridge::app
