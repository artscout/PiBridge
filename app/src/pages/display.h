// SPDX-License-Identifier: MIT
// Display: the display/visual settings that CustomHeadsetOpenVR (CHOVR) applies through its shader, independent of the
// tracking backend (SLAM or Lighthouse). They are written to CHOVR's ~/.config/CustomHeadset/settings.json and take
// effect the next time the VR app starts. Render quality, centre rendering and the hidden-area mask live on the
// Performance page, not here.
#pragma once

#include <QWidget>

#include "../core/actions.h"
#include "../core/conf.h"
#include "../core/state.h"

namespace pibridge::ui {
class SliderRow;
class Segmented;
} // namespace pibridge::ui

class QComboBox;
class QLabel;

namespace pibridge::app {

class DisplayPage : public QWidget {
    Q_OBJECT
public:
    DisplayPage(StateSource& source, Conf& conf, Actions& actions, QWidget* parent = nullptr);

protected:
    void showEvent(QShowEvent*) override;

private:
    void load();                    // read the CHOVR settings into the sliders
    void save();                    // write the sliders back to the CHOVR settings file
    void refreshInfo();
    void loadLens();                // the lens profile (pibridge.conf lens_profile) into the dropdown

    StateSource& source_;
    Conf& conf_;
    Actions& actions_;
    QLabel* info_ = nullptr;
    QLabel* note_ = nullptr;
    pibridge::ui::Segmented* refresh_ = nullptr; // panel refresh rate (refresh_hz), applied live via Actions
    pibridge::ui::SliderRow* brightness_ = nullptr;
    pibridge::ui::SliderRow* contrast_ = nullptr;
    pibridge::ui::SliderRow* blackLevel_ = nullptr;
    pibridge::ui::SliderRow* gamma_ = nullptr;
    pibridge::ui::SliderRow* saturation_ = nullptr;
    QComboBox* lens_ = nullptr;     // lens_profile: Crystal Super Oled chovr | Crystal Super Oled native
    QLabel* lensNote_ = nullptr;    // shown while an explicit rec_lens_conf overrides the profile
    bool loading_ = false;
};

} // namespace pibridge::app
