// SPDX-License-Identifier: MIT
// Performance -> Quadview foveated rendering: the settings of our quad views layer (qv-vulkan), written to
// ~/.config/qv-vulkan/settings.conf through the layer's own Settings code (app/src/qv/), per-game profiles,
// presets, the layer switch, the preview and the last game's status. Logic from artscout-qv-vulkan-linux gui/src/
// mainwindow.cpp (MIT), laid out in the app's style.
#pragma once

#include <QJsonObject>
#include <QTimer>
#include <QWidget>
#include <functional>
#include <vector>

#include "../qv/layerinstall.h"
#include "../qv/settings.h"

class QComboBox;
class QLabel;
class QPushButton;
class QVBoxLayout;

namespace qvgui {
class Preview;
}
namespace pibridge::ui {
class Segmented;
class SliderRow;
class Toggle;
} // namespace pibridge::ui

namespace pibridge::app {

class PerformancePage : public QWidget {
    Q_OBJECT
public:
    explicit PerformancePage(QWidget* parent = nullptr);
    // For tests and screenshots: the settings shown now.
    const qv::Settings& settings() const { return s_; }

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    pibridge::ui::SliderRow* percent(const QString& label, int lo, int hi, bool restartDot, const QString& tip,
                             std::function<float&(qv::Settings&)> field, QVBoxLayout* into);
    void loadProfile();
    void showSettings();
    void changed();
    void save();
    void updateQvButtons(); // enable Save/Revert when the quad views settings have unsaved edits
    void refreshStatus();
    void refreshProfiles();
    void updateBudget();
    QString fileText() const;
    bool writeFileText(const QString& text);
    QString currentSection() const;

    // CHOVR (CustomHeadsetOpenVR) render settings: an in-memory working copy edited with OK / Cancel, separate from
    // the quad views layer above (which keeps its own instant-apply behaviour and its own file).
    void loadChovr();          // read settings.json into chovrRoot_ and populate the controls quietly
    void chovrEdited();        // a control changed: mark the page dirty (does NOT write to disk)
    void applyChovr();         // OK: write chovrRoot_ to settings.json and clear the dirty flag
    void setChovrDirty(bool dirty);

    qv::Settings s_;
    bool loading_ = false;
    QTimer saveTimer_, statusTimer_;
    qvgui::Install install_;

    pibridge::ui::Toggle* enabled_ = nullptr;
    QLabel* installText_ = nullptr;
    QComboBox* profile_ = nullptr;
    QPushButton* removeProfile_ = nullptr;
    pibridge::ui::Segmented* preset_ = nullptr;
    pibridge::ui::Segmented* tracking_ = nullptr;
    pibridge::ui::Segmented* filter_ = nullptr;
    pibridge::ui::Toggle* debug_ = nullptr;
    pibridge::ui::Toggle* gazeFilter_ = nullptr;
    QLabel* budget_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* saved_ = nullptr;
    qvgui::Preview* preview_ = nullptr;
    std::vector<std::function<void()>> refreshers_;
    // Quad views settings are buffered like the CHOVR card: edits stay in s_ (and the preview) and reach the file only
    // on Save; Revert reloads the saved values.
    bool qvDirty_ = false;
    QPushButton* qvSave_ = nullptr;
    QPushButton* qvRevert_ = nullptr;

    // CHOVR render settings (settings.json), with OK / Cancel.
    QJsonObject chovrRoot_;        // the working copy; written to settings.json only on OK
    bool chovrLoading_ = false;    // true while populating the controls (suppresses chovrEdited)
    bool chovrDirty_ = false;
    pibridge::ui::SliderRow* renderQuality_ = nullptr;  // generalHeadset.renderResolutionMultiplierX/Y
    pibridge::ui::Toggle* hiddenArea_ = nullptr;        // generalHeadset.hiddenArea.enable
    QPushButton* chovrOk_ = nullptr;
    QPushButton* chovrCancel_ = nullptr;
};

} // namespace pibridge::app
