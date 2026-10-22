// SPDX-License-Identifier: MIT
// Development: RECORD TRACKING. Records a EuRoC dataset (cameras + IMU) from a Monado 6DoF session for offline SLAM
// analysis, with guided steps read out loud (the user wears the headset), then checks it; the past recordings with
// Open folder / Delete. The logic is in core/rec_*.h; this page only shows it and ticks the session.
// Also here: the Monado build every start uses (the A/B choice of the Rust-linked and the PiBridge-linked build, its
// markers, its cap_sys_nice and the Fix permissions button; core/monado_build.h) and "Monado directly", the debug /
// A-B session where monado-service owns the headset (core/io_session.h, IoVariant::MonadoDirect).
#pragma once

#include <QElapsedTimer>
#include <QTimer>
#include <QWidget>
#include <memory>

#include "../core/actions.h"
#include "../core/conf.h"
#include "../core/io_session.h"
#include "../core/rec_session.h"
#include "../core/state.h"

class QLabel;
class QPushButton;
class QVBoxLayout;
class QGridLayout;

namespace pibridge::ui {
class Card;
class Chip;
class Segmented;
class Toggle;
} // namespace pibridge::ui

namespace pibridge::app {

class DevelopmentPage : public QWidget {
    Q_OBJECT
public:
    DevelopmentPage(StateSource& source, Conf& conf, IoSessionControl& direct, Actions& actions,
                    QWidget* parent = nullptr);
    ~DevelopmentPage() override;

    // Fake mode only (screenshots): start a guided recording and jump `ms` into its steps; finish() stops it.
    void demoRecording(qint64 ms);
    void demoFinish();
    // Another session blocks the recording (the tracking session): the reason, "" = none.
    void setExternalBlock(const QString& why);

signals:
    void recordingActiveChanged(bool active);

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    qint64 now() const { return clock_.elapsed() + offsetMs_; }
    void refreshSetup();
    void refreshRecordings();
    void startRecording();
    void tick();
    void updateView();
    void openPath(const QString& path);
    void say(const QString& text, bool error = false);
    std::vector<RecStep> protocol(QString* error) const;
    RecProbe fakeProbe();
    void updateBuild();
    void updateDirect();
    void fixPermissions();
    void importEyeCalibration();
    QString home() const;

    StateSource& source_;
    Conf& conf_;
    IoSessionControl& direct_;
    Actions& actions_;
    bool fake_;
    std::unique_ptr<Runner> runner_; // speech only
    std::unique_ptr<Announcer> announcer_;
    std::unique_ptr<RecProcesses> procs_;
    std::unique_ptr<RecSession> session_;
    RecSetup setup_;
    RecReadiness readiness_;
    QElapsedTimer clock_;
    qint64 offsetMs_ = 0;
    QTimer tickTimer_;
    QTimer refreshTimer_;
    qint64 fakeStartMs_ = -1;
    bool fakeWritten_ = false;
    QString externalBlock_;
    bool lastActive_ = false;

    pibridge::ui::Card* recCard_ = nullptr;
    pibridge::ui::Segmented* mode_ = nullptr;
    QPushButton* record_ = nullptr;
    QPushButton* stopTop_ = nullptr;
    QLabel* message_ = nullptr;
    QLabel* reasons_ = nullptr;
    QPushButton* sessionLogs_ = nullptr;

    pibridge::ui::Card* stepsCard_ = nullptr;
    QWidget* idleView_ = nullptr;
    QLabel* plan_ = nullptr;
    QWidget* runView_ = nullptr;
    QLabel* stepText_ = nullptr;
    QLabel* countdown_ = nullptr;
    QLabel* stepInfo_ = nullptr;
    QLabel* next_ = nullptr;
    QPushButton* skip_ = nullptr;
    QPushButton* stop_ = nullptr;

    pibridge::ui::Card* buildCard_ = nullptr;
    pibridge::ui::Segmented* build_ = nullptr;
    QLabel* buildPath_ = nullptr;
    QLabel* buildWhat_ = nullptr;
    QLabel* buildDriver_ = nullptr;
    QLabel* capLine_ = nullptr;
    QPushButton* fixCaps_ = nullptr;
    bool fixRunning_ = false;

    pibridge::ui::Card* directCard_ = nullptr;
    pibridge::ui::Toggle* home_ = nullptr;
    QLabel* homeLine_ = nullptr;
    QPushButton* directStart_ = nullptr;
    QPushButton* directStop_ = nullptr;
    QLabel* directMsg_ = nullptr;
    QLabel* directReasons_ = nullptr;

    pibridge::ui::Card* listCard_ = nullptr;
    QVBoxLayout* list_ = nullptr;
    QLabel* setupView_ = nullptr;
    QPushButton* importEye_ = nullptr; // Import calibration from Windows (pibridge-eye profile)
    QLabel* importResult_ = nullptr;
};

} // namespace pibridge::app
