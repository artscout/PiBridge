// SPDX-License-Identifier: MIT
// The Sound settings card's tests (core/sound_test.h): TEST SPEAKERS, TEST MICROPHONE with a live level meter, and
// the result in plain words. Fake mode plays and records nothing.
#pragma once

#include <QWidget>
#include <memory>

class QLabel;
class QPushButton;

namespace pibridge::app {

class AudioCapture;
class Runner;
class SoundTest;
class LevelMeter;

class SoundTestPanel : public QWidget {
    Q_OBJECT
public:
    explicit SoundTestPanel(bool fake, QWidget* parent = nullptr);
    ~SoundTestPanel() override;

    // The tests play/record on the headset, so they only work once SteamVR runs; the Dashboard gates them.
    void setVrRunning(bool on);
    // TEST SOUND / TEST MICROPHONE: placed by the Dashboard in its Headset sound / Headset mic rows; the panel itself
    // shows the level meter and the result line.
    QPushButton* speakersButton() const { return speakers_; }
    QPushButton* micButton() const { return mic_; }

private:
    void update();

    bool fake_;
    bool vrRunning_ = false;
    std::unique_ptr<Runner> runner_;
    std::unique_ptr<AudioCapture> capture_;
    std::unique_ptr<SoundTest> test_; // after what it uses
    QPushButton* speakers_ = nullptr;
    QPushButton* mic_ = nullptr;
    LevelMeter* meter_ = nullptr;
    QLabel* line_ = nullptr;
};

} // namespace pibridge::app
