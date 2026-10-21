// SPDX-License-Identifier: MIT
// The Dashboard's Record card: Record / Stop, eyes, frame rate, quality, the sound switch, and the last file with
// "Open folder". The words and the commands are in core/video_record.h; the recorder is pibridge-record (docs/record.md).
#pragma once

#include <QTimer>

#include "../core/runner.h"
#include "../core/video_record.h"
#include "../ui/widgets.h"

#include <memory>

class QLabel;
class QPushButton;

namespace pibridge::app {

class IoSessionControl;
class StateSource;

class RecordCard : public pibridge::ui::Card {
    Q_OBJECT
public:
    // `direct`: "Monado directly" (Development page), the session the recorder works with.
    RecordCard(StateSource& source, IoSessionControl& direct, QWidget* parent = nullptr);
    // Fake mode / screenshots: show these inputs instead of the files and /proc.
    void demo(const RecordInputs& in);
    void refresh();

public slots:
    // From the SteamVR dashboard overlay: start (true) / stop (false) a recording, the same as the RECORD / STOP
    // buttons and only when that is allowed right now (otherwise it does nothing).
    void recordFromVr(bool start);
    // From the overlay's segment controls: a recording setting (key = view | fps | max_height) to write to record.toml,
    // so a later start uses it. Ignores an unknown key.
    void setFromVr(const QString& key, const QString& value);

signals:
    // A message for the Dashboard's message line.
    void message(const QString& text, bool error);

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    void apply(const RecordCardModel& m);
    void applySettings();
    void doStart(); // the RECORD button / overlay start
    void doStop();  // the STOP button / overlay stop
    void change(const char* key, const QString& value);
    QString tool() const;

    StateSource& source_;
    IoSessionControl& direct_;
    std::unique_ptr<Runner> runner_;
    QTimer poll_;
    bool busy_ = false;
    bool canStart_ = false; // the RECORD action is allowed now (mirrors the button's enabled state)
    bool canStop_ = false;  // the STOP action is allowed now (a recording is running)
    bool demo_ = false;
    QString lastFolder_;
    QLabel* headline_ = nullptr;
    QLabel* detail_ = nullptr;
    QLabel* last_ = nullptr;
    QPushButton* record_ = nullptr;
    QPushButton* stop_ = nullptr;
    QPushButton* folder_ = nullptr;
    pibridge::ui::Segmented* eyes_ = nullptr;
    pibridge::ui::Segmented* fps_ = nullptr;
    pibridge::ui::Segmented* quality_ = nullptr;
    pibridge::ui::Toggle* mic_ = nullptr;
};

} // namespace pibridge::app
