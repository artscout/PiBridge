// SPDX-License-Identifier: MIT
#include "sound_tests.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

#include "../core/paths.h"
#include "../core/runner.h"
#include "../core/sound_test.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

// A thin bar of ticks filled up to the level (amber near the top, as a clipping hint).
class LevelMeter : public QWidget {
public:
    explicit LevelMeter(QWidget* parent = nullptr) : QWidget(parent) { setFixedHeight(14); }
    void setLevel(float l) {
        level_ = l;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        const int n = 40;
        const double w = double(width()) / n;
        const int lit = qRound(level_ * n);
        for (int i = 0; i < n; i++) {
            const QColor c = i < lit ? (i >= n - 4 ? pibridge::ui::color::amber : pibridge::ui::color::accentText)
                                     : pibridge::ui::color::border;
            p.fillRect(QRectF(i * w + 1, 0, w - 2, height()), c);
        }
    }

private:
    float level_ = 0;
};

SoundTestPanel::SoundTestPanel(bool fake, QWidget* parent) : QWidget(parent), fake_(fake) {
    if (fake) {
        runner_ = std::make_unique<FakeRunner>();
        capture_ = std::make_unique<FakeCapture>();
    } else {
        runner_ = std::make_unique<ProcessRunner>();
        capture_ = std::make_unique<ProcessCapture>();
    }
    test_ = std::make_unique<SoundTest>(*runner_, *capture_, statusDir());

    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(8);
    // Labelled like omarchyvr's Headset sound / Headset mic tests (one plays notes, the other records and plays back).
    const auto testButton = [](const QString& text, pibridge::ui::Icon icon) {
        auto* b = pibridge::ui::toolButton(text);
        b->setFont(pibridge::ui::labelFont(11));
        b->setIcon(pibridge::ui::lineIcon(icon, pibridge::ui::color::dim));
        b->setIconSize(QSize(14, 14));
        b->setMinimumHeight(28);
        return b;
    };
    speakers_ = testButton(QStringLiteral("TEST SOUND"), pibridge::ui::Icon::Sound);
    speakers_->setToolTip(QStringLiteral("Plays a low tone on the left ear, then a high one on the right ear of the "
                                         "headset. The default output is not changed."));
    mic_ = testButton(QStringLiteral("TEST MICROPHONE"), pibridge::ui::Icon::Mic);
    mic_->setToolTip(QStringLiteral("Records %1 s from the headset's microphone and plays it back in the headset - "
                                    "start VR first, the headset mic is silent while the display is off. Nothing is kept.")
                         .arg(kMicSeconds));
    meter_ = new LevelMeter;
    meter_->setToolTip(QStringLiteral("Microphone level while recording"));
    v->addWidget(meter_);
    line_ = pibridge::ui::dimLabel(QString());
    line_->setWordWrap(true);
    // two lines always: a longer test result does not push the card (and the row) taller
    line_->setFixedHeight(2 * line_->fontMetrics().lineSpacing() + 4);
    line_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    v->addWidget(line_);

    connect(speakers_, &QPushButton::clicked, this, [this] {
        if (fake_) {
            line_->setText(QStringLiteral("Fake mode: nothing was played."));
            return;
        }
        test_->testSpeakers();
    });
    connect(mic_, &QPushButton::clicked, this, [this] {
        if (fake_) {
            line_->setText(QStringLiteral("Fake mode: nothing was recorded."));
            return;
        }
        test_->testMicrophone();
    });
    connect(test_.get(), &SoundTest::changed, this, &SoundTestPanel::update);
    connect(test_.get(), &SoundTest::levelChanged, this, [this](float l) { meter_->setLevel(l); });
    update();
}

SoundTestPanel::~SoundTestPanel() = default;

void SoundTestPanel::setVrRunning(bool on) {
    if (vrRunning_ != on) {
        vrRunning_ = on;
        update();
    }
}

void SoundTestPanel::update() {
    const bool busy = test_->busy();
    const bool ready = vrRunning_ || fake_; // the headset sound only works while SteamVR runs
    speakers_->setEnabled(!busy && ready);
    mic_->setEnabled(!busy && ready);
    if (test_->state() != SoundTest::State::Recording) {
        meter_->setLevel(0);
    }
    line_->setText(test_->message().isEmpty()
                       ? QStringLiteral("Plays and records on the headset only; your default output stays as it is.")
                       : test_->message());
    line_->setStyleSheet(test_->error() ? QStringLiteral("color:#ef5a55;") : QString());
}

} // namespace pibridge::app
