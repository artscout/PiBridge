// SPDX-License-Identifier: MIT
#include "record_card.h"

#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStyle>
#include <QVBoxLayout>

#include "../core/io_session.h"
#include "../core/state.h"
#include "../core/system.h"
#include "../ui/icons.h"
#include "../ui/theme.h"

namespace pibridge::app {

namespace rec = pibridge::record;

namespace {
QLabel* caps(const QString& t) {
    auto* l = pibridge::ui::capsLabel(t, 12);
    l->setObjectName(QStringLiteral("dim"));
    return l;
}
// A caps label with a visible "?" help badge beside it (the hint that used to be an invisible tooltip on the control).
QWidget* capsHelp(const QString& t, const QString& tip) {
    auto* w = new QWidget;
    auto* h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(6);
    h->addWidget(caps(t));
    h->addWidget(pibridge::ui::helpBadge(tip));
    h->addStretch(1);
    return w;
}
} // namespace

RecordCard::RecordCard(StateSource& source, IoSessionControl& direct, QWidget* parent)
    : pibridge::ui::Card(QStringLiteral("Record video"), parent), source_(source), direct_(direct) {
    if (source_.fake()) {
        runner_ = std::make_unique<FakeRunner>();
    } else {
        runner_ = std::make_unique<ProcessRunner>();
    }
    auto* row = new QHBoxLayout;
    row->setSpacing(24);

    // what happens now
    auto* left = new QVBoxLayout;
    left->setSpacing(4);
    headline_ = new QLabel;
    headline_->setFont(pibridge::ui::bodyFont(15));
    left->addWidget(headline_);
    detail_ = pibridge::ui::dimLabel(QString());
    detail_->setWordWrap(true);
    left->addWidget(detail_);
    auto* lastRow = new QHBoxLayout;
    last_ = pibridge::ui::dimLabel(QString());
    last_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    lastRow->addWidget(last_, 1);
    folder_ = pibridge::ui::linkButton(QStringLiteral("Open folder"));
    folder_->setToolTip(QStringLiteral("Opens the folder of the last recording in the file manager."));
    lastRow->addWidget(folder_);
    left->addLayout(lastRow);
    left->addStretch(1);
    row->addLayout(left, 3);

    // the settings (record.toml, shared with pibridge-record)
    auto* g = new QGridLayout;
    g->setHorizontalSpacing(12);
    g->setVerticalSpacing(6);
    eyes_ = new pibridge::ui::Segmented({QStringLiteral("One eye"), QStringLiteral("Both eyes")});
    eyes_->setToolTip(QStringLiteral("One eye (the left, before the lens distortion) or both side by side."));
    fps_ = new pibridge::ui::Segmented({QStringLiteral("30"), QStringLiteral("60"), QStringLiteral("72")});
    fps_->setToolTip(QStringLiteral("Frames per second of the video."));
    quality_ = new pibridge::ui::Segmented({QStringLiteral("Low"), QStringLiteral("Medium"), QStringLiteral("High")});
    quality_->setToolTip(QStringLiteral("Bigger files look better. The GPU encoder (NVENC) is used when ffmpeg has it."));
    // System sound is always recorded now; the only audio choice is whether to add the microphone.
    mic_ = new pibridge::ui::Toggle;
    g->addWidget(capsHelp(QStringLiteral("Eyes"),
                          QStringLiteral("One eye (the left, before the lens distortion) or both side by side.")),
                 0, 0);
    g->addWidget(eyes_, 0, 1, Qt::AlignLeft);
    g->addWidget(capsHelp(QStringLiteral("FPS"), QStringLiteral("Frames per second of the video.")), 0, 2);
    g->addWidget(fps_, 0, 3, Qt::AlignLeft);
    g->addWidget(capsHelp(QStringLiteral("Quality"),
                          QStringLiteral("Bigger files look better. The GPU encoder (NVENC) is used when ffmpeg has "
                                         "it.")),
                 1, 0);
    g->addWidget(quality_, 1, 1, Qt::AlignLeft);
    // Sound (system audio) is always recorded; only the mic has a switch, in the former Sound slot.
    g->addWidget(capsHelp(QStringLiteral("Mic"),
                          QStringLiteral("Also record your microphone (the default input). The system sound is always "
                                         "recorded.")),
                 1, 2);
    g->addWidget(mic_, 1, 3, Qt::AlignLeft);
    auto* mid = new QVBoxLayout;
    mid->addLayout(g);
    mid->addStretch(1);
    row->addLayout(mid, 3);

    // the buttons
    auto* right = new QVBoxLayout;
    record_ = pibridge::ui::accentButton(QStringLiteral("RECORD"));
    record_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Play, pibridge::ui::color::text));
    record_->setMinimumHeight(30);
    record_->setFixedWidth(96); // about the width of the FPS segmented above
    record_->setCursor(Qt::PointingHandCursor);
    stop_ = pibridge::ui::toolButton(QStringLiteral("STOP"));
    stop_->setMinimumHeight(30);
    stop_->setFixedWidth(96);
    right->addWidget(record_);
    right->addWidget(stop_);
    right->addStretch(1);
    row->addLayout(right);
    body()->addLayout(row);

    connect(eyes_, &pibridge::ui::Segmented::activated, this,
            [this](int i) { change("eyes", i == 1 ? QStringLiteral("both") : QStringLiteral("left")); });
    connect(fps_, &pibridge::ui::Segmented::activated, this,
            [this](int i) { change("fps", QString::number(rec::kFpsChoices[static_cast<std::size_t>(i)])); });
    connect(quality_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        static const char* const names[] = {"low", "medium", "high"};
        change("quality", QString::fromLatin1(names[i]));
    });
    connect(mic_, &QAbstractButton::toggled, this,
            [this](bool on) { change("mic", on ? QStringLiteral("on") : QStringLiteral("off")); });
    connect(record_, &QPushButton::clicked, this, [this] {
        if (source_.fake()) {
            emit message(QStringLiteral("Fake mode: nothing was recorded."), false);
            return;
        }
        if (tool().isEmpty()) {
            emit message(QStringLiteral("pibridge-record is not installed."), true);
            return;
        }
        busy_ = true;
        record_->setEnabled(false);
        emit message(QStringLiteral("Starting the recording..."), false);
        startRecording(*runner_, tool(), [this](int code, const QString& out) {
            busy_ = false;
            emit message(code == 0 ? QStringLiteral("Recording.") : out.trimmed().section(QLatin1Char('\n'), -1)
                                                                        .remove(QStringLiteral("pibridge-record: ")),
                         code != 0);
            refresh();
        });
    });
    connect(stop_, &QPushButton::clicked, this, [this] {
        if (source_.fake()) {
            emit message(QStringLiteral("Fake mode: nothing was stopped."), false);
            return;
        }
        busy_ = true;
        stop_->setEnabled(false);
        emit message(QStringLiteral("Finishing the file..."), false);
        stopRecording(*runner_, tool(), [this](int code, const QString& out) {
            busy_ = false;
            const QString line = out.trimmed().section(QLatin1Char('\n'), -1).remove(QStringLiteral("pibridge-record: "));
            emit message(code == 0 ? QStringLiteral("Saved. %1").arg(line) : line, code != 0);
            refresh();
        });
    });
    connect(folder_, &QPushButton::clicked, this, [this] {
        if (lastFolder_.isEmpty() || source_.fake()) {
            return;
        }
        if (!runner_->startDetached(QStringLiteral("xdg-open"), {lastFolder_})) {
            emit message(QStringLiteral("Could not open %1").arg(lastFolder_), true);
        }
    });
    poll_.setInterval(1000);
    connect(&poll_, &QTimer::timeout, this, &RecordCard::refresh);
    connect(&direct_, &IoSessionControl::changed, this, &RecordCard::refresh);
    connect(&source_, &StateSource::changed, this, &RecordCard::refresh);
    applySettings();
    refresh();
}

QString RecordCard::tool() const { return source_.fake() ? QStringLiteral("/fake/pibridge-record") : findTool(QStringLiteral("pibridge-record")); }

void RecordCard::change(const char* key, const QString& value) {
    QString err;
    rec::Settings s = loadRecordSettings(&err);
    if (auto r = s.set(key, value.toStdString()); !r) {
        emit message(QString::fromStdString(r.error()), true);
        return;
    }
    if (const QString e = saveRecordSettings(s); !e.isEmpty()) {
        emit message(QStringLiteral("Could not write %1: %2").arg(recordSettingsPath(), e), true);
        applySettings();
        return;
    }
    emit message(QStringLiteral("Saved: the next recording uses it."), false);
}

void RecordCard::applySettings() {
    QString err;
    rec::Settings s = loadRecordSettings(&err);
    // System sound is always recorded now (there is no Sound switch); keep the stored setting on so the recorder adds it.
    if (!s.audio) {
        (void)s.set("audio", "on");
        (void)saveRecordSettings(s);
    }
    eyes_->setCurrent(s.eyes == rec::Eyes::Both ? 1 : 0);
    fps_->setCurrent(s.fps == 72 ? 2 : s.fps == 60 ? 1 : 0);
    quality_->setCurrent(s.quality == rec::Quality::Low ? 0 : s.quality == rec::Quality::Medium ? 1 : 2);
    mic_->setCheckedQuiet(s.mic);
    if (!err.isEmpty()) {
        emit message(QStringLiteral("The recording settings could not be read (%1): the defaults are shown.").arg(err), true);
    }
}

void RecordCard::demo(const RecordInputs& in) {
    demo_ = true;
    apply(recordCardModel(in));
}

void RecordCard::refresh() {
    if (demo_) {
        return;
    }
    RecordInputs in;
    if (source_.fake()) {
        in.directRunning = direct_.active();
        in.steamvrRunning = source_.state().steamvrRunning;
    } else {
        in = readRecordInputs(direct_.active(), source_.state().steamvrRunning);
    }
    apply(recordCardModel(in));
}

void RecordCard::apply(const RecordCardModel& m) {
    chip()->set(m.chip, m.chipKind);
    headline_->setText(m.headline);
    detail_->setText(m.detail);
    detail_->setObjectName(m.error ? QStringLiteral("error") : QStringLiteral("dim"));
    detail_->style()->unpolish(detail_);
    detail_->style()->polish(detail_);
    lastFolder_ = m.lastFolder;
    last_->setText(m.lastFile.isEmpty() ? QStringLiteral("No recording yet.")
                                        : QStringLiteral("Last file: %1").arg(QFileInfo(m.lastFile).fileName()));
    last_->setToolTip(m.lastFile);
    folder_->setVisible(!m.lastFolder.isEmpty());
    record_->setEnabled(m.canStart && !busy_);
    stop_->setEnabled(m.canStop && !busy_);
    record_->setVisible(!m.canStop);
    stop_->setVisible(m.canStop);
}

void RecordCard::showEvent(QShowEvent* e) {
    pibridge::ui::Card::showEvent(e);
    if (!demo_) {
        poll_.start();
        refresh();
    }
}

void RecordCard::hideEvent(QHideEvent* e) {
    poll_.stop();
    pibridge::ui::Card::hideEvent(e);
}

} // namespace pibridge::app
