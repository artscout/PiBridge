// SPDX-License-Identifier: MIT
#include "recording.h"

#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "../core/conf.h"
#include "../core/paths.h"
#include "../core/video_record.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"
#include "record_card.h"

namespace pibridge::app {

namespace {
QString recordTomlPath() { return configDir() + QStringLiteral("/record.toml"); }
QString defaultRecordDir() { return QDir::homePath() + QStringLiteral("/Videos/pimax"); }
} // namespace

QString RecordingPage::recordDir() const {
    QFile f(recordTomlPath());
    if (f.open(QIODevice::ReadOnly)) {
        for (const QString& raw : QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'))) {
            const QString l = raw.trimmed();
            if (l.startsWith(QLatin1String("dir"))) {
                const qsizetype eq = l.indexOf(QLatin1Char('='));
                if (eq > 0) {
                    QString v = l.mid(eq + 1).trimmed();
                    if (v.size() >= 2 && v.startsWith(QLatin1Char('"')) && v.endsWith(QLatin1Char('"'))) {
                        v = v.mid(1, v.size() - 2);
                    }
                    if (!v.isEmpty()) {
                        return v;
                    }
                }
            }
        }
    }
    return defaultRecordDir();
}

RecordingPage::RecordingPage(StateSource& source, IoSessionControl& direct, Conf& conf, QWidget* parent)
    : QWidget(parent), conf_(conf) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* heading = new QLabel(QStringLiteral("Recording"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);

    // the full record controls from the Dashboard
    root->addWidget(new RecordCard(source, direct));

    // encoding: codec + per-eye resolution, written to record.toml (pibridge-record reads them)
    namespace rec = pibridge::record;
    QString err;
    const rec::Settings cur = loadRecordSettings(&err);
    const auto setRec = [](const char* key, const QString& value) {
        rec::Settings s = loadRecordSettings();
        if (s.set(key, value.toStdString())) {
            (void)saveRecordSettings(s);
        }
    };
    auto* encCard = new pibridge::ui::Card(QStringLiteral("Encoding"));
    encCard->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "The video codec and the per-eye resolution of the recording.")));
    auto* codec = new pibridge::ui::Segmented({QStringLiteral("H.264"), QStringLiteral("H.265")});
    codec->setCurrent(cur.codec == rec::Codec::Hevc ? 1 : 0);
    connect(codec, &pibridge::ui::Segmented::activated, this,
            [setRec](int i) { setRec("codec", i == 1 ? QStringLiteral("hevc") : QStringLiteral("h264")); });
    encCard->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Codec"), codec, nullptr, false,
        QStringLiteral("H.264 plays everywhere; H.265 (HEVC) makes smaller files at the same quality.")));
    auto* res = new pibridge::ui::Segmented({QStringLiteral("1080p"), QStringLiteral("2K"), QStringLiteral("4K")});
    res->setCurrent(cur.max_height >= 2160 ? 2 : cur.max_height >= 1440 ? 1 : 0);
    connect(res, &pibridge::ui::Segmented::activated, this, [setRec](int i) {
        setRec("max_height", i == 2 ? QStringLiteral("2160") : i == 1 ? QStringLiteral("1440") : QStringLiteral("1080"));
    });
    encCard->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Resolution"), res, nullptr, false,
        QStringLiteral("Per-eye output height: 1080p, 2K (1440p) or 4K (2160p), capped by what the headset renders.")));
    root->addWidget(encCard);

    // where to save
    auto* saveCard = new pibridge::ui::Card(QStringLiteral("Save location"));
    saveCard->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "Where the recordings are written (pibridge-record --dir). The files are named pimax-YYYYmmdd-HHMMSS.mp4.")));
    auto* row = new QHBoxLayout;
    dir_ = new QLineEdit(recordDir());
    dir_->setReadOnly(true);
    row->addWidget(dir_, 1);
    auto* browse = pibridge::ui::toolButton(QStringLiteral("CHANGE..."));
    row->addWidget(browse);
    saveCard->body()->addLayout(row);
    connect(browse, &QPushButton::clicked, this, &RecordingPage::chooseDir);
    root->addWidget(saveCard);
    root->addStretch(1);
}

void RecordingPage::chooseDir() {
    const QString d = QFileDialog::getExistingDirectory(this, QStringLiteral("Where to save recordings"), dir_->text());
    if (d.isEmpty()) {
        return;
    }
    dir_->setText(d);
    saveDir();
}

void RecordingPage::saveDir() {
    // read record.toml, replace or add the `dir =` line, write it back
    const QString path = recordTomlPath();
    QStringList lines;
    QFile in(path);
    if (in.open(QIODevice::ReadOnly)) {
        lines = QString::fromUtf8(in.readAll()).split(QLatin1Char('\n'));
        in.close();
    }
    const QString line = QStringLiteral("dir = \"%1\"").arg(dir_->text());
    bool replaced = false;
    for (QString& l : lines) {
        if (l.trimmed().startsWith(QLatin1String("dir"))) {
            l = line;
            replaced = true;
            break;
        }
    }
    if (!replaced) {
        lines.append(line);
    }
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile out(path);
    if (out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        out.write(lines.join(QLatin1Char('\n')).toUtf8());
    }
}

} // namespace pibridge::app
