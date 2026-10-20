// SPDX-License-Identifier: MIT
#include "logs.h"

#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QScrollBar>
#include <QVBoxLayout>

#include "../qv/layerinstall.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

QPlainTextEdit* logView() {
    auto* t = new QPlainTextEdit;
    t->setReadOnly(true);
    t->setFont(pibridge::ui::monoFont(12));
    t->setMaximumBlockCount(3000);
    t->setLineWrapMode(QPlainTextEdit::NoWrap);
    return t;
}

const char* const kFakeJournal =
    "Oct 19 10:00:01 pibridge-keepalived[4242]: headset found: Crystal Super\n"
    "Oct 19 10:00:01 pibridge-keepalived[4242]: following vrserver: not running, idle\n"
    "Oct 19 10:02:13 pibridge-keepalived[4242]: vrserver started: waking the headset\n"
    "Oct 19 10:02:15 pibridge-keepalived[4242]: panel on, 90 Hz\n"
    "Oct 19 10:02:15 pibridge-keepalived[4242]: audio: default output set to the headset\n"
    "Oct 19 10:02:16 pibridge-eyed[4243]: eye tracker found, starting (profile: user)\n"
    "Oct 19 10:02:18 pibridge-eyed[4243]: streaming, 2 eyes, 120 samples/s\n";

const char* const kFakeLayer =
    "[qv] instance: app \"Falcon BMS\" exe Falcon_BMS.exe, runtime SteamVR/OpenXR\n"
    "[qv] quad views on: edge 0.20, center 1.00, eye tracking available\n"
    "[qv] session: wide 710x768, focus 1244x1344 per eye\n"
    "[qv] settings changed: transition 0.20 edge_opacity 0.50\n";

} // namespace

LogsPage::LogsPage(bool fake, QWidget* parent) : QWidget(parent), fake_(fake) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* heading = new QLabel(QStringLiteral("Logs"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);

    auto* svc = new pibridge::ui::Card(QStringLiteral("Services"));
    svc->chip()->set(QStringLiteral("JOURNAL"), ChipKind::Info);
    svc->body()->addWidget(
        pibridge::ui::dimLabel(QStringLiteral("pibridge-keepalived and pibridge-eyed, the last 200 lines and everything new.")));
    services_ = logView();
    svc->body()->addWidget(services_, 1);
    root->addWidget(svc, 3);

    auto* lay = new pibridge::ui::Card(QStringLiteral("Quad views layer"));
    layerTimer_.setInterval(1000);
    connect(&layerTimer_, &QTimer::timeout, this, &LogsPage::readLayerLog);
    lay->body()->addWidget(pibridge::ui::dimLabel(qvgui::logPath()));
    layer_ = logView();
    lay->body()->addWidget(layer_, 1);
    root->addWidget(lay, 2);

    if (fake_) {
        services_->setPlainText(QString::fromLatin1(kFakeJournal));
        layer_->setPlainText(QString::fromLatin1(kFakeLayer));
    }
}

LogsPage::~LogsPage() {
    if (journal_) {
        journal_->kill(); // our own journalctl child only
        journal_->waitForFinished(500);
    }
}

void LogsPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    if (fake_) {
        return;
    }
    startJournal();
    readLayerLog();
    layerTimer_.start();
}

void LogsPage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    layerTimer_.stop();
}

void LogsPage::startJournal() {
    if (journal_) {
        return;
    }
    journal_ = new QProcess(this);
    journal_->setProcessChannelMode(QProcess::MergedChannels);
    connect(journal_, &QProcess::readyRead, this, [this] {
        const QString chunk = QString::fromLocal8Bit(journal_->readAll());
        services_->moveCursor(QTextCursor::End);
        services_->insertPlainText(chunk);
        services_->verticalScrollBar()->setValue(services_->verticalScrollBar()->maximum());
    });
    connect(journal_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError) {
        services_->appendPlainText(QStringLiteral("Could not run journalctl: %1").arg(journal_->errorString()));
    });
    journal_->start(QStringLiteral("journalctl"),
                    {QStringLiteral("--user"), QStringLiteral("-u"), QStringLiteral("pibridge-keepalived"),
                     QStringLiteral("-u"), QStringLiteral("pibridge-eyed"), QStringLiteral("-n"), QStringLiteral("200"),
                     QStringLiteral("-f"), QStringLiteral("--no-pager")});
}

void LogsPage::readLayerLog() {
    const QString path = qvgui::logPath();
    const QFileInfo fi(path);
    if (!fi.exists()) {
        if (layerSize_ != 0) {
            layer_->setPlainText(QStringLiteral("No log yet: no game has used the quad views layer."));
            layerSize_ = 0;
        }
        return;
    }
    if (fi.size() == layerSize_) {
        return;
    }
    layerSize_ = fi.size();
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        return;
    }
    const qint64 keep = 64 * 1024; // the tail is enough
    if (f.size() > keep) {
        f.seek(f.size() - keep);
    }
    QStringList lines = QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'));
    while (lines.size() > 200) {
        lines.removeFirst();
    }
    layer_->setPlainText(lines.join(QLatin1Char('\n')));
    layer_->verticalScrollBar()->setValue(layer_->verticalScrollBar()->maximum());
}

} // namespace pibridge::app
