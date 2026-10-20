// SPDX-License-Identifier: MIT
// Logs: the journal of the two services (journalctl --user -f, started when the page is first shown, never in fake
// mode) and the tail of the quad views layer's log file.
#pragma once

#include <QTimer>
#include <QWidget>

class QPlainTextEdit;
class QProcess;

namespace pibridge::app {

class LogsPage : public QWidget {
    Q_OBJECT
public:
    explicit LogsPage(bool fake, QWidget* parent = nullptr);
    ~LogsPage() override;

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    void startJournal();
    void readLayerLog();

    bool fake_;
    QProcess* journal_ = nullptr;
    QPlainTextEdit* services_ = nullptr;
    QPlainTextEdit* layer_ = nullptr;
    QTimer layerTimer_;
    qint64 layerSize_ = -1;
};

} // namespace pibridge::app
