// SPDX-License-Identifier: MIT
// The help window: PiBridge's docs (docs/*.md, installed or from the source tree) shown inside the app in its own
// style, instead of handing the Markdown file to whatever the desktop opens it with. Links to other docs open in the
// same window (Back returns), web links go to the browser. One window for the whole app (showHelp).
#pragma once

#include <QDialog>
#include <QStringList>

class QLabel;
class QPushButton;
class QTextBrowser;

namespace pibridge::app {

class HelpWindow : public QDialog {
    Q_OBJECT
public:
    explicit HelpWindow(QWidget* parent = nullptr);
    // `doc`: a file name under docs/ ("app.md"), optionally with "#anchor".
    void open(const QString& doc);

private:
    void load(const QString& path, const QString& anchor, bool record);
    void updateButtons();

    QTextBrowser* view_ = nullptr;
    QLabel* title_ = nullptr;
    QPushButton* back_ = nullptr;
    QPushButton* home_ = nullptr;
    QStringList history_;
    QString current_;
};

// The docs directory in use (installed docs first, then the source tree), "" when neither exists.
QString docsDir();
// Opens `doc` in the app's help window (created on first use, kept for the session).
void showHelp(QWidget* parent, const QString& doc);

} // namespace pibridge::app
