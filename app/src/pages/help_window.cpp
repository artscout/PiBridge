// SPDX-License-Identifier: MIT
#include "help_window.h"

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QPointer>
#include <QImageReader>
#include <QPushButton>
#include <QRegularExpression>
#include <QTextBlock>
#include <QTextBrowser>
#include <QTextCursor>
#include <QTextFrame>
#include <QTextTable>
#include <QTextDocument>
#include <QUrl>
#include <QVBoxLayout>

#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

// Qt's Markdown import drops the text that follows a raw HTML block (the guide's centred screenshots,
// `<p align="center"><img src=".." alt=".." width=".."></p>`): such a line becomes a plain Markdown image, any other
// HTML tag line is left out.
QString withoutHtml(const QString& md) {
    static const QRegularExpression img(QStringLiteral(R"re(<img\s[^>]*src="([^"]+)"[^>]*?(?:alt="([^"]*)")?[^>]*>)re"));
    QStringList out;
    for (const QString& line : md.split(QLatin1Char('\n'))) {
        const QString t = line.trimmed();
        if (!t.startsWith(QLatin1Char('<'))) {
            out << line;
        } else if (const auto m = img.match(t); m.hasMatch()) {
            out << QStringLiteral("![%1](%2)").arg(m.captured(2), m.captured(1));
        }
    }
    return out.join(QLatin1Char('\n'));
}

// Qt's Markdown import ignores style sheets: the app's look is put on the parsed document instead. Headings get their
// size on every fragment (a `code` span in a heading kept the body size), code its colours, links the accent, tables a
// border.
void restyle(QTextDocument* doc, const QString& dir) {
    namespace c = pibridge::ui::color;
    static const int headingPx[] = {0, 26, 19, 16, 14, 13, 13};
    // collected first, applied after: changing formats while walking the fragments would invalidate the walk
    struct Change {
        int pos, len;
        QTextCharFormat cf;
    };
    std::vector<Change> changes;
    std::vector<std::pair<int, int>> headings; // block position, level
    for (QTextBlock b = doc->begin(); b.isValid(); b = b.next()) {
        const int level = std::clamp(b.blockFormat().headingLevel(), 0, 6);
        if (level > 0) {
            headings.emplace_back(b.position(), level);
        }
        for (auto it = b.begin(); !it.atEnd(); ++it) {
            const QTextFragment f = it.fragment();
            if (!f.isValid()) {
                continue;
            }
            QTextCharFormat cf;
            const QTextCharFormat old = f.charFormat();
            if (old.isImageFormat()) {
                // screenshots at most 820 px wide (the guide's own width), the height in proportion
                QTextImageFormat im = old.toImageFormat();
                const QSize size = QImageReader(QDir(dir).absoluteFilePath(im.name())).size();
                if (size.isValid() && size.width() > 820) {
                    im.setWidth(820);
                    im.setHeight(820.0 * size.height() / size.width());
                    changes.push_back({f.position(), f.length(), im});
                }
                continue;
            }
            const bool code = old.fontFixedPitch() ||
                              old.fontFamilies().toStringList().contains(QStringLiteral("monospace"));
            if (level > 0) {
                cf.setProperty(QTextFormat::FontPixelSize, headingPx[level]);
                cf.setFontWeight(level == 1 ? QFont::Light : (level == 2 ? QFont::Normal : QFont::DemiBold));
            }
            if (old.isAnchor()) {
                cf.setForeground(c::accentText);
                cf.setFontUnderline(false);
            } else if (code) {
                cf.setForeground(QColor(0xc8, 0xcc, 0xf0));
                cf.setBackground(c::nav);
            } else {
                // after a raw HTML block (an <img> in the guide) Qt's import goes on in the HTML default, black
                cf.setForeground(c::text);
            }
            changes.push_back({f.position(), f.length(), cf});
        }
    }
    for (const Change& ch : changes) {
        QTextCursor cur(doc);
        cur.setPosition(ch.pos);
        cur.setPosition(ch.pos + ch.len, QTextCursor::KeepAnchor);
        if (ch.cf.isImageFormat()) {
            cur.setCharFormat(ch.cf);
        } else {
            cur.mergeCharFormat(ch.cf);
        }
    }
    // paragraphs a little apart (the import puts none between them)
    for (QTextBlock b = doc->begin(); b.isValid(); b = b.next()) {
        if (b.blockFormat().headingLevel() == 0 && b.textList() == nullptr && !b.text().isEmpty()) {
            QTextCursor cur(b);
            QTextBlockFormat bf = cur.blockFormat();
            bf.setBottomMargin(8);
            bf.setLineHeight(118, QTextBlockFormat::ProportionalHeight);
            cur.setBlockFormat(bf);
        }
    }
    for (const auto& [pos, level] : headings) {
        QTextCursor cur(doc);
        cur.setPosition(pos);
        QTextBlockFormat bf = cur.blockFormat();
        bf.setTopMargin(level <= 2 ? 18 : 12);
        bf.setBottomMargin(6);
        cur.setBlockFormat(bf);
    }
    // tables: a thin border in the card colour, some padding
    QList<QTextFrame*> frames{doc->rootFrame()};
    while (!frames.isEmpty()) {
        QTextFrame* fr = frames.takeFirst();
        if (auto* t = qobject_cast<QTextTable*>(fr)) {
            QTextTableFormat tf = t->format();
            tf.setBorder(1);
            tf.setBorderBrush(c::border);
            tf.setBorderStyle(QTextFrameFormat::BorderStyle_Solid);
            tf.setBorderCollapse(true);
            tf.setCellPadding(5);
            tf.setCellSpacing(0);
            t->setFormat(tf);
        }
        frames.append(fr->childFrames());
    }
}

} // namespace

QString docsDir() {
    // the source tree this binary was built from first (a development build; an older install may sit next to it), then
    // the installed docs (a package: no source tree)
    for (const QString& d : {QStringLiteral(PIBRIDGE_SOURCE_DOCS), QStringLiteral(PIBRIDGE_INSTALLED_DOCS)}) {
        if (QFileInfo::exists(d + QStringLiteral("/USER-GUIDE.md"))) {
            return d;
        }
    }
    return {};
}

HelpWindow::HelpWindow(QWidget* parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("PiBridge help"));
    setModal(false);
    resize(920, 760);
    setStyleSheet(QStringLiteral("QDialog { background: %1; }").arg(pibridge::ui::color::bg.name()));

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 14, 18, 14);
    root->setSpacing(10);
    auto* head = new QHBoxLayout;
    head->setSpacing(8);
    back_ = pibridge::ui::toolButton(QStringLiteral("BACK"));
    back_->setFont(pibridge::ui::labelFont(11));
    home_ = pibridge::ui::toolButton(QStringLiteral("CONTENTS"));
    home_->setFont(pibridge::ui::labelFont(11));
    home_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Help, pibridge::ui::color::dim));
    title_ = new QLabel;
    title_->setFont(pibridge::ui::labelFont(12));
    title_->setObjectName(QStringLiteral("dim"));
    head->addWidget(back_);
    head->addWidget(home_);
    head->addSpacing(8);
    head->addWidget(title_, 1);
    root->addLayout(head);

    view_ = new QTextBrowser;
    view_->setOpenLinks(false); // every link goes through anchorClicked
    view_->setFrameShape(QFrame::NoFrame);
    view_->setStyleSheet(QStringLiteral("QTextBrowser { background: %1; border: 1px solid %2; border-radius: 6px; "
                                        "padding: 10px 14px; }")
                             .arg(pibridge::ui::color::panel.name(), pibridge::ui::color::border.name()));
    root->addWidget(view_, 1);

    connect(view_, &QTextBrowser::anchorClicked, this, [this](const QUrl& url) {
        if (url.scheme() == QLatin1String("http") || url.scheme() == QLatin1String("https") ||
            url.scheme() == QLatin1String("mailto")) {
            QDesktopServices::openUrl(url);
            return;
        }
        if (url.path().isEmpty() && url.hasFragment()) {
            view_->scrollToAnchor(url.fragment());
            return;
        }
        const QString base = QFileInfo(current_).absolutePath();
        const QString path = QDir::cleanPath(QDir(base).absoluteFilePath(url.path()));
        if (path.endsWith(QLatin1String(".md")) && QFileInfo::exists(path)) {
            load(path, url.fragment(), true);
        } else if (QFileInfo::exists(path)) {
            QDesktopServices::openUrl(QUrl::fromLocalFile(path)); // an image, a script: the desktop's viewer
        }
    });
    connect(back_, &QPushButton::clicked, this, [this] {
        if (!history_.isEmpty()) {
            load(history_.takeLast(), {}, false);
        }
    });
    connect(home_, &QPushButton::clicked, this, [this] { open(QStringLiteral("USER-GUIDE.md")); });
    updateButtons();
}

void HelpWindow::open(const QString& doc) {
    const QString dir = docsDir();
    const qsizetype hash = doc.indexOf(QLatin1Char('#'));
    const QString name = hash < 0 ? doc : doc.left(hash);
    const QString anchor = hash < 0 ? QString() : doc.mid(hash + 1);
    if (dir.isEmpty()) {
        view_->setMarkdown(QStringLiteral("# Help not found\n\nThe documentation is not installed (%1).")
                               .arg(QStringLiteral(PIBRIDGE_INSTALLED_DOCS)));
    } else {
        load(dir + QLatin1Char('/') + name, anchor, !current_.isEmpty());
    }
    show();
    raise();
    activateWindow();
}

void HelpWindow::load(const QString& path, const QString& anchor, bool record) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        return;
    }
    if (record && !current_.isEmpty() && current_ != path) {
        history_.append(current_);
    }
    current_ = path;
    view_->setSearchPaths({QFileInfo(path).absolutePath()}); // relative images resolve next to the doc
    view_->setMarkdown(withoutHtml(QString::fromUtf8(f.readAll())));
    restyle(view_->document(), QFileInfo(path).absolutePath());
    title_->setText(QFileInfo(path).fileName());
    if (anchor.isEmpty()) {
        view_->moveCursor(QTextCursor::Start);
    } else {
        view_->scrollToAnchor(anchor);
    }
    updateButtons();
}

void HelpWindow::updateButtons() {
    back_->setEnabled(!history_.isEmpty());
}

void showHelp(QWidget* parent, const QString& doc) {
    static QPointer<HelpWindow> w;
    if (!w) {
        w = new HelpWindow(parent != nullptr ? parent->window() : nullptr);
    }
    w->open(doc);
}

} // namespace pibridge::app
