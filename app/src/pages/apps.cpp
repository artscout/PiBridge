// SPDX-License-Identifier: MIT
#include "apps.h"

#include <QAction>
#include <QContextMenuEvent>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QImageReader>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QStyle>
#include <QDir>
#include <QVBoxLayout>
#include <chrono>

#include "../core/runner.h"
#include "../core/state.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

constexpr int kTileW = 200;
constexpr int kCoverH = 300; // 2:3, the shape of Steam's library capsules
constexpr int kTileH = kCoverH + 96;

qint64 steadyMs() {
    return static_cast<qint64>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}

QLabel* sectionTitle(const QString& text) {
    auto* l = new QLabel(text);
    QFont f = pibridge::ui::bodyFont(17);
    f.setWeight(QFont::Medium);
    f.setCapitalization(QFont::AllUppercase);
    l->setFont(f);
    return l;
}

// The first letters of up to two words ("Echo Canyon VR" -> "EC").
QString initials(const QString& name) {
    QString out;
    for (const QString& w : name.split(QLatin1Char(' '), Qt::SkipEmptyParts)) {
        if (w.front().isLetterOrNumber()) {
            out += w.front().toUpper();
        }
        if (out.size() == 2) {
            break;
        }
    }
    return out.isEmpty() ? QStringLiteral("?") : out;
}

// Images are read once at tile size and kept (the library cache files are a few hundred KB each).
QImage loadImage(const QString& path, const QSize& fit) {
    static QHash<QString, QImage> cache;
    const QString key = QStringLiteral("%1@%2x%3").arg(path).arg(fit.width()).arg(fit.height());
    if (const auto it = cache.constFind(key); it != cache.cend()) {
        return *it;
    }
    QImageReader r(path);
    r.setAutoTransform(true);
    const QSize sz = r.size();
    if (sz.isValid() && (sz.width() > fit.width() * 2 || sz.height() > fit.height() * 2)) {
        r.setScaledSize(sz.scaled(fit * 2, Qt::KeepAspectRatioByExpanding));
    }
    const QImage img = r.read();
    if (cache.size() > 400) {
        cache.clear();
    }
    cache.insert(key, img);
    return img;
}

// Fake mode: a generated cover image (gradient, rings and the title) written into the fake mode's temporary
// cache dir, so the screenshots go through the same image code as Steam's real covers. Never a real path.
QString makeFakeCover(const QString& file, QSize size, const QString& title, int hue) {
    const QString dir = qEnvironmentVariable("XDG_CACHE_HOME");
    if (dir.isEmpty()) {
        return {};
    }
    const QString path = dir + QStringLiteral("/pibridge-fake-covers/") + file;
    QDir().mkpath(QFileInfo(path).absolutePath());
    QImage img(size, QImage::Format_RGB32);
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    QLinearGradient g(0, 0, size.width(), size.height());
    g.setColorAt(0, QColor::fromHsv(hue, 160, 150));
    g.setColorAt(1, QColor::fromHsv((hue + 60) % 360, 200, 40));
    p.fillRect(img.rect(), g);
    p.setPen(QPen(QColor(255, 255, 255, 60), 3));
    for (int i = 1; i <= 6; i++) {
        const double r = i * size.width() / 6.0;
        p.drawEllipse(QPointF(size.width() * 0.7, size.height() * 0.3), r, r);
    }
    QFont f = pibridge::ui::titleFont(size.width() / 9);
    f.setWeight(QFont::DemiBold);
    p.setFont(f);
    p.setPen(Qt::white);
    p.drawText(img.rect().adjusted(16, 16, -16, -16), Qt::AlignHCenter | Qt::AlignBottom | Qt::TextWordWrap, title);
    p.end();
    return img.save(path) ? path : QString();
}

} // namespace

QIcon appsNavIcon(const QColor& normal, const QColor& checked) {
    const auto draw = [](const QColor& c) {
        QPixmap pm(QSize(40, 40));
        pm.fill(Qt::transparent);
        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QPen(c, 2.6));
        p.setBrush(Qt::NoBrush);
        for (int y = 0; y < 2; y++) {
            for (int x = 0; x < 2; x++) {
                p.drawRoundedRect(QRectF(6 + x * 15, 6 + y * 15, 12, 12), 2.5, 2.5);
            }
        }
        return pm;
    };
    QIcon icon;
    icon.addPixmap(draw(normal), QIcon::Normal, QIcon::Off);
    icon.addPixmap(draw(checked), QIcon::Normal, QIcon::On);
    return icon;
}

// ---------------------------------------------------------------- flow layout (tiles in rows that wrap)

class FlowLayout : public QLayout {
public:
    explicit FlowLayout(QWidget* parent, int spacing) : QLayout(parent), spacing_(spacing) {
        setContentsMargins(0, 0, 0, 0);
    }
    ~FlowLayout() override {
        while (QLayoutItem* i = takeAt(0)) {
            delete i;
        }
    }
    void addItem(QLayoutItem* item) override { items_.push_back(item); }
    int count() const override { return static_cast<int>(items_.size()); }
    QLayoutItem* itemAt(int i) const override {
        return i >= 0 && i < count() ? items_[static_cast<size_t>(i)] : nullptr;
    }
    QLayoutItem* takeAt(int i) override {
        if (i < 0 || i >= count()) {
            return nullptr;
        }
        QLayoutItem* it = items_[static_cast<size_t>(i)];
        items_.erase(items_.begin() + i);
        return it;
    }
    Qt::Orientations expandingDirections() const override { return {}; }
    bool hasHeightForWidth() const override { return true; }
    int heightForWidth(int w) const override { return place(QRect(0, 0, w, 0), false); }
    void setGeometry(const QRect& r) override {
        QLayout::setGeometry(r);
        place(r, true);
    }
    QSize sizeHint() const override { return minimumSize(); }
    QSize minimumSize() const override {
        QSize s;
        for (QLayoutItem* i : items_) {
            s = s.expandedTo(i->minimumSize());
        }
        return s;
    }

private:
    int place(const QRect& r, bool apply) const {
        int x = r.x(), y = r.y(), rowH = 0;
        for (QLayoutItem* i : items_) {
            if (i->widget() && i->widget()->isHidden()) {
                continue;
            }
            const QSize s = i->sizeHint();
            if (x > r.x() && x + s.width() > r.right() + 1) {
                x = r.x();
                y += rowH + spacing_;
                rowH = 0;
            }
            if (apply) {
                i->setGeometry(QRect(QPoint(x, y), s));
            }
            x += s.width() + spacing_;
            rowH = std::max(rowH, s.height());
        }
        return y + rowH - r.y();
    }

    std::vector<QLayoutItem*> items_;
    int spacing_;
};

// ---------------------------------------------------------------- cover art

class CoverArt : public QWidget {
public:
    explicit CoverArt(QWidget* parent) : QWidget(parent) { setFixedSize(kTileW, kCoverH); }
    void set(const QString& name, const QString& image, bool portrait, bool isIcon) {
        name_ = name;
        image_ = image;
        portrait_ = portrait;
        icon_ = isIcon;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.setRenderHint(QPainter::SmoothPixmapTransform);
        const QRectF r = rect();
        QPainterPath clip;
        clip.addRoundedRect(r.adjusted(1, 1, -1, 0), 2, 2);
        p.setClipPath(clip);
        const QImage img = image_.isEmpty() ? QImage() : loadImage(image_, size());
        if (img.isNull() || icon_) {
            placeholder(p, r, img);
        } else if (portrait_) {
            drawFilled(p, r, img);
        } else {
            // a wide header: a dark, enlarged copy behind it, the header itself across the middle
            drawFilled(p, r, img);
            p.fillRect(r, QColor(8, 8, 12, 225));
            const QSizeF hs = QSizeF(img.size()).scaled(r.size(), Qt::KeepAspectRatio);
            const QRectF hr(r.left(), r.center().y() - hs.height() / 2 - 20, hs.width(), hs.height());
            p.drawImage(hr, img);
        }
        // a soft fade to the panel at the bottom, so the name below reads as part of the tile
        QLinearGradient fade(0, r.bottom() - 46, 0, r.bottom());
        fade.setColorAt(0, QColor(20, 20, 23, 0));
        fade.setColorAt(1, QColor(20, 20, 23, 170));
        p.fillRect(QRectF(r.left(), r.bottom() - 46, r.width(), 46), fade);
    }

private:
    static void drawFilled(QPainter& p, const QRectF& r, const QImage& img) {
        const QSizeF s = QSizeF(img.size()).scaled(r.size(), Qt::KeepAspectRatioByExpanding);
        p.drawImage(QRectF(r.center().x() - s.width() / 2, r.center().y() - s.height() / 2, s.width(), s.height()), img);
    }

    void placeholder(QPainter& p, const QRectF& r, const QImage& icon) const {
        // our own generated art: a muted gradient from the name, facet lines, the initials or the app's icon
        const int hue = static_cast<int>(qHash(name_) % 360u);
        QLinearGradient g(r.topLeft(), r.bottomRight());
        g.setColorAt(0, QColor::fromHsv(hue, 120, 92));
        g.setColorAt(1, QColor::fromHsv((hue + 40) % 360, 140, 34));
        p.fillRect(r, g);
        p.setPen(QPen(QColor(255, 255, 255, 26), 1.2));
        const QPointF c(r.center().x(), r.height() * 0.42);
        for (int i = 0; i < 6; i++) {
            const double a = i * 3.14159265 / 3 + hue * 0.01;
            p.drawLine(c, QPointF(c.x() + std::cos(a) * r.width(), c.y() + std::sin(a) * r.width()));
        }
        if (!icon.isNull()) {
            const QSizeF s = QSizeF(icon.size()).scaled(QSizeF(120, 120), Qt::KeepAspectRatio);
            p.drawImage(QRectF(c.x() - s.width() / 2, c.y() - s.height() / 2, s.width(), s.height()), icon);
            return;
        }
        QFont f = pibridge::ui::titleFont(64);
        p.setFont(f);
        p.setPen(QColor(255, 255, 255, 215));
        p.drawText(QRectF(r.left(), c.y() - 50, r.width(), 100), Qt::AlignCenter, initials(name_));
    }

    QString name_, image_;
    bool portrait_ = true;
    bool icon_ = false;
};

// ---------------------------------------------------------------- tile

class AppTile : public QFrame {
public:
    enum Kind { Steam, Own };
    AppTile(Kind kind, QString id, QWidget* parent = nullptr) : QFrame(parent), kind_(kind), id_(std::move(id)) {
        setObjectName(QStringLiteral("card"));
        setFixedSize(kTileW, kTileH);
        auto* v = new QVBoxLayout(this);
        v->setContentsMargins(0, 0, 0, 10);
        v->setSpacing(4);
        cover_ = new CoverArt(this);
        v->addWidget(cover_);
        chip_ = new pibridge::ui::Chip(cover_);
        chip_->move(8, 8);
        vrBadge_ = new QLabel(QStringLiteral("VR"), cover_);
        vrBadge_->setFont(pibridge::ui::labelFont(11, QFont::DemiBold));
        vrBadge_->setStyleSheet(QStringLiteral("QLabel { background: rgba(12,12,16,190); color: #c9cfff; "
                                               "border: 1px solid #4a5ae8; border-radius: 2px; padding: 1px 5px; }"));
        vrBadge_->adjustSize();
        vrBadge_->move(kTileW - vrBadge_->width() - 8, 8);
        auto* text = new QVBoxLayout;
        text->setContentsMargins(12, 2, 12, 0);
        text->setSpacing(2);
        name_ = new QLabel;
        QFont nf = pibridge::ui::bodyFont(14);
        nf.setWeight(QFont::Medium);
        name_->setFont(nf);
        text->addWidget(name_);
        info_ = new QLabel;
        info_->setFont(pibridge::ui::bodyFont(12));
        info_->setObjectName(QStringLiteral("dim"));
        text->addWidget(info_);
        v->addLayout(text);
        v->addStretch(1);
        auto* row = new QHBoxLayout;
        row->setContentsMargins(10, 0, 10, 0);
        row->setSpacing(6);
        if (kind_ == Own) {
            edit_ = pibridge::ui::linkButton(QStringLiteral("Edit"));
            edit_->setToolTip(QStringLiteral("Change the program, its arguments, environment and quad views."));
            row->addWidget(edit_);
        } else {
            auto* steam = new QLabel(QStringLiteral("STEAM"));
            steam->setFont(pibridge::ui::labelFont(11));
            steam->setObjectName(QStringLiteral("dim"));
            row->addWidget(steam);
        }
        row->addStretch(1);
        play_ = pibridge::ui::accentButton(QStringLiteral("Play"));
        play_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Play, QColor(0xe2, 0xe6, 0xff)));
        play_->setMinimumWidth(92);
        row->addWidget(play_);
        v->addLayout(row);
    }

    Kind kind() const { return kind_; }
    const QString& id() const { return id_; }
    const QString& name() const { return nameText_; }
    QPushButton* play() const { return play_; }
    QPushButton* edit() const { return edit_; }
    bool vr() const { return vr_; }
    bool stopMode() const { return stop_; }

    void setApp(const QString& name, const QString& image, bool portrait, bool isIcon, bool vr) {
        nameText_ = name;
        vr_ = vr;
        cover_->set(name, image, portrait, isIcon);
        name_->setText(QFontMetrics(name_->font()).elidedText(name, Qt::ElideRight, kTileW - 24));
        name_->setToolTip(name);
        vrBadge_->setVisible(vr && kind_ == Steam);
    }

    // chip over the cover, the line under the name (red for an error), and the button: PLAY, or STOP for a
    // running own app, or a disabled PLAY while a Steam game runs or an app is starting.
    void setState(const ChipText& chip, const QString& line, bool error, bool canStop, bool canPlay) {
        chip_->set(chip);
        chip_->setVisible(!chip.text.isEmpty()); // Chip::set skips an unchanged (also the first, empty) text
        chip_->adjustSize();
        info_->setObjectName(error ? QStringLiteral("error") : QStringLiteral("dim"));
        info_->style()->unpolish(info_);
        info_->style()->polish(info_);
        info_->setText(QFontMetrics(info_->font()).elidedText(line, Qt::ElideRight, kTileW - 24));
        info_->setToolTip(line);
        stop_ = canStop;
        if (canStop) {
            play_->setObjectName(QStringLiteral("tool"));
            play_->setText(QStringLiteral("Stop"));
            play_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Power, pibridge::ui::color::text));
        } else {
            play_->setObjectName(QStringLiteral("accent"));
            play_->setText(QStringLiteral("Play"));
            play_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Play, QColor(0xe2, 0xe6, 0xff)));
        }
        play_->style()->unpolish(play_);
        play_->style()->polish(play_);
        play_->setEnabled(canStop || canPlay);
    }

    std::function<void(QContextMenuEvent*)> onMenu;

protected:
    void contextMenuEvent(QContextMenuEvent* e) override {
        if (onMenu) {
            onMenu(e);
        }
    }

private:
    Kind kind_;
    QString id_;
    QString nameText_;
    bool vr_ = false;
    bool stop_ = false;
    CoverArt* cover_;
    pibridge::ui::Chip* chip_;
    QLabel* vrBadge_;
    QLabel* name_;
    QLabel* info_;
    QPushButton* edit_ = nullptr;
    QPushButton* play_;
};

// ---------------------------------------------------------------- the editor

namespace {

class AppEditor : public QDialog {
public:
    AppEditor(const OwnApp& a, bool isNew, QWidget* parent) : QDialog(parent), app_(a) {
        setWindowTitle(isNew ? QStringLiteral("Add app") : QStringLiteral("Edit %1").arg(a.name));
        setMinimumWidth(640);
        setStyleSheet(QStringLiteral("QDialog { background: #141417; } QPlainTextEdit { font-family: monospace; }"));
        auto* v = new QVBoxLayout(this);
        v->setContentsMargins(18, 16, 18, 14);
        v->setSpacing(10);
        auto* title = new QLabel(isNew ? QStringLiteral("Add app") : QStringLiteral("Edit app"));
        title->setFont(pibridge::ui::titleFont(22));
        v->addWidget(title);
        v->addWidget(pibridge::ui::dimLabel(QStringLiteral(
            "A program PiBridge starts for you: when SteamVR is not fully ready yet, PiBridge starts it first and waits for "
            "the headset and the compositor. Its output goes to a log file.")));
        auto* form = new QFormLayout;
        form->setLabelAlignment(Qt::AlignRight);
        form->setHorizontalSpacing(14);
        form->setVerticalSpacing(8);
        name_ = new QLineEdit(a.name);
        name_->setPlaceholderText(QStringLiteral("Falcon BMS"));
        form->addRow(label(QStringLiteral("Name")), name_);
        exec_ = new QLineEdit(a.exec);
        exec_->setPlaceholderText(QStringLiteral("The program or script to start"));
        form->addRow(label(QStringLiteral("Program")), withBrowse(exec_, [this] {
                         const QString f = QFileDialog::getOpenFileName(this, QStringLiteral("Program"),
                                                                        start(exec_->text()));
                         if (!f.isEmpty()) {
                             exec_->setText(f);
                         }
                     }));
        args_ = new QLineEdit(joinArgs(a.args));
        args_->setPlaceholderText(QStringLiteral("-xr   (quote arguments with spaces: \"two words\")"));
        form->addRow(label(QStringLiteral("Arguments")), args_);
        workdir_ = new QLineEdit(a.workdir);
        workdir_->setPlaceholderText(QStringLiteral("Empty: the program's folder"));
        form->addRow(label(QStringLiteral("Working folder")), withBrowse(workdir_, [this] {
                         const QString d = QFileDialog::getExistingDirectory(this, QStringLiteral("Working folder"),
                                                                             start(workdir_->text()));
                         if (!d.isEmpty()) {
                             workdir_->setText(d);
                         }
                     }));
        env_ = new QPlainTextEdit(a.env.join(QLatin1Char('\n')));
        env_->setPlaceholderText(QStringLiteral("KEY=value, one per line"));
        env_->setFixedHeight(84);
        form->addRow(label(QStringLiteral("Environment")), env_);
        icon_ = new QLineEdit(a.icon);
        icon_->setPlaceholderText(QStringLiteral("Optional image for the tile"));
        form->addRow(label(QStringLiteral("Icon")), withBrowse(icon_, [this] {
                         const QString f = QFileDialog::getOpenFileName(
                             this, QStringLiteral("Icon"), start(icon_->text()),
                             QStringLiteral("Images (*.png *.jpg *.jpeg *.svg *.webp *.ico);;All files (*)"));
                         if (!f.isEmpty()) {
                             icon_->setText(f);
                         }
                     }));
        auto* qvRow = new QHBoxLayout;
        qvRow->setSpacing(10);
        qv_ = new pibridge::ui::Toggle;
        qv_->setChecked(a.quadViews);
        const QString qvTip = QStringLiteral("Off: the quad views layer is not loaded for this app "
                                             "(DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1 in its environment).");
        qv_->setToolTip(qvTip);
        qvRow->addWidget(qv_);
        qvRow->addWidget(pibridge::ui::helpBadge(qvTip));
        qvRow->addWidget(pibridge::ui::dimLabel(QStringLiteral("Quad views foveated rendering for this app")), 1);
        form->addRow(label(QStringLiteral("Quad views")), qvRow);
        auto* profRow = new QHBoxLayout;
        profRow->setSpacing(8);
        profile_ = new QLineEdit(a.qvProfile);
        profile_->setPlaceholderText(QStringLiteral("Empty: the program's name"));
        profile_->setToolTip(QStringLiteral("The name of its profile on the Performance page (the game's process "
                                            "name, e.g. \"Falcon BMS\")."));
        profRow->addWidget(profile_, 1);
        auto* openProfile = pibridge::ui::linkButton(QStringLiteral("Quad views profile..."));
        openProfile->setToolTip(QStringLiteral("Opens the Performance page with this app's profile."));
        profRow->addWidget(openProfile);
        form->addRow(label(QStringLiteral("Profile")), profRow);
        v->addLayout(form);
        problems_ = new QLabel;
        problems_->setObjectName(QStringLiteral("error"));
        problems_->setWordWrap(true);
        problems_->hide();
        v->addWidget(problems_);
        auto* buttons = new QHBoxLayout;
        if (!isNew) {
            auto* remove = pibridge::ui::toolButton(QStringLiteral("Remove"));
            connect(remove, &QPushButton::clicked, this, [this] {
                removed_ = true;
                accept();
            });
            buttons->addWidget(remove);
        }
        buttons->addStretch(1);
        auto* cancel = pibridge::ui::toolButton(QStringLiteral("Cancel"));
        auto* save = pibridge::ui::accentButton(QStringLiteral("Save"));
        save->setDefault(true);
        buttons->addWidget(cancel);
        buttons->addWidget(save);
        v->addLayout(buttons);
        connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
        connect(save, &QPushButton::clicked, this, [this] {
            const QStringList p = validateOwnApp(result());
            if (!p.isEmpty()) {
                problems_->setText(p.join(QLatin1Char(' ')));
                problems_->show();
                return;
            }
            accept();
        });
        connect(openProfile, &QPushButton::clicked, this, [this] {
            wantsProfile_ = true;
            if (validateOwnApp(result()).isEmpty()) {
                accept(); // saved, then the page switches
            } else {
                reject();
            }
        });
    }

    OwnApp result() const {
        OwnApp a = app_;
        a.name = name_->text().trimmed();
        a.exec = exec_->text().trimmed();
        a.args = splitArgs(args_->text());
        a.workdir = workdir_->text().trimmed();
        a.env.clear();
        for (const QString& l : env_->toPlainText().split(QLatin1Char('\n'))) {
            if (!l.trimmed().isEmpty()) {
                a.env << l.trimmed();
            }
        }
        a.icon = icon_->text().trimmed();
        a.quadViews = qv_->isChecked();
        a.qvProfile = profile_->text().trimmed();
        return a;
    }
    bool removed() const { return removed_; }
    bool wantsProfile() const { return wantsProfile_; }

private:
    static QLabel* label(const QString& t) {
        auto* l = pibridge::ui::capsLabel(t, 12);
        l->setObjectName(QStringLiteral("dim"));
        return l;
    }
    static QString start(const QString& current) {
        const QString p = expandHome(current.trimmed());
        return p.isEmpty() ? QDir::homePath() : QFileInfo(p).absolutePath();
    }
    QWidget* withBrowse(QLineEdit* edit, std::function<void()> browse) {
        auto* w = new QWidget;
        auto* h = new QHBoxLayout(w);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(8);
        h->addWidget(edit, 1);
        auto* b = pibridge::ui::toolButton(QStringLiteral("Browse..."));
        connect(b, &QPushButton::clicked, this, std::move(browse));
        h->addWidget(b);
        return w;
    }

    OwnApp app_;
    QLineEdit* name_;
    QLineEdit* exec_;
    QLineEdit* args_;
    QLineEdit* workdir_;
    QPlainTextEdit* env_;
    QLineEdit* icon_;
    pibridge::ui::Toggle* qv_;
    QLineEdit* profile_;
    QLabel* problems_;
    bool removed_ = false;
    bool wantsProfile_ = false;
};

} // namespace

// ---------------------------------------------------------------- the page

AppsPage::AppsPage(StateSource& source, Runner& runner, Conf* conf, IoSessionControl* io, IoSessionControl* tracking,
                   QWidget* parent)
    : QWidget(parent), source_(source), runner_(runner), fake_(source.fake()), io_(io), tracking_(tracking),
      store_(appsConfigPath()) {
    setObjectName(QStringLiteral("page"));
    if (fake_) {
        procs_ = std::make_unique<FakeAppProcesses>();
    } else {
        procs_ = std::make_unique<QtAppProcesses>();
    }
    // fake mode: no time passes between the fake state and the screenshot, so no settle time
    ready_ = std::make_unique<SteamVrReady>(SteamVrReady::Clock{}, fake_ ? 0 : kReadySettleMs);
    launcher_ = std::make_unique<AppLauncher>(
        runner_, *procs_, *ready_, [this] { return source_.state().steamvrRunning; }, appsLogDir());
    if (conf && tracking) {
        runtime_ = std::make_unique<TrackingRuntime>(*conf, *tracking, io);
        launcher_->setRuntime(runtime_.get());
    }

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);

    // ---- toolbar: search, VR only, SteamVR readiness, Add app
    auto* bar = new QHBoxLayout;
    bar->setSpacing(10);
    search_ = new QLineEdit;
    search_->setPlaceholderText(QStringLiteral("Search"));
    search_->setClearButtonEnabled(true);
    search_->setFixedWidth(260);
    bar->addWidget(search_);
    bar->addSpacing(8);
    vrOnly_ = new pibridge::ui::Toggle;
    vrOnly_->setChecked(true);
    const QString vrOnlyTip =
        QStringLiteral("Show only the Steam games SteamVR lists as VR apps. Your own apps are always shown.");
    vrOnly_->setToolTip(vrOnlyTip);
    bar->addWidget(vrOnly_);
    auto* vrl = pibridge::ui::capsLabel(QStringLiteral("VR only"), 12);
    bar->addWidget(vrl);
    bar->addWidget(pibridge::ui::helpBadge(vrOnlyTip));
    vrOnlyNote_ = new QLabel;
    vrOnlyNote_->setObjectName(QStringLiteral("dim"));
    vrOnlyNote_->setFont(pibridge::ui::bodyFont(12));
    bar->addWidget(vrOnlyNote_);
    bar->addStretch(1);
    readyChip_ = new pibridge::ui::Chip;
    readyChip_->hide();
    bar->addWidget(readyChip_);
    readyWords_ = new QLabel;
    readyWords_->setObjectName(QStringLiteral("dim"));
    bar->addWidget(readyWords_);
    bar->addSpacing(8);
    auto* add = pibridge::ui::accentButton(QStringLiteral("Add app..."));
    add->setToolTip(QStringLiteral("Add a program of your own (a script, a Wine game, a VR tool)."));
    bar->addWidget(add);
    root->addLayout(bar);
    message_ = new QLabel;
    message_->setObjectName(QStringLiteral("dim"));
    message_->hide();
    root->addWidget(message_);
    messageTimer_.setSingleShot(true);
    connect(&messageTimer_, &QTimer::timeout, message_, &QLabel::hide);

    // ---- the tiles, in a scroll area
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto* content = new QWidget;
    auto* cv = new QVBoxLayout(content);
    cv->setContentsMargins(0, 0, 10, 10);
    cv->setSpacing(10);
    auto* steamHead = new QHBoxLayout;
    steamTitle_ = sectionTitle(QStringLiteral("Steam"));
    steamHead->addWidget(steamTitle_);
    steamHead->addStretch(1);
    cv->addLayout(steamHead);
    steamEmpty_ = pibridge::ui::dimLabel(QString());
    cv->addWidget(steamEmpty_);
    steamGrid_ = new QWidget;
    steamFlow_ = new FlowLayout(steamGrid_, 14);
    cv->addWidget(steamGrid_);
    cv->addSpacing(8);
    ownTitle_ = sectionTitle(QStringLiteral("My apps"));
    cv->addWidget(ownTitle_);
    ownEmpty_ = pibridge::ui::dimLabel(QStringLiteral(
        "Programs of your own (a script that starts a Wine game, a VR tool) start here once SteamVR is fully ready. "
        "Add one with \"Add app...\"."));
    cv->addWidget(ownEmpty_);
    ownGrid_ = new QWidget;
    ownFlow_ = new FlowLayout(ownGrid_, 14);
    cv->addWidget(ownGrid_);
    cv->addStretch(1);
    scroll->setWidget(content);
    root->addWidget(scroll, 1);

    connect(search_, &QLineEdit::textChanged, this, &AppsPage::applyFilter);
    connect(vrOnly_, &QAbstractButton::toggled, this, &AppsPage::applyFilter);
    connect(add, &QPushButton::clicked, this, [this] { editApp(QString()); });
    connect(launcher_.get(), &AppLauncher::changed, this, [this](const QString& id) {
        updateRuntimeNow();
        for (AppTile* t : tiles_) {
            if (t->id() == id) {
                refreshTile(t);
            }
        }
    });
    connect(launcher_.get(), &AppLauncher::played, this, [this](const QString& id) {
        const qint64 now = QDateTime::currentSecsSinceEpoch();
        if (!fake_) {
            store_.load(); // the file may have been edited by hand since: change only this one value
        }
        if (auto a = store_.doc().find(id)) {
            a->lastPlayed = now;
            store_.doc().upsert(*a);
            if (!fake_) {
                saveStore();
            }
        } else {
            for (SteamGame& g : steam_.games) {
                if (g.appId == id) {
                    g.lastPlayed = now; // Steam writes its own files; this only updates the tile at once
                }
            }
        }
        for (AppTile* t : tiles_) {
            if (t->id() == id) {
                refreshTile(t);
            }
        }
    });
    connect(ready_.get(), &SteamVrReady::changed, this, [this] {
        updateReadyLine();
        refreshTiles();
    });
    connect(&source_, &StateSource::changed, this, [this] {
        ready_->setInputs(readyInputsFrom(source_.state(), fake_));
        if (fake_) {
            showFakeStates();
        }
    });
    ready_->setInputs(readyInputsFrom(source_.state(), fake_));
    for (IoSessionControl* c : {io_, tracking_}) {
        if (!c) {
            continue;
        }
        // a session moves on: the waiting apps start, the chips follow
        connect(c, &IoSessionControl::changed, this, [this] {
            launcher_->pollMonado();
            updateReadyLine();
            refreshTiles();
            if (fake_) {
                showFakeStates();
            }
        });
    }

    runningTimer_.setInterval(2000);
    connect(&runningTimer_, &QTimer::timeout, this, [this] {
        if (fake_) {
            return;
        }
        const QSet<QString> now = runningSteamApps();
        if (now != steamRunning_) {
            steamRunning_ = now;
        }
        refreshTiles(); // also ends the STARTING time of a Steam game
    });

    if (fake_) {
        steam_ = fakeSteamLibrary(QDateTime::currentSecsSinceEpoch());
        // one portrait capsule, one wide header, one without art (the initials)
        steam_.games[0].cover = makeFakeCover(QStringLiteral("900001.png"), {600, 900}, steam_.games[0].name, 250);
        steam_.games[0].coverPortrait = true;
        steam_.games[1].cover = makeFakeCover(QStringLiteral("900002.png"), {460, 215}, steam_.games[1].name, 20);
        steam_.games[1].coverPortrait = false;
        for (const OwnApp& a : fakeOwnApps(QDateTime::currentSecsSinceEpoch())) {
            store_.doc().upsert(a); // in memory only: fake mode writes nothing
        }
        rebuild();
        showFakeStates();
    } else {
        store_.load();
        rebuild();
    }
    updateReadyLine();
}

AppsPage::~AppsPage() {
    launcher_.reset(); // its pending waits refer to ready_
}

void AppsPage::showFakeStates() {
    // SteamVR on: a game and an app running; off: an app waiting for SteamVR and one that gave up.
    // Monado directly: an app running on Monado while tracking runs, nothing while it is off.
    AppRunState a, b;
    if (launcher_->target() == AppTarget::Monado) {
        if (io_ && io_->monadoReadiness().ready()) {
            a.run = AppRun::Running;
            a.pid = 4321;
        }
        steamRunning_.clear();
    } else if (source_.state().steamvrRunning) {
        a.run = AppRun::Running;
        a.pid = 4321;
        steamRunning_ = {QStringLiteral("900001")};
    } else {
        a.run = AppRun::StartingSteamVr;
        b.run = AppRun::Error;
        b.message = QStringLiteral("SteamVR does not see the headset");
        steamRunning_.clear();
    }
    launcher_->setOwnStateForShow(QStringLiteral("flight-sim"), a);
    launcher_->setOwnStateForShow(QStringLiteral("model-viewer"), b);
    refreshTiles();
}

void AppsPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    if (!fake_) {
        // re-read the library when the page is opened (at most every 10 s: it is a few small files)
        const qint64 now = steadyMs();
        if (lastScanMs_ == 0 || now - lastScanMs_ > 10000) {
            rescan();
        }
        steamRunning_ = runningSteamApps();
    }
    // the tracking mode may have changed on the Tracking page
    if (fake_) {
        showFakeStates();
    }
    refreshTiles();
    updateReadyLine();
    applyFilter();
    runningTimer_.start();
}

void AppsPage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    runningTimer_.stop();
}

void AppsPage::rescan() {
    lastScanMs_ = steadyMs();
    steam_ = readSteamLibrary();
    store_.load();
    rebuild();
}

void AppsPage::rebuild() {
    for (AppTile* t : tiles_) {
        delete t;
    }
    tiles_.clear();
    const auto menuFor = [this](AppTile* t) {
        t->onMenu = [this, t](QContextMenuEvent* e) {
            QMenu m(this);
            const QString id = t->id();
            if (t->kind() == AppTile::Steam) {
                const bool vr = t->vr();
                QAction* play = m.addAction(QStringLiteral("Play"), this, [this, id, vr] {
                    if (!fake_) {
                        launcher_->launchSteam(id, vr);
                    }
                });
                play->setEnabled(steamBlockedWhy(launcher_->target()).isEmpty());
            } else {
                if (t->stopMode()) {
                    m.addAction(QStringLiteral("Stop"), this, [this, id] { launcher_->stopOwn(id); });
                } else {
                    m.addAction(QStringLiteral("Play"), this, [this, id] { playOwn(id); });
                }
                m.addAction(QStringLiteral("Edit..."), this, [this, id] { editApp(id); });
                m.addAction(QStringLiteral("Quad views profile..."), this, [this, id] {
                    if (auto a = store_.doc().find(id)) {
                        emit quadViewsProfileRequested(qvProfileName(*a));
                    }
                });
                QAction* log = m.addAction(QStringLiteral("Open log"), this, [this, id] { openLog(id); });
                log->setEnabled(!fake_ && QFileInfo::exists(appsLogDir() + QLatin1Char('/') + id + QStringLiteral("-latest.log")));
                m.addSeparator();
                m.addAction(QStringLiteral("Remove"), this, [this, id] { removeApp(id); });
            }
            m.exec(e->globalPos());
        };
    };
    for (const SteamGame& g : steam_.games) {
        auto* t = new AppTile(AppTile::Steam, g.appId);
        t->setApp(g.name, g.cover, g.coverPortrait, false, g.vr);
        const QString id = g.appId;
        const bool vr = g.vr;
        connect(t->play(), &QPushButton::clicked, this, [this, id, vr] {
            if (fake_) {
                say(QStringLiteral("Fake mode: nothing is started."));
                return;
            }
            if (launcher_->steamState(id, {}).run == AppRun::StartingSteamVr) {
                launcher_->cancelSteam(id);
                return;
            }
            if (const QString why = steamBlockedWhy(launcher_->target()); !why.isEmpty()) {
                say(why, true);
                return;
            }
            if (!launcher_->launchSteam(id, vr)) {
                say(QStringLiteral("Could not start Steam. Is it installed?"), true);
            }
        });
        menuFor(t);
        steamFlow_->addWidget(t);
        tiles_.push_back(t);
    }
    for (const OwnApp& a : store_.doc().apps()) {
        auto* t = new AppTile(AppTile::Own, a.id);
        const QString icon = expandHome(a.icon);
        t->setApp(a.name, icon, false, true, true);
        const QString id = a.id;
        connect(t->play(), &QPushButton::clicked, this, [this, id, t] {
            if (t->stopMode()) {
                if (!launcher_->stopOwn(id)) {
                    say(QStringLiteral("Could not stop it: it is not running any more."), true);
                }
                return;
            }
            playOwn(id);
        });
        connect(t->edit(), &QPushButton::clicked, this, [this, id] { editApp(id); });
        menuFor(t);
        ownFlow_->addWidget(t);
        tiles_.push_back(t);
    }
    refreshTiles();
    applyFilter();
}

void AppsPage::playOwn(const QString& id) {
    const auto a = store_.doc().find(id);
    if (!a) {
        return;
    }
    if (fake_) {
        say(QStringLiteral("Fake mode: nothing is started."));
        return;
    }
    launcher_->launchOwn(*a);
}

void AppsPage::updateRuntimeNow() {
    target_ = launcher_->target();
    monadoNow_ = target_ == AppTarget::Monado && io_ ? io_->monadoReadiness() : MonadoReadiness{};
}

void AppsPage::refreshTiles() {
    updateRuntimeNow();
    for (AppTile* t : tiles_) {
        refreshTile(t);
    }
}

void AppsPage::refreshTile(AppTile* t) {
    const bool monado = target_ == AppTarget::Monado;
    const MonadoReadiness& mr = monadoNow_;
    const bool ready = monado ? mr.ready() : ready_->current().ready();
    const QDateTime now = QDateTime::currentDateTime();
    if (t->kind() == AppTile::Steam && monado) {
        // no OpenVR-to-OpenXR layer for Monado here: Play is off, with the reason (nothing is ever started)
        const QString why = steamBlockedWhy(target_);
        t->setState(ChipText{}, QStringLiteral("Needs SteamVR"), false, false, false);
        t->play()->setToolTip(why);
        return;
    }
    if (t->kind() == AppTile::Steam) {
        const AppRunState s = launcher_->steamState(t->id(), steamRunning_);
        qint64 played = 0;
        for (const SteamGame& g : steam_.games) {
            if (g.appId == t->id()) {
                played = g.lastPlayed;
            }
        }
        const bool err = s.run == AppRun::Error;
        const QString line = err ? s.message : s.run == AppRun::Running ? QStringLiteral("Running") : lastPlayedWords(played, now);
        // Steam starts SteamVR itself for a VR game, so READY here only says SteamVR is up already
        t->setState(appChip(s, ready), line, err, false, s.run != AppRun::Running && s.run != AppRun::Starting);
        return;
    }
    const AppRunState s = launcher_->ownState(t->id());
    const auto a = store_.doc().find(t->id());
    QString line = appRunWords(s);
    if (s.run == AppRun::StartingTracking) {
        line = mr.words.isEmpty() ? s.message : mr.words;
    } else if (s.run == AppRun::StartingSteamVr) {
        line = ready_->current().step == ReadyStep::NoServer ? QStringLiteral("Starting SteamVR...") : ready_->current().words;
    }
    if (line.isEmpty()) {
        line = lastPlayedWords(a ? a->lastPlayed : 0, now);
    }
    const bool err = s.run == AppRun::Error || (s.run == AppRun::Exited && (s.crashed || s.exitCode != 0));
    const bool stop = s.run == AppRun::Running || s.run == AppRun::StartingSteamVr || s.run == AppRun::StartingTracking;
    t->setState(appChip(s, ready), line, err, stop, s.run != AppRun::Starting);
    if (s.run == AppRun::StartingTracking) {
        t->play()->setText(QStringLiteral("Cancel"));
        t->play()->setToolTip(QStringLiteral("Stop waiting for Monado (tracking itself keeps running)."));
    } else if (s.run == AppRun::StartingSteamVr) {
        t->play()->setText(QStringLiteral("Cancel"));
        t->play()->setToolTip(QStringLiteral("Stop waiting for SteamVR (SteamVR itself keeps running)."));
    } else if (stop) {
        t->play()->setToolTip(QStringLiteral("Asks the program to quit (SIGTERM to PID %1 only).").arg(s.pid));
    } else if (monado) {
        t->play()->setToolTip(ready ? QStringLiteral("Starts the program on Monado now.")
                                    : QStringLiteral("Starts tracking when it is off, waits for Monado, then starts the "
                                                     "program on Monado (never SteamVR)."));
    } else {
        t->play()->setToolTip(ready ? QStringLiteral("Starts the program now.")
                                    : QStringLiteral("Starts SteamVR, waits until it is fully ready, then starts the program."));
    }
}

void AppsPage::applyFilter() {
    const QString q = search_->text().trimmed();
    const bool vrOnly = vrOnly_->isChecked();
    int steamShown = 0, ownShown = 0;
    for (AppTile* t : tiles_) {
        const bool match = q.isEmpty() || t->name().contains(q, Qt::CaseInsensitive);
        // without SteamVR's list nothing is known about VR: every (non-tool) game is shown
        const bool vrOk = t->kind() == AppTile::Own || !vrOnly || !steam_.vrKnown || t->vr();
        t->setVisible(match && vrOk);
        if (match && vrOk) {
            (t->kind() == AppTile::Steam ? steamShown : ownShown)++;
        }
    }
    vrOnlyNote_->setText(vrOnly && steam_.found && !steam_.vrKnown
                             ? QStringLiteral("(SteamVR has not listed its VR games yet: all games are shown)")
                             : QString());
    const int steamTotal = static_cast<int>(steam_.games.size());
    steamTitle_->setText(steamTotal == 0 ? QStringLiteral("Steam")
                                         : QStringLiteral("Steam  ·  %1 of %2").arg(steamShown).arg(steamTotal));
    if (!steam_.found) {
        steamEmpty_->setText(QStringLiteral("Steam was not found (no ~/.local/share/Steam or ~/.steam/steam)."));
    } else if (steamTotal == 0) {
        steamEmpty_->setText(QStringLiteral("No installed Steam games."));
    } else if (steamShown == 0) {
        steamEmpty_->setText(vrOnly ? QStringLiteral("No VR game matches. Switch off \"VR only\" to see every game.")
                                    : QStringLiteral("No game matches the search."));
    } else {
        steamEmpty_->clear();
    }
    steamEmpty_->setVisible(!steamEmpty_->text().isEmpty());
    if (!steam_.errors.isEmpty()) {
        steamTitle_->setToolTip(QStringLiteral("Could not read: ") + steam_.errors.join(QStringLiteral("; ")));
    }
    const bool anyOwn = !store_.doc().apps().empty();
    ownEmpty_->setText(anyOwn ? QStringLiteral("No app matches the search.")
                       : launcher_->target() == AppTarget::Monado
                           ? QStringLiteral("Programs of your own (an OpenXR game, a VR tool) start here on Monado once "
                                            "tracking runs. Add one with \"Add app...\".")
                           : QStringLiteral("Programs of your own (a script that starts a Wine game, a VR tool) start "
                                            "here once SteamVR is fully ready. Add one with \"Add app...\"."));
    ownEmpty_->setVisible(ownShown == 0);
    steamGrid_->updateGeometry();
    ownGrid_->updateGeometry();
}

void AppsPage::updateReadyLine() {
    if (launcher_->target() == AppTarget::Monado) {
        const MonadoReadiness m = io_ ? io_->monadoReadiness() : MonadoReadiness{};
        ChipText chip;
        switch (m.step) {
        case MonadoStep::Ready: chip = {QStringLiteral("TRACKING READY"), ChipKind::Info}; break;
        case MonadoStep::Off: chip = {QStringLiteral("TRACKING OFF"), ChipKind::Off}; break;
        case MonadoStep::Failed: chip = {QStringLiteral("TRACKING STOPPED"), ChipKind::Off}; break;
        case MonadoStep::NoRuntimeFile: chip = {QStringLiteral("NO MONADO RUNTIME"), ChipKind::Error}; break;
        default: chip = {QStringLiteral("STARTING TRACKING"), ChipKind::Warn}; break;
        }
        readyChip_->set(chip);
        readyChip_->setVisible(true);
        readyWords_->setText(m.ready() ? QStringLiteral("Apps run on Monado")
                             : m.step == MonadoStep::Off ? QStringLiteral("Play starts it")
                                                         : m.words);
        readyChip_->setToolTip(QStringLiteral("\"Monado directly\" runs (Development page). Own apps get "
                                              "XR_RUNTIME_JSON=%1; SteamVR is not used.")
                                   .arg(m.runtimeJson.isEmpty() ? QStringLiteral("(none found)") : m.runtimeJson));
        return;
    }
    const Readiness& r = ready_->current();
    ChipText chip;
    switch (r.step) {
    case ReadyStep::Ready: chip = {QStringLiteral("STEAMVR READY"), ChipKind::Info}; break;
    case ReadyStep::NotInstalled: chip = {QStringLiteral("NO STEAMVR"), ChipKind::Off}; break;
    case ReadyStep::NoServer: chip = {QStringLiteral("STEAMVR OFF"), ChipKind::Off}; break;
    default: chip = {QStringLiteral("STEAMVR STARTING"), ChipKind::Warn}; break;
    }
    readyChip_->set(chip);
    readyChip_->setVisible(true);
    const bool viaTracking = runtime_ && runtime_->steamVrThroughTracking();
    readyWords_->setText(r.ready()                       ? QString()
                         : r.step == ReadyStep::NoServer ? (viaTracking ? QStringLiteral("Play starts tracking (SteamVR + Monado)")
                                                                        : QStringLiteral("Play starts it"))
                                                         : r.words);
    readyChip_->setToolTip(r.detail.isEmpty() ? r.words : QStringLiteral("%1 (%2)").arg(r.words, r.detail));
}

void AppsPage::say(const QString& text, bool error) {
    message_->setObjectName(error ? QStringLiteral("error") : QStringLiteral("dim"));
    message_->style()->unpolish(message_);
    message_->style()->polish(message_);
    message_->setText(text);
    message_->show();
    messageTimer_.start(error ? 15000 : 6000);
}

bool AppsPage::saveStore() {
    if (fake_) {
        return true; // fake mode writes nothing
    }
    QString err;
    if (!store_.save(&err)) {
        say(err, true);
        return false;
    }
    return true;
}

void AppsPage::editApp(const QString& id) {
    OwnApp a;
    const bool isNew = id.isEmpty();
    if (!fake_) {
        store_.load(); // edit what is on disk now (hand edits and unknown keys stay)
    }
    if (!isNew) {
        const auto found = store_.doc().find(id);
        if (!found) {
            return;
        }
        a = *found;
    }
    AppEditor dlg(a, isNew, this);
    if (dlg.exec() != QDialog::Accepted) {
        return;
    }
    if (dlg.removed()) {
        removeApp(id);
        return;
    }
    const OwnApp edited = dlg.result();
    if (!fake_) {
        store_.load(); // the dialog may have been open a while
    }
    const QString newId = store_.doc().upsert(edited);
    if (saveStore()) {
        say(isNew ? QStringLiteral("%1 added.").arg(edited.name) : QStringLiteral("%1 saved.").arg(edited.name));
    }
    rebuild();
    if (dlg.wantsProfile()) {
        if (auto saved = store_.doc().find(newId)) {
            emit quadViewsProfileRequested(qvProfileName(*saved));
        }
    }
}

void AppsPage::removeApp(const QString& id) {
    if (!fake_) {
        store_.load();
    }
    const auto a = store_.doc().find(id);
    if (!a) {
        return;
    }
    if (QMessageBox::question(this, QStringLiteral("Remove app"),
                              QStringLiteral("Remove %1 from PiBridge? The program itself is not touched.").arg(a->name)) !=
        QMessageBox::Yes) {
        return;
    }
    store_.doc().remove(id);
    if (saveStore()) {
        say(QStringLiteral("%1 removed.").arg(a->name));
    }
    rebuild();
}

void AppsPage::openLog(const QString& id) {
    if (fake_) {
        return;
    }
    const QString link = appsLogDir() + QLatin1Char('/') + id + QStringLiteral("-latest.log");
    if (!runner_.startDetached(QStringLiteral("xdg-open"), {QFileInfo(link).canonicalFilePath()})) {
        say(QStringLiteral("Could not open %1").arg(link), true);
    }
}

} // namespace pibridge::app
