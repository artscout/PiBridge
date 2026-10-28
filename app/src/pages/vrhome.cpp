// SPDX-License-Identifier: MIT
#include "vrhome.h"

#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "../core/conf.h"
#include "../core/paths.h"
#include "../core/runner.h"
#include "../core/state.h"
#include "../core/status.h"
#include "../core/vr_backgrounds.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"
#include "backgrounds.h"

namespace pibridge::app {

namespace {

namespace bg = pibridge::home::bg;

struct HomeFile {
    bool running = false;
    QString state, background, note, window, runtime;
};

HomeFile readHome(bool fake) {
    HomeFile h;
    if (fake) {
        h.running = true;
        h.state = QStringLiteral("focused");
        h.background = QStringLiteral("passthrough");
        h.window = QStringLiteral("linked");
        h.runtime = QStringLiteral("Monado");
        return h;
    }
    QFile f(statusDir() + QStringLiteral("/home.json"));
    if (!f.open(QIODevice::ReadOnly)) {
        return h;
    }
    const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
    const qint64 pid = o.value(QStringLiteral("pid")).toInteger();
    h.state = o.value(QStringLiteral("state")).toString();
    h.running = pid > 0 && QFileInfo::exists(QStringLiteral("/proc/%1").arg(pid)) && h.state != QLatin1String("ending");
    h.background = o.value(QStringLiteral("background")).toString();
    h.note = o.value(QStringLiteral("background_note")).toString();
    h.window = o.value(QStringLiteral("window")).toString();
    h.runtime = o.value(QStringLiteral("runtime")).toString();
    return h;
}

QPixmap tilePixmap(const QImage& img, const QString& word, const QColor& top, const QColor& bottom) {
    QPixmap pm(160, 80);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    QPainterPath path;
    path.addRoundedRect(QRectF(0, 0, 160, 80), 6, 6);
    p.setClipPath(path);
    if (!img.isNull()) {
        p.drawImage(QRect(0, 0, 160, 80), img);
    } else {
        QLinearGradient g(0, 0, 0, 80);
        g.setColorAt(0, top);
        g.setColorAt(1, bottom);
        p.fillRect(pm.rect(), g);
        p.setPen(pibridge::ui::color::dim);
        p.setFont(pibridge::ui::labelFont(11));
        p.drawText(pm.rect(), Qt::AlignCenter, word);
    }
    return pm;
}

} // namespace

VrHomePage::VrHomePage(StateSource& source, Conf& conf, Runner& runner, ToolFinder findTool, BackgroundLibrary& lib,
                       QWidget* parent)
    : QWidget(parent), source_(source), conf_(conf), runner_(runner), findTool_(std::move(findTool)), lib_(lib) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* heading = new QLabel(QStringLiteral("VR home"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);

    // ---- the home in the headset
    homeCard_ = new pibridge::ui::Card(QStringLiteral("In the headset"));
    homeCard_->body()->addWidget(pibridge::ui::dimLabel(
        QStringLiteral("With tracking on (Monado directly), the headset shows this window on a curved screen in front of "
                       "you, over the room or a background. Point a controller at it: the trigger clicks, the stick "
                       "scrolls, holding the grip button brings the window in front of you again.")));
    homeLine_ = new QLabel;
    homeLine_->setFont(pibridge::ui::bodyFont(13));
    homeLine_->setWordWrap(true);
    homeCard_->body()->addWidget(homeLine_);
    homeNote_ = pibridge::ui::dimLabel(QString());
    homeCard_->body()->addWidget(homeNote_);
    panel_ = new pibridge::ui::Toggle;
    connect(panel_, &QAbstractButton::toggled, this, [this](bool on) {
        conf_.set(bg::kKeyPanel, on ? "on" : "off");
        updateView();
    });
    homeCard_->body()->addWidget(pibridge::ui::settingRow(QStringLiteral("This window in the headset"), panel_));
    passthrough_ = new pibridge::ui::Toggle;
    connect(passthrough_, &QAbstractButton::toggled, this, [this](bool on) {
        if (on) {
            const QString cur = conf_.value(bg::kKeyBackground, QStringLiteral("passthrough"));
            if (cur != QLatin1String("passthrough")) {
                conf_.set("home_background_last", cur.toStdString());
            }
            choose(QStringLiteral("passthrough"));
        } else {
            choose(conf_.value("home_background_last", QStringLiteral("dusk")));
        }
    });
    homeCard_->body()->addWidget(pibridge::ui::settingRow(QStringLiteral("Passthrough (see the room)"), passthrough_));
    auto* actions = new QHBoxLayout;
    actions->setSpacing(8);
    auto* map = pibridge::ui::toolButton(QStringLiteral("ROOM SETUP"));
    map->setToolTip(QStringLiteral("Maps the room in the headset (the Room page's MAP THE ROOM, with its checks)."));
    connect(map, &QPushButton::clicked, this, [this] { emit roomAction(QStringLiteral("map")); });
    auto* recenter = pibridge::ui::toolButton(QStringLiteral("RECENTER"));
    recenter->setToolTip(QStringLiteral("Sets forward and the centre where you stand (the Room page's RECENTER IN HEADSET)."));
    connect(recenter, &QPushButton::clicked, this, [this] { emit roomAction(QStringLiteral("recenter")); });
    actions->addWidget(map);
    actions->addWidget(recenter);
    actions->addStretch(1);
    homeCard_->body()->addLayout(actions);
    homeCard_->body()->addWidget(pibridge::ui::rule());
    steamvr_ = pibridge::ui::toolButton(QStringLiteral("SHOW THE HOME IN STEAMVR"));
    steamvr_->setToolTip(QStringLiteral("Starts the VR home as an OpenXR app in SteamVR (SteamVR Home is off): the same "
                                        "window, background and passthrough. Start a game from PiBridge or SteamVR and "
                                        "it takes the headset."));
    connect(steamvr_, &QPushButton::clicked, this, [this] {
        const QString tool = findTool_ ? findTool_(QStringLiteral("pibridge-home")) : QString();
        if (tool.isEmpty() || !runner_.startDetached(tool, {})) {
            steamvrWhy_->setText(QStringLiteral("pibridge-home is not installed next to PiBridge."));
        } else {
            steamvrWhy_->setText(QStringLiteral("Starting the VR home in SteamVR ..."));
        }
    });
    auto* srow = new QHBoxLayout;
    srow->addWidget(steamvr_);
    steamvrWhy_ = pibridge::ui::dimLabel(QString());
    srow->addWidget(steamvrWhy_, 1);
    homeCard_->body()->addLayout(srow);
    root->addWidget(homeCard_);

    // ---- backgrounds
    auto* bgCard = new pibridge::ui::Card(QStringLiteral("Background"));
    bgCard->body()->addWidget(pibridge::ui::dimLabel(
        QStringLiteral("A 360-degree picture around you in the VR home. Pictures from Poly Haven (CC0, free to use) are "
                       "downloaded the first time you pick one (10 to 30 MB each) into ") +
        lib_.dir() + QStringLiteral(".")));
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);
    scroll->setMinimumHeight(260);
    auto* holder = new QWidget;
    holder->setObjectName(QStringLiteral("card"));
    holder->setStyleSheet(QStringLiteral("QWidget#card { background: transparent; border: none; }"
                                         "QToolButton { border: 2px solid transparent; border-radius: 8px; padding: 4px; "
                                         "color: #9a9aa5; }"
                                         "QToolButton:checked { border-color: #4a5ae8; color: #e6e6ec; }"
                                         "QToolButton:hover { border-color: #3a3a43; }"));
    grid_ = new QGridLayout(holder);
    grid_->setContentsMargins(0, 0, 6, 0);
    grid_->setHorizontalSpacing(8);
    grid_->setVerticalSpacing(8);
    scroll->setWidget(holder);
    bgCard->body()->addWidget(scroll, 1);
    credit_ = pibridge::ui::dimLabel(QString());
    bgCard->body()->addWidget(credit_);
    downloadLine_ = new QLabel;
    downloadLine_->setWordWrap(true);
    bgCard->body()->addWidget(downloadLine_);
    root->addWidget(bgCard, 1);

    connect(&lib_, &BackgroundLibrary::progress, this, [this](const QString& id, const QString& words) {
        const auto* p = bg::find(id.toStdString());
        downloadLine_->setStyleSheet(QString());
        downloadLine_->setText(QString::fromStdString(p ? p->title : id.toStdString()) + QStringLiteral(": ") + words);
    });
    connect(&lib_, &BackgroundLibrary::finished, this, [this](const QString& id, bool ok, const QString& words) {
        const auto* p = bg::find(id.toStdString());
        downloadLine_->setStyleSheet(ok ? QString() : QStringLiteral("color: #ef5a55;"));
        downloadLine_->setText(QString::fromStdString(p ? p->title : id.toStdString()) + QStringLiteral(": ") + words);
        if (ok) {
            choose(id);
        }
        rebuildTiles();
    });
    connect(&source_, &StateSource::changed, this, &VrHomePage::updateView);
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &VrHomePage::updateView);
    if (!source_.fake()) {
        timer->start(2000);
    }
    rebuildTiles();
    updateView();
}

void VrHomePage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    conf_.reload(); // the settings may have changed on disk (the CLI, another page)
    rebuildTiles();
    updateView();
}

void VrHomePage::setShownInHeadset(bool on) {
    shownInHeadset_ = on;
    updateView();
}

void VrHomePage::choose(const QString& value) {
    conf_.set(bg::kKeyBackground, value.toStdString());
    rebuildTiles();
    updateView();
}

void VrHomePage::rebuildTiles() {
    for (auto* t : tiles_) {
        grid_->removeWidget(t);
        t->hide();
        t->deleteLater(); // a tile may be the sender of the click that got here
    }
    tiles_.clear();
    const QString cur = QString::fromStdString(bg::choiceValue(bg::parseChoice(conf_.value(bg::kKeyBackground, QString()).toStdString())));
    int i = 0;
    const auto add = [&](const QString& value, const QString& label, const QPixmap& pm, const QString& tip) {
        auto* b = new QToolButton;
        b->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        b->setIcon(QIcon(pm));
        b->setIconSize(QSize(160, 80));
        b->setText(label);
        b->setToolTip(tip);
        b->setCheckable(true);
        b->setChecked(value == cur);
        b->setCursor(Qt::PointingHandCursor);
        b->setFixedWidth(180);
        connect(b, &QToolButton::clicked, this, [this, value] {
            const auto c = bg::parseChoice(value.toStdString());
            if (c.kind == bg::Kind::Picture && !lib_.downloaded(value)) {
                if (lib_.busy()) {
                    downloadLine_->setText(QStringLiteral("One download at a time: wait for the one running."));
                } else if (!lib_.download(value)) {
                    downloadLine_->setText(source_.fake() ? QStringLiteral("(no downloads in the preview)")
                                                          : QStringLiteral("This picture cannot be downloaded."));
                }
                rebuildTiles();
                return;
            }
            choose(value);
        });
        grid_->addWidget(b, i / 5, i % 5);
        tiles_.push_back(b);
        i++;
    };
    add(QStringLiteral("passthrough"), QStringLiteral("Room (passthrough)"),
        tilePixmap({}, QStringLiteral("CAMERAS"), QColor(0x55, 0x55, 0x5c), QColor(0x2a, 0x2a, 0x30)),
        QStringLiteral("The room through the headset's four cameras."));
    add(QStringLiteral("dusk"), QStringLiteral("Dusk (built in)"),
        tilePixmap({}, QString(), QColor(0x40, 0x3c, 0x78), QColor(0xd9, 0x8f, 0x9a)), QStringLiteral("Always there, no download."));
    add(QStringLiteral("none"), QStringLiteral("Dark grid"),
        tilePixmap({}, QStringLiteral("GRID"), QColor(0x10, 0x12, 0x1c), QColor(0x05, 0x06, 0x0a)),
        QStringLiteral("The dark home with a floor grid and the horizon."));
    for (const auto& p : bg::catalogue()) {
        const QString id = QString::fromStdString(p.id);
        const bool have = lib_.downloaded(id);
        const bool loading = lib_.busyId() == id;
        QString label = QString::fromStdString(p.title);
        if (loading) {
            label += QStringLiteral(" (downloading)");
        } else if (!have) {
            label += QStringLiteral(" (download)");
        }
        add(id, label,
            tilePixmap(lib_.thumb(id), loading ? QStringLiteral("DOWNLOADING") : QStringLiteral("TAP TO DOWNLOAD"),
                       QColor(0x22, 0x24, 0x30), QColor(0x16, 0x16, 0x1c)),
            QStringLiteral("%1 (%2) by %3, %4, polyhaven.com").arg(QString::fromStdString(p.title), QString::fromStdString(p.theme),
                                                                  QString::fromStdString(p.author), QString::fromStdString(p.licence)));
    }
    const auto c = bg::parseChoice(cur.toStdString());
    if (c.kind == bg::Kind::Picture) {
        const auto* p = bg::find(c.id);
        credit_->setText(QStringLiteral("Picture: %1 by %2 (%3), %4").arg(QString::fromStdString(p->title), QString::fromStdString(p->author),
                                                                           QString::fromStdString(p->licence),
                                                                           QString::fromStdString(bg::pageUrl(p->id))));
    } else {
        credit_->setText(QString());
    }
}

void VrHomePage::updateView() {
    const HomeFile h = readHome(source_.fake());
    const bool steamvr = source_.state().steamvrRunning;
    panel_->setCheckedQuiet(conf_.flag(bg::kKeyPanel, true));
    const auto c = bg::parseChoice(conf_.value(bg::kKeyBackground, QString()).toStdString());
    passthrough_->setCheckedQuiet(c.kind == bg::Kind::Passthrough);
    if (!h.running) {
        homeCard_->chip()->set(QStringLiteral("NOT RUNNING"), ChipKind::Off);
        homeLine_->setText(QStringLiteral("The VR home starts here in SteamVR."));
        homeNote_->setText(QString());
    } else {
        const bool shown = h.state == QLatin1String("focused") || h.state == QLatin1String("visible");
        homeCard_->chip()->set(shown ? QStringLiteral("IN THE HEADSET") : QStringLiteral("BEHIND AN APP"),
                               shown ? ChipKind::Info : ChipKind::Off);
        QString what = h.background == QLatin1String("passthrough") ? QStringLiteral("the room through the cameras")
                       : h.background == QLatin1String("picture")    ? QStringLiteral("the background picture")
                       : h.background == QLatin1String("dusk")       ? QStringLiteral("the dusk sky")
                                                                     : QStringLiteral("the dark grid");
        homeLine_->setText(QStringLiteral("The home runs in %1 and shows %2; this window: %3.")
                               .arg(h.runtime.isEmpty() ? QStringLiteral("the runtime") : h.runtime, what,
                                    shownInHeadset_ || source_.fake() ? QStringLiteral("in the headset") : h.window));
        homeNote_->setText(h.note);
    }
    homeNote_->setVisible(!homeNote_->text().isEmpty());
    steamvr_->setEnabled(steamvr && !h.running);
    if (!steamvr) {
        steamvrWhy_->setText(QStringLiteral("VR is not running (START VR on the Dashboard)."));
    } else if (h.running) {
        steamvrWhy_->setText(QStringLiteral("The home runs already."));
    } else if (steamvrWhy_->text().startsWith(QLatin1String("SteamVR is not")) || steamvrWhy_->text().startsWith(QLatin1String("The home runs"))) {
        steamvrWhy_->setText(QString());
    }
}

} // namespace pibridge::app
