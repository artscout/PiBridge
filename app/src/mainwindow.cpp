// SPDX-License-Identifier: MIT
#include "mainwindow.h"

#include <algorithm>

#include <QApplication>
#include <QButtonGroup>
#include <QComboBox>
#include <QDesktopServices>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QUrl>
#include <QVBoxLayout>

#include "core/paths.h"
#include "pages/about.h"
#include "pages/apps.h"
#include "pages/dashboard.h"
#include "pages/development.h"
#include "pages/record_card.h"
#include "pages/device.h"
#include "pages/display.h"
#include "pages/logs.h"
#include "pages/performance.h"
#include "pages/recording.h"
#include "pages/room.h"
#include "pages/tracking.h"
#include "core/vr_panel.h"
#include "ui/brand.h"
#include "ui/icons.h"
#include "ui/theme.h"
#include "ui/widgets.h"

namespace pibridge::app {

namespace {

QFrame* navSep() {
    auto* f = new QFrame;
    f->setObjectName(QStringLiteral("navSep"));
    return f;
}

QLabel* statusItem(QHBoxLayout* into, pibridge::ui::Icon icon) {
    auto* ic = new QLabel;
    ic->setPixmap(pibridge::ui::lineIcon(icon, pibridge::ui::color::accentText).pixmap(18, 18));
    auto* l = new QLabel;
    l->setFont(pibridge::ui::labelFont(12));
    l->setObjectName(QStringLiteral("dim"));
    into->addWidget(ic);
    into->addWidget(l);
    into->addSpacing(18);
    return l;
}

} // namespace

QString MainWindow::pageSlug(int page) {
    switch (page) {
    case Dashboard: return QStringLiteral("dashboard");
    case Apps: return QStringLiteral("apps");
    case Performance: return QStringLiteral("performance-quadviews");
    case Tracking: return QStringLiteral("tracking");
    case Room: return QStringLiteral("room");
    case Development: return QStringLiteral("development");
    case Logs: return QStringLiteral("logs");
    case Device: return QStringLiteral("device");
    case Display: return QStringLiteral("display");
    case Recording: return QStringLiteral("recording");
    default: return QStringLiteral("about");
    }
}

MainWindow::MainWindow(StateSource& source, Runner& runner, Actions::ToolFinder findTool, QWidget* parent)
    : QMainWindow(parent), source_(source), conf_(confPath(), legacyConfPath()),
      actions_(runner, conf_, findTool, eyeProfileOut()) {
    setWindowTitle(pibridge::ui::productName());
    setWindowIcon(QApplication::windowIcon());
    auto* root = new QWidget;
    root->setObjectName(QStringLiteral("root"));
    auto* rv = new QVBoxLayout(root);
    rv->setContentsMargins(0, 0, 0, 0);
    rv->setSpacing(0);
    auto* main = new QHBoxLayout;
    main->setSpacing(0);
    rv->addLayout(main, 1);

    // ---- navigation
    auto* nav = new QFrame;
    nav->setObjectName(QStringLiteral("nav"));
    nav->setFixedWidth(230);
    auto* nv = new QVBoxLayout(nav);
    nv->setContentsMargins(10, 14, 10, 14);
    nv->setSpacing(6);
    auto* brandRow = new QHBoxLayout;
    brandRow->setContentsMargins(8, 0, 0, 10);
    brandRow->setSpacing(8);
    auto* gem = new QLabel;
    gem->setPixmap(pibridge::ui::logoPixmap(34)); // sized to the "PiBridge" title next to it
    brandRow->addWidget(gem);
    auto* brand = new QLabel(pibridge::ui::productName());
    QFont bf = pibridge::ui::titleFont(22);
    bf.setCapitalization(QFont::MixedCase);
    bf.setWeight(QFont::Normal);
    brand->setFont(bf);
    brandRow->addWidget(brand);
    brandRow->addStretch(1);
    nv->addLayout(brandRow);
    auto* group = new QButtonGroup(this);
    const auto navButtonIcon = [&](const QString& text, const QIcon& icon, int page) {
        auto* b = new QPushButton(text);
        b->setObjectName(QStringLiteral("nav"));
        b->setCheckable(true);
        b->setFont(pibridge::ui::labelFont(13));
        b->setIcon(icon);
        b->setIconSize({20, 20});
        b->setCursor(Qt::PointingHandCursor);
        group->addButton(b, page);
        nav_.resize(PageCount);
        nav_[static_cast<size_t>(page)] = b; // by page: the buttons are not in page order (Device)
        nv->addWidget(b);
    };
    const auto navButton = [&](const QString& text, pibridge::ui::Icon icon, int page) {
        navButtonIcon(text, pibridge::ui::lineIcon(icon, pibridge::ui::color::dim, Qt::white), page);
    };
    navButton(QStringLiteral("Dashboard"), pibridge::ui::Icon::Dashboard, Dashboard);
    navButton(QStringLiteral("Display"), pibridge::ui::Icon::Dashboard, Display);
    navButtonIcon(QStringLiteral("Apps"), appsNavIcon(pibridge::ui::color::dim, Qt::white), Apps);
    navButton(QStringLiteral("Device"), pibridge::ui::Icon::About, Device);
    nv->addWidget(navSep());
    navButton(QStringLiteral("Performance"), pibridge::ui::Icon::Performance, Performance);
    navButton(QStringLiteral("Tracking"), pibridge::ui::Icon::Tracking, Tracking);
    navButton(QStringLiteral("Room"), pibridge::ui::Icon::Room, Room);
    navButton(QStringLiteral("Recording"), pibridge::ui::Icon::Logs, Recording);
    nv->addWidget(navSep());
    navButton(QStringLiteral("Development"), pibridge::ui::Icon::Controller, Development);
    navButton(QStringLiteral("Logs"), pibridge::ui::Icon::Logs, Logs);
    navButton(QStringLiteral("About"), pibridge::ui::Icon::About, About);
    nv->addStretch(1);
    main->addWidget(nav);

    // ---- pages
    IoLinks links;
    links.steamvrRunning = [this] { return source_.state().steamvrRunning; };
    links.steamvrHmd = [this] {
        const SteamVrStatus& st = source_.state().steamvr;
        return st.state == VrLinkState::Connected &&
               std::any_of(st.devices.begin(), st.devices.end(), [](const VrDevice& d) { return d.cls == VrClass::Hmd; });
    };
    const auto deps = [&](IoVariant v) {
        return source_.fake() ? IoSessionControl::fakeDeps(v, conf_, links) : IoSessionControl::realDeps(v, conf_, links);
    };
    io_ = std::make_unique<IoSessionControl>(deps(IoVariant::SteamVr));
    direct_ = std::make_unique<IoSessionControl>(deps(IoVariant::MonadoDirect));
    // one session at a time: each blocks the other (and the recorder blocks both)
    connect(io_.get(), &IoSessionControl::changed, direct_.get(), [this] {
        direct_->setOtherSession(io_->active() ? QStringLiteral("Tracking with SteamVR + Monado runs (Tracking page). "
                                                                "Stop it first.")
                                               : QString());
    });
    connect(direct_.get(), &IoSessionControl::changed, io_.get(), [this] {
        io_->setOtherSession(direct_->active() ? QStringLiteral("\"Monado directly\" runs (Development page). Stop it "
                                                                "first.")
                                               : QString());
    });
    // The SteamVR dashboard launcher (a leaf): its switch is in the Dashboard's Quick settings; its clicks launch room setup.
    overlay_ = std::make_unique<OverlayControl>(conf_, source_.fake());

    stack_ = new QStackedWidget;
    auto* dash = new DashboardPage(source_, actions_, conf_, *io_, *direct_, *overlay_);
    connect(dash, &DashboardPage::openTracking, this, [this] { showPage(Tracking); });
    connect(dash, &DashboardPage::openRoom, this, [this] { showPage(Room); });
    stack_->addWidget(dash);
    auto* apps = new AppsPage(source_, runner, &conf_, direct_.get(), io_.get());
    connect(apps, &AppsPage::quadViewsProfileRequested, this, &MainWindow::openQuadViewsProfile);
    stack_->addWidget(apps);
    stack_->addWidget(new PerformancePage);
    stack_->addWidget(new TrackingPage(source_, conf_, *io_));
    auto* dev = new DevelopmentPage(source_, conf_, *direct_);
    connect(dev, &DevelopmentPage::recordingActiveChanged, io_.get(), &IoSessionControl::setRecorderActive);
    connect(dev, &DevelopmentPage::recordingActiveChanged, direct_.get(), &IoSessionControl::setRecorderActive);
    const auto block = [this, dev] {
        dev->setExternalBlock(io_->active()       ? QStringLiteral("Tracking with SteamVR + Monado runs (START TRACKING on "
                                                                   "the Tracking page). Stop it first.")
                              : direct_->active() ? QStringLiteral("\"Monado directly\" runs (below). Stop it first.")
                                                  : QString());
    };
    connect(io_.get(), &IoSessionControl::changed, dev, block);
    connect(direct_.get(), &IoSessionControl::changed, dev, block);
    auto* roomPage = new RoomPage(source_, runner, findTool, &conf_);
    // The Dashboard's ROOM SETUP button launches the in-headset setup directly, without opening the Room page.
    connect(dash, &DashboardPage::startRoomSetup, roomPage, &RoomPage::launchSetup);
    // QUICK ROOM SETUP launches straight into the quick circle setup (skips the in-VR Full/Quick mode menu).
    connect(dash, &DashboardPage::startQuickRoomSetup, roomPage, &RoomPage::launchQuickSetup);
    // A click on the SteamVR dashboard launcher does the same, from inside VR (via the overlay request file).
    connect(overlay_.get(), &OverlayControl::roomSetupRequested, roomPage, [roomPage](bool quick) {
        if (quick) {
            roomPage->launchQuickSetup();
        } else {
            roomPage->launchSetup();
        }
    });
    // A click on the overlay's record button starts or stops a video recording through the Dashboard's card; its
    // segment controls (view / fps / size) write the recording settings the next start uses.
    if (RecordCard* rc = dash->recordCard()) {
        connect(overlay_.get(), &OverlayControl::recordRequested, rc, &RecordCard::recordFromVr);
        connect(overlay_.get(), &OverlayControl::recordSettingChanged, rc, &RecordCard::setFromVr);
    }
    stack_->addWidget(roomPage);
    stack_->addWidget(dev);
    stack_->addWidget(new LogsPage(source_.fake()));
    stack_->addWidget(new AboutPage);
    stack_->addWidget(new DevicePage(source_, conf_, *io_, *direct_)); // index = Device
    stack_->addWidget(new DisplayPage(source_, conf_, actions_));      // index = Display
    stack_->addWidget(new RecordingPage(source_, *direct_, conf_));    // index = Recording
    if (!source_.fake()) {
        // shows this window in the headset (the SteamVR-panel embed; VR home was dropped, user 2026-10-20)
        vrPanel_ = new VrPanelLink(this, this);
        vrPanel_->start();
    }
    main->addWidget(stack_, 1);
    connect(group, &QButtonGroup::idClicked, this, &MainWindow::showPage);

    // ---- status bar
    auto* sb = new QFrame;
    sb->setObjectName(QStringLiteral("statusbar"));
    sb->setFixedHeight(34);
    auto* sh = new QHBoxLayout(sb);
    sh->setContentsMargins(16, 0, 16, 0);
    sh->setSpacing(6);
    sbSteam_ = statusItem(sh, pibridge::ui::Icon::Play);
    sbHeadset_ = statusItem(sh, pibridge::ui::Icon::Dashboard);
    sbEye_ = statusItem(sh, pibridge::ui::Icon::Eye);
    sh->addStretch(1);
    auto* help = new QPushButton(QStringLiteral("Help"));
    help->setObjectName(QStringLiteral("link"));
    help->setFont(pibridge::ui::labelFont(12));
    help->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Help, pibridge::ui::color::accentText));
    help->setCursor(Qt::PointingHandCursor);
    help->setToolTip(QStringLiteral("Opens the documentation of the app (docs/app.md)."));
    connect(help, &QPushButton::clicked, this, [] {
        // installed docs first, then the source tree this binary was built from
        for (const QString& p : {QStringLiteral(PIBRIDGE_INSTALLED_DOCS "/app.md"), QStringLiteral(PIBRIDGE_SOURCE_DOCS "/app.md")}) {
            if (QFileInfo::exists(p)) {
                QDesktopServices::openUrl(QUrl::fromLocalFile(p));
                return;
            }
        }
    });
    sh->addWidget(help);
    rv->addWidget(sb);

    setCentralWidget(root);
    resize(1200, 860); // resizable; the layout stretches fine (user 2026-10-20)
    connect(&source_, &StateSource::changed, this, &MainWindow::applyState);
    applyState();
    showPage(Dashboard);
}

void MainWindow::showPage(int page) {
    if (page < 0 || page >= PageCount) {
        return;
    }
    stack_->setCurrentIndex(page);
    nav_[static_cast<size_t>(page)]->setChecked(true);
}

void MainWindow::openQuadViewsProfile(const QString& name) {
    showPage(Performance);
    // The Performance page lists its profiles in its "Settings for:" box with the section as the item data
    // ("exe:Falcon BMS" / "app:Falcon BMS"); pick the one for this name if there is one.
    auto* box = page(Performance)->findChild<QComboBox*>();
    if (!box || name.isEmpty()) {
        return;
    }
    for (const QString& prefix : {QStringLiteral("exe:"), QStringLiteral("app:")}) {
        for (int i = 0; i < box->count(); i++) {
            const QString sec = box->itemData(i).toString();
            if (sec.startsWith(prefix) && sec.mid(prefix.size()).compare(name, Qt::CaseInsensitive) == 0) {
                box->setCurrentIndex(i);
                return;
            }
        }
    }
}

QWidget* MainWindow::page(int p) const {
    return stack_->widget(p);
}

void MainWindow::applyState() {
    const AppState& s = source_.state();
    sbSteam_->setText(s.steamvrRunning ? QStringLiteral("SteamVR running") : QStringLiteral("SteamVR not running"));
    const ChipText h = headsetChip(s.keepalive);
    sbHeadset_->setText(QStringLiteral("Headset: %1").arg(s.keepalive.running ? h.text.toLower()
                                                                               : QStringLiteral("service not running")));
    const ChipText e = eyeChip(s.eyed);
    sbEye_->setText(QStringLiteral("Eye tracking: %1").arg(s.eyed.running ? e.text.toLower()
                                                                          : QStringLiteral("service not running")));
}

} // namespace pibridge::app
