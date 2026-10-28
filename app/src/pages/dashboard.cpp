// SPDX-License-Identifier: MIT
#include "dashboard.h"
#include <QSignalBlocker>
#include <QComboBox>
#include "core/apps_runtime.h"
#include "core/openxr_runtime.h"
#include "record_card.h"
#include "sound_tests.h"

#include <QGuiApplication>

#include <QClipboard>
#include <QFile>
#include <QDateTime>
#include <QDialog>

#include <QDir>
#include <QFileDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <memory>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QHash>
#include <QLayoutItem>
#include <QSaveFile>
#include <QStandardPaths>
#include <QStyle>
#include <QVBoxLayout>
#include <cmath>

#include "../core/apps_runtime.h"
#include "../core/chovr.h"
#include "../core/inside_out.h"
#include "../core/monado_build.h"
#include "../core/overlay_control.h"
#include "../core/paths.h"
#include "../core/system.h"
#include "../core/rec_setup.h"
#include "../core/steamvr.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

// "LABEL      value" rows of the headset card.
QLabel* infoRow(QGridLayout* g, int row, const QString& label) {
    auto* l = pibridge::ui::capsLabel(label, 13);
    l->setObjectName(QStringLiteral("dim"));
    auto* v = new QLabel;
    v->setFont(pibridge::ui::bodyFont(14));
    g->addWidget(l, row, 0);
    g->addWidget(v, row, 1);
    return v;
}

// The last lines of a tool's output, for the result line.
QString tail(const QString& out) {
    QStringList lines = out.trimmed().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    while (lines.size() > 2) {
        lines.removeFirst();
    }
    return lines.join(QStringLiteral(" / "));
}

} // namespace

namespace {

// $XDG_STATE_HOME/pibridge/controller-serials ("headset", "optics", "left", "right" lines; libpibridge-hmd
// ctrl_serials.h), read from the devices by the headset service.
QString keptSerial(const QString& side) {
    QString dir = qEnvironmentVariable("XDG_STATE_HOME");
    if (dir.isEmpty()) {
        dir = QDir::homePath() + QStringLiteral("/.local/state");
    }
    QFile f(dir + QStringLiteral("/pibridge/controller-serials"));
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    for (const QByteArray& line : f.readAll().split('\n')) {
        const QList<QByteArray> kv = line.trimmed().split(' ');
        if (kv.size() == 2 && QString::fromLatin1(kv[0]) == side) {
            return QString::fromLatin1(kv[1]);
        }
    }
    return {};
}

} // namespace

DashboardPage::DashboardPage(StateSource& source, Actions& actions, Conf& conf, IoSessionControl& io,
                             IoSessionControl& direct, OverlayControl& overlay, QWidget* parent)
    : QWidget(parent), source_(source), actions_(actions), conf_(conf), io_(io), direct_(direct), overlay_(overlay),
      ctrlOps_(statusDir(), qEnvironmentVariable("XDG_RUNTIME_DIR", QStringLiteral("/tmp"))) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);

    // ---- toolbar
    auto* bar = new QHBoxLayout;
    bar->setSpacing(8);
    // the system's active OpenXR runtime (core/openxr_runtime.h): what every OpenXR game starts on, Steam's included
    auto* runtime = runtime_ = new QComboBox;
    runtime->addItem(QStringLiteral("SteamVR"), QStringLiteral("steamvr"));
    runtime->addItem(QStringLiteral("Monado"), QStringLiteral("monado"));
    runtime->setToolTip(QStringLiteral(
        "The system's active OpenXR runtime (~/.config/openxr/1/active_runtime.json): what OpenXR games start on, also "
        "from Steam. SteamVR: SteamVR's runtime. Monado: PiBridge's Monado build (start tracking with the Monado "
        "backend first). OpenVR-only games always need SteamVR."));
    connect(runtime, &QComboBox::activated, this, [this](int i) {
        const RecSetup s = resolveRecSetup(conf_, RecEnv::defaults());
        const bool monado = runtime_->itemData(i).toString() == QLatin1String("monado");
        const QString target = monado ? monadoRuntimeJsonFor(s.monado.path) : steamVrRuntimeJsonFor(s.steamVrStartup);
        if (target.isEmpty()) {
            say(monado ? QStringLiteral("No Monado build with an OpenXR runtime file was found.")
                       : QStringLiteral("SteamVR's OpenXR runtime file (steamxr_linux64.json) was not found."),
                true);
        } else if (auto r = setActiveRuntime(activeRuntimePath(), target); !r) {
            say(r.error(), true);
        } else {
            say(QStringLiteral("OpenXR games now start on %1 (%2). Games already running keep theirs.")
                    .arg(monado ? QStringLiteral("Monado") : QStringLiteral("SteamVR"), target));
        }
        updateRuntime();
    });
    // LAUNCH STEAMVR was removed from the top bar (user 2026-10-20): the big START VR button in the headset box does it.
    auto* restartServices = pibridge::ui::toolButton(QStringLiteral("Restart services"));
    restartServices->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Restart, pibridge::ui::color::text));
    restartServices->setToolTip(
        QStringLiteral("Restarts the headset service and the eye tracking service (pibridge-keepalived, pibridge-eyed)."));
    auto* restartHeadset = pibridge::ui::toolButton(QStringLiteral("Restart headset"));
    restartHeadset->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Power, pibridge::ui::color::text));
    restartHeadset->setToolTip(QStringLiteral("Restarts the headset service (pibridge-keepalived): wakes the headset again."));
    reset_ = pibridge::ui::toolButton(QStringLiteral("Reset SteamVR"));
    reset_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Restart, pibridge::ui::color::text));
    reset_->setToolTip(QStringLiteral(
        "Restarts SteamVR: finds its processes (also a stuck vrserver that holds SteamVR's socket), closes exactly "
        "those after you confirm and starts SteamVR again. When no SteamVR process runs, it offers to restart Steam."));
    // "Headset driver settings" was removed from the top bar (user 2026-10-20): the Tracking card's Settings opens it
    // in Lighthouse mode (openTrackingSettings).
    message_ = new QLabel;
    // never let a long status/close report push the window wide (user 2026-10-20: it stretched across two monitors)
    message_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    messageTimer_.setSingleShot(true);
    connect(&messageTimer_, &QTimer::timeout, message_, [this] {
        waitingForSteamVr_ = false;
        message_->clear();
    });
    message_->setObjectName(QStringLiteral("dim"));
    // right of Runtime: a noticeable "Fix permissions" button, shown only when monado-service lacks cap_sys_nice
    fixCaps_ = pibridge::ui::accentButton(QStringLiteral("Fix permissions"));
    fixCaps_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Power, Qt::white));
    fixCaps_->setCursor(Qt::PointingHandCursor);
    fixCaps_->setToolTip(QStringLiteral("Lets monado-service use realtime priority: runs pkexec setcap cap_sys_nice=ep "
                                        "on it (your password). Needed after a Monado rebuild."));
    fixCaps_->setVisible(false);
    connect(fixCaps_, &QPushButton::clicked, this, [this] {
        if (fixCapsPath_.isEmpty()) {
            return;
        }
        fixCaps_->setEnabled(false);
        say(QStringLiteral("Fixing permissions..."));
        runCapFix(*steamRunner_, fixCapsPath_, [this](bool ok, const QString& words) {
            say(words, !ok);
            fixCaps_->setEnabled(true);
            updateFixCaps();
        });
    });
    bar->addWidget(pibridge::ui::capsLabel(QStringLiteral("OpenXR runtime"), 12));
    bar->addWidget(runtime);
    bar->addWidget(fixCaps_);
    bar->addSpacing(12);
    bar->addWidget(message_, 1);
    bar->addWidget(reset_);
    bar->addWidget(restartServices);
    bar->addWidget(restartHeadset);
    root->addLayout(bar);

    const auto systemctlDone = [this](const QString& ok) {
        return [this, ok](int code, const QString& out) {
            if (code == 0) {
                say(ok);
            } else {
                say(QStringLiteral("Could not restart: %1").arg(out.trimmed().isEmpty() ? QStringLiteral("systemctl failed")
                                                                                        : tail(out)),
                    true);
            }
            source_.refresh();
        };
    };
    connect(restartServices, &QPushButton::clicked, this, [this, systemctlDone] {
        say(QStringLiteral("Restarting the services..."));
        actions_.restartServices(systemctlDone(QStringLiteral("Services restarted.")));
    });
    connect(restartHeadset, &QPushButton::clicked, this, [this, systemctlDone] {
        say(QStringLiteral("Restarting the headset service..."));
        actions_.restartHeadset(systemctlDone(QStringLiteral("Headset service restarted.")));
    });

    connect(reset_, &QPushButton::clicked, this, &DashboardPage::resetSteamVr);
    resetTimer_.setInterval(200);
    connect(&resetTimer_, &QTimer::timeout, this, [this] {
        const qint64 now = resetClock_.elapsed();
        if (closer_ && closer_->step(now)) {
            resetTimer_.stop();
            const bool gone = closer_->allGone();
            say(closer_->report(), !gone);
            closer_.reset();
            reset_->setEnabled(true);
            source_.refresh();
            if (std::exchange(startAfterClose_, false) && gone) {
                startVr(); // Reset SteamVR is a restart (user 2026-10-22)
            }
        } else if (restarter_ && restarter_->step(now)) {
            resetTimer_.stop();
            say(restarter_->message(), restarter_->state() == SteamRestarter::State::Failed);
            restarter_.reset();
            reset_->setEnabled(true);
        }
    });
    if (source_.fake()) {
        steamRunner_ = std::make_unique<FakeRunner>();
    } else {
        steamRunner_ = std::make_unique<ProcessRunner>();
    }
    updateChovr();

    auto* grid = new QGridLayout;
    grid->setSpacing(10);
    root->addLayout(grid); // the cards size to their content; the window sizes to end right under the Record card

    // ---- headset card
    auto* hs = new pibridge::ui::Card(QStringLiteral("No headset"));
    hsTitle_ = hs->titleLabel();
    hsChip_ = hs->chip();
    // The tracking mode (inside-out / Lighthouse) — set in applyState; we cannot detect the faceplate, so we do not
    // claim one.
    hsSub_ = pibridge::ui::dimLabel(QString());
    hs->body()->addWidget(hsSub_);
    hsHint_ = new QLabel;
    hsHint_->setObjectName(QStringLiteral("dim"));
    hsHint_->setWordWrap(true);
    hsHint_->setAlignment(Qt::AlignLeft);
    hsError_ = new QLabel;
    hsError_->setObjectName(QStringLiteral("error"));
    hsError_->setWordWrap(true);
    hsError_->setAlignment(Qt::AlignLeft);
    auto* hsRow = new QHBoxLayout;
    art_ = new pibridge::ui::HeadsetArt;
    art_->setMaximumHeight(104); // compact, closer to omarchyvr's density (user 2026-10-20)
    hsRow->addWidget(art_, 1, Qt::AlignTop); // raised to line up with the STATE row on the right
    auto* right = new QVBoxLayout;
    right->setSpacing(10);
    auto* info = new QGridLayout;
    info->setHorizontalSpacing(24);
    info->setVerticalSpacing(14);
    hsState_ = infoRow(info, 0, QStringLiteral("State"));
    hsSerial_ = infoRow(info, 1, QStringLiteral("Headset SN"));
    hsSerial_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    hsOptics_ = infoRow(info, 2, QStringLiteral("Optics SN"));
    hsOptics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    hsOpticsLabel_ = info->itemAtPosition(2, 0)->widget();
    auto* rl = pibridge::ui::capsLabel(QStringLiteral("Refresh rate"), 13);
    rl->setObjectName(QStringLiteral("dim"));
    const QString refreshTip = QStringLiteral("The panel rate the headset service sets (refresh_hz in pibridge.conf). "
                                              "Takes effect the next time the headset wakes.");
    refresh_ = new pibridge::ui::Segmented({QStringLiteral("72 Hz"), QStringLiteral("90 Hz")});
    refresh_->setToolTip(refreshTip);
    // the original Crystal: its rate is the display mode SteamVR picks (CustomHeadsetOpenVR reports it), not a
    // headset command; 120 Hz is its native rate
    refreshCrystal_ = new pibridge::ui::Segmented({QStringLiteral("72 Hz"), QStringLiteral("90 Hz"), QStringLiteral("120 Hz")});
    refreshCrystal_->setToolTip(QStringLiteral("The display mode SteamVR runs the Crystal at. A running SteamVR is "
                                               "restarted for it."));
    refreshCrystal_->setVisible(false);
    // never squeezed below their own height (the digits were cut at the bottom)
    refresh_->setMinimumHeight(refresh_->sizeHint().height());
    refreshCrystal_->setMinimumHeight(refreshCrystal_->sizeHint().height());
    auto* rlRow = new QHBoxLayout;
    rlRow->setSpacing(6);
    rlRow->setContentsMargins(0, 0, 0, 0);
    rlRow->addWidget(rl);
    rlRow->addWidget(pibridge::ui::helpBadge(refreshTip));
    rlRow->addStretch(1);
    info->addLayout(rlRow, 3, 0);
    auto* refreshBox = new QHBoxLayout;
    refreshBox->setContentsMargins(0, 0, 0, 0);
    refreshBox->addWidget(refresh_);
    refreshBox->addWidget(refreshCrystal_);
    refreshBox->addStretch(1);
    info->addLayout(refreshBox, 3, 1);
    // the original Crystal (standalone, on its own battery): its charge, from its head reports via pibridge-keepalived
    hsBatteryLabel_ = pibridge::ui::capsLabel(QStringLiteral("Battery"), 13);
    hsBatteryLabel_->setObjectName(QStringLiteral("dim"));
    hsBattery_ = new pibridge::ui::Battery;
    hsBatteryText_ = new QLabel;
    auto* batRow = new QHBoxLayout;
    batRow->setSpacing(8);
    batRow->setContentsMargins(0, 0, 0, 0);
    batRow->addWidget(hsBattery_);
    batRow->addWidget(hsBatteryText_);
    batRow->addStretch(1);
    info->addWidget(hsBatteryLabel_, 4, 0);
    info->addLayout(batRow, 4, 1);
    for (QWidget* w : {static_cast<QWidget*>(hsBatteryLabel_), static_cast<QWidget*>(hsBattery_),
                       static_cast<QWidget*>(hsBatteryText_)}) {
        w->setVisible(false);
    }
    info->setColumnStretch(1, 1);
    // the rows keep their places whatever is shown (an empty Optics SN, a long state)
    for (int r = 0; r < 4; r++) {
        info->setRowMinimumHeight(r, 34);
    }
    info->setAlignment(Qt::AlignTop);
    right->addLayout(info);
    right->addStretch(1);
    hsRow->addLayout(right, 1);
    hs->body()->addLayout(hsRow);
    // a hint or an error goes under the picture and the rows, never between them
    hs->body()->addWidget(hsHint_);
    hs->body()->addWidget(hsError_); // content height, so the buttons sit right under the info (no middle gap)
    // START VR / STOP in the headset box (like omarchyvr's Play/Stop). Inside-out: starts the tracking session.
    // Lighthouse: just launches SteamVR. One button, no separate one (user 2026-10-20).
    auto* ioRow = new QHBoxLayout;
    ioStart_ = pibridge::ui::accentButton(QStringLiteral("START VR"));
    ioStart_->setMinimumHeight(40);
    ioStart_->setMinimumWidth(170);
    ioStart_->setFont(pibridge::ui::labelFont(14, QFont::DemiBold));
    ioStart_->setCursor(Qt::PointingHandCursor);
    ioStart_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Play, Qt::white));
    ioStart_->setIconSize({18, 18});
    // second button next to START VR, like omarchyvr's Play + Work: ROOM SETUP (opens the Room page)
    roomBtn_ = pibridge::ui::toolButton(QStringLiteral("ROOM SETUP"));
    roomBtn_->setMinimumHeight(40);
    roomBtn_->setMinimumWidth(150);
    roomBtn_->setFont(pibridge::ui::labelFont(14, QFont::DemiBold));
    roomBtn_->setCursor(Qt::PointingHandCursor);
    roomBtn_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Room, pibridge::ui::color::text));
    roomBtn_->setIconSize({18, 18});
    // QUICK ROOM SETUP is in the SteamVR dashboard tab (pibridge-overlay) only, where it is used (user 2026-10-22)
    ioRow->addWidget(ioStart_);
    ioRow->addWidget(roomBtn_);
    // the original Crystal only: it is switched on with its own button, switched off from here (shutDown) or there
    crystalOffBtn_ = pibridge::ui::toolButton(QStringLiteral("SWITCH OFF"));
    crystalOffBtn_->setMinimumHeight(40);
    crystalOffBtn_->setMinimumWidth(130);
    crystalOffBtn_->setFont(pibridge::ui::labelFont(14, QFont::DemiBold));
    crystalOffBtn_->setCursor(Qt::PointingHandCursor);
    crystalOffBtn_->setToolTip(QStringLiteral("Switches the Crystal off. It is switched on with its own power button."));
    crystalOffBtn_->setVisible(false);
    ioRow->addWidget(crystalOffBtn_);
    ioRow->addStretch(1);
    hs->body()->addLayout(ioRow);
    vrStatus_ = pibridge::ui::dimLabel(QString()); // "VR running 1h 5m   Resolution: 5376 x 4972"
    hs->body()->addWidget(vrStatus_);
    // at start: an inside-out session an earlier PiBridge left running after SteamVR closed (PiBridge restarted while it
    // ran) holds the cameras and winbridge, and its readiness check would keep START VR disabled
    if (!source_.fake()) {
        if (const QStringList left = stopLeftoverSession(); !left.isEmpty()) {
            QTimer::singleShot(0, this, [this, left] {
                say(QStringLiteral("Stopped what an earlier session left running: %1.").arg(left.join(QStringLiteral(", "))));
            });
        }
    }
    connect(ioStart_, &QPushButton::clicked, this, [this] {
        conf_.reload();
        const bool inside = monadoSession();
        const bool running = inside ? io_.active() : source_.state().steamvrRunning;
        if (running) {
            // the same button is red STOP VR while running; no confirmation (user 2026-10-20)
            if (inside) {
                io_.stop();
            } else if (!source_.fake() && !closer_ && !restarter_) {
                closeSteamVr(scanSteamVr().targets()); // Lighthouse: close SteamVR straight away
            }
            if (crystalOn() && !source_.fake()) {
                // the original Crystal runs on its battery: STOP VR switches it off too (user 2026-10-22)
                say(actions_.crystalShutdown(statusDir()) ? QStringLiteral("Closing SteamVR and switching the Crystal off...")
                                                          : QStringLiteral("Could not write the request to pibridge-keepalived."),
                    false);
            }
            return;
        }
        startVr();
    });
    connect(roomBtn_, &QPushButton::clicked, this, [this] {
        say(QStringLiteral("Starting room setup in the headset..."));
        emit startRoomSetup();
    });
    connect(crystalOffBtn_, &QPushButton::clicked, this, [this] {
        if (QMessageBox::question(this, QStringLiteral("Switch off the Crystal"),
                                  QStringLiteral("Switch the headset off? Switch it on again with its power button.")) !=
            QMessageBox::Yes) {
            return;
        }
        say(actions_.crystalShutdown(statusDir()) ? QStringLiteral("Switching the Crystal off...")
                                                  : QStringLiteral("Could not write the request to pibridge-keepalived."),
            false);
    });
    connect(refresh_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        const int hz = i == 0 ? 72 : 90;
        if (!actions_.setRefreshHz(hz, {})) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            applyConf();
            return;
        }
        say(QStringLiteral("Refresh rate set to %1 Hz.").arg(hz));
    });
    connect(refreshCrystal_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        const int hz = i == 0 ? 72 : i == 1 ? 90 : 120;
        if (!chovrSetCrystalRefresh(chovrSettingsFile(), hz)) {
            say(QStringLiteral("Could not write %1").arg(chovrSettingsFile()), true);
            return;
        }
        // SteamVR reads the rate at its start: a running one is restarted (pibridge-keepalived does it)
        if (source_.state().steamvrRunning && !source_.fake() && actions_.crystalRestartSteamVr(statusDir())) {
            say(QStringLiteral("Crystal: %1 Hz, restarting SteamVR...").arg(hz));
        } else {
            say(QStringLiteral("Crystal: %1 Hz from the next SteamVR start.").arg(hz));
        }
    });

    // ---- controllers card (live from SteamVR, core/steamvr.h)
    auto* ctrl = new pibridge::ui::Card(QStringLiteral("Controllers"));
    ctrlChip_ = ctrl->chip();
    const QString sides[2] = {QStringLiteral("Left controller"), QStringLiteral("Right controller")};
    // as omarchyvr's controller rows: the picture, the name, the status chips, the buttons, then the model and serial
    const auto opButton = [](const QString& text, pibridge::ui::Icon icon) {
        auto* b = pibridge::ui::toolButton(text);
        b->setFont(pibridge::ui::labelFont(11));
        b->setIcon(pibridge::ui::lineIcon(icon, pibridge::ui::color::dim));
        b->setIconSize(QSize(14, 14));
        b->setMinimumHeight(28);
        return b;
    };
    for (int i = 0; i < 2; i++) {
        ControllerWidgets& w = ctrlRows_[i];
        auto* row = new QHBoxLayout;
        row->setSpacing(14);
        w.icon = new QLabel;
        w.icon->setFixedWidth(68);
        row->addWidget(w.icon, 0, Qt::AlignTop);
        auto* col = new QVBoxLayout;
        col->setSpacing(6);
        col->addWidget(pibridge::ui::capsLabel(sides[i], 13));
        auto* status = new QHBoxLayout;
        status->setSpacing(10);
        w.chip = new pibridge::ui::Chip;
        status->addWidget(w.chip);
        w.battery = new pibridge::ui::Battery;
        status->addWidget(w.battery);
        w.batteryText = new QLabel;
        w.batteryText->setFont(pibridge::ui::bodyFont(12));
        w.batteryText->setObjectName(QStringLiteral("dim"));
        status->addWidget(w.batteryText);
        status->addStretch(1);
        col->addLayout(status);
        auto* ops = new QHBoxLayout;
        ops->setSpacing(8);
        w.pair = opButton(QStringLiteral("PAIR"), pibridge::ui::Icon::Link);
        w.pair->setToolTip(QStringLiteral("Pair a controller with the headset (a short guide opens)"));
        w.unpair = opButton(QStringLiteral("UNPAIR"), pibridge::ui::Icon::Unlink);
        w.unpair->setToolTip(QStringLiteral("Forget this controller in the headset"));
        w.buzz = opButton(QStringLiteral("VIBRATE"), pibridge::ui::Icon::Pulse);
        w.buzz->setToolTip(QStringLiteral("A short buzz (60 %, 0.3 s)"));
        const int slot = i == 0 ? kSlotLeft : kSlotRight;
        connect(w.pair, &QPushButton::clicked, this, [this, slot] { showPairDialog(slot); });
        connect(w.unpair, &QPushButton::clicked, this, [this, slot] { controllerAction(slot, 1); });
        connect(w.buzz, &QPushButton::clicked, this, [this, slot] { controllerAction(slot, 2); });
        ops->addWidget(w.pair);
        ops->addWidget(w.unpair);
        ops->addWidget(w.buzz);
        ops->addStretch(1);
        col->addLayout(ops);
        w.model = pibridge::ui::dimLabel(QString());
        w.model->setFont(pibridge::ui::bodyFont(12));
        col->addWidget(w.model);
        w.detail = new QLabel;
        w.detail->setObjectName(QStringLiteral("dim"));
        w.detail->setFont(pibridge::ui::bodyFont(12));
        col->addWidget(w.detail);
        // the serial, for support: shown with a Copy button, never logged
        auto* sn = new QHBoxLayout;
        sn->setSpacing(8);
        w.serial = new QLabel;
        w.serial->setObjectName(QStringLiteral("dim"));
        w.serial->setFont(pibridge::ui::bodyFont(12));
        w.serial->setTextInteractionFlags(Qt::TextSelectableByMouse);
        sn->addWidget(w.serial);
        w.copy = new QPushButton(QStringLiteral("COPY"));
        w.copy->setObjectName(QStringLiteral("link"));
        w.copy->setCursor(Qt::PointingHandCursor);
        w.copy->setToolTip(QStringLiteral("Copy the serial number (for support)"));
        connect(w.copy, &QPushButton::clicked, this, [this, i] {
            QGuiApplication::clipboard()->setText(ctrlRows_[i].serialText);
        });
        sn->addWidget(w.copy);
        sn->addStretch(1);
        col->addLayout(sn);
        w.opText = pibridge::ui::dimLabel(QString());
        w.opText->setFont(pibridge::ui::bodyFont(12));
        w.opText->setWordWrap(true);
        w.opText->hide();
        col->addWidget(w.opText);
        row->addLayout(col, 1);
        ctrl->body()->addLayout(row);
        if (i == 0) {
            ctrl->body()->addWidget(pibridge::ui::rule());
        }
    }
    ctrlExtra_ = pibridge::ui::dimLabel(QString());
    ctrl->body()->addWidget(ctrlExtra_);
    ctrlText_ = pibridge::ui::dimLabel(QString());
    ctrl->body()->addWidget(ctrlText_);
    ctrl->body()->addStretch(1);
    // the top row: the headset card narrower than before, the controllers wider (5 : 4)
    auto* top = new QHBoxLayout;
    top->setSpacing(grid->horizontalSpacing());
    top->addWidget(hs, 5);
    top->addWidget(ctrl, 4);
    grid->addLayout(top, 0, 0, 1, 3);
    ctrlOpsTimer_ = new QTimer(this);
    ctrlOpsTimer_->setInterval(250);
    connect(ctrlOpsTimer_, &QTimer::timeout, this, [this] {
        ctrlOps_.poll(source_.state().keepalive, QDateTime::currentMSecsSinceEpoch());
        applyControllerOps();
        if (!ctrlOps_.busy()) {
            ctrlOpsTimer_->stop();
        }
    });
    connect(&source_, &StateSource::changed, this, &DashboardPage::applyControllerOps);

    // ---- quick settings
    auto* quick = new pibridge::ui::Card(QStringLiteral("Quick settings"));
    // Passthrough (the headset cameras shown around you), above Eye tracking like omarchyvr
    passthrough_ = new pibridge::ui::Toggle;
    passthrough_->setChecked(conf_.flag("passthrough", false));
    const QString passthroughTip = QStringLiteral("Shows the room through the headset cameras (passthrough). Saved as "
                                                  "passthrough in pibridge.conf.");
    passthrough_->setToolTip(passthroughTip);
    quick->body()->addWidget(
        pibridge::ui::settingRow(QStringLiteral("Passthrough"), passthrough_, nullptr, false, passthroughTip));
    connect(passthrough_, &QAbstractButton::toggled, this, [this](bool on) {
        conf_.set("passthrough", on ? "on" : "off");
        say(on ? QStringLiteral("Passthrough on.") : QStringLiteral("Passthrough off."));
    });
    // SteamVR dashboard panel (pibridge-overlay): a leaf launcher tab with ROOM SETUP / QUICK ROOM SETUP, right under
    // Passthrough. Clicking from inside VR launches the setup (the overlay writes a request the app watches).
    dashOverlay_ = new pibridge::ui::Toggle;
    dashOverlay_->setChecked(overlay_.wanted());
    const QString dashTip = QStringLiteral("Puts a small PiBridge tab in the SteamVR dashboard with a ROOM SETUP button, "
                                           "clickable with the controller laser. Saved as "
                                           "steamvr_dashboard in pibridge.conf; a leaf, so if it fails VR keeps running.");
    dashOverlay_->setToolTip(dashTip);
    quick->body()->addWidget(
        pibridge::ui::settingRow(QStringLiteral("SteamVR dashboard"), dashOverlay_, nullptr, false, dashTip));
    dashOverlayLine_ = pibridge::ui::dimLabel(QString());
    dashOverlayLine_->setFont(pibridge::ui::bodyFont(12));
    dashOverlayLine_->setWordWrap(true);
    dashOverlayLine_->hide(); // no status line here: the rows below keep their places (the tab's state is in its tooltip)
    connect(dashOverlay_, &QAbstractButton::toggled, this, [this](bool on) {
        if (!overlay_.setWanted(on)) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
        }
        updateOverlay();
    });
    connect(&overlay_, &OverlayControl::changed, this, &DashboardPage::updateOverlay);
    eyeToggle_ = new pibridge::ui::Toggle;
    const QString eyeTip = QStringLiteral("Eye tracking on or off (eye_tracking in pibridge.conf; starts or stops "
                                          "the pibridge-eyed service).");
    eyeToggle_->setToolTip(eyeTip);
    calibrate_ = pibridge::ui::linkButton(QStringLiteral("Calibrate"));
    recenter_ = pibridge::ui::linkButton(QStringLiteral("Recenter"));
    importEye_ = pibridge::ui::linkButton(QStringLiteral("Import"));
    importEye_->setToolTip(actions_.eyeTool().isEmpty()
                               ? QStringLiteral("pibridge-eye was not found (PATH or this build).")
                               : QStringLiteral("Reads calibration.setpm from Pimax Play / EVO under Windows and saves it "
                                                "as your eye profile (pibridge-eye profile)."));
    importEye_->setEnabled(!actions_.eyeTool().isEmpty());
    auto* eyeLinks = new QWidget;
    auto* el = new QHBoxLayout(eyeLinks);
    el->setContentsMargins(0, 0, 0, 0);
    el->setSpacing(4);
    el->addWidget(calibrate_);
    // only CALIBRATE here (user 2026-10-23): Recenter and the Windows import are not offered
    recenter_->hide();
    importEye_->hide();
    quick->body()->addWidget(pibridge::ui::settingRow(QStringLiteral("Eye tracking"), eyeToggle_, eyeLinks, false, eyeTip));
    // IPD in mm (ipd_mm): the lens motors are moved to it (as Pimax Play / EVO's IPD ADJUSTMENT) and the renderers
    // use the same value for the eye separation - Monado (OXR_DEBUG_IPD_MM, and live from pibridge.conf) and
    // CustomHeadsetOpenVR ("ipd", live from its settings.json). Sent 0.6 s after the last change so a drag does not
    // run the motors step by step.
    ipd_ = new pibridge::ui::SliderRow(QStringLiteral("IPD"), 58, 72, QStringLiteral("mm"), false,
                                       QStringLiteral("Moves the lenses to your IPD and renders with the same eye "
                                                      "distance, at once (Lighthouse and inside-out). Starts at the "
                                                      "lens position the headset reports."));
    ipd_->setRealRange(58.0, 72.0, 0.5, 1); // the motor's range and step
    quick->body()->addWidget(ipd_);
    auto* driverLink = pibridge::ui::linkButton(QStringLiteral("SBOYS3 driver settings"));
    driverLink->setToolTip(QStringLiteral("CustomHeadsetOpenVR's own settings (sboys3's driver: lens, brightness, "
                                          "resolution...)."));
    connect(driverLink, &QPushButton::clicked, this, &DashboardPage::openDriverSettings);
    quick->body()->addWidget(driverLink, 0, Qt::AlignLeft);
    ipdTimer_ = new QTimer(this);
    ipdTimer_->setSingleShot(true);
    ipdTimer_->setInterval(600);
    connect(ipd_, &pibridge::ui::SliderRow::realValueChanged, ipdTimer_, qOverload<>(&QTimer::start));
    connect(ipdTimer_, &QTimer::timeout, this, [this] {
        const double mm = ipd_->realValue();
        const QString v = QString::number(mm, 'f', 1);
        const bool confOk = conf_.set(reckey::kIpdMm, v.toStdString());
        const bool chovrOk = chovrSetIpd(chovrSettingsFile(), mm);
        // the lens motor: whoever owns the headset (pibridge-keepalived, or Monado's driver in inside-out) takes it
        bool motorOk = false;
        ipdSent_.start();
        {
            QDir().mkpath(statusDir());
            QSaveFile f(statusDir() + QStringLiteral("/ipd-request"));
            if (f.open(QIODevice::WriteOnly)) {
                f.write(v.toUtf8() + '\n');
                motorOk = f.commit();
            }
        }
        if (confOk && chovrOk && motorOk) {
            say(QStringLiteral("IPD %1 mm: lenses moving, rendering with %1 mm.").arg(v));
        } else {
            say(QStringLiteral("IPD %1 mm: could not write %2.")
                    .arg(v, !confOk ? conf_.path() : (!chovrOk ? chovrSettingsFile() : statusDir())),
                true);
        }
    });
    quick->body()->addWidget(pibridge::ui::rule());
    auto* eyeRow = new QHBoxLayout;
    // no gaze picture: it shows something only while the headset is on your head (user 2026-10-23)
    auto* eyeCol = new QVBoxLayout;
    eyeCol->setSpacing(6);
    auto* eyeHead = new QHBoxLayout;
    eyeHead->addWidget(pibridge::ui::capsLabel(QStringLiteral("Eye tracker"), 13));
    eyeHead->addStretch(1);
    eyeChip_ = new pibridge::ui::Chip;
    eyeHead->addWidget(eyeChip_);
    eyeCol->addLayout(eyeHead);
    eyeLine_ = pibridge::ui::dimLabel(QString());
    eyeCol->addWidget(eyeLine_);
    eyeCol->addStretch(1);
    eyeRow->addLayout(eyeCol, 1);
    quick->body()->addLayout(eyeRow);
    // the calibration's result under the eye tracker (not between IPD and it: the rows above never move)
    toolResult_ = pibridge::ui::dimLabel(QString());
    toolResult_->setWordWrap(true);
    toolResult_->hide();
    quick->body()->addWidget(toolResult_);
    quick->body()->addStretch(1);
    grid->addWidget(quick, 1, 0);

    connect(eyeToggle_, &QAbstractButton::toggled, this, [this](bool on) {
        const bool ok = actions_.setEyeTracking(on, [this, on](int code, const QString& out) {
            if (code == 0) {
                say(on ? QStringLiteral("Eye tracking on.") : QStringLiteral("Eye tracking off."));
            } else {
                say(QStringLiteral("The setting is saved, but the eye service did not %1: %2")
                        .arg(on ? QStringLiteral("start") : QStringLiteral("stop"), tail(out)),
                    true);
            }
            source_.refresh();
        });
        if (!ok) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            applyConf();
        }
    });
    connect(calibrate_, &QPushButton::clicked, this, [this] {
        setBusy(QStringLiteral("Calibrating in the headset: A starts, B at the end saves, B on the grid exits..."));
        if (!actions_.calibrate([this](int c, const QString& o) { toolDone(QStringLiteral("Calibration"), c, o); })) {
            setBusy({});
        }
    });
    connect(importEye_, &QPushButton::clicked, this, [this] {
        const QString file = QFileDialog::getOpenFileName(
            this, QStringLiteral("Calibration from Pimax Play / EVO (Windows)"),
            QStandardPaths::writableLocation(QStandardPaths::HomeLocation),
            QStringLiteral("Pimax calibration (calibration.setpm *.setpm);;All files (*)"));
        if (file.isEmpty()) {
            return;
        }
        importEye_->setEnabled(false);
        const bool started = actions_.importProfile(file, [this](int code, const QString& out) {
            importEye_->setEnabled(true);
            say(code == 0 ? QStringLiteral("Eye calibration saved to %1. Eye tracking uses it from now on.")
                                .arg(actions_.eyeProfileOut())
                          : QStringLiteral("Eye calibration import failed (%1): %2").arg(code).arg(out.trimmed()),
                code != 0);
        });
        if (!started) {
            importEye_->setEnabled(true);
            say(QStringLiteral("Could not start pibridge-eye."), true);
        }
    });
    connect(recenter_, &QPushButton::clicked, this, [this] {
        setBusy(QStringLiteral("Recentering in the headset: A starts, B at the end saves, B on the grid exits..."));
        if (!actions_.recenter([this](int c, const QString& o) { toolDone(QStringLiteral("Recenter"), c, o); })) {
            setBusy({});
        }
    });

    // ---- tracking (base stations from SteamVR)
    auto* tracking = new pibridge::ui::Card(QStringLiteral("Tracking"));
    trChip_ = tracking->chip();
    // The head-tracking mode selector lives in the card header (right of the title, where the mode type is shown), the
    // same control as the Tracking page: switching saves tracking_mode and refreshes. The selector shows the mode, so
    // the chip stays hidden.
    trModeSel_ = new pibridge::ui::Segmented({QStringLiteral("Lighthouse"), QStringLiteral("Inside-out")});
    trModeSel_->setItemIcon(0, pibridge::ui::lineIcon(pibridge::ui::Icon::Tracking, pibridge::ui::color::dim, Qt::white));
    trModeSel_->setItemIcon(1, pibridge::ui::lineIcon(pibridge::ui::Icon::Dashboard, pibridge::ui::color::dim, Qt::white));
    trModeSel_->setToolTip(QStringLiteral("Lighthouse: SteamVR base stations (works today). Inside-out: the headset's "
                                          "own cameras (SLAM, experimental)."));
    connect(trModeSel_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        if (source_.state().keepalive.crystalPresent) {
            // the original Crystal switches its own tracking (Lighthouse faceplate / its inside-out), in the headset
            say(actions_.setCrystalTracking(i == 0)
                    ? (i == 0 ? QStringLiteral("Crystal: switching to Lighthouse (about 10 s)...")
                              : QStringLiteral("Crystal: switching to its own inside-out tracking..."))
                    : QStringLiteral("Could not write %1").arg(conf_.path()),
                false);
            conf_.reload(); // the selector shows the chosen mode from now on
            updateMode();
            return;
        }
        conf_.set(iokey::kMode, i == 1 ? "inside-out" : "lighthouse");
        io_.refresh();
        updateMode();
        applyState();
    });
    tracking->header()->addWidget(trModeSel_, 0, Qt::AlignVCenter);
    // the VR backend (core/vr_backend.h): a choice, labelled, first in the card - who runs the headset and the apps
    backendSel_ = new pibridge::ui::Segmented({QStringLiteral("SteamVR"), QStringLiteral("Monado")});
    backendSel_->setToolTip(QStringLiteral(
        "SteamVR: PiBridge starts SteamVR with its head tracking (CustomHeadsetOpenVR); games start through Steam. "
        "Monado: PiBridge's own OpenXR runtime with the Pimax driver (inside-out or Lighthouse), PiBridge's lens "
        "profile, eye tracking and quad views. Each keeps its own rooms. Changes only while VR is stopped."));
    connect(backendSel_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        if (io_.active() || direct_.active()) {
            say(QStringLiteral("Stop VR first: the backend changes only between sessions."), true);
            updateMode();
            return;
        }
        const VrBackend b = i == 1 ? VrBackend::Monado : VrBackend::SteamVr;
        if (!conf_.set(kVrBackendKey, vrBackendName(b).toStdString())) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            return;
        }
        exportVrBackend(b);
        io_.setVariant(b == VrBackend::Monado ? IoVariant::MonadoDirect : IoVariant::SteamVr);
        overlay_.setBlocked(b == VrBackend::Monado);
        say(QStringLiteral("VR backend: %1.").arg(vrBackendWord(b)));
        updateMode();
        applyState();
    });
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(10);
        auto* lbl = pibridge::ui::capsLabel(QStringLiteral("VR backend"), 13);
        row->addWidget(lbl);
        row->addWidget(backendSel_);
        row->addStretch(1);
        tracking->body()->addLayout(row);
    }
    trBaseHead_ = new QWidget; // the base stations' summary: Lighthouse only
    auto* trHead = new QHBoxLayout(trBaseHead_);
    trHead->setContentsMargins(0, 0, 0, 0);
    trHead->setSpacing(12);
    auto* trIcon = new QLabel;
    trIcon->setPixmap(pibridge::ui::lineIcon(pibridge::ui::Icon::Tracking, pibridge::ui::color::accentText).pixmap(36, 36));
    trHead->addWidget(trIcon);
    trSummary_ = new QLabel;
    trSummary_->setFont(pibridge::ui::bodyFont(15));
    trHead->addWidget(trSummary_, 1);
    tracking->body()->addWidget(trBaseHead_);
    trStations_ = new QWidget;
    auto* stl = new QVBoxLayout(trStations_);
    stl->setContentsMargins(0, 0, 0, 0);
    stl->setSpacing(6);
    tracking->body()->addWidget(trStations_);
    trText_ = pibridge::ui::dimLabel(QString());
    tracking->body()->addWidget(trText_);
    tracking->body()->addStretch(1);
    // no mode sentence and no settings link: the mode is the selector in the header (user 2026-10-23)
    trMode_ = pibridge::ui::dimLabel(QString());
    trMode_->hide();
    // START VR / STOP now live in the headset box above (moved there 2026-10-20, like omarchyvr).
    connect(&io_, &IoSessionControl::changed, this, &DashboardPage::updateMode);
    connect(&direct_, &IoSessionControl::changed, this, &DashboardPage::updateMode);
    grid->addWidget(tracking, 1, 1);

    // ---- sound
    // as omarchyvr's Sound card: Headset sound / Headset mic, each with its test and its switch, then Input now
    auto* sound = new pibridge::ui::Card(QStringLiteral("Sound"));
    soundPanel_ = new SoundTestPanel(source_.fake()); // pages/sound_tests.h: the tests, the level meter, the result
    const auto testAndSwitch = [](QWidget* test, QWidget* sw) {
        auto* w = new QWidget;
        auto* h = new QHBoxLayout(w);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(10);
        h->addWidget(test);
        h->addWidget(sw);
        return w;
    };
    audio_ = new pibridge::ui::Toggle;
    const QString audioTip = QStringLiteral("The headset is the default audio output while it is on "
                                            "(audio_to_headset in pibridge.conf).");
    audio_->setToolTip(audioTip);
    sound->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Headset sound"), testAndSwitch(soundPanel_->speakersButton(), audio_), nullptr, false, audioTip));
    mic_ = new pibridge::ui::Toggle;
    const QString micTip = QStringLiteral("The headset's microphone is the default input while it is on "
                                          "(mic_to_headset in pibridge.conf).");
    mic_->setToolTip(micTip);
    sound->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Headset mic"), testAndSwitch(soundPanel_->micButton(), mic_), nullptr, false, micTip));
    inputNow_ = new QLabel;
    inputNow_->setFont(pibridge::ui::bodyFont(13));
    sound->body()->addWidget(pibridge::ui::settingRow(QStringLiteral("Input now"), inputNow_));
    sound->body()->addWidget(soundPanel_);
    sound->body()->addStretch(1);
    grid->addWidget(sound, 1, 2);
    connect(audio_, &QAbstractButton::toggled, this, [this](bool on) {
        if (!actions_.setAudioToHeadset(on, {})) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            applyConf();
            return;
        }
        say(on ? QStringLiteral("Sound goes to the headset while it is on.")
               : QStringLiteral("Sound stays on the current output."));
    });
    connect(mic_, &QAbstractButton::toggled, this, [this](bool on) {
        if (!actions_.setMicToHeadset(on)) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            applyConf();
            return;
        }
        say(on ? QStringLiteral("The headset's microphone is the input while it is on.")
               : QStringLiteral("The input stays as it is."));
    });
    auto* inputTimer = new QTimer(this);
    inputTimer->setInterval(3000);
    connect(inputTimer, &QTimer::timeout, this, &DashboardPage::updateInputNow);
    inputTimer->start();
    updateInputNow();

    // ---- record video (pibridge-record, pages/record_card.h): works while Monado's own compositor runs
    record_ = new RecordCard(source_, direct_);
    connect(record_, &RecordCard::message, this, [this](const QString& t, bool e) { say(t, e); });
    grid->addWidget(record_, 2, 0, 1, 3);

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(2, 1);
    // Cards are content-sized. A modest minimum keeps a changing status line from jumping the layout, without padding
    // the boxes with empty space.
    hs->setMinimumHeight(210);
    for (pibridge::ui::Card* c : {quick, tracking, sound}) {
        c->setMinimumHeight(150);
    }

    gazeTimer_.setInterval(33); // ~30 Hz, only while the page is visible
    connect(&gazeTimer_, &QTimer::timeout, this, &DashboardPage::updateGaze);
    connect(&source_, &StateSource::changed, this, &DashboardPage::applyState);
    applyConf();
    applyState();
    updateToolButtons();
}

namespace {

// What fake mode shows in the Reset SteamVR dialog: the running SteamVR of the fake state, or nothing.
SteamVrScan fakeResetScan(bool active) {
    SteamVrScan s;
    s.steamRunning = true;
    if (!active) {
        return s;
    }
    const auto add = [&](int pid, const QString& name, qint64 runningS, QStringList sockets = {}) {
        SteamVrProc p;
        p.pid = pid;
        p.name = p.comm = name;
        p.runningS = runningS;
        p.sockets = std::move(sockets);
        s.procs.push_back(p);
    };
    add(4301, QStringLiteral("vrserver"), 1520, {QStringLiteral("@/steamvr/SteamVR_Namespace")});
    add(4318, QStringLiteral("vrcompositor"), 1515);
    add(4322, QStringLiteral("vrmonitor"), 1514);
    return s;
}

} // namespace

void DashboardPage::resetSteamVr() {
    if (closer_ || restarter_) {
        return; // one at a time
    }
    const AppState& st = source_.state();
    const SteamVrScan scan = source_.fake() ? fakeResetScan(st.steamvrRunning) : scanSteamVr();
    const ResetDialogModel m = resetDialogModel(scan, st.steamvr.state == VrLinkState::Connected);
    if (!confirmReset(m)) {
        return;
    }
    if (source_.fake()) {
        say(QStringLiteral("Fake mode: nothing was closed or started."));
        return;
    }
    if (m.mode == ResetDialogModel::Mode::RestartSteam) {
        restartSteam();
    } else {
        closeSteamVr(scan.targets(), true); // exactly the PIDs the dialog listed, then start it again
    }
}

bool DashboardPage::confirmReset(const ResetDialogModel& m) {
    QDialog dlg(this);
    dlg.setWindowTitle(m.title);
    dlg.setMinimumWidth(560);
    dlg.setStyleSheet(QStringLiteral("QDialog { background: #141417; } QLabel#warn { color: #f2b02e; }"));
    auto* v = new QVBoxLayout(&dlg);
    v->setContentsMargins(18, 16, 18, 14);
    v->setSpacing(10);
    auto* title = new QLabel(m.title);
    title->setFont(pibridge::ui::titleFont(22));
    v->addWidget(title);
    v->addWidget(pibridge::ui::dimLabel(m.intro));
    if (!m.lines.isEmpty()) {
        auto* list = new QVBoxLayout;
        list->setContentsMargins(10, 2, 0, 2);
        list->setSpacing(4);
        for (const QString& l : m.lines) {
            auto* row = new QLabel(QStringLiteral("\u2022  ") + l);
            row->setWordWrap(true);
            row->setTextInteractionFlags(Qt::TextSelectableByMouse);
            row->setFont(pibridge::ui::bodyFont(13));
            list->addWidget(row);
        }
        v->addLayout(list);
    }
    if (!m.warning.isEmpty()) {
        auto* w = new QLabel(m.warning);
        w->setObjectName(QStringLiteral("warn"));
        w->setWordWrap(true);
        v->addWidget(w);
    }
    auto* q = new QLabel(m.question);
    q->setWordWrap(true);
    v->addWidget(q);
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    auto* cancel = pibridge::ui::toolButton(QStringLiteral("Cancel"));
    auto* ok = pibridge::ui::accentButton(m.confirm);
    cancel->setDefault(true); // Enter does not close anything by accident
    buttons->addWidget(cancel);
    buttons->addWidget(ok);
    v->addLayout(buttons);
    connect(cancel, &QPushButton::clicked, &dlg, &QDialog::reject);
    connect(ok, &QPushButton::clicked, &dlg, &QDialog::accept);
    return dlg.exec() == QDialog::Accepted;
}

void DashboardPage::startVr() {
    const bool inside = monadoSession();
    if (inside) {
        // what a previous PiBridge left running (restarted while a session ran, SteamVR gone since) holds the cameras
        // and winbridge and would block the start
        if (!source_.fake() && !io_.active()) {
            if (const QStringList left = stopLeftoverSession(); !left.isEmpty()) {
                say(QStringLiteral("Stopped what an earlier session left running: %1.").arg(left.join(QStringLiteral(", "))));
            }
        }
        io_.refresh();
        if (!io_.start()) {
            say(QStringLiteral("VR could not start: %1")
                    .arg(io_.readiness().blocking.isEmpty() ? io_.message() : io_.readiness().blocking.join(QLatin1Char(' '))),
                true);
            return;
        }
        say(io_.variant() == IoVariant::MonadoDirect ? QStringLiteral("Starting VR: Monado (head tracking in the Pimax driver)...")
                                                     : QStringLiteral("Starting VR: head tracking and SteamVR..."));
        return;
    }
    const bool steamUp = source_.fake() || steamClientRunning();
    const bool started = actions_.launchSteamVR();
    if (!started) {
        say(QStringLiteral("Could not start Steam. Is it installed?"), true);
    } else if (!steamUp) {
        say(QStringLiteral("Steam is not running — starting it first; SteamVR will launch once Steam is up "
                           "(press START VR again if it does not)."));
    } else {
        say(QStringLiteral("Starting SteamVR..."));
    }
    waitingForSteamVr_ = started;
    if (started) {
        messageTimer_.start(120000);
    }
}

void DashboardPage::closeSteamVr(std::vector<SteamVrProc> targets, bool startAfter) {
    reset_->setEnabled(false);
    startAfterClose_ = startAfter;
    say(startAfter ? QStringLiteral("Restarting SteamVR: closing it...") : QStringLiteral("Closing SteamVR..."));
    closer_ = std::make_unique<SteamVrCloser>(std::move(targets), realProcSignals());
    resetClock_.start();
    closer_->begin(resetClock_.elapsed());
    resetTimer_.start();
}

void DashboardPage::restartSteam() {
    reset_->setEnabled(false);
    restarter_ = std::make_unique<SteamRestarter>(*steamRunner_, [] { return steamClientRunning(); });
    resetClock_.start();
    restarter_->begin(resetClock_.elapsed());
    say(restarter_->message());
    resetTimer_.start();
}

void DashboardPage::updateChovr() {
    // just resolves the CustomHeadsetOpenVR GUI path (chovrGui_); openDriverSettings() uses it (no top button any more)
    if (source_.fake()) {
        chovrGui_ = QStringLiteral("/fake/custom-headset-gui");
    } else {
        const bool active = chovrActive(openVrPathsFile(), chovrSettingsFile());
        chovrGui_ = active ? chovrGuiPath(conf_.value("chovr_gui", {}), QDir::homePath()) : QString();
    }
}

void DashboardPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    updateChovr();
    applyConf(); // the mode may have changed on the Tracking page
    updateGaze();
    gazeTimer_.start();
}

void DashboardPage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    gazeTimer_.stop();
}

void DashboardPage::say(const QString& text, bool error) {
    waitingForSteamVr_ = false;
    message_->setText(text);
    messageTimer_.start(error ? 20000 : 8000);
    message_->setObjectName(error ? QStringLiteral("error") : QStringLiteral("dim"));
    message_->style()->unpolish(message_);
    message_->style()->polish(message_);
}

void DashboardPage::openDriverSettings() {
    updateChovr();
    if (chovrGui_.isEmpty()) {
        say(QStringLiteral("CustomHeadsetOpenVR (the headset driver's settings) is not installed."), true);
        return;
    }
    const bool ok = startChovrGui(*steamRunner_, chovrGui_);
    say(ok ? QStringLiteral("Opening CustomHeadsetOpenVR's settings...") : QStringLiteral("Could not start %1").arg(chovrGui_),
        !ok);
}

void DashboardPage::openTrackingSettings() {
    // inside-out: the SLAM / tracking settings on the Tracking page; Lighthouse: the headset driver's configurator
    if (ioSettings(conf_).mode == TrackingMode::InsideOut) {
        emit openTracking();
    } else {
        openDriverSettings();
    }
}

void DashboardPage::updateFixCaps() {
    if (fixCaps_ == nullptr) {
        return;
    }
    if (source_.fake()) {
        fixCaps_->setVisible(false);
        return;
    }
    const RecSetup s = resolveRecSetup(conf_, RecEnv::defaults());
    fixCapsPath_ = s.monado.path;
    // only a session that runs monado-service needs it: the Monado backend, or SteamVR with the old driver_monado
    // (pibridge-headtrack does not)
    const bool usesMonado = s.backend == VrBackend::Monado || !s.io.headtrack;
    const bool need = usesMonado && !fixCapsPath_.isEmpty() && QFileInfo::exists(fixCapsPath_) &&
                      !fileCaps(fixCapsPath_).ok();
    fixCaps_->setVisible(need);
}

const IoSessionControl& DashboardPage::monadoControl() const {
    // the Monado backend's main session, else "Monado directly" (Development page)
    if (io_.variant() == IoVariant::MonadoDirect && io_.active()) {
        return io_;
    }
    return direct_;
}

void DashboardPage::updateRuntime() {
    const OpenXrRuntime r = activeRuntimeOf(activeRuntimePath());
    const QSignalBlocker block(runtime_);
    if (r == OpenXrRuntime::SteamVr || r == OpenXrRuntime::Monado) {
        if (runtime_->count() > 2) {
            runtime_->removeItem(2);
        }
        runtime_->setCurrentIndex(r == OpenXrRuntime::Monado ? 1 : 0);
    } else {
        if (runtime_->count() < 3) {
            runtime_->addItem(QString(), QString());
        }
        runtime_->setItemText(2, openXrRuntimeWord(r));
        runtime_->setCurrentIndex(2);
    }
}

bool DashboardPage::crystalOn() const {
    return source_.state().keepalive.crystalPresent;
}

bool DashboardPage::insideOutNow() const {
    const KeepaliveStatus& k = source_.state().keepalive;
    if (k.crystalPresent && !k.crystalLighthouse.isEmpty()) {
        return k.crystalLighthouse != QLatin1String("1");
    }
    return ioSettings(conf_).mode == TrackingMode::InsideOut;
}

bool DashboardPage::selectedInsideOut() const {
    // the original Crystal: the mode chosen for it (crystal_tracking), not the one it runs this second, so the selector
    // does not flip back and forth while it switches and SteamVR restarts; the headset card says what it runs
    if (crystalOn()) {
        const QString want = conf_.value("crystal_tracking", QString()).trimmed();
        if (want == QLatin1String("slam")) {
            return true;
        }
        if (want == QLatin1String("lighthouse")) {
            return false;
        }
    }
    return insideOutNow();
}

bool DashboardPage::monadoSession() const {
    // START VR runs PiBridge's session (io_): inside-out with SteamVR, or the Monado backend in either mode;
    // only SteamVR with Lighthouse is a plain SteamVR launch
    return !crystalOn() && (ioSettings(conf_).mode == TrackingMode::InsideOut || vrBackend(conf_) == VrBackend::Monado);
}

void DashboardPage::updateVrStatus() {
    const bool inside = monadoSession();
    const bool running = inside ? io_.active() : source_.state().steamvrRunning;
    if (running && vrStartMs_ == 0) {
        vrStartMs_ = QDateTime::currentMSecsSinceEpoch();
    } else if (!running) {
        vrStartMs_ = 0;
    }
    QString line;
    if (running) {
        const qint64 mins = (QDateTime::currentMSecsSinceEpoch() - vrStartMs_) / 60000;
        const qint64 h = mins / 60, m = mins % 60;
        line = h > 0 ? QStringLiteral("VR running %1 h %2 m").arg(h).arg(m) : QStringLiteral("VR running %1 m").arg(m);
    } else {
        line = QStringLiteral("VR not running");
        // SteamVR is a Steam app and cannot start unless the Steam client is running, so say so plainly: otherwise
        // START VR looks like it does nothing when Steam is closed.
        if (vrBackend(conf_) == VrBackend::Monado) {
            // the Monado backend never starts SteamVR; say what would block its start instead
            const RecReadiness& r = io_.readiness();
            if (!r.ready() && !r.blocking.isEmpty()) {
                line += QStringLiteral("      Cannot start: ") + r.blocking.first();
            }
        }
    }
    const VrDevice* hmd = nullptr;
    for (const VrDevice& d : source_.state().steamvr.devices) {
        if (d.cls == VrClass::Hmd) {
            hmd = &d;
            break;
        }
    }
    if (hmd != nullptr && hmd->renderWidth > 0) {
        line += QStringLiteral("      Resolution: %1 x %2").arg(hmd->renderWidth).arg(hmd->renderHeight);
    }
    vrStatus_->setText(line);
}

void DashboardPage::updateMode() {
    updateVrStatus();
    const bool inside = insideOutNow();
    if (trModeSel_ != nullptr) {
        trModeSel_->setCurrent(selectedInsideOut() ? 1 : 0); // the chosen mode (programmatic: no activated)
    }
    if (backendSel_ != nullptr) {
        backendSel_->setCurrent(vrBackend(conf_) == VrBackend::Monado ? 1 : 0);
        backendSel_->setEnabled(!io_.active() && !direct_.active());
    }
    const bool active = io_.active();
    if (crystalOn()) {
        trMode_->setText(inside ? QStringLiteral("Mode: Inside-out, tracked by the Crystal itself (SteamVR).")
                                : QStringLiteral("Mode: Lighthouse (SteamVR and the base stations)."));
    } else if (direct_.active()) {
        trMode_->setText(QStringLiteral("\"Monado directly\" runs (Development page). %1").arg(direct_.message()));
    } else if (!inside) {
        trMode_->setText(QStringLiteral("Mode: Lighthouse (SteamVR and the base stations)."));
    } else if (active || io_.state() != RecState::Idle) {
        trMode_->setText(QStringLiteral("Mode: Inside-out, %1. %2").arg(vrBackendWord(vrBackend(conf_)), io_.message()));
    } else {
        trMode_->setText(QStringLiteral("Mode: Inside-out, %1.").arg(vrBackendWord(vrBackend(conf_))));
    }
    const RecReadiness& r = io_.readiness();
    const bool steamvr = source_.state().steamvrRunning;
    const bool session = monadoSession();
    const bool running = session ? active : steamvr; // the same button toggles: blue START VR / red STOP VR
    ioStart_->setVisible(true);

    if (running) {
        ioStart_->setText(QStringLiteral("STOP VR"));
        ioStart_->setStyleSheet(QStringLiteral("QPushButton { background: #c0392b; color: #ffffff; border: none; "
                                               "border-radius: 4px; } QPushButton:hover { background: #d64533; }"));
        ioStart_->setEnabled(session ? io_.state() != RecState::Stopping : true);
        ioStart_->setToolTip(session      ? QStringLiteral("Stops the inside-out tracking session.")
                             : crystalOn() ? QStringLiteral("Closes SteamVR and switches the Crystal off.")
                                           : QStringLiteral("Closes SteamVR (the Reset flow)."));
    } else {
        ioStart_->setText(QStringLiteral("START VR"));
        ioStart_->setStyleSheet(QString()); // back to the accent (blue) style
        ioStart_->setEnabled(session ? r.ready() : true);
        ioStart_->setToolTip(session ? (!r.ready() ? r.blocking.join(QLatin1Char('\n'))
                                        : io_.variant() == IoVariant::MonadoDirect
                                            ? QStringLiteral("Starts Monado with the Pimax driver (and the camera broker "
                                                             "and Pimax's engine for inside-out) with the Tracking page's "
                                                             "settings.")
                                            : QStringLiteral("Starts the camera broker, head tracking "
                                                             "(pibridge-headtrack) and SteamVR with the Tracking "
                                                             "page's settings."))
                                    : QStringLiteral("Launches SteamVR (the base stations track the headset)."));
    }

    // the Controllers card reads Monado (libmonado) while a Monado session runs (the Monado backend's or "Monado
    // directly"), SteamVR otherwise
    const IoSessionControl& mc = monadoControl();
    const bool monado = mc.active();
    updateRuntime();
    if (monado && !mnd_) {
        mnd_ = std::make_unique<MonadoWorker>(
            source_.fake() ? [] {
                auto f = std::make_unique<FakeMonadoSource>();
                f->polls = {fakeMonadoDevices()};
                return std::unique_ptr<MonadoSource>(std::move(f));
            }()
                           : makeLibMonadoSource(),
            [this](const MonadoStatus& st) {
                QMetaObject::invokeMethod(this, [this, st] {
                    mndStatus_ = st;
                    applyState();
                }, Qt::QueuedConnection);
            });
    }
    if (mnd_) {
        const QString svc = mc.setup().monado.path;
        if (svc != mndService_) {
            mndService_ = svc; // looked up again only when the program changes (this runs at every session tick)
            mndLibrary_ = source_.fake() ? QStringLiteral("/fake/monado/libmonado.so") : libmonadoFor(svc);
        }
        MonadoTarget t;
        t.wanted = monado && mc.state() == RecState::Recording;
        t.library = mndLibrary_;
        t.socketPath = source_.fake() ? QString() : monadoSocketPath(RecEnv::defaults().runtimeDir);
        mnd_->setTarget(t);
    }
    if (monado != monadoMode_) {
        monadoMode_ = monado;
        if (!monado) {
            mndStatus_ = {};
        }
        applyState();
    }
}

void DashboardPage::applyConf() {
    conf_.reload();
    refresh_->setCurrent(conf_.refreshHz() == 72 ? 0 : 1);
    eyeToggle_->setCheckedQuiet(conf_.eyeTracking());
    audio_->setCheckedQuiet(conf_.audioToHeadset());
    mic_->setCheckedQuiet(conf_.micToHeadset());
    {
        bool ok = false;
        const double ipd = conf_.value(reckey::kIpdMm, QString()).trimmed().toDouble(&ok);
        if (!ipdFromHeadset_) {
            ipd_->setRealValueQuiet(ok && ipd >= 58.0 && ipd <= 72.0 ? ipd : reckey::kIpdDefaultMm);
        }
    }
    io_.refresh();
    updateMode();
}

void DashboardPage::updateOverlay() {
    if (dashOverlay_ == nullptr) {
        return;
    }
    const OverlayStatus s = overlay_.status(source_.state().steamvrRunning);
    // the Monado backend: the SteamVR panel is off and locked (its own dashboard in the headset does the same job)
    dashOverlay_->setEnabled(!overlay_.blocked());
    dashOverlay_->setCheckedQuiet(overlay_.wanted() && !overlay_.blocked());
    dashOverlay_->setToolTip(s.words.isEmpty() ? dashOverlay_->toolTip().section(QStringLiteral("\n\n"), 0, 0)
                                               : dashOverlay_->toolTip().section(QStringLiteral("\n\n"), 0, 0) +
                                                     QStringLiteral("\n\n") + s.words);
    dashOverlayLine_->setStyleSheet(s.error ? QStringLiteral("color: #ef5a55;")
                                    : s.run == OverlayRun::WaitingForSteamVr ? QStringLiteral("color: #f2b02e;")
                                                                             : QString());
}

void DashboardPage::applyState() {
    updateFixCaps();
    if (isVisible() && !io_.active()) {
        io_.refresh(); // SteamVR started or stopped: START TRACKING's reasons follow
        updateMode();
    }
    const AppState& s = source_.state();
    if (waitingForSteamVr_ && s.steamvrRunning) {
        say(QStringLiteral("SteamVR is running."));
    }
    const KeepaliveStatus& k = s.keepalive;
    // the original Crystal: battery, switch-off button, IPD by its own buttons only, the tracking selector shows its mode
    const bool crystal = k.crystalPresent;
    const bool batKnown = crystal && k.crystalBattery >= 0;
    hsBattery_->set(batKnown ? k.crystalBattery : -1, false);
    hsBatteryText_->setText(batKnown ? QStringLiteral("%1 %").arg(k.crystalBattery) : QString());
    for (QWidget* w : {static_cast<QWidget*>(hsBatteryLabel_), static_cast<QWidget*>(hsBattery_),
                       static_cast<QWidget*>(hsBatteryText_)}) {
        w->setVisible(batKnown);
    }
    crystalOffBtn_->setVisible(crystal);
    refresh_->setVisible(!crystal);
    refreshCrystal_->setVisible(crystal);
    if (crystal && !wasCrystalRefresh_) {
        const int hz = chovrCrystalRefresh(chovrSettingsFile());
        refreshCrystal_->setCurrent(hz == 72 ? 0 : hz == 120 ? 2 : 1); // 0 / absent: CHOVR reports 90
    }
    wasCrystalRefresh_ = crystal;
    if (crystal && !crystalChovrChecked_) {
        crystalChovrChecked_ = true; // once per app start
        bool changed = false;
        if (chovrEnableCrystalOg(chovrSettingsFile(), &changed) && changed) {
            say(QStringLiteral("Enabled the original Crystal in CustomHeadsetOpenVR (%1).").arg(chovrSettingsFile()));
        }
    }
    ipd_->setEnabled(!crystal);
    ipd_->setToolTip(crystal ? QStringLiteral("The Crystal's IPD is set with the buttons on the headset; the slider "
                                              "follows them and the picture uses the same value.")
                             : QStringLiteral("Moves the lenses to your IPD and renders with the same eye distance, at "
                                              "once (Lighthouse and inside-out). Starts at the lens position the "
                                              "headset reports."));
    if (crystal && !k.crystalLighthouse.isEmpty()) {
        trModeSel_->setCurrent(selectedInsideOut() ? 1 : 0);
    }
    // The lens IPD the headset reports: at start and whenever it changes (the IPD buttons on the headset move the
    // lenses themselves), put the slider there and render with it; the motors are not touched. Not while the user is
    // moving the slider, nor for a few seconds after our own request (the lenses are still travelling).
    const bool settled = !ipdTimer_->isActive() && (!ipdSent_.isValid() || ipdSent_.elapsed() > 4000);
    if (settled && k.lensIpdMm >= 58.0 && k.lensIpdMm <= 72.0 &&
        (!ipdFromHeadset_ || std::abs(k.lensIpdMm - lastLensIpdMm_) >= 0.25)) {
        ipdFromHeadset_ = true;
        lastLensIpdMm_ = k.lensIpdMm;
        const double mm = std::round(k.lensIpdMm * 2.0) / 2.0;
        ipd_->setRealValueQuiet(mm);
        const QString v = QString::number(mm, 'f', 1);
        if (conf_.value(reckey::kIpdMm, QString()).trimmed() != v) {
            conf_.set(reckey::kIpdMm, v.toStdString());
            chovrSetIpd(chovrSettingsFile(), mm);
        }
    }
    // During a live SteamVR session keepalived is paused (Monado owns the headset HID) and reports no model/serial, so
    // read the headset from what SteamVR shows instead.
    const VrDevice* hmd = nullptr;
    if (s.steamvrRunning) {
        for (const VrDevice& d : s.steamvr.devices) {
            if (d.cls == VrClass::Hmd) {
                hmd = &d;
                break;
            }
        }
    }
    const bool present = (k.running && k.headsetPresent) || k.crystalPresent || hmd != nullptr || monadoControl().active();
    // the original Crystal: SteamVR shows its Lighthouse module ("REF-HMD"), keepalived knows the headset
    const QString model = k.crystalPresent                               ? QStringLiteral("Pimax Crystal")
                          : (k.running && !k.model.isEmpty())             ? k.model
                          : (hmd != nullptr && !hmd->model.isEmpty())    ? hmd->model
                                                                         : QStringLiteral("Headset");
    hsTitle_->setText(present ? model : QStringLiteral("No headset"));
    // The tracking mode: the original Crystal's own, else the settings (the Super's faceplate we cannot tell).
    const bool insideOut = insideOutNow();
    // the original Crystal: its own inside-out without a pose right now (covered cameras, a dark room, just switched)
    const bool poseLost = k.crystalPresent && insideOut && !k.crystalTracked;
    hsSub_->setText(QStringLiteral("%1%2 · SteamVR")
                        .arg(insideOut ? QStringLiteral("Inside-out") : QStringLiteral("Lighthouse"),
                             poseLost ? QStringLiteral(" · tracking lost") : QString()));
    // The chip: during a live session show the panel state, not keepalived's internal "paused".
    hsChip_->set(hmd != nullptr ? ChipText{QStringLiteral("SCREEN ON"), ChipKind::Info} : headsetChip(k));
    hsHint_->setText(headsetHint(k, s.steamvrRunning));
    // "Monado directly" owns the headset: keepalived follows vrserver and only waits, so say what Monado does
    const IoSessionControl& hc = monadoControl();
    const MonadoHeadsetWords mw = hc.active() ? monadoHeadsetWords(hc.state(), hc.message()) : MonadoHeadsetWords{};
    if (!mw.state.isEmpty()) {
        hsHint_->setText(mw.hint);
    }
    hsError_->setText(k.running && k.state != QLatin1String("error") ? k.error : QString());
    hsError_->setVisible(!hsError_->text().isEmpty());
    art_->setLit(k.running && k.panelOn);
    if (hmd != nullptr) {
        // A live SteamVR session owns the headset: show what it is doing, not keepalived's internal "paused".
        hsState_->setText(QStringLiteral("Active"));
    } else if (!k.running) {
        hsState_->setText(mw.state.isEmpty() ? QStringLiteral("Service not running") : mw.state);
    } else {
        static const QHash<QString, QString> words{
            {QStringLiteral("idle"), QStringLiteral("Waiting for SteamVR")},
            {QStringLiteral("waking"), QStringLiteral("Waking")},
            {QStringLiteral("active"), QStringLiteral("Active")},
            {QStringLiteral("no-headset"), QStringLiteral("No headset")},
            {QStringLiteral("error"), QStringLiteral("Error")},
            {QStringLiteral("paused"), QStringLiteral("Paused: inside-out tracking")}};
        hsState_->setText(!mw.state.isEmpty() && (k.state == QLatin1String("idle") || k.state == QLatin1String("paused")) ? mw.state : words.value(k.state, k.state));
    }
    // Serial: from the SteamVR headset during a session; keepalived does not report one yet. "—" when unknown.
    // the headset's and its optical module's own serials (read by the headset service), else SteamVR's
    const QString ownHs = crystalOn() ? QString() : keptSerial(QStringLiteral("headset"));
    const QString hsSerial = !ownHs.isEmpty() ? ownHs : (hmd != nullptr && !hmd->serial.isEmpty()) ? hmd->serial : QString();
    hsSerial_->setText(hsSerial.isEmpty() ? QStringLiteral("—") : hsSerial);
    const QString optics = crystalOn() ? QString() : keptSerial(QStringLiteral("optics"));
    hsOptics_->setText(optics.isEmpty() ? QStringLiteral("—") : optics);
    hsOpticsLabel_->setVisible(!crystalOn());
    hsOptics_->setVisible(!crystalOn());
    // Room setup and the sound tests need a live SteamVR session; grey them out until it runs. Room setup builds the
    // inside-out room map: in Lighthouse mode SteamVR's own Room Setup does that job (user 2026-10-22).
    const bool roomInside = insideOutNow();
    // the room setup needs the running session of the backend: SteamVR (inside-out), or Monado (either mode)
    const bool monadoBackend = vrBackend(conf_) == VrBackend::Monado;
    roomBtn_->setEnabled(monadoBackend ? monadoControl().active() : s.steamvrRunning && roomInside);
    roomBtn_->setToolTip(monadoBackend ? QStringLiteral("Room setup in the headset for Monado (its own rooms).")
                         : roomInside  ? QStringLiteral("Room setup in the headset (floor, boundary, room map).")
                                       : QStringLiteral("Lighthouse: SteamVR's own Room Setup sets up the room."));
    if (soundPanel_ != nullptr) {
        soundPanel_->setVrRunning(s.steamvrRunning);
    }
    updateOverlay(); // SteamVR up/down changes the launcher's status words
    // the Monado backend shows Monado's controllers (MONADO OFF until its session runs), SteamVR's otherwise
    const ControllersView cv = monadoMode_                              ? monadoControllersView(mndStatus_)
                               : vrBackend(conf_) == VrBackend::Monado ? monadoControllersView(MonadoStatus{})
                                                                        : controllersView(s.steamvr, s.steamvrRunning);
    ctrlChip_->set(cv.header);
    const bool connected = monadoMode_ ? mndStatus_.state == MndLinkState::Connected
                                       : s.steamvrRunning && s.steamvr.state == VrLinkState::Connected;
    const ControllerRow* rows[2] = {&cv.left, &cv.right};
    for (int i = 0; i < 2; i++) {
        const ControllerRow& r = *rows[i];
        ControllerWidgets& w = ctrlRows_[i];
        const QColor ic = !connected || !r.present        ? pibridge::ui::color::faint
                          : r.chip.kind == ChipKind::Info ? pibridge::ui::color::accentText
                                                          : pibridge::ui::color::amber;
        w.icon->setPixmap(pibridge::ui::controllerPixmap(84, ic, i == 1));
        // no model line: SteamVR's name is the profile's ("Oculus Touch Controller"), not the controller's
        w.model->setVisible(false);
        w.detail->setText(connected ? r.detail : QString());
        w.detail->setVisible(!w.detail->text().isEmpty());
        w.chip->set(connected ? r.chip : ChipText{});
        ctrlSeen_[i] = connected && r.present;
        w.battery->set(connected ? r.batteryPercent : -1, r.charging);
        w.battery->setVisible(connected && r.batteryPercent >= 0);
        w.batteryText->setText(connected ? r.batteryText : QString());
        w.batteryText->setVisible(!w.batteryText->text().isEmpty());
        // the controller's own serial, read from it by the headset service (ctrl_serials.h); SteamVR only knows
        // CustomHeadsetOpenVR's names ("pimaxleft")
        const QString own = keptSerial(i == 0 ? QStringLiteral("left") : QStringLiteral("right"));
        w.serialText = !connected ? QString() : !own.isEmpty() ? own : r.serial;
        w.serial->setText(w.serialText.isEmpty() ? QString() : QStringLiteral("SN ") + w.serialText);
        w.serial->setVisible(!w.serialText.isEmpty());
        w.copy->setVisible(!w.serialText.isEmpty());
    }
    applyControllerOps(); // the buttons follow what SteamVR shows
    ctrlExtra_->setText(cv.extra.isEmpty() ? QString() : QStringLiteral("Also: ") + cv.extra.join(QStringLiteral("\n")));
    ctrlExtra_->setVisible(!cv.extra.isEmpty());
    ctrlText_->setText(cv.message);
    ctrlText_->setVisible(!cv.message.isEmpty());

    TrackingView tv = trackingView(s.steamvr, s.steamvrRunning);
    if (monadoMode_) {
        tv = {}; // SteamVR does not run in this mode: no base stations to show
        tv.message = QStringLiteral("Inside-out tracking: the headset's cameras, no base stations.");
    }
    trSummary_->setText(tv.summary.isEmpty() ? QStringLiteral("Base stations") : tv.summary);
    trText_->setText(tv.message);
    trText_->setVisible(!tv.message.isEmpty());
    QString key;
    for (const StationRow& st : tv.stations) {
        key += st.name + st.model + st.text + st.serial + QLatin1Char('|');
    }
    if (key != trKey_) {
        trKey_ = key;
        auto* lay = static_cast<QVBoxLayout*>(trStations_->layout());
        while (QLayoutItem* it = lay->takeAt(0)) {
            delete it->widget();
            delete it;
        }
        for (const StationRow& st : tv.stations) {
            auto* row = new QWidget;
            auto* h = new QHBoxLayout(row);
            h->setContentsMargins(0, 0, 0, 0);
            h->setSpacing(10);
            auto* n = pibridge::ui::capsLabel(st.name, 12);
            n->setMinimumWidth(90);
            h->addWidget(n);
            auto* m = new QLabel(st.model);
            m->setObjectName(QStringLiteral("dim"));
            h->addWidget(m, 1);
            if (!st.serial.isEmpty()) {
                auto* c = new QPushButton(QStringLiteral("COPY SN"));
                c->setObjectName(QStringLiteral("link"));
                c->setCursor(Qt::PointingHandCursor);
                c->setToolTip(QStringLiteral("SN ") + st.serial);
                const QString sn = st.serial;
                connect(c, &QPushButton::clicked, row, [sn] { QGuiApplication::clipboard()->setText(sn); });
                h->addWidget(c);
            }
            auto* t = new QLabel(st.text);
            t->setFont(pibridge::ui::labelFont(12));
            t->setStyleSheet(QStringLiteral("color: %1;").arg(st.tracking ? pibridge::ui::color::accentText.name()
                                                                          : pibridge::ui::color::amber.name()));
            h->addWidget(t);
            lay->addWidget(row);
        }
    }
    // base stations only in Lighthouse mode
    const bool lighthouse = !insideOutNow();
    trBaseHead_->setVisible(lighthouse);
    trText_->setVisible(lighthouse && !trText_->text().isEmpty()); // "The base stations show while VR runs"
    trStations_->setVisible(lighthouse && !tv.stations.empty());
    eyeChip_->set(eyeChip(s.eyed));
    eyeLine_->setText(eyeLine(s.eyed) +
                      (s.eyed.running && s.eyed.profile == QLatin1String("none")
                                 ? QStringLiteral("\nNo calibration profile yet: press Calibrate.")
                                 : QString()));
}

void DashboardPage::updateGaze() {
    const auto g = source_.gaze();
    std::optional<std::pair<float, float>> t;
    if (g && g->state == 2 && monotonicNs() - g->writtenNs < 200'000'000) { // older than 200 ms: the tracker stopped
        float x = 0, y = 0;
        if (gazeTangent(*g, x, y)) {
            t = std::pair{x, y};
        }
    }
    if (gaze_ != nullptr) {
        gaze_->setGaze(t);
    }
}

void DashboardPage::setBusy(const QString& what) {
    busy_ = what;
    if (!what.isEmpty()) {
        toolResult_->setText(what);
        toolResult_->setVisible(!what.isEmpty());
    }
    updateToolButtons();
}

void DashboardPage::toolDone(const QString& what, int code, const QString& out) {
    busy_.clear();
    toolResult_->show();
    toolResult_->setText(code == 0 ? QStringLiteral("%1 done. %2").arg(what, tail(out))
                                   : QStringLiteral("%1 failed (%2): %3").arg(what).arg(code).arg(tail(out)));
    updateToolButtons();
}

void DashboardPage::updateToolButtons() {
    const bool eyecal = !actions_.eyecalTool().isEmpty();
    const bool idle = busy_.isEmpty();
    calibrate_->setEnabled(eyecal && idle);
    recenter_->setEnabled(eyecal && idle);
    const QString missingCal = QStringLiteral("pibridge-eyecal was not found (PATH or this build).");
    calibrate_->setToolTip(eyecal ? QStringLiteral("Calibrates the eye tracker itself, as Pimax Play does. The headset shows a grid "
                                                   "with your gaze; A starts, follow the green dot, B saves and "
                                                   "shows the grid with the new calibration, B there exits.")
                                  : missingCal);
    recenter_->setToolTip(eyecal ? QStringLiteral("A quick re-centre: 5 dots, removes a constant offset. Starts and ends on the "
                                                  "gaze grid.")
                                 : missingCal);
}

} // namespace pibridge::app

namespace pibridge::app {

void DashboardPage::controllerAction(int slot, int action) {
    const KeepaliveStatus& k = source_.state().keepalive;
    if (action == 1) {
        const auto b = QMessageBox::question(
            this, QStringLiteral("Unpair controller"),
            QStringLiteral("Unpair the %1 controller? It stops working until you pair it again.").arg(sideName(slot)));
        if (b != QMessageBox::Yes) {
            return;
        }
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (action == 0) {
        (void)ctrlOps_.pair(slot, k, now);
    } else if (action == 1) {
        (void)ctrlOps_.unpair(slot, k, now);
    } else {
        (void)ctrlOps_.testVibration(slot, k, now);
    }
    if (ctrlOps_.busy()) {
        ctrlOpsTimer_->start();
    }
    applyControllerOps();
}

void DashboardPage::applyControllerOps() {
    const KeepaliveStatus& k = source_.state().keepalive;
    const bool paused = k.running && k.state == QLatin1String("paused");
    for (int i = 0; i < 2; i++) {
        const int slot = i == 0 ? kSlotLeft : kSlotRight;
        ControllerWidgets& w = ctrlRows_[i];
        // active: the headset service hears it (not while inside-out holds the headset: its flags are stale then), or
        // SteamVR shows it. Unpair and Test vibration only for an active controller: an absent one may not be paired.
        const bool connected = ((i == 0 ? k.leftConnected : k.rightConnected) && !paused) || ctrlSeen_[i];
        const ControllerOps::Side sd = ctrlOps_.side(slot);
        w.opText->setText(sd.text);
        w.opText->setVisible(!sd.text.isEmpty());
        w.opText->setStyleSheet(sd.error ? QStringLiteral("color: %1;").arg(pibridge::ui::color::red.name()) : QString());
        // a connected controller shows Unpair, an absent one Pair (keepalive keeps no list of bonds)
        w.pair->setVisible(!connected);
        w.unpair->setVisible(connected);
        // Pair / Unpair need the headset free (service up, no inside-out session holding it); vibration works always
        const bool idle = !ctrlOps_.busy();
        const bool canAct = idle && k.running && !paused && k.headsetPresent;
        const QString why = !k.running ? QStringLiteral("The headset service is off.")
                            : paused   ? QStringLiteral("Inside-out tracking holds the headset: stop VR first.")
                            : !k.headsetPresent ? QStringLiteral("No headset.")
                                                : QString();
        w.pair->setEnabled(canAct);
        w.unpair->setEnabled(canAct);
        w.pair->setToolTip(why.isEmpty() ? QStringLiteral("Put the controller in pairing mode, then press Pair and wait "
                                                          "up to 30 s")
                                         : why);
        w.unpair->setToolTip(why.isEmpty() ? QStringLiteral("Forget this controller in the headset") : why);
        w.buzz->setVisible(connected);
        w.buzz->setEnabled(idle);
    }
}

} // namespace pibridge::app

namespace pibridge::app {

// The default input now (pactl get-default-source, asynchronously): "headset" for the Pimax microphone.
void DashboardPage::updateInputNow() {
    if (source_.fake()) {
        inputNow_->setText(QStringLiteral("headset"));
        return;
    }
    auto* p = new QProcess(this);
    connect(p, &QProcess::finished, this, [this, p](int code, QProcess::ExitStatus) {
        const QString name = QString::fromUtf8(p->readAllStandardOutput()).trimmed();
        p->deleteLater();
        const bool headset = name.startsWith(QLatin1String("alsa_input.usb-Pimax_Corporation_")) ||
                             name.startsWith(QLatin1String("alsa_input.usb-PIMAX_"));
        inputNow_->setText(code != 0 || name.isEmpty() ? QStringLiteral("unknown")
                           : headset                   ? QStringLiteral("headset")
                                                       : QStringLiteral("another device"));
        inputNow_->setToolTip(name);
    });
    p->start(QStringLiteral("pactl"), {QStringLiteral("get-default-source")});
}

} // namespace pibridge::app

namespace pibridge::app {

void DashboardPage::showPairDialog(int slot) {
    auto* dlg = new QDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("Pair the %1 controller").arg(sideName(slot)));
    dlg->setStyleSheet(QStringLiteral("QDialog { background: %1; }").arg(pibridge::ui::color::bg.name()));
    dlg->resize(520, 330);
    auto* v = new QVBoxLayout(dlg);
    v->setContentsMargins(22, 18, 22, 18);
    v->setSpacing(12);
    auto* title = new QLabel(QStringLiteral("Pair the %1 controller").arg(sideName(slot)).toUpper());
    title->setFont(pibridge::ui::titleFont(20));
    v->addWidget(title);
    auto* steps = new QLabel(QStringLiteral(
        "<ol style='margin-left:-20px'>"
        "<li>Put the controller in pairing mode: press and hold its pairing buttons until its light blinks "
        "(see the Pimax manual).</li>"
        "<li>Press <b>START PAIRING</b> and keep the controller near the headset.</li>"
        "<li>The headset listens for up to %1 s; the other controller may drop out for a moment meanwhile.</li>"
        "</ol>")
                                  .arg(kPairTimeoutS));
    steps->setWordWrap(true);
    steps->setTextFormat(Qt::RichText);
    steps->setFont(pibridge::ui::bodyFont(13));
    v->addWidget(steps);
    auto* bar = new QProgressBar;
    bar->setRange(0, static_cast<int>(kPairTimeoutS));
    bar->setTextVisible(false);
    bar->setFixedHeight(6);
    bar->setStyleSheet(QStringLiteral("QProgressBar { background: %1; border: none; border-radius: 3px; } "
                                      "QProgressBar::chunk { background: %2; border-radius: 3px; }")
                           .arg(pibridge::ui::color::border.name(), pibridge::ui::color::accent.name()));
    v->addWidget(bar);
    auto* state = pibridge::ui::dimLabel(QStringLiteral("Not started."));
    state->setWordWrap(true);
    v->addWidget(state);
    v->addStretch(1);
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    auto* close = pibridge::ui::toolButton(QStringLiteral("CLOSE"));
    auto* start = pibridge::ui::accentButton(QStringLiteral("START PAIRING"));
    start->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Link, Qt::white));
    buttons->addWidget(close);
    buttons->addWidget(start);
    v->addLayout(buttons);

    auto* timer = new QTimer(dlg);
    timer->setInterval(250);
    auto startedAt = std::make_shared<qint64>(0);
    const auto refresh = [this, slot, bar, state, start, timer, startedAt] {
        const ControllerOps::Side sd = ctrlOps_.side(slot);
        if (!sd.text.isEmpty()) {
            state->setText(sd.text);
            state->setStyleSheet(sd.error ? QStringLiteral("color: %1;").arg(pibridge::ui::color::red.name())
                                          : QString());
        }
        if (sd.busy) {
            bar->setValue(static_cast<int>(
                std::min<qint64>(kPairTimeoutS, (QDateTime::currentMSecsSinceEpoch() - *startedAt) / 1000)));
        } else if (*startedAt != 0) {
            timer->stop();
            bar->setValue(sd.error ? 0 : static_cast<int>(kPairTimeoutS));
            start->setText(QStringLiteral("TRY AGAIN"));
            start->setEnabled(true);
        }
    };
    connect(timer, &QTimer::timeout, dlg, refresh);
    connect(close, &QPushButton::clicked, dlg, &QDialog::close);
    connect(start, &QPushButton::clicked, dlg, [this, slot, start, bar, timer, startedAt, refresh] {
        *startedAt = QDateTime::currentMSecsSinceEpoch();
        bar->setValue(0);
        controllerAction(slot, 0);
        if (ctrlOps_.side(slot).busy) {
            start->setEnabled(false);
            timer->start();
        } else {
            *startedAt = 0; // refused: the reason is shown
        }
        refresh();
    });
    dlg->show();
}

} // namespace pibridge::app
