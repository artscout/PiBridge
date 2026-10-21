// SPDX-License-Identifier: MIT
#include "dashboard.h"
#include "record_card.h"
#include "sound_tests.h"

#include <QGuiApplication>

#include <QClipboard>
#include <QDateTime>
#include <QDialog>

#include <QDir>
#include <QFileDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QHash>
#include <QLayoutItem>
#include <QStandardPaths>
#include <QStyle>
#include <QVBoxLayout>

#include "../core/apps_runtime.h"
#include "../core/chovr.h"
#include "../core/inside_out.h"
#include "../core/monado_build.h"
#include "../core/overlay_control.h"
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

DashboardPage::DashboardPage(StateSource& source, Actions& actions, Conf& conf, IoSessionControl& io,
                             IoSessionControl& direct, OverlayControl& overlay, QWidget* parent)
    : QWidget(parent), source_(source), actions_(actions), conf_(conf), io_(io), direct_(direct), overlay_(overlay) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);

    // ---- toolbar
    auto* bar = new QHBoxLayout;
    bar->setSpacing(8);
    auto* runtime = runtime_ = new QLabel(QStringLiteral("Runtime: SteamVR"));
    runtime->setObjectName(QStringLiteral("dim"));
    runtime->setStyleSheet(QStringLiteral("QLabel { border: 1px solid #3a3a43; border-radius: 2px; padding: 5px 10px; }"));
    runtime->setToolTip(QStringLiteral("The OpenXR / OpenVR runtime games use. Only SteamVR for now."));
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
        "Finds what is left of SteamVR (also a stuck vrserver that holds SteamVR's socket) and closes exactly those "
        "processes after you confirm. When no SteamVR process runs, it offers to restart Steam."));
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
            say(closer_->report(), !closer_->allGone());
            closer_.reset();
            reset_->setEnabled(true);
            source_.refresh();
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
    hsHint_->setAlignment(Qt::AlignRight);
    hsError_ = new QLabel;
    hsError_->setObjectName(QStringLiteral("error"));
    hsError_->setWordWrap(true);
    hsError_->setAlignment(Qt::AlignRight);
    auto* hsRow = new QHBoxLayout;
    art_ = new pibridge::ui::HeadsetArt;
    art_->setMaximumHeight(104); // compact, closer to omarchyvr's density (user 2026-10-20)
    hsRow->addWidget(art_, 1, Qt::AlignTop); // raised to line up with the STATE row on the right
    auto* right = new QVBoxLayout;
    right->setSpacing(10);
    right->addWidget(hsHint_);
    right->addWidget(hsError_);
    auto* info = new QGridLayout;
    info->setHorizontalSpacing(24);
    info->setVerticalSpacing(14);
    hsState_ = infoRow(info, 0, QStringLiteral("State"));
    hsSerial_ = infoRow(info, 1, QStringLiteral("Serial"));
    hsSerial_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    auto* rl = pibridge::ui::capsLabel(QStringLiteral("Refresh rate"), 13);
    rl->setObjectName(QStringLiteral("dim"));
    const QString refreshTip = QStringLiteral("The panel rate the headset service sets (refresh_hz in pibridge.conf). "
                                              "Takes effect the next time the headset wakes.");
    refresh_ = new pibridge::ui::Segmented({QStringLiteral("72 Hz"), QStringLiteral("90 Hz")});
    refresh_->setToolTip(refreshTip);
    auto* rlRow = new QHBoxLayout;
    rlRow->setSpacing(6);
    rlRow->setContentsMargins(0, 0, 0, 0);
    rlRow->addWidget(rl);
    rlRow->addWidget(pibridge::ui::helpBadge(refreshTip));
    rlRow->addStretch(1);
    info->addLayout(rlRow, 2, 0);
    info->addWidget(refresh_, 2, 1, Qt::AlignLeft);
    info->setColumnStretch(1, 1);
    right->addLayout(info);
    right->addStretch(1);
    hsRow->addLayout(right, 1);
    hs->body()->addLayout(hsRow); // content height, so the buttons sit right under the info (no middle gap)
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
    // QUICK ROOM SETUP: the quick circle setup straight away (posture -> size -> finish), skipping the in-VR mode menu
    quickRoomBtn_ = pibridge::ui::toolButton(QStringLiteral("QUICK ROOM SETUP"));
    quickRoomBtn_->setMinimumHeight(40);
    quickRoomBtn_->setMinimumWidth(150);
    quickRoomBtn_->setFont(pibridge::ui::labelFont(14, QFont::DemiBold));
    quickRoomBtn_->setCursor(Qt::PointingHandCursor);
    quickRoomBtn_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Room, pibridge::ui::color::text));
    quickRoomBtn_->setIconSize({18, 18});
    ioRow->addWidget(ioStart_);
    ioRow->addWidget(roomBtn_);
    ioRow->addWidget(quickRoomBtn_);
    ioRow->addStretch(1);
    hs->body()->addLayout(ioRow);
    vrStatus_ = pibridge::ui::dimLabel(QString()); // "VR running 1h 5m   Resolution: 5376 x 4972"
    hs->body()->addWidget(vrStatus_);
    connect(ioStart_, &QPushButton::clicked, this, [this] {
        conf_.reload();
        const bool inside = ioSettings(conf_).mode == TrackingMode::InsideOut;
        const bool running = inside ? io_.active() : source_.state().steamvrRunning;
        if (running) {
            // the same button is red STOP VR while running; no confirmation (user 2026-10-20)
            if (inside) {
                io_.stop();
            } else if (!source_.fake() && !closer_ && !restarter_) {
                closeSteamVr(scanSteamVr().targets()); // Lighthouse: close SteamVR straight away
            }
            return;
        }
        if (inside) {
            io_.refresh();
            if (!io_.start()) {
                say(QStringLiteral("VR could not start: %1")
                        .arg(io_.readiness().blocking.isEmpty() ? io_.message() : io_.readiness().blocking.join(QLatin1Char(' '))),
                    true);
                return;
            }
            say(QStringLiteral("Starting VR: SteamVR with the Monado driver..."));
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
    });
    connect(roomBtn_, &QPushButton::clicked, this, [this] {
        say(QStringLiteral("Starting room setup in the headset..."));
        emit startRoomSetup();
    });
    connect(quickRoomBtn_, &QPushButton::clicked, this, [this] {
        say(QStringLiteral("Starting quick room setup in the headset..."));
        emit startQuickRoomSetup();
    });
    grid->addWidget(hs, 0, 0, 1, 2);
    connect(refresh_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        const int hz = i == 0 ? 72 : 90;
        if (!actions_.setRefreshHz(hz, {})) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            applyConf();
            return;
        }
        say(QStringLiteral("Refresh rate set to %1 Hz.").arg(hz));
    });

    // ---- controllers card (live from SteamVR, core/steamvr.h)
    auto* ctrl = new pibridge::ui::Card(QStringLiteral("Controllers"));
    ctrlChip_ = ctrl->chip();
    const QString sides[2] = {QStringLiteral("Left controller"), QStringLiteral("Right controller")};
    for (int i = 0; i < 2; i++) {
        ControllerWidgets& w = ctrlRows_[i];
        auto* row = new QHBoxLayout;
        row->setSpacing(12);
        w.icon = new QLabel;
        row->addWidget(w.icon, 0, Qt::AlignTop);
        auto* col = new QVBoxLayout;
        col->setSpacing(3);
        auto* n = new QLabel(sides[i]);
        n->setFont(pibridge::ui::bodyFont(15));
        col->addWidget(n);
        w.model = pibridge::ui::dimLabel(QString());
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
        row->addLayout(col, 1);
        auto* rcol = new QVBoxLayout;
        rcol->setSpacing(6);
        w.chip = new pibridge::ui::Chip;
        rcol->addWidget(w.chip, 0, Qt::AlignRight);
        auto* bat = new QHBoxLayout;
        bat->setSpacing(6);
        bat->addStretch(1);
        w.battery = new pibridge::ui::Battery;
        bat->addWidget(w.battery);
        w.batteryText = new QLabel;
        w.batteryText->setFont(pibridge::ui::bodyFont(13));
        bat->addWidget(w.batteryText);
        rcol->addLayout(bat);
        rcol->addStretch(1);
        row->addLayout(rcol);
        ctrl->body()->addLayout(row);
        ctrl->body()->addWidget(pibridge::ui::rule());
    }
    ctrlExtra_ = pibridge::ui::dimLabel(QString());
    ctrl->body()->addWidget(ctrlExtra_);
    ctrlText_ = pibridge::ui::dimLabel(QString());
    ctrl->body()->addWidget(ctrlText_);
    ctrl->body()->addStretch(1);
    grid->addWidget(ctrl, 0, 2);

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
    const QString dashTip = QStringLiteral("Puts a small PiBridge tab in the SteamVR dashboard with ROOM SETUP and QUICK "
                                           "ROOM SETUP buttons, clickable with the controller laser. Saved as "
                                           "steamvr_dashboard in pibridge.conf; a leaf, so if it fails VR keeps running.");
    dashOverlay_->setToolTip(dashTip);
    quick->body()->addWidget(
        pibridge::ui::settingRow(QStringLiteral("SteamVR dashboard"), dashOverlay_, nullptr, false, dashTip));
    dashOverlayLine_ = pibridge::ui::dimLabel(QString());
    dashOverlayLine_->setFont(pibridge::ui::bodyFont(12));
    dashOverlayLine_->setWordWrap(true);
    quick->body()->addWidget(dashOverlayLine_);
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
    auto* eyeLinks = new QWidget;
    auto* el = new QHBoxLayout(eyeLinks);
    el->setContentsMargins(0, 0, 0, 0);
    el->setSpacing(4);
    el->addWidget(calibrate_);
    el->addWidget(recenter_);
    quick->body()->addWidget(pibridge::ui::settingRow(QStringLiteral("Eye tracking"), eyeToggle_, eyeLinks, false, eyeTip));
    import_ = pibridge::ui::toolButton(QStringLiteral("Import calibration from Windows..."));
    quick->body()->addWidget(import_, 0, Qt::AlignLeft);
    toolResult_ = pibridge::ui::dimLabel(QString());
    quick->body()->addWidget(toolResult_);
    quick->body()->addWidget(pibridge::ui::rule());
    auto* eyeRow = new QHBoxLayout;
    gaze_ = new pibridge::ui::GazeDot;
    eyeRow->addWidget(gaze_, 0, Qt::AlignTop);
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
    connect(recenter_, &QPushButton::clicked, this, [this] {
        setBusy(QStringLiteral("Recentering in the headset: A starts, B at the end saves, B on the grid exits..."));
        if (!actions_.recenter([this](int c, const QString& o) { toolDone(QStringLiteral("Recenter"), c, o); })) {
            setBusy({});
        }
    });
    connect(import_, &QPushButton::clicked, this, [this] {
        const QString file = QFileDialog::getOpenFileName(
            this, QStringLiteral("Calibration from Pimax Play / EVO (Windows)"),
            QStandardPaths::writableLocation(QStandardPaths::HomeLocation),
            QStringLiteral("Pimax calibration (calibration.setpm *.setpm);;All files (*)"));
        if (file.isEmpty()) {
            return;
        }
        setBusy(QStringLiteral("Importing %1...").arg(file));
        if (!actions_.importProfile(file, [this](int c, const QString& o) {
                toolDone(QStringLiteral("Import"), c,
                         c == 0 ? QStringLiteral("Saved to %1. Eye tracking uses it from now on.")
                                      .arg(actions_.eyeProfileOut())
                                : o);
            })) {
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
    trModeSel_->setToolTip(QStringLiteral("Lighthouse: SteamVR base stations (works today). Inside-out: the headset's "
                                          "own cameras (SLAM, experimental)."));
    connect(trModeSel_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        conf_.set(iokey::kMode, i == 1 ? "inside-out" : "lighthouse");
        io_.refresh();
        updateMode();
        applyState();
    });
    tracking->header()->addWidget(trModeSel_, 0, Qt::AlignVCenter);
    auto* trHead = new QHBoxLayout;
    trHead->setSpacing(12);
    auto* trIcon = new QLabel;
    trIcon->setPixmap(pibridge::ui::lineIcon(pibridge::ui::Icon::Tracking, pibridge::ui::color::accentText).pixmap(36, 36));
    trHead->addWidget(trIcon);
    trSummary_ = new QLabel;
    trSummary_->setFont(pibridge::ui::bodyFont(15));
    trHead->addWidget(trSummary_, 1);
    tracking->body()->addLayout(trHead);
    trStations_ = new QWidget;
    auto* stl = new QVBoxLayout(trStations_);
    stl->setContentsMargins(0, 0, 0, 0);
    stl->setSpacing(6);
    tracking->body()->addWidget(trStations_);
    trText_ = pibridge::ui::dimLabel(QString());
    tracking->body()->addWidget(trText_);
    tracking->body()->addStretch(1);
    // the mode (Tracking page) and, in inside-out mode, START TRACKING (core/io_session.h)
    tracking->body()->addWidget(pibridge::ui::rule());
    auto* modeRow = new QHBoxLayout;
    trMode_ = pibridge::ui::dimLabel(QString());
    modeRow->addWidget(trMode_, 1);
    auto* modeLink = pibridge::ui::linkButton(QStringLiteral("Settings"));
    modeLink->setToolTip(QStringLiteral("Inside-out: the SLAM / tracking settings (Tracking page). Lighthouse: the "
                                        "headset driver's settings (CustomHeadsetOpenVR)."));
    connect(modeLink, &QPushButton::clicked, this, &DashboardPage::openTrackingSettings);
    modeRow->addWidget(modeLink, 0, Qt::AlignTop);
    tracking->body()->addLayout(modeRow);
    // START VR / STOP now live in the headset box above (moved there 2026-10-20, like omarchyvr).
    connect(&io_, &IoSessionControl::changed, this, &DashboardPage::updateMode);
    connect(&direct_, &IoSessionControl::changed, this, &DashboardPage::updateMode);
    grid->addWidget(tracking, 1, 1);

    // ---- sound
    auto* sound = new pibridge::ui::Card(QStringLiteral("Sound settings"));
    audio_ = new pibridge::ui::Toggle;
    const QString audioTip = QStringLiteral("Makes the headset the default audio output while it is on "
                                            "(audio_to_headset in pibridge.conf).");
    audio_->setToolTip(audioTip);
    sound->body()->addWidget(
        pibridge::ui::settingRow(QStringLiteral("Default audio output"), audio_, nullptr, false, audioTip));
    sound->body()->addWidget(
        pibridge::ui::dimLabel(QStringLiteral("Make the headset the default audio output while it is on.")));
    sound->body()->addWidget(pibridge::ui::rule());
    soundPanel_ = new SoundTestPanel(source_.fake()); // pages/sound_tests.h
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
        closeSteamVr(scan.targets()); // exactly the PIDs the dialog listed
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

void DashboardPage::closeSteamVr(std::vector<SteamVrProc> targets) {
    reset_->setEnabled(false);
    say(QStringLiteral("Closing SteamVR..."));
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
    const bool need = !fixCapsPath_.isEmpty() && QFileInfo::exists(fixCapsPath_) && !fileCaps(fixCapsPath_).ok();
    fixCaps_->setVisible(need);
}

void DashboardPage::updateVrStatus() {
    const bool inside = ioSettings(conf_).mode == TrackingMode::InsideOut;
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
        if (!source_.fake() && !steamClientRunning()) {
            line += QStringLiteral("      Start Steam first to launch SteamVR");
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
    const IoSettings io = ioSettings(conf_);
    const bool inside = io.mode == TrackingMode::InsideOut;
    if (trModeSel_ != nullptr) {
        trModeSel_->setCurrent(inside ? 1 : 0); // reflect the saved mode (programmatic: does not emit activated)
    }
    const bool active = io_.active();
    if (direct_.active()) {
        trMode_->setText(QStringLiteral("\"Monado directly\" runs (Development page). %1").arg(direct_.message()));
    } else if (!inside) {
        trMode_->setText(QStringLiteral("Mode: Lighthouse (SteamVR and the base stations)."));
    } else if (active || io_.state() != RecState::Idle) {
        trMode_->setText(QStringLiteral("Mode: Inside-out (experimental), SteamVR + Monado. %1").arg(io_.message()));
    } else {
        trMode_->setText(QStringLiteral("Mode: Inside-out (experimental)."));
    }
    const RecReadiness& r = io_.readiness();
    const bool steamvr = source_.state().steamvrRunning;
    const bool running = inside ? active : steamvr; // the same button toggles: blue START VR / red STOP VR
    ioStart_->setVisible(true);
    if (running) {
        ioStart_->setText(QStringLiteral("STOP VR"));
        ioStart_->setStyleSheet(QStringLiteral("QPushButton { background: #c0392b; color: #ffffff; border: none; "
                                               "border-radius: 4px; } QPushButton:hover { background: #d64533; }"));
        ioStart_->setEnabled(inside ? io_.state() != RecState::Stopping : true);
        ioStart_->setToolTip(inside ? QStringLiteral("Stops the inside-out tracking session.")
                                    : QStringLiteral("Closes SteamVR (the Reset flow)."));
    } else {
        ioStart_->setText(QStringLiteral("START VR"));
        ioStart_->setStyleSheet(QString()); // back to the accent (blue) style
        ioStart_->setEnabled(inside ? r.ready() : true);
        ioStart_->setToolTip(inside ? (r.ready() ? QStringLiteral("Starts the camera broker, the controller tracker and "
                                                                  "SteamVR with the Monado driver and the Tracking "
                                                                  "page's settings.")
                                                 : r.blocking.join(QLatin1Char('\n')))
                                    : QStringLiteral("Launches SteamVR (the base stations track the headset)."));
    }

    // the Controllers card reads Monado (libmonado) in "Monado directly" mode, SteamVR otherwise
    const bool monado = direct_.active();
    runtime_->setText(monado ? QStringLiteral("Runtime: Monado") : QStringLiteral("Runtime: SteamVR"));
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
        const QString svc = direct_.setup().monado.path;
        if (svc != mndService_) {
            mndService_ = svc; // looked up again only when the program changes (this runs at every session tick)
            mndLibrary_ = source_.fake() ? QStringLiteral("/fake/monado/libmonado.so") : libmonadoFor(svc);
        }
        MonadoTarget t;
        t.wanted = monado && direct_.state() == RecState::Recording;
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
    io_.refresh();
    updateMode();
}

void DashboardPage::updateOverlay() {
    if (dashOverlay_ == nullptr) {
        return;
    }
    const OverlayStatus s = overlay_.status(source_.state().steamvrRunning);
    dashOverlay_->setCheckedQuiet(overlay_.wanted());
    dashOverlayLine_->setText(s.words);
    dashOverlayLine_->setVisible(!s.words.isEmpty());
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
    const bool present = (k.running && k.headsetPresent) || hmd != nullptr || direct_.active();
    const QString model = (k.running && !k.model.isEmpty()) ? k.model
                          : (hmd != nullptr && !hmd->model.isEmpty()) ? hmd->model
                                                                      : QStringLiteral("Headset");
    hsTitle_->setText(present ? model : QStringLiteral("No headset"));
    // The tracking mode we can tell (from the settings); the faceplate we cannot, so we never claim one.
    const bool insideOut = ioSettings(conf_).mode == TrackingMode::InsideOut;
    hsSub_->setText(QStringLiteral("%1 · SteamVR")
                        .arg(insideOut ? QStringLiteral("Inside-out") : QStringLiteral("Lighthouse")));
    // The chip: during a live session show the panel state, not keepalived's internal "paused".
    hsChip_->set(hmd != nullptr ? ChipText{QStringLiteral("SCREEN ON"), ChipKind::Info} : headsetChip(k));
    hsHint_->setText(headsetHint(k, s.steamvrRunning));
    // "Monado directly" owns the headset: keepalived follows vrserver and only waits, so say what Monado does
    const MonadoHeadsetWords mw = direct_.active() ? monadoHeadsetWords(direct_.state(), direct_.message()) : MonadoHeadsetWords{};
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
    const QString hsSerial = (hmd != nullptr && !hmd->serial.isEmpty()) ? hmd->serial : QString();
    hsSerial_->setText(hsSerial.isEmpty() ? QStringLiteral("—") : hsSerial);
    // Room setup and the sound tests need a live SteamVR session; grey them out until it runs.
    roomBtn_->setEnabled(s.steamvrRunning);
    if (quickRoomBtn_ != nullptr) {
        quickRoomBtn_->setEnabled(s.steamvrRunning);
    }
    if (soundPanel_ != nullptr) {
        soundPanel_->setVrRunning(s.steamvrRunning);
    }
    updateOverlay(); // SteamVR up/down changes the launcher's status words
    const ControllersView cv = monadoMode_ ? monadoControllersView(mndStatus_) : controllersView(s.steamvr, s.steamvrRunning);
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
        w.icon->setPixmap(pibridge::ui::lineIcon(pibridge::ui::Icon::Controller, ic).pixmap(40, 40));
        w.model->setText(r.present ? r.name : QStringLiteral("Not connected"));
        w.model->setVisible(connected);
        w.detail->setText(connected ? r.detail : QString());
        w.detail->setVisible(!w.detail->text().isEmpty());
        w.chip->set(connected ? r.chip : ChipText{});
        w.battery->set(connected ? r.batteryPercent : -1, r.charging);
        w.battery->setVisible(connected && r.batteryPercent >= 0);
        w.batteryText->setText(connected ? r.batteryText : QString());
        w.batteryText->setVisible(!w.batteryText->text().isEmpty());
        w.serialText = connected ? r.serial : QString();
        w.serial->setText(w.serialText.isEmpty() ? QString() : QStringLiteral("SN ") + w.serialText);
        w.serial->setVisible(!w.serialText.isEmpty());
        w.copy->setVisible(!w.serialText.isEmpty());
    }
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
    trStations_->setVisible(!tv.stations.empty());
    eyeChip_->set(eyeChip(s.eyed));
    eyeLine_->setText(eyeLine(s.eyed) +
                      (s.eyed.running && s.eyed.profile == QLatin1String("user")
                           ? QStringLiteral("\nCalibration: your profile")
                           : s.eyed.running && s.eyed.profile == QLatin1String("none")
                                 ? QStringLiteral("\nNo calibration profile yet: import it from Windows.")
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
    gaze_->setGaze(t);
}

void DashboardPage::setBusy(const QString& what) {
    busy_ = what;
    if (!what.isEmpty()) {
        toolResult_->setText(what);
    }
    updateToolButtons();
}

void DashboardPage::toolDone(const QString& what, int code, const QString& out) {
    busy_.clear();
    toolResult_->setText(code == 0 ? QStringLiteral("%1 done. %2").arg(what, tail(out))
                                   : QStringLiteral("%1 failed (%2): %3").arg(what).arg(code).arg(tail(out)));
    updateToolButtons();
}

void DashboardPage::updateToolButtons() {
    const bool eyecal = !actions_.eyecalTool().isEmpty();
    const bool eyeTool = !actions_.eyeTool().isEmpty();
    const bool idle = busy_.isEmpty();
    calibrate_->setEnabled(eyecal && idle);
    recenter_->setEnabled(eyecal && idle);
    import_->setEnabled(eyeTool && idle);
    const QString missingCal = QStringLiteral("pibridge-eyecal was not found (PATH or this build).");
    calibrate_->setToolTip(eyecal ? QStringLiteral("Calibrates the eye tracker itself, as Pimax Play does. The headset shows a grid "
                                                   "with your gaze; A starts, follow the green dot, B saves and "
                                                   "shows the grid with the new calibration, B there exits.")
                                  : missingCal);
    recenter_->setToolTip(eyecal ? QStringLiteral("A quick re-centre: 5 dots, removes a constant offset. Starts and ends on the "
                                                  "gaze grid.")
                                 : missingCal);
    import_->setToolTip(eyeTool ? QStringLiteral("Reads calibration.setpm from Pimax Play / EVO under Windows and "
                                                 "saves your eye profile (pibridge-eye profile).")
                                : QStringLiteral("pibridge-eye was not found (PATH, this build, or the omarchyvr build)."));
}

} // namespace pibridge::app
