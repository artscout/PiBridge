// SPDX-License-Identifier: MIT
#include "tracking.h"

#include <QComboBox>
#include <QDesktopServices>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStyle>
#include <QUrl>
#include <QVBoxLayout>

#include "../core/paths.h"
#include "../core/roomsetup.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

// One setting: LABEL [RECOMMENDED] .......... control, the help line under it, and the reason it does not apply.
class OptionRow : public QWidget {
public:
    OptionRow(const QString& label, const QString& help, QWidget* control, QWidget* parent = nullptr) : QWidget(parent) {
        auto* v = new QVBoxLayout(this);
        v->setContentsMargins(0, 4, 0, 4);
        v->setSpacing(3);
        auto* h = new QHBoxLayout;
        h->setSpacing(8);
        label_ = pibridge::ui::capsLabel(label, 13);
        h->addWidget(label_);
        rec_ = new QLabel(QStringLiteral("RECOMMENDED"));
        rec_->setFont(pibridge::ui::labelFont(10));
        rec_->setObjectName(QStringLiteral("accentText"));
        h->addWidget(rec_);
        h->addStretch(1);
        if (control) {
            h->addWidget(control);
        }
        control_ = control;
        v->addLayout(h);
        help_ = pibridge::ui::dimLabel(help);
        help_->setFont(pibridge::ui::bodyFont(12));
        v->addWidget(help_);
        reason_ = new QLabel;
        reason_->setWordWrap(true);
        reason_->setFont(pibridge::ui::bodyFont(12));
        reason_->setStyleSheet(QStringLiteral("color: #f2b02e;"));
        reason_->hide();
        v->addWidget(reason_);
    }
    void setRecommended(bool on) { rec_->setVisible(on); }
    // "" = applies; else the control is disabled and the reason shown
    void setNotApplicable(const QString& why) {
        if (control_) {
            control_->setEnabled(why.isEmpty());
        }
        label_->setEnabled(why.isEmpty());
        reason_->setText(why);
        reason_->setVisible(!why.isEmpty());
    }

private:
    QLabel* label_;
    QLabel* rec_;
    QLabel* help_;
    QLabel* reason_;
    QWidget* control_;
};

namespace {

QString esc(const QString& s) {
    return s.toHtmlEscaped();
}

QString docPath(const QString& name) {
    for (const QString& d : {QStringLiteral(PIBRIDGE_INSTALLED_DOCS), QStringLiteral(PIBRIDGE_SOURCE_DOCS)}) {
        const QString p = d + QLatin1Char('/') + name;
        if (QFileInfo::exists(p)) {
            return p;
        }
    }
    return {};
}

QString onOff(bool on) {
    return on ? QStringLiteral("on") : QStringLiteral("off");
}

QSpinBox* spin(int lo, int hi, const QString& suffix) {
    auto* s = new QSpinBox;
    s->setRange(lo, hi);
    s->setSuffix(suffix);
    s->setMinimumWidth(96);
    return s;
}

} // namespace

TrackingPage::TrackingPage(StateSource& source, Conf& conf, IoSessionControl& control, QWidget* parent)
    : QWidget(parent), source_(source), conf_(conf), control_(control) {
    setObjectName(QStringLiteral("page"));
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(18, 12, 18, 12);
    outer->setSpacing(8);
    auto* headRow = new QHBoxLayout;
    auto* heading = new QLabel(QStringLiteral("Tracking"));
    heading->setFont(pibridge::ui::titleFont(24));
    headRow->addWidget(heading);
    auto* exp = new pibridge::ui::Chip;
    exp->set(QStringLiteral("EXPERIMENTAL"), ChipKind::Warn);
    headRow->addWidget(exp);
    headRow->addSpacing(12);
    message_ = new QLabel;
    message_->setObjectName(QStringLiteral("dim"));
    headRow->addWidget(message_, 1);
    auto* help = pibridge::ui::linkButton(QStringLiteral("How tracking works"));
    help->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Help, pibridge::ui::color::accentText));
    help->setToolTip(QStringLiteral("Opens docs/TRACKING.md: every setting, where it comes from, what is not verified."));
    connect(help, &QPushButton::clicked, this, [this] {
        const QString p = docPath(QStringLiteral("TRACKING.md"));
        if (source_.fake()) {
            say(QStringLiteral("Fake mode: nothing is opened (%1).").arg(p));
        } else if (!p.isEmpty()) {
            QDesktopServices::openUrl(QUrl::fromLocalFile(p));
        }
    });
    headRow->addWidget(help);
    outer->addLayout(headRow);

    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* holder = new QWidget;
    holder->setObjectName(QStringLiteral("page"));
    auto* cols = new QHBoxLayout(holder);
    cols->setContentsMargins(0, 0, 8, 0);
    cols->setSpacing(10);
    auto* left = new QVBoxLayout;
    left->setSpacing(10);
    auto* right = new QVBoxLayout;
    right->setSpacing(10);
    cols->addLayout(left, 1);
    cols->addLayout(right, 1);
    scroll->setWidget(holder);
    outer->addWidget(scroll, 1);

    const auto addRow = [this](pibridge::ui::Card* card, const QString& label, const QString& helpText, QWidget* ctl,
                               std::function<bool(const IoSettings&)> rec,
                               std::function<QString(const IoSettings&, const RecSetup&)> na = {}) {
        auto* r = new OptionRow(label, helpText, ctl);
        card->body()->addWidget(r);
        rows_.push_back({r, std::move(rec), std::move(na)});
        return r;
    };
    const auto toggle = [this](std::string_view key) {
        auto* t = new pibridge::ui::Toggle;
        const std::string k(key);
        connect(t, &QAbstractButton::toggled, this, [this, k](bool on) { save(k, onOff(on)); });
        return t;
    };
    const auto seg = [this](const QStringList& items, std::string_view key, const QStringList& values) {
        auto* s = new pibridge::ui::Segmented(items);
        const std::string k(key);
        connect(s, &pibridge::ui::Segmented::activated, this, [this, k, values](int i) {
            if (i >= 0 && i < values.size()) {
                save(k, values[i]);
            }
        });
        return s;
    };
    // what the chosen engine still needs ("" = it can run)
    const auto engineMissing = [](const IoSettings& io, const RecSetup& s) -> QString {
        if (io.slamEngine == SlamEngine::Pimax) {
            QString why;
            return pimaxSlamReady(s, &why) ? QString() : QStringLiteral("Pimax engine: ") + why + QLatin1Char('.');
        }
        return s.basalt.path.isEmpty() ? QStringLiteral("Needs the Basalt library (libbasalt.so); set it under Advanced.")
                                       : QString();
    };
    const auto noSlam = [engineMissing](const IoSettings& io, const RecSetup& s) -> QString {
        if (QString w = engineMissing(io, s); !w.isEmpty()) {
            return w;
        }
        return io.slam ? QString() : QStringLiteral("Head tracking (SLAM) is off.");
    };
    // the settings Basalt reads; Pimax's engine has its own calibration and settings
    const auto basaltOnly = [noSlam](const IoSettings& io, const RecSetup& s) -> QString {
        if (io.slamEngine == SlamEngine::Pimax) {
            return QStringLiteral("For Basalt only: Pimax's engine uses its own calibration and settings.");
        }
        return noSlam(io, s);
    };

    // ---- mode
    modeCard_ = new pibridge::ui::Card(QStringLiteral("Tracking mode"));
    mode_ = new pibridge::ui::Segmented({QStringLiteral("Lighthouse"), QStringLiteral("Inside-out")});
    connect(mode_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        save(iokey::kMode, i == 1 ? QStringLiteral("inside-out") : QStringLiteral("lighthouse"));
    });
    modeCard_->body()->addWidget(pibridge::ui::settingRow(QStringLiteral("Head tracking by"), mode_));
    modeText_ = new QLabel;
    modeText_->setWordWrap(true);
    modeText_->setTextFormat(Qt::RichText);
    modeCard_->body()->addWidget(modeText_);
    left->addWidget(modeCard_);

    // ---- session
    sessionCard_ = new pibridge::ui::Card(QStringLiteral("Session: SteamVR + Monado"));
    sessionCard_->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "Inside-out runs as a SteamVR driver: START VR (Dashboard) starts the camera broker, the controller tracker and "
        "SteamVR with the Monado driver, which feeds the head and controller poses into SteamVR using the settings "
        "below. Games start through SteamVR as usual. The display driver CustomHeadsetOpenVR keeps running (brightness, "
        "contrast, the lens and quad views all still apply); Stop, closing SteamVR or closing PiBridge returns the "
        "headset to Lighthouse.")));
    connector_ = new QComboBox;
    connector_->setMinimumWidth(170);
    connect(connector_, &QComboBox::activated, this, [this](int i) {
        save(reckey::kConnector, i == 0 ? QString() : connector_->itemData(i).toString());
    });
    addRow(sessionCard_, QStringLiteral("Display connector"),
           QStringLiteral("The DRM output monado-service takes over for the headset (XRT_COMPOSITOR_WAYLAND_CONNECTOR): "
                          "used by Record tracking and \"Monado directly\" on the Development page; with SteamVR its "
                          "own compositor picks the display. Auto: the connector whose EDID says Pimax."),
           connector_, [](const IoSettings&) { return true; });
    // Refresh rate is a display setting; it lives on the Display page and the Dashboard headset card, not here.
    drivers_ = pibridge::ui::dimLabel(QString());
    drivers_->setFont(pibridge::ui::bodyFont(12));
    drivers_->setTextFormat(Qt::RichText);
    sessionCard_->body()->addWidget(drivers_);
    auto* btnRow = new QHBoxLayout;
    start_ = pibridge::ui::accentButton(QStringLiteral("START VR"));
    start_->setMinimumHeight(46);
    start_->setMinimumWidth(240);
    start_->setFont(pibridge::ui::labelFont(15, QFont::DemiBold));
    start_->setCursor(Qt::PointingHandCursor);
    stop_ = pibridge::ui::toolButton(QStringLiteral("STOP"));
    stop_->setMinimumHeight(46);
    stop_->setMinimumWidth(110);
    btnRow->addWidget(start_);
    btnRow->addWidget(stop_);
    btnRow->addStretch(1);
    sessionCard_->body()->addLayout(btnRow);
    sessionMsg_ = new QLabel;
    sessionMsg_->setWordWrap(true);
    sessionMsg_->setFont(pibridge::ui::bodyFont(13));
    sessionCard_->body()->addWidget(sessionMsg_);
    reasons_ = new QLabel;
    reasons_->setWordWrap(true);
    reasons_->setTextFormat(Qt::RichText);
    sessionCard_->body()->addWidget(reasons_);
    left->addWidget(sessionCard_);
    connect(start_, &QPushButton::clicked, this, [this] { control_.start(); });
    connect(stop_, &QPushButton::clicked, this, [this] { control_.stop(); });
    connect(&control_, &IoSessionControl::changed, this, &TrackingPage::updateSession);

    // ---- head tracking
    auto* head = new pibridge::ui::Card(QStringLiteral("Head tracking (SLAM)"));
    slam_ = toggle(iokey::kSlam);
    addRow(head, QStringLiteral("6DoF head tracking"),
           QStringLiteral("The engine below follows the room in the two tracking cameras plus the IMU (PIMAX_SLAM). "
                          "Off: 3DoF, the head turns but does not move."),
           slam_, [](const IoSettings& io) { return io.slam; }, engineMissing);
    engine_ = seg({QStringLiteral("Basalt"), QStringLiteral("Pimax")}, iokey::kSlamEngine,
                  {QStringLiteral("basalt"), QStringLiteral("pimax")});
    addRow(head, QStringLiteral("Head tracking engine"),
           QStringLiteral("Basalt: the open-source tracker. Pimax: Pimax Play's own tracking engine with the headset's "
                          "factory calibration, run under Wine while tracking runs (its files are copied from Pimax Play "
                          "once, see docs/WINBRIDGE.md). Start and Stop start and stop it with SteamVR."),
           engine_, [](const IoSettings& io) { return io.slamEngine == SlamEngine::Basalt; },
           [engineMissing](const IoSettings& io, const RecSetup& s) { return engineMissing(io, s); });
    const auto needBase = [basaltOnly](const IoSettings& io, const RecSetup& s) -> QString {
        if (QString w = basaltOnly(io, s); !w.isEmpty()) {
            return w;
        }
        return s.slamConfig.path.isEmpty()
                   ? QStringLiteral("Needs a base Basalt config (pimax-basalt.toml, Advanced); without it Monado uses "
                                    "the driver's calibration and Basalt's built-in settings.")
                   : QString();
    };
    cams_ = seg({QStringLiteral("2 (lower pair)"), QStringLiteral("4 (all)")}, iokey::kSlamCams,
                {QStringLiteral("2"), QStringLiteral("4")});
    addRow(head, QStringLiteral("Head tracking cameras"),
           QStringLiteral("Which cameras Basalt follows the room in: the lower tracking pair, or also the upper pair "
                          "on the sides (more walls and ceiling in view). Four is being tested: the lower pair stays "
                          "the default until a recorded comparison shows four is better (PIMAX_SLAM_CAMS)."),
           cams_, [](const IoSettings& io) { return io.slamCams == 2; },
           [basaltOnly](const IoSettings& io, const RecSetup& s) -> QString {
               if (QString w = basaltOnly(io, s); !w.isEmpty()) {
                   return w;
               }
               if (!s.slamConfig.path.isEmpty() && s.slamTool.isEmpty()) {
                   return QStringLiteral("Four cameras need pibridge-slam (it writes their Basalt calibration); it was "
                                         "not found.");
               }
               return QString();
           });
    grid_ = seg({QStringLiteral("30 px"), QStringLiteral("40 px"), QStringLiteral("50 px")}, iokey::kGrid,
                {QStringLiteral("30"), QStringLiteral("40"), QStringLiteral("50")});
    addRow(head, QStringLiteral("Detection grid"),
           QStringLiteral("Basalt looks for one new corner per cell. Smaller cells find more corners in a dark or "
                          "plain view (study: 30 px clearly steadier on the desk). Written into PiBridge's Basalt config."),
           grid_, [](const IoSettings& io) { return io.grid == 30; }, needBase);
    accel_ = seg({QStringLiteral("Factory"), QStringLiteral("Off")}, iokey::kAccelBias,
                 {QStringLiteral("factory"), QStringLiteral("off")});
    addRow(head, QStringLiteral("Accelerometer bias"),
           QStringLiteral("Subtract the headset's factory accelerometer bias (aBias); without it gravity reads 3 % too "
                          "strong. The factory scale and misalignment are never applied (they make it worse)."),
           accel_, [](const IoSettings& io) { return io.accelBias == AccelBias::Factory; }, needBase);
    gyro_ = seg({QStringLiteral("Measured"), QStringLiteral("Factory"), QStringLiteral("Off")}, iokey::kGyroBias,
                {QStringLiteral("measured"), QStringLiteral("factory"), QStringLiteral("off")});
    addRow(head, QStringLiteral("Gyro bias start value"),
           QStringLiteral("What Basalt starts from: the bias the driver last measured with the headset at rest (falls "
                          "back to the factory value), the factory value, or none. Basalt refines it while it runs."),
           gyro_, [](const IoSettings& io) { return io.gyroBias == GyroBias::Measured; }, needBase);
    slamPreview_ = pibridge::ui::dimLabel(QString());
    slamPreview_->setFont(pibridge::ui::bodyFont(12));
    slamPreview_->setTextFormat(Qt::RichText);
    head->body()->addWidget(slamPreview_);
    head->body()->addWidget(pibridge::ui::rule());
    driftHold_ = toggle(iokey::kDriftHold);
    addRow(head, QStringLiteral("Drift hold (experimental)"),
           QStringLiteral("Holds the head position while the accelerometer says the head does not move "
                          "(PIMAX_SLAM_DRIFT_HOLD). On the Crystal Super it takes slow steps and leans for drift and "
                          "shifts the room by 20 to 30 cm: off is recommended for it."),
           driftHold_, [](const IoSettings& io) { return !io.driftHold; }, basaltOnly);
    stillHold_ = toggle(iokey::kStillHold);
    addRow(head, QStringLiteral("Still gate"),
           QStringLiteral("While the headset lies still the output is held, and the tracker is reset when it slides "
                          "more than 3 cm (PIMAX_SLAM_STILL)."),
           stillHold_, [](const IoSettings& io) { return io.stillHold; }, basaltOnly);
    headTilt_ = toggle(iokey::kHeadTilt);
    addRow(head, QStringLiteral("Tilt correction"),
           QStringLiteral("Pulls pitch and roll slowly towards gravity, at most 0.5 deg/s (PIMAX_HEAD_TILT)."), headTilt_,
           [](const IoSettings& io) { return io.headTilt; }, basaltOnly);
    fuseAuto_ = toggle(iokey::kFuseAuto);
    addRow(head, QStringLiteral("Automatic fusion"),
           QStringLiteral("Trusts the IMU more when the tracker is noisy or blind (PIMAX_FUSE_AUTO)."), fuseAuto_,
           [](const IoSettings& io) { return io.fuseAuto; }, basaltOnly);
    smooth_ = toggle(iokey::kSmooth);
    addRow(head, QStringLiteral("Smooth corrections"),
           QStringLiteral("Tracker corrections slide in over about 150 ms instead of jumping (PIMAX_SLAM_SMOOTH)."),
           smooth_, [](const IoSettings& io) { return io.smooth; }, basaltOnly);
    head->body()->addWidget(pibridge::ui::rule());
    anchor_ = toggle(iokey::kAnchor);
    addRow(head, QStringLiteral("Room anchor"),
           QStringLiteral("Lets Monado move its world by an offset (PIMAX_ANCHOR): needed for RECENTER IN HEADSET on "
                          "the Room page, which puts the play area centre and forward where you stand and look."),
           anchor_, [](const IoSettings& io) { return !io.anchor; });
    anchorGlide_ = toggle(iokey::kAnchorGlide);
    addRow(head, QStringLiteral("Room corrections glide"),
           QStringLiteral("After the first placement every room correction glides (5 cm/s, 2 deg/s) instead of "
                          "jumping (PIMAX_ANCHOR_GLIDE)."),
           anchorGlide_, [](const IoSettings& io) { return io.anchorGlide; },
           [](const IoSettings& io, const RecSetup&) {
               return io.anchor ? QString() : QStringLiteral("The room anchor is off.");
           });
    left->addWidget(head);
    left->addStretch(1);

    // ---- cameras
    auto* cams = new pibridge::ui::Card(QStringLiteral("Cameras"));
    camAe_ = seg({QStringLiteral("On"), QStringLiteral("Dry run"), QStringLiteral("Off")}, iokey::kCamAe,
                 {QStringLiteral("on"), QStringLiteral("dry"), QStringLiteral("off")});
    addRow(cams, QStringLiteral("Auto exposure"),
           QStringLiteral("Sets the tracking cameras' exposure and gain every 80 ms as Pimax Play does (PIMAX_CAM_AE). "
                          "Only the Crystal Super's Windows ranges are ever sent (2200..10410 us, gain 13..449 / "
                          "2200..8941 us, gain 13..439); the allowlist refuses anything else. Dry run: computes and "
                          "logs, sends nothing. Off: the cameras stay at their dark power-on setting."),
           camAe_, [](const IoSettings& io) { return io.camAe == CamAe::On; });
    camModel_ = seg({QStringLiteral("Crystal Super"), QStringLiteral("From USB")}, iokey::kCamAeModel,
                    {QStringLiteral("crystal-super"), QStringLiteral("auto")});
    addRow(cams, QStringLiteral("Exposure model"),
           QStringLiteral("Whose Windows exposure ladder and ranges to follow (PIMAX_CAM_AE_MODEL). From USB: the "
                          "headset's id (34a4:0040 = Crystal Super)."),
           camModel_, [](const IoSettings& io) { return io.camAeCrystal; },
           [](const IoSettings& io, const RecSetup&) {
               return io.camAe == CamAe::Off ? QStringLiteral("Auto exposure is off.") : QString();
           });
    camLaw_ = new pibridge::ui::Toggle;
    connect(camLaw_, &QAbstractButton::toggled, this,
            [this](bool on) { save(iokey::kCamAeLaw, on ? QStringLiteral("new") : QStringLiteral("old")); });
    addRow(cams, QStringLiteral("New exposure law"),
           QStringLiteral("Meters only the parts of the view that are not blown out, at most 2 changes a second. Off: "
                          "the previous law (PIMAX_CAM_AE_LAW=old)."),
           camLaw_, [](const IoSettings& io) { return io.camAeNewLaw; },
           [](const IoSettings& io, const RecSetup&) {
               return io.camAe == CamAe::Off ? QStringLiteral("Auto exposure is off.") : QString();
           });
    right->addWidget(cams);

    // ---- controllers
    auto* ctrl = new pibridge::ui::Card(QStringLiteral("Controllers"));
    ctrlTrack_ = toggle(iokey::kCtrlTrack);
    addRow(ctrl, QStringLiteral("Optical tracking"),
           QStringLiteral("Finds the controllers' LEDs in the cameras. With the Pimax head tracking engine: Pimax's own "
                          "LED engine in the same winbridge, as in Pimax Play (PIMAX_CTRL_ENGINE); otherwise "
                          "pimax-ctrltrack (PIMAX_CTRL_TRACK). Off: the controllers only turn, at a fixed place by the "
                          "head."),
           ctrlTrack_, [](const IoSettings& io) { return io.ctrlTrack; },
           [](const IoSettings& io, const RecSetup& s) {
               RecSetup cur = s;
               cur.io = io;
               if (pimaxCtrlEngine(cur)) {
                   return QString();
               }
               if (io.slamEngine == SlamEngine::Pimax && pimaxSlamReady(cur) && !s.winCtrlModels) {
                   return QStringLiteral("The Pimax files have no controller LED models (slam/H1); pimax-ctrltrack "
                                         "tracks the controllers instead.");
               }
               return s.ctrl.path.isEmpty() ? QStringLiteral("pimax-ctrltrack was not found; set it under Advanced.")
                                            : QString();
           });
    const auto noCtrl = [](const IoSettings& io, const RecSetup& s) -> QString {
        if (s.ctrl.path.isEmpty()) {
            return QStringLiteral("pimax-ctrltrack was not found.");
        }
        return io.ctrlTrack ? QString() : QStringLiteral("Optical tracking is off.");
    };
    ctrlAngle_ = spin(kCtrlAngleMin, kCtrlAngleMax, QStringLiteral(" deg"));
    connect(ctrlAngle_, &QSpinBox::valueChanged, this, [this](int v) { save(iokey::kCtrlAngle, QString::number(v)); });
    addRow(ctrl, QStringLiteral("Controller angle"),
           QStringLiteral("Pitches grip and pointer up by this much (PIMAX_CTRL_ANGLE, -20 to +70). Positive = the "
                          "pointer points higher."),
           ctrlAngle_, [](const IoSettings& io) { return io.ctrlAngle == 30; });
    ctrlLost_ = spin(kCtrlLostMin, kCtrlLostMax, QStringLiteral(" s"));
    connect(ctrlLost_, &QSpinBox::valueChanged, this, [this](int v) { save(iokey::kCtrlLost, QString::number(v)); });
    addRow(ctrl, QStringLiteral("Out-of-view hold"),
           QStringLiteral("How long a controller the cameras lost keeps its place next to the head (gyro + arm model) "
                          "before it goes to its rest pose (PIMAX_CTRL_TRACK_LOST_S)."),
           ctrlLost_, [](const IoSettings& io) { return io.ctrlLostS == 4; }, noCtrl);
    ctrlSteady_ = toggle(iokey::kCtrlSteady);
    addRow(ctrl, QStringLiteral("Steadier controllers"),
           QStringLiteral("Holds a camera fix for 220 ms instead of 80 ms and confirms before switching back, so they "
                          "jump less at the edge of the view (off: PIMAX_CTRL_TRACK_HYST=0)."),
           ctrlSteady_, [](const IoSettings& io) { return io.ctrlSteady; }, noCtrl);
    right->addWidget(ctrl);

    // ---- advanced
    auto* adv = new pibridge::ui::Card(QStringLiteral("Advanced"));
    extraEnv_ = new QLineEdit;
    extraEnv_->setPlaceholderText(QStringLiteral("KEY=value KEY2=value (none)"));
    extraEnv_->setMinimumWidth(260);
    connect(extraEnv_, &QLineEdit::editingFinished, this, [this] { save(reckey::kEnv, extraEnv_->text().trimmed()); });
    auto* envRow = new OptionRow(QStringLiteral("Extra environment"),
                                 QStringLiteral("Added last to the Pimax driver's settings, so it overrides them "
                                                "(rec_env, shared with Record tracking)."),
                                 extraEnv_);
    envRow->setRecommended(false);
    adv->body()->addWidget(envRow);
    adv->body()->addWidget(pibridge::ui::capsLabel(QStringLiteral("Programs and files"), 12));
    auto* pathsBox = new QWidget;
    paths_ = new QVBoxLayout(pathsBox);
    paths_->setContentsMargins(0, 0, 0, 0);
    paths_->setSpacing(4);
    adv->body()->addWidget(pathsBox);
    adv->body()->addWidget(pibridge::ui::capsLabel(QStringLiteral("What a start sets"), 12));
    envPreview_ = new QLabel;
    envPreview_->setFont(pibridge::ui::monoFont(11));
    envPreview_->setWordWrap(true);
    envPreview_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    envPreview_->setObjectName(QStringLiteral("dim"));
    adv->body()->addWidget(envPreview_);
    auto* reset = pibridge::ui::toolButton(QStringLiteral("Reset to recommended"));
    reset->setToolTip(QStringLiteral("Sets every inside-out setting above to its recommended value. The mode, how apps "
                                     "run, the refresh rate, the paths and the extra environment stay."));
    connect(reset, &QPushButton::clicked, this, [this] {
        if (!source_.fake()) {
            QMessageBox ask(QMessageBox::Question, QStringLiteral("Reset to recommended"),
                            QStringLiteral("Set every inside-out setting to its recommended value?"),
                            QMessageBox::Cancel, this);
            auto* yes = ask.addButton(QStringLiteral("Reset"), QMessageBox::AcceptRole);
            ask.exec();
            if (ask.clickedButton() != yes) {
                return;
            }
        }
        for (const auto& [k, v] : ioRecommendedLines()) {
            if (!conf_.set(k, v)) {
                say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
                break;
            }
        }
        load();
        say(QStringLiteral("Every inside-out setting is back at its recommended value (from the next start)."));
    });
    adv->body()->addWidget(reset, 0, Qt::AlignLeft);
    right->addWidget(adv);
    right->addStretch(1);

    load();
}

void TrackingPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    load();
}

void TrackingPage::say(const QString& text, bool error) {
    message_->setObjectName(error ? QStringLiteral("error") : QStringLiteral("dim"));
    message_->style()->unpolish(message_);
    message_->style()->polish(message_);
    message_->setText(text);
}

void TrackingPage::save(std::string_view key, const QString& value) {
    if (loading_) {
        return;
    }
    if (!conf_.set(key, value.toStdString())) {
        say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
    } else {
        say(control_.active() ? QStringLiteral("Saved; applies at the next start (the running session keeps its settings).")
                              : QStringLiteral("Saved (%1=%2).").arg(QString::fromUtf8(key.data(), qsizetype(key.size())), value));
    }
    load();
}

void TrackingPage::load() {
    loading_ = true;
    conf_.reload();
    io_ = ioSettings(conf_);
    control_.refresh();
    const RecSetup& s = control_.setup();

    mode_->setCurrent(io_.mode == TrackingMode::InsideOut ? 1 : 0);
    modeCard_->chip()->set(modeWord(io_.mode), io_.mode == TrackingMode::InsideOut ? ChipKind::Warn : ChipKind::Info);
    modeText_->setText(QStringLiteral(
        "<p style='color:#9a9aa5'><b style='color:#e6e6ec'>Lighthouse</b> (works today): SteamVR tracks the head and the "
        "controllers with the base stations, the same as on Windows. Needs the Lighthouse faceplate; the headset picture "
        "comes from the CustomHeadsetOpenVR display driver. Press START VR on the Dashboard.</p>"
        "<p style='color:#9a9aa5'><b style='color:#f2b02e'>Inside-out</b> (experimental, no base stations): the "
        "headset's own cameras track the head in 6DoF and the controllers by their LEDs. By default it uses the "
        "<b style='color:#e6e6ec'>Pimax SLAM engine</b> &mdash; the same tracking Pimax Play uses on Windows, run on "
        "Linux through Monado (the open-source Basalt engine is an alternative under Advanced). It needs monado-service "
        "with the Pimax driver, the camera broker (pimax-cam), the controller tracker (pimax-ctrltrack) and the "
        "headset's factory calibration. START VR runs it as a SteamVR driver, so games start through SteamVR as usual. "
        "The settings below are used by START VR and, in every mode, by Record tracking and \"Monado directly\" "
        "(Development page).</p>"));

    connector_->clear();
    const QString conn = conf_.value(reckey::kConnector, QString()).trimmed();
    const QString found = source_.fake() ? QStringLiteral("DP-6") : pimaxConnector(RecEnv::defaults().drmRoot);
    connector_->addItem(found.isEmpty() ? QStringLiteral("Auto (none found)") : QStringLiteral("Auto (found: %1)").arg(found));
    QStringList list = source_.fake() ? QStringList{QStringLiteral("DP-6"), QStringLiteral("DP-4"), QStringLiteral("HDMI-A-1")}
                                      : drmConnectors();
    if (!conn.isEmpty() && !list.contains(conn)) {
        list.prepend(conn);
    }
    for (const QString& c : list) {
        connector_->addItem(c, c);
    }
    connector_->setCurrentIndex(conn.isEmpty() ? 0 : connector_->findData(conn));

    slam_->setCheckedQuiet(io_.slam);
    engine_->setCurrent(io_.slamEngine == SlamEngine::Pimax ? 1 : 0);
    grid_->setCurrent(io_.grid == 30 ? 0 : (io_.grid == 40 ? 1 : 2));
    cams_->setCurrent(io_.slamCams == 4 ? 1 : 0);
    accel_->setCurrent(io_.accelBias == AccelBias::Factory ? 0 : 1);
    gyro_->setCurrent(io_.gyroBias == GyroBias::Measured ? 0 : (io_.gyroBias == GyroBias::Factory ? 1 : 2));
    driftHold_->setCheckedQuiet(io_.driftHold);
    stillHold_->setCheckedQuiet(io_.stillHold);
    headTilt_->setCheckedQuiet(io_.headTilt);
    fuseAuto_->setCheckedQuiet(io_.fuseAuto);
    smooth_->setCheckedQuiet(io_.smooth);
    anchor_->setCheckedQuiet(io_.anchor);
    anchorGlide_->setCheckedQuiet(io_.anchorGlide);
    camAe_->setCurrent(io_.camAe == CamAe::On ? 0 : (io_.camAe == CamAe::Dry ? 1 : 2));
    camModel_->setCurrent(io_.camAeCrystal ? 0 : 1);
    camLaw_->setCheckedQuiet(io_.camAeNewLaw);
    ctrlTrack_->setCheckedQuiet(io_.ctrlTrack);
    ctrlAngle_->setValue(io_.ctrlAngle);
    ctrlLost_->setValue(io_.ctrlLostS);
    ctrlSteady_->setCheckedQuiet(io_.ctrlSteady);
    extraEnv_->setText(conf_.value(reckey::kEnv, QString()).trimmed());

    for (const Row& r : rows_) {
        r.row->setRecommended(r.isRecommended && r.isRecommended(io_));
        r.row->setNotApplicable(r.notApplicable ? r.notApplicable(io_, s) : QString());
    }

    // what PiBridge's Basalt copy would hold (nothing is written here)
    if (io_.slam && !s.basalt.path.isEmpty()) {
        SlamGenInput in;
        in.baseToml = s.slamConfig.path;
        in.outDir = slamDir();
        in.grid = io_.grid;
        in.accelBias = io_.accelBias;
        in.gyroBias = io_.gyroBias;
        in.factoryXml = s.calib.path;
        in.gyroCache = gyroBiasCachePath();
        in.write = false;
        const SlamGenResult g = generateSlamConfig(in);
        if (g.ok) {
            const auto v3 = [](const std::array<double, 3>& a, double k) {
                return QStringLiteral("%1, %2, %3").arg(a[0] * k, 0, 'f', 3).arg(a[1] * k, 0, 'f', 3).arg(a[2] * k, 0, 'f', 3);
            };
            slamPreview_->setText(
                QStringLiteral("PiBridge's Basalt config (written at every start into %1, from %2):<br>"
                               "accel bias %3 m/s² (%4) &nbsp;·&nbsp; gyro bias %5 deg/s (%6)")
                    .arg(esc(slamDir()), esc(s.slamConfig.path), v3(g.accel, 1.0), esc(g.accelFrom),
                         v3(g.gyro, 180.0 / M_PI), esc(g.gyroFrom)) +
                (io_.slamCams == 4 ? QStringLiteral("<br>cameras: all four (the upper pair's calibration is written by "
                                                    "pibridge-slam at the start, from the factory file)")
                                   : QStringLiteral("<br>cameras: the lower pair (%1 in the base calibration)").arg(g.cams)));
        } else {
            slamPreview_->setText(QStringLiteral("<span style='color:#f2b02e'>%1</span> Monado then uses the base "
                                                 "config as it is, or the driver's calibration without one.")
                                      .arg(esc(g.error)));
        }
        slamPreview_->show();
    } else {
        slamPreview_->hide();
    }

    // the environment of a start, as the one builder makes it
    QString slamCfg = s.slamConfig.path;
    if (io_.slam && !slamCfg.isEmpty()) {
        slamCfg = slamDir() + QStringLiteral("/pimax-basalt.toml");
    }
    const QString driverFile = configDir() + QStringLiteral("/steamvr-driver.env");
    // as a start makes it: four cameras when pibridge-slam can write their calibration (prepareSessionSlam)
    RecSetup ps = s;
    ps.sessionSlamCams = io_.slamCams == 4 && !ps.slamTool.isEmpty() && !ps.calib.path.isEmpty() ? 4 : 2;
    const IoSessionEnv env = buildIoEnv(io_, ps, conf_.refreshHz(), slamCfg, QString(), driverFile, s.steamVrPreload);
    QString pv;
    if (s.steamVrPreload.isEmpty()) {
        pv = QStringLiteral("Note: pimax_rttime_preload.so was not found next to monado-service (steamvr-monado/); "
                            "without it vrserver's realtime limit kills vrcompositor about 1 s after the start.\n");
    }
    pv += QStringLiteral("START TRACKING: driver_monado reads (%1):\n%2\nSteamVR (%3) starts with:\n%4\n\n"
                         "Record tracking and \"Monado directly\" (Development): monado-service:\n%5")
              .arg(driverFile, env.driver.join(QLatin1Char('\n')),
                   s.steamVrStartup.isEmpty() ? QStringLiteral("not found") : s.steamVrStartup,
                   env.steamvr.join(QLatin1Char('\n')), env.monado.join(QLatin1Char('\n')));
    for (const QString& n : env.notes) {
        pv += QStringLiteral("\nNote: ") + n;
    }
    envPreview_->setText(pv);
    rebuildPaths();
    loading_ = false;
    updateSession();
}

void TrackingPage::rebuildPaths() {
    while (QLayoutItem* it = paths_->takeAt(0)) {
        if (it->widget()) {
            it->widget()->deleteLater();
        }
        delete it;
    }
    const RecSetup& s = control_.setup();
    for (const RecPath* p : s.paths()) {
        auto* row = new QWidget;
        auto* h = new QHBoxLayout(row);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(6);
        auto* l = new QLabel(p->label);
        l->setObjectName(QStringLiteral("dim"));
        l->setMinimumWidth(190);
        h->addWidget(l);
        auto* v = new QLabel(p->path.isEmpty() ? (p->required ? QStringLiteral("not found") : QStringLiteral("not found (optional)"))
                                               : p->path);
        v->setWordWrap(true);
        v->setStyleSheet(p->path.isEmpty() ? (p->required ? QStringLiteral("color:#ef5a55") : QStringLiteral("color:#f2b02e"))
                                           : QString());
        v->setToolTip(p->key);
        h->addWidget(v, 1);
        auto* choose = pibridge::ui::linkButton(QStringLiteral("Choose..."));
        auto* autoBtn = pibridge::ui::linkButton(QStringLiteral("Auto"));
        autoBtn->setVisible(p->fromConf);
        h->addWidget(choose);
        h->addWidget(autoBtn);
        const std::string key = p->key.toStdString();
        const QString label = p->label;
        connect(choose, &QPushButton::clicked, this, [this, key, label] {
            if (source_.fake()) {
                say(QStringLiteral("Fake mode: no file dialog."));
                return;
            }
            const QString f = QFileDialog::getOpenFileName(this, label);
            if (!f.isEmpty()) {
                save(key, f);
            }
        });
        connect(autoBtn, &QPushButton::clicked, this, [this, key] { save(key, QString()); });
        paths_->addWidget(row);
    }
}

void TrackingPage::updateSession() {
    const RecReadiness& r = control_.readiness();
    const bool active = control_.active();
    const RecState st = control_.state();
    sessionCard_->chip()->set(ioChip(st, r.ready()));
    start_->setVisible(!active);
    start_->setEnabled(r.ready());
    start_->setToolTip(r.ready() ? QStringLiteral("Starts the camera broker, the controller tracker and SteamVR with the "
                                                  "Monado driver and the settings on this page. Stop closes exactly "
                                                  "those and puts SteamVR's drivers back.")
                                 : r.blocking.join(QLatin1Char('\n')));
    stop_->setVisible(active);
    stop_->setEnabled(st != RecState::Stopping);
    QString m = control_.message();
    if (m.isEmpty() && r.ready()) {
        m = QStringLiteral("Ready.");
    }
    bool roomMapped = true;
    if (io_.mode == TrackingMode::InsideOut) {
        // the room map of this session (pibridge-roommap auto): recognised / searching / not found
        const RoomWords w = control_.room();
        roomMapped = w.look != RoomLook::NoRoom;
        m += QStringLiteral("\nRoom: ") + w.headline + (w.detail.isEmpty() ? QString() : QStringLiteral(". ") + w.detail);
    }
    if (st == RecState::Recording && roomMapped) {
        m += QStringLiteral("\n") + recenterReminder();
    }
    sessionMsg_->setText(m);
    // what a start changes in SteamVR's files (nothing is written while you look)
    QString d;
    if (io_.mode == TrackingMode::InsideOut) {
        const auto pv = control_.driverPreview();
        if (control_.driversPending()) {
            d = QStringLiteral("SteamVR's drivers are switched for inside-out; they are put back when the session ends "
                               "(or once SteamVR is closed).");
        } else if (!pv) {
            d = QStringLiteral("<span style='color:#f2b02e'>%1</span>").arg(esc(pv.error()));
        } else if (pv->isEmpty()) {
            d = QStringLiteral("SteamVR's drivers are set up for inside-out already: a start changes nothing in its files.");
        } else {
            QStringList l;
            for (const QString& w : *pv) {
                l << esc(w);
            }
            d = QStringLiteral("A start changes, for the session only (backups first, put back afterwards):<br>&nbsp;&nbsp;") +
                l.join(QStringLiteral("<br>&nbsp;&nbsp;"));
        }
    }
    drivers_->setText(d);
    drivers_->setVisible(!d.isEmpty());
    sessionMsg_->setObjectName(st == RecState::Failed ? QStringLiteral("error") : QString());
    sessionMsg_->style()->unpolish(sessionMsg_);
    sessionMsg_->style()->polish(sessionMsg_);
    QString reasons;
    if (!active) {
        for (const QString& b : r.blocking) {
            reasons += QStringLiteral("<div style='color:#ef5a55; margin:2px 0'>%1</div>").arg(esc(b));
        }
        for (const QString& w : r.warnings) {
            reasons += QStringLiteral("<div style='color:#f2b02e; margin:2px 0'>%1</div>").arg(esc(w));
        }
    }
    reasons_->setText(reasons);
    reasons_->setVisible(!reasons.isEmpty());
}

} // namespace pibridge::app
