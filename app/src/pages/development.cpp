// SPDX-License-Identifier: MIT
#include "development.h"

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStandardPaths>
#include <QStyle>
#include <QUrl>
#include <QVBoxLayout>

#include "../core/paths.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

constexpr int kGuided = 0;

QString docPath(const QString& name) {
    for (const QString& d : {QStringLiteral(PIBRIDGE_INSTALLED_DOCS), QStringLiteral(PIBRIDGE_SOURCE_DOCS)}) {
        const QString p = d + QLatin1Char('/') + name;
        if (QFileInfo::exists(p)) {
            return p;
        }
    }
    return {};
}

QString kindWords(const QString& k) {
    if (k == QLatin1String("prepare")) {
        return QStringLiteral("get ready");
    }
    if (k == QLatin1String("tilt-hold")) {
        return QStringLiteral("hold, tilted");
    }
    if (k == QLatin1String("still")) {
        return QStringLiteral("hold still");
    }
    return QStringLiteral("move");
}

QString esc(const QString& s) {
    return s.toHtmlEscaped();
}

} // namespace

DevelopmentPage::DevelopmentPage(StateSource& source, Conf& conf, IoSessionControl& direct, QWidget* parent)
    : QWidget(parent), source_(source), conf_(conf), direct_(direct), fake_(source.fake()) {
    setObjectName(QStringLiteral("page"));
    clock_.start();
    if (fake_) {
        runner_ = std::make_unique<FakeRunner>();
        announcer_ = std::make_unique<Announcer>(*runner_, [](const QString& n) {
            return n == QLatin1String("spd-say") ? QStringLiteral("/fake/spd-say") : QString();
        });
        procs_ = std::make_unique<FakeRecProcesses>();
    } else {
        runner_ = std::make_unique<ProcessRunner>();
        announcer_ = std::make_unique<Announcer>(*runner_, [](const QString& n) { return QStandardPaths::findExecutable(n); });
        procs_ = std::make_unique<QtRecProcesses>();
    }

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* headRow = new QHBoxLayout;
    auto* heading = new QLabel(QStringLiteral("Development"));
    heading->setFont(pibridge::ui::titleFont(24));
    headRow->addWidget(heading);
    headRow->addStretch(1);
    auto* help = pibridge::ui::linkButton(QStringLiteral("How recordings work"));
    help->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Help, pibridge::ui::color::accentText));
    help->setToolTip(QStringLiteral("Opens docs/DEVELOPMENT.md: what is recorded, where, and how to read steps.json."));
    connect(help, &QPushButton::clicked, this, [this] {
        const QString p = docPath(QStringLiteral("DEVELOPMENT.md"));
        if (!p.isEmpty()) {
            openPath(p);
        }
    });
    headRow->addWidget(help);
    root->addLayout(headRow);

    auto* cols = new QHBoxLayout;
    cols->setSpacing(10);
    root->addLayout(cols, 1);
    auto* left = new QVBoxLayout;
    left->setSpacing(10);
    auto* right = new QVBoxLayout;
    right->setSpacing(10);
    cols->addLayout(left, 5);
    cols->addLayout(right, 4);

    // ---- record card
    recCard_ = new pibridge::ui::Card(QStringLiteral("Record tracking"));
    recCard_->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "Records the headset's tracking cameras and IMU (a EuRoC dataset) from a Monado 6DoF session, for offline "
        "SLAM analysis. The headset shows no picture meanwhile; that is fine. Each step is shown here and read out "
        "loud, so you can follow it with the headset on.")));
    auto* modeRow = new QHBoxLayout;
    mode_ = new pibridge::ui::Segmented({QStringLiteral("Guided steps"), QStringLiteral("Free recording")});
    mode_->setCurrent(kGuided);
    mode_->setToolTip(QStringLiteral("Guided: the app runs the steps below and stops at the end. Free: just start and "
                                     "stop, no steps."));
    modeRow->addWidget(mode_);
    modeRow->addWidget(pibridge::ui::helpBadge(QStringLiteral(
        "Guided: the app runs the steps below and stops at the end. Free: just start and stop, no steps.")));
    modeRow->addStretch(1);
    recCard_->body()->addLayout(modeRow);
    auto* btnRow = new QHBoxLayout;
    record_ = pibridge::ui::accentButton(QStringLiteral("RECORD TRACKING"));
    record_->setFont(pibridge::ui::labelFont(16, QFont::DemiBold));
    record_->setMinimumHeight(52);
    record_->setMinimumWidth(260);
    record_->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Tracking, Qt::white));
    record_->setIconSize({22, 22});
    record_->setCursor(Qt::PointingHandCursor);
    stopTop_ = pibridge::ui::toolButton(QStringLiteral("STOP"));
    stopTop_->setMinimumHeight(52);
    stopTop_->setMinimumWidth(120);
    stopTop_->setFont(pibridge::ui::labelFont(15, QFont::DemiBold));
    sessionLogs_ = pibridge::ui::linkButton(QStringLiteral("Open this recording's logs"));
    btnRow->addWidget(record_);
    btnRow->addWidget(stopTop_);
    btnRow->addStretch(1);
    btnRow->addWidget(sessionLogs_, 0, Qt::AlignBottom);
    recCard_->body()->addLayout(btnRow);
    message_ = new QLabel;
    message_->setWordWrap(true);
    message_->setFont(pibridge::ui::bodyFont(14));
    recCard_->body()->addWidget(message_);
    reasons_ = new QLabel;
    reasons_->setWordWrap(true);
    reasons_->setTextFormat(Qt::RichText);
    recCard_->body()->addWidget(reasons_);
    left->addWidget(recCard_);
    connect(record_, &QPushButton::clicked, this, &DevelopmentPage::startRecording);
    const auto stopNow = [this] {
        if (session_) {
            session_->stop(now());
            tick();
        }
    };
    connect(stopTop_, &QPushButton::clicked, this, stopNow);
    connect(sessionLogs_, &QPushButton::clicked, this, [this] {
        if (session_) {
            openPath(session_->logDir());
        }
    });

    // ---- steps card
    stepsCard_ = new pibridge::ui::Card(QStringLiteral("Steps"));
    idleView_ = new QWidget;
    auto* iv = new QVBoxLayout(idleView_);
    iv->setContentsMargins(0, 0, 0, 0);
    plan_ = new QLabel;
    plan_->setTextFormat(Qt::RichText);
    plan_->setWordWrap(true);
    iv->addWidget(plan_);
    iv->addStretch(1);
    stepsCard_->body()->addWidget(idleView_, 1);
    runView_ = new QWidget;
    auto* rv = new QVBoxLayout(runView_);
    rv->setContentsMargins(0, 8, 0, 0);
    rv->setSpacing(12);
    stepInfo_ = pibridge::ui::capsLabel(QString(), 13);
    stepInfo_->setObjectName(QStringLiteral("accentText"));
    rv->addWidget(stepInfo_);
    auto* bigRow = new QHBoxLayout;
    stepText_ = new QLabel;
    stepText_->setWordWrap(true);
    QFont big = pibridge::ui::titleFont(40);
    big.setCapitalization(QFont::MixedCase);
    stepText_->setFont(big);
    stepText_->setMinimumHeight(150);
    stepText_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    countdown_ = new QLabel;
    QFont cd = pibridge::ui::titleFont(84);
    countdown_->setFont(cd);
    countdown_->setObjectName(QStringLiteral("accentText"));
    countdown_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    countdown_->setMinimumWidth(170);
    bigRow->addWidget(stepText_, 1);
    bigRow->addWidget(countdown_);
    rv->addLayout(bigRow);
    next_ = pibridge::ui::dimLabel(QString());
    next_->setFont(pibridge::ui::bodyFont(15));
    rv->addWidget(next_);
    auto* ctl = new QHBoxLayout;
    skip_ = pibridge::ui::toolButton(QStringLiteral("SKIP STEP"));
    skip_->setMinimumHeight(40);
    stop_ = pibridge::ui::toolButton(QStringLiteral("STOP"));
    stop_->setMinimumHeight(40);
    ctl->addWidget(skip_);
    ctl->addWidget(stop_);
    ctl->addStretch(1);
    rv->addLayout(ctl);
    rv->addStretch(1);
    stepsCard_->body()->addWidget(runView_, 1);
    // the word-wrapped texts ask for a lot of height at their narrowest; the card takes the space that is left
    stepsCard_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);
    stepsCard_->setMinimumHeight(380);
    left->addWidget(stepsCard_, 1);
    connect(skip_, &QPushButton::clicked, this, [this] {
        if (session_) {
            session_->skipStep(now());
            updateView();
        }
    });
    connect(stop_, &QPushButton::clicked, this, stopNow);

    // ---- the Monado build (A/B)
    buildCard_ = new pibridge::ui::Card(QStringLiteral("Monado build"));
    buildCard_->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "The Monado every start uses: Record tracking, Monado directly, and START TRACKING (its SteamVR driver, "
        "steamvr-monado/ of the same build).")));
    QStringList labels;
    for (const MonadoBuildChoice& c : monadoBuildChoices(home())) {
        labels << c.label;
    }
    build_ = new pibridge::ui::Segmented(labels);
    const QString buildTip = QStringLiteral("Saved as rec_monado_service in pibridge.conf. Auto: the usual lookup "
                                            "(PATH, the installed omarchyvr build, the live build).");
    build_->setToolTip(buildTip);
    connect(build_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        const auto choices = monadoBuildChoices(home());
        if (i < 0 || i >= int(choices.size())) {
            return;
        }
        if (!conf_.set(reckey::kMonado, choices[size_t(i)].service.toStdString())) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
            return;
        }
        say(QStringLiteral("Monado build: %1 (from the next start).").arg(choices[size_t(i)].label));
        refreshSetup();
        direct_.refresh();
        updateView();
    });
    auto* buildRow = new QHBoxLayout;
    buildRow->setSpacing(8);
    buildRow->addWidget(build_);
    buildRow->addWidget(pibridge::ui::helpBadge(buildTip));
    buildRow->addStretch(1);
    buildCard_->body()->addLayout(buildRow);
    buildPath_ = pibridge::ui::dimLabel(QString());
    buildPath_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    buildCard_->body()->addWidget(buildPath_);
    buildWhat_ = new QLabel;
    buildWhat_->setWordWrap(true);
    buildCard_->body()->addWidget(buildWhat_);
    buildDriver_ = pibridge::ui::dimLabel(QString());
    buildCard_->body()->addWidget(buildDriver_);
    auto* capRow = new QHBoxLayout;
    capLine_ = new QLabel;
    capLine_->setWordWrap(true);
    capRow->addWidget(capLine_, 1);
    fixCaps_ = pibridge::ui::toolButton(QStringLiteral("Fix permissions"));
    fixCaps_->setToolTip(QStringLiteral("Lets this monado-service use realtime priority: runs pkexec setcap "
                                        "cap_sys_nice=ep on it after asking you (your password)."));
    capRow->addWidget(fixCaps_, 0, Qt::AlignTop);
    buildCard_->body()->addLayout(capRow);
    connect(fixCaps_, &QPushButton::clicked, this, &DevelopmentPage::fixPermissions);
    right->addWidget(buildCard_);
    buildCard_->setVisible(false); // removed from the UI (user 2026-10-20): one build now; permissions fix is the
                                   // Dashboard's top-bar button

    // ---- Monado directly (debug / A-B)
    directCard_ = new pibridge::ui::Card(QStringLiteral("Monado directly (debug / A-B)"));
    directCard_->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "monado-service owns the headset instead of SteamVR, with the Tracking page's settings: for A/B tests of the "
        "builds and against SteamVR + Monado. While it runs, your own apps on the Apps page start on it "
        "(XR_RUNTIME_JSON); Steam games need SteamVR.")));
    home_ = new pibridge::ui::Toggle;
    connect(home_, &QAbstractButton::toggled, this, [this](bool on) {
        if (!conf_.set(iokey::kHome, on ? "on" : "off")) {
            say(QStringLiteral("Could not write %1").arg(conf_.path()), true);
        }
        direct_.refresh();
        updateDirect();
    });
    directCard_->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Home scene in the headset"), home_, nullptr, false,
        QStringLiteral("Shows Monado's own home scene (a room) in the headset while \"Monado directly\" runs, instead "
                       "of a black view (home in pibridge.conf).")));
    homeLine_ = pibridge::ui::dimLabel(QString());
    homeLine_->setFont(pibridge::ui::bodyFont(12));
    directCard_->body()->addWidget(homeLine_);
    auto* dRow = new QHBoxLayout;
    directStart_ = pibridge::ui::accentButton(QStringLiteral("START MONADO"));
    directStart_->setMinimumHeight(40);
    directStart_->setMinimumWidth(200);
    directStart_->setCursor(Qt::PointingHandCursor);
    directStop_ = pibridge::ui::toolButton(QStringLiteral("STOP"));
    directStop_->setMinimumHeight(40);
    dRow->addWidget(directStart_);
    dRow->addWidget(directStop_);
    dRow->addStretch(1);
    directCard_->body()->addLayout(dRow);
    directMsg_ = new QLabel;
    directMsg_->setWordWrap(true);
    directCard_->body()->addWidget(directMsg_);
    directReasons_ = new QLabel;
    directReasons_->setWordWrap(true);
    directReasons_->setTextFormat(Qt::RichText);
    directCard_->body()->addWidget(directReasons_);
    connect(directStart_, &QPushButton::clicked, this, [this] { direct_.start(); });
    connect(directStop_, &QPushButton::clicked, this, [this] { direct_.stop(); });
    connect(&direct_, &IoSessionControl::changed, this, &DevelopmentPage::updateDirect);
    right->addWidget(directCard_);

    // ---- recordings
    listCard_ = new pibridge::ui::Card(QStringLiteral("Recordings"));
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    // the list scrolls; it never makes the window taller
    scroll->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);
    scroll->setMinimumHeight(160);
    auto* holder = new QWidget;
    holder->setObjectName(QStringLiteral("card"));
    holder->setStyleSheet(QStringLiteral("QWidget#card { background: transparent; border: none; }"));
    list_ = new QVBoxLayout(holder);
    list_->setContentsMargins(0, 0, 6, 0);
    list_->setSpacing(8);
    scroll->setWidget(holder);
    listCard_->body()->addWidget(scroll, 1);
    right->addWidget(listCard_, 1);

    // ---- setup
    auto* setupCard = new pibridge::ui::Card(QStringLiteral("Setup"));
    setupCard->body()->addWidget(pibridge::ui::dimLabel(
        QStringLiteral("What a recording uses. Change any of it with the key in ~/.config/pibridge/pibridge.conf.")));
    setupView_ = new QLabel;
    setupView_->setTextFormat(Qt::RichText);
    setupView_->setWordWrap(true);
    setupView_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    setupCard->body()->addWidget(setupView_);
    right->addWidget(setupCard);

    tickTimer_.setInterval(100);
    connect(&tickTimer_, &QTimer::timeout, this, &DevelopmentPage::tick);
    refreshTimer_.setInterval(1000);
    connect(&refreshTimer_, &QTimer::timeout, this, [this] {
        if (!session_ || !session_->active()) {
            refreshSetup();
            updateView();
        }
    });
    connect(&source_, &StateSource::changed, this, [this] {
        if (fake_ && (!session_ || !session_->active())) {
            refreshSetup();
            updateView();
        }
    });

    if (fake_) {
        // made-up recordings in the fake mode's temporary data dir
        const QString dir = qEnvironmentVariable("XDG_DATA_HOME") + QStringLiteral("/pibridge/recordings");
        const qint64 base = 3600LL * 1000000000;
        writeFakeRecording(dir + QStringLiteral("/20261019-141205"), 120, false, QStringLiteral("guided"), base);
        writeFakeRecording(dir + QStringLiteral("/20261019-113040"), 45, true, QStringLiteral("guided"), 2 * base);
        writeFakeRecording(dir + QStringLiteral("/20261018-221517"), 70, false, QStringLiteral("free"), 3 * base);
    }
    refreshSetup();
    refreshRecordings();
    updateView();
}

DevelopmentPage::~DevelopmentPage() {
    if (session_) {
        session_->shutdown(); // PiBridge closes during a recording: stop exactly what it started
    }
}

void DevelopmentPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    refreshSetup();
    refreshRecordings();
    updateView();
    refreshTimer_.start();
}

void DevelopmentPage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    refreshTimer_.stop(); // the session's own tick goes on while hidden
}

std::vector<RecStep> DevelopmentPage::protocol(QString* error) const {
    if (setup_.stepsFile.isEmpty()) {
        return defaultRecSteps();
    }
    QFile f(setup_.stepsFile);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error) {
            *error = QStringLiteral("The steps file %1 (rec_steps=) cannot be read; the built-in steps are used.")
                         .arg(setup_.stepsFile);
        }
        return defaultRecSteps();
    }
    QString err;
    auto steps = parseRecSteps(f.readAll(), &err);
    if (steps.empty()) {
        if (error) {
            *error = QStringLiteral("The steps file %1 (rec_steps=) is broken (%2); the built-in steps are used.")
                         .arg(setup_.stepsFile, err);
        }
        return defaultRecSteps();
    }
    return steps;
}

void DevelopmentPage::refreshSetup() {
    RecSystem sys;
    if (fake_) {
        setup_ = fakeRecSetup(qEnvironmentVariable("XDG_DATA_HOME") + QStringLiteral("/pibridge/recordings"));
        conf_.reload();
        setup_.io = ioSettings(conf_);
        setup_.refreshHz = conf_.refreshHz();
        sys.vrserver = source_.state().steamvrRunning;
        // the build the selector names (made-up binaries: the Rust one without the PiBridge marker)
        const QString choice = monadoBuildChoiceOf(conf_.value(reckey::kMonado, QString()), home());
        for (const MonadoBuildChoice& c : monadoBuildChoices(home())) {
            if (c.id == choice && !c.service.isEmpty()) {
                setup_.monado.path = c.service;
                setup_.monado.fromConf = true;
            }
        }
        setup_.build.path = setup_.monado.path;
        if (const qsizetype at = setup_.monado.path.indexOf(QLatin1String("/src/xrt/targets/service/")); at > 0) {
            setup_.steamVrDriverDir = setup_.monado.path.left(at) + QStringLiteral("/steamvr-monado");
        }
        if (choice == QLatin1String("rust")) {
            setup_.build.kind = BuildKind::Rust;
            setup_.build.abiText.clear();
        }
    } else {
        conf_.reload();
        setup_ = resolveRecSetup(conf_, RecEnv::defaults());
        sys = readRecSystem();
        sys.vrserver = sys.vrserver || source_.state().steamvrRunning;
    }
    readiness_ = recReadiness(setup_, sys);
    updateBuild();
    if (!externalBlock_.isEmpty()) {
        readiness_.blocking.prepend(externalBlock_);
    }
    QString err;
    protocol(&err);
    if (!err.isEmpty()) {
        readiness_.warnings << err;
    }

    // the setup list
    QString h = QStringLiteral("<table cellspacing='0' cellpadding='3'>");
    const auto row = [&h](const QString& label, const QString& value, const QString& key, bool ok, bool required) {
        const QString colour = ok ? QStringLiteral("#e6e6ec") : (required ? QStringLiteral("#ef5a55") : QStringLiteral("#f2b02e"));
        h += QStringLiteral("<tr><td style='color:#9a9aa5; padding-right:14px'>%1</td><td style='color:%2'>%3</td>"
                            "<td style='color:#5c5c66; padding-left:10px'>%4</td></tr>")
                 .arg(esc(label), colour, esc(value), esc(key));
    };
    for (const RecPath* p : setup_.paths()) {
        row(p->label, p->path.isEmpty() ? (p->required ? QStringLiteral("not found") : QStringLiteral("not found (optional)"))
                                        : p->path,
            p->key, !p->path.isEmpty(), p->required);
    }
    row(QStringLiteral("display connector"),
        setup_.connector.isEmpty() ? QStringLiteral("not found")
                                   : setup_.connector + (setup_.connectorFromConf ? QString() : QStringLiteral(" (found by its EDID)")),
        QString::fromLatin1(reckey::kConnector), !setup_.connector.isEmpty(), true);
    row(QStringLiteral("tracking cameras"), setup_.camDevice.isEmpty() ? QStringLiteral("not found") : setup_.camDevice,
        QString::fromLatin1(reckey::kCamDevice), !setup_.camDevice.isEmpty(), true);
    row(QStringLiteral("recordings folder"), setup_.recordingsDir, QString::fromLatin1(reckey::kDir), true, true);
    row(QStringLiteral("steps"), setup_.stepsFile.isEmpty() ? QStringLiteral("built-in") : setup_.stepsFile,
        QString::fromLatin1(reckey::kSteps), true, false);
    row(QStringLiteral("voice"), announcer_->how(), QString(), announcer_->how() != QLatin1String("none"), false);
    h += QStringLiteral("</table>");
    setupView_->setText(h);
}

void DevelopmentPage::refreshRecordings() {
    while (QLayoutItem* it = list_->takeAt(0)) {
        if (it->widget()) {
            it->widget()->deleteLater();
        }
        delete it;
    }
    const auto recs = listRecordings(setup_.recordingsDir);
    listCard_->chip()->set(QStringLiteral("%1 SAVED").arg(recs.size()), ChipKind::Off);
    if (recs.empty()) {
        list_->addWidget(pibridge::ui::dimLabel(QStringLiteral("No recordings yet. They are saved in %1.").arg(setup_.recordingsDir)));
    }
    const QLocale c(QLocale::English);
    for (const RecordingInfo& r : recs) {
        auto* box = new QFrame;
        box->setStyleSheet(QStringLiteral("QFrame { background: #111114; border: 1px solid #26262c; border-radius: 3px; }"
                                          "QLabel { border: none; background: transparent; }"));
        auto* bv = new QVBoxLayout(box);
        bv->setContentsMargins(12, 10, 12, 10);
        bv->setSpacing(4);
        auto* top = new QHBoxLayout;
        auto* when = new QLabel(c.toString(r.when, QStringLiteral("d MMM yyyy, HH:mm")));
        when->setFont(pibridge::ui::bodyFont(15));
        top->addWidget(when);
        auto* chip = new pibridge::ui::Chip;
        chip->set(r.ok ? QStringLiteral("OK") : (r.checked ? QStringLiteral("PROBLEM") : QStringLiteral("NOT CHECKED")),
                  r.ok ? ChipKind::Info : ChipKind::Warn);
        top->addStretch(1);
        top->addWidget(chip);
        bv->addLayout(top);
        const QString mode = r.mode == QLatin1String("guided") ? QStringLiteral("guided steps")
                             : r.mode == QLatin1String("free") ? QStringLiteral("free recording")
                                                               : QString();
        QStringList facts{durationWords(r.durationS), sizeWords(r.bytes)};
        if (!mode.isEmpty()) {
            facts << mode;
        }
        bv->addWidget(pibridge::ui::dimLabel(facts.join(QStringLiteral("  ·  "))));
        auto* sum = new QLabel(r.summary);
        sum->setWordWrap(true);
        sum->setObjectName(r.ok ? QStringLiteral("dim") : QStringLiteral("error"));
        bv->addWidget(sum);
        auto* btns = new QHBoxLayout;
        auto* open = pibridge::ui::linkButton(QStringLiteral("Open folder"));
        auto* logs = pibridge::ui::linkButton(QStringLiteral("Logs"));
        auto* del = pibridge::ui::linkButton(QStringLiteral("Delete"));
        btns->addWidget(open);
        btns->addWidget(logs);
        btns->addStretch(1);
        btns->addWidget(del);
        bv->addLayout(btns);
        const QString path = r.path;
        connect(open, &QPushButton::clicked, this, [this, path] { openPath(path); });
        connect(logs, &QPushButton::clicked, this, [this, path] { openPath(path + QStringLiteral("/logs")); });
        connect(del, &QPushButton::clicked, this, [this, path, r, c] {
            if (session_ && session_->active() && session_->folder() == path) {
                return;
            }
            QMessageBox ask(QMessageBox::Warning, QStringLiteral("Delete recording"),
                            QStringLiteral("Delete the recording of %1 (%2)? This cannot be undone.")
                                .arg(c.toString(r.when, QStringLiteral("d MMM yyyy, HH:mm")), sizeWords(r.bytes)),
                            QMessageBox::Cancel, this);
            auto* yes = ask.addButton(QStringLiteral("Delete"), QMessageBox::DestructiveRole);
            ask.setDefaultButton(QMessageBox::Cancel);
            ask.exec();
            if (ask.clickedButton() != yes) {
                return;
            }
            QString err;
            if (!deleteRecording(setup_.recordingsDir, path, &err)) {
                say(err, true);
            } else {
                say(QStringLiteral("Recording deleted."));
            }
            refreshRecordings();
        });
        list_->addWidget(box);
    }
    list_->addStretch(1);
}

RecProbe DevelopmentPage::fakeProbe() {
    RecProbe p;
    p.socketReady = [this](const QString&) { return now() - fakeStartMs_ >= 800; };
    p.imuBytes = [this](const QString& dir) -> qint64 {
        if (now() - fakeStartMs_ < 2000) {
            return -1;
        }
        if (!fakeWritten_) {
            fakeWritten_ = true;
            const double s = session_ && session_->mode() == RecSession::Mode::Free ? 60 : recStepsSeconds(defaultRecSteps());
            writeFakeRecording(dir, s, false,
                               session_ && session_->mode() == RecSession::Mode::Free ? QStringLiteral("free")
                                                                                       : QStringLiteral("guided"),
                               5000LL * 1000000000);
        }
        return 4096;
    };
    p.readLog = [](const QString&) { return QString(); };
    p.monoNs = [this] { return 5000LL * 1000000000 + now() * 1000000; };
    return p;
}

void DevelopmentPage::startRecording() {
    refreshSetup();
    if (!readiness_.ready() || (session_ && session_->active())) {
        updateView();
        return;
    }
    QString err;
    auto steps = protocol(&err);
    fakeStartMs_ = now();
    fakeWritten_ = false;
    QString slamNote;
    prepareSessionSlam(setup_, &slamNote); // PiBridge's Basalt config for the inside-out settings (core/inside_out.h)
    if (!slamNote.isEmpty()) {
        readiness_.warnings << slamNote;
    }
    session_ = std::make_unique<RecSession>(setup_, *procs_, fake_ ? fakeProbe() : RecProbe::real(),
                                            [this](const QString& t) { announcer_->say(t); });
    const bool guided = mode_->current() == kGuided;
    session_->start(guided ? RecSession::Mode::Guided : RecSession::Mode::Free, std::move(steps), now());
    tickTimer_.start();
    updateView();
}

void DevelopmentPage::tick() {
    if (!session_) {
        tickTimer_.stop();
        return;
    }
    session_->tick(now());
    if (!session_->active()) {
        tickTimer_.stop();
        refreshRecordings();
    }
    updateView();
}

void DevelopmentPage::say(const QString& text, bool error) {
    message_->setObjectName(error ? QStringLiteral("error") : QString());
    message_->style()->unpolish(message_);
    message_->style()->polish(message_);
    message_->setText(text);
}

void DevelopmentPage::openPath(const QString& path) {
    if (fake_) {
        say(QStringLiteral("Fake mode: nothing is opened (%1).").arg(path));
        return;
    }
    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
}

void DevelopmentPage::setExternalBlock(const QString& why) {
    if (externalBlock_ == why) {
        return;
    }
    externalBlock_ = why;
    if (!session_ || !session_->active()) {
        refreshSetup();
        updateView();
    }
}

void DevelopmentPage::updateView() {
    const bool active = session_ && session_->active();
    if (active != lastActive_) {
        lastActive_ = active;
        emit recordingActiveChanged(active);
    }
    const RecState st = session_ ? session_->state() : RecState::Idle;
    recCard_->chip()->set(recChip(st, session_ ? session_->result() : std::nullopt, readiness_.ready()));
    record_->setVisible(!active);
    record_->setEnabled(readiness_.ready());
    record_->setToolTip(readiness_.ready() ? QStringLiteral("Starts the camera broker, the controller tracker and "
                                                            "Monado, and records until the steps end (or Stop).")
                                           : readiness_.blocking.join(QLatin1Char('\n')));
    stopTop_->setVisible(active);
    stopTop_->setEnabled(st != RecState::Stopping);
    mode_->setEnabled(!active);
    sessionLogs_->setVisible(session_ != nullptr && !session_->folder().isEmpty());

    if (session_ && st != RecState::Idle) {
        say(session_->message(), st == RecState::Failed);
    } else if (!readiness_.ready()) {
        say(QString());
    } else if (message_->text().isEmpty()) {
        say(QStringLiteral("Ready."));
    }
    QString reasons;
    if (!active) {
        for (const QString& b : readiness_.blocking) {
            reasons += QStringLiteral("<div style='color:#ef5a55; margin:2px 0'>%1</div>").arg(esc(b));
        }
        for (const QString& w : readiness_.warnings) {
            reasons += QStringLiteral("<div style='color:#f2b02e; margin:2px 0'>%1</div>").arg(esc(w));
        }
    }
    reasons_->setText(reasons);
    reasons_->setVisible(!reasons.isEmpty());

    // steps
    const StepRunner* run = session_ ? session_->steps() : nullptr;
    const bool showRun = active && (st == RecState::Recording || st == RecState::Stopping || st == RecState::Starting ||
                                    st == RecState::RetryWait);
    idleView_->setVisible(!showRun);
    runView_->setVisible(showRun);
    QString err;
    const auto plan = protocol(&err);
    if (!showRun) {
        stepsCard_->titleLabel()->setText(QStringLiteral("Steps"));
        stepsCard_->chip()->set(QStringLiteral("%1 STEPS, %2").arg(plan.size()).arg(durationWords(recStepsSeconds(plan)).toUpper()),
                                ChipKind::Off);
        QString h = QStringLiteral("<table cellspacing='0' cellpadding='2' width='100%'>");
        int n = 0;
        for (const RecStep& s : plan) {
            n++;
            h += QStringLiteral("<tr><td width='26' style='color:#5c5c66; padding-right:10px' align='right'>%1</td>"
                                "<td style='color:#e6e6ec'>%2</td><td style='color:#9a9aa5; padding-left:12px'>%3</td>"
                                "<td style='color:#9a9aa5; padding-left:12px' align='right'>%4 s</td></tr>")
                     .arg(n)
                     .arg(esc(s.text), kindWords(s.kind))
                     .arg(s.seconds);
        }
        h += QStringLiteral("</table><p style='color:#9a9aa5'>With <b>Free recording</b> there are no steps: it records "
                            "until you press Stop.</p>");
        plan_->setText(h);
    } else if (st == RecState::Starting || st == RecState::RetryWait) {
        stepsCard_->chip()->set(QStringLiteral("STARTING"), ChipKind::Warn);
        stepInfo_->setText(QStringLiteral("Please wait"));
        stepText_->setText(QStringLiteral("Starting the recording..."));
        countdown_->setText(QString());
        next_->setText(run && !run->steps().empty() ? QStringLiteral("First: %1").arg(run->steps().front().text) : QString());
        skip_->setEnabled(false);
    } else if (run && run->running()) {
        const RecStep* s = run->currentStep();
        stepsCard_->chip()->set(QStringLiteral("STEP %1 / %2").arg(run->current() + 1).arg(run->steps().size()),
                                ChipKind::Error);
        stepInfo_->setText(QStringLiteral("Step %1 of %2  ·  %3  ·  recording %4")
                               .arg(run->current() + 1)
                               .arg(run->steps().size())
                               .arg(kindWords(s->kind), durationWords(double(session_->recordingMs(now())) / 1000.0)));
        stepText_->setText(s->text);
        countdown_->setText(QString::number(run->secondsLeft(now())));
        const RecStep* nx = run->nextStep();
        next_->setText(nx ? QStringLiteral("Next: %1").arg(nx->text) : QStringLiteral("Last step. Then the recording stops."));
        skip_->setEnabled(true);
    } else if (st == RecState::Recording) { // free
        stepsCard_->chip()->set(QStringLiteral("FREE RECORDING"), ChipKind::Error);
        stepInfo_->setText(QStringLiteral("Free recording"));
        stepText_->setText(QStringLiteral("Recording. Move as you like; press Stop when you are done."));
        const qint64 sec = session_->recordingMs(now()) / 1000;
        countdown_->setText(QStringLiteral("%1:%2").arg(sec / 60).arg(sec % 60, 2, 10, QLatin1Char('0')));
        next_->setText(QString());
        skip_->setEnabled(false);
    } else {
        stepsCard_->chip()->set(QStringLiteral("STOPPING"), ChipKind::Warn);
        stepInfo_->setText(QStringLiteral("Saving"));
        stepText_->setText(QStringLiteral("Stopping and saving..."));
        countdown_->setText(QString());
        next_->setText(QString());
        skip_->setEnabled(false);
    }
    stop_->setEnabled(st != RecState::Stopping);
    updateDirect();
}

QString DevelopmentPage::home() const {
    return fake_ ? QStringLiteral("/fake/home") : QDir::homePath();
}

void DevelopmentPage::updateBuild() {
    if (!build_) {
        return;
    }
    const QString value = conf_.value(reckey::kMonado, QString());
    const QString choice = monadoBuildChoiceOf(value, home());
    const auto choices = monadoBuildChoices(home());
    int idx = -1;
    for (size_t i = 0; i < choices.size(); i++) {
        if (choices[i].id == choice) {
            idx = int(i);
        }
    }
    build_->setCurrent(idx);
    for (size_t i = 1; i < choices.size(); i++) {
        build_->setItemEnabled(int(i), fake_ || QFileInfo(choices[i].service).isFile());
    }
    const MonadoBuildInfo& b = setup_.build;
    const QString path = setup_.monado.path;
    buildPath_->setText(path.isEmpty() ? QStringLiteral("monado-service: not found%1")
                                             .arg(setup_.monado.wanted.isEmpty() ? QString()
                                                                                  : QStringLiteral(" (%1)").arg(setup_.monado.wanted))
                                       : QStringLiteral("monado-service: %1%2").arg(path, choice == QLatin1String("other")
                                                                                          ? QStringLiteral(" (chosen by hand)")
                                                                                          : QString()));
    const QString blocking = buildBlockingReason(b);
    buildWhat_->setText(path.isEmpty() ? QString() : buildSummary(b));
    buildWhat_->setStyleSheet(!blocking.isEmpty() && !b.checking ? QStringLiteral("color:#ef5a55;") : QString());
    buildWhat_->setVisible(!path.isEmpty());
    buildDriver_->setText(path.isEmpty() ? QString()
                          : setup_.steamVrDriverDir.isEmpty()
                              ? QStringLiteral("No SteamVR driver (steamvr-monado/) in this build: START TRACKING needs one.")
                              : QStringLiteral("SteamVR driver: %1").arg(setup_.steamVrDriverDir));
    buildDriver_->setVisible(!buildDriver_->text().isEmpty());
    if (path.isEmpty()) {
        buildCard_->chip()->set(QStringLiteral("NOT FOUND"), ChipKind::Error);
    } else if (b.checking) {
        buildCard_->chip()->set(QStringLiteral("CHECKING..."), ChipKind::Off);
    } else if (!blocking.isEmpty()) {
        buildCard_->chip()->set(QStringLiteral("MARKERS MISSING"), ChipKind::Error);
    } else {
        buildCard_->chip()->set(buildKindWords(b.kind).toUpper(), b.kind == BuildKind::PiBridge ? ChipKind::Info : ChipKind::Off);
    }
    const bool exists = fake_ ? !path.isEmpty() : QFileInfo(path).isFile();
    const CapState caps = setup_.caps.value_or(CapState{});
    capLine_->setText(capWords(caps, exists));
    capLine_->setStyleSheet(exists && caps.readable && !caps.ok() ? QStringLiteral("color:#f2b02e;") : QString());
    fixCaps_->setVisible(exists && !caps.ok());
    fixCaps_->setEnabled(!fixRunning_);
}

void DevelopmentPage::fixPermissions() {
    const QString path = setup_.monado.path;
    if (path.isEmpty() || fixRunning_) {
        return;
    }
    if (fake_) {
        say(QStringLiteral("Fake mode: nothing is run (pkexec %1).").arg(capFixArgs(path).join(QLatin1Char(' '))));
        return;
    }
    QMessageBox ask(QMessageBox::Question, QStringLiteral("Fix permissions"),
                    QStringLiteral("Monado runs its compositor with realtime priority so the headset's picture stays "
                                   "smooth when the PC is busy. Linux allows that only with the capability "
                                   "cap_sys_nice on the program file, and every rebuild of Monado drops it.\n\n"
                                   "PiBridge now asks for your password (pkexec) to run:\n"
                                   "setcap cap_sys_nice=ep %1\n\nNothing else is changed.")
                        .arg(path),
                    QMessageBox::Cancel, this);
    auto* yes = ask.addButton(QStringLiteral("Ask for the password"), QMessageBox::AcceptRole);
    ask.exec();
    if (ask.clickedButton() != yes) {
        return;
    }
    fixRunning_ = true;
    updateBuild();
    say(QStringLiteral("Waiting for the password prompt..."));
    runCapFix(*runner_, path, [this](bool ok, const QString& words) {
        fixRunning_ = false;
        say(words, !ok);
        refreshSetup();
        updateView();
    });
}

void DevelopmentPage::updateDirect() {
    if (!directCard_) {
        return;
    }
    const RecReadiness& r = direct_.readiness();
    const bool active = direct_.active();
    const RecState st = direct_.state();
    directCard_->chip()->set(ioChip(st, r.ready()));
    directStart_->setVisible(!active);
    directStart_->setEnabled(r.ready());
    directStart_->setToolTip(r.ready() ? QStringLiteral("Starts the camera broker, the controller tracker, monado-service "
                                                        "and the home scene. Stop ends exactly those.")
                                       : r.blocking.join(QLatin1Char('\n')));
    directStop_->setVisible(active);
    directStop_->setEnabled(st != RecState::Stopping);
    QString m = direct_.message();
    if (m.isEmpty() && r.ready()) {
        m = QStringLiteral("Ready.");
    }
    directMsg_->setText(m);
    directMsg_->setObjectName(st == RecState::Failed ? QStringLiteral("error") : QString());
    directMsg_->style()->unpolish(directMsg_);
    directMsg_->style()->polish(directMsg_);
    const IoSettings io = ioSettings(conf_);
    home_->setCheckedQuiet(io.homeScene);
    const HomeState h = direct_.home();
    homeLine_->setText(io.homeScene ? QStringLiteral("Home scene: ") + h.words
                                    : QStringLiteral("Off: the headset stays black until an app starts."));
    homeLine_->setStyleSheet(h.run == HomeRun::NotFound || h.run == HomeRun::NoRuntimeFile ||
                                     (h.run == HomeRun::Exited && h.words != QStringLiteral("Ended (code 0)."))
                                 ? QStringLiteral("color: #f2b02e;")
                                 : QString());
    QString reasons;
    if (!active) {
        for (const QString& b : r.blocking) {
            reasons += QStringLiteral("<div style='color:#ef5a55; margin:2px 0'>%1</div>").arg(esc(b));
        }
        for (const QString& w : r.warnings) {
            reasons += QStringLiteral("<div style='color:#f2b02e; margin:2px 0'>%1</div>").arg(esc(w));
        }
    }
    directReasons_->setText(reasons);
    directReasons_->setVisible(!reasons.isEmpty());
}

void DevelopmentPage::demoRecording(qint64 ms) {
    if (!fake_) {
        return;
    }
    mode_->setCurrent(kGuided);
    startRecording();
    for (qint64 step : {1000LL, 1500LL}) {
        offsetMs_ += step;
        tick();
    }
    offsetMs_ += ms;
    tick();
}

void DevelopmentPage::demoFinish() {
    if (!fake_ || !session_) {
        return;
    }
    offsetMs_ += 200000; // past the last step
    tick();
    tick();
}

} // namespace pibridge::app
