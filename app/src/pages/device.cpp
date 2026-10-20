// SPDX-License-Identifier: MIT
#include "device.h"

#include <QClipboard>
#include <QCoreApplication>
#include <QDir>
#include <QGridLayout>
#include <QGuiApplication>
#include <QLabel>
#include <QDateTime>
#include <QHBoxLayout>
#include <QLayoutItem>
#include <QMessageBox>
#include <QTimer>
#include <QPointer>
#include <QPushButton>
#include <QVBoxLayout>

#include "../core/chovr.h"
#include "../core/gazeshm.h"
#include "../core/inside_out.h"
#include "../core/paths.h"
#include "../core/rec_setup.h"
#include "../core/steamvr_drivers.h"
#include "../qv/layerinstall.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

void clear(QGridLayout* g) {
    while (QLayoutItem* it = g->takeAt(0)) {
        delete it->widget();
        delete it;
    }
}

// KEY   value   COPY
//       note
void fill(QGridLayout* g, const std::vector<InfoRow>& rows, QWidget* owner) {
    clear(g);
    int r = 0;
    for (const InfoRow& row : rows) {
        auto* k = pibridge::ui::capsLabel(row.key, 13);
        k->setObjectName(QStringLiteral("dim"));
        g->addWidget(k, r, 0, Qt::AlignTop);
        auto* v = new QLabel(row.value.isEmpty() ? QStringLiteral("-") : row.value);
        v->setFont(pibridge::ui::bodyFont(14));
        v->setTextInteractionFlags(Qt::TextSelectableByMouse);
        g->addWidget(v, r, 1, Qt::AlignTop);
        if (row.copy && !row.value.isEmpty()) {
            auto* c = new QPushButton(QStringLiteral("COPY"));
            c->setObjectName(QStringLiteral("link"));
            c->setCursor(Qt::PointingHandCursor);
            c->setToolTip(QStringLiteral("Copy the serial number (for support)"));
            const QString text = row.value;
            QObject::connect(c, &QPushButton::clicked, owner, [text] { QGuiApplication::clipboard()->setText(text); });
            g->addWidget(c, r, 2, Qt::AlignTop | Qt::AlignRight);
        }
        r++;
        if (!row.note.isEmpty()) {
            auto* n = pibridge::ui::dimLabel(row.note);
            n->setFont(pibridge::ui::bodyFont(12));
            g->addWidget(n, r, 1, 1, 2);
            r++;
        }
    }
    g->setColumnStretch(1, 1);
}

QString keyOf(const std::vector<InfoRow>& rows) {
    QString k;
    for (const InfoRow& r : rows) {
        k += r.key + QLatin1Char('\x1f') + r.value + QLatin1Char('\x1f') + r.note + QLatin1Char('\x1e');
    }
    return k;
}

QGridLayout* rowsIn(pibridge::ui::Card* card) {
    auto* g = new QGridLayout;
    g->setHorizontalSpacing(24);
    g->setVerticalSpacing(8);
    card->body()->addLayout(g);
    return g;
}

QString runtimeDir() {
    const QByteArray x = qgetenv("XDG_RUNTIME_DIR");
    return x.isEmpty() ? QStringLiteral("/tmp") : QString::fromLocal8Bit(x);
}

qint64 nowMs() { return QDateTime::currentMSecsSinceEpoch(); }

} // namespace

DevicePage::DevicePage(StateSource& source, Conf& conf, IoSessionControl& io, IoSessionControl& direct, QWidget* parent)
    : QWidget(parent), source_(source), conf_(conf), io_(io), direct_(direct), ctrl_(statusDir(), runtimeDir()) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* heading = new QLabel(QStringLiteral("Device"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);
    root->addWidget(pibridge::ui::dimLabel(
        QStringLiteral("What the headset, the controllers and the base stations are, and how the app renders now. "
                       "Serial numbers are read live and shown only here, for support.")));

    auto* grid = new QGridLayout;
    grid->setSpacing(10);
    root->addLayout(grid, 1);
    headset_ = new pibridge::ui::Card(QStringLiteral("No headset"));
    headsetRows_ = rowsIn(headset_);
    headset_->body()->addStretch(1);
    grid->addWidget(headset_, 0, 0);

    auto* devices = new pibridge::ui::Card(QStringLiteral("Controllers and base stations"));
    deviceRows_ = rowsIn(devices);
    devicesNote_ = pibridge::ui::dimLabel(QString());
    devices->body()->addWidget(devicesNote_);
    devices->body()->addStretch(1);
    grid->addWidget(devices, 0, 1);

    // Pair / Unpair / Test vibration per hand, like Pimax Play's controller page
    auto* pairing = new pibridge::ui::Card(QStringLiteral("Pair and test controllers"));
    auto* pg = new QGridLayout;
    pg->setHorizontalSpacing(12);
    pg->setVerticalSpacing(6);
    pairing->body()->addLayout(pg);
    for (int i = 0; i < 2; i++) {
        const int slot = i == 0 ? kSlotLeft : kSlotRight;
        CtrlRow& row = ctrlRows_[i];
        auto* k = pibridge::ui::capsLabel(i == 0 ? QStringLiteral("Left controller") : QStringLiteral("Right controller"), 13);
        k->setObjectName(QStringLiteral("dim"));
        pg->addWidget(k, i * 2, 0);
        row.state = new QLabel;
        row.state->setFont(pibridge::ui::bodyFont(14));
        pg->addWidget(row.state, i * 2, 1);
        auto* buttons = new QHBoxLayout;
        row.pair = pibridge::ui::toolButton(QStringLiteral("Pair"));
        row.pair->setToolTip(QStringLiteral("Put the controller in pairing mode, then press Pair and wait up to 30 s"));
        row.unpair = pibridge::ui::toolButton(QStringLiteral("Unpair"));
        row.unpair->setToolTip(QStringLiteral("Forget this controller in the headset"));
        row.buzz = pibridge::ui::toolButton(QStringLiteral("Test vibration"));
        row.buzz->setToolTip(QStringLiteral("A short buzz (60 %, 0.3 s)"));
        for (auto* b : {row.pair, row.unpair, row.buzz}) {
            buttons->addWidget(b);
        }
        buttons->addStretch(1);
        pg->addLayout(buttons, i * 2, 2);
        row.text = pibridge::ui::dimLabel(QString());
        row.text->setFont(pibridge::ui::bodyFont(12));
        row.text->setWordWrap(true);
        pg->addWidget(row.text, i * 2 + 1, 1, 1, 2);
        connect(row.pair, &QPushButton::clicked, this, [this, slot] { controllerAction(slot, 0); });
        connect(row.unpair, &QPushButton::clicked, this, [this, slot] { controllerAction(slot, 1); });
        connect(row.buzz, &QPushButton::clicked, this, [this, slot] { controllerAction(slot, 2); });
    }
    pg->setColumnStretch(2, 1);
    pairing->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "Pair and Unpair work while no inside-out tracking runs (the headset service holds the headset then). Test "
        "vibration works always, also during inside-out tracking.")));
    grid->addWidget(pairing, 1, 0, 1, 2);
    ctrlTimer_ = new QTimer(this);
    ctrlTimer_->setInterval(250);
    connect(ctrlTimer_, &QTimer::timeout, this, [this] {
        ctrl_.poll(source_.state().keepalive, nowMs());
        applyControllers();
        if (!ctrl_.busy()) {
            ctrlTimer_->stop();
        }
    });

    auto* render = new pibridge::ui::Card(QStringLiteral("Rendering"));
    renderRows_ = rowsIn(render);
    render->body()->addStretch(1);
    grid->addWidget(render, 2, 0, 1, 2);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    connect(&source_, &StateSource::changed, this, [this] {
        if (isVisible()) {
            apply();
        }
    });
    connect(&io_, &IoSessionControl::changed, this, &DevicePage::apply);
    connect(&direct_, &IoSessionControl::changed, this, &DevicePage::apply);
    readFiles();
    apply();
}

void DevicePage::controllerAction(int slot, int action) {
    const KeepaliveStatus& k = source_.state().keepalive;
    if (action == 1) {
        const auto b = QMessageBox::question(
            this, QStringLiteral("Unpair controller"),
            QStringLiteral("Unpair the %1 controller? It stops working until you pair it again.").arg(sideName(slot)));
        if (b != QMessageBox::Yes) {
            return;
        }
    }
    const qint64 now = nowMs();
    if (action == 0) {
        (void)ctrl_.pair(slot, k, now);
    } else if (action == 1) {
        (void)ctrl_.unpair(slot, k, now);
    } else {
        (void)ctrl_.testVibration(slot, k, now);
    }
    if (ctrl_.busy()) {
        ctrlTimer_->start();
    }
    applyControllers();
}

void DevicePage::applyControllers() {
    const KeepaliveStatus& k = source_.state().keepalive;
    const bool paused = k.running && k.state == QLatin1String("paused");
    for (int i = 0; i < 2; i++) {
        const int slot = i == 0 ? kSlotLeft : kSlotRight;
        CtrlRow& row = ctrlRows_[i];
        const bool connected = i == 0 ? k.leftConnected : k.rightConnected;
        row.state->setText(!k.running ? QStringLiteral("Headset service off")
                           : paused   ? QStringLiteral("In use by inside-out tracking")
                           : !k.headsetPresent ? QStringLiteral("No headset")
                           : connected ? QStringLiteral("Connected")
                                       : QStringLiteral("Not connected"));
        const ControllerOps::Side sd = ctrl_.side(slot);
        row.text->setText(sd.text);
        row.text->setVisible(!sd.text.isEmpty());
        row.text->setStyleSheet(sd.error ? QStringLiteral("color: %1;").arg(pibridge::ui::color::red.name())
                                         : QString());
        // EVO-style: a connected (paired, heard in the last 10 s) controller shows Unpair; an absent one shows Pair.
        // A paired controller that fell asleep reads as not connected, so it offers Pair again - keepalive gives no
        // stored-bond list to tell "asleep" from "never paired" apart.
        row.pair->setVisible(!connected);
        row.unpair->setVisible(connected);
        // Pair / Unpair need the headset free (service up, no inside-out session owning it); Test needs it connected.
        const bool idle = !ctrl_.busy();
        const bool canAct = idle && k.running && !paused && k.headsetPresent;
        row.pair->setEnabled(canAct);
        row.unpair->setEnabled(canAct);
        row.buzz->setEnabled(idle); // vibration works always, even during inside-out tracking
    }
}

void DevicePage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    readFiles();
    apply();
}

void DevicePage::readFiles() {
    if (source_.fake()) {
        files_ = fakeDeviceInputs(true);
        return;
    }
    conf_.reload();
    files_.chovrIpd = chovrConfiguredIpd(chovrSettingsFile());
    files_.supersample = steamVrSupersample(steamVrSettingsFileFor(openVrPathsFile(), QDir::homePath()));
    files_.confRefreshHz = conf_.refreshHz();
    const qvgui::Install qv = qvgui::findInstall();
    files_.qvInstalled = qv.userInstalled;
    files_.qvEnabled = qv.userInstalled && qv.userEnabled;
    files_.insideOut = ioSettings(conf_).mode == TrackingMode::InsideOut;
    // The Monado build: scanned on a worker thread (200+ MB); the page updates when the scan is done.
    RecEnv env = RecEnv::defaults();
    QPointer<DevicePage> self(this);
    env.buildInfo = [self](const QString& p) {
        return monadoBuildInfoAsync(p, [self, p] {
            QMetaObject::invokeMethod(
                QCoreApplication::instance(), [self, p] {
                    if (self) {
                        self->files_.monado = monadoBuildInfoAsync(p); // the cached result now
                        self->files_.monado.path = p;
                        self->apply();
                    }
                },
                Qt::QueuedConnection);
        });
    };
    const RecSetup s = resolveRecSetup(conf_, env);
    files_.monado = s.build;
    files_.monado.path = s.monado.path;
}

void DevicePage::apply() {
    DeviceInputs in = files_;
    const AppState& st = source_.state();
    in.steamvrRunning = st.steamvrRunning;
    in.steamvr = st.steamvr;
    in.keepalive = st.keepalive;
    in.gazeIpdMm = 0;
    if (const auto g = source_.gaze();
        g && g->state == 2 && (source_.fake() || monotonicNs() - g->writtenNs < 200'000'000)) {
        in.gazeIpdMm = g->ipdMm > 0 ? g->ipdMm : 0;
    }
    if (source_.fake()) {
        in.gazeIpdMm = st.eyed.state == QLatin1String("streaming") ? 63.8f : 0.0f;
    }
    in.ioRunning = io_.active();
    in.directRunning = direct_.active();
    const DeviceView v = deviceView(in);
    headset_->titleLabel()->setText(v.title);
    headset_->chip()->set(v.chip);
    const QString key = keyOf(v.headset) + QLatin1Char('|') + keyOf(v.controllers) + keyOf(v.stations) +
                        QLatin1Char('|') + keyOf(v.rendering) + v.devicesNote;
    applyControllers();
    if (key == key_) {
        return;
    }
    key_ = key;
    fill(headsetRows_, v.headset, this);
    std::vector<InfoRow> devs = v.controllers;
    devs.insert(devs.end(), v.stations.begin(), v.stations.end());
    fill(deviceRows_, devs, this);
    devicesNote_->setText(v.devicesNote);
    devicesNote_->setVisible(!v.devicesNote.isEmpty());
    fill(renderRows_, v.rendering, this);
}

} // namespace pibridge::app
