// SPDX-License-Identifier: MIT
#include "room.h"

#include "core/vr_backend.h"

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QStyle>
#include <QUrl>
#include <QVBoxLayout>
#include <cmath>
#include <ctime>
#include <numbers>

#include "help_window.h"
#include "../core/anchor_offset.h"
#include "../core/inside_out.h"
#include "../core/roommap_link.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"
#include "pibridge/room/geom.h"

namespace pibridge::app {

namespace room = pibridge::room;

namespace {

QString metres(double v) { return QLocale(QLocale::English).toString(v, 'f', 2) + QStringLiteral(" m"); }

const QColor kGreen{0x3d, 0xdb, 0x85};
const QColor kYellow{0xff, 0xd6, 0x47};
const QColor kArea{0xff, 0x4d, 0xd9};
const QColor kWarn{0xff, 0x8c, 0x26};

double steadyNow() {
    static QElapsedTimer t = [] {
        QElapsedTimer e;
        e.start();
        return e;
    }();
    return static_cast<double>(t.elapsed()) / 1000.0 + 100.0;
}

} // namespace

// ------------------------------------------------------------------------------------------------ floor plan

FloorPlan::FloorPlan(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setCursor(Qt::CrossCursor);
    setToolTip(QStringLiteral("Drag a corner to move it. Double-click a wall to add a corner, right-click a corner to "
                              "delete it. Forward (where games face you) is up."));
}

void FloorPlan::setEditor(room::Editor* ed, double warnDistance) {
    ed_ = ed;
    warn_ = warnDistance;
    update();
}

void FloorPlan::fit() {
    double x0 = -1, x1 = 1, z0 = -1, z1 = 1;
    if (ed_ != nullptr) {
        room::Poly all = ed_->pts;
        const auto cs = ed_->areaCorners(ed_->centerPose(), ed_->area.w, ed_->area.h);
        all.insert(all.end(), cs.begin(), cs.end());
        for (const room::Pt2 p : all) {
            x0 = std::min(x0, p.x);
            x1 = std::max(x1, p.x);
            z0 = std::min(z0, p.z);
            z1 = std::max(z1, p.z);
        }
    }
    const double m = 0.6;
    x0 -= m;
    x1 += m;
    z0 -= m;
    z1 += m;
    scale_ = std::min(width() / (x1 - x0), height() / (z1 - z0));
    origin_ = QPointF(width() / 2.0 - (x0 + x1) / 2 * scale_, height() / 2.0 - (z0 + z1) / 2 * scale_);
}

QPointF FloorPlan::toScreen(room::Pt2 p) const { return {origin_.x() + p.x * scale_, origin_.y() + p.z * scale_}; }

room::Pt2 FloorPlan::toFloor(QPointF s) const { return {(s.x() - origin_.x()) / scale_, (s.y() - origin_.y()) / scale_}; }

int FloorPlan::vertexAt(QPointF s) const {
    if (ed_ == nullptr) {
        return -1;
    }
    int best = -1;
    double bd = 12;
    for (std::size_t i = 0; i < ed_->pts.size(); i++) {
        const QPointF q = toScreen(ed_->pts[i]);
        const double d = std::hypot(q.x() - s.x(), q.y() - s.y());
        if (d < bd) {
            bd = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

void FloorPlan::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0x0f, 0x10, 0x12));
    if (drag_ < 0) {
        fit(); // keep the scale steady while a corner is dragged
    }
    // grid: 0.5 m, every metre brighter
    const room::Pt2 tl = toFloor({0, 0});
    const room::Pt2 br = toFloor({static_cast<double>(width()), static_cast<double>(height())});
    for (int i = static_cast<int>(std::floor(tl.x * 2)); i <= static_cast<int>(std::ceil(br.x * 2)); i++) {
        p.setPen(QPen(i % 2 == 0 ? QColor(0x2a, 0x30, 0x2e) : QColor(0x1c, 0x20, 0x1f), 1));
        const double x = toScreen({i * 0.5, 0}).x();
        p.drawLine(QPointF(x, 0), QPointF(x, height()));
    }
    for (int i = static_cast<int>(std::floor(tl.z * 2)); i <= static_cast<int>(std::ceil(br.z * 2)); i++) {
        p.setPen(QPen(i % 2 == 0 ? QColor(0x2a, 0x30, 0x2e) : QColor(0x1c, 0x20, 0x1f), 1));
        const double y = toScreen({0, i * 0.5}).y();
        p.drawLine(QPointF(0, y), QPointF(width(), y));
    }
    p.setFont(pibridge::ui::labelFont(11));
    p.setPen(pibridge::ui::color::faint);
    p.drawText(QRectF(8, height() - 22, 200, 18), Qt::AlignLeft, QStringLiteral("GRID 0.5 M"));
    if (ed_ == nullptr || ed_->pts.size() < 3) {
        p.setPen(pibridge::ui::color::dim);
        p.setFont(pibridge::ui::bodyFont(14));
        p.drawText(rect(), Qt::AlignCenter, QStringLiteral("No boundary to show"));
        return;
    }
    const auto poly = [&](const room::Poly& pts) {
        QPolygonF q;
        for (const room::Pt2 v : pts) {
            q << toScreen(v);
        }
        return q;
    };
    // the saved outline while it is being changed
    if (ed_->changed() && ed_->orig.size() > 2) {
        p.setPen(QPen(QColor(0x6b, 0x7a, 0x73), 1.5, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        p.drawPolygon(poly(ed_->orig));
    }
    // the boundary
    QColor fill = kGreen;
    fill.setAlpha(28);
    p.setBrush(fill);
    p.setPen(QPen(kGreen, 2.2));
    p.drawPolygon(poly(ed_->pts));
    // warning distance: where SteamVR starts to show the walls
    if (warn_ > 0) {
        const room::Poly in = room::insetPoly(ed_->pts, warn_);
        if (in.size() > 2) {
            p.setBrush(Qt::NoBrush);
            p.setPen(QPen(kWarn, 1.2, Qt::DotLine));
            p.drawPolygon(poly(in));
        }
    }
    // wall lengths
    p.setFont(pibridge::ui::labelFont(11));
    for (std::size_t i = 0; i < ed_->pts.size(); i++) {
        const room::Pt2 a = ed_->pts[i];
        const room::Pt2 b = ed_->pts[(i + 1) % ed_->pts.size()];
        const double l = room::dist(a, b);
        if (l * scale_ < 60) {
            continue;
        }
        const QPointF m = (toScreen(a) + toScreen(b)) / 2;
        // outside the room: along the outward normal (the polygon is counter-clockwise or not; test both)
        QPointF n(-(b.z - a.z) / l, (b.x - a.x) / l);
        const room::Pt2 probe{(a.x + b.x) / 2 + n.x() * 0.05, (a.z + b.z) / 2 + n.y() * 0.05};
        if (room::pointInPoly(ed_->pts, probe.x, probe.z)) {
            n = -n;
        }
        p.setPen(pibridge::ui::color::text);
        const QPointF c = m + n * 16;
        p.drawText(QRectF(c.x() - 40, c.y() - 9, 80, 18), Qt::AlignCenter, metres(l));
    }
    // play area and centre
    const room::Rigid2 g = ed_->centerPose();
    const auto cs = ed_->areaCorners(g, ed_->area.w, ed_->area.h);
    QPolygonF area;
    for (const room::Pt2 c : cs) {
        area << toScreen(c);
    }
    QColor af = kArea;
    af.setAlpha(18);
    p.setBrush(af);
    p.setPen(QPen(kArea, 1.6));
    p.drawPolygon(area);
    const QPointF o = toScreen(g.apply({0, 0}));
    const QPointF tip = toScreen(g.apply({0, -0.45}));
    p.setPen(QPen(kYellow, 2.2, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(toScreen(g.apply({-0.2, 0})), toScreen(g.apply({0.2, 0})));
    p.drawLine(toScreen(g.apply({0, 0.2})), tip);
    p.drawLine(tip, toScreen(g.apply({-0.08, -0.35})));
    p.drawLine(tip, toScreen(g.apply({0.08, -0.35})));
    p.setFont(pibridge::ui::labelFont(11));
    p.drawText(QRectF(tip.x() - 50, tip.y() - 22, 100, 16), Qt::AlignCenter, QStringLiteral("FORWARD"));
    p.setPen(kArea);
    const QPointF lab = toScreen(g.apply({0, ed_->area.h / 2})) + QPointF(0, 14);
    p.drawText(QRectF(lab.x() - 110, lab.y() - 8, 220, 16), Qt::AlignCenter,
               QStringLiteral("PLAY AREA %1 X %2").arg(metres(ed_->area.w).toUpper(), metres(ed_->area.h).toUpper()));
    (void)o;
    // corners
    for (std::size_t i = 0; i < ed_->pts.size(); i++) {
        const QPointF q = toScreen(ed_->pts[i]);
        const bool hot = static_cast<int>(i) == drag_;
        p.setPen(QPen(hot ? QColor(0xff, 0xb3, 0x40) : kGreen, 2));
        p.setBrush(QColor(0x0f, 0x10, 0x12));
        p.drawEllipse(q, hot ? 7 : 5.5, hot ? 7 : 5.5);
    }
}

void FloorPlan::mousePressEvent(QMouseEvent* e) {
    if (ed_ == nullptr) {
        return;
    }
    const int v = vertexAt(e->position());
    if (e->button() == Qt::RightButton && v >= 0) {
        if (ed_->pts.size() <= 3) {
            ed_->say("A boundary needs at least 3 points");
        } else {
            ed_->push();
            ed_->pts.erase(ed_->pts.begin() + v);
        }
        emit edited();
        update();
        return;
    }
    if (e->button() == Qt::LeftButton && v >= 0) {
        ed_->push();
        drag_ = v;
        const room::Pt2 f = toFloor(e->position());
        dragOff_ = {ed_->pts[static_cast<std::size_t>(v)].x - f.x, ed_->pts[static_cast<std::size_t>(v)].z - f.z};
        update();
    }
}

void FloorPlan::mouseMoveEvent(QMouseEvent* e) {
    if (ed_ == nullptr || drag_ < 0) {
        setCursor(vertexAt(e->position()) >= 0 ? Qt::OpenHandCursor : Qt::CrossCursor);
        return;
    }
    const room::Pt2 f = toFloor(e->position());
    // 1 cm steps: the plan is for rough placement, the headset for the exact one
    ed_->pts[static_cast<std::size_t>(drag_)] = {std::round((f.x + dragOff_.x) * 100) / 100,
                                                 std::round((f.z + dragOff_.z) * 100) / 100};
    emit edited();
    update();
}

void FloorPlan::mouseReleaseEvent(QMouseEvent*) {
    if (drag_ >= 0) {
        drag_ = -1;
        emit edited();
        update();
    }
}

void FloorPlan::mouseDoubleClickEvent(QMouseEvent* e) {
    if (ed_ == nullptr || ed_->pts.size() < 3) {
        return;
    }
    const room::Pt2 f = toFloor(e->position());
    int be = -1;
    double bd = 14 / scale_;
    double bt = 0;
    for (std::size_t i = 0; i < ed_->pts.size(); i++) {
        const auto sd = room::segDist(f, ed_->pts[i], ed_->pts[(i + 1) % ed_->pts.size()]);
        if (sd.d < bd) {
            bd = sd.d;
            be = static_cast<int>(i);
            bt = sd.t;
        }
    }
    if (be < 0) {
        return;
    }
    const room::Pt2 a = ed_->pts[static_cast<std::size_t>(be)];
    const room::Pt2 b = ed_->pts[(static_cast<std::size_t>(be) + 1) % ed_->pts.size()];
    ed_->push();
    ed_->pts.insert(ed_->pts.begin() + be + 1, {a.x + (b.x - a.x) * bt, a.z + (b.z - a.z) * bt});
    emit edited();
    update();
}

// ------------------------------------------------------------------------------------------------ the page

RoomPage::RoomPage(StateSource& source, Runner& runner, Actions::ToolFinder findTool, Conf* conf, QWidget* parent)
    : QWidget(parent), source_(source), runner_(runner), conf_(conf), findTool_(std::move(findTool)), fake_(source.fake()),
      env_(roomEnv(fake_)) {
    setObjectName(QStringLiteral("page"));
    if (fake_) {
        writeFakeRoomFiles(env_);
    }
    auto* rootL = new QVBoxLayout(this);
    rootL->setContentsMargins(18, 12, 18, 12);
    rootL->setSpacing(10);
    auto* headRow = new QHBoxLayout;
    auto* heading = new QLabel(QStringLiteral("Room"));
    heading->setFont(pibridge::ui::titleFont(24));
    headRow->addWidget(heading);
    headRow->addStretch(1);
    auto* help = pibridge::ui::linkButton(QStringLiteral("How room setup works"));
    help->setIcon(pibridge::ui::lineIcon(pibridge::ui::Icon::Help, pibridge::ui::color::accentText));
    help->setToolTip(QStringLiteral("Shows docs/ROOM-SETUP.md: the controls in the headset, the files, what is not done yet."));
    connect(help, &QPushButton::clicked, this, [this] { showHelp(this, QStringLiteral("ROOM-SETUP.md")); });
    headRow->addWidget(help);
    rootL->addLayout(headRow);
    // the inside-out mode: universe 30 and the room map (an explanation, not a warning)
    auto* ioRow = new QHBoxLayout;
    ioNote_ = new QLabel;
    ioNote_->setWordWrap(true);
    ioNote_->setStyleSheet(QStringLiteral("color: #9aa3ad;"));
    ioRow->addWidget(ioNote_, 1);
    createIo_ = pibridge::ui::toolButton(QStringLiteral("CREATE INSIDE-OUT BOUNDARY"));
    createIo_->setToolTip(QStringLiteral("Adds universe 30 (the inside-out headset's) with a 2 x 2 m boundary to SteamVR's chaperone "
                                         "file (a backup first); the Lighthouse universes stay as they are."));
    ioRow->addWidget(createIo_, 0, Qt::AlignTop);
    rootL->addLayout(ioRow);
    connect(createIo_, &QPushButton::clicked, this, &RoomPage::createInsideOutUniverse);

    auto* cols = new QHBoxLayout;
    cols->setSpacing(10);
    rootL->addLayout(cols, 1);
    auto* left = new QVBoxLayout;
    left->setSpacing(10);
    auto* right = new QVBoxLayout;
    right->setSpacing(10);
    cols->addLayout(left, 5);
    cols->addLayout(right, 3);

    // The "Set up your playspace" card (its SETUP PLAYSPACE button and the body/eye-height inputs) is gone: the floor
    // now defaults to ~1.60 m by the eye (no height entry), and the setup is launched from the Dashboard (START VR ->
    // RoomPage::launchSetup/launchQuickSetup -> mapRoom). The floor plan is the top of the left column.

    // ---- the floor plan
    planCard_ = new pibridge::ui::Card(QStringLiteral("Floor plan"));
    planInfo_ = pibridge::ui::dimLabel(QString());
    planCard_->body()->addWidget(planInfo_);
    plan_ = new FloorPlan;
    plan_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    planCard_->body()->addWidget(plan_, 1);
    connect(plan_, &FloorPlan::edited, this, [this] {
        if (ed_) {
            ed_->dirty = true;
        }
        updateView();
    });
    auto* tools = new QHBoxLayout;
    tools->setSpacing(6);
    const auto tool = [&](const QString& text, const QString& tip, auto fn) {
        auto* b = pibridge::ui::toolButton(text);
        b->setToolTip(tip);
        b->setMinimumHeight(34);
        connect(b, &QPushButton::clicked, this, [this, fn] {
            if (ed_) {
                fn(*ed_);
                say(QString::fromStdString(ed_->status()));
                updateView();
            }
        });
        tools->addWidget(b);
        return b;
    };
    tool(QStringLiteral("NEW 2 X 2 M"), QStringLiteral("A new square 2 x 2 m boundary around the play centre."),
         [](room::Editor& e) {
             e.setPts(e.centerPose().all(room::rectangle(2, 2)), "New 2 x 2 m room. Drag the corners to your walls");
         });
    tool(QStringLiteral("FIT PLAY AREA"), QStringLiteral("The biggest rectangle around the play centre that fits inside "
                                                         "the walls, 10 cm from them."),
         [](room::Editor& e) { e.fitArea(); });
    tool(QStringLiteral("CENTER"), QStringLiteral("Moves the play centre (where games put you) to the middle of the room."),
         [](room::Editor& e) { e.centerInRoom(); });
    tool(QStringLiteral("TURN -15°"), QStringLiteral("Turns the play centre's forward direction 15 degrees to the left."),
         [](room::Editor& e) {
             e.turnArea(-std::numbers::pi / 12);
             e.say("Forward turned 15 degrees left");
         });
    tool(QStringLiteral("TURN +15°"), QStringLiteral("Turns the play centre's forward direction 15 degrees to the right."),
         [](room::Editor& e) {
             e.turnArea(std::numbers::pi / 12);
             e.say("Forward turned 15 degrees right");
         });
    tool(QStringLiteral("SQUARE TO WALLS"), QStringLiteral("Turns the play area so its edges run along the walls."),
         [](room::Editor& e) { e.squareToRoom(); });
    undo_ = tool(QStringLiteral("UNDO"), QStringLiteral("Steps back one change."), [](room::Editor& e) { e.doUndo(); });
    tools->addStretch(1);
    planCard_->body()->addLayout(tools);
    auto* acts = new QHBoxLayout;
    acts->setSpacing(6);
    apply_ = pibridge::ui::accentButton(QStringLiteral("APPLY TO STEAMVR"));
    apply_->setMinimumHeight(38);
    apply_->setToolTip(QStringLiteral("Writes this boundary, play area and centre into SteamVR's chaperone file (a "
                                      "backup of the file is kept first)."));
    revert_ = pibridge::ui::toolButton(QStringLiteral("REVERT"));
    revert_->setMinimumHeight(38);
    revert_->setToolTip(QStringLiteral("Back to what the chaperone file holds now."));
    restore_ = pibridge::ui::toolButton(QStringLiteral("RESTORE ORIGINAL"));
    restore_->setMinimumHeight(38);
    restore_->setToolTip(QStringLiteral("Loads the boundary from the oldest backup, from before PiBridge changed anything. "
                                        "Apply to keep it."));
    auto* saveRoom = pibridge::ui::toolButton(QStringLiteral("SAVE AS ROOM..."));
    saveRoom->setMinimumHeight(38);
    saveRoom->setToolTip(QStringLiteral("Keeps what you see as a named room (\"Door open\", \"Door closed\") to switch to later."));
    acts->addWidget(apply_);
    acts->addWidget(revert_);
    acts->addWidget(restore_);
    acts->addStretch(1);
    acts->addWidget(saveRoom);
    planCard_->body()->addLayout(acts);
    applyWhy_ = new QLabel;
    applyWhy_->setWordWrap(true);
    applyWhy_->setObjectName(QStringLiteral("dim"));
    planCard_->body()->addWidget(applyWhy_);
    message_ = new QLabel;
    message_->setWordWrap(true);
    message_->setFont(pibridge::ui::bodyFont(14));
    planCard_->body()->addWidget(message_);
    planCard_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);
    planCard_->setMinimumHeight(520);
    left->addWidget(planCard_, 1);
    connect(apply_, &QPushButton::clicked, this, &RoomPage::apply);
    connect(revert_, &QPushButton::clicked, this, [this] {
        if (ed_) {
            const std::string u = ed_->st.universe();
            reload(u);
            say(QStringLiteral("Back to the boundary in the chaperone file."));
        }
    });
    connect(restore_, &QPushButton::clicked, this, [this] {
        if (ed_) {
            ed_->restoreOriginal();
            say(QString::fromStdString(ed_->status()));
            updateView();
        }
    });
    connect(saveRoom, &QPushButton::clicked, this, &RoomPage::saveAsRoom);

    // ---- universes
    uniCard_ = new pibridge::ui::Card(QStringLiteral("SteamVR universes"));
    uniCard_->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "SteamVR keeps one boundary per tracking universe (each base station setup, each tracking system). The one it "
        "tracks in now is marked while SteamVR runs.")));
    auto* uh = new QWidget;
    uniList_ = new QVBoxLayout(uh);
    uniList_->setContentsMargins(0, 0, 0, 0);
    uniList_->setSpacing(6);
    uniCard_->body()->addWidget(uh);
    right->addWidget(uniCard_);

    // ---- saved rooms
    roomsCard_ = new pibridge::ui::Card(QStringLiteral("Saved rooms"));
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);
    scroll->setMinimumHeight(200);
    auto* holder = new QWidget;
    holder->setObjectName(QStringLiteral("card"));
    holder->setStyleSheet(QStringLiteral("QWidget#card { background: transparent; border: none; }"));
    roomList_ = new QVBoxLayout(holder);
    roomList_->setContentsMargins(0, 0, 6, 0);
    roomList_->setSpacing(8);
    scroll->setWidget(holder);
    roomsCard_->body()->addWidget(scroll, 1);
    right->addWidget(roomsCard_, 1);

    connect(&source_, &StateSource::changed, this, [this] {
        const QString act = activeUniverse(source_.state().steamvr);
        if (act != lastActive_) {
            lastActive_ = act;
            refreshLists();
        }
        updateView();
    });
    reload();
}

RoomPage::~RoomPage() = default;

void RoomPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    if (!ed_ || !ed_->dirty) {
        // the same universe again, unless the tracking mode changed meanwhile (inside-out works on universe 30)
        reload(ed_ && insideOut() == loadedInsideOut_ ? ed_->st.universe() : std::string());
    }
}

bool RoomPage::steamvrRunning() const { return source_.state().steamvrRunning; }

bool RoomPage::insideOut() const {
    if (!conf_) {
        return false;
    }
    conf_->reload();
    return ioSettings(*conf_).mode == TrackingMode::InsideOut;
}

QString RoomPage::roommapTool() const { return findTool_ ? findTool_(QStringLiteral("pibridge-roommap")) : QString(); }

QString RoomPage::toolPath() const { return findTool_ ? findTool_(QStringLiteral("pibridge-roomsetup")) : QString(); }

void RoomPage::launchSetup() { mapRoom(false); }

void RoomPage::launchQuickSetup() { mapRoom(true); }

void RoomPage::mapRoom(bool quick) {
    const RingFeed feed = fake_ ? RingFeed{} : readRingFeed();
    const VrBackend backend = currentVrBackend();
    if (backend == VrBackend::Monado && !insideOut()) {
        // Monado + the Lighthouse faceplate: no room map; the room is the backend's Lighthouse stage (centre, forward,
        // the floor in the headset), applied at once (pibridge-roomsetup --lh-stage)
        if (toolPath().isEmpty() || mapRunning_) {
            say(mapRunning_ ? QStringLiteral("The room setup is already running.")
                            : QStringLiteral("pibridge-roomsetup was not found next to pibridge."),
                true);
            return;
        }
        const QString name = conf_ ? roomName(*conf_) : QString::fromLatin1(kDefaultRoom);
        QStringList args{QStringLiteral("--lh-stage"), QStringLiteral("--room"), name};
        if (conf_) {
            if (const auto eh = eyeHeightOf(*conf_)) {
                args << QStringLiteral("--eye-height") << QString::number(*eh, 'f', 3);
            }
        }
        args << vrBackendArgs(backend);
        mapRunning_ = true;
        say(QStringLiteral("Room setup (Monado, Lighthouse) for \"%1\": put the headset on, stand in the middle, look "
                           "forward and follow the steps.").arg(name));
        updateView();
        runner_.run(toolPath(), args, [this](int code, const QString& output) {
            mapRunning_ = false;
            say(code == 0 ? QStringLiteral("Room set up: the centre, forward and the floor are in place.")
                          : QStringLiteral("The room setup ended (code %1). %2").arg(code).arg(output.trimmed().section(QLatin1Char('\n'), -1)),
                code != 0);
            updateView();
        });
        return;
    }
    const RoomGate g = mapRoomHeadsetGate(insideOut(), roommapTool(), toolPath(), feed, mapRunning_);
    if (!g.ok) {
        say(g.why, true);
        updateView();
        return;
    }
    const QString name = conf_ ? roomName(*conf_) : QString::fromLatin1(kDefaultRoom);
    mapRunning_ = true;
    say(QStringLiteral("Setting up the playspace \"%1\": put the headset on. You see your room through the cameras with "
                       "the steps: stand in the middle, look around, then stand still looking forward.").arg(name));
    updateView();
    // The commit is the backend's (core/vr_backend.h): SteamVR -> the chaperone (--chaperone); Monado -> the world offset
    // (pibridge-roommap's finish), so no --chaperone there (pibridge-roomsetup refuses it under Monado).
    const bool chaperone = backend == VrBackend::SteamVr;
    // the floor is left to the room setup (the Pimax ground / the ~1.60 m eye default); no body-height entry any more
    runner_.run(toolPath(), roomMapHeadsetArgs(name, roommapTool(), AnchorFiles::defaults().ring,
                                               std::nullopt, chaperone, quick),
                [this, name, pid = feed.pid](int code, const QString& output) {
        mapRunning_ = false;
        QFile f(roomStatusPath());
        const auto st = f.open(QIODevice::ReadOnly) ? parseRoomStatus(f.readAll()) : std::nullopt;
        const bool mine = st && st->room == name && st->pid == pid;
        if (code == 0) {
            say(mine && !st->text.isEmpty() ? st->text
                                            : QStringLiteral("Playspace \"%1\" set up and put in place: the floor is at 0. "
                                                             "It comes back by itself at every start.").arg(name));
        } else {
            QString why = mine ? st->text : output.trimmed().section(QLatin1Char('\n'), -1);
            if (why.isEmpty()) {
                why = QStringLiteral("pibridge-roommap ended with code %1.").arg(code);
            }
            say(QStringLiteral("The playspace was not set up: %1").arg(why), true);
        }
        updateView();
    });
}


void RoomPage::createInsideOutUniverse() {
    const Gate g = applyGate(steamvrRunning(), store_ != nullptr);
    if (!g.ok || !store_ || store_->files().empty()) {
        say(g.ok ? QStringLiteral("There is no chaperone file to add the inside-out boundary to.") : g.why, true);
        return;
    }
    const QString file = QString::fromStdString(store_->files().front());
    if (auto r = addInsideOutUniverse(file, std::time(nullptr)); !r) {
        say(r.error(), true);
        return;
    }
    reload(kInsideOutUniverse);
    say(QStringLiteral("Universe 30 added to %1 with a 2 x 2 m boundary (backup kept). Drag the corners to your walls "
                       "and apply, or edit it in the headset.").arg(file));
}

void RoomPage::reload(const std::string& universe) {
    ed_.reset();
    zone_.reset();
    store_.reset();
    loadedRoom_.clear();
    const auto dirs = room::steamConfigDirs(env_);
    const bool io = insideOut();
    loadedInsideOut_ = io;
    const std::string want =
        universe.empty() ? roomTargetUniverse(io, activeUniverse(source_.state().steamvr)).toStdString() : universe;
    auto st = room::Store::load(dirs, want);
    monadoMissing_ = false;
    if (st && io && want == kInsideOutUniverse && !st->universes().contains(want)) {
        // never fall back to a Lighthouse universe here: an edit would go there
        monadoMissing_ = true;
        store_ = std::make_unique<room::Store>(std::move(*st));
        loadError_ = QStringLiteral("SteamVR has no inside-out boundary yet (universe 30). It asks for one at the first "
                                    "inside-out start; or create one here (CREATE INSIDE-OUT BOUNDARY) and "
                                    "fit it to your walls.");
    } else if (!st) {
        loadError_ = QString::fromStdString(st.error());
    } else {
        loadError_.clear();
        store_ = std::make_unique<room::Store>(std::move(*st));
        room::RoomLib lib(room::roomsDir(env_));
        const bool pimax = store_->universes().at(store_->universe()).pimax;
        if (const auto m = lib.frameMode(room::defaultFrameMode(pimax)); m != store_->mode()) {
            (void)store_->setMode(m, store_->stagePts());
        }
        zone_ = std::make_unique<room::FileZoneControl>(dirs.front() + "/steamvr.vrsettings", room::roomsDir(env_),
                                                        [] { return std::time(nullptr); });
        room::EditorHost host;
        host.now = steadyNow;
        host.wallClock = [] { return std::time(nullptr); };
        host.afterSave = [](const room::SaveResult&) { return std::string("SteamVR shows it at its next start"); };
        host.zone = zone_.get();
        ed_ = std::make_unique<room::Editor>(*store_, std::move(lib), host);
    }
    refreshLists();
    updateView();
}

void RoomPage::say(const QString& text, bool error) {
    message_->setObjectName(error ? QStringLiteral("error") : QString());
    message_->style()->unpolish(message_);
    message_->style()->polish(message_);
    message_->setText(text);
}

void RoomPage::refreshLists() {
    const auto clear = [](QVBoxLayout* l) {
        while (QLayoutItem* it = l->takeAt(0)) {
            if (it->widget()) {
                it->widget()->hide(); // gone at once, deleted when the event loop runs
                it->widget()->deleteLater();
            }
            delete it;
        }
    };
    clear(uniList_);
    clear(roomList_);
    const QString act = activeUniverse(source_.state().steamvr);
    const QString sel = ed_ && loadedRoom_.isEmpty() ? QString::fromStdString(ed_->st.universe()) : QString();
    const auto boxStyle = [](bool on) {
        return QStringLiteral("QFrame { background: %1; border: 1px solid %2; border-radius: 3px; }"
                              "QLabel { border: none; background: transparent; }")
            .arg(on ? QStringLiteral("#161a33") : QStringLiteral("#111114"),
                 on ? QStringLiteral("#4a5ae8") : QStringLiteral("#26262c"));
    };
    if (!store_) {
        uniCard_->chip()->set(QStringLiteral("NO FILE"), ChipKind::Warn);
        uniList_->addWidget(pibridge::ui::dimLabel(loadError_));
    } else {
        uniCard_->chip()->set(QStringLiteral("%1 IN THE FILE").arg(store_->ids().size()), ChipKind::Off);
        for (const UniverseRow& r : universeRows(*store_, act)) {
            auto* box = new QFrame;
            box->setStyleSheet(boxStyle(r.id == sel));
            box->setCursor(Qt::PointingHandCursor);
            auto* bv = new QVBoxLayout(box);
            bv->setContentsMargins(12, 8, 12, 8);
            bv->setSpacing(2);
            auto* top = new QHBoxLayout;
            auto* name = new QLabel(QStringLiteral("Universe %1").arg(r.id));
            name->setFont(pibridge::ui::bodyFont(15));
            top->addWidget(name);
            top->addStretch(1);
            if (r.active) {
                auto* chip = new pibridge::ui::Chip;
                chip->set(QStringLiteral("STEAMVR USES IT"), ChipKind::Info);
                top->addWidget(chip);
            }
            if (r.pimax) {
                auto* chip = new pibridge::ui::Chip;
                chip->set(QStringLiteral("PIMAX"), ChipKind::Off);
                top->addWidget(chip);
            }
            if (r.monado) {
                auto* chip = new pibridge::ui::Chip;
                chip->set(QStringLiteral("INSIDE-OUT"), ChipKind::Warn);
                top->addWidget(chip);
            }
            bv->addLayout(top);
            auto* d = pibridge::ui::dimLabel(r.detail);
            bv->addWidget(d);
            auto* show = pibridge::ui::linkButton(r.id == sel ? QStringLiteral("Shown") : QStringLiteral("Show"));
            show->setEnabled(r.id != sel);
            bv->addWidget(show, 0, Qt::AlignLeft);
            const QString id = r.id;
            connect(show, &QPushButton::clicked, this, [this, id] { selectUniverse(id); });
            uniList_->addWidget(box);
        }
    }
    room::RoomLib lib(room::roomsDir(env_));
    const auto rows = roomRows(lib);
    roomsCard_->chip()->set(QStringLiteral("%1 SAVED").arg(rows.size()), ChipKind::Off);
    if (rows.empty()) {
        roomList_->addWidget(pibridge::ui::dimLabel(
            QStringLiteral("No saved rooms yet. Save what the floor plan shows with Save as room, or in the headset "
                           "(Saved spaces).")));
    }
    for (const RoomRow& r : rows) {
        auto* box = new QFrame;
        box->setStyleSheet(boxStyle(r.name == loadedRoom_));
        auto* bv = new QVBoxLayout(box);
        bv->setContentsMargins(12, 8, 12, 8);
        bv->setSpacing(2);
        auto* top = new QHBoxLayout;
        auto* name = new QLabel(r.name);
        name->setFont(pibridge::ui::bodyFont(15));
        top->addWidget(name);
        top->addStretch(1);
        if (r.active) {
            auto* chip = new pibridge::ui::Chip;
            chip->set(QStringLiteral("IN USE"), ChipKind::Info);
            top->addWidget(chip);
        }
        bv->addLayout(top);
        bv->addWidget(pibridge::ui::dimLabel(r.detail));
        auto* btns = new QHBoxLayout;
        auto* load = pibridge::ui::linkButton(QStringLiteral("Show"));
        auto* ren = pibridge::ui::linkButton(QStringLiteral("Rename"));
        auto* del = pibridge::ui::linkButton(QStringLiteral("Delete"));
        btns->addWidget(load);
        btns->addWidget(ren);
        btns->addStretch(1);
        btns->addWidget(del);
        bv->addLayout(btns);
        const QString n = r.name;
        connect(load, &QPushButton::clicked, this, [this, n] { loadRoom(n); });
        connect(ren, &QPushButton::clicked, this, [this, n] { renameRoom(n); });
        connect(del, &QPushButton::clicked, this, [this, n] { deleteRoom(n); });
        roomList_->addWidget(box);
    }
    roomList_->addStretch(1);
}

void RoomPage::updateView() {
    const bool running = steamvrRunning();
    const bool io = insideOut();
    // (the "Set up your playspace" card and its running-session status were removed; setup is launched from the Dashboard)
    ioNote_->setText(io ? insideOutRoomNote() : QString());
    ioNote_->setVisible(io);
    createIo_->setVisible(io && monadoMissing_);
    createIo_->setEnabled(!running);
    createIo_->setToolTip(running ? applyGate(true, true).why
                                  : QStringLiteral("Adds universe 30 (the inside-out headset's) with a 2 x 2 m boundary to SteamVR's "
                                                   "chaperone file (a backup first); the Lighthouse universes stay as they are."));
    const Gate ag = applyGate(running, store_ != nullptr);
    apply_->setEnabled(ag.ok && ed_ != nullptr);
    applyWhy_->setText(ag.ok ? QStringLiteral("SteamVR is not running, so the file can be written now. SteamVR shows "
                                              "the new boundary at its next start.")
                             : ag.why);
    revert_->setEnabled(ed_ != nullptr);
    restore_->setEnabled(ed_ != nullptr);
    undo_->setEnabled(ed_ != nullptr && !ed_->undo.empty());
    if (!ed_) {
        plan_->setEditor(nullptr, 0);
        planCard_->chip()->set(QStringLiteral("NO FILE"), ChipKind::Warn);
        planInfo_->setText(loadError_);
        return;
    }
    plan_->setEditor(ed_.get(), ed_->warnDist());
    const QString uni = QString::fromStdString(ed_->st.universe());
    const bool edited = ed_->dirty || ed_->centerDirty || ed_->areaDirty;
    planCard_->chip()->set(edited ? QStringLiteral("NOT APPLIED") : QStringLiteral("AS IN THE FILE"),
                           edited ? ChipKind::Warn : ChipKind::Off);
    bool known = false;
    const double wd = ed_->warnDist(&known);
    planInfo_->setText(
        QStringLiteral("%1universe %2  ·  %3 corners  ·  %4 m²  ·  walls %5 high  ·  warning distance %6%7")
            .arg(loadedRoom_.isEmpty() ? QString() : QStringLiteral("Room \"%1\" on ").arg(loadedRoom_))
            .arg(uni)
            .arg(ed_->pts.size())
            .arg(QLocale(QLocale::English).toString(std::fabs(room::polyArea(ed_->pts)), 'f', 1))
            .arg(metres(ed_->height), metres(wd), known ? QString() : QStringLiteral(" (SteamVR's default)")));
}

void RoomPage::selectUniverse(const QString& id) {
    if (ed_ && (ed_->dirty || ed_->centerDirty || ed_->areaDirty)) {
        QMessageBox ask(QMessageBox::Question, QStringLiteral("Show another universe"),
                        QStringLiteral("The floor plan has changes that are not applied. Show universe %1 and drop them?").arg(id),
                        QMessageBox::Cancel, this);
        auto* yes = ask.addButton(QStringLiteral("Drop changes"), QMessageBox::DestructiveRole);
        ask.exec();
        if (ask.clickedButton() != yes) {
            return;
        }
    }
    reload(id.toStdString());
    say(QStringLiteral("Showing universe %1.").arg(id));
}

void RoomPage::loadRoom(const QString& name) {
    room::RoomLib lib(room::roomsDir(env_));
    const auto r = lib.find(name.toStdString());
    if (!r) {
        say(QStringLiteral("No room called %1.").arg(name), true);
        return;
    }
    if (const Gate g = showRoomGate(insideOut(), QString::fromStdString(r->universe),
                                    store_ && store_->universes().contains(r->universe));
        !g.ok) {
        say(g.why, true);
        return;
    }
    if (!store_ || !store_->universes().contains(r->universe)) {
        // the room's universe is not in the chaperone file: show it on the current one
        if (!ed_) {
            say(QStringLiteral("There is no chaperone file to show the room on."), true);
            return;
        }
    } else if (ed_->st.universe() != r->universe) {
        reload(r->universe);
    }
    ed_->loadSpace(*r);
    ed_->dirty = true;
    loadedRoom_ = name;
    refreshLists();
    updateView();
    say(QStringLiteral("Room \"%1\" shown. Apply to SteamVR to use it.").arg(name));
}

void RoomPage::renameRoom(const QString& name) {
    bool ok = false;
    const QString to = QInputDialog::getText(this, QStringLiteral("Rename room"), QStringLiteral("New name:"),
                                             QLineEdit::Normal, name, &ok);
    if (!ok || to.trimmed().isEmpty() || to == name) {
        return;
    }
    room::RoomLib lib(room::roomsDir(env_));
    if (auto r = lib.rename(name.toStdString(), to.toStdString(), std::time(nullptr)); !r) {
        say(QString::fromStdString(r.error()), true);
        return;
    }
    if (loadedRoom_ == name) {
        loadedRoom_ = to.trimmed();
    }
    refreshLists();
    updateView();
    say(QStringLiteral("Renamed to \"%1\".").arg(to.trimmed()));
}

void RoomPage::deleteRoom(const QString& name) {
    QMessageBox ask(QMessageBox::Warning, QStringLiteral("Delete room"),
                    QStringLiteral("Delete the saved room \"%1\"? SteamVR's boundary is not changed. This cannot be undone.")
                        .arg(name),
                    QMessageBox::Cancel, this);
    auto* yes = ask.addButton(QStringLiteral("Delete"), QMessageBox::DestructiveRole);
    ask.setDefaultButton(QMessageBox::Cancel);
    ask.exec();
    if (ask.clickedButton() != yes) {
        return;
    }
    room::RoomLib lib(room::roomsDir(env_));
    if (auto r = lib.remove(name.toStdString()); !r) {
        say(QString::fromStdString(r.error()), true);
        return;
    }
    if (loadedRoom_ == name) {
        loadedRoom_.clear();
    }
    refreshLists();
    say(QStringLiteral("Room \"%1\" deleted.").arg(name));
}

void RoomPage::saveAsRoom() {
    if (!ed_) {
        return;
    }
    bool ok = false;
    const QString name = QInputDialog::getText(this, QStringLiteral("Save as room"),
                                               QStringLiteral("Name (for example \"Door open\"):"), QLineEdit::Normal,
                                               loadedRoom_, &ok);
    if (!ok || name.trimmed().isEmpty()) {
        return;
    }
    if (const Gate g = saveRoomGate(ed_->lib, name, QString::fromStdString(ed_->st.universe())); !g.ok) {
        say(g.why, true);
        return;
    }
    if (auto r = ed_->saveAsNew(name.trimmed().toStdString()); !r) {
        say(QString::fromStdString(r.error()), true);
        return;
    }
    loadedRoom_ = name.trimmed();
    refreshLists();
    updateView();
    say(QStringLiteral("Saved as \"%1\".").arg(name.trimmed()));
}

void RoomPage::apply() {
    if (!ed_) {
        return;
    }
    const Gate g = applyGate(steamvrRunning(), store_ != nullptr);
    if (!g.ok) {
        say(g.why, true);
        return;
    }
    if (!room::isSimple(ed_->pts)) {
        say(QStringLiteral("The boundary crosses itself or has corners on top of each other: fix it first."), true);
        return;
    }
    if (ed_->save()) {
        if (!loadedRoom_.isEmpty()) {
            ed_->lib.setActive(loadedRoom_.toStdString());
        }
        refreshLists();
        say(QString::fromStdString(ed_->status()));
    } else {
        say(QString::fromStdString(ed_->status()), true);
    }
    updateView();
}

} // namespace pibridge::app
