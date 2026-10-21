// SPDX-License-Identifier: MIT
#include "performance.h"

#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QVBoxLayout>
#include <cmath>

#include "../core/chovr.h"
#include "../core/presets.h"
#include "../qv/fovmath.h"
#include "../qv/preview.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

QString settingsFile() {
    return QString::fromStdString(qv::settingsPath().string());
}

QString mp(uint64_t px) {
    return QString::number(double(px) / 1e6, 'f', 1) + QStringLiteral(" MP");
}

QLabel* section(const QString& text) {
    auto* l = new QLabel(text);
    QFont f = pibridge::ui::bodyFont(17);
    f.setWeight(QFont::Medium);
    f.setCapitalization(QFont::AllUppercase);
    l->setFont(f);
    l->setContentsMargins(0, 8, 0, 0);
    return l;
}

// The recommended per-eye size for the pixel budget until a game reports the headset's own.
constexpr qv::Size kDefaultRec{3552, 3840};

// ---- CHOVR settings.json helpers (same shape as the Display page's, kept local to this file).
QJsonObject readChovr() {
    QFile f(chovrSettingsFile());
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    return QJsonDocument::fromJson(f.readAll()).object();
}

void writeChovr(const QJsonObject& root) {
    const QString path = chovrSettingsFile();
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile f(path);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
        f.commit();
    }
}

double numOr(const QJsonObject& o, const QString& key, double def) {
    return o.contains(key) && o.value(key).isDouble() ? o.value(key).toDouble() : def;
}

} // namespace

PerformancePage::PerformancePage(QWidget* parent) : QWidget(parent) {
    setObjectName(QStringLiteral("page"));
    saveTimer_.setSingleShot(true);
    saveTimer_.setInterval(300);
    connect(&saveTimer_, &QTimer::timeout, this, &PerformancePage::save);
    statusTimer_.setInterval(1000);
    connect(&statusTimer_, &QTimer::timeout, this, &PerformancePage::refreshStatus);

    // One scroll area holds the whole page, so the quad views settings and the preview scroll together as one.
    auto* pageLayout = new QVBoxLayout(this);
    pageLayout->setContentsMargins(0, 0, 0, 0);
    auto* pageScroll = new QScrollArea;
    pageScroll->setWidgetResizable(true);
    pageScroll->setFrameShape(QFrame::NoFrame);
    pageLayout->addWidget(pageScroll);
    auto* holder = new QWidget;
    holder->setObjectName(QStringLiteral("page"));
    pageScroll->setWidget(holder);
    auto* root = new QVBoxLayout(holder);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);

    // ---- banner: profile, restart note, restore default
    auto* banner = new QFrame;
    banner->setObjectName(QStringLiteral("banner"));
    auto* bh = new QHBoxLayout(banner);
    bh->setContentsMargins(16, 10, 16, 10);
    bh->setSpacing(10);
    auto* bv = new QVBoxLayout;
    auto* prow = new QHBoxLayout;
    prow->setSpacing(8);
    prow->addWidget(pibridge::ui::capsLabel(QStringLiteral("Settings for:"), 13));
    const QString profileTip = QStringLiteral(
        "\"All games\" is the base; a game profile overrides it for the games whose OpenXR name (app:) or process "
        "name (exe:) contains the text.");
    profile_ = new QComboBox;
    profile_->setMinimumWidth(240);
    profile_->setToolTip(profileTip);
    prow->addWidget(pibridge::ui::helpBadge(profileTip));
    prow->addWidget(profile_);
    auto* add = pibridge::ui::toolButton(QStringLiteral("Add game..."));
    removeProfile_ = pibridge::ui::toolButton(QStringLiteral("Remove"));
    prow->addWidget(add);
    prow->addWidget(removeProfile_);
    prow->addStretch(1);
    bv->addLayout(prow);
    auto* note = new QHBoxLayout;
    note->setSpacing(8);
    note->addWidget(new pibridge::ui::RestartDot);
    note->addWidget(new QLabel(QStringLiteral("Settings marked with a dot need the game to be restarted. "
                                              "The others act on a running game within a second.")),
                    1);
    bv->addLayout(note);
    bh->addLayout(bv, 1);
    auto* restore = pibridge::ui::toolButton(QStringLiteral("Restore default"));
    bh->addWidget(restore, 0, Qt::AlignVCenter);
    root->addWidget(banner);

    auto* heading = new QLabel(QStringLiteral("Performance & rendering"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);

    // ---- CHOVR render settings card (CustomHeadsetOpenVR, ~/.config/CustomHeadset/settings.json).
    // These are the headset display driver's render settings. Like the quad views layer below, this card uses an
    // in-memory working copy: editing a control changes only chovrRoot_ and marks the page dirty, and nothing is written
    // until Save (Revert reloads from disk). Save / Revert here govern ONLY these settings.json fields; the quad views
    // layer has its own Save / Revert.
    auto* chovr = new pibridge::ui::Card(QStringLiteral("Render & display performance"));
    chovr->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "Render settings of the headset display driver (CustomHeadsetOpenVR). Changes are saved with Save and undone "
        "with Revert; marked settings take effect the next time the VR app starts.")));

    // Render resolution: generalHeadset.renderResolutionMultiplierX/Y (1.0 = 100 %, CHOVR's default). One slider sets
    // both axes, like a single quality control.
    renderQuality_ = new pibridge::ui::SliderRow(
        QStringLiteral("Render resolution"), 50, 200, QStringLiteral("%"), true,
        QStringLiteral("Multiplies the driver's 100 % render resolution on both axes. Below 100 % is faster, above is "
                       "sharper (supersampling) and heavier on the GPU."));
    chovr->body()->addWidget(renderQuality_);
    connect(renderQuality_, &pibridge::ui::SliderRow::valueChanged, this, [this] { chovrEdited(); });

    // Hidden area mask: generalHeadset.hiddenArea.enable. ON tells the compositor to skip the edge pixels the lenses
    // never show, saving GPU. CHOVR's default is off.
    hiddenArea_ = new pibridge::ui::Toggle;
    const QString hiddenAreaTip = QStringLiteral(
        "Skips rendering the edge pixels that the lenses hide, which saves GPU time. "
        "Takes effect when the game or SteamVR restarts.");
    hiddenArea_->setToolTip(hiddenAreaTip);
    chovr->body()->addWidget(
        pibridge::ui::settingRow(QStringLiteral("Hide hidden area mask"), hiddenArea_, nullptr, true, hiddenAreaTip));
    connect(hiddenArea_, &QAbstractButton::toggled, this, [this] { chovrEdited(); });

    // Pimax's "Center Rendering" (fixed foveated: full-res centre, low-res edges, no eye tracking) has no
    // CustomHeadsetOpenVR settings.json key on Linux; the equivalent is the Quadview foveated layer below in FIXED mode,
    // so there is no separate control here.

    // OK / Cancel for the CHOVR fields only (the quad views layer below keeps its own instant-apply behaviour).
    auto* chovrActs = new QHBoxLayout;
    chovrActs->setSpacing(6);
    chovrOk_ = pibridge::ui::accentButton(QStringLiteral("Save"));
    chovrOk_->setToolTip(QStringLiteral("Saves these render settings to the display driver's settings.json."));
    chovrCancel_ = pibridge::ui::toolButton(QStringLiteral("Revert"));
    chovrCancel_->setToolTip(QStringLiteral("Discards the changes and reloads the saved settings."));
    chovrActs->addStretch(1);
    chovrActs->addWidget(chovrOk_);
    chovrActs->addWidget(chovrCancel_);
    chovr->body()->addLayout(chovrActs);
    root->addWidget(chovr);
    connect(chovrOk_, &QPushButton::clicked, this, &PerformancePage::applyChovr);
    connect(chovrCancel_, &QPushButton::clicked, this, [this] { loadChovr(); });

    // ---- the card
    auto* card = new QFrame;
    card->setObjectName(QStringLiteral("card"));
    auto* cv = new QVBoxLayout(card);
    cv->setContentsMargins(16, 12, 16, 12);
    cv->setSpacing(8);
    auto* ch = new QHBoxLayout;
    auto* title = new QLabel(QStringLiteral("Quadview foveated rendering"));
    QFont tf = pibridge::ui::bodyFont(18);
    tf.setWeight(QFont::Medium);
    tf.setCapitalization(QFont::AllUppercase);
    title->setFont(tf);
    ch->addWidget(title);
    ch->addWidget(new pibridge::ui::RestartDot);
    ch->addSpacing(12);
    installText_ = new QLabel;
    installText_->setObjectName(QStringLiteral("dim"));
    installText_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ch->addWidget(installText_, 1);
    enabled_ = new pibridge::ui::Toggle;
    const QString enabledTip = QStringLiteral(
        "The quad views layer on or off. Games that ask for quad views get them from this layer; off, the OpenXR "
        "loader does not load it at all. Takes effect when a game starts.");
    enabled_->setToolTip(enabledTip);
    ch->addWidget(pibridge::ui::helpBadge(enabledTip));
    ch->addWidget(enabled_);
    cv->addLayout(ch);
    cv->addWidget(pibridge::ui::rule());

    auto* body = new QHBoxLayout;
    body->setSpacing(18);
    cv->addLayout(body, 1);
    root->addWidget(card);
    root->addStretch(1);

    // left: settings (no inner scroll — the whole page scrolls as one with the preview)
    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    lv->setContentsMargins(0, 0, 10, 0);
    lv->setSpacing(8);
    body->addWidget(left, 3);

    {
        auto* ph = new QHBoxLayout;
        ph->setSpacing(8);
        ph->addWidget(section(QStringLiteral("Quadview preset")));
        ph->addWidget(pibridge::ui::helpBadge(
            QStringLiteral("A preset sets the two resolutions and the gaze area size. CUSTOM: your own values.")));
        ph->addStretch(1);
        lv->addLayout(ph);
    }
    QStringList presetNames{QStringLiteral("Custom")};
    for (const auto& p : kQvPresets) {
        presetNames << QString::fromLatin1(p.name);
    }
    preset_ = new pibridge::ui::Segmented(presetNames);
    preset_->setToolTip(QStringLiteral("A preset sets the two resolutions and the gaze area size. CUSTOM: your own "
                                       "values."));
    lv->addWidget(preset_);
    connect(preset_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        if (i == 0) {
            preset_->setCurrent(matchPreset(s_) + 1); // CUSTOM is not a choice, it is what is left
            return;
        }
        applyPreset(s_, i - 1);
        showSettings();
        changed();
    });

    lv->addWidget(section(QStringLiteral("Resolution")));
    percent(QStringLiteral("Center resolution"), 50, 200, true,
            QStringLiteral("Pixels of the focus views (where you look) per pixel the runtime recommends. Above 100 % "
                           "is supersampling where you look."),
            [](qv::Settings& s) -> float& { return s.focusDensity; }, lv);
    percent(QStringLiteral("Edge resolution"), 5, 150, true,
            QStringLiteral("Pixels of the wide views (the whole field) per recommended pixel. 20 % = a fifth in each "
                           "direction."),
            [](qv::Settings& s) -> float& { return s.peripheralDensity; }, lv);

    {
        // Pimax's "Center Rendering" (fixed foveated) has no CHOVR key on Linux; its equivalent is this layer in
        // Fixed mode, so the explanation lives as a "?" badge on the Focus area header (see the note above).
        auto* fh = new QHBoxLayout;
        fh->setSpacing(8);
        fh->addWidget(section(QStringLiteral("Focus area")));
        fh->addWidget(pibridge::ui::helpBadge(QStringLiteral(
            "Pimax's \"Center Rendering\" (fixed foveated: full-res centre, low-res edges, no eye tracking) is not a "
            "CustomHeadsetOpenVR setting on Linux. Its equivalent is this quad views layer with the focus area set to "
            "Fixed.")));
        fh->addStretch(1);
        lv->addLayout(fh);
    }
    tracking_ = new pibridge::ui::Segmented({QStringLiteral("Follow the eyes"), QStringLiteral("Fixed")});
    tracking_->setToolTip(QStringLiteral("Follow the eyes: the focus moves with the gaze when the headset tracks "
                                         "the eyes, else it stays in the fixed area."));
    lv->addWidget(tracking_);
    connect(tracking_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        s_.tracking = i == 0 ? qv::Tracking::Eye : qv::Tracking::Fixed;
        changed();
    });
    refreshers_.push_back([this] { tracking_->setCurrent(s_.tracking == qv::Tracking::Eye ? 0 : 1); });
    percent(QStringLiteral("Gaze area width"), 10, 90, false,
            QStringLiteral("Share of the eye's field that follows the gaze."),
            [](qv::Settings& s) -> float& { return s.gazeSize[0]; }, lv);
    percent(QStringLiteral("Gaze area height"), 10, 90, false,
            QStringLiteral("Share of the eye's field that follows the gaze."),
            [](qv::Settings& s) -> float& { return s.gazeSize[1]; }, lv);
    percent(QStringLiteral("Fixed area width"), 10, 90, false,
            QStringLiteral("Share of the eye's field without eye tracking (larger: the eyes move)."),
            [](qv::Settings& s) -> float& { return s.fixedSize[0]; }, lv);
    percent(QStringLiteral("Fixed area height"), 10, 90, false,
            QStringLiteral("Share of the eye's field without eye tracking."),
            [](qv::Settings& s) -> float& { return s.fixedSize[1]; }, lv);
    {
        auto* r = new pibridge::ui::SliderRow(QStringLiteral("Vertical offset"), -30, 30, QStringLiteral("°"), false,
                                      QStringLiteral("Move the focus up (+) or down (-), degrees, both eyes. Cockpits "
                                                     "often want it a little lower."));
        lv->addWidget(r);
        connect(r, &pibridge::ui::SliderRow::valueChanged, this, [this](int v) {
            s_.offsetDeg[1] = float(v);
            changed();
        });
        refreshers_.push_back([this, r] { r->setValueQuiet(int(std::lround(s_.offsetDeg[1]))); });
    }

    lv->addWidget(section(QStringLiteral("Transition")));
    percent(QStringLiteral("Blend band"), 0, 50, false,
            QStringLiteral("Width of the soft edge of the focus, share of its size on each side. 0 = hard edge."),
            [](qv::Settings& s) -> float& { return s.transition; }, lv);
    percent(QStringLiteral("Edge opacity"), 0, 100, false,
            QStringLiteral("How much of the focus is already there at the outer edge of the blend band. 0 = fades in "
                           "from the edge view, 50 % = starts half way, 100 % = hard edge."),
            [](qv::Settings& s) -> float& { return s.edgeOpacity; }, lv);

    lv->addWidget(section(QStringLiteral("Image")));
    percent(QStringLiteral("Sharpen"), 0, 100, false,
            QStringLiteral("Contrast-adaptive sharpening of the focus view (0 = off)."),
            [](qv::Settings& s) -> float& { return s.sharpen; }, lv);
    filter_ = new pibridge::ui::Segmented({QStringLiteral("Bicubic"), QStringLiteral("Bilinear")});
    const QString filterTip = QStringLiteral("How the wide views are upscaled: bicubic is sharper, bilinear cheaper.");
    filter_->setToolTip(filterTip);
    lv->addWidget(pibridge::ui::settingRow(QStringLiteral("Filter"), filter_, nullptr, false, filterTip));
    connect(filter_, &pibridge::ui::Segmented::activated, this, [this](int i) {
        s_.peripheralFilter = i == 0 ? qv::Filter::Bicubic : qv::Filter::Bilinear;
        changed();
    });
    refreshers_.push_back([this] { filter_->setCurrent(s_.peripheralFilter == qv::Filter::Bicubic ? 0 : 1); });
    gazeFilter_ = new pibridge::ui::Toggle;
    const QString gazeTip = QStringLiteral("One-Euro smoothing of the gaze; it jumps at once on saccades.");
    gazeFilter_->setToolTip(gazeTip);
    lv->addWidget(pibridge::ui::settingRow(QStringLiteral("Smooth the gaze"), gazeFilter_, nullptr, false, gazeTip));
    connect(gazeFilter_, &QAbstractButton::toggled, this, [this](bool v) {
        s_.gazeFilter = v;
        changed();
    });
    refreshers_.push_back([this] { gazeFilter_->setCheckedQuiet(s_.gazeFilter); });
    debug_ = new pibridge::ui::Toggle;
    const QString debugTip = QStringLiteral("Tints the focus area, outlines the blend band and marks the gaze in the "
                                            "headset: for tuning.");
    debug_->setToolTip(debugTip);
    lv->addWidget(pibridge::ui::settingRow(QStringLiteral("Debug overlay"), debug_, nullptr, false, debugTip));
    connect(debug_, &QAbstractButton::toggled, this, [this](bool v) {
        s_.debugOverlay = v;
        changed();
    });
    refreshers_.push_back([this] { debug_->setCheckedQuiet(s_.debugOverlay); });
    lv->addStretch(1);

    // right: preview, budget, last game
    auto* right = new QVBoxLayout;
    right->setSpacing(8);
    body->addLayout(right, 2);
    right->addWidget(pibridge::ui::capsLabel(QStringLiteral("Quadview preview"), 14));
    preview_ = new qvgui::Preview;
    preview_->setMinimumHeight(300); // the whole page scrolls now, so the preview needs its own size
    right->addWidget(preview_, 1);
    budget_ = pibridge::ui::dimLabel(QString());
    right->addWidget(budget_);
    right->addWidget(pibridge::ui::rule());
    right->addWidget(pibridge::ui::capsLabel(QStringLiteral("Last game"), 14));
    status_ = pibridge::ui::dimLabel(QString());
    status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    right->addWidget(status_);
    saved_ = pibridge::ui::dimLabel(QString());
    right->addWidget(saved_);
    right->addStretch(1);
    // Save / Revert for the quad views settings (buffered), centered at the bottom of the whole card.
    qvSave_ = pibridge::ui::accentButton(QStringLiteral("Save"));
    qvSave_->setToolTip(QStringLiteral("Writes the quad views settings; the layer applies them to a running game within a second."));
    qvRevert_ = pibridge::ui::toolButton(QStringLiteral("Revert"));
    qvRevert_->setToolTip(QStringLiteral("Discards the changes and reloads the saved quad views settings."));
    auto* qvActs = new QHBoxLayout;
    qvActs->setSpacing(6);
    qvActs->addStretch(1);
    qvActs->addWidget(qvSave_);
    qvActs->addWidget(qvRevert_);
    qvActs->addStretch(1);
    cv->addWidget(pibridge::ui::rule());
    cv->addLayout(qvActs);
    connect(qvSave_, &QPushButton::clicked, this, [this] {
        save();
        qvDirty_ = false;
        updateQvButtons();
    });
    connect(qvRevert_, &QPushButton::clicked, this, [this] {
        loadProfile(); // re-reads the file into s_ and repopulates the controls
        qvDirty_ = false;
        updateQvButtons();
        saved_->setText(QStringLiteral("Reverted"));
    });
    qvSave_->setEnabled(false);
    qvRevert_->setEnabled(false);

    // ---- wiring
    connect(enabled_, &QAbstractButton::toggled, this, [this](bool on) {
        QString err;
        if (!qvgui::setEnabled(install_, on, err)) {
            QMessageBox::warning(this, QStringLiteral("Quad views"), err);
        }
        refreshStatus();
    });
    connect(profile_, &QComboBox::currentIndexChanged, this, [this] {
        if (!loading_) {
            loadProfile();
        }
    });
    connect(add, &QPushButton::clicked, this, [this] {
        const qvgui::Status st = qvgui::readStatus();
        const QString last = st.values.value(QStringLiteral("app"));
        bool ok = false;
        const QString name = QInputDialog::getText(
            this, QStringLiteral("Game profile"),
            QStringLiteral("app:TEXT matches the game's OpenXR application name, exe:TEXT its process name.\n"
                           "The last game was \"%1\" (process %2).")
                .arg(last, st.values.value(QStringLiteral("exe"))),
            QLineEdit::Normal, QStringLiteral("app:") + last, &ok);
        if (!ok || !(name.startsWith(QLatin1String("app:")) || name.startsWith(QLatin1String("exe:"))) ||
            name.size() < 5) {
            return;
        }
        writeFileText(QString::fromStdString(qv::replaceSection(fileText().toStdString(), name.toStdString(), s_)));
        refreshProfiles();
        profile_->setCurrentIndex(profile_->findData(name));
    });
    connect(removeProfile_, &QPushButton::clicked, this, [this] {
        const QString sec = currentSection();
        if (sec.isEmpty()) {
            return;
        }
        writeFileText(QString::fromStdString(qv::removeSection(fileText().toStdString(), sec.toStdString())));
        refreshProfiles();
        profile_->setCurrentIndex(0);
        loadProfile();
    });
    connect(restore, &QPushButton::clicked, this, [this] {
        s_ = qv::Settings{};
        showSettings();
        changed();
    });

    refreshProfiles();
    loadProfile();
    loadChovr();
    refreshStatus();
}

pibridge::ui::SliderRow* PerformancePage::percent(const QString& label, int lo, int hi, bool restartDot, const QString& tip,
                                          std::function<float&(qv::Settings&)> field, QVBoxLayout* into) {
    auto* r = new pibridge::ui::SliderRow(label, lo, hi, QStringLiteral("%"), restartDot, tip);
    into->addWidget(r);
    connect(r, &pibridge::ui::SliderRow::valueChanged, this, [this, field](int v) {
        field(s_) = float(v) / 100.f;
        changed();
    });
    refreshers_.push_back([this, r, field] { r->setValueQuiet(int(std::lround(field(s_) * 100.f))); });
    return r;
}

void PerformancePage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    loadChovr(); // reload the CHOVR working copy from disk every time the page is shown
    refreshStatus();
    statusTimer_.start();
}

void PerformancePage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    statusTimer_.stop();
}

QString PerformancePage::fileText() const {
    QFile f(settingsFile());
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }
    return QString::fromUtf8(f.readAll());
}

bool PerformancePage::writeFileText(const QString& text) {
    const QString path = settingsFile();
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile f(path); // temp file + rename: the layer never reads half a file
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    f.write(text.toUtf8());
    return f.commit();
}

QString PerformancePage::currentSection() const {
    return profile_->currentData().toString();
}

void PerformancePage::refreshProfiles() {
    loading_ = true;
    const QString keep = currentSection();
    profile_->clear();
    profile_->addItem(QStringLiteral("All games"), QString());
    for (const auto& n : qv::sectionNames(fileText().toStdString())) {
        const QString q = QString::fromStdString(n);
        profile_->addItem(q.startsWith(QLatin1String("app:")) ? QStringLiteral("Game \"%1\"").arg(q.mid(4))
                                                              : QStringLiteral("Process \"%1\"").arg(q.mid(4)),
                          q);
    }
    const int i = profile_->findData(keep);
    profile_->setCurrentIndex(i < 0 ? 0 : i);
    loading_ = false;
}

void PerformancePage::loadProfile() {
    const std::string text = fileText().toStdString();
    const QString sec = currentSection();
    s_ = sec.isEmpty() ? qv::parseSettings(text, qv::AppId{"\x01", "\x01"}) // no section matches
                       : qv::parseSection(text, sec.toStdString());
    removeProfile_->setEnabled(!sec.isEmpty());
    showSettings();
}

void PerformancePage::showSettings() {
    loading_ = true;
    for (auto& r : refreshers_) {
        r();
    }
    preset_->setCurrent(matchPreset(s_) + 1);
    loading_ = false;
    preview_->setSettings(s_);
    updateBudget();
}

void PerformancePage::changed() {
    if (loading_) {
        return;
    }
    preset_->setCurrent(matchPreset(s_) + 1);
    preview_->setSettings(s_); // the preview follows the in-memory edits; the file is written only on Save
    updateBudget();
    qvDirty_ = true;
    updateQvButtons();
}

void PerformancePage::updateQvButtons() {
    if (qvSave_ != nullptr) {
        qvSave_->setEnabled(qvDirty_);
        qvRevert_->setEnabled(qvDirty_);
    }
    if (qvDirty_) {
        saved_->setText(QStringLiteral("Unsaved changes"));
    }
    // when clean, leave saved_ to whatever save()/Revert set, so the "Saved" / "Reverted" message stays visible
}

void PerformancePage::save() {
    const std::string text = fileText().toStdString();
    const QString sec = currentSection();
    const std::string out =
        sec.isEmpty() ? qv::replaceGlobal(text, s_) : qv::replaceSection(text, sec.toStdString(), s_);
    saved_->setText(writeFileText(QString::fromStdString(out)) ? QStringLiteral("Saved to %1").arg(settingsFile())
                                                                : QStringLiteral("Could not write %1").arg(settingsFile()));
}

void PerformancePage::loadChovr() {
    chovrLoading_ = true;
    chovrRoot_ = readChovr();
    const QJsonObject gen = chovrRoot_.value(QStringLiteral("generalHeadset")).toObject();
    const double mult = numOr(gen, QStringLiteral("renderResolutionMultiplierX"), 1.0); // X and Y kept in step
    renderQuality_->setValueQuiet(static_cast<int>(std::lround(mult * 100.0)));
    const QJsonObject hidden = gen.value(QStringLiteral("hiddenArea")).toObject();
    hiddenArea_->setCheckedQuiet(hidden.value(QStringLiteral("enable")).toBool(false));
    chovrLoading_ = false;
    setChovrDirty(false);
}

void PerformancePage::chovrEdited() {
    if (chovrLoading_) {
        return; // populating the controls, not a user edit
    }
    // Update the in-memory working copy only; settings.json is written on OK.
    QJsonObject gen = chovrRoot_.value(QStringLiteral("generalHeadset")).toObject();
    const double mult = renderQuality_->value() / 100.0;
    gen[QStringLiteral("renderResolutionMultiplierX")] = mult;
    gen[QStringLiteral("renderResolutionMultiplierY")] = mult;
    QJsonObject hidden = gen.value(QStringLiteral("hiddenArea")).toObject();
    hidden[QStringLiteral("enable")] = hiddenArea_->isChecked();
    gen[QStringLiteral("hiddenArea")] = hidden;
    chovrRoot_[QStringLiteral("generalHeadset")] = gen;
    setChovrDirty(true);
}

void PerformancePage::applyChovr() {
    writeChovr(chovrRoot_);
    setChovrDirty(false);
}

void PerformancePage::setChovrDirty(bool dirty) {
    chovrDirty_ = dirty;
    chovrOk_->setEnabled(dirty);
    chovrCancel_->setEnabled(dirty);
}

void PerformancePage::updateBudget() {
    qv::Size rec = kDefaultRec;
    const QStringList r = qvgui::readStatus().values.value(QStringLiteral("recommended")).split(QLatin1Char('x'));
    if (r.size() == 2 && r[0].toInt() > 0 && r[1].toInt() > 0) {
        rec = {uint32_t(r[0].toInt()), uint32_t(r[1].toInt())};
    }
    const qv::Budget g = qv::budget(rec, s_, true), f = qv::budget(rec, s_, false);
    budget_->setText(QStringLiteral("Pixels per frame at %1 x %2 per eye: plain stereo %3; with eye tracking %4 "
                                    "(%5 % fewer); fixed %6 (%7 % fewer).")
                         .arg(rec.w)
                         .arg(rec.h)
                         .arg(mp(g.stereoPixels), mp(g.quadPixels))
                         .arg(int(std::lround(g.saving * 100)))
                         .arg(mp(f.quadPixels))
                         .arg(int(std::lround(f.saving * 100))));
}

void PerformancePage::refreshStatus() {
    install_ = qvgui::findInstall();
    enabled_->setEnabled(install_.userInstalled);
    enabled_->setCheckedQuiet(install_.userInstalled ? install_.userEnabled : !install_.systemManifest.isEmpty());
    if (install_.userInstalled) {
        installText_->setText(install_.userEnabled ? QStringLiteral("Layer on") : QStringLiteral("Layer off"));
        installText_->setToolTip(install_.userManifest);
    } else if (!install_.systemManifest.isEmpty()) {
        installText_->setText(QStringLiteral("Installed for all users: the switch works only for a user install."));
        installText_->setToolTip(install_.systemManifest);
    } else {
        installText_->setText(QStringLiteral("The quad views layer is not installed."));
        installText_->setToolTip({});
    }

    const qvgui::Status st = qvgui::readStatus();
    if (!st.valid()) {
        status_->setText(QStringLiteral("No game has used the layer yet."));
        return;
    }
    const auto v = [&](const char* k) { return st.values.value(QString::fromLatin1(k)).toHtmlEscaped(); };
    const qint64 age = st.written.secsTo(QDateTime::currentDateTime());
    const bool live = age < 5 && (v("session") == QLatin1String("quad") || v("session") == QLatin1String("stereo"));
    const QString session = v("session") == QLatin1String("quad")     ? QStringLiteral("quad views")
                            : v("session") == QLatin1String("stereo") ? QStringLiteral("stereo (no quad views asked)")
                                                                     : QStringLiteral("ended");
    const QString eye = v("eye_tracking") == QLatin1String("off") ? QStringLiteral("off (fixed area)")
                        : !live                                   ? QStringLiteral("available")
                        : v("eye_tracking") == QLatin1String("live") ? QStringLiteral("following the gaze")
                                                                    : QStringLiteral("no gaze now (fixed area)");
    const QString when = live ? QStringLiteral("running")
                              : age < 120 ? QStringLiteral("%1 s ago").arg(age)
                                          : QStringLiteral("%1 min ago").arg(age / 60);
    status_->setText(QStringLiteral("<b style='color:#e6e6ec'>%1</b> (%2)<br>Session: %3<br>Eye tracking: %4<br>"
                                    "Edge %5, center %6 per eye<br>Composition: %7 ms GPU")
                         .arg(v("app"), when, session, eye, v("peripheral"), v("focus"))
                         .arg(QString::number(st.values.value(QStringLiteral("compose_gpu_ms")).toDouble(), 'f', 2)));
}

} // namespace pibridge::app
