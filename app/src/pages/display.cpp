// SPDX-License-Identifier: MIT
#include "display.h"

#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QSaveFile>
#include <QVBoxLayout>
#include <algorithm>

#include "../core/chovr.h"
#include "../core/rec_setup.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

namespace {

QJsonObject readSettings() {
    QFile f(chovrSettingsFile());
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    return QJsonDocument::fromJson(f.readAll()).object();
}

void writeSettings(const QJsonObject& root) {
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

DisplayPage::DisplayPage(StateSource& source, Conf& conf, Actions& actions, QWidget* parent)
    : QWidget(parent), source_(source), conf_(conf), actions_(actions) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* heading = new QLabel(QStringLiteral("Display"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);

    auto* card = new pibridge::ui::Card(QStringLiteral("Display & visuals"));
    card->body()->addWidget(pibridge::ui::dimLabel(QStringLiteral(
        "Brightness, contrast, black level, gamma and color applied by the headset driver "
        "(CustomHeadsetOpenVR). They work the same whether tracking is inside-out (Monado) or Lighthouse.")));
    info_ = pibridge::ui::dimLabel(QString());
    card->body()->addWidget(info_);

    // Panel refresh rate (refresh_hz): a headset setting, applied live by the headset service (not a CHOVR shader
    // setting). It also has a quick toggle on the Dashboard headset card; both write the same setting.
    refresh_ = new pibridge::ui::Segmented({QStringLiteral("72 Hz"), QStringLiteral("90 Hz")});
    connect(refresh_, &pibridge::ui::Segmented::activated, this,
            [this](int i) { actions_.setRefreshHz(i == 0 ? 72 : 90, {}); });
    card->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Refresh rate"), refresh_, nullptr, false,
        QStringLiteral("The panel refresh rate the headset runs at (72 or 90 Hz). Takes effect the next time the "
                       "headset wakes.")));

    brightness_ = new pibridge::ui::SliderRow(QStringLiteral("Brightness"), 20, 200, QStringLiteral("%"), true,
                                              QStringLiteral("Scales the rendered image (software). 100% leaves it as is."));
    contrast_ = new pibridge::ui::SliderRow(QStringLiteral("Contrast"), -50, 50, QStringLiteral(""), true,
                                            QStringLiteral("Raises or lowers contrast in the driver's shader. 0 leaves it as is."));
    // Black level (generalHeadset.blackLevel, 0..1 in CHOVR): lifts the darkest pixels off true black to hide
    // smearing on the OLED panels. 0 % leaves black as is.
    blackLevel_ = new pibridge::ui::SliderRow(QStringLiteral("Black level"), 0, 100, QStringLiteral("%"), true,
                                              QStringLiteral("Raises the minimum black the panel shows. 0% is true black; "
                                                             "a little helps on OLED panels that smear in the dark."));
    // Gamma (customShader.gamma): CHOVR's default is 2.2, shown here as gamma x 100 (220 = 2.2) since the slider is
    // whole numbers. Lower is brighter mid tones.
    gamma_ = new pibridge::ui::SliderRow(QStringLiteral("Gamma"), 100, 300, QStringLiteral(""), true,
                                         QStringLiteral("Output gamma x 100 (220 = the 2.2 default). Lower lifts the mid "
                                                        "tones (brighter), higher deepens them."));
    // Colour saturation (customShader.saturation): CHOVR uses 50 as neutral (saturation/50 in the shader).
    saturation_ = new pibridge::ui::SliderRow(QStringLiteral("Color saturation"), 0, 100, QStringLiteral(""), true,
                                              QStringLiteral("Strength of the colors. 50 leaves them as is, 0 is grayscale, "
                                                             "100 is the most vivid."));
    card->body()->addWidget(brightness_);
    card->body()->addWidget(contrast_);
    card->body()->addWidget(blackLevel_);
    card->body()->addWidget(gamma_);
    card->body()->addWidget(saturation_);
    root->addWidget(card);

    // Lens profile (pibridge.conf lens_profile): which lens file the Pimax (Monado) driver renders through,
    // resolved in core/rec_setup.cpp. Inside-out only; in Lighthouse CHOVR draws with its own profile.
    auto* lensCard = new pibridge::ui::Card(QStringLiteral("Lens"));
    lens_ = new QComboBox;
    lens_->setMinimumWidth(320);
    lens_->addItem(QStringLiteral("Crystal Super Oled chovr"), QStringLiteral("default"));
    lens_->addItem(QStringLiteral("Crystal Super Oled native"), QStringLiteral("native"));
    connect(lens_, &QComboBox::activated, this, [this](int i) {
        conf_.set(reckey::kLensProfile, lens_->itemData(i).toString().toStdString());
        loadLens();
    });
    lensCard->body()->addWidget(pibridge::ui::settingRow(
        QStringLiteral("Lens profile"), lens_, nullptr, true,
        QStringLiteral("The distortion and field of view the headset renders through in inside-out (Monado). "
                       "chovr: CustomHeadsetOpenVR's camera-calibrated profile. native: the Pimax geometry with the "
                       "eye axis set by a true rotation (correct world scale, straight lines on head turns). "
                       "Lighthouse uses CustomHeadsetOpenVR's own profile.")));
    lensNote_ = pibridge::ui::dimLabel(QString());
    lensNote_->setWordWrap(true);
    lensCard->body()->addWidget(lensNote_);
    root->addWidget(lensCard);

    note_ = pibridge::ui::dimLabel(QStringLiteral("Marked settings take effect the next time the VR app starts."));
    root->addWidget(note_);
    root->addStretch(1);

    for (pibridge::ui::SliderRow* s : {brightness_, contrast_, blackLevel_, gamma_, saturation_}) {
        connect(s, &pibridge::ui::SliderRow::valueChanged, this, [this] {
            if (!loading_) {
                save();
            }
        });
    }
    load();
    refreshInfo();
    connect(&source_, &StateSource::changed, this, &DisplayPage::refreshInfo);
}

void DisplayPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    load();
    refreshInfo();
}

void DisplayPage::loadLens() {
    conf_.reload();
    const QString profile = conf_.value(reckey::kLensProfile, QString()).trimmed();
    const int idx = lens_->findData(profile == QLatin1String("native") ? profile : QStringLiteral("default"));
    lens_->setCurrentIndex(idx < 0 ? 0 : idx);
    // an explicit lens file wins over the profile (rec_setup.cpp); say so instead of silently ignoring the choice
    const QString explicitLens = conf_.value(reckey::kLens, QString()).trimmed();
    lensNote_->setText(explicitLens.isEmpty()
                           ? QString()
                           : QStringLiteral("Overridden: rec_lens_conf=%1 in %2 is used instead of this profile.")
                                 .arg(explicitLens, conf_.path()));
    lensNote_->setVisible(!explicitLens.isEmpty());
}

void DisplayPage::load() {
    loadLens();
    loading_ = true;
    refresh_->setCurrent(conf_.refreshHz() == 72 ? 0 : 1);
    const QJsonObject root = readSettings();
    const QJsonObject gen = root.value(QStringLiteral("generalHeadset")).toObject();
    const QJsonObject col = gen.value(QStringLiteral("colorMultiplier")).toObject();
    const QJsonObject shader = root.value(QStringLiteral("customShader")).toObject();
    const double bright = numOr(col, QStringLiteral("r"), 1.0); // r=g=b used as one brightness
    brightness_->setValueQuiet(static_cast<int>(std::lround(bright * 100.0)));
    contrast_->setValueQuiet(static_cast<int>(std::lround(numOr(shader, QStringLiteral("contrast"), 0.0) * 100.0)));
    // blackLevel is 0..1 in CHOVR, shown as a percent.
    blackLevel_->setValueQuiet(static_cast<int>(std::lround(numOr(gen, QStringLiteral("blackLevel"), 0.0) * 100.0)));
    // gamma: CHOVR default 2.2, shown as gamma x 100.
    gamma_->setValueQuiet(static_cast<int>(std::lround(numOr(shader, QStringLiteral("gamma"), 2.2) * 100.0)));
    // saturation: CHOVR default (and neutral) is 50, written as the raw value.
    saturation_->setValueQuiet(static_cast<int>(std::lround(numOr(shader, QStringLiteral("saturation"), 50.0))));
    loading_ = false;
}

void DisplayPage::save() {
    QJsonObject root = readSettings();
    // brightness goes in the general headset section (applies to the running headset)
    QJsonObject gen = root.value(QStringLiteral("generalHeadset")).toObject();
    const double bright = brightness_->value() / 100.0;
    QJsonObject col = gen.value(QStringLiteral("colorMultiplier")).toObject();
    col[QStringLiteral("r")] = bright;
    col[QStringLiteral("g")] = bright;
    col[QStringLiteral("b")] = bright;
    gen[QStringLiteral("colorMultiplier")] = col;
    gen[QStringLiteral("blackLevel")] = blackLevel_->value() / 100.0; // CHOVR expects 0..1
    root[QStringLiteral("generalHeadset")] = gen;
    // contrast, gamma and saturation are the custom shader (enable it for Pimax so it applies)
    QJsonObject shader = root.value(QStringLiteral("customShader")).toObject();
    shader[QStringLiteral("enable")] = true;
    shader[QStringLiteral("enableForPimax")] = true;
    shader[QStringLiteral("contrast")] = contrast_->value() / 100.0;
    shader[QStringLiteral("gamma")] = gamma_->value() / 100.0; // slider is gamma x 100
    shader[QStringLiteral("saturation")] = saturation_->value(); // 50 = neutral in CHOVR
    root[QStringLiteral("customShader")] = shader;
    writeSettings(root);
}

void DisplayPage::refreshInfo() {
    const auto& devs = source_.state().steamvr.devices;
    const auto it = std::find_if(devs.begin(), devs.end(), [](const VrDevice& d) { return d.cls == VrClass::Hmd; });
    if (it != devs.end() && it->displayHz > 0) {
        info_->setText(QStringLiteral("%1  ·  %2 Hz  ·  %3 x %4 per eye")
                           .arg(it->model.isEmpty() ? QStringLiteral("Headset") : it->model)
                           .arg(static_cast<double>(it->displayHz), 0, 'f', 0)
                           .arg(it->renderWidth)
                           .arg(it->renderHeight));
    } else {
        info_->setText(QStringLiteral("The headset details are shown while SteamVR runs."));
    }
}

} // namespace pibridge::app
