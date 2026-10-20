// SPDX-License-Identifier: MIT
#include "about.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include "../core/paths.h"
#include "../qv/layerinstall.h"
#include "../qv/settings.h"
#include "../ui/brand.h"
#include "../ui/icons.h"
#include "../ui/theme.h"
#include "../ui/widgets.h"

namespace pibridge::app {

AboutPage::AboutPage(QWidget* parent) : QWidget(parent) {
    setObjectName(QStringLiteral("page"));
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 12, 18, 12);
    root->setSpacing(10);
    auto* heading = new QLabel(QStringLiteral("About"));
    heading->setFont(pibridge::ui::titleFont(24));
    root->addWidget(heading);

    auto* card = new pibridge::ui::Card(pibridge::ui::productName());
    card->chip()->set(QStringLiteral("VERSION " PIBRIDGE_VERSION), ChipKind::Info);
    auto* top = new QHBoxLayout;
    top->setSpacing(20);
    auto* logo = new QLabel;
    logo->setPixmap(pibridge::ui::logoPixmap(150));
    top->addWidget(logo, 0, Qt::AlignTop);
    auto* tcol = new QVBoxLayout;
    tcol->setSpacing(6);
    auto* tag = new QLabel(pibridge::ui::tagline());
    tag->setFont(pibridge::ui::bodyFont(15));
    tag->setWordWrap(true);
    tcol->addWidget(tag);
    tcol->addStretch(1);
    top->addLayout(tcol, 1);
    card->body()->addLayout(top);
    card->body()->addWidget(pibridge::ui::dimLabel(
        QStringLiteral("Runs a Pimax headset on Linux under SteamVR: the headset service, eye tracking, and the "
                       "quad views layer's settings, in one app.")));
    auto* g = new QGridLayout;
    g->setHorizontalSpacing(24);
    g->setVerticalSpacing(10);
    int row = 0;
    const auto add = [&](const QString& k, const QString& v) {
        auto* l = pibridge::ui::capsLabel(k, 13);
        l->setObjectName(QStringLiteral("dim"));
        auto* val = new QLabel(v);
        val->setTextInteractionFlags(Qt::TextSelectableByMouse);
        val->setWordWrap(true);
        g->addWidget(l, row, 0, Qt::AlignTop);
        g->addWidget(val, row, 1);
        row++;
    };
    add(QStringLiteral("Version"), QStringLiteral(PIBRIDGE_VERSION));
    add(QStringLiteral("Licence"), QStringLiteral("MIT"));
    add(QStringLiteral("Built on"),
        QStringLiteral("omarchyvr by laughtt and contributors (MIT OR Apache-2.0): the headset and eye tracker "
                       "protocols, the gaze math and the two services, ported from Rust to C++. The quad views page "
                       "uses the settings code of artscout-qv-vulkan-linux (MIT). The SteamVR display driver is "
                       "CustomHeadsetOpenVR by sboys3."));
    add(QStringLiteral("Not affiliated"),
        QStringLiteral("An independent project, not made or endorsed by Pimax. Pimax and the headset names are "
                       "trademarks of their owners."));
    add(QStringLiteral("Settings"), confPath());
    add(QStringLiteral("Service status"), statusDir());
    add(QStringLiteral("Quad views"), QString::fromStdString(qv::settingsPath().string()));
    add(QStringLiteral("Quad views log"), qvgui::logPath());
    g->setColumnStretch(1, 1);
    card->body()->addLayout(g);
    card->body()->addStretch(1);
    root->addWidget(card, 1);
}

} // namespace pibridge::app
