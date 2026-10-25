// SPDX-License-Identifier: MIT
// The SteamVR dashboard panel (core/overlay_control.h) without starting a process: the plain-words status for every
// combination, the chip, and finding the binary. Files only in a temporary folder.
#include "core/conf.h"
#include "core/overlay_control.h"

#include "pibridge_test.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>

using namespace pibridge;
using namespace pibridge::app;

TEST(overlay_status_words) {
    // off: plain words, no error, OFF chip
    const OverlayStatus off = overlayStatusWords(false, false, false, QString());
    CHECK(off.run == OverlayRun::Off && !off.error);
    CHECK(overlayChip(off).text == QLatin1String("OFF"));
    CHECK(overlayChip(off).kind == ChipKind::Off);

    // wanted but not running -> failed, the last error carried through
    const OverlayStatus failed = overlayStatusWords(true, false, true, QStringLiteral("pibridge-overlay could not start."));
    CHECK(failed.run == OverlayRun::Failed && failed.error);
    CHECK(failed.words == QLatin1String("pibridge-overlay could not start."));
    CHECK(overlayChip(failed).kind == ChipKind::Error);

    // wanted, not running, no error text -> a default line (still failed)
    CHECK(overlayStatusWords(true, false, true, QString()).words == QLatin1String("Not running."));

    // running but SteamVR down -> waiting (warn), not an error
    const OverlayStatus wait = overlayStatusWords(true, true, false, QString());
    CHECK(wait.run == OverlayRun::WaitingForSteamVr && !wait.error);
    CHECK(overlayChip(wait).kind == ChipKind::Warn);

    // running with SteamVR up -> running (info)
    const OverlayStatus run = overlayStatusWords(true, true, true, QString());
    CHECK(run.run == OverlayRun::Running && !run.error);
    CHECK(overlayChip(run).kind == ChipKind::Info);
}

TEST(overlay_find_program) {
    QTemporaryDir dir;
    CHECK(dir.isValid());
    // nothing there, no PATH lookup -> ""
    CHECK(findOverlayProgram(dir.path(), nullptr).isEmpty());
    // an executable next to the app binary is found
    const QString exe = dir.path() + QStringLiteral("/pibridge-overlay");
    QFile f(exe);
    CHECK(f.open(QIODevice::WriteOnly));
    f.write("#!/bin/sh\n");
    f.close();
    QFile::setPermissions(exe, QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);
    CHECK(findOverlayProgram(dir.path(), nullptr) == QDir::cleanPath(exe));
    // the PATH fallback is used when it is not next to the binary
    bool asked = false;
    const QString viaPath = findOverlayProgram(dir.path() + QStringLiteral("/empty"),
                                               [&](const QString& n) {
                                                   asked = true;
                                                   return QStringLiteral("/usr/bin/") + n;
                                               });
    CHECK(asked && viaPath == QLatin1String("/usr/bin/pibridge-overlay"));
}

TEST(overlay_parse_request) {
    using R = OverlayRequest;
    CHECK(parseOverlayRequest(QByteArray("room-setup\n")) == std::optional<R>(R{OverlayAction::RoomSetupFull}));
    CHECK(parseOverlayRequest(QByteArray("quick-room-setup\n")) == std::optional<R>(R{OverlayAction::RoomSetupQuick}));
    CHECK(parseOverlayRequest(QByteArray("quick-room-setup")) ==
          std::optional<R>(R{OverlayAction::RoomSetupQuick})); // no newline
    CHECK(parseOverlayRequest(QByteArray("record-start\n")) == std::optional<R>(R{OverlayAction::RecordStart}));
    CHECK(parseOverlayRequest(QByteArray("record-stop\n")) == std::optional<R>(R{OverlayAction::RecordStop}));
    // record-set carries the key and value
    CHECK(parseOverlayRequest(QByteArray("record-set view both\n")) ==
          std::optional<R>(R{OverlayAction::RecordSet, QStringLiteral("view"), QStringLiteral("both")}));
    CHECK(parseOverlayRequest(QByteArray("record-set max_height 0")) ==
          std::optional<R>(R{OverlayAction::RecordSet, QStringLiteral("max_height"), QStringLiteral("0")}));
    CHECK(!parseOverlayRequest(QByteArray("record-set bogus 1\n")).has_value()); // unknown key
    CHECK(!parseOverlayRequest(QByteArray("record-set view\n")).has_value());    // missing value
    CHECK(!parseOverlayRequest(QByteArray("")).has_value());
    CHECK(!parseOverlayRequest(QByteArray("something-else\n")).has_value());
}

TEST(overlay_control_persists) {
    QTemporaryDir dir;
    CHECK(dir.isValid());
    const QString conf = dir.path() + QStringLiteral("/pibridge.conf");
    {
        Conf c(conf);
        OverlayControl ctl(c, /*fake=*/true);
        CHECK(ctl.wanted() && ctl.running()); // default ON: it runs without the user turning it on
        CHECK(ctl.setWanted(false));
        CHECK(!ctl.wanted() && !ctl.running());
        CHECK(ctl.setWanted(true));
        CHECK(ctl.wanted() && ctl.running()); // fake: "running" follows the switch
        CHECK(ctl.status(false).run == OverlayRun::WaitingForSteamVr);
        CHECK(ctl.status(true).run == OverlayRun::Running);
    }
    // a new control reads the saved choice and follows it (a restart keeps it on)
    Conf c2(conf);
    OverlayControl again(c2, /*fake=*/true);
    CHECK(again.wanted() && again.running());
    again.setWanted(false);
    CHECK(!again.wanted() && !again.running());
}

PIBRIDGE_TEST_MAIN();
