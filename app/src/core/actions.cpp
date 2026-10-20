// SPDX-License-Identifier: MIT
#include "actions.h"
#include "paths.h"

#include <QDir>
#include <QFileInfo>

namespace pibridge::app {

namespace {
const QString kKeepalive = QStringLiteral("pibridge-keepalived");
const QString kEyed = QStringLiteral("pibridge-eyed");
} // namespace

Actions::Actions(Runner& runner, Conf& conf, ToolFinder findTool, QString eyeProfileOut)
    : runner_(runner), conf_(conf), findTool_(std::move(findTool)), profileOut_(std::move(eyeProfileOut)),
      eyeDirs_(eyeFileDirs()) {}

QString Actions::eyeFile(std::string_view key, const QString& name) const {
    const QString set = conf_.value(key, {});
    if (!set.isEmpty()) {
        return set;
    }
    for (const QString& d : eyeDirs_) {
        const QString p = d + QLatin1Char('/') + name;
        if (QFileInfo::exists(p)) {
            return p;
        }
    }
    return {};
}

void Actions::systemctl(const QStringList& args, Done done) {
    runner_.run(QStringLiteral("systemctl"), QStringList{QStringLiteral("--user")} + args, std::move(done));
}

// The daemons read the conf at start and on SIGHUP (docs/CONVENTIONS.md). A stopped unit just ignores it.
void Actions::reload(const QString& unit) {
    systemctl({QStringLiteral("kill"), QStringLiteral("--signal=HUP"), unit}, {});
}

bool Actions::launchSteamVR() {
    return runner_.startDetached(QStringLiteral("steam"), {QStringLiteral("steam://rungameid/250820")});
}

void Actions::restartServices(Done done) {
    systemctl({QStringLiteral("restart"), kKeepalive, kEyed}, std::move(done));
}

void Actions::restartHeadset(Done done) {
    systemctl({QStringLiteral("restart"), kKeepalive}, std::move(done));
}

bool Actions::setEyeTracking(bool on, Done done) {
    if (!conf_.set("eye_tracking", on ? "on" : "off")) {
        return false;
    }
    systemctl({on ? QStringLiteral("start") : QStringLiteral("stop"), kEyed}, std::move(done));
    return true;
}

bool Actions::setRefreshHz(int hz, Done done) {
    if (hz != 72 && hz != 90) {
        return false;
    }
    if (!conf_.set("refresh_hz", confNumber(hz))) {
        return false;
    }
    reload(kKeepalive);
    if (done) {
        done(0, {});
    }
    return true;
}

bool Actions::setAudioToHeadset(bool on, Done done) {
    if (!conf_.set("audio_to_headset", on ? "on" : "off")) {
        return false;
    }
    reload(kKeepalive);
    if (done) {
        done(0, {});
    }
    return true;
}

QString Actions::eyecalTool() const {
    if (QString t = findTool_(QStringLiteral("pibridge-eyecal")); !t.isEmpty()) {
        return t;
    }
    return findTool_(QStringLiteral("pimax-eyecal"));
}

// The eye chip has one owner: pibridge-eyed lets go of it (unsubscribe, release) while the tool runs, and takes it again
// when the tool exits, unless eye tracking is switched off; its new start uploads the profile the tool saved.
// pibridge-eyecal reads the start list and the profile from pibridge.conf itself (as pibridge-eyed). The pimax-eyecal fallback
// opens the chip without knowing PiBridge's settings, so it gets the same start list (--frames) and user profile
// (PIMAX_EYE_PROFILE through env: pimax-eyecal has no option for it, and as omarchyvr's tool it reads the older name) as
// the daemon.
void Actions::runEyecal(const QString& tool, bool recenter, Done done) {
    QString program = tool;
    QStringList args;
    if (QFileInfo(tool).fileName() == QStringLiteral("pibridge-eyecal")) {
        if (recenter) {
            args << QStringLiteral("--recenter");
        }
    } else {
        if (recenter) {
            args << QStringLiteral("--recenter");
        }
        args << QStringLiteral("--pattern") << QStringLiteral("pimax");
        const QString list = eyeFile("eye_start_list", QStringLiteral("eye-start-crystal-super.txt"));
        if (!list.isEmpty()) {
            args << QStringLiteral("--frames") << list;
        }
        const QString profile = eyeFile("eye_profile", QStringLiteral("eye-profile-crystal-super.bin"));
        if (!profile.isEmpty()) {
            args.prepend(tool);
            args.prepend(QStringLiteral("PIMAX_EYE_PROFILE=") + profile);
            program = QStringLiteral("env");
        }
    }
    systemctl({QStringLiteral("stop"), kEyed}, [this, program, args, done](int, const QString&) {
        runner_.run(program, args, [this, done](int code, const QString& out) {
            if (conf_.eyeTracking()) {
                systemctl({QStringLiteral("start"), kEyed}, {});
            }
            if (done) {
                done(code, out);
            }
        });
    });
}

bool Actions::calibrate(Done done) {
    const QString tool = eyecalTool();
    if (tool.isEmpty()) {
        return false;
    }
    runEyecal(tool, false, std::move(done));
    return true;
}

bool Actions::recenter(Done done) {
    const QString tool = eyecalTool();
    if (tool.isEmpty()) {
        return false;
    }
    runEyecal(tool, true, std::move(done));
    return true;
}

bool Actions::importProfile(const QString& setpm, Done done) {
    const QString tool = eyeTool();
    if (tool.isEmpty() || setpm.isEmpty()) {
        return false;
    }
    QDir().mkpath(QFileInfo(profileOut_).absolutePath());
    runner_.run(tool, {QStringLiteral("profile"), setpm, QStringLiteral("-o"), profileOut_},
                [this, done](int code, const QString& out) {
                    if (code == 0) {
                        conf_.set("eye_profile", profileOut_.toStdString());
                        reload(kEyed);
                    }
                    if (done) {
                        done(code, out);
                    }
                });
    return true;
}

} // namespace pibridge::app
