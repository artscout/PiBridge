// SPDX-License-Identifier: MIT
// What the app's buttons and switches do: the commands they run (through a Runner) and the settings they write.
// Kept apart from the widgets so the tests check every command with a FakeRunner.
#pragma once

#include <QString>
#include <QStringList>
#include <functional>

#include "conf.h"
#include "runner.h"

namespace pibridge::app {

class Actions {
public:
    using ToolFinder = std::function<QString(const QString&)>;
    Actions(Runner& runner, Conf& conf, ToolFinder findTool, QString eyeProfileOut);

    // steam steam://rungameid/250820, detached.
    bool launchSteamVR();
    // systemctl --user restart pibridge-keepalived pibridge-eyed
    void restartServices(Done done);
    // systemctl --user restart pibridge-keepalived (wakes the headset again)
    void restartHeadset(Done done);
    // eye_tracking=on|off in the conf, then systemctl --user start|stop pibridge-eyed.
    bool setEyeTracking(bool on, Done done);
    // refresh_hz=72|90, then the keepalive daemon re-reads the conf (SIGHUP through systemctl).
    bool setRefreshHz(int hz, Done done);
    // audio_to_headset=on|off, then SIGHUP to the keepalive daemon.
    bool setAudioToHeadset(bool on, Done done);

    // The eye calibration tool: pibridge-eyecal, else (only when pibridge-eyecal is not found) omarchyvr's pimax-eyecal;
    // empty when neither is found.
    QString eyecalTool() const;
    QString eyeTool() const { return findTool_(QStringLiteral("pibridge-eye")); }
    // systemctl --user stop pibridge-eyed, pibridge-eyecal (the chip's own calibration; it reads the start list and the
    // profile from pibridge.conf like pibridge-eyed), then (eye tracking on) systemctl --user start pibridge-eyed: the eye chip
    // has one owner, and the restarted daemon uploads the new profile. With the pimax-eyecal fallback:
    // pimax-eyecal --pattern pimax --frames <list>, the profile through PIMAX_EYE_PROFILE. False when no tool is
    // found (nothing runs, done is not called).
    bool calibrate(Done done);
    // The same around pibridge-eyecal --recenter (pimax-eyecal --recenter --pattern pimax).
    bool recenter(Done done);
    // Where the eye start list and profile are looked for when the conf names none (first match wins): by default
    // ~/.config/pibridge, then ~/.config/pimax-linux (PiBridge's earlier name), then ~/.config/omarchyvr. Tests point it at
    // a temp dir.
    void setEyeFileDirs(QStringList dirs) { eyeDirs_ = std::move(dirs); }
    // The file eyed uses for `key` (eye_start_list | eye_profile): the conf value, else `name` in the dirs above.
    QString eyeFile(std::string_view key, const QString& name) const;
    // pibridge-eye profile <setpm> -o <eyeProfileOut>; on success eye_profile=<out> in the conf and SIGHUP to pibridge-eyed.
    bool importProfile(const QString& setpm, Done done);
    const QString& eyeProfileOut() const { return profileOut_; }

private:
    void systemctl(const QStringList& args, Done done);
    void reload(const QString& unit);
    void runEyecal(const QString& tool, bool recenter, Done done);

    Runner& runner_;
    Conf& conf_;
    ToolFinder findTool_;
    QString profileOut_;
    QStringList eyeDirs_;
};

} // namespace pibridge::app
