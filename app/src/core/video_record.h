// SPDX-License-Identifier: MIT
// The Dashboard's Record card without widgets (tested): what it shows for the recorder's status file and the VR
// runtime that runs, its settings (~/.config/pibridge/record.toml, shared with `pibridge-record`), and the commands its
// buttons run (pibridge-record start | stop, through a Runner). docs/record.md.
//
// The source follows the session (pibridge-record start --source auto): while SteamVR runs (also SteamVR + Monado,
// whose image SteamVR's compositor draws), SteamVR's mirror texture through pibridge-record-mirror; while Monado's own
// compositor runs ("Monado directly", Development page), Monado's recorder tap (comp_recorder.c).
#pragma once

#include <QString>
#include <QStringList>

#include "pibridge/record/settings.h"
#include "pibridge/record/status.h"
#include "runner.h"
#include "status.h"

namespace pibridge::app {

struct RecordInputs {
    pibridge::record::Status status; // <session dir>/record.status (empty when none)
    bool recorderRunning = false; // the pid file names a live pibridge-record
    bool directRunning = false;   // "Monado directly" (Development page) runs
    bool monadoRunning = false;   // some monado-service runs (also one started outside the app)
    bool steamvrRunning = false;  // vrserver runs
};

struct RecordCardModel {
    QString chip;
    ChipKind chipKind = ChipKind::Off;
    QString headline; // "Recording 0:12" / "Ready to record" / "Not available under SteamVR" ...
    QString detail;   // plain words: what happens, or why not
    bool canStart = false;
    bool canStop = false;
    QString lastFile;   // the newest file (path), or empty
    QString lastFolder; // its folder, or empty
    bool error = false; // the detail is an error (shown in the error colour)
};

RecordCardModel recordCardModel(const RecordInputs& in);

// The inputs from disk and /proc (not in fake mode: there the caller fills them).
RecordInputs readRecordInputs(bool directRunning, bool steamvrRunning);

// The settings the card shows: PiBridge's record.toml, else omarchyvr's, else the defaults (a broken file shows the
// defaults, and `error` says why).
pibridge::record::Settings loadRecordSettings(QString* error = nullptr);
// Write them to ~/.config/pibridge/record.toml. Empty on success, else why.
QString saveRecordSettings(const pibridge::record::Settings& s);
QString recordSettingsPath();

// The shared-memory ring both sides use: $PIMAX_RECORD_SHM, else $XDG_RUNTIME_DIR/pimax-record.shm.
QString recordShmPath();

// pibridge-record start / stop with the saved settings. `tool`: the pibridge-record binary. `done` gets its output line.
void startRecording(Runner& runner, const QString& tool, Done done);
void stopRecording(Runner& runner, const QString& tool, Done done);

} // namespace pibridge::app
