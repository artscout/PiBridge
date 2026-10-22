// SPDX-License-Identifier: MIT
// The app's reader of the daemons' status files ($XDG_RUNTIME_DIR/pibridge/keepalived.json and eyed.json,
// docs/CONVENTIONS.md), and the plain words the pages show for them. A missing, unreadable or stale file (not
// rewritten for more than kStaleMs; the daemons rewrite it at least every 2 s) means the daemon is not running.
#pragma once

#include <QByteArray>
#include <QString>
#include <optional>

namespace pibridge::app {

inline constexpr qint64 kStaleMs = 6000;

struct KeepaliveStatus {
    bool running = false; // the file is there and fresh
    QString state;        // idle | waking | active | no-headset | error | paused (an inside-out session owns the headset)
    QString model;
    bool headsetPresent = false;
    bool panelOn = false;
    int rateHz = 0;
    double lensIpdMm = 0;    // the lens IPD the headset reports (lens_ipd_x100 / 100), 0 = not known
    QString follow;
    bool followRunning = false;
    bool audioOnHeadset = false;
    QString error;
    // Controllers (docs/keepalived.md "Controllers"): the last pair / unpair request and its result, which controllers
    // were heard from in the last 10 s, the vibration tests taken and the words for the last one.
    qint64 ctrlId = 0;
    QString ctrlOp;     // pair | unpair
    QString ctrlSide;   // left | right
    QString ctrlState;  // waiting | paired | timeout | unpaired | failed | refused
    QString ctrlDetail; // plain words
    bool leftConnected = false;
    bool rightConnected = false;
    qint64 hapticSeq = 0;
    QString hapticResult;
    qint64 pid = 0;
    qint64 updatedMs = 0;
};

struct EyedStatus {
    bool running = false;
    QString state; // idle | searching | connecting | streaming | error | paused (inside-out: Monado owns the eye chip)
    QString profile; // user | list | none
    int eyesSeen = 0;
    double samplesPerS = 0;
    QString calibration;
    QString follow;
    bool followRunning = false;
    QString error;
    qint64 pid = 0;
    qint64 updatedMs = 0;
};

// Parse the JSON text; nullopt when it is not a JSON object. `running` is set to true (freshness is the reader's job).
std::optional<KeepaliveStatus> parseKeepalive(const QByteArray& json);
std::optional<EyedStatus> parseEyed(const QByteArray& json);
// Read and parse a file; `running` false when it is missing, broken or older than kStaleMs at `nowMs` (unix ms).
// Without "updated_unix_ms" the file's modification time is used.
KeepaliveStatus readKeepalive(const QString& path, qint64 nowMs);
EyedStatus readEyed(const QString& path, qint64 nowMs);

enum class ChipKind { Info, Warn, Error, Off };
struct ChipText {
    QString text;
    ChipKind kind;
};

// The headset card's chip: SCREEN ON / SCREEN OFF / NO HEADSET / ERROR, or SERVICE OFF without the daemon.
ChipText headsetChip(const KeepaliveStatus& k);
// One line under the headset title: what is going on and what to do, in plain words.
QString headsetHint(const KeepaliveStatus& k, bool steamvrRunning);
// The eye tracking status line ("Streaming: 2 eyes, 120 samples/s").
QString eyeLine(const EyedStatus& e);
// Short eye state for chips and the status bar ("STREAMING", "SERVICE OFF").
ChipText eyeChip(const EyedStatus& e);

} // namespace pibridge::app
