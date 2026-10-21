// SPDX-License-Identifier: MIT
// The SteamVR dashboard launcher: pibridge-overlay (tools/overlay, docs/OVERLAY.md) draws a small dashboard panel with
// two buttons (ROOM SETUP, QUICK ROOM SETUP), clickable with the controller laser. On a click it writes a one-line
// request to $XDG_RUNTIME_DIR/pibridge/overlay-request, which this class watches and turns into roomSetupRequested(),
// so the app launches the setup with its own machinery (the overlay never spawns pibridge-roomsetup itself). A persisted
// on/off switch (the Development page) starts and stops the overlay; it is a leaf (a background process that waits for
// SteamVR and re-connects on its own), so if it fails VR and the app go on. The choice lives in pibridge.conf
// (steamvr_dashboard), so a restart keeps it.
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>
#include <QTimer>
#include <functional>
#include <memory>
#include <optional>

#include "status.h" // ChipText

namespace pibridge::app {

class Conf;

// pibridge.conf key: the SteamVR dashboard panel on/off (read by the app only; the daemons ignore it).
inline constexpr char kDashboardKey[] = "steamvr_dashboard";

// What the overlay does now, in plain words for the page.
enum class OverlayRun { Off, Running, WaitingForSteamVr, Failed };
struct OverlayStatus {
    OverlayRun run = OverlayRun::Off;
    QString words; // plain words
    bool error = false;
};
// Pure: the status from the saved choice, whether the process is up, whether SteamVR runs, and the last error ("").
OverlayStatus overlayStatusWords(bool wanted, bool processRunning, bool steamvrRunning, const QString& lastError);
// The card chip for a status.
ChipText overlayChip(const OverlayStatus& s);

// pibridge-overlay next to the pibridge binary (`appDir`), in the build tree (<appDir>/../tools/overlay/), else in
// PATH; "" when not found.
QString findOverlayProgram(const QString& appDir, const std::function<QString(const QString&)>& which);

// What the overlay asked the app to do (the first line of the request file). Matches the strings pibridge-overlay
// writes (tools/overlay/src/overlay_core.h).
enum class OverlayAction { RoomSetupFull, RoomSetupQuick, RecordStart, RecordStop };
// Pure: the overlay's request line -> the action, or nullopt when it is not a request the app knows.
std::optional<OverlayAction> parseOverlayRequest(const QByteArray& text);

// Owns pibridge-overlay as a leaf background process driven by the persisted switch, and watches its request file.
class OverlayControl : public QObject {
    Q_OBJECT
public:
    // `fake` (screenshots / tests): never starts a real process; running() follows the switch.
    OverlayControl(Conf& conf, bool fake, QObject* parent = nullptr);
    ~OverlayControl() override;

    bool wanted() const;                 // the saved choice (pibridge.conf)
    bool running() const;                // the process is up
    OverlayStatus status(bool steamvrRunning) const;
    const QString& program() const { return program_; }

    // Write the choice and start / stop to match; false only when pibridge.conf could not be written (it still starts /
    // stops).
    bool setWanted(bool on);
    // Start / stop to match the saved choice (the constructor calls it, so a restart keeps the panel on).
    void follow();

signals:
    void changed();
    // The overlay was clicked: launch room setup (quick = the quick circle setup). The app wires this to the Room page.
    void roomSetupRequested(bool quick);
    // The overlay was clicked: start (true) or stop (false) a video recording. The app wires this to the Record card.
    void recordRequested(bool start);

private:
    void start();
    void stop();
    void pollRequest();

    Conf& conf_;
    bool fake_;
    QString program_;
    QString requestPath_;
    std::unique_ptr<QProcess> proc_;
    QTimer requestTimer_;
    bool fakeRunning_ = false;
    bool stopping_ = false;
    QString lastError_;
};

} // namespace pibridge::app
