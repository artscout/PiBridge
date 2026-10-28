// SPDX-License-Identifier: MIT
// The live state the pages show, re-read every second: the two status files, whether SteamVR (vrserver) runs, and
// the SteamVR devices (a worker thread, core/steamvr.h, connected only while vrserver runs).
// In fake mode (--fake, PIBRIDGE_FAKE_STATE=1, --screenshots) it holds fixed values and reads nothing.
#pragma once

#include <QObject>
#include <QTimer>
#include <atomic>
#include <memory>
#include <optional>

#include "gazeshm.h"
#include "status.h"
#include "steamvr.h"

namespace pibridge::app {

struct AppState {
    KeepaliveStatus keepalive;
    EyedStatus eyed;
    bool steamvrRunning = false;
    bool monadoRunning = false; // monado-service ("Monado directly", PiBridge World)
    SteamVrStatus steamvr; // the devices SteamVR reports (empty unless connected)
};

// The fixed states of fake mode: "active" (SteamVR on, screen on, eyes streaming, fakeVrDevices() read through a
// FakeSteamVrSource) and "idle" (nothing running).
AppState fakeState(bool active);
// The status bar's runtime line: "SteamVR running", "Monado running", "SteamVR and Monado running" or
// "VR not running".
QString vrRuntimeWords(const AppState& s);
// A fixed fake gaze (a little right and up of straight ahead).
GazeSnapshot fakeGaze();

class StateSource : public QObject {
    Q_OBJECT
public:
    // Real mode starts the SteamVR worker with `vr` (default: the OpenVR backend); fake mode reads nothing.
    explicit StateSource(bool fake, QObject* parent = nullptr, std::unique_ptr<SteamVrSource> vr = {});
    ~StateSource() override;
    const AppState& state() const { return state_; }
    bool fake() const { return fake_; }
    // Fake mode: show this state now (screenshots).
    void setFakeState(const AppState& s);
    // The gaze for the dashboard dot (read only when asked, so only while the page is visible).
    std::optional<GazeSnapshot> gaze() const;
    void refresh();

signals:
    void changed();

private:
    bool fake_;
    AppState state_;
    QTimer timer_;
    std::atomic<bool> vrserver_{false}; // the last process check, read by the SteamVR worker
    std::unique_ptr<SteamVrWorker> vrWorker_;
};

} // namespace pibridge::app
