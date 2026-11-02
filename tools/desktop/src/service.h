// SPDX-License-Identifier: MIT
// pibridge-desktop's run loop: captures the monitors only while an overlay shows them (its beat in desktop.shm is
// fresh and it wants a monitor), turns its pointer into the desktop's input, and ends itself once nobody has looked for
// a while (the overlays start it when the desktop is opened). The portal's permission is the restore token in
// tokenPath(): without one it never asks (a question on the desktop is not seen in the headset), it says
// NeedsPermission and waits for `pibridge-desktop allow` to make one.
#pragma once

#include "input.h"
#include "pibridge_desktop.h"
#include "portal.h"
#include "pw_capture.h"

#include <QObject>
#include <QString>
#include <QTimer>

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::desktopsvc {

/// $XDG_STATE_HOME/pibridge/desktop-token (or ~/.local/state/...): the portal's restore token.
QString tokenPath();
QString readToken();
bool writeToken(const QString& token);

struct RunOptions {
    int idleExitS = 30;    // end after this long without an overlay looking (0: never)
    int stopCaptureS = 5;  // stop capturing this long after the overlay stopped looking
    int fps = 120;         // at most this many pictures a second per monitor (KWin sends one per change, at the refresh)
};

class Service : public QObject {
    Q_OBJECT
public:
    Service(desktop::Link& link, const RunOptions& opt);
    ~Service() override;
    void start();

private:
    void tick();
    void startCapture();
    void stopCapture(const std::string& why, desktop::State st);
    static void onFrame(void* user, unsigned index, const std::uint8_t* data, std::uint32_t w, std::uint32_t h,
                        std::uint32_t stride, int format);
    static void onError(void* user, unsigned index, const char* words);
    void log(const std::string& s);

    desktop::Link& link_;
    RunOptions opt_;
    QTimer timer_;
    Portal portal_;
    pibridge_pw* pw_ = nullptr;
    std::vector<PortalStream> streams_;
    InputDiff input_;
    std::vector<desktop::MonitorInfo> layout_;
    std::uint64_t lastClientNs_ = 0;  // the last tick the overlay was looking
    std::uint64_t lastBeat_ = 0;      // the overlay's beat as last seen
    std::uint64_t beatSeenNs_ = 0;    // when it last changed (this clock; 0: never while running)
    std::uint64_t startedNs_ = 0;
    std::uint64_t retryNs_ = 0;       // no new try before this
    bool opening_ = false;
    std::atomic<std::uint32_t> want_{0};
    std::atomic<bool> streamFailed_{false};
    std::array<std::atomic<std::uint64_t>, desktop::kMaxMonitors> lastFrameNs_{};
};

} // namespace pibridge::desktopsvc
