// SPDX-License-Identifier: MIT
// The controllers as Monado sees them, for the Dashboard's Controllers card while the Tracking session runs with apps
// through "Monado directly" (SteamVR is not running then, so core/steamvr.h has nothing to say). Read through
// libmonado (the mnd_* C API that monado-ctl uses: a plain IPC client, no OpenXR session, so it never takes the
// headset from an app), loaded with dlopen from the session's Monado build (core/apps_runtime.h libmonadoFor()):
// PiBridge builds and runs without it.
//
// What libmonado tells: the device list with names, which device has the "left" / "right" role, and the battery of a
// device that reports one (mnd_root_get_device_battery_status; the Pimax driver says "present" once the controller
// has sent a report, i.e. it is on). It does not say whether a controller is tracked.
//
// Layers (all but the dlopen backend are tested with fakes), as core/steamvr.h:
//   MonadoSource   the reader: the libmonado backend or FakeMonadoSource
//   MonadoLink     connect / poll / back off / disconnect (no threads; an injected clock)
//   MonadoWorker   a std::jthread that runs the link once a second and hands each status to a callback; the GUI sets
//                  its target (wanted, the library, the socket) at any time
//   monadoControllersView  the card's words
#pragma once

#include <QString>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "steamvr.h"

namespace pibridge::app {

struct MonadoDevice {
    int index = 0;
    QString name;            // MND_PROPERTY_NAME_STRING ("Pimax Crystal Super Left Controller")
    VrRole role = VrRole::None; // from mnd_root_get_device_from_role("left" / "right")
    bool head = false;       // the "head" role
    bool batterySupported = false;
    bool batteryPresent = false; // the device reports a battery now (Pimax: it has sent a report: it is on)
    bool charging = false;
    float battery = 0;       // 0..1
};

enum class MndLinkState {
    Off,        // not wanted (the Tracking session does not run with Monado directly)
    NoLibrary,  // no libmonado.so in the Monado build, or it could not be loaded
    NotRunning, // Monado's IPC socket is not there
    Waiting,    // the last connect failed; the next try after kMndRetryMs
    Connected,
};

struct MonadoStatus {
    MndLinkState state = MndLinkState::Off;
    QString error; // plain words of the last failure, "" when none
    QString library;
    std::vector<MonadoDevice> devices;
};

struct MndConnect {
    enum Result { Ok, NoLibrary, Failed } result = Failed;
    QString error;
};

class MonadoSource {
public:
    virtual ~MonadoSource() = default;
    // Load `library` (once) and connect (mnd_root_create).
    virtual MndConnect connect(const QString& library) = 0;
    // The devices now; nullopt when Monado is gone (any mnd_* call failed).
    virtual std::optional<std::vector<MonadoDevice>> poll() = 0;
    // mnd_root_destroy. Safe to call when not connected.
    virtual void disconnect() = 0;
};

// libmonado through dlopen (RTLD_LOCAL); the library stays loaded once it was.
std::unique_ptr<MonadoSource> makeLibMonadoSource();

// Scripted answers, for tests and the fake mode: each connect() takes the next entry of `connects` (the last
// repeats; empty = Ok), each poll() the next of `polls` (the last repeats; nullopt = gone).
class FakeMonadoSource : public MonadoSource {
public:
    MndConnect connect(const QString& library) override;
    std::optional<std::vector<MonadoDevice>> poll() override;
    void disconnect() override;

    std::vector<MndConnect> connects;
    std::vector<std::optional<std::vector<MonadoDevice>>> polls;
    QStringList libraries; // what connect() was asked to load
    int pollCalls = 0;
    int disconnectCalls = 0;
    bool connected = false;
};

// The fake mode's devices: the headset, the left controller on (64 %, charging), the right one off.
std::vector<MonadoDevice> fakeMonadoDevices();

inline constexpr qint64 kMndRetryMs = 3000;
inline constexpr int kMndPollMs = 1000;

struct MonadoTarget {
    bool wanted = false;
    QString library;    // libmonado.so ("" = not found)
    QString socketPath; // Monado's IPC socket; "" = do not check (fake mode)
    bool operator==(const MonadoTarget&) const = default;
};

class MonadoLink {
public:
    using Clock = std::function<qint64()>;
    using Exists = std::function<bool(const QString&)>;
    MonadoLink(MonadoSource& source, Clock clock, Exists exists = {});
    ~MonadoLink();
    MonadoLink(const MonadoLink&) = delete;
    MonadoLink& operator=(const MonadoLink&) = delete;
    // One step for `t`: connects (at most every kMndRetryMs), polls, or drops the connection.
    MonadoStatus tick(const MonadoTarget& t);
    void stop();
    bool connected() const { return connected_; }

private:
    MonadoSource& source_;
    Clock clock_;
    Exists exists_;
    bool connected_ = false;
    QString connectedLib_;
    std::optional<qint64> lastAttemptMs_;
    MonadoStatus last_;
};

class MonadoWorker {
public:
    using Deliver = std::function<void(const MonadoStatus&)>; // called on the worker thread
    MonadoWorker(std::unique_ptr<MonadoSource> source, Deliver deliver, int intervalMs = kMndPollMs);
    ~MonadoWorker();
    // What to read (thread-safe; wakes the thread, which ticks with it at once).
    void setTarget(const MonadoTarget& t);

private:
    std::unique_ptr<MonadoSource> source_;
    Deliver deliver_;
    int intervalMs_;
    std::mutex m_;
    std::condition_variable_any cv_;
    MonadoTarget target_;
    bool changed_ = false;
    std::jthread thread_; // last: started after everything it uses
};

// The card: header "MONADO: 1/2 ON" (or MONADO OFF / NO LIBMONADO / CONNECTING), the left and right rows, the
// message ("Turn the controllers on.", "Start tracking to see the controllers.", "libmonado was not found ...").
ControllersView monadoControllersView(const MonadoStatus& s);

} // namespace pibridge::app
