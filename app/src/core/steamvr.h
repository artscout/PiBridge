// SPDX-License-Identifier: MIT
// Live device status from SteamVR (controllers, base stations, trackers), read through OpenVR as a *background*
// application: that kind of client never starts SteamVR and never changes its state; it fails at once when
// vrserver is not running. The app connects only while vrserver runs, retries at most every kRetryMs, and drops
// the connection on VREvent_Quit or when vrserver goes away.
//
// Layers (all but the OpenVR backend are tested with fakes):
//   SteamVrSource      the device reader: the dlopen backend (steamvr_openvr.cpp) or FakeSteamVrSource
//   SteamVrLink        connect / poll / back off / disconnect, with an injected clock (no threads)
//   SteamVrWorker      a std::jthread that runs the link once per second and hands each status to a callback
//   controllersView,   the plain words and chips the dashboard shows
//   trackingView
// Serial numbers are read for support (shown next to the device with a Copy button); they are never logged, never
// written to a file and never put in a screenshot (the fake devices carry made-up ones).
#pragma once

#include <array>

#include <QString>
#include <QStringList>
#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "status.h"

namespace pibridge::app {

// ---------------------------------------------------------------- what SteamVR tells us

enum class VrClass { Hmd, Controller, TrackingReference, GenericTracker, Other };
enum class VrRole { None, Left, Right };

// OpenVR's ETrackingResult values (openvr.h), kept as numbers so this header does not need openvr.h.
namespace vrtrack {
inline constexpr int Uninitialized = 1;
inline constexpr int CalibratingInProgress = 100;
inline constexpr int CalibratingOutOfRange = 101;
inline constexpr int RunningOk = 200;
inline constexpr int RunningOutOfRange = 201;
inline constexpr int FallbackRotationOnly = 300;
} // namespace vrtrack

// OpenVR's EDeviceActivityLevel values.
namespace vractivity {
inline constexpr int Unknown = -1;
inline constexpr int Idle = 0;
inline constexpr int UserInteraction = 1;
inline constexpr int UserInteractionTimeout = 2;
inline constexpr int Standby = 3;
inline constexpr int IdleTimeout = 4;
} // namespace vractivity

struct VrDevice {
    int index = 0; // tracked device index (0..63)
    VrClass cls = VrClass::Other;
    VrRole role = VrRole::None;
    QString model;          // Prop_ModelNumber_String ("Knuckles Left", "Valve SR Imp")
    QString controllerType; // Prop_ControllerType_String ("knuckles"), may be empty
    QString serial;         // Prop_SerialNumber_String, for support only (UI + clipboard)
    bool hasBattery = false;
    float battery = 0;      // 0..1
    bool charging = false;
    bool poseValid = false;
    int trackingResult = vrtrack::Uninitialized;
    int activity = vractivity::Unknown;
    // Prop_CurrentUniverseId_Uint64 (the headset only; 0 = not reported): which universe of the chaperone file is in
    // use (the Room page marks it).
    quint64 universe = 0;
    // The headset only (0 = not reported): what SteamVR renders with (the Device page).
    float ipdMeters = 0;      // Prop_UserIpdMeters_Float
    float displayHz = 0;      // Prop_DisplayFrequency_Float
    quint32 renderWidth = 0;  // IVRSystem::GetRecommendedRenderTargetSize, per eye, supersampling included
    quint32 renderHeight = 0;
};

enum class VrLinkState {
    NotRunning,   // vrserver is not running: nothing is tried
    NotInstalled, // no openvrpaths.vrpath, no runtime in it, or libopenvr_api.so / its symbols are missing
    Waiting,      // vrserver runs but the last connect failed; the next try comes after kRetryMs
    Connected,
};

struct SteamVrStatus {
    VrLinkState state = VrLinkState::NotRunning;
    QString error; // plain words of the last failure, "" when none
    std::vector<VrDevice> devices;
    // The SteamVR compositor's process id from IVRApplications::GetApplicationProcessId("openvr.component.vrcompositor")
    // while connected: > 0 runs, 0 does not run, -1 unknown (not connected, or IVRApplications not available).
    int compositorPid = -1;
};

// ---------------------------------------------------------------- SteamVR's standing frame for the room map

// The file pibridge-roommap reads to place the room under SteamVR (libpibridge-roommap floor.h SteamVrStage): SteamVR's
// GetRawZeroPoseToStandingAbsoluteTrackingPose (row major 3x4) for the running vrserver, written by the SteamVR link
// whenever it changes. $XDG_RUNTIME_DIR/pibridge/steamvr-stage.
QString steamVrStagePath();
QString steamVrStageText(qint64 vrserverPid, quint64 universe, const std::array<float, 12>& standingFromRaw);
// The pid of a running vrserver under `procRoot` (0: none).
qint64 vrserverPid(const QString& procRoot = QStringLiteral("/proc"));

// ---------------------------------------------------------------- the source

struct VrConnect {
    enum Result { Ok, NotInstalled, Failed } result = Failed;
    QString error;
};

class SteamVrSource {
public:
    virtual ~SteamVrSource() = default;
    // Connect as a background application. Called only while vrserver runs.
    virtual VrConnect connect() = 0;
    // Read the events and the devices. nullopt: SteamVR quits (VREvent_Quit) or the connection is gone.
    virtual std::optional<std::vector<VrDevice>> poll() = 0;
    // Drop the connection (VR_ShutdownInternal). Safe to call when not connected.
    virtual void disconnect() = 0;
    // The compositor's process id (see SteamVrStatus::compositorPid), asked right after each successful poll().
    // The default: unknown.
    virtual int compositorPid() { return -1; }
};

// The real backend: libopenvr_api.so of the installed SteamVR, loaded with dlopen on the first connect.
std::unique_ptr<SteamVrSource> makeOpenVrSource();

// Scripted answers, for tests and the fake mode. Each connect() takes the next entry of `connects` (the last one
// repeats; empty = Ok); each poll() takes the next entry of `polls` (the last repeats; nullopt = quit).
class FakeSteamVrSource : public SteamVrSource {
public:
    VrConnect connect() override;
    std::optional<std::vector<VrDevice>> poll() override;
    void disconnect() override;
    int compositorPid() override { return compositor; }

    std::vector<VrConnect> connects;
    std::vector<std::optional<std::vector<VrDevice>>> polls;
    int connectCalls = 0;
    int pollCalls = 0;
    int disconnectCalls = 0;
    bool connected = false;
    int compositor = -1; // what compositorPid() answers
};

// The devices the fake mode shows: headset, two Index controllers (left 82 % charging and tracked, right 45 % not
// tracked), two base stations.
std::vector<VrDevice> fakeVrDevices();

// ---------------------------------------------------------------- where SteamVR is

// $XDG_CONFIG_HOME/openvr/openvrpaths.vrpath (default ~/.config/openvr/...), the file vrpathreg writes.
QString openVrPathsFile();
// The first entry of "runtime" in that JSON file; empty when the file is missing, broken or has none.
QString openVrRuntimeDir(const QString& vrpathFile);
// <runtime>/bin/linux64/libopenvr_api.so; empty when `runtimeDir` is empty.
QString openVrApiLibrary(const QString& runtimeDir);

// ---------------------------------------------------------------- the link (connect, back off, disconnect)

inline constexpr qint64 kRetryMs = 5000;

class SteamVrLink {
public:
    using Clock = std::function<qint64()>; // monotonic ms
    SteamVrLink(SteamVrSource& source, Clock clock) : source_(source), clock_(std::move(clock)) {}
    ~SteamVrLink();
    SteamVrLink(const SteamVrLink&) = delete;
    SteamVrLink& operator=(const SteamVrLink&) = delete;

    // One step: `vrserverRunning` is the app's process check. Connects (at most every kRetryMs), polls, or drops
    // the connection, and returns what is known now.
    SteamVrStatus tick(bool vrserverRunning);
    // Drop the connection now (the worker stops).
    void stop();
    bool connected() const { return connected_; }

private:
    SteamVrSource& source_;
    Clock clock_;
    bool connected_ = false;
    std::optional<qint64> lastAttemptMs_;
    SteamVrStatus last_;
};

// ---------------------------------------------------------------- the worker thread

class SteamVrWorker {
public:
    using Deliver = std::function<void(const SteamVrStatus&)>; // called on the worker thread
    // Starts the thread at once: every `intervalMs` it runs one link tick with `vrserverRunning()`'s answer.
    SteamVrWorker(std::unique_ptr<SteamVrSource> source, std::function<bool()> vrserverRunning, Deliver deliver,
                  int intervalMs = 1000);
    // Stops the thread and drops the connection (on the worker thread, as every OpenVR call).
    ~SteamVrWorker();

private:
    std::unique_ptr<SteamVrSource> source_;
    std::function<bool()> vrserverRunning_;
    Deliver deliver_;
    int intervalMs_;
    std::jthread thread_; // last: started after everything it uses
};

// ---------------------------------------------------------------- the words the dashboard shows

struct ControllerRow {
    bool present = false;
    QString name;    // "Valve Index Controller", "" when absent
    ChipText chip;   // CONNECTED / NOT TRACKED / DISCONNECTED
    QString detail;  // "Out of range", "Standby", ""
    int batteryPercent = -1; // -1: no battery reported
    bool charging = false;
    QString batteryText; // "82 % charging", "45 %", ""
    QString serial;      // "" when absent or not reported
};

struct ControllersView {
    ChipText header;  // "1/2 CONNECTED", "STEAMVR OFF", ...
    QString message;  // "Start SteamVR to see the controllers", "" when connected
    ControllerRow left;
    ControllerRow right;
    QStringList extra; // other controllers and trackers, one line each
};

struct StationRow {
    QString name;  // "Station 1"
    QString model; // "SteamVR Base Station 2.0"
    bool tracking = false;
    QString text;  // "ok" / "not seen"
    QString serial;
};

struct TrackingView {
    QString summary; // "2 base stations, 2 tracking"; "" without SteamVR
    QString message; // what to do, when SteamVR is not connected
    std::vector<StationRow> stations;
};

// A friendly device name from the model number and the controller type.
QString vrDeviceName(const VrDevice& d);
// Tracking result in plain words ("Tracking", "Out of range", "Calibrating", ...).
QString vrTrackingWords(int trackingResult);
ControllersView controllersView(const SteamVrStatus& s, bool steamvrRunning);
TrackingView trackingView(const SteamVrStatus& s, bool steamvrRunning);

} // namespace pibridge::app
