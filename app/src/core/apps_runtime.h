// SPDX-License-Identifier: MIT
// Which OpenXR runtime the Apps page starts own apps on, and when that runtime is ready.
//   SteamVR (Lighthouse mode, and the inside-out mode's SteamVR + Monado): the system's active runtime
//            (~/.config/openxr/1/active_runtime.json points at SteamVR and is never changed by PiBridge); the launcher
//            waits for "SteamVR fully ready" (core/steamvr_ready.h), as before.
//   Monado   (while "Monado directly" runs, Development page): that session's monado-service is the runtime.
//            An own app gets XR_RUNTIME_JSON=<that Monado build's openxr_monado-dev.json> (only in its own
//            environment; the system's active runtime stays SteamVR) and starts once the session runs (Monado stayed
//            up, its IPC socket $XDG_RUNTIME_DIR/monado_comp_ipc exists) and the home scene has its session. SteamVR
//            is never started in this mode. Steam games cannot run here: they need an OpenVR-to-OpenXR layer
//            (xrizer / OpenComposite), which is not installed, so their Play is off with the reason.
#pragma once

#include <QString>

#include "inside_out.h"
#include "rec_session.h"

namespace pibridge::app {

enum class AppTarget { SteamVr, Monado };


// Monado's IPC socket (XRT_IPC_MSG_SOCK_FILENAME of the Monado build: monado_comp_ipc) in `runtimeDir`.
inline constexpr char kMonadoSocketName[] = "monado_comp_ipc";
QString monadoSocketPath(const QString& runtimeDir);

// The OpenXR runtime file of the Monado build that holds `monadoService`: openxr_monado-dev.json next to it or up to
// four folders above (a build tree: <build>/src/xrt/targets/service/monado-service, the file in <build>/), else
// <prefix>/share/openxr/1/openxr_monado.json for an installed <prefix>/bin/monado-service. "" when none.
QString monadoRuntimeJsonFor(const QString& monadoService);
// libmonado.so of that build: the "MND_libmonado_path" of the runtime file, else
// <build>/src/xrt/targets/libmonado/libmonado.so, else <prefix>/lib/libmonado.so. "" when none.
QString libmonadoFor(const QString& monadoService);

// ---------------------------------------------------------------- is Monado ready for an app?

enum class MonadoStep {
    NoRuntimeFile, // the Monado build has no openxr_monado-dev.json: an app would not find it
    Off,           // the Tracking session does not run (Play starts it)
    Starting,      // it is starting (broker, tracker, Monado, the 5 s settle time)
    NoSocket,      // it runs but Monado's IPC socket is not there
    HomeStarting,  // the home scene is opening its session (an app started now could be hidden behind it)
    Failed,        // the session ended or failed
    Ready,
};

struct MonadoInputs {
    QString runtimeJson;     // monadoRuntimeJsonFor() of the session's monado-service
    RecState state = RecState::Idle; // the Tracking session (RecState::Recording = running)
    bool socket = false;     // Monado's IPC socket exists
    bool homeSettled = true; // the home scene is off, has its session, gave up, or had its time
    QString message;         // the session's own line (why it failed)
};

struct MonadoReadiness {
    MonadoStep step = MonadoStep::Off;
    QString words;       // plain words for the tile and the toolbar
    QString runtimeJson; // what XR_RUNTIME_JSON gets
    bool ready() const { return step == MonadoStep::Ready; }
};

// The first thing that does not hold (pure).
MonadoReadiness evaluateMonadoReady(const MonadoInputs& in);

// What the launcher asks about the runtime (TrackingRuntime over the Tracking session in the app, a fake in tests).
class AppRuntime {
public:
    virtual ~AppRuntime() = default;
    virtual AppTarget target() const = 0;
    // Monado's readiness now (only asked for AppTarget::Monado).
    virtual MonadoReadiness monado() const = 0;
    // Start the session of the target (Play while it is off): Monado directly for Monado, SteamVR + Monado for
    // SteamVR in the inside-out mode. False with `why` in plain words when it cannot start.
    virtual bool startTracking(QString* why) = 0;
    // SteamVR is started by PiBridge's tracking session (the inside-out mode), not through Steam (steam://rungameid).
    virtual bool steamVrThroughTracking() const { return false; }
};

// Why Steam games cannot be played in this target ("" = they can).
QString steamBlockedWhy(AppTarget t);

} // namespace pibridge::app
