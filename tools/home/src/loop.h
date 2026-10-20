// SPDX-License-Identifier: MIT
// pibridge-home's session rules (pure; the XR loop follows them, the self test and the tests drive them by hand).
//
// Monado shows one primary application: the client whose session became active (xrBeginSession) last; when it ends,
// Monado falls back to another active session (ipc_server_process.c update_server_state_locked). So the home begins
// its session once, right after the Tracking session starts it, and a game started later is shown instead of it; when
// the game ends, the home is shown again. To never take the headset back from a game:
//   - xrBeginSession only at the first READY; a STOPPING is answered with xrEndSession and the program then leaves
//     (it never begins again and never makes a new session);
//   - while not VISIBLE (SYNCHRONIZED: another app is shown) it keeps the frame loop going (every frame the runtime
//     asks) but submits no layer and draws nothing;
//   - EXITING, LOSS_PENDING and an instance loss: leave at once;
//   - SIGTERM / SIGINT (the Tracking session's Stop): xrRequestExitSession once, then STOPPING -> end -> leave; when the
//     runtime does not answer within 3 s, leave anyway.
#pragma once

#include "pibridge_status.h"

#include <string>

namespace pibridge::home {

// XrSessionState, without the OpenXR header (the values are not used)
enum class XrState { Unknown, Idle, Ready, Synchronized, Visible, Focused, Stopping, LossPending, Exiting };

enum class Step { None, Begin, End, Leave };

inline constexpr double kExitWaitS = 3.0;

class Loop {
public:
    /// A session state change; what to call now.
    Step onState(XrState s);
    /// The instance is being lost.
    Step onInstanceLoss();
    /// Stop asked (a signal) at `nowS`: true when xrRequestExitSession should be called now (once, while running).
    bool requestStop(double nowS);
    /// Leave now (after End, a loss, or the stop wait ran out).
    bool shouldLeave(double nowS) const;

    bool running() const { return running_; }   // between xrBeginSession and xrEndSession
    bool visible() const { return visible_; }   // VISIBLE or FOCUSED: this is what the headset shows
    bool focused() const { return focused_; }
    /// Draw and submit the scene this frame (`shouldRender` from xrWaitFrame).
    bool submit(bool shouldRender) const { return running_ && visible_ && shouldRender; }
    /// For home.json: "starting" (no session yet), "running" (begun, another app is shown), "visible", "focused",
    /// "ending".
    std::string status() const;

private:
    bool begun_ = false;   // xrBeginSession was called once: never again
    bool running_ = false;
    bool visible_ = false;
    bool focused_ = false;
    bool ended_ = false;
    bool lost_ = false;
    bool stopAsked_ = false;
    double stopAtS_ = 0;
};

const char* stateName(XrState s);

/// home.json ($XDG_RUNTIME_DIR/pibridge/, common/pibridge_status.h): "state" (Loop::status()), "visible", "focused",
/// "runtime", "space" ("stage" | "local"), "room" (the active saved room or ""), "error" (plain words), "pid".
status::StatusWriter statusFields(const Loop& l, long long pid, const std::string& runtime, bool stage,
                                  const std::string& room, const std::string& error);

} // namespace pibridge::home
