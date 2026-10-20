// SPDX-License-Identifier: MIT
#include "loop.h"

namespace pibridge::home {

Step Loop::onState(XrState s) {
    switch (s) {
    case XrState::Ready:
        if (begun_ || ended_ || stopAsked_) {
            return Step::None; // once only: a second begin would make the home Monado's primary app again
        }
        begun_ = true;
        running_ = true;
        return Step::Begin;
    case XrState::Synchronized:
        visible_ = false;
        focused_ = false;
        return Step::None;
    case XrState::Visible:
        visible_ = true;
        focused_ = false;
        return Step::None;
    case XrState::Focused:
        visible_ = true;
        focused_ = true;
        return Step::None;
    case XrState::Stopping:
        visible_ = false;
        focused_ = false;
        if (!running_) {
            return Step::None;
        }
        running_ = false;
        ended_ = true;
        return Step::End;
    case XrState::LossPending:
    case XrState::Exiting:
        lost_ = true;
        running_ = false;
        visible_ = false;
        focused_ = false;
        return Step::Leave;
    case XrState::Idle:
    case XrState::Unknown: break;
    }
    return Step::None;
}

Step Loop::onInstanceLoss() {
    lost_ = true;
    running_ = false;
    visible_ = false;
    return Step::Leave;
}

bool Loop::requestStop(double nowS) {
    if (stopAsked_) {
        return false;
    }
    stopAsked_ = true;
    stopAtS_ = nowS;
    return running_;
}

bool Loop::shouldLeave(double nowS) const {
    if (lost_ || ended_) {
        return true;
    }
    if (stopAsked_ && (!begun_ || nowS - stopAtS_ >= kExitWaitS)) {
        return true; // never began, or the runtime did not answer the exit request
    }
    return false;
}

std::string Loop::status() const {
    if (ended_ || lost_ || stopAsked_) {
        return "ending";
    }
    if (!begun_) {
        return "starting";
    }
    return focused_ ? "focused" : visible_ ? "visible" : "running";
}

const char* stateName(XrState s) {
    switch (s) {
    case XrState::Idle: return "IDLE";
    case XrState::Ready: return "READY";
    case XrState::Synchronized: return "SYNCHRONIZED";
    case XrState::Visible: return "VISIBLE";
    case XrState::Focused: return "FOCUSED";
    case XrState::Stopping: return "STOPPING";
    case XrState::LossPending: return "LOSS_PENDING";
    case XrState::Exiting: return "EXITING";
    case XrState::Unknown: break;
    }
    return "UNKNOWN";
}

status::StatusWriter statusFields(const Loop& l, long long pid, const std::string& runtime, bool stage,
                                  const std::string& room, const std::string& error) {
    status::StatusWriter w;
    w.str("state", l.status())
        .boolean("visible", l.visible())
        .boolean("focused", l.focused())
        .str("runtime", runtime)
        .str("space", stage ? "stage" : "local")
        .str("room", room)
        .str("error", error)
        .integer("pid", pid);
    return w;
}

} // namespace pibridge::home
