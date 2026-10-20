// SPDX-License-Identifier: MIT
// The Device page's controller buttons: Pair, Unpair and Test vibration for each hand (docs/keepalived.md
// "Controllers"). No HID here: the app writes a request file and reads the answer from keepalived.json.
//
// - Pair / Unpair: one line `ID pair|unpair left|right` to statusDir()/controller-request (written to a temporary
//   name and renamed into place); pibridge-keepalived takes it on its session and answers with ctrl_id = ID and
//   ctrl_state / ctrl_detail. Refused before writing anything while an inside-out session runs (Monado's Pimax driver
//   owns the headset), without the service or without a headset.
// - Test vibration: `SLOT 60 300` (60 %, 300 ms) to $XDG_RUNTIME_DIR/pimax-haptic-request. Whoever holds the headset
//   takes it: pibridge-keepalived (its answer: haptic_seq / haptic_result) or, during an inside-out session, Monado's
//   Pimax driver (it removes the file; no other answer).
//
// One action at a time. The page calls poll() from a timer with the latest status; tests call it with made-up ones.
#pragma once

#include <QString>
#include <array>

#include "status.h"

namespace pibridge::app {

inline constexpr int kSlotLeft = 2;
inline constexpr int kSlotRight = 3;
// The test pulse: 60 %, 300 ms (Pimax Play's is 25 %, 300 ms; a little stronger so it is felt in the hand).
inline constexpr int kTestPulseAmp = 60;
inline constexpr int kTestPulseMs = 300;
// No answer to a request in this time: the service did not take it.
inline constexpr qint64 kCtrlAckMs = 3000;
// Pairing gives up on its own after 35 s; the app stops waiting a little later.
inline constexpr qint64 kPairMaxMs = 45000;
inline constexpr qint64 kPairTimeoutS = 35;

class ControllerOps {
public:
    // `statusDir`: $XDG_RUNTIME_DIR/pibridge (the request goes there), `runtimeDir`: $XDG_RUNTIME_DIR (the vibration file).
    ControllerOps(QString statusDir, QString runtimeDir);

    // Start an action for slot 2 (left) or 3 (right). "" when it started; else the plain reason (also shown for that
    // hand) and nothing was written.
    QString pair(int slot, const KeepaliveStatus& k, qint64 nowMs);
    QString unpair(int slot, const KeepaliveStatus& k, qint64 nowMs);
    QString testVibration(int slot, const KeepaliveStatus& k, qint64 nowMs);
    // Follow the running action with the latest status.
    void poll(const KeepaliveStatus& k, qint64 nowMs);

    struct Side {
        QString text;   // the last step or result, "" = nothing to say
        bool error = false;
        bool busy = false; // this hand's action runs
    };
    Side side(int slot) const;
    bool busy() const { return op_ != Op::None; }

    QString requestPath() const;
    QString hapticPath() const;

private:
    enum class Op { None, Pair, Unpair, Buzz };
    QString start(Op op, int slot, const KeepaliveStatus& k, qint64 nowMs);
    void finish(const QString& text, bool error);
    void set(int slot, const QString& text, bool error);

    QString statusDir_;
    QString runtimeDir_;
    Op op_ = Op::None;
    int slot_ = 0;
    qint64 id_ = 0;
    qint64 lastId_ = 0;
    qint64 startMs_ = 0;
    bool monado_ = false;     // the vibration goes to Monado's Pimax driver (inside-out session)
    qint64 hapticSeq_ = 0;    // keepalived's count before the request
    bool acked_ = false;
    std::array<Side, 2> sides_{};
};

// "left" / "right" for slot 2 / 3.
QString sideName(int slot);

} // namespace pibridge::app
