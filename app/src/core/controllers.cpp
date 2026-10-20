// SPDX-License-Identifier: MIT
#include "controllers.h"

#include <QFile>
#include <QSaveFile>
#include <utility>

namespace pibridge::app {

namespace {

bool writeAtomic(const QString& path, const QByteArray& text) {
    QSaveFile f(path); // a temporary name, renamed into place on commit
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(text);
    return f.commit();
}

int index(int slot) { return slot == kSlotLeft ? 0 : 1; }

} // namespace

QString sideName(int slot) { return slot == kSlotLeft ? QStringLiteral("left") : QStringLiteral("right"); }

ControllerOps::ControllerOps(QString statusDir, QString runtimeDir)
    : statusDir_(std::move(statusDir)), runtimeDir_(std::move(runtimeDir)) {}

QString ControllerOps::requestPath() const { return statusDir_ + QStringLiteral("/controller-request"); }
QString ControllerOps::hapticPath() const { return runtimeDir_ + QStringLiteral("/pimax-haptic-request"); }

ControllerOps::Side ControllerOps::side(int slot) const { return sides_[index(slot)]; }

void ControllerOps::set(int slot, const QString& text, bool error) {
    Side& s = sides_[index(slot)];
    s.text = text;
    s.error = error;
    s.busy = op_ != Op::None && slot == slot_;
}

void ControllerOps::finish(const QString& text, bool error) {
    const int slot = slot_;
    op_ = Op::None;
    set(slot, text, error);
}

QString ControllerOps::pair(int slot, const KeepaliveStatus& k, qint64 nowMs) { return start(Op::Pair, slot, k, nowMs); }
QString ControllerOps::unpair(int slot, const KeepaliveStatus& k, qint64 nowMs) {
    return start(Op::Unpair, slot, k, nowMs);
}
QString ControllerOps::testVibration(int slot, const KeepaliveStatus& k, qint64 nowMs) {
    return start(Op::Buzz, slot, k, nowMs);
}

QString ControllerOps::start(Op op, int slot, const KeepaliveStatus& k, qint64 nowMs) {
    if (slot != kSlotLeft && slot != kSlotRight) {
        return QStringLiteral("Unknown controller.");
    }
    const QString side = sideName(slot);
    const QString verb = op == Op::Pair ? QStringLiteral("pair") : QStringLiteral("unpair");
    QString why;
    const bool paused = k.running && k.state == QLatin1String("paused");
    if (op_ != Op::None) {
        why = QStringLiteral("Wait until the current controller action ends.");
    } else if (!k.running) {
        why = QStringLiteral("The headset service is not running: press Restart services on the Dashboard, then try "
                             "again.");
    } else if (paused && op != Op::Buzz) {
        why = QStringLiteral("Inside-out tracking is running and Monado's Pimax driver owns the headset. Stop "
                             "tracking first, then %1 the controller.")
                  .arg(verb);
    } else if (!paused && !k.headsetPresent) {
        why = QStringLiteral("No headset: switch the headset on and try again.");
    } else if (op == Op::Buzz && !paused && !(slot == kSlotLeft ? k.leftConnected : k.rightConnected)) {
        why = QStringLiteral("The %1 controller is not connected: switch it on (or pair it) and try again.").arg(side);
    } else if (op == Op::Pair && (slot == kSlotLeft ? k.leftConnected : k.rightConnected)) {
        why = QStringLiteral("The %1 controller is already paired and connected. Press Unpair first to pair it "
                             "again.")
                  .arg(side);
    }
    if (!why.isEmpty()) {
        set(slot, why, true);
        return why;
    }
    slot_ = slot;
    startMs_ = nowMs;
    acked_ = false;
    if (op == Op::Buzz) {
        monado_ = paused;
        hapticSeq_ = k.hapticSeq;
        const QByteArray line = QByteArray::number(slot) + ' ' + QByteArray::number(kTestPulseAmp) + ' ' +
                                QByteArray::number(kTestPulseMs) + '\n';
        if (!writeAtomic(hapticPath(), line)) {
            why = QStringLiteral("Could not write the vibration request (%1).").arg(hapticPath());
            set(slot, why, true);
            return why;
        }
        op_ = op;
        set(slot, QStringLiteral("Buzzing the %1 controller...").arg(side), false);
        return {};
    }
    id_ = std::max(nowMs, lastId_ + 1);
    lastId_ = id_;
    const QByteArray line = QByteArray::number(id_) + ' ' + verb.toLatin1() + ' ' + side.toLatin1() + '\n';
    if (!writeAtomic(requestPath(), line)) {
        why = QStringLiteral("Could not write the controller request (%1).").arg(requestPath());
        set(slot, why, true);
        return why;
    }
    op_ = op;
    set(slot, op == Op::Pair ? QStringLiteral("Starting the pairing...") : QStringLiteral("Unpairing..."), false);
    return {};
}

void ControllerOps::poll(const KeepaliveStatus& k, qint64 nowMs) {
    if (op_ == Op::None) {
        return;
    }
    const qint64 age = nowMs - startMs_;
    const QString side = sideName(slot_);
    if (op_ == Op::Buzz) {
        if (monado_) {
            if (!QFile::exists(hapticPath())) {
                return finish(QStringLiteral("Vibration sent to the %1 controller (through the VR session).").arg(side),
                              false);
            }
        } else if (k.running && k.hapticSeq > hapticSeq_) {
            return finish(k.hapticResult, !k.hapticResult.startsWith(QLatin1String("Vibration sent")));
        }
        if (age > kCtrlAckMs) {
            QFile::remove(hapticPath());
            return finish(monado_ ? QStringLiteral("The VR session did not take the vibration request. Is tracking "
                                                   "still running?")
                                  : QStringLiteral("The headset service did not answer. Press Restart services on "
                                                   "the Dashboard and try again."),
                          true);
        }
        return;
    }
    if (k.running && k.ctrlId == id_) {
        acked_ = true;
        if (k.ctrlState == QLatin1String("waiting")) {
            const qint64 left = std::max<qint64>(0, kPairTimeoutS - age / 1000);
            set(slot_, QStringLiteral("%1 (%2 s left)").arg(k.ctrlDetail).arg(left), false);
        } else {
            const bool ok = k.ctrlState == QLatin1String("paired") || k.ctrlState == QLatin1String("unpaired");
            return finish(k.ctrlDetail.isEmpty() ? k.ctrlState : k.ctrlDetail, !ok);
        }
    }
    if (!acked_ && age > kCtrlAckMs) {
        QFile::remove(requestPath());
        return finish(QStringLiteral("The headset service did not take the request. Press Restart services on the "
                                     "Dashboard and try again."),
                      true);
    }
    if (age > kPairMaxMs) {
        finish(QStringLiteral("No answer from the headset service in time. Try again."), true);
    }
}

} // namespace pibridge::app
