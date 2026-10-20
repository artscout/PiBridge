// SPDX-License-Identifier: MIT
#include "runner.h"

#include <QProcess>

namespace pibridge::app {

void ProcessRunner::run(const QString& program, const QStringList& args, Done done) {
    auto* p = new QProcess(this);
    p->setProcessChannelMode(QProcess::MergedChannels);
    connect(p, &QProcess::finished, this, [p, done](int code, QProcess::ExitStatus st) {
        const QString out = QString::fromLocal8Bit(p->readAll());
        if (done) {
            done(st == QProcess::NormalExit ? code : -1, out);
        }
        p->deleteLater();
    });
    connect(p, &QProcess::errorOccurred, this, [p, done, program](QProcess::ProcessError e) {
        if (e != QProcess::FailedToStart) {
            return; // crashes end in finished()
        }
        if (done) {
            done(-1, QStringLiteral("Could not start %1: %2").arg(program, p->errorString()));
        }
        p->deleteLater();
    });
    p->start(program, args);
}

bool ProcessRunner::startDetached(const QString& program, const QStringList& args) {
    return QProcess::startDetached(program, args);
}

void FakeRunner::run(const QString& program, const QStringList& args, Done done) {
    calls.push_back({program, args, false});
    if (answer && done) {
        done(exitCode, output);
    }
}

bool FakeRunner::startDetached(const QString& program, const QStringList& args) {
    calls.push_back({program, args, true});
    return true;
}

} // namespace pibridge::app
