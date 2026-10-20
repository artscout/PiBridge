// SPDX-License-Identifier: MIT
// Every command the app runs goes through a Runner, so tests and the screenshot/fake mode inject FakeRunner and
// nothing is ever started. The real one is QProcess. Commands only run when the user clicks something.
#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <functional>
#include <vector>

namespace pibridge::app {

// exit code (-1 when it could not start or crashed) and the combined stdout+stderr.
using Done = std::function<void(int code, const QString& output)>;

class Runner {
public:
    virtual ~Runner() = default;
    // Run and wait asynchronously; `done` (may be empty) is called on the GUI thread when it ends.
    virtual void run(const QString& program, const QStringList& args, Done done) = 0;
    // Start and forget (SteamVR); false when it could not start.
    virtual bool startDetached(const QString& program, const QStringList& args) = 0;
};

class ProcessRunner : public QObject, public Runner {
    Q_OBJECT
public:
    explicit ProcessRunner(QObject* parent = nullptr) : QObject(parent) {}
    void run(const QString& program, const QStringList& args, Done done) override;
    bool startDetached(const QString& program, const QStringList& args) override;
};

// Records the calls and answers at once with `exitCode` / `output`.
class FakeRunner : public Runner {
public:
    struct Call {
        QString program;
        QStringList args;
        bool detached = false;
        QString line() const { return (QStringList{program} + args).join(QLatin1Char(' ')); }
    };
    void run(const QString& program, const QStringList& args, Done done) override;
    bool startDetached(const QString& program, const QStringList& args) override;

    std::vector<Call> calls;
    int exitCode = 0;
    QString output;
    bool answer = true; // false: keep the commands "running" (never call done)
};

} // namespace pibridge::app
