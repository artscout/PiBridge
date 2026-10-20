// SPDX-License-Identifier: MIT
// The guided steps of "Record tracking": a timed script the app reads out (spd-say) and shows in big letters while
// the dataset is recorded, and steps.json, which tells the offline analysis which time window is which step.
//
// The protocol is data: the built-in table below (everything with the headset on the head), or a JSON file named by
// rec_steps= in pibridge.conf ({"steps": [{"id", "kind", "text", "seconds", "settle"}]}). Kinds:
//   prepare     not a measurement (putting the headset on)
//   still       head still in its normal pose
//   tilt-hold   head held still at a large tilt (gravity on other axes)
//   move        the head moves
// `settle` (seconds) is the part at the start of a still / tilt-hold step in which the user is still getting into the
// pose; steps.json gives the still window as still_from_ns..end_ns.
#pragma once

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <vector>

namespace pibridge::app {

struct RecStep {
    QString id;
    QString kind;   // prepare | still | tilt-hold | move
    QString text;   // shown and spoken
    double seconds = 0;
    double settle = 0;
};

// The built-in protocol (docs/DEVELOPMENT.md).
std::vector<RecStep> defaultRecSteps();
// A protocol from JSON text; empty with `error` set when it is broken (unknown kind, no text, seconds <= 0).
std::vector<RecStep> parseRecSteps(const QByteArray& json, QString* error);
// The total planned length in seconds.
double recStepsSeconds(const std::vector<RecStep>& steps);

// Runs the steps against an injected clock (ms). Pure: the page ticks it and reads what to show.
class StepRunner {
public:
    struct Done {
        int index = 0;
        qint64 startMs = 0;
        qint64 endMs = 0;
        bool skipped = false;   // Skip step
        bool cutShort = false;  // Stop during it
    };

    explicit StepRunner(std::vector<RecStep> steps) : steps_(std::move(steps)) {}
    void start(qint64 nowMs);
    // Advances past every step whose time is up; true when the current step changed (or the script ended).
    bool tick(qint64 nowMs);
    // Ends the current step now and goes on with the next.
    void skip(qint64 nowMs);
    // Ends the script now (the current step is marked cut short).
    void stop(qint64 nowMs);

    bool running() const { return started_ && !finished_; }
    bool finished() const { return finished_; }
    int current() const { return current_; } // -1 before start; steps().size() once finished
    const RecStep* currentStep() const;
    const RecStep* nextStep() const;
    // Whole seconds left in the current step (rounded up), 0 when none.
    int secondsLeft(qint64 nowMs) const;
    qint64 stepStartMs() const { return stepStart_; }
    const std::vector<RecStep>& steps() const { return steps_; }
    const std::vector<Done>& done() const { return done_; }

private:
    void finishCurrent(qint64 endMs, bool skipped, bool cutShort);
    qint64 stepEnd() const;

    std::vector<RecStep> steps_;
    std::vector<Done> done_;
    int current_ = -1;
    qint64 stepStart_ = 0;
    bool started_ = false;
    bool finished_ = false;
};

// What steps.json says (docs/DEVELOPMENT.md). Times on the app's CLOCK_MONOTONIC, which is the dataset's clock
// (Monado stamps the IMU and camera samples with os_monotonic_get_ns): `monoAtMs0Ns` is that clock at the session
// clock's 0 ms, so a step's start_ns = monoAtMs0Ns + startMs * 1e6.
struct StepsRecord {
    QString mode;            // "guided" | "free"
    QString startWall;       // ISO 8601 with offset, when the data started to come in
    qint64 monoAtMs0Ns = 0;
    qint64 recordStartMs = 0;  // data seen coming in
    qint64 recordEndMs = -1;   // stop sent (-1 while recording)
    qint64 imuFirstNs = 0, imuLastNs = 0; // from the dataset after the stop (0 = unknown)
    QString dataset;         // the euroc_<datetime> folder name
    bool stoppedEarly = false;
    QString endReason;       // "finished" | "stopped by the user" | the error
    int leaseRetries = 0;
};
QJsonObject stepsJson(const StepsRecord& r, const StepRunner* runner);
// Written atomically; false when it could not be.
bool writeStepsJson(const QString& path, const StepsRecord& r, const StepRunner* runner);

} // namespace pibridge::app
