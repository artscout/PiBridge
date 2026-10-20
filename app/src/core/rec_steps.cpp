// SPDX-License-Identifier: MIT
#include "rec_steps.h"

#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <algorithm>
#include <cmath>

namespace pibridge::app {

namespace {

qint64 ms(double s) {
    return static_cast<qint64>(std::llround(s * 1000.0));
}

bool knownKind(const QString& k) {
    return k == QLatin1String("prepare") || k == QLatin1String("still") || k == QLatin1String("tilt-hold") ||
           k == QLatin1String("move");
}

} // namespace

std::vector<RecStep> defaultRecSteps() {
    // Everything with the headset on the head (laying the headset on a table in set poses did not work: it has to
    // be held by hand). Still holds at large tilts put gravity on the other axes.
    return {
        {QStringLiteral("put-on"), QStringLiteral("prepare"), QStringLiteral("Put the headset on. Get ready."), 8, 0},
        {QStringLiteral("still-mark"), QStringLiteral("still"),
         QStringLiteral("Stand on your mark. Look straight ahead. Hold still."), 10, 2},
        {QStringLiteral("tilt-left"), QStringLiteral("tilt-hold"),
         QStringLiteral("Tilt your head to the left shoulder and hold."), 6, 2},
        {QStringLiteral("tilt-right"), QStringLiteral("tilt-hold"), QStringLiteral("Now to the right shoulder and hold."), 6,
         2},
        {QStringLiteral("look-down"), QStringLiteral("tilt-hold"), QStringLiteral("Look down at the floor and hold."), 6, 2},
        {QStringLiteral("look-up"), QStringLiteral("tilt-hold"), QStringLiteral("Look up at the ceiling and hold."), 6, 2},
        {QStringLiteral("still-ahead"), QStringLiteral("still"), QStringLiteral("Look straight ahead. Hold still."), 5, 2},
        {QStringLiteral("step-away"), QStringLiteral("move"),
         QStringLiteral("Lean or step away and come back to the mark."), 20, 0},
        {QStringLiteral("back-on-mark"), QStringLiteral("still"), QStringLiteral("Back on the mark. Hold still."), 10, 2},
        {QStringLiteral("turn"), QStringLiteral("move"), QStringLiteral("Turn your head slowly left and right."), 12, 0},
        {QStringLiteral("hold-after-turn"), QStringLiteral("still"), QStringLiteral("Hold still."), 3, 1},
        {QStringLiteral("nod"), QStringLiteral("move"), QStringLiteral("Nod slowly."), 10, 0},
        {QStringLiteral("hold-after-nod"), QStringLiteral("still"), QStringLiteral("Hold still."), 3, 1},
        {QStringLiteral("shake"), QStringLiteral("move"), QStringLiteral("Shake your head quickly."), 5, 0},
        {QStringLiteral("back-on-mark-end"), QStringLiteral("still"), QStringLiteral("Back on the mark. Hold still."), 10,
         2},
    };
}

std::vector<RecStep> parseRecSteps(const QByteArray& json, QString* error) {
    const auto fail = [error](const QString& e) {
        if (error) {
            *error = e;
        }
        return std::vector<RecStep>{};
    };
    QJsonParseError pe;
    const QJsonDocument d = QJsonDocument::fromJson(json, &pe);
    if (!d.isObject()) {
        return fail(pe.error != QJsonParseError::NoError ? pe.errorString() : QStringLiteral("not a JSON object"));
    }
    const QJsonArray arr = d.object().value(QStringLiteral("steps")).toArray();
    if (arr.isEmpty()) {
        return fail(QStringLiteral("no \"steps\" list"));
    }
    std::vector<RecStep> out;
    int n = 0;
    for (const QJsonValue& v : arr) {
        n++;
        const QJsonObject o = v.toObject();
        RecStep s;
        s.id = o.value(QStringLiteral("id")).toString(QStringLiteral("step-%1").arg(n));
        s.kind = o.value(QStringLiteral("kind")).toString(QStringLiteral("move"));
        s.text = o.value(QStringLiteral("text")).toString().trimmed();
        s.seconds = o.value(QStringLiteral("seconds")).toDouble(0);
        s.settle = o.value(QStringLiteral("settle")).toDouble(0);
        if (!knownKind(s.kind)) {
            return fail(QStringLiteral("step %1: unknown kind \"%2\"").arg(n).arg(s.kind));
        }
        if (s.text.isEmpty()) {
            return fail(QStringLiteral("step %1: no text").arg(n));
        }
        if (!(s.seconds > 0) || s.settle < 0 || s.settle >= s.seconds) {
            return fail(QStringLiteral("step %1: seconds must be > 0 and settle below it").arg(n));
        }
        out.push_back(s);
    }
    return out;
}

double recStepsSeconds(const std::vector<RecStep>& steps) {
    double t = 0;
    for (const RecStep& s : steps) {
        t += s.seconds;
    }
    return t;
}

// ---------------------------------------------------------------- runner

void StepRunner::start(qint64 nowMs) {
    started_ = true;
    done_.clear();
    current_ = 0;
    stepStart_ = nowMs;
    finished_ = steps_.empty();
    if (finished_) {
        current_ = 0;
    }
}

qint64 StepRunner::stepEnd() const {
    return stepStart_ + ms(steps_[static_cast<size_t>(current_)].seconds);
}

void StepRunner::finishCurrent(qint64 endMs, bool skipped, bool cutShort) {
    done_.push_back({current_, stepStart_, endMs, skipped, cutShort});
    current_++;
    stepStart_ = endMs;
    if (current_ >= static_cast<int>(steps_.size())) {
        finished_ = true;
    }
}

bool StepRunner::tick(qint64 nowMs) {
    if (!running()) {
        return false;
    }
    bool changed = false;
    // a late tick still gives every step its planned window (the times stay on the plan, not on the tick)
    while (running() && nowMs >= stepEnd()) {
        finishCurrent(stepEnd(), false, false);
        changed = true;
    }
    return changed;
}

void StepRunner::skip(qint64 nowMs) {
    if (!running()) {
        return;
    }
    tick(nowMs);
    if (running()) {
        finishCurrent(nowMs, true, false);
    }
}

void StepRunner::stop(qint64 nowMs) {
    if (!running()) {
        return;
    }
    tick(nowMs);
    if (running()) {
        finishCurrent(nowMs, false, true);
    }
    finished_ = true;
}

const RecStep* StepRunner::currentStep() const {
    return running() ? &steps_[static_cast<size_t>(current_)] : nullptr;
}

const RecStep* StepRunner::nextStep() const {
    return running() && current_ + 1 < static_cast<int>(steps_.size()) ? &steps_[static_cast<size_t>(current_ + 1)]
                                                                         : nullptr;
}

int StepRunner::secondsLeft(qint64 nowMs) const {
    if (!running()) {
        return 0;
    }
    const qint64 left = stepEnd() - nowMs;
    return left <= 0 ? 0 : static_cast<int>((left + 999) / 1000);
}

// ---------------------------------------------------------------- steps.json

QJsonObject stepsJson(const StepsRecord& r, const StepRunner* runner) {
    // integers (Qt 6 keeps 64-bit integers exact in JSON): a double would round ns after ~104 days of uptime
    const auto ns = [&r](qint64 msOnClock) { return qint64(r.monoAtMs0Ns + msOnClock * 1000000); };
    QJsonObject o;
    o.insert(QStringLiteral("format"), QStringLiteral("pibridge-record-steps 1"));
    o.insert(QStringLiteral("mode"), r.mode);
    o.insert(QStringLiteral("clock"),
             QStringLiteral("CLOCK_MONOTONIC ns, the dataset's clock (EuRoC timestamps of imu0 and camN)"));
    o.insert(QStringLiteral("start_wall"), r.startWall);
    o.insert(QStringLiteral("record_start_ns"), ns(r.recordStartMs));
    if (r.recordEndMs >= 0) {
        o.insert(QStringLiteral("record_end_ns"), ns(r.recordEndMs));
    }
    o.insert(QStringLiteral("dataset"), r.dataset);
    if (r.imuFirstNs > 0) {
        o.insert(QStringLiteral("imu_first_ns"), r.imuFirstNs);
        o.insert(QStringLiteral("imu_last_ns"), r.imuLastNs);
        // The clocks match when the IMU's span overlaps the recording's (they are the same clock on one machine);
        // when not, offset_ns maps step times onto the dataset: dataset_ns = step_ns + offset_ns.
        const qint64 start = ns(r.recordStartMs);
        const qint64 end = r.recordEndMs >= 0 ? ns(r.recordEndMs) : start;
        constexpr qint64 kSlackNs = 60'000'000'000;
        const bool match = r.imuLastNs >= start - kSlackNs && r.imuFirstNs <= end + kSlackNs;
        o.insert(QStringLiteral("clock_matches_dataset"), match);
        o.insert(QStringLiteral("offset_ns"), match ? qint64(0) : r.imuFirstNs - start);
    }
    o.insert(QStringLiteral("stopped_early"), r.stoppedEarly);
    o.insert(QStringLiteral("end_reason"), r.endReason);
    o.insert(QStringLiteral("display_lease_retries"), r.leaseRetries);
    QJsonArray steps;
    QJsonArray notReached;
    if (runner) {
        const auto& all = runner->steps();
        for (const auto& d : runner->done()) {
            const RecStep& s = all[static_cast<size_t>(d.index)];
            QJsonObject j;
            j.insert(QStringLiteral("index"), d.index + 1);
            j.insert(QStringLiteral("id"), s.id);
            j.insert(QStringLiteral("kind"), s.kind);
            j.insert(QStringLiteral("text"), s.text);
            j.insert(QStringLiteral("planned_s"), s.seconds);
            j.insert(QStringLiteral("settle_s"), s.settle);
            j.insert(QStringLiteral("start_ns"), ns(d.startMs));
            j.insert(QStringLiteral("end_ns"), ns(d.endMs));
            if (s.kind == QLatin1String("still") || s.kind == QLatin1String("tilt-hold")) {
                const qint64 still = std::min(d.endMs, d.startMs + ms(s.settle));
                j.insert(QStringLiteral("still_from_ns"), ns(still));
            }
            if (r.imuFirstNs > 0) {
                j.insert(QStringLiteral("start_s"), double(ns(d.startMs) - r.imuFirstNs) / 1e9);
                j.insert(QStringLiteral("end_s"), double(ns(d.endMs) - r.imuFirstNs) / 1e9);
            }
            j.insert(QStringLiteral("skipped"), d.skipped);
            j.insert(QStringLiteral("cut_short"), d.cutShort);
            steps.append(j);
        }
        for (size_t i = runner->done().size(); i < all.size(); i++) {
            notReached.append(all[i].id);
        }
    }
    o.insert(QStringLiteral("steps"), steps);
    o.insert(QStringLiteral("not_reached"), notReached);
    return o;
}

bool writeStepsJson(const QString& path, const StepsRecord& r, const StepRunner* runner) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(QJsonDocument(stepsJson(r, runner)).toJson(QJsonDocument::Indented));
    return f.commit();
}

} // namespace pibridge::app
