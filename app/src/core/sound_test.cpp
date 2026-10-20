// SPDX-License-Identifier: MIT
#include "sound_test.h"

#include <QDir>
#include <QFile>
#include <QProcess>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <numbers>

#include <pibridge/hmd/audio.h>

namespace pibridge::app {

namespace {

QString pick(const QString& list, std::string_view prefix) {
    const std::string l = list.toStdString();
    const auto n = pibridge::hmd::audio::pick(l, prefix);
    return n ? QString::fromStdString(*n) : QString();
}

// The last non-empty line of a program's output, for an error message.
QString lastLine(const QString& out) {
    const QStringList lines = out.trimmed().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    return lines.isEmpty() ? QString() : lines.last().trimmed();
}

void putLe16(QByteArray& b, quint16 v) {
    char c[2];
    qToLittleEndian(v, c);
    b.append(c, 2);
}

void putLe32(QByteArray& b, quint32 v) {
    char c[4];
    qToLittleEndian(v, c);
    b.append(c, 4);
}

int16_t peakAbs(const QByteArray& s16le) {
    int peak = 0;
    const auto* p = reinterpret_cast<const uchar*>(s16le.constData());
    for (qsizetype i = 0; i + 1 < s16le.size(); i += 2) {
        const int v = qFromLittleEndian<qint16>(p + i);
        peak = std::max(peak, v < 0 ? -v : v);
    }
    return static_cast<int16_t>(std::min(peak, 32767));
}

const QString kPactlMissing = QStringLiteral(
    "pactl was not found. Install PipeWire's pulse tools (pipewire-pulse and pulseaudio-utils).");

} // namespace

QString pickHeadsetSink(const QString& pactlSinks) {
    return pick(pactlSinks, pibridge::hmd::audio::SINK_PREFIX);
}

QString pickHeadsetSource(const QString& pactlSources) {
    return pick(pactlSources, pibridge::hmd::audio::SOURCE_PREFIX);
}

QByteArray wavFile(const std::vector<int16_t>& samples, int rate, int channels) {
    const quint32 data = static_cast<quint32>(samples.size() * 2);
    QByteArray b;
    b.reserve(static_cast<qsizetype>(44 + data));
    b.append("RIFF", 4);
    putLe32(b, 36 + data);
    b.append("WAVEfmt ", 8);
    putLe32(b, 16);
    putLe16(b, 1); // PCM
    putLe16(b, static_cast<quint16>(channels));
    putLe32(b, static_cast<quint32>(rate));
    putLe32(b, static_cast<quint32>(rate * channels * 2));
    putLe16(b, static_cast<quint16>(channels * 2));
    putLe16(b, 16);
    b.append("data", 4);
    putLe32(b, data);
    for (int16_t s : samples) {
        putLe16(b, static_cast<quint16>(s));
    }
    return b;
}

std::vector<int16_t> stereoTestTone(int rate) {
    const int tone = rate * 7 / 10;
    const int gap = rate / 4;
    const int fade = rate / 50; // 20 ms
    std::vector<int16_t> out;
    out.reserve(static_cast<size_t>(2 * (2 * tone + gap)));
    const auto burst = [&](double hz, bool left) {
        for (int i = 0; i < tone; i++) {
            const double env = std::min({1.0, double(i) / fade, double(tone - 1 - i) / fade});
            const auto v = static_cast<int16_t>(std::lround(
                10000.0 * env * std::sin(2.0 * std::numbers::pi * hz * double(i) / double(rate))));
            out.push_back(left ? v : int16_t(0));
            out.push_back(left ? int16_t(0) : v);
        }
    };
    burst(440.0, true);
    out.insert(out.end(), static_cast<size_t>(2 * gap), int16_t(0));
    burst(660.0, false);
    return out;
}

double peakDb(const QByteArray& s16le) {
    const int peak = peakAbs(s16le);
    if (peak <= 0) {
        return -96.0;
    }
    return std::max(-96.0, 20.0 * std::log10(double(peak) / 32767.0));
}

float meterLevel(const QByteArray& s16le) {
    const double db = peakDb(s16le);
    return static_cast<float>(std::clamp((db + 60.0) / 60.0, 0.0, 1.0));
}

// ---------------------------------------------------------------- parec

ProcessCapture::~ProcessCapture() {
    stop();
}

bool ProcessCapture::start(const QString& source, int rate, Data data, Ended ended, QString& error) {
    stop();
    auto* p = new QProcess(this);
    p->setProcessChannelMode(QProcess::SeparateChannels);
    connect(p, &QProcess::readyReadStandardOutput, this, [p, data] {
        const QByteArray b = p->readAllStandardOutput();
        if (data && !b.isEmpty()) {
            data(b);
        }
    });
    connect(p, &QProcess::finished, this, [this, p, ended](int code, QProcess::ExitStatus st) {
        if (proc_ != p) {
            return; // stopped by us
        }
        proc_ = nullptr;
        const QString err = QString::fromLocal8Bit(p->readAllStandardError()).trimmed();
        p->deleteLater();
        if (ended) {
            ended(err.isEmpty() ? QStringLiteral("the recorder stopped (%1)")
                                      .arg(st == QProcess::NormalExit ? QString::number(code) : QStringLiteral("crashed"))
                                : err.split(QLatin1Char('\n')).last());
        }
    });
    p->start(QStringLiteral("parec"),
             {QStringLiteral("--device=") + source, QStringLiteral("--raw"), QStringLiteral("--format=s16le"),
              QStringLiteral("--rate=%1").arg(rate), QStringLiteral("--channels=1"), QStringLiteral("--latency-msec=50")});
    if (!p->waitForStarted(2000)) {
        error = QStringLiteral("parec was not found. Install PipeWire's pulse tools (pulseaudio-utils).");
        p->deleteLater();
        return false;
    }
    proc_ = p;
    return true;
}

void ProcessCapture::stop() {
    if (!proc_) {
        return;
    }
    QProcess* p = proc_;
    proc_ = nullptr; // its finished() is ignored from here on
    p->terminate();  // parec's own process only (our child), never by name
    if (!p->waitForFinished(1000)) {
        p->kill();
        p->waitForFinished(500);
    }
    p->deleteLater();
}

// ---------------------------------------------------------------- fake capture

bool FakeCapture::start(const QString& src, int r, Data data, Ended ended, QString& error) {
    starts++;
    source = src;
    rate = r;
    if (!startOk) {
        error = startError;
        return false;
    }
    data_ = std::move(data);
    ended_ = std::move(ended);
    running = true;
    return true;
}

void FakeCapture::stop() {
    stops++;
    running = false;
    data_ = {};
    ended_ = {};
}

void FakeCapture::fail(const QString& e) {
    running = false;
    auto end = std::move(ended_);
    data_ = {};
    ended_ = {};
    if (end) {
        end(e);
    }
}

// ---------------------------------------------------------------- the tests

SoundTest::SoundTest(Runner& runner, AudioCapture& capture, QString workDir, bool autoTimer, QObject* parent)
    : QObject(parent), runner_(runner), capture_(capture), dir_(std::move(workDir)), autoTimer_(autoTimer) {
    timer_.setSingleShot(true);
    timer_.setInterval(kMicSeconds * 1000);
    connect(&timer_, &QTimer::timeout, this, &SoundTest::finishRecording);
}

SoundTest::~SoundTest() {
    if (state_ == State::Recording) {
        capture_.stop();
    }
    QFile::remove(micFile());
}

QString SoundTest::speakerFile() const {
    return dir_ + QStringLiteral("/sound-test-speakers.wav");
}

QString SoundTest::micFile() const {
    return dir_ + QStringLiteral("/sound-test-microphone.wav");
}

void SoundTest::set(State s, const QString& text, bool error) {
    state_ = s;
    message_ = text;
    error_ = error;
    emit changed();
}

void SoundTest::fail(const QString& text) {
    if (level_ != 0) {
        level_ = 0;
        emit levelChanged(0);
    }
    set(State::Idle, text, true);
}

void SoundTest::testSpeakers() {
    if (busy()) {
        return;
    }
    set(State::Finding, QStringLiteral("Looking for the headset's speakers..."));
    findNodes(false);
}

void SoundTest::testMicrophone() {
    if (busy()) {
        return;
    }
    set(State::Finding, QStringLiteral("Looking for the headset's microphone..."));
    findNodes(true);
}

void SoundTest::findNodes(bool needSource) {
    std::weak_ptr<int> alive = alive_;
    runner_.run(QStringLiteral("pactl"), {QStringLiteral("list"), QStringLiteral("short"), QStringLiteral("sinks")},
                [this, alive, needSource](int code, const QString& sinks) {
                    if (alive.expired()) {
                        return;
                    }
                    if (code != 0) {
                        fail(code < 0 ? kPactlMissing
                                      : QStringLiteral("The sound server did not answer: %1").arg(lastLine(sinks)));
                        return;
                    }
                    if (!needSource) {
                        found(sinks, {}, false);
                        return;
                    }
                    runner_.run(QStringLiteral("pactl"),
                                {QStringLiteral("list"), QStringLiteral("short"), QStringLiteral("sources")},
                                [this, alive, sinks](int c2, const QString& sources) {
                                    if (alive.expired()) {
                                        return;
                                    }
                                    if (c2 != 0) {
                                        fail(c2 < 0 ? kPactlMissing
                                                    : QStringLiteral("The sound server did not answer: %1")
                                                          .arg(lastLine(sources)));
                                        return;
                                    }
                                    found(sinks, sources, true);
                                });
                });
}

void SoundTest::found(const QString& sinks, const QString& sources, bool needSource) {
    const QString sink = pickHeadsetSink(sinks);
    const QString source = needSource ? pickHeadsetSource(sources) : QString();
    if (needSource && source.isEmpty()) {
        fail(QStringLiteral("Headset microphone not found. Turn the headset on (screen on) and check its USB cable."));
        return;
    }
    if (sink.isEmpty()) {
        fail(needSource ? QStringLiteral("Headset speakers not found (the recording is played back on them). Turn the "
                                         "headset on (screen on) and check its USB cable.")
                        : QStringLiteral("Headset speakers not found. Turn the headset on (screen on) and check its "
                                         "USB cable."));
        return;
    }
    if (needSource) {
        startRecording(source, sink);
    } else {
        playTone(sink);
    }
}

void SoundTest::playTone(const QString& sink) {
    QDir().mkpath(dir_);
    QFile f(speakerFile());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate) || f.write(wavFile(stereoTestTone(), kSoundRate, 2)) < 0) {
        fail(QStringLiteral("Could not write the test sound to %1").arg(speakerFile()));
        return;
    }
    f.close();
    set(State::Speakers, QStringLiteral("Playing in the headset: a low tone on the left, then a high one on the right..."));
    std::weak_ptr<int> alive = alive_;
    runner_.run(QStringLiteral("paplay"), {QStringLiteral("--device=") + sink, speakerFile()},
                [this, alive](int code, const QString& out) {
                    if (alive.expired()) {
                        return;
                    }
                    QFile::remove(speakerFile());
                    if (code != 0) {
                        fail(code < 0 ? QStringLiteral("paplay was not found. Install PipeWire's pulse tools "
                                                       "(pulseaudio-utils).")
                                      : QStringLiteral("Playback failed: %1").arg(lastLine(out)));
                        return;
                    }
                    set(State::Idle, QStringLiteral("Done. You should have heard the low tone on the left and the high "
                                                    "one on the right. Nothing? Check the headset's volume in the "
                                                    "system sound settings."));
                });
}

void SoundTest::startRecording(const QString& source, const QString& sink) {
    recorded_.clear();
    peak_ = -96;
    playSink_ = sink;
    std::weak_ptr<int> alive = alive_;
    QString err;
    const bool ok = capture_.start(
        source, kSoundRate,
        [this, alive](const QByteArray& b) {
            if (alive.expired() || state_ != State::Recording) {
                return;
            }
            recorded_.append(b);
            peak_ = std::max(peak_, peakDb(b));
            level_ = meterLevel(b);
            emit levelChanged(level_);
        },
        [this, alive](const QString& e) {
            if (alive.expired() || state_ != State::Recording) {
                return;
            }
            timer_.stop();
            fail(QStringLiteral("Recording stopped: %1").arg(e));
        },
        err);
    if (!ok) {
        fail(err);
        return;
    }
    set(State::Recording, QStringLiteral("Recording %1 s: speak into the headset's microphone...").arg(kMicSeconds));
    if (autoTimer_) {
        timer_.start();
    }
}

void SoundTest::finishRecording() {
    if (state_ != State::Recording) {
        return;
    }
    timer_.stop();
    capture_.stop();
    level_ = 0;
    emit levelChanged(0);
    if (recorded_.size() < kSoundRate / 10 * 2) { // less than 0.1 s of sound arrived
        fail(QStringLiteral("The headset's microphone sent no sound. Is it muted in the system sound settings?"));
        return;
    }
    QDir().mkpath(dir_);
    std::vector<int16_t> samples(static_cast<size_t>(recorded_.size() / 2));
    const auto* p = reinterpret_cast<const uchar*>(recorded_.constData());
    for (size_t i = 0; i < samples.size(); i++) {
        samples[i] = qFromLittleEndian<qint16>(p + 2 * i);
    }
    recorded_.clear(); // the sound stays only in the temporary file, deleted after the playback
    QFile f(micFile());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate) || f.write(wavFile(samples, kSoundRate, 1)) < 0) {
        fail(QStringLiteral("Could not write the recording to %1").arg(micFile()));
        return;
    }
    f.close();
    const double peak = peak_;
    set(State::Playback, QStringLiteral("Playing the recording back in the headset..."));
    std::weak_ptr<int> alive = alive_;
    runner_.run(QStringLiteral("paplay"), {QStringLiteral("--device=") + playSink_, micFile()},
                [this, alive, peak](int code, const QString& out) {
                    if (alive.expired()) {
                        return;
                    }
                    QFile::remove(micFile());
                    if (code != 0) {
                        fail(code < 0 ? QStringLiteral("paplay was not found. Install PipeWire's pulse tools "
                                                       "(pulseaudio-utils).")
                                      : QStringLiteral("Playback failed: %1").arg(lastLine(out)));
                        return;
                    }
                    const QString level = QStringLiteral("Loudest: %1 dB.").arg(qRound(peak));
                    if (peak < -40) {
                        set(State::Idle, QStringLiteral("Done, but the microphone was very quiet (%1). Is it muted or "
                                                        "turned down in the system sound settings?")
                                             .arg(level.chopped(1)), true);
                    } else {
                        set(State::Idle, QStringLiteral("Done: you should have heard yourself in the headset. ") + level);
                    }
                });
}

} // namespace pibridge::app
