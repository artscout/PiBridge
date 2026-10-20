// SPDX-License-Identifier: MIT
// The Dashboard's sound tests, like Pimax Play / EVO: TEST SPEAKERS plays a short tone on the left, then on the right
// ear of the headset; TEST MICROPHONE records 3 s from the headset's microphone (with a live level meter) and plays
// it back on the headset. Both go straight to the headset's PipeWire nodes (found by the same name prefixes as
// pibridge-keepalived's audio switch, libpibridge-hmd audio.h); the default output and input are never changed.
//
// Programs (PipeWire's pulse tools): `pactl list short sinks|sources` (find the nodes), `paplay --device=<sink>
// <file.wav>` (play), `parec --device=<source> --raw ...` (record, streamed so the meter moves). pactl and paplay go
// through the Runner, parec through an AudioCapture; tests use fakes of both, so nothing is ever played or recorded.
// The recording lives only in a temporary WAV under $XDG_RUNTIME_DIR/pibridge and is deleted after the playback.
#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QTimer>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "runner.h"

class QProcess;

namespace pibridge::app {

inline constexpr int kSoundRate = 48000;     // the headset's USB audio runs at 48 kHz
inline constexpr int kMicSeconds = 3;

// The headset's nodes in `pactl list short sinks` / `... sources` output ("" when not there; monitors left out).
QString pickHeadsetSink(const QString& pactlSinks);
QString pickHeadsetSource(const QString& pactlSources);

// A 16-bit PCM WAV file of interleaved `samples`.
QByteArray wavFile(const std::vector<int16_t>& samples, int rate, int channels);
// The speaker test: 0.7 s of 440 Hz on the left only, 0.25 s of silence, 0.7 s of 660 Hz on the right only (stereo,
// soft start and end so it does not click), at about a third of full scale.
std::vector<int16_t> stereoTestTone(int rate = kSoundRate);
// The meter value of a block of 16-bit little-endian samples: its peak on a -60..0 dBFS scale as 0..1.
float meterLevel(const QByteArray& s16le);
// The peak of a block in dBFS (-96 for silence or no data).
double peakDb(const QByteArray& s16le);

// A streamed recording (parec). Chunks and the end arrive on the GUI thread.
class AudioCapture {
public:
    virtual ~AudioCapture() = default;
    using Data = std::function<void(const QByteArray&)>;
    using Ended = std::function<void(const QString& error)>; // "" when stopped by stop()
    // Starts recording mono 16-bit at `rate` from `source`; false (with `error`) when it could not start.
    virtual bool start(const QString& source, int rate, Data data, Ended ended, QString& error) = 0;
    // Ends the recording; `ended` is not called after this.
    virtual void stop() = 0;
};

// parec through QProcess.
class ProcessCapture : public QObject, public AudioCapture {
    Q_OBJECT
public:
    explicit ProcessCapture(QObject* parent = nullptr) : QObject(parent) {}
    ~ProcessCapture() override;
    bool start(const QString& source, int rate, Data data, Ended ended, QString& error) override;
    void stop() override;

private:
    QProcess* proc_ = nullptr;
};

// For tests: records what was asked; feed() hands data, fail() ends with an error.
class FakeCapture : public AudioCapture {
public:
    bool start(const QString& source, int rate, Data data, Ended ended, QString& error) override;
    void stop() override;
    void feed(const QByteArray& b) { if (data_) data_(b); }
    void fail(const QString& e);

    bool startOk = true;
    QString startError = QStringLiteral("parec is not installed");
    QString source;
    int rate = 0;
    int starts = 0;
    int stops = 0;
    bool running = false;

private:
    Data data_;
    Ended ended_;
};

class SoundTest : public QObject {
    Q_OBJECT
public:
    enum class State { Idle, Finding, Speakers, Recording, Playback };
    // `workDir`: where the WAV files go ($XDG_RUNTIME_DIR/pibridge). `autoTimer` false: the tests end the recording by
    // calling finishRecording() instead of waiting kMicSeconds.
    SoundTest(Runner& runner, AudioCapture& capture, QString workDir, bool autoTimer = true, QObject* parent = nullptr);
    ~SoundTest() override;

    void testSpeakers();
    void testMicrophone();
    // Ends the recording now and plays it back (the 3 s timer calls it).
    void finishRecording();

    State state() const { return state_; }
    bool busy() const { return state_ != State::Idle; }
    QString message() const { return message_; } // plain words of the last step or result
    bool error() const { return error_; }
    float level() const { return level_; }       // the meter while recording, 0..1
    QString speakerFile() const;
    QString micFile() const;

signals:
    void changed();
    void levelChanged(float level);

private:
    void findNodes(bool needSource);
    void found(const QString& sinks, const QString& sources, bool needSource);
    void playTone(const QString& sink);
    void startRecording(const QString& source, const QString& sink);
    void fail(const QString& text);
    void set(State s, const QString& text, bool error = false);

    Runner& runner_;
    AudioCapture& capture_;
    QString dir_;
    bool autoTimer_;
    State state_ = State::Idle;
    QString message_;
    bool error_ = false;
    float level_ = 0;
    double peak_ = -96;
    QByteArray recorded_;
    QString playSink_;
    QTimer timer_;
    std::shared_ptr<int> alive_ = std::make_shared<int>(0); // callbacks after destruction do nothing
};

} // namespace pibridge::app
