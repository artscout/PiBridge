// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/encode.rs (MIT OR Apache-2.0).
// The encoder: an `ffmpeg` child fed raw RGBA frames on stdin, writing a fragmented mp4 (playable at any moment,
// even if everything is killed). NVENC when available, libx264/libx265 on the CPU otherwise. The child runs at
// nice 10 with the lowest best-effort I/O priority so it never competes with the VR session.
#pragma once

#include "pibridge/record/settings.h"

#include <chrono>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace pibridge::record {

enum class Encoder { Nvenc, Cpu };
std::string_view encoderName(Encoder e, Codec c);

// What `ffmpeg -encoders` offers (probed once per recording).
struct Available {
    bool h264_nvenc = false, hevc_nvenc = false, libx264 = false, libx265 = false;
    static Available parse(std::string_view encoders);
    static std::optional<Available> probe(const std::filesystem::path& ffmpeg);
    bool nvenc(Codec c) const { return c == Codec::H264 ? h264_nvenc : hevc_nvenc; }
    bool cpu(Codec c) const { return c == Codec::H264 ? libx264 : libx265; }
    bool operator==(const Available&) const = default;
};

// The encoders to try, in order. `gpuOk`: false = NVENC is not used (omarchyvr's GPU gate; PiBridge has none and
// always passes true).
std::expected<std::vector<Encoder>, std::string> plan(EncoderPref pref, Codec codec, const Available& avail, bool gpuOk);
// The quality knob: constant quality (NVENC -cq, x264/x265 -crf) plus a bitrate ceiling.
std::vector<std::string> qualityArgs(Encoder enc, Codec codec, Quality q);

// PulseAudio source names (PipeWire's pulse server understands them).
inline constexpr char kMonitorSrc[] = "@DEFAULT_MONITOR@";
inline constexpr char kMicSrc[] = "@DEFAULT_SOURCE@";
// The audio inputs (ffmpeg inputs 1 and 2): the output's monitor, then the mic. Empty without audio.
std::vector<std::string> audioInputs(const Settings& s);
// Stream mapping and the audio codec: AAC 160k stereo 48 kHz (both sources mixed at full level).
std::vector<std::string> audioOutput(const Settings& s);
// A 0.3 s test read of a PulseAudio source: empty when it can be recorded, else why.
std::optional<std::string> probeAudio(const std::filesystem::path& ffmpeg, const std::string& src);
// The full ffmpeg command line (without argv[0]) for raw RGBA w x h frames on stdin -> out.
std::vector<std::string> ffmpegArgs(Encoder enc, const Settings& s, std::uint32_t w, std::uint32_t h,
                                    const std::filesystem::path& out);
// ffmpeg from $PIBRIDGE_FFMPEG / $PIMAX_FFMPEG or the PATH.
std::filesystem::path ffmpegPath();

// A running ffmpeg.
class Ffmpeg {
public:
    static std::expected<Ffmpeg, std::string> spawn(const std::filesystem::path& ffmpeg, Encoder enc, const Settings& s,
                                                    std::uint32_t w, std::uint32_t h, const std::filesystem::path& out,
                                                    const std::filesystem::path& log);
    Ffmpeg(Ffmpeg&& o) noexcept;
    Ffmpeg& operator=(Ffmpeg&& o) noexcept;
    Ffmpeg(const Ffmpeg&) = delete;
    Ffmpeg& operator=(const Ffmpeg&) = delete;
    ~Ffmpeg();
    // Write one frame (blocks while ffmpeg is behind: the ring then fills and the tap drops frames). False: the
    // pipe is closed (EPIPE).
    bool write(std::span<const std::uint8_t> rgba);
    int pid() const { return pid_; }
    // Exited already? (its status text)
    std::optional<std::string> exited();
    // Close stdin so ffmpeg writes the last fragment, wait up to `grace` (2 s with sound: a live sound input never
    // ends), then SIGINT, then SIGKILL 5 s later. How it ended ("ok" or the status).
    std::string finish(std::chrono::milliseconds grace);

    std::chrono::steady_clock::time_point started;
    Encoder encoder = Encoder::Cpu;
    std::filesystem::path out;
    std::uint64_t frames = 0;

private:
    Ffmpeg() = default;
    int pid_ = -1;
    int stdin_ = -1;
    bool audio_ = false;
    std::optional<int> status_;
};

} // namespace pibridge::record
