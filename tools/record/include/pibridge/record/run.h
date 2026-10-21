// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/run.rs (MIT OR Apache-2.0).
// `pibridge-record run`: the recorder process (pid file `record.pid` in the session dir).
//
// Never hurts the session: it only reads the shared-memory ring (Monado's tap drops frames instead of waiting), runs
// at nice 10 with the lowest best-effort I/O priority, and every way out (SIGINT/SIGTERM, Monado gone, disk almost
// full, encoder error) closes the encoder's input so the fragmented mp4 is finished and playable. A clean stop for
// any of those reasons exits 0; only a real crash is non-zero.
#pragma once

#include "pibridge/record/settings.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace pibridge::record {

// Where pibridge-record keeps its files. Tests point every one at a temporary directory with the variables.
struct Roots {
    std::filesystem::path home;    // $HOME
    std::filesystem::path session; // $PIBRIDGE_VR_SESSION_DIR / $PIMAX_VR_SESSION_DIR, else <runtime>/pibridge
    std::filesystem::path runtime; // $XDG_RUNTIME_DIR, else /tmp
    std::filesystem::path config;  // $XDG_CONFIG_HOME/pibridge (~/.config/pibridge)
    std::filesystem::path legacyConfig; // $XDG_CONFIG_HOME/omarchyvr: its record.toml is read while PiBridge's is missing
    std::filesystem::path proc;    // procRoot()
    static Roots fromEnv();
};

// Frames per file second -> how many times each delivered frame is written (constant frame rate output).
class Pacer {
public:
    // A gap is filled with at most this many copies of the next frame; a longer gap is cut out.
    static constexpr std::uint64_t kMaxCopies = 3;
    explicit Pacer(std::uint32_t fps) : fps_(fps) {}
    // How many times to write the frame shown at ts_ns (0: it came too early, skip it).
    std::uint64_t copies(std::uint64_t tsNs);
    std::uint64_t duplicated = 0;
    std::uint64_t early = 0;
    double lost_s = 0; // seconds not represented in the file (gaps longer than kMaxCopies frames)

private:
    std::uint32_t fps_;
    bool haveT0_ = false;
    std::uint64_t t0_ = 0;
    std::uint64_t next_ = 0;
};

// Where the frames come from:
//  - Tap: Monado's tap (comp_recorder.c, "Monado directly"), via the shared-memory ring.
//  - Mirror: SteamVR's GL mirror texture (pibridge-record-mirror), via the same ring. Crashes vrserver on
//    NVIDIA/Vulkan, so it is no longer the SteamVR default (kept for anyone who wants it).
//  - Layer: PiBridge's Vulkan capture layer (tools/record-layer), which runs inside vrcompositor and shares the eye
//    image over a unix socket (pibridge_record_capture_ipc.h). The default under SteamVR. runLayer() is the encoder.
enum class Source { Tap, Mirror, Layer };
std::string_view name(Source s);
std::optional<Source> parseSource(std::string_view s); // tap | monado, mirror | steamvr, layer | vk
// pibridge-record-mirror: $PIBRIDGE_RECORD_MIRROR_BIN, else next to this executable, else on the PATH.
std::filesystem::path mirrorBin();

// Is monado-service running (a live process with that comm)?
bool monadoRunning(const Roots& r);
// Is SteamVR's vrserver running? (Under SteamVR, SteamVR's compositor draws the headset image: the tap sees nothing.)
bool steamvrRunning(const Roots& r);

// Refuse to start below this; stop when the free space drops below kStopFree.
inline constexpr std::uint64_t kStartFree = 2ull << 30;
inline constexpr std::uint64_t kStopFree = 1ull << 30;

// A file name in `dir` that does not exist yet (name, name-2, name-3, ...).
std::filesystem::path uniquePath(const std::filesystem::path& dir, const std::string& name);

// The recorder. Returns the exit code. For Source::Layer it hands off to runLayer().
int run(const Settings& s, const Roots& r, Source source = Source::Tap);

// The encoder for Source::Layer (capture_client.cpp): connect to the Vulkan capture layer's socket
// ($XDG_RUNTIME_DIR/pibridge/record-capture.sock), import the shared eye image, read it back per frame and encode it
// through the same ffmpeg pipeline as the tap/mirror path. Returns the exit code. Built only with Vulkan; without it,
// it fails cleanly ("this build has no Vulkan capture support").
int runLayer(const Settings& s, const Roots& r);

} // namespace pibridge::record
