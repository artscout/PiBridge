// SPDX-License-Identifier: MIT
// The SteamVR source of pibridge-record: `pibridge-record-mirror RING`, a background OpenVR client that reads SteamVR's
// mirror texture (IVRCompositor::GetMirrorTextureGL, one or both eyes) in a headless EGL/OpenGL context and feeds
// the same shared-memory ring Monado's tap feeds, so the recorder (run.cpp) and the encoder are the same for both.
//
// It is a leaf like the tap: it never waits for SteamVR (an asynchronous readback through pixel buffers; a frame is
// dropped and counted when every readback is still busy or the ring is full), it runs at nice 10, and it ends
// cleanly when the recorder asks, when the recorder stops answering (3 s), when SteamVR quits (VREvent_Quit, or
// vrserver gone) or on an OpenVR / GL error.
//
// The pump (pacing, the ring, the stop conditions) is separate from the OpenVR + GL backend so the tests drive it
// with a fake backend.
#pragma once

#include "pibridge/record/shm.h"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::record {

struct MirrorFrame {
    std::uint64_t seq = 0;
    std::uint64_t timestamp_ns = 0;
    std::span<const std::uint8_t> rgba; // width x height x 4, top row first; valid until release()
};

class MirrorBackend {
public:
    enum class Submit { Queued, Busy, NoImage, Failed };
    virtual ~MirrorBackend() = default;
    // Connect to SteamVR and get the mirror texture of `eyes` eyes. Empty on success, else why (plain words).
    virtual std::string init(std::uint32_t eyes) = 0;
    // The size of one eye's mirror texture.
    virtual std::pair<std::uint32_t, std::uint32_t> eyeSize() = 0;
    // Set up the readback for an output of `eyes` eyes side by side, each eyeW x eyeH. Empty on success.
    virtual std::string configure(std::uint32_t eyes, std::uint32_t eyeW, std::uint32_t eyeH) = 0;
    // Start reading the current mirror image (never waits). `error` gets the reason of a Failed.
    virtual Submit submit(std::uint64_t seq, std::uint64_t timestampNs, std::string& error) = 0;
    // The oldest finished readback, if any (never waits); release() it when copied.
    virtual std::optional<MirrorFrame> poll() = 0;
    virtual void release() = 0;
    // SteamVR asked its applications to quit (VREvent_Quit, acknowledged).
    virtual bool quitRequested() = 0;
    virtual std::string note() const = 0;
    // Why eyeSize() is 0 x 0, in plain words with the details for the log ("" when it is not).
    virtual std::string sizeProblem() const { return {}; }
};

struct MirrorOptions {
    // Is SteamVR still there? Asked about once a second (the real one: a vrserver process).
    std::function<bool()> steamvrAlive = [] { return true; };
    // Stop after this long without a consumer heartbeat (the tap's rule).
    double consumerTimeoutS = 3.0;
};

struct MirrorResult {
    int code = 0;       // 0: a clean end (asked, SteamVR quit, recorder gone); 3: could not start; 4: failed later
    std::string reason; // plain words
};

// Feed `ring` from `backend` at the ring's requested fps and size until something ends it. `stop`: set by SIGINT /
// SIGTERM.
MirrorResult runMirror(const Ring& ring, MirrorBackend& backend, const std::atomic<bool>& stop,
                       const MirrorOptions& opt = {});

// The OpenVR + EGL backend (mirror_openvr.cpp; only in pibridge-record-mirror). `library`: libopenvr_api.so.
std::unique_ptr<MirrorBackend> makeOpenVrMirror(const std::filesystem::path& library);
// libopenvr_api.so of the installed SteamVR: $PIBRIDGE_OPENVR_LIB, else <runtime>/bin/linux64/libopenvr_api.so with
// the runtime from $XDG_CONFIG_HOME/openvr/openvrpaths.vrpath (the first "runtime" entry; the app looks it up the
// same way). Empty when not found.
std::filesystem::path openVrLibrary();
// The first "runtime" path of an openvrpaths.vrpath text ("" when none).
std::string vrpathRuntime(std::string_view json);
// The order to try EGL devices in: the NVIDIA ones first (SteamVR's GL mirror is shared with the NVIDIA driver's
// OpenGL; a Mesa context on the same card - "failed to create dri2 screen" - gets texture names that mean nothing),
// then the others in their order. `vendors[i]`: EGL_VENDOR of device i ("" when it did not initialise). `forced`:
// PIBRIDGE_RECORD_EGL_DEVICE (an index; -1 = none).
std::vector<int> eglDeviceOrder(const std::vector<std::string>& vendors, int forced);
// The mirror's source size: the GL_TEXTURE_2D level 0 size, else the rectangle target's, else SteamVR's recommended
// render size (the blit then samples that rectangle of the texture). `how` says which one, for the log.
std::pair<std::uint32_t, std::uint32_t> chooseMirrorSize(std::pair<int, int> tex2d, std::pair<int, int> rect,
                                                         std::pair<std::uint32_t, std::uint32_t> recommended,
                                                         std::string& how);

// OpenVR's EVRCompositorError values in plain words (SharedTexturesNotSupported and friends).
std::string compositorErrorWords(int error);

} // namespace pibridge::record
