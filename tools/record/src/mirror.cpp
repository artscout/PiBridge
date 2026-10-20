// SPDX-License-Identifier: MIT
// The SteamVR mirror source's pump (pacing, the ring, the stop conditions) and the OpenVR lookups. New in PiBridge
// (omarchyvr records Monado only); the ring protocol is the tap's (comp_recorder.c).
#include "pibridge/record/mirror.h"

#include "pibridge_env.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>
#include <thread>

#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

MirrorResult runMirror(const Ring& ring, MirrorBackend& backend, const std::atomic<bool>& stop, const MirrorOptions& opt) {
    const ShmHeader& h = ring.hdr();
    const std::uint32_t eyes = std::clamp<std::uint32_t>(h.eyes, 1, 2);
    const std::uint32_t fps = h.fps == 0 ? 30 : h.fps;
    if (std::string e = backend.init(eyes); !e.empty()) {
        return {3, e};
    }
    const auto src = backend.eyeSize();
    if (src.first == 0 || src.second == 0) {
        const std::string why = backend.sizeProblem();
        return {3, "SteamVR's mirror texture has no size" + (why.empty() ? std::string() : " (" + why + ")")};
    }
    const auto ext = pickEyeExtent(src, h.max_height, h.scale_permille, eyes, h.slot_size);
    if (!ext) {
        return {3, "no output size fits the recorder's slots"};
    }
    if (std::string e = backend.configure(eyes, ext->first, ext->second); !e.empty()) {
        return {3, e};
    }
    const std::uint32_t w = ext->first * eyes, ht = ext->second;
    ring.attachProducer(w, ht, backend.note(), static_cast<std::uint32_t>(getpid()), src.first, src.second);

    const std::uint64_t interval = 1'000'000'000ull / fps;
    std::uint64_t next = monotonicNs();
    std::uint64_t seq = 0;
    auto lastAlive = std::chrono::steady_clock::now();
    MirrorResult res{0, "stopped"};
    for (;;) {
        if (stop.load() || ring.stopRequested()) {
            res = {0, "the recorder asked to stop"};
            break;
        }
        const std::uint64_t now = monotonicNs();
        const std::uint64_t hb = ring.load64(offsetof(ShmHeader, consumer_heartbeat_ns));
        if (hb != 0 && double(now - std::min(hb, now)) / 1e9 > opt.consumerTimeoutS) {
            res = {0, "the recorder stopped answering"};
            break;
        }
        if (backend.quitRequested()) {
            res = {0, "SteamVR is closing"};
            break;
        }
        if (std::chrono::steady_clock::now() - lastAlive >= 1s) {
            lastAlive = std::chrono::steady_clock::now();
            if (!opt.steamvrAlive()) {
                res = {0, "SteamVR ended"};
                break;
            }
        }
        ring.producerHeartbeat();
        // finished readbacks into the ring (a full ring drops the frame and counts it there)
        while (auto f = backend.poll()) {
            if (f->rgba.size() == std::size_t(w) * ht * 4) {
                ring.publish(w, ht, f->seq, f->timestamp_ns,
                             [&](std::span<std::uint8_t> dst) { std::memcpy(dst.data(), f->rgba.data(), dst.size()); });
            }
            backend.release();
        }
        if (now >= next) {
            std::string err;
            switch (backend.submit(seq, now, err)) {
            case MirrorBackend::Submit::Queued: seq++; break;
            case MirrorBackend::Submit::Busy: ring.countDroppedBusy(); break;
            case MirrorBackend::Submit::NoImage: ring.countSkipped(); break;
            case MirrorBackend::Submit::Failed: res = {4, err}; break;
            }
            if (res.code == 4) {
                break;
            }
            next += interval;
            if (next + interval < now) {
                next = now + interval; // fell behind (a stall): no burst of catch-up frames
            }
        }
        const std::uint64_t left = next > monotonicNs() ? next - monotonicNs() : 0;
        std::this_thread::sleep_for(std::chrono::nanoseconds(std::min<std::uint64_t>(left, 2'000'000)));
    }
    ring.detachProducer();
    return res;
}

std::vector<int> eglDeviceOrder(const std::vector<std::string>& vendors, int forced) {
    std::vector<int> order;
    const int n = static_cast<int>(vendors.size());
    if (forced >= 0 && forced < n) {
        return {forced};
    }
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < n; i++) {
            const bool nvidia = vendors[std::size_t(i)].find("NVIDIA") != std::string::npos;
            if (!vendors[std::size_t(i)].empty() && nvidia == (pass == 0)) {
                order.push_back(i);
            }
        }
    }
    return order;
}

std::pair<std::uint32_t, std::uint32_t> chooseMirrorSize(std::pair<int, int> tex2d, std::pair<int, int> rect,
                                                         std::pair<std::uint32_t, std::uint32_t> recommended,
                                                         std::string& how) {
    if (tex2d.first > 0 && tex2d.second > 0) {
        how = "texture size";
        return {std::uint32_t(tex2d.first), std::uint32_t(tex2d.second)};
    }
    if (rect.first > 0 && rect.second > 0) {
        how = "rectangle texture size";
        return {std::uint32_t(rect.first), std::uint32_t(rect.second)};
    }
    if (recommended.first > 0 && recommended.second > 0) {
        how = "SteamVR's recommended render size (the texture reports none)";
        return recommended;
    }
    how = "none";
    return {0, 0};
}

std::string vrpathRuntime(std::string_view j) {
    const std::size_t k = j.find("\"runtime\"");
    if (k == std::string_view::npos) {
        return {};
    }
    std::size_t i = j.find_first_not_of(" \t\r\n", k + 9);
    if (i == std::string_view::npos || j[i] != ':') {
        return {};
    }
    i = j.find_first_not_of(" \t\r\n", i + 1);
    if (i == std::string_view::npos || j[i] != '[') {
        return {};
    }
    i = j.find_first_not_of(" \t\r\n", i + 1);
    if (i == std::string_view::npos || j[i] != '"') {
        return {};
    }
    std::string out;
    for (i++; i < j.size() && j[i] != '"'; i++) {
        if (j[i] == '\\' && i + 1 < j.size()) {
            i++;
            switch (j[i]) {
            case 'n': out.push_back('\n'); break;
            case 't': out.push_back('\t'); break;
            default: out.push_back(j[i]); break; // \" \\ \/
            }
        } else {
            out.push_back(j[i]);
        }
    }
    while (out.size() > 1 && out.back() == '/') {
        out.pop_back();
    }
    return out;
}

fs::path openVrLibrary() {
    if (const char* p = pibridge::env("OPENVR_LIB"); p != nullptr && *p != '\0') {
        return p;
    }
    fs::path cfg;
    if (const char* x = std::getenv("XDG_CONFIG_HOME"); x != nullptr && *x != '\0') {
        cfg = x;
    } else if (const char* home = std::getenv("HOME"); home != nullptr) {
        cfg = fs::path(home) / ".config";
    }
    std::ifstream f(cfg / "openvr/openvrpaths.vrpath");
    if (!f) {
        return {};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string rt = vrpathRuntime(ss.str());
    if (rt.empty()) {
        return {};
    }
    const fs::path lib = fs::path(rt) / "bin/linux64/libopenvr_api.so";
    std::error_code ec;
    return fs::exists(lib, ec) ? lib : fs::path();
}

std::string compositorErrorWords(int e) {
    switch (e) {
    case 0: return "no error";
    case 1: return "SteamVR's compositor refused the request (VRCompositorError_RequestFailed)";
    case 100: return "this SteamVR does not match the OpenVR version (VRCompositorError_IncompatibleVersion)";
    case 101: return "the compositor wants the focus (VRCompositorError_DoNotHaveFocus)";
    case 102: return "the mirror texture is not valid (VRCompositorError_InvalidTexture)";
    case 103: return "SteamVR gives its mirror texture to scene applications only (VRCompositorError_IsNotSceneApplication)";
    case 104: return "the mirror texture is on another GPU (VRCompositorError_TextureIsOnWrongDevice)";
    case 105: return "the mirror texture has a format OpenGL cannot use (VRCompositorError_TextureUsesUnsupportedFormat)";
    case 106: return "SteamVR does not share its mirror texture with OpenGL here (VRCompositorError_SharedTexturesNotSupported)";
    case 107: return "no such eye (VRCompositorError_IndexOutOfRange)";
    default: return std::format("SteamVR compositor error {}", e);
    }
}

} // namespace pibridge::record
