// SPDX-License-Identifier: MIT
// The OpenXR (XR_KHR_vulkan_enable2) side of pibridge-roomsetup: the session in STAGE space (LOCAL when the runtime has
// no stage), one swapchain per eye drawn with two small pipelines (triangles and lines of the scene lists), the
// controllers as ZoneTweak binds them (Index, Vive, Touch, simple controller) with haptics, and the reference space
// change events (a recentre) handed to the editor. Everything the user does is the editor's (pibridge/room/editor.h).
#pragma once

#include "pibridge/room/editor.h"
#include "mapflow.h"
#include "recenter.h"

#include <atomic>
#include <expected>
#include <optional>
#include <functional>
#include <string>

namespace pibridge::roomsetup {

struct XrOpts {
    unsigned resDiv = 1;
};

/// Called once the session exists, with the runtime's name and the haptics output (hand, ms, amplitude); returns the
/// editor to drive (it must outlive the run).
using Haptic = std::function<void(int hand, int ms, float amp)>;
using EditorFactory = std::function<room::Editor*(const std::string& runtimeName, Haptic haptic)>;

/// Run until the editor quits, the runtime ends the session or `stop` is set (SIGINT/SIGTERM).
std::expected<void, std::string> runXr(const EditorFactory& make, const XrOpts& opts, const std::atomic<bool>& stop);
/// The recenter (--recenter, --lh-stage): the same session and input, the flow's prompts drawn in front of the head.
/// `extra` (optional) draws more into each frame (--lh-stage: the base stations).
std::expected<void, std::string> runRecenterXr(RecenterFlow& flow, const XrOpts& opts, const std::atomic<bool>& stop,
                                               const std::function<std::optional<RingSample>()>& readRing,
                                               const std::function<void(room::Batch&)>& extra = {});

/// --map: the room mapping over the passthrough. The callbacks are main's: pibridge-roommap's state and answers, its
/// exit, the head ring.
struct MapLink {
    std::function<std::optional<guide::State>()> readState;
    std::function<std::vector<guide::P3f>()> readScanPoints; // the guide `points` file (scan cloud, room frame); optional
    std::function<void(const guide::Answer&)> writeAnswer;
    std::function<std::optional<int>()> childExit; // the exit code once pibridge-roommap has ended
    std::function<std::string()> childLastLine;
    std::function<std::optional<RingSample>()> readRing;
    std::string camera = "broker"; // the passthrough's frames; "off" for none
};
std::expected<void, std::string> runMapXr(MapFlow& flow, const MapLink& link, const XrOpts& opts, const std::atomic<bool>& stop);

} // namespace pibridge::roomsetup
