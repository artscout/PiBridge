// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/xrapp.rs and src/gpu.rs (the Vulkan pass) (MIT OR Apache-2.0).
//
// The OpenXR (XR_KHR_vulkan_enable2) application: session, one swapchain per eye drawn by the marks pass
// (shaders/marks.frag, one full-screen triangle per eye), the head-locked text as a VIEW-space quad layer (hud.h, so it
// is single and sharp with the canted displays), controller input (Valve Index: A next, B back / stop / save and
// exit, trigger start; Oculus Touch as in the Rust tool) and the terminal keys, and the end-to-end check through the
// runtime's XR_EXT_eye_gaze_interaction pose. The tracker, the procedure and the files are the Runner's.
#pragma once

#include "runner.h"

#include <atomic>
#include <cstdint>
#include <expected>
#include <string>

namespace pibridge::eyecal {

struct XrOpts {
    /// Render at 1/N of the recommended resolution.
    std::uint32_t res_div = 1;
};

/// Run the application until the procedure quits, the runtime ends the session or `stop` is set (SIGINT/SIGTERM).
/// The runner's tracker is started once the session exists.
std::expected<void, std::string> run_xr(Runner& runner, const XrOpts& opts, const std::atomic<bool>& stop);

} // namespace pibridge::eyecal
