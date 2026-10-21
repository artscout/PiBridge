// SPDX-License-Identifier: MIT
// The capture CONTROL file: `$XDG_RUNTIME_DIR/pibridge/record-capture`. `pibridge-record start|stop` writes it; the
// Vulkan capture layer (tools/record-layer, inside vrcompositor) polls it. Plain `key value` lines, one per line:
//
//   active 1              # 0 = off (the layer stays a pass-through and captures nothing)
//   view both            # left | right | both (which of the game's eyes to capture, from record.toml's `view`)
//   fps 60               # the pace the layer blits at
//   max_height 1080      # the captured image is scaled down so its height is at most this (aspect kept)
//
// Separate from record-capture.sock (the fd/frame IPC, pibridge_record_capture_ipc.h): the control file turns capture
// on/off and sets its parameters; the socket carries the shared image and per-frame signals.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

namespace pibridge::reclayer {

// `$XDG_RUNTIME_DIR/pibridge/record-capture` (else `/tmp/pibridge-record-capture`).
inline std::string captureControlPath() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    return (rt != nullptr && rt[0] != '\0') ? std::string(rt) + "/pibridge/record-capture"
                                            : std::string("/tmp/pibridge-record-capture");
}

// `$XDG_RUNTIME_DIR/pibridge/record-capture.sock` (else `/tmp/pibridge-record-capture.sock`).
inline std::string captureSocketPath() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    return (rt != nullptr && rt[0] != '\0') ? std::string(rt) + "/pibridge/record-capture.sock"
                                            : std::string("/tmp/pibridge-record-capture.sock");
}

} // namespace pibridge::reclayer
