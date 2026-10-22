// SPDX-License-Identifier: MIT
// The app's read-only reader of the gaze shared memory /dev/shm/pibridge-eye-gaze (136 bytes, magic "OVEG",
// seqlock; layout of libpibridge-eye/include/pibridge/eye/shm.h and patches/chovr/pibridge_eye_gaze.h). The app only
// reads it for the live dot on the dashboard; it never creates or writes it.
#pragma once

#include <cstdint>
#include <optional>
#include <span>

namespace pibridge::app {

inline constexpr const char* kGazeShmPath = "/dev/shm/pibridge-eye-gaze";
inline constexpr uint32_t kGazeMagic = 0x4745564fu; // "OVEG" little endian
inline constexpr uint32_t kGazeVersion = 1;
inline constexpr size_t kGazeSize = 136;
inline constexpr uint32_t kGazeLeft = 1, kGazeRight = 2, kGazeCombined = 4;

struct GazeSnapshot {
    int64_t tNs = 0;       // sample time, CLOCK_MONOTONIC
    int64_t writtenNs = 0; // when it was published, CLOCK_MONOTONIC
    uint32_t flags = 0;
    float ipdMm = 0;
    float origin[3][3]{};    // left, right, combined
    float direction[3][3]{}; // head frame: x right, y up, -z forward
    uint64_t sample = 0;
    uint32_t state = 0; // 0 searching, 1 connecting, 2 streaming
};

// Decode a consistent copy of the whole file; nullopt when torn (seq_begin != seq_end, odd or 0), short or of another
// magic/version.
std::optional<GazeSnapshot> decodeGaze(std::span<const uint8_t> bytes);
// Read the file as readers must: seq_end, then the body, then seq_begin (separate preads), a few retries when torn.
std::optional<GazeSnapshot> readGaze(const char* path = kGazeShmPath);
// The gaze as tangents (x right, y up) from the combined direction, else the mean of the valid eyes; false when no
// direction is valid or it does not point forward.
bool gazeTangent(const GazeSnapshot& g, float& tx, float& ty);
// CLOCK_MONOTONIC now, ns.
int64_t monotonicNs();

} // namespace pibridge::app
