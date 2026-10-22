// SPDX-License-Identifier: MIT
//
// Motion-JPEG -> grey, the decode half of the Crystal Super camera profile (v4l2.h). On a 480 Mbps USB link the
// tracking cameras report MJPEG and no YUYV, so every captured buffer is one JPEG image; its luma plane is the two
// tracking cameras, which is what the rest of the pipeline wants as grey pixels.
#pragma once

#include "pibridge/cam/io.h"

#include <cstdint>
#include <span>

namespace pibridge::cam {

/// Decode one JPEG image into `out` as grey (the luma plane, one byte per pixel). `w`/`h` come back as the decoded
/// size; `out` must hold w*h bytes. Error when the bytes are not a JPEG, when the image is empty, or when `out` is
/// too small for it.
Result<void> decode_grey_jpeg(std::span<const std::uint8_t> jpeg, std::span<std::uint8_t> out, std::uint32_t& w,
                              std::uint32_t& h);

/// True when the bytes start with a JPEG image marker (SOI, FF D8). Cheap first check on a captured buffer.
bool looks_like_jpeg(std::span<const std::uint8_t> bytes);

} // namespace pibridge::cam
