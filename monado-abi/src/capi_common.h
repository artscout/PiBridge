// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/capi.rs (MIT OR Apache-2.0): the helpers every pimax_cam_* entry point shares
// (cam_capi.cpp, slam_capi.cpp, ae_capi.cpp): the thread-local error text, the catch_unwind-style guard, C string and calibration
// conversions.
#pragma once

#include "pimax_cam.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#define PIBRIDGE_ABI extern "C" __attribute__((visibility("default")))

// The camera handle (the header only declares it); cam_capi.cpp creates it, ae_capi.cpp reads its current frame.
struct pimax_cam {
    std::unique_ptr<pibridge::cam::Camera> cam;
};

namespace pibridge::abi {

/// pimax_cam_last_error()'s text (one per thread, as the Rust thread_local).
extern thread_local std::string g_last_error;

inline void set_error(std::string msg) {
    std::replace(msg.begin(), msg.end(), '\0', ' ');
    g_last_error = std::move(msg);
}

using Err = std::string;

/// catch_unwind + Result: the value, or the fallback with the error recorded.
template <class T, class F>
T guard(T fallback, F&& f) noexcept {
    try {
        std::expected<T, Err> r = f();
        if (r) {
            return *r;
        }
        set_error(std::move(r.error()));
    } catch (...) {
        try {
            set_error("panic inside pimax_cam");
        } catch (...) {
        }
    }
    return fallback;
}

inline std::unexpected<Err> err(std::string s) {
    return std::unexpected(std::move(s));
}

/// CStr::to_string_lossy
inline std::optional<std::string> opt_str(const char* p) {
    if (p == nullptr) {
        return std::nullopt;
    }
    return pibridge::cam::rfmt::utf8_lossy(p);
}

/// capi.rs from_c: the library's calibration from the C struct (names, uid and the raw Stateinit attributes are not
/// in it, so they come back empty).
pibridge::cam::DeviceCalib from_c(const pimax_cam_calib& c);

} // namespace pibridge::abi
