// SPDX-License-Identifier: MIT
// What the libpimax (pimax_capi.cpp) and eye (eye_capi.cpp) ABIs share: the HID allowlist of a model. Both send only
// through it (docs/monado-abi.md, "libpimax").
#pragma once

#include "pibridge/hmd/allowlist.h"
#include "pibridge/hmd/proto.h"

#include <expected>
#include <memory>
#include <string_view>

namespace pibridge::abi {

/// The description `devices/<id>.toml` compiled into the archive (libpimax.cmake); empty for an unknown id.
std::string_view embedded_device_toml(std::string_view id);

/// The allowlist of `model`: the PiBridge devices folders (PIBRIDGE_DEVICES_DIR, the installed descriptions), else the
/// copy compiled into the archive.
std::expected<std::shared_ptr<const pibridge::hmd::Allowlist>, pibridge::hmd::SpecError>
device_allowlist(const pibridge::hmd::Model& model);

} // namespace pibridge::abi
