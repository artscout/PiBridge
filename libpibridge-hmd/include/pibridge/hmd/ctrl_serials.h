// SPDX-License-Identifier: MIT
// The controllers' own serial numbers (cmd 0x16, `58 04 16 SLOT 1e`, slot 2 left / 3 right: the first command of Pimax
// Play's controller attach burst). Whoever owns the headset (pibridge-keepalived, or pibridge-headtrack during
// inside-out) reads them once per controller connection and keeps them in
// $XDG_STATE_HOME/pibridge/controller-serials ("left SERIAL" / "right SERIAL" lines; ~/.local/state when the variable
// is missing), where the app shows them (SteamVR only knows CustomHeadsetOpenVR's names, "pimaxleft").
#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace pibridge::hmd::ctrl_serials {

std::filesystem::path path();
/// Keep `serial` for slot 0 (headset, "headset"), 2 / 3 (controllers) or 5 (the optical module, "optics"); the other
/// lines stay. False when it could not be written.
bool store(int slot, const std::string& serial);
/// The kept serial of slot 0, 2, 3 or 5.
std::optional<std::string> load(int slot);

} // namespace pibridge::hmd::ctrl_serials
