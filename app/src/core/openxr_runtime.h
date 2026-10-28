// SPDX-License-Identifier: MIT
// The system's active OpenXR runtime (the Dashboard's Runtime switch): $XDG_CONFIG_HOME/openxr/1/active_runtime.json
// (default ~/.config/openxr/1/active_runtime.json), which the OpenXR loader of every app reads (unless the app has its
// own XR_RUNTIME_JSON). SteamVR installs it as a symlink to <SteamVR>/steamxr_linux64.json; PiBridge points it at
// SteamVR's file or at the Monado build's openxr_monado-dev.json, always as a symlink swapped in atomically. A regular
// file there that is neither (set by another tool) is kept once as active_runtime.json.pibridge-backup.
#pragma once

#include <QString>
#include <expected>

namespace pibridge::app {

enum class OpenXrRuntime { None, SteamVr, Monado, Other };

// $XDG_CONFIG_HOME/openxr/1/active_runtime.json
QString activeRuntimePath();
// What `path` points to: its runtime JSON's "name" / VALVE_runtime_is_steamvr / library_path (None: no file).
OpenXrRuntime activeRuntimeOf(const QString& path);
// <SteamVR runtime>/steamxr_linux64.json for <SteamVR runtime>/bin/vrstartup.sh ("" when it is not there).
QString steamVrRuntimeJsonFor(const QString& steamVrStartup);
// Point `path` at `target` (a runtime JSON): a new symlink renamed over it. A regular file there that is not a
// runtime we know is kept once as <path>.pibridge-backup. Error: plain words.
std::expected<void, QString> setActiveRuntime(const QString& path, const QString& target);
QString openXrRuntimeWord(OpenXrRuntime r); // "SteamVR" / "Monado" / "other" / "none"

} // namespace pibridge::app
