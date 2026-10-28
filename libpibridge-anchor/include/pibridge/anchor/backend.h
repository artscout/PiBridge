// SPDX-License-Identifier: MIT
// The VR backend a process works for (docs/plans/MONADO-BACKEND.md): SteamVR (pibridge-headtrack -> CustomHeadsetOpenVR)
// or Monado (monado-service with the Pimax driver). Each keeps its own runtime files (the head ring, the room offset
// and its lease) and its own rooms: nothing is shared between the two, and nothing guesses the backend from which
// process happens to write a file. The starter of a session sets PIBRIDGE_BACKEND; the tools take --backend.
#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace pibridge::anchor {

enum class Backend { SteamVr, Monado };

inline constexpr char BACKEND_ENV[] = "PIBRIDGE_BACKEND";

/// "steamvr" / "monado" (exactly); nullopt for anything else.
std::optional<Backend> parse_backend(std::string_view s);
std::string_view backend_name(Backend b);
/// PIBRIDGE_BACKEND when set to a valid name; nullopt when unset or empty. A set but invalid value is nullopt too and
/// `*bad` (when given) gets it.
std::optional<Backend> backend_from_env(std::string* bad = nullptr);
/// <runtime>/pibridge/<backend>: the backend's runtime files (head ring, offset, lease).
std::string backend_runtime_dir(Backend b);

} // namespace pibridge::anchor
