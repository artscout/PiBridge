// SPDX-License-Identifier: MIT
// The OpenVR side of pibridge-roomsetup, run as its own short process (pibridge-roomsetup --vr-...), so the OpenXR
// process never loads a second VR client next to the runtime's. It connects as a *background* application: that
// kind of client never starts SteamVR and fails at once when SteamVR is not running. libopenvr_api.so of the
// installed SteamVR is loaded with dlopen (no link-time dependency), as the app's SteamVR status does.
#pragma once

#include "options.h"

#include <optional>
#include <string>

namespace pibridge::roomsetup {

/// Print the universe id SteamVR tracks in (the HMD's Prop_CurrentUniverseId_Uint64). 0 = ok, 1 = no answer.
int vrUniverseMain();
/// IVRChaperoneSetup::ReloadFromDisk(Live): SteamVR reads the chaperone file again.
int vrReloadMain();
/// IVRSettings: set (or remove) collisionBounds.CollisionBoundsFadeDistance / CollisionBoundsColorGammaA.
int vrZoneMain(const ZoneArg& z);
/// Set the SteamVR chaperone standing pose from the live raw head pose: the floor `eyeHeight` m below the eyes, the
/// centre where the head is and forward where it looks, written to the Live chaperone (IVRChaperoneSetup). The
/// existing collision bounds are kept. The user must be standing in the centre, looking forward, when this runs.
int vrStandingMain(std::optional<double> eyeHeight, std::optional<double> radius = std::nullopt, bool seated = false);

} // namespace pibridge::roomsetup
