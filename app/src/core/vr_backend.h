// SPDX-License-Identifier: MIT
// The VR backend (docs/plans/MONADO-BACKEND.md): SteamVR (pibridge-headtrack -> CustomHeadsetOpenVR, the default) or
// Monado (monado-service with the Pimax driver: inside-out, or Lighthouse through the faceplate). pibridge.conf
// vr_backend=steamvr|monado. Each backend keeps its own rooms and runtime files; the app exports PIBRIDGE_BACKEND, so
// every tool it starts (pibridge-roommap, pibridge-roomsetup, the trackers) works for the same one (libpibridge-anchor
// backend.h). Nothing anywhere guesses the backend.
#pragma once

#include <QString>
#include <QStringList>

#include "conf.h"

namespace pibridge::app {

inline constexpr char kVrBackendKey[] = "vr_backend";

enum class VrBackend { SteamVr, Monado };

// pibridge.conf vr_backend; anything but "monado" is SteamVR.
VrBackend vrBackend(const Conf& conf);
QString vrBackendName(VrBackend b); // "steamvr" / "monado"
QString vrBackendWord(VrBackend b); // "SteamVR" / "Monado" (the UI)
// PIBRIDGE_BACKEND of this process (inherited by everything it starts).
void exportVrBackend(VrBackend b);
// The backend this process exported (SteamVR when none).
VrBackend currentVrBackend();
// {"--backend", name}: the room tools' argument.
QStringList vrBackendArgs(VrBackend b);
// <runtime>/pibridge/<backend>: the backend's runtime files (anchor ring/offset, the Lighthouse stage).
QString vrBackendRuntimeDir(VrBackend b);
// ~/.local/share/pibridge/rooms/<backend>: the backend's rooms. The SteamVR one is first filled from the rooms saved
// before the split (~/.local/share/pibridge/roommap, copied once, the old folder kept) - as pibridge-roommap does.
QString vrBackendRoomsDir(VrBackend b, const QString& home);

} // namespace pibridge::app
