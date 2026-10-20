// SPDX-License-Identifier: MIT
// The Device page (like the headset details of Pimax EVO): what the headset, the controllers and the base stations
// are (model and serial number), the IPD, and how the app renders now (per-eye resolution, refresh rate, quad views,
// tracking mode, Monado build). Everything is read at run time and shown only in the app: serials are never logged
// and never written to a file; fake mode uses made-up ones.
//
// Live sources: SteamVR (core/steamvr.h; serials, the IPD SteamVR renders with, the display frequency and the
// recommended per-eye render size) and the services' status files. When SteamVR is off the page falls back to the
// configured values (CustomHeadsetOpenVR's settings.json ipd, steamvr.vrsettings supersampling, pibridge.conf
// refresh_hz) and says so. The Crystal Super reports no hardware IPD on Linux (CustomHeadsetOpenVR's hardwareIpd
// reads it through Pimax's Windows runtime only); the eye tracker's measured IPD is shown when it streams.
#pragma once

#include <QString>
#include <optional>
#include <vector>

#include "monado_build.h"
#include "status.h"
#include "steamvr.h"

namespace pibridge::app {

// CustomHeadsetOpenVR's configured IPD (mm): the "ipd" (+ "ipdOffset") of the first enabled Crystal Super entry
// of its settings.json (any enabled headset entry when none is a Crystal Super); `hardwareIpd` tells whether that
// entry asks for the platform's IPD. nullopt when the file is missing or broken or no entry has an "ipd".
struct ChovrIpd {
    double mm = 0;
    bool hardwareIpd = false;
    QString headset; // the settings.json key ("crystalSuperMicroOLED")
};
std::optional<ChovrIpd> chovrConfiguredIpd(const QString& settingsFile);

// SteamVR's supersampling from steamvr.vrsettings ("steamvr": supersampleScale, supersampleManualOverride).
// nullopt when the file is missing or broken; scale 1 when the key is not set (SteamVR's default).
struct SteamVrSupersample {
    double scale = 1.0;
    bool manual = false; // false: SteamVR picks the resolution itself ("Auto")
};
std::optional<SteamVrSupersample> steamVrSupersample(const QString& vrsettingsFile);

// Everything the page needs, gathered by the page (tests fill it by hand).
struct DeviceInputs {
    bool steamvrRunning = false;
    SteamVrStatus steamvr;
    KeepaliveStatus keepalive;
    float gazeIpdMm = 0;                     // the eye tracker's measured IPD (0 = no gaze)
    std::optional<ChovrIpd> chovrIpd;        // configured IPD
    std::optional<SteamVrSupersample> supersample;
    int confRefreshHz = 90;                  // pibridge.conf refresh_hz
    bool qvInstalled = false;                // the quad views layer's user manifest
    bool qvEnabled = false;
    bool insideOut = false;                  // pibridge.conf tracking_mode = inside-out
    bool ioRunning = false;                  // START TRACKING (SteamVR + Monado) runs
    bool directRunning = false;              // "Monado directly" runs
    MonadoBuildInfo monado;                  // the Monado build the sessions use
};

struct InfoRow {
    QString key;   // "Headset SN"
    QString value; // "" when not known: `note` says why
    QString note;  // where it comes from / what to do ("from SteamVR", "Start SteamVR to read it")
    bool copy = false; // a COPY button (serials)
};

struct DeviceView {
    QString title;    // "Pimax Crystal Super" / "No headset"
    ChipText chip;    // STEAMVR LIVE / STEAMVR OFF
    std::vector<InfoRow> headset;     // model, serial, IPD, measured IPD
    std::vector<InfoRow> controllers; // one row per controller / tracker
    std::vector<InfoRow> stations;    // one row per base station
    QString devicesNote;              // "Start SteamVR to see the serial numbers."
    std::vector<InfoRow> rendering;   // resolution, refresh, quad views, tracking mode, Monado build
};

DeviceView deviceView(const DeviceInputs& in);

// The inputs of the fake mode (the fake devices with made-up serials).
DeviceInputs fakeDeviceInputs(bool active);

} // namespace pibridge::app
