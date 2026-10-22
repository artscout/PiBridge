// SPDX-License-Identifier: MIT
// CustomHeadsetOpenVR (sboys3), the SteamVR display driver of the Lighthouse/SteamVR mode: is it the headset's
// driver, and where is its configurator (the Tauri GUI `custom-headset-gui`). The Dashboard's HEADSET DRIVER
// SETTINGS button is shown only while CustomHeadsetOpenVR is the driver and starts the configurator detached.
#pragma once

#include <QString>

#include "runner.h"

namespace pibridge::app {

// $XDG_CONFIG_HOME/CustomHeadset/settings.json (default ~/.config/CustomHeadset/settings.json).
QString chovrSettingsFile();

// CustomHeadsetOpenVR is registered with SteamVR: an `external_drivers` entry of the vrpath file whose path ends in
// /CustomHeadsetOpenVR, or <runtime>/drivers/CustomHeadsetOpenVR exists (runtime = runtime[0] of the same file).
bool chovrDriverRegistered(const QString& vrpathFile);
// settings.json exists, is a JSON object, and some headset entry is enabled ("enable": true, or no "enable" key:
// CHOVR's default). A file without any headset entry runs on the defaults and counts too; "customShader" is not a
// headset.
bool chovrHeadsetEnabled(const QString& settingsFile);
// Sets the virtual-camera IPD (mm) CustomHeadsetOpenVR renders with: "ipd" in every Crystal entry (the
// Micro-OLED one is made when there is none), "hardwareIpd": false and no generalHeadset.horizontalIPDOffset (CHOVR reads the hardware IPD only through
// Pimax's Windows runtime). Takes effect the next time SteamVR starts. False when the file could not be written.
bool chovrSetIpd(const QString& settingsFile, double mm);

// Enables CustomHeadsetOpenVR's entry for the original Crystal ("crystalOG", off by default in its builds): its own
// profile (2880x2880 per eye, 120 Hz, eye rotation 5 deg, "Crystal OG Default" distortion) then draws the picture in
// Lighthouse mode. `changed`: whether the file was written (SteamVR then needs a restart to pick it up).
bool chovrEnableCrystalOg(const QString& settingsFile, bool* changed = nullptr);

// The original Crystal's display refresh rate (Hz) SteamVR picks the display mode by: "refreshRate" in the
// "crystalOG" entry (our CHOVR patch; 0 or absent = the tracked device's own, 90 Hz). Read at SteamVR's start.
int chovrCrystalRefresh(const QString& settingsFile);
bool chovrSetCrystalRefresh(const QString& settingsFile, int hz);

// Both: the Lighthouse/SteamVR mode with CustomHeadsetOpenVR as the headset's driver.
bool chovrActive(const QString& vrpathFile, const QString& settingsFile);

// The configurator, first match: `custom-headset-gui` in PATH, <home>/.local/bin/custom-headset-gui, `confPath`
// (pibridge.conf `chovr_gui=`), the source build <home>/gamedev/ext/CustomHeadsetOpenVR/output/CustomHeadsetGUI/release/
// custom-headset-gui. Only executable files count; "" when none.
QString chovrGuiPath(const QString& confPath, const QString& home);

// Starts it detached (never waits for it); false when `path` is empty or it could not start.
bool startChovrGui(Runner& runner, const QString& path);

} // namespace pibridge::app
