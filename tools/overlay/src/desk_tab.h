// SPDX-License-Identifier: MIT
// The desktop tab in the SteamVR dashboard (docs/desktop.md): a fourth dashboard overlay, "Desktop", whose page is the
// shown monitors (from pibridge-desktop, desktop.shm) with PiBridge's bar under them (the monitors' choice, Record
// video, Quick room setup, Recenter; tools/home's deskLayout, so it looks as in the Monado overlay). The page goes to
// SteamVR as a Vulkan texture (a per-frame SetOverlayRaw crashed SteamVR): each monitor's picture is copied into its
// own image and blitted (scaled) into the page on the GPU, double buffered. SteamVR's laser is the mouse: on a monitor
// it is the desktop's pointer, on the bar it clicks the bar. Nothing is captured while the tab is not shown.
#pragma once

#include <openvr.h>

#include <memory>
#include <string>

namespace pibridge::overlay {

class DeskTab {
public:
    DeskTab(vr::IVROverlay* ov, vr::IVRSystem* sys, vr::IVRCompositor* comp);
    ~DeskTab();
    DeskTab(const DeskTab&) = delete;
    DeskTab& operator=(const DeskTab&) = delete;
    /// The dashboard overlay; false (with error()) when SteamVR refused it.
    bool create();
    /// Events and, while shown, the pictures; false on VREvent_Quit.
    bool update();
    /// The tab is shown in the dashboard now (the caller polls faster then).
    bool shown() const;
    const std::string& error() const { return err_; }

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
    std::string err_;
};

} // namespace pibridge::overlay
