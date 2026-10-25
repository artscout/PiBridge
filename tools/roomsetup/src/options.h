// SPDX-License-Identifier: MIT
// pibridge-roomsetup's command line.
#pragma once

#include <expected>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::roomsetup {

enum class Run {
    Headset,   // the OpenXR editor (default)
    SelfTest,  // the editor on synthetic files with scripted frames, no runtime, no GPU
    List,      // print the universes and the saved rooms
    Use,       // write a saved room to its universe without VR (ZoneTweak --use)
    VrUniverse, // helper: print the universe SteamVR tracks in (OpenVR background client)
    VrReload,  // helper: ask SteamVR to reload the chaperone from disk
    VrZone,    // helper: set a zone setting through IVRSettings
    VrStanding, // helper: set the SteamVR chaperone standing pose (floor + centre + forward) from the live raw head pose
    Recenter,  // the inside-out recenter in the headset (recenter.h), the result into --recenter-out
    FloorHeight, // the floor from the head standing upright minus --eye-height, in the running session (mapflow.h)
    Map,       // map the room in the headset over the passthrough (mapflow.h), driving pibridge-roommap setup fast --guide
};

struct Options {
    Run run = Run::Headset;
    bool help = false;
    std::string universe;                // --universe ID: edit this universe
    std::vector<std::string> configDirs; // --config-dir DIR (repeatable): instead of SteamVR's config folders
    std::string roomsDir;                // --rooms DIR: instead of ~/.config/pibridge/rooms
    std::string settingsFile;            // --settings FILE: instead of <first config dir>/steamvr.vrsettings
    std::string useName;                 // --use NAME
    std::string snapshotDir;             // --snapshot DIR (self test): write the views as PPM
    std::string zoneArg;                 // --vr-zone fade=0.5|fade=default|alpha=0|alpha=default
    bool noOpenVr = false;               // --no-openvr: never run the OpenVR helper
    unsigned resDiv = 1;                 // --res-div N
    std::string recenterOut;             // --recenter-out FILE (with --recenter)
    std::string ringFile;                // --ring FILE: Monado's head ring (default $XDG_RUNTIME_DIR/pimax-anchor.head)
    std::string room = "home";           // --room NAME (with --map): the room map's name
    std::string roommapTool;             // --roommap PATH: pibridge-roommap (default: next to this binary, else PATH)
    std::string camera = "broker";       // --camera SPEC: the passthrough's frames (broker, a .pxcam, "off")
    std::string guideDir;                // --guide-dir DIR (default $XDG_RUNTIME_DIR/pibridge/roommap-guide-<pid>)
    std::string logDir;                  // --log-dir DIR (with --map; default: the newest inside-out session's logs)
    std::string mapSeconds;
    std::optional<double> eyeHeight;     // --eye-height M: the user's eye height standing upright (1.0..2.3)              // --map-seconds S: pibridge-roommap's mapping time (its default 30)
    std::optional<double> radius;        // --radius M (with --vr-standing): Quick room setup - a circle of this radius
                                         // around the play centre as the SteamVR chaperone boundary (0.3..5.0)
    std::string perimeterFile;           // --perimeter FILE (with --vr-standing): the traced boundary as the SteamVR
                                         // chaperone perimeter; "x z" setup-frame points one per line (over --radius)
    bool seated = false;                 // --seated (with --vr-standing): set the seated zero pose (centre at the head
                                         // where you sit) instead of the standing one (centre on the floor)
    bool keepFloor = false;              // --keep-floor (with --vr-standing): keep the chaperone's current floor, set
                                         // centre / forward only (the in-game recenter)
    bool floorOnly = false;              // --floor-only (with --vr-standing): keep the chaperone's centre / forward, set
                                         // only the floor from the saved room floor (the room recognised in a new
                                         // session, whose raw space is new)
    bool chaperone = false;              // --chaperone (with --map): SteamVR path - pibridge-roommap saves the setup with
                                         // --no-offset, then this tool writes the floor/centre/forward into SteamVR's
                                         // chaperone (--vr-standing) and reloads it, instead of a Monado world offset
    bool quickDirect = false;            // --quick-direct (with --map): start straight in the quick circle setup,
                                         // skipping the in-VR Full/Quick mode menu
};

std::expected<Options, std::string> parseArgs(const std::vector<std::string>& args);
std::string usage();

/// --vr-zone's value.
struct ZoneArg {
    enum Key { Fade, Alpha } key = Fade;
    std::optional<double> value; // nullopt: back to SteamVR's default
};
std::expected<ZoneArg, std::string> parseZoneArg(const std::string& s);

} // namespace pibridge::roomsetup
