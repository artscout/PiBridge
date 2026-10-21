// SPDX-License-Identifier: MIT
#include "options.h"

#include <charconv>
#include <format>

namespace pibridge::roomsetup {

std::string usage() {
    return "usage: pibridge-roomsetup [options]\n"
           "  (no options)          the room setup in the headset (OpenXR; SteamVR must be running)\n"
           "  --universe ID         edit this tracking universe (default: the one SteamVR tracks in)\n"
           "  --list                print the universes of the chaperone file and the saved rooms\n"
           "  --use NAME            write the saved room NAME to its universe (SteamVR must not be running)\n"
           "  --selftest            run the editor on made-up files with scripted controllers (no headset)\n"
           "  --snapshot DIR        with --selftest: write what the headset would show as DIR/*.ppm\n"
           "  --config-dir DIR      read the chaperone files from DIR instead of SteamVR's config folders\n"
           "  --rooms DIR           saved rooms in DIR instead of ~/.config/pibridge/rooms\n"
           "  --settings FILE       steamvr.vrsettings to use for the zone settings\n"
           "  --recenter --recenter-out FILE [--ring FILE]\n"
           "                        inside-out recenter in the headset: centre and forward from the head, the floor\n"
           "                        from a controller touch; the result into FILE (PiBridge writes Monado's offset)\n"
           "  --map [--room NAME] [--roommap PATH] [--camera SPEC] [--ring FILE] [--map-seconds S] [--guide-dir DIR]\n"
           "        [--eye-height M]  the floor step first offers: stand upright, the floor M below your eyes\n"
           "  --floor-height --eye-height M [--room NAME] [--roommap PATH] [--camera SPEC] [--ring FILE]\n"
           "                        set the floor from your height in the running session (the room recognised)\n"
           "        [--log-dir DIR]   (default: the logs folder of the newest inside-out session)\n"
           "                        map the room in the headset over the passthrough (pibridge-roommap setup fast does the\n"
           "                        work; SPEC: broker (default), a .pxcam, or off)\n"
           "  --no-openvr           never ask SteamVR (universe, reload, zone settings) through OpenVR\n"
           "  --res-div N           render at 1/N of the recommended resolution\n"
           "helpers (run by the tool itself, one OpenVR background client each, never start SteamVR):\n"
           "  --vr-universe         print the universe id SteamVR tracks in\n"
           "  --vr-reload           ask SteamVR to reload the chaperone file\n"
           "  --vr-zone KEY=VALUE   fade=METRES|fade=default|alpha=0..255|alpha=default\n"
           "  --vr-standing --eye-height M  set the SteamVR chaperone standing pose (floor M below your eyes, centre and\n"
           "                        forward from the live head pose) and commit it\n";
}

std::expected<Options, std::string> parseArgs(const std::vector<std::string>& args) {
    Options o;
    const auto value = [&](std::size_t& i) -> std::expected<std::string, std::string> {
        if (i + 1 >= args.size()) {
            return std::unexpected(std::format("{} needs a value", args[i]));
        }
        return args[++i];
    };
    for (std::size_t i = 0; i < args.size(); i++) {
        const std::string& a = args[i];
        std::expected<std::string, std::string> v;
        if (a == "-h" || a == "--help") {
            o.help = true;
        } else if (a == "--selftest") {
            o.run = Run::SelfTest;
        } else if (a == "--list") {
            o.run = Run::List;
        } else if (a == "--vr-universe") {
            o.run = Run::VrUniverse;
        } else if (a == "--vr-reload") {
            o.run = Run::VrReload;
        } else if (a == "--vr-standing") {
            o.run = Run::VrStanding;
        } else if (a == "--chaperone") {
            o.chaperone = true;
        } else if (a == "--quick-direct") {
            o.quickDirect = true;
        } else if (a == "--seated") {
            o.seated = true;
        } else if (a == "--recenter") {
            o.run = Run::Recenter;
        } else if (a == "--map") {
            o.run = Run::Map;
        } else if (a == "--floor-height") {
            o.run = Run::FloorHeight;
        } else if (a == "--eye-height") {
            v = value(i);
            if (!v) {
                return std::unexpected(v.error());
            }
            double d = 0;
            const auto r = std::from_chars(v->data(), v->data() + v->size(), d);
            if (r.ec != std::errc() || r.ptr != v->data() + v->size() || d < 1.0 || d > 2.3) {
                return std::unexpected(std::string("--eye-height is your eye height standing upright, 1.0-2.3 m"));
            }
            o.eyeHeight = d;
        } else if (a == "--radius") {
            v = value(i);
            if (!v) {
                return std::unexpected(v.error());
            }
            double d = 0;
            const auto r = std::from_chars(v->data(), v->data() + v->size(), d);
            if (r.ec != std::errc() || r.ptr != v->data() + v->size() || d < 0.3 || d > 5.0) {
                return std::unexpected(std::string("--radius is the quick play-circle radius, 0.3-5.0 m"));
            }
            o.radius = d;
        } else if (a == "--perimeter") {
            v = value(i);
            if (!v) {
                return std::unexpected(v.error());
            }
            o.perimeterFile = *v;
        } else if (a == "--room" || a == "--roommap" || a == "--camera" || a == "--guide-dir" || a == "--map-seconds" || a == "--log-dir") {
            v = value(i);
            if (!v) {
                return std::unexpected(v.error());
            }
            (a == "--room" ? o.room : a == "--roommap" ? o.roommapTool : a == "--camera" ? o.camera
                                                       : a == "--guide-dir" ? o.guideDir : a == "--log-dir" ? o.logDir : o.mapSeconds) = *v;
        } else if (a == "--recenter-out" || a == "--ring") {
            v = value(i);
            if (!v) {
                return std::unexpected(v.error());
            }
            (a == "--ring" ? o.ringFile : o.recenterOut) = *v;
        } else if (a == "--no-openvr") {
            o.noOpenVr = true;
        } else if (a == "--use" || a == "--universe" || a == "--config-dir" || a == "--rooms" || a == "--settings" ||
                   a == "--snapshot" || a == "--vr-zone" || a == "--res-div") {
            v = value(i);
            if (!v) {
                return std::unexpected(v.error());
            }
            if (a == "--use") {
                o.run = Run::Use;
                o.useName = *v;
                // ZoneTweak: the rest of the line is the name ("--use Door open")
                while (i + 1 < args.size() && !args[i + 1].starts_with("--")) {
                    o.useName += " " + args[++i];
                }
            } else if (a == "--universe") {
                o.universe = *v;
            } else if (a == "--config-dir") {
                o.configDirs.push_back(*v);
            } else if (a == "--rooms") {
                o.roomsDir = *v;
            } else if (a == "--settings") {
                o.settingsFile = *v;
            } else if (a == "--snapshot") {
                o.snapshotDir = *v;
            } else if (a == "--vr-zone") {
                o.run = Run::VrZone;
                if (auto z = parseZoneArg(*v); !z) {
                    return std::unexpected(z.error());
                }
                o.zoneArg = *v;
            } else {
                unsigned n = 0;
                const auto r = std::from_chars(v->data(), v->data() + v->size(), n);
                if (r.ec != std::errc() || n == 0 || n > 8) {
                    return std::unexpected(std::string("--res-div takes 1..8"));
                }
                o.resDiv = n;
            }
        } else {
            return std::unexpected(std::format("unknown option {}", a));
        }
    }
    if (o.run == Run::Recenter && o.recenterOut.empty()) {
        return std::unexpected(std::string("--recenter needs --recenter-out FILE"));
    }
    if (o.run == Run::FloorHeight && !o.eyeHeight) {
        return std::unexpected(std::string("--floor-height needs --eye-height M"));
    }
    if (o.run == Run::VrStanding && !o.seated && !o.eyeHeight) {
        // Seated sets a seated-zero pose at the head (no floor drop), so it needs no --eye-height; standing does.
        return std::unexpected(std::string("--vr-standing needs --eye-height M (the floor goes that far below your eyes)"));
    }
    if (o.run == Run::Map || o.run == Run::FloorHeight) {
        if (o.room.empty() || o.room.find('/') != std::string::npos || o.room.starts_with(".")) {
            return std::unexpected(std::string("--room takes a plain name"));
        }
        if (!o.mapSeconds.empty()) {
            double d = 0;
            const auto r = std::from_chars(o.mapSeconds.data(), o.mapSeconds.data() + o.mapSeconds.size(), d);
            if (r.ec != std::errc() || r.ptr != o.mapSeconds.data() + o.mapSeconds.size() || d < 10 || d > 300) {
                return std::unexpected(std::string("--map-seconds takes 10..300"));
            }
        }
    }
    if (!o.snapshotDir.empty() && o.run != Run::SelfTest) {
        return std::unexpected(std::string("--snapshot goes with --selftest"));
    }
    return o;
}

std::expected<ZoneArg, std::string> parseZoneArg(const std::string& s) {
    const auto eq = s.find('=');
    if (eq == std::string::npos) {
        return std::unexpected(std::string("--vr-zone takes KEY=VALUE"));
    }
    const std::string k = s.substr(0, eq);
    const std::string v = s.substr(eq + 1);
    ZoneArg z;
    if (k == "fade") {
        z.key = ZoneArg::Fade;
    } else if (k == "alpha") {
        z.key = ZoneArg::Alpha;
    } else {
        return std::unexpected(std::format("unknown zone key {}", k));
    }
    if (v == "default") {
        return z;
    }
    double d = 0;
    const auto r = std::from_chars(v.data(), v.data() + v.size(), d);
    if (r.ec != std::errc() || r.ptr != v.data() + v.size()) {
        return std::unexpected(std::format("bad zone value {}", v));
    }
    if ((z.key == ZoneArg::Fade && (d < 0.05 || d > 2.0)) || (z.key == ZoneArg::Alpha && (d < 0 || d > 255))) {
        return std::unexpected(std::format("zone value {} out of range", v));
    }
    z.value = d;
    return z;
}

} // namespace pibridge::roomsetup
