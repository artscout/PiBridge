// SPDX-License-Identifier: MIT
#include "options.h"

#include <charconv>

namespace pibridge::home {

std::expected<Options, std::string> parseArgs(const std::vector<std::string>& args) {
    Options o;
    for (std::size_t i = 0; i < args.size(); i++) {
        const std::string& a = args[i];
        const auto value = [&](const char* name) -> std::expected<std::string, std::string> {
            if (i + 1 >= args.size()) {
                return std::unexpected(std::string(name) + " needs a value");
            }
            return args[++i];
        };
        if (a == "-h" || a == "--help") {
            o.help = true;
        } else if (a == "--passthrough-overlay") {
            o.passthroughOverlay = true;
        } else if (a == "--dashboard-overlay") {
            o.dashboardOverlay = true;
        } else if (a == "--selftest") {
            o.selfTest = true;
        } else if (a == "--snapshot" || a == "--rooms" || a == "--camera" || a == "--panel-ppm") {
            auto v = value(a.c_str());
            if (!v) {
                return std::unexpected(v.error());
            }
            (a == "--snapshot" ? o.snapshotDir : a == "--rooms" ? o.roomsDir : a == "--camera" ? o.camera : o.panelPpm) = *v;
        } else if (a == "--res-div") {
            auto v = value("--res-div");
            if (!v) {
                return std::unexpected(v.error());
            }
            unsigned n = 0;
            const auto r = std::from_chars(v->data(), v->data() + v->size(), n);
            if (r.ec != std::errc() || r.ptr != v->data() + v->size() || n < 1 || n > 8) {
                return std::unexpected("--res-div takes 1 to 8, not " + *v);
            }
            o.resDiv = n;
        } else {
            return std::unexpected("unknown option " + a);
        }
    }
    if (!o.panelPpm.empty() && o.snapshotDir.empty()) {
        return std::unexpected(std::string("--panel-ppm goes with --snapshot"));
    }
    if (!o.snapshotDir.empty() && !o.selfTest) {
        return std::unexpected(std::string("--snapshot goes with --selftest"));
    }
    return o;
}

std::string usage() {
    return "pibridge-home: PiBridge's dashboard and home in the headset (OpenXR + Vulkan).\n"
           "  --dashboard-overlay: the dashboard over games on the Monado backend (the menu button opens it; Quick room\n"
           "    setup, Recenter, Close app; it brings PiBridge World back after a game). The Monado session starts it.\n"
           "  --passthrough-overlay: PiBridge World's passthrough (the room through the cameras, a window to the world's\n"
           "    panel). The world starts it.\n"
           "  no mode: the classic VR home (vr_home=classic, or no Godot for PiBridge World): the PiBridge window on a\n"
           "    curved panel over the room or a 360-degree background, the saved room and the controllers.\n"
           "Every mode runs inside a running Monado (XR_RUNTIME_JSON); it ends on SIGTERM or when the runtime goes away.\n"
           "\n"
           "  pibridge-home [--rooms DIR] [--res-div N] [--camera SPEC]   run (OpenXR + Vulkan)\n"
           "  pibridge-home --dashboard-overlay | --passthrough-overlay\n"
           "  pibridge-home --selftest [--snapshot DIR [--panel-ppm FILE]] checks without a runtime or GPU\n"
           "  pibridge-home --help\n";
}

} // namespace pibridge::home
