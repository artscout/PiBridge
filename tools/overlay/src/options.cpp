// SPDX-License-Identifier: MIT
#include "options.h"

#include <charconv>

namespace pibridge::overlay {

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
        } else if (a == "--selftest") {
            o.selfTest = true;
        } else if (a == "--print-manifest") {
            o.printManifest = true;
        } else if (a == "--register") {
            o.doRegister = true;
        } else if (a == "--unregister") {
            o.doUnregister = true;
        } else if (a == "--icon" || a == "--binary" || a == "--manifest") {
            auto v = value(a.c_str());
            if (!v) {
                return std::unexpected(v.error());
            }
            (a == "--icon" ? o.icon : a == "--binary" ? o.binary : o.manifest) = *v;
        } else if (a == "--width") {
            auto v = value("--width");
            if (!v) {
                return std::unexpected(v.error());
            }
            double m = 0;
            const auto r = std::from_chars(v->data(), v->data() + v->size(), m);
            if (r.ec != std::errc() || r.ptr != v->data() + v->size() || m < 0.2 || m > 3.0) {
                return std::unexpected("--width takes 0.2 to 3 metres, not " + *v);
            }
            o.width = m;
        } else {
            return std::unexpected("unknown option " + a);
        }
    }
    if (o.doRegister && o.doUnregister) {
        return std::unexpected(std::string("--register and --unregister are mutually exclusive"));
    }
    return o;
}

std::string usage() {
    return "pibridge-overlay: a small SteamVR dashboard launcher. It draws a little panel with two buttons (ROOM SETUP\n"
           "and QUICK ROOM SETUP); the controller laser clicks them, and on a click it writes a request the PiBridge app\n"
           "reads and launches the setup from. A leaf: minimal OpenVR, no app-window embed, never takes a VR session down.\n"
           "\n"
           "  pibridge-overlay [--width M] [--icon PATH]      run the dashboard launcher (needs SteamVR)\n"
           "  pibridge-overlay --register [--manifest PATH] [--binary PATH] [--icon PATH]\n"
           "                                                  register with SteamVR and turn on auto-launch (SteamVR must run)\n"
           "  pibridge-overlay --unregister [--manifest PATH] remove the registration and auto-launch\n"
           "  pibridge-overlay --print-manifest [--binary PATH] [--icon PATH]   print the .vrmanifest JSON\n"
           "  pibridge-overlay --selftest                     checks the pure parts, no runtime or GPU\n"
           "  pibridge-overlay --help\n";
}

} // namespace pibridge::overlay
