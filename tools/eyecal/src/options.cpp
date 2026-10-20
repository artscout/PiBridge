// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/main.rs (MIT OR Apache-2.0).
#include "options.h"

#include <charconv>
#include <cstdlib>
#include <format>

namespace pibridge::eyecal {

namespace fs = std::filesystem;

Dirs Dirs::from_env() {
    const fs::path p = pibridge::hmd::Settings::config_dir();
    return Dirs{p, p.parent_path() / "pimax-linux", p.parent_path() / "omarchyvr"};
}

std::optional<fs::path> Dirs::find(std::string_view name) const {
    std::error_code ec;
    for (const auto& d : {pibridge, legacy, omarchyvr}) {
        if (d.empty()) {
            continue;
        }
        const fs::path p = d / name;
        if (fs::is_regular_file(p, ec)) {
            return p;
        }
    }
    return std::nullopt;
}

namespace {

// `~/x` as the app or the user may write it; everything else as given.
fs::path expand(const std::string& s) {
    if (s.starts_with("~/")) {
        if (const char* h = std::getenv("HOME"); h != nullptr && *h != 0) {
            return fs::path(h) / s.substr(2);
        }
    }
    return fs::path(s);
}

std::optional<bool> flag_value(const char* v) {
    if (v == nullptr) {
        return std::nullopt;
    }
    std::string_view s(v);
    while (!s.empty() && s.front() == ' ') {
        s.remove_prefix(1);
    }
    while (!s.empty() && s.back() == ' ') {
        s.remove_suffix(1);
    }
    return s == "1" || s == "true" || s == "yes" || s == "on";
}

template <class T>
bool number(const std::string& s, T& out) {
    const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc() && p == s.data() + s.size();
}

} // namespace

fs::path settings_path_of(const std::vector<std::string>& args) {
    for (std::size_t i = 0; i + 1 < args.size(); i++) {
        if (args[i] == "--settings") {
            return args[i + 1];
        }
    }
    return pibridge::hmd::Settings::default_path();
}

std::string usage() {
    return R"(pibridge-eyecal: eye-tracking calibration for Pimax headsets (Crystal Super under SteamVR)

  pibridge-eyecal [--recenter | --host-only] [--pattern pimax|grid] [--points 9|13] [--pursuit]
               [--flip-x 0|1] [--flip-y 0|1] [--head-limit DEG] [--res-div N] [--calibration PATH] [--no-raw-log]
               [--frames FILE] [--profile FILE | --no-profile] [--chip-mirror none|x|y|xy] [--settings FILE]
  pibridge-eyecal --simulate [--seed N] [--recenter | --host-only] [--pattern ...] [--calibration PATH]

Put the headset on: it shows the check grid (circles at 10 to 50 degrees) and a violet dot where your eyes look,
as pibridge-eyed would report it now. A calibrates, B exits.

By default the eye chip calibrates itself, as Pimax Play does: keep the head still, the green dot travels to 9
targets (5 on black, then 4 on white) and the chip collects each one while you look at it. The chip computes its new
calibration and hands over the new profile. A short check with 5 targets follows; its constant offset (if any) goes
into ~/.config/pibridge/eye-calibration.toml. At the end A calibrates again, B saves (the profile where pibridge-eyed
uploads it at every start, the old file kept as .bak-<utc>, and the host map) and goes back to the check grid, whose
dot now uses the new calibration; B there exits. During the calibration B stops and goes back to the grid. Leaving
without a save (B in the grid after an unsaved run, closing, an error) puts the profile of the last save back, or
the one from before the tool. --recenter starts and ends in the grid too (A recentre, at the end A again, B save).

  --recenter        only the 5-target update of the drift offsets of the existing eye-calibration.toml
  --host-only       the host-side fit without chip calibration (the older method; the chip keeps its profile)
  --pattern P       for --host-only / --recenter: pimax (default: Pimax Play's path) or grid (9 or 13 targets)
  --points N        9 (default) or 13 grid targets (13 also fit a quadratic model)
  --pursuit         run the smooth-pursuit test after the validation (--host-only)
  --flip-x 0|1      device-to-head x conversion (default PIBRIDGE_EYE_FLIP_X, else eye_flip_x of pibridge.conf: 1)
  --flip-y 0|1      device-to-head y conversion (default PIBRIDGE_EYE_FLIP_Y, else 0)
  --head-limit DEG  head rotation that restarts a target (default 2.5)
  --res-div N       render at 1/N of the recommended resolution
  --calibration P   the host map file (default ~/.config/pibridge/eye-calibration.toml)
  --no-raw-log      do not write ~/.local/share/pibridge/eye-calibration/eyecal-<utc>-raw.csv
  --frames FILE     the eye chip's start list (default: eye_start_list of pibridge.conf, as pibridge-eyed)
  --profile FILE    the eye profile uploaded at the chip start and replaced by the new one (default: eye_profile
                    of pibridge.conf, else ~/.config/pibridge/eye-profile-crystal-super.bin)
  --no-profile      no profile upload at the start (the list's own); the new one still goes to the default file
  --chip-mirror M   show the chip's targets mirrored (x, y or xy; for the first test of a new headset)
  --settings FILE   another pibridge.conf
  --simulate        run the whole procedure on a simulated wearer and chip (no headset), print the report
  --seed N          the simulation's random seed

The eye chip has one owner: the tool refuses to start while pibridge-eyed (or pimax-eyed, omarchyvr-eyed) runs; the
PiBridge app stops pibridge-eyed around it. Results go to ~/.local/share/pibridge/eye-calibration/eyecal-<utc>.json.
Keys on the terminal: space/a = A, b/r = B, y save, p pursuit, c cursor, s runtime check, q quit.
)";
}

std::expected<Options, std::string> parse_args(const std::vector<std::string>& args, const pibridge::hmd::Settings& settings,
                                               const Dirs& dirs, const EnvFn& env) {
    Options o;
    o.settings_path = settings_path_of(args);
    o.flip_x = flag_value(env("EYE_FLIP_X")).value_or(settings.eye_flip_x);
    o.flip_y = flag_value(env("EYE_FLIP_Y")).value_or(false);
    // the start list and the profile: environment, else the settings, else the default files (eyed's rule)
    if (const char* v = env("EYE_FRAMES"); v != nullptr && *v != 0) {
        o.frames = expand(v);
    } else if (!settings.eye_start_list.empty()) {
        o.frames = expand(settings.eye_start_list);
    } else {
        o.frames = dirs.find(START_LIST_FILE);
    }
    std::optional<fs::path> named_profile;
    if (const char* v = env("EYE_PROFILE"); v != nullptr && *v != 0) {
        named_profile = expand(v);
    } else if (!settings.eye_profile.empty()) {
        named_profile = expand(settings.eye_profile);
    }
    o.profile = named_profile ? named_profile : dirs.find(PROFILE_FILE);
    bool recenter = false;
    bool host_only = false;
    std::optional<fs::path> profile_flag;
    for (std::size_t i = 0; i < args.size(); i++) {
        const std::string& a = args[i];
        const auto value = [&]() -> std::expected<std::string, std::string> {
            if (i + 1 >= args.size()) {
                return std::unexpected(a + " needs a value");
            }
            return args[++i];
        };
        if (a == "--help" || a == "-h") {
            o.help = true;
        } else if (a == "--recenter") {
            recenter = true;
        } else if (a == "--host-only") {
            host_only = true;
        } else if (a == "--pursuit") {
            o.pursuit = true;
        } else if (a == "--no-raw-log") {
            o.raw_log = false;
        } else if (a == "--simulate") {
            o.simulate = true;
        } else if (a == "--no-profile") {
            o.no_profile = true;
        } else if (a == "--pattern" || a == "--points" || a == "--flip-x" || a == "--flip-y" || a == "--head-limit" ||
                   a == "--res-div" || a == "--calibration" || a == "--frames" || a == "--profile" || a == "--seed" ||
                   a == "--chip-mirror" || a == "--settings") {
            auto v = value();
            if (!v) {
                return std::unexpected(v.error());
            }
            if (a == "--pattern") {
                if (*v == "pimax") {
                    o.pattern = Pattern::Pimax;
                } else if (*v == "grid") {
                    o.pattern = Pattern::Grid;
                } else {
                    return std::unexpected(std::string("--pattern must be pimax or grid"));
                }
            } else if (a == "--points") {
                if (*v != "9" && *v != "13") {
                    return std::unexpected(std::string("--points must be 9 or 13"));
                }
                o.points = *v == "9" ? 9 : 13;
            } else if (a == "--flip-x") {
                o.flip_x = *v == "1";
            } else if (a == "--flip-y") {
                o.flip_y = *v == "1";
            } else if (a == "--head-limit") {
                if (!number(*v, o.head_limit_deg)) {
                    return std::unexpected(std::string("--head-limit: a number of degrees"));
                }
            } else if (a == "--res-div") {
                if (!number(*v, o.res_div) || o.res_div == 0) {
                    return std::unexpected(std::string("--res-div: a number"));
                }
            } else if (a == "--calibration") {
                o.calibration = expand(*v);
            } else if (a == "--frames") {
                o.frames = expand(*v);
            } else if (a == "--profile") {
                profile_flag = expand(*v);
            } else if (a == "--seed") {
                if (!number(*v, o.seed)) {
                    return std::unexpected(std::string("--seed: a number"));
                }
            } else if (a == "--chip-mirror") {
                if (*v != "none" && *v != "x" && *v != "y" && *v != "xy") {
                    return std::unexpected(std::string("--chip-mirror must be none, x, y or xy"));
                }
                o.chip_mirror_x = v->find('x') != std::string::npos;
                o.chip_mirror_y = v->find('y') != std::string::npos;
            }
        } else {
            return std::unexpected(std::format("unexpected argument '{}'", a));
        }
    }
    if (recenter && host_only) {
        return std::unexpected(std::string("--recenter and --host-only do not go together"));
    }
    o.run = recenter ? RunKind::Recenter : (host_only ? RunKind::HostOnly : RunKind::Chip);
    if (profile_flag) {
        o.profile = profile_flag;
        named_profile = profile_flag;
    }
    if (o.no_profile) {
        o.profile.reset();
    }
    o.profile_out = named_profile ? *named_profile : dirs.pibridge / PROFILE_FILE;
    return o;
}

Settings procedure_settings(const Options& o) {
    Settings s;
    s.cal_points = o.points;
    s.pursuit = o.pursuit;
    s.conv.flip_x = o.flip_x;
    s.conv.flip_y = o.flip_y;
    s.head_limit_deg = o.head_limit_deg;
    s.chip_mirror_x = o.chip_mirror_x;
    s.chip_mirror_y = o.chip_mirror_y;
    switch (o.run) {
    case RunKind::Chip: return s.chip();
    case RunKind::HostOnly:
        if (o.pattern == Pattern::Pimax) {
            return s.pimax();
        }
        return s;
    case RunKind::Recenter: {
        Settings r = o.pattern == Pattern::Pimax ? s.pimax() : s;
        r.mode = Mode::Recenter;
        return r;
    }
    }
    return s;
}

} // namespace pibridge::eyecal
