// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/main.rs (MIT OR Apache-2.0).
// pibridge-eyecal: the options, the one-owner check, the tracker it runs itself (start list and profile as pibridge-eyed,
// the calibration channel in between), then the OpenXR app or the simulation.
#include "chip.h"
#include "options.h"
#include "procedure.h"
#include "report.h"
#include "runner.h"
#include "sim.h"

#ifdef PIBRIDGE_EYECAL_XR
#include "xrapp.h"
#endif

#include "eyed.h"
#include "hid_commands.h"

#include "pibridge_env.h"

#include "pibridge/eye/live.h"
#include "pibridge/eyemath/calib.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <format>
#include <numbers>

using namespace pibridge;
using namespace pibridge::eyecal;

namespace {

#ifdef PIBRIDGE_EYECAL_XR
std::atomic<bool> g_stop{false};

extern "C" void on_signal(int) { g_stop.store(true); }
#endif

void say(const std::string& s) {
    std::printf("%s\n", s.c_str());
    std::fflush(stdout);
}

int fail(const std::string& e, int code) {
    std::fprintf(stderr, "pibridge-eyecal: %s\n", e.c_str());
    return code;
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const auto settings = pibridge::hmd::Settings::load_or_legacy(settings_path_of(args));
    auto opts = parse_args(args, settings, Dirs::from_env(), [](std::string_view n) { return pibridge::env(n); });
    if (!opts) {
        std::fprintf(stderr, "pibridge-eyecal: %s\n\n%s", opts.error().c_str(), usage().c_str());
        return 2;
    }
    if (opts->help) {
        std::fputs(usage().c_str(), stdout);
        return 0;
    }
    Settings ps = procedure_settings(*opts);
    const auto home = pibridge::eyemath::home_dir();
    if (opts->run == RunKind::Recenter) {
        const auto from = opts->calibration ? opts->calibration : pibridge::eyemath::Calibration::find_default(home);
        if (!from) {
            return fail("--recenter needs an existing calibration (no eye-calibration.toml in ~/.config/pibridge, "
                        "~/.config/pimax-linux or ~/.config/omarchyvr): run a calibration first",
                        2);
        }
        auto c = pibridge::eyemath::Calibration::load(*from);
        if (!c) {
            return fail(std::format("--recenter needs an existing calibration: {}", c.error().message), 2);
        }
        ps.conv = c->conv;
        ps.existing = *c;
        ps.view = *c; // the check grid shows the gaze through it
    }
    const auto calibration_out = opts->calibration ? opts->calibration : pibridge::eyemath::Calibration::default_path();
    if (opts->simulate) {
        SimParams sp;
        sp.seed = opts->seed;
        const auto out = run(Procedure(ps), sp);
        for (const auto& l : out.log) {
            say(l);
        }
        if (ps.mode != Mode::Chip) {
            std::fputs(summary(out.procedure).text.c_str(), stdout);
        }
        for (const auto& l : out.procedure.chip_lines()) {
            say(l);
        }
        if (out.profile) {
            say(std::format("simulated chip profile: {} bytes (not written: a simulation never touches the profile)",
                            out.profile->size()));
        }
        if (out.grid_gaze) {
            const auto& g = *out.grid_gaze;
            const auto& l = out.grid_look;
            const double n = std::sqrt(l[0] * l[0] + l[1] * l[1] + l[2] * l[2]);
            const double c = std::clamp((g[0] * l[0] + g[1] * l[1] + g[2] * l[2]) / n, -1.0, 1.0);
            say(std::format("check grid after the save: the gaze dot is {:.2f} deg from where the wearer looks; B: exit",
                            std::acos(c) * 180.0 / std::numbers::pi));
        }
        if (out.saved && opts->calibration) {
            if (auto r = write_with_backup(*opts->calibration, *out.saved); !r) {
                return fail(r.error(), 1);
            }
            say("simulated calibration written to " + opts->calibration->string());
        }
        return 0;
    }
    if (!calibration_out) {
        return fail("no $HOME: nowhere to save the calibration", 2);
    }
    if (const auto owner = chip_owner()) {
        return fail(owner_message(*owner), 3);
    }
#ifndef PIBRIDGE_EYECAL_XR
    return fail("this build has no OpenXR app (the OpenXR loader, Vulkan or glslangValidator were missing at build "
                "time); only --simulate works",
                1);
#else
    // the tracker exactly as pibridge-eyed starts it (start list, the user's profile), but raw: no host map, no filter
    const auto log = [](std::string_view m) { say("pibridge-eye: " + std::string(m)); };
    pibridge::eyed::Options eo;
    eo.frames = opts->frames;
    eo.profile = opts->profile;
    eo.no_profile = opts->no_profile;
    std::string kind;
    auto cfg = pibridge::eyed::make_config(eo, pibridge::eye::live::Config::from_env(log), [](std::string_view m) { say(std::string(m)); },
                                        kind);
    if (!cfg) {
        return fail(cfg.error(), 2);
    }
    // the check grid shows the gaze pibridge-eyed would publish: its host map (when it was measured with this tool's
    // frame conversion) and its filter, applied by the procedure to the raw samples
    if (ps.mode == Mode::Chip && cfg->calibration) {
        if (cfg->calibration->conv == ps.conv) {
            ps.view = *cfg->calibration;
            say("check grid: the gaze through " + cfg->calibration_path.string());
        } else {
            say("check grid: " + cfg->calibration_path.string() +
                " was measured with another frame conversion (flip x / y); the grid shows the gaze without it");
        }
    }
    ps.view_filter = cfg->filter;
    cfg->calibration.reset();
    cfg->calibration_path.clear();
    cfg->correction.reset();
    pibridge::eyemath::FilterParams nofilter;
    nofilter.enabled = false;
    cfg->filter = nofilter;
    cfg->history = 1024;
    auto link = std::make_shared<pibridge::eye::calibration::Link>();
    auto hid = std::make_shared<pibridge::eyed::HidHeadsetCommands>();
    const pibridge::eye::live::Config base = *cfg;
    Runner::StartTracker start = [base, link, hid, log](const pibridge::eyemath::FrameConv& conv) {
        auto c = base;
        c.conv = conv;
        return pibridge::eye::live::Tracker::start(
            c, std::make_unique<ChannelDevice>(std::make_unique<pibridge::eye::live::RealDevice>(hid), link), log);
    };
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    const std::string st = stamp(unix_now());
    RunFiles files{results_dir(), st, *calibration_out, opts->profile_out, opts->raw_log};
    say(std::format("eye profile: {} (the chip's new profile goes there; the old one is kept as .bak-{})",
                    opts->profile_out.string(), st));
    Runner runner(Procedure(ps), files, link, start, say);
    auto r = run_xr(runner, XrOpts{opts->res_div}, g_stop);
    runner.finish();
    if (!r) {
        return fail(r.error(), 1);
    }
    return 0;
#endif
}
