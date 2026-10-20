// SPDX-License-Identifier: MIT
// The command line over pibridge.conf and the environment (main.rs had no tests; these pin the eyed-compatible defaults):
// the start list and profile lookup (flag, environment, settings, the default files pibridge -> pimax-linux -> omarchyvr),
// where the new profile goes, flip x, the modes and the errors. Temporary folders and a fake environment only.
#include "options.h"

#include "pibridge_test.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <unistd.h>

using namespace pibridge::eyecal;
namespace fs = std::filesystem;

namespace {

struct Env {
    std::map<std::string, std::string> v;
    EnvFn fn() const {
        return [this](std::string_view n) -> const char* {
            const auto it = v.find(std::string(n));
            return it == v.end() ? nullptr : it->second.c_str();
        };
    }
};

struct Tmp {
    fs::path root;
    Dirs dirs;
    Tmp() {
        root = fs::temp_directory_path() / ("pibridge-eyecal-opts-" + std::to_string(::getpid()));
        fs::remove_all(root);
        dirs = Dirs{root / "pibridge", root / "pimax-linux", root / "omarchyvr"};
        for (const auto& d : {dirs.pibridge, dirs.legacy, dirs.omarchyvr}) {
            fs::create_directories(d);
        }
    }
    ~Tmp() { fs::remove_all(root); }
    void touch(const fs::path& p) const { std::ofstream(p) << "x"; }
};

std::expected<Options, std::string> parse(const std::vector<std::string>& a, const pibridge::hmd::Settings& s, const Tmp& t,
                                          const Env& e = {}) {
    return parse_args(a, s, t.dirs, e.fn());
}

} // namespace

TEST(defaults_are_the_files_eyed_uses) {
    const Tmp t;
    pibridge::hmd::Settings s;
    auto o = parse({}, s, t);
    CHECK(o.has_value());
    CHECK(o->run == RunKind::Chip && o->pattern == Pattern::Pimax && o->flip_x && !o->flip_y);
    CHECK(!o->frames && !o->profile);
    CHECK(o->profile_out == t.dirs.pibridge / "eye-profile-crystal-super.bin");
    // only in the omarchyvr folder: found there; the new profile still goes to the pibridge folder
    t.touch(t.dirs.omarchyvr / "eye-start-crystal-super.txt");
    t.touch(t.dirs.omarchyvr / "eye-profile-crystal-super.bin");
    o = parse({}, s, t);
    CHECK(o->frames == t.dirs.omarchyvr / "eye-start-crystal-super.txt");
    CHECK(o->profile == t.dirs.omarchyvr / "eye-profile-crystal-super.bin");
    CHECK(o->profile_out == t.dirs.pibridge / "eye-profile-crystal-super.bin");
    // pimax-linux before omarchyvr, pibridge before both
    t.touch(t.dirs.legacy / "eye-profile-crystal-super.bin");
    CHECK(parse({}, s, t)->profile == t.dirs.legacy / "eye-profile-crystal-super.bin");
    t.touch(t.dirs.pibridge / "eye-profile-crystal-super.bin");
    CHECK(parse({}, s, t)->profile == t.dirs.pibridge / "eye-profile-crystal-super.bin");
}

TEST(settings_then_environment_then_flags) {
    const Tmp t;
    pibridge::hmd::Settings s;
    s.eye_start_list = "/x/list.txt";
    s.eye_profile = "/x/profile.bin";
    s.eye_flip_x = false;
    auto o = parse({}, s, t);
    CHECK(o->frames == fs::path("/x/list.txt") && o->profile == fs::path("/x/profile.bin"));
    CHECK(o->profile_out == fs::path("/x/profile.bin")); // the new profile goes where eyed reads it
    CHECK(!o->flip_x);
    Env e;
    e.v["EYE_FRAMES"] = "/env/list.txt";
    e.v["EYE_PROFILE"] = "/env/p.bin";
    e.v["EYE_FLIP_X"] = "1";
    o = parse({}, s, t, e);
    CHECK(o->frames == fs::path("/env/list.txt") && o->profile == fs::path("/env/p.bin") && o->flip_x);
    CHECK(o->profile_out == fs::path("/env/p.bin"));
    o = parse({"--frames", "/f/l.txt", "--profile", "/f/p.bin", "--flip-x", "0"}, s, t, e);
    CHECK(o->frames == fs::path("/f/l.txt") && o->profile == fs::path("/f/p.bin") && !o->flip_x);
    CHECK(o->profile_out == fs::path("/f/p.bin"));
    // no upload at the start; the new profile still has its place
    o = parse({"--no-profile"}, s, t, e);
    CHECK(!o->profile && o->no_profile && o->profile_out == fs::path("/env/p.bin"));
}

TEST(modes_and_flags) {
    const Tmp t;
    const pibridge::hmd::Settings s;
    auto o = parse({"--recenter"}, s, t);
    CHECK(o->run == RunKind::Recenter);
    CHECK(procedure_settings(*o).mode == Mode::Recenter && procedure_settings(*o).pattern == Pattern::Pimax);
    o = parse({"--host-only", "--pattern", "grid", "--points", "13", "--pursuit"}, s, t);
    CHECK(o->run == RunKind::HostOnly);
    const auto ps = procedure_settings(*o);
    CHECK(ps.mode == Mode::Full && ps.pattern == Pattern::Grid && ps.cal_points == 13 && ps.pursuit);
    o = parse({}, s, t);
    const auto cs = procedure_settings(*o);
    CHECK(cs.mode == Mode::Chip && cs.pattern == Pattern::Pimax && cs.attract_s == 1.0 && cs.conv.flip_x);
    o = parse({"--chip-mirror", "xy", "--head-limit", "3.5", "--res-div", "2", "--no-raw-log", "--simulate", "--seed", "7",
               "--calibration", "/c/cal.toml"},
              s, t);
    CHECK(o->chip_mirror_x && o->chip_mirror_y && o->head_limit_deg == 3.5 && o->res_div == 2 && !o->raw_log);
    CHECK(o->simulate && o->seed == 7 && o->calibration == fs::path("/c/cal.toml"));
    CHECK(procedure_settings(*o).chip_mirror_x);
    CHECK(parse({"--help"}, s, t)->help);
    CHECK(settings_path_of({"--settings", "/s/pibridge.conf"}) == fs::path("/s/pibridge.conf"));
    CHECK(usage().find("--recenter") != std::string::npos);
}

TEST(bad_arguments_are_refused) {
    const Tmp t;
    const pibridge::hmd::Settings s;
    CHECK(!parse({"--pattern", "spiral"}, s, t).has_value());
    CHECK(!parse({"--points", "10"}, s, t).has_value());
    CHECK(!parse({"--head-limit", "x"}, s, t).has_value());
    CHECK(!parse({"--res-div", "0"}, s, t).has_value());
    CHECK(!parse({"--chip-mirror", "z"}, s, t).has_value());
    CHECK(!parse({"--frames"}, s, t).has_value());
    CHECK(!parse({"--recenter", "--host-only"}, s, t).has_value());
    const auto e = parse({"extra"}, s, t);
    CHECK(!e.has_value() && e.error() == "unexpected argument 'extra'");
}

PIBRIDGE_TEST_MAIN()
