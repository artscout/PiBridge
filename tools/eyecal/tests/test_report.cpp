// SPDX-License-Identifier: MIT
// The report (report.rs has no Rust tests of its own; its output is checked by the sim tests there): the stamp, the
// backup writer, the summary of a run and the results file.
#include "report.h"

#include "procedure.h"
#include "sim.h"

#include "pibridge_test.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace pibridge::eyecal;
namespace fs = std::filesystem;

namespace {

fs::path temp_dir(const std::string& name) {
    const auto d = fs::temp_directory_path() / ("pibridge-eyecal-" + name + "-" + std::to_string(::getpid()));
    fs::remove_all(d);
    fs::create_directories(d);
    return d;
}

std::string read(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

TEST(stamps_are_utc_civil_time) {
    CHECK(stamp(0) == "19700101-000000");
    CHECK(stamp(951782400) == "20000229-000000"); // a leap day
    CHECK(stamp(1791956330) == "20261014-053850");
    CHECK(stamp(4102444799) == "20991231-235959");
}

TEST(write_with_backup_keeps_the_old_file) {
    const auto d = temp_dir("backup");
    const auto p = d / "sub" / "eye-calibration.toml";
    auto r = write_with_backup(p, "one", "20261019-120000");
    CHECK(r.has_value() && !r->has_value());
    CHECK(read(p) == "one");
    r = write_with_backup(p, "two", "20261019-120000");
    CHECK(r.has_value() && r->has_value());
    CHECK(read(p) == "two");
    CHECK(read(d / "sub" / "eye-calibration.toml.bak-20261019-120000") == "one");
    // the same stamp again gets a number
    r = write_with_backup(p, "three", "20261019-120000");
    CHECK(r.has_value() && **r == d / "sub" / "eye-calibration.toml.bak-20261019-120000-1");
    CHECK(read(**r) == "two");
    fs::remove_all(d);
}

TEST(summary_and_results_of_a_run) {
    const auto out = run(Procedure(Settings{}), SimParams{});
    const auto s = summary(out.procedure);
    CHECK(s.text.starts_with("validation (10 of 10 targets): accuracy mean"));
    CHECK(s.hud.size() >= 4 && s.hud[0].starts_with("ACCURACY "));
    CHECK(s.notes.size() == 1 && s.notes[0].starts_with("validated "));
    const auto j = json(out.procedure);
    for (const char* key : {"\"tool\": \"pibridge-eyecal\"", "\"mode\": \"host\"", "\"flip_xyz\": [true, false, true]",
                            "\"eye_pos_m\": [[-0.0320, 0.0000, 0.0000], [0.0320, 0.0000, 0.0000]]", "\"fits\": {\"left\": {",
                            "\"calibration_targets\": [", "\"validation_targets\": [", "\"chip_validation\": null",
                            "\"calibration\": \"# pimax eye calibration"}) {
        CHECK(j.find(key) != std::string::npos);
    }
    // an empty run
    const Procedure p(Settings{});
    const auto e = summary(p);
    CHECK(e.hud.size() == 1 && e.hud[0] == "NO CALIBRATION");
    CHECK(json(p).find("\"calibration\": null") != std::string::npos);
    // the chip mode's results carry the chip part
    const auto c = run(Procedure(Settings{}.chip()), SimParams{});
    const auto cj = json(c.procedure);
    CHECK(cj.find("\"mode\": \"chip\"") != std::string::npos);
    CHECK(cj.find("\"chip_profile_bytes\": 33887") != std::string::npos);
    CHECK(cj.find("\"chip_validation\": {\"left\": {\"n\": 5") != std::string::npos);
    CHECK(cj.find("\"chip_target\": 8") != std::string::npos);
}

PIBRIDGE_TEST_MAIN()
