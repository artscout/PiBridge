// SPDX-License-Identifier: MIT
// PiBridge's additions to the SLAM C ABI (not in the Rust code, docs/research/head-tracking-improvements.md): the
// recorder's pimax_cam_slam_write_basalt writes the calibration the tracker really uses (PIMAX_SLAM_RECORD_ACCEL_BIAS,
// off with PIBRIDGE_HEAD_PARITY=1), and pimax_cam_track_create's describe line names the PiBridge stages.
#include "pimax_cam.h"

#include "pibridge_test.h"

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

// Every scratch directory of this run, removed at exit.
struct Scratch {
    std::vector<std::string> dirs;
    ~Scratch() {
        for (const auto& d : dirs) {
            std::error_code ec;
            std::filesystem::remove_all(d, ec);
        }
    }
};
Scratch g_scratch;

std::string scratch(const std::string& name) {
    const char* t = std::getenv("TMPDIR");
    const std::string d = std::string(t != nullptr && *t != 0 ? t : "/tmp") + "/pibridge-capi-slam-pibridge-" +
                          std::to_string(::getpid()) + "-" + name;
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    g_scratch.dirs.push_back(d);
    return d;
}

std::string read(const std::string& p) {
    std::ifstream f(p);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

void put(const std::string& path, const std::string& text) {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream(path) << text;
}

void clean_env(const std::string& home) {
    for (const char* k : {"PIBRIDGE_HEAD_PARITY", "PIMAX_SLAM_RECORD_ACCEL_BIAS", "SLAM_CONFIG", "PIMAX_CAM_CALIB",
                          "PIMAX_CALIB", "PIBRIDGE_CAM_CALIB", "PIBRIDGE_CALIB", "XDG_CACHE_HOME", "PIMAX_REPO"}) {
        ::unsetenv(k);
    }
    ::setenv("HOME", home.c_str(), 1);
    ::setenv("PIMAX_NO_REFERENCE", "1", 1);
}

pimax_cam_calib load() {
    pimax_cam_calib c;
    std::memset(&c, 0, sizeof c);
    CHECK(pimax_cam_calib_load(PIBRIDGE_CAM_REFERENCE_XML, &c, nullptr, 0) == 0);
    return c;
}

int write(const std::string& dir) {
    const pimax_cam_calib c = load();
    const uint32_t ids[2] = {0, 1};
    return pimax_cam_slam_write_basalt(&c, ids, 2, 25.0, 0, nullptr, dir.c_str(), "test");
}

constexpr const char* USED =
    "{\"value0\": {\"calib_accel_bias\": [-0.05, 0.34, -0.02, 0, 0, 0, 0, 0, 0], \"calib_gyro_bias\": [0.001, "
    "-0.002, 0.0235, 0, 0, 0, 0, 0, 0, 0, 0, 0], \"marker\": \"the tracker's own file\"}}\n";

} // namespace

TEST(recorder_copies_the_slam_config_calibration) {
    const std::string h = scratch("copy");
    clean_env(h);
    put(h + "/slam/used.json", USED);
    put(h + "/slam/pimax-basalt.toml", "show-gui=0\ncam-calib=\"" + h + "/slam/used.json\"\n");
    ::setenv("SLAM_CONFIG", (h + "/slam/pimax-basalt.toml").c_str(), 1);
    CHECK(write(h + "/rec") == 0);
    CHECK(read(h + "/rec/pimax-basalt-calib.json") == USED);
    const std::string info = read(h + "/rec/pimax-slam-info.json");
    CHECK(info.find("\"accel_bias_m_s2\": [-0.05, 0.34, -0.02]") != std::string::npos);
    CHECK(info.find("\"gyro_bias_rad_s\": [0.001, -0.002, 0.0235]") != std::string::npos);
    CHECK(info.find("copy of the SLAM_CONFIG calibration " + h + "/slam/used.json") != std::string::npos);
    clean_env(h);
}

TEST(recorder_off_and_parity_keep_the_driver_file) {
    const std::string h = scratch("off");
    for (const char* sw : {"PIMAX_SLAM_RECORD_ACCEL_BIAS", "PIBRIDGE_HEAD_PARITY"}) {
        clean_env(h);
        put(h + "/slam/used.json", USED);
        put(h + "/slam/pimax-basalt.toml", "cam-calib=\"" + h + "/slam/used.json\"\n");
        ::setenv("SLAM_CONFIG", (h + "/slam/pimax-basalt.toml").c_str(), 1);
        ::setenv(sw, std::strcmp(sw, "PIBRIDGE_HEAD_PARITY") == 0 ? "1" : "0", 1);
        std::filesystem::remove_all(h + "/rec");
        CHECK(write(h + "/rec") == 0);
        const std::string calib = read(h + "/rec/pimax-basalt-calib.json");
        CHECK(calib != USED);
        CHECK(calib.find("\"calib_accel_bias\": [0.0, 0.0, 0.0") != std::string::npos);
        CHECK(read(h + "/rec/pimax-slam-info.json").find("calib_source") == std::string::npos);
    }
    clean_env(h);
}

TEST(recorder_without_slam_config_takes_the_factory_abias) {
    const std::string h = scratch("factory");
    clean_env(h);
    // The shipped reference counts as no headset file (its aBias is not this headset's): nothing is added.
    ::setenv("PIMAX_CAM_CALIB", PIBRIDGE_CAM_REFERENCE_XML, 1);
    CHECK(write(h + "/rec0") == 0);
    CHECK(read(h + "/rec0/pimax-basalt-calib.json").find("\"calib_accel_bias\": [0.0, 0.0, 0.0") != std::string::npos);
    // A headset's own file (here the reference with another device id): its Stateinit aBias goes in.
    std::string xml = read(PIBRIDGE_CAM_REFERENCE_XML);
    const std::size_t at = xml.find("deviceUID=\"reference\"");
    CHECK(at != std::string::npos);
    xml.replace(at, std::strlen("deviceUID=\"reference\""), "deviceUID=\"test-unit\"");
    put(h + "/headset.xml", xml);
    ::setenv("PIMAX_CAM_CALIB", (h + "/headset.xml").c_str(), 1);
    CHECK(write(h + "/rec1") == 0);
    CHECK(read(h + "/rec1/pimax-basalt-calib.json")
              .find("\"calib_accel_bias\": [0.07980569618, -0.02250049623, 0.03881475541") != std::string::npos);
    CHECK(read(h + "/rec1/pimax-slam-info.json").find("the factory aBias of " + h + "/headset.xml") !=
          std::string::npos);
    clean_env(h);
}

PIBRIDGE_TEST_MAIN()
