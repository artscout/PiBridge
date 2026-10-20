// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/capi_slam.rs (MIT OR Apache-2.0), through libpibridge_pimax.a. The Rust
// lab-data tests load the project headset's calibration; the twins load the shipped reference (the same file with
// the serial scrubbed).
#include "pimax_cam.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/slam.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

namespace {

pimax_cam_calib load() {
    pimax_cam_calib c;
    std::memset(&c, 0, sizeof c);
    const int r = pimax_cam_calib_load(PIBRIDGE_CAM_REFERENCE_XML, &c, nullptr, 0);
    CHECK(r == 0);
    return c;
}

std::string read(const std::string& p) {
    std::ifstream f(p);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

} // namespace

TEST(t_imu_cam_column_major) {
    const auto c = load();
    double m[16] = {};
    CHECK(pimax_cam_slam_t_imu_cam(&c, 1, m) == 0);
    const auto cal = *pibridge::cam::DeviceCalib::load(PIBRIDGE_CAM_REFERENCE_XML);
    const auto p = *pibridge::cam::slam::t_imu_cam(cal, 1);
    CHECK(m[12] == p.t[0] && m[13] == p.t[1] && m[14] == p.t[2]);
    CHECK(m[1] == p.r[1][0]); // column 0, row 1
    CHECK(m[15] == 1.0);
    CHECK(pimax_cam_slam_t_imu_cam(&c, 7, m) == -1);
}

TEST(write_files_and_cams) {
    const auto c = load();
    const char* t = std::getenv("TMPDIR");
    const std::string dir = std::string(t != nullptr && *t != 0 ? t : "/tmp") + "/omarchyvr-slam-capi-" +
                            std::to_string(::getpid());
    pimax_cam_slam_imu imu;
    std::memset(&imu, 0, sizeof imu);
    CHECK(pimax_cam_slam_imu_load("/nonexistent/slam-imu.json", &imu) == -1);
    // filled with defaults even on error
    CHECK(imu.rate_hz == 1000.0);
    uint32_t ids[4] = {};
    CHECK(pimax_cam_slam_parse_cams("0,1,2,3", 4, ids, 4) == 4);
    CHECK(pimax_cam_slam_write_basalt(&c, ids, 2, 25.0, 0, &imu, dir.c_str(), nullptr) == 0);
    const std::string txt = read(dir + "/pimax-basalt-calib.json");
    CHECK(txt.find("\"kb4\"") != std::string::npos);
    CHECK(!read(dir + "/pimax-slam-info.json").empty());
    [[maybe_unused]] const int rc = std::system(("rm -rf '" + dir + "'").c_str());
}

TEST(track_through_c) {
    pimax_cam_track_params p;
    pimax_cam_track_default_params(&p);
    p.acquire_poses = 2;
    pimax_cam_track* t = pimax_cam_track_create(&p, nullptr);
    CHECK(t != nullptr);
    pimax_cam_track_out out;
    std::memset(&out, 0, sizeof out);
    for (uint64_t i = 0; i < 3; i++) {
        const pimax_cam_track_sample s = {1'000'000'000ull + i * 40'000'000ull, {1.0, 2.0, 3.0}, {0, 0, 0, 1},
                                          {0, 0, 0, 1}};
        CHECK(pimax_cam_track_update(t, s.ts_ns, &s, &out) == 0);
    }
    CHECK(out.state == 1);
    CHECK(out.world_from_slam_pos[0] == -1.0 && out.world_from_slam_pos[1] == -2.0 && out.world_from_slam_pos[2] == -3.0);
    pimax_cam_track_destroy(t);

    // New path: IMU + VIO with velocity + head query.
    t = pimax_cam_track_create(&p, nullptr);
    const double g[3] = {0.0, 0.0, 0.0};
    const double a[3] = {0.0, 9.80665, 0.0};
    for (int64_t i = 0; i < 200; i++) {
        pimax_cam_track_push_imu(t, 1'000'000'000 + i * 1'000'000, a, g);
    }
    for (uint64_t i = 0; i < 3; i++) {
        const pimax_cam_track_vio s = {1'000'000'000ull + i * 40'000'000ull, {1.0, 2.0, 3.0}, {0, 0, 0, 1}, {0, 0, 0},
                                       {0, 0, 0, 1}};
        CHECK(pimax_cam_track_update_vio(t, s.ts_ns, &s, &out) == 0);
    }
    pimax_cam_rel r3;
    std::memset(&r3, 0, sizeof r3);
    r3.q[3] = 1.0;
    r3.flags = 17;
    pimax_cam_rel r;
    std::memset(&r, 0, sizeof r);
    CHECK(pimax_cam_track_head(t, 1'150'000'000, 1'170'000'000, &r3, &r) == 1);
    CHECK(r.flags == 63);
    CHECK(std::fabs(r.pos[0]) < 1e-6 && std::fabs(r.pos[1]) < 1e-6 && std::fabs(r.pos[2]) < 1e-6);
    pimax_cam_track_info info;
    std::memset(&info, 0, sizeof info);
    CHECK(pimax_cam_track_get_info(t, &info) == 0);
    CHECK(info.last_imu_ns == 1'199'000'000);
    pimax_cam_track_destroy(t);
}

PIBRIDGE_TEST_MAIN()
