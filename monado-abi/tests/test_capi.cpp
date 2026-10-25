// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/capi.rs (MIT OR Apache-2.0), through libpibridge_pimax.a and the
// verbatim header. The Rust calibration test reads tests/fixtures/headset-calibration.xml, whose numbers are the
// shipped Dream Air reference's; the twin reads the reference (deviceUID "reference").
#include "pimax_cam.h"

#include "pibridge_test.h"

#include <cstdio>
#include <cstring>
#include <string>

static std::string read(const char* path) {
    std::string s;
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) {
        return s;
    }
    char buf[4096];
    std::size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) {
        s.append(buf, n);
    }
    std::fclose(f);
    return s;
}

TEST(calib_through_c_abi) {
    const std::string xml = read(PIBRIDGE_CAM_REFERENCE_XML);
    CHECK(!xml.empty());
    pimax_cam_calib c;
    std::memset(&c, 0, sizeof c);
    CHECK(pimax_cam_calib_parse(xml.data(), xml.size(), &c) == 0);
    CHECK(c.camera_count == 4);
    CHECK(c.has_imu == 1);
    CHECK(std::string(c.cameras[1].name) == "trackingB");
    CHECK(std::string(c.device_uid) == "reference");
    CHECK(c.cameras[0].intr.extra[0] == 0.0011599741 && c.cameras[0].intr.extra[1] == -0.0001122173);
    double r[9];
    double t[3];
    CHECK(pimax_cam_calib_stereo(&c, 0, 1, r, t) == 0);
    CHECK(r[0] == -0.5777754392);
    CHECK(t[0] == -0.0996975113 && t[1] == 0.0070973858 && t[2] == -0.0497311326);
    CHECK(pimax_cam_calib_body_from_cam(&c, 0, r, t) == 0);
    CHECK(t[0] == -0.006352548001 && t[1] == -0.01406156932 && t[2] == -0.01943735371);
    CHECK(pimax_cam_calib_stereo(&c, 0, 7, r, t) == -1);
    CHECK(std::string(pimax_cam_last_error()).find("bad camera index") != std::string::npos);
    CHECK(pimax_cam_calib_parse("<x/>", 4, &c) == -1);
}

TEST(open_failure_sets_error) {
    CHECK(pimax_cam_open("/nonexistent/clip.pxcam", nullptr) == nullptr);
    CHECK(std::string(pimax_cam_last_error()).find("open") != std::string::npos);
    pimax_cam_close(nullptr);
    CHECK(pimax_cam_next(nullptr, 0, nullptr) == -1);
}

TEST(clock_through_c_abi) {
    const pimax_cam_imu_clock_ref r{1000, 1, 1000, 5};
    CHECK(pimax_cam_dev_to_mono_ns(3000, &r) == 3'000'005ull);
    CHECK(pimax_cam_dev_to_mono_ns(3000, nullptr) == 0);
}

TEST(errors_never_escape_the_abi) {
    // a struct claiming 8 cameras: the index check trusts camera_count, the conversion keeps 4 (Rust panics and its
    // guard reports it); here the library's exception is caught the same way
    pimax_cam_calib c;
    std::memset(&c, 0, sizeof c);
    c.camera_count = 8;
    double r[9];
    double t[3];
    CHECK(pimax_cam_calib_stereo(&c, 0, 5, r, t) == -1);
    CHECK(std::string(pimax_cam_last_error()) == "panic inside pimax_cam");
    CHECK(std::string(pimax_cam_version()) == "0.1.0");
}

PIBRIDGE_TEST_MAIN()
