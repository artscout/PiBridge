// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/overlay.rs (MIT OR Apache-2.0); calibration: the embedded reference.
#include "common.h"
#include "pibridge/cam/overlay.h"
#include "pibridge_test.h"

#include <cmath>
#include <filesystem>

using namespace pibridge::cam;
using namespace pibridge::cam::overlay;
namespace geom = pibridge::cam::geom;

static DeviceCalib calib() {
    return *DeviceCalib::parse(reference_xml());
}

static CalibOverlay sample(const DeviceCalib& c) {
    CalibOverlay o = CalibOverlay::for_calib(c, "test");
    CameraRotation r;
    r.id = 1;
    r.rotation_vector = {0.004, -0.001, 0.0005};
    r.sigma = geom::Vec3{1e-4, 3e-4, 1e-4};
    o.rotations.push_back(r);
    o.quality.emplace_back("vertical_rms_after_px", 0.31);
    o.note = "unit \"test\"";
    return o;
}

static std::string replace(std::string s, const std::string& a, const std::string& b) {
    const auto p = s.find(a);
    if (p != std::string::npos) {
        s.replace(p, a.size(), b);
    }
    return s;
}

TEST(json_roundtrip) {
    const auto c = calib();
    const auto o = sample(c);
    const auto back = CalibOverlay::parse(o.to_json());
    CHECK(back && *back == o);
    CHECK(std::fabs(o.max_angle_deg() - 0.2381) < 1e-3);
    const auto empty = CalibOverlay::for_calib(c, "t");
    CHECK(*CalibOverlay::parse(empty.to_json()) == empty);
}

TEST(rejects_bad_files) {
    CHECK(!CalibOverlay::parse("{}"));
    CHECK(!CalibOverlay::parse("not json"));
    const auto o = sample(calib()).to_json();
    const auto v2 = CalibOverlay::parse(replace(o, "\"version\": 1", "\"version\": 2"));
    CHECK(!v2 && v2.error().text() == "overlay: version 2, this reader knows 1");
    CHECK(!CalibOverlay::parse(replace(o, "\"id\": 1", "\"id\": 1.5")));
    CHECK(!CalibOverlay::parse(replace(o, "[0.004, -0.001, 0.0005]", "[0.004]")));
}

TEST(apply_rotates_about_the_camera_centre) {
    const auto c = calib();
    const auto o = sample(c);
    const auto c2 = *with_overlay(c, o, DEFAULT_MAX_ANGLE_DEG);
    const auto conv = RigConvention::CamFromCam0;
    // camera centre unchanged, orientation changed by exactly the correction
    const auto centre = [&](const DeviceCalib& cal) { return cal.cam_from_cam0(1, conv).inverse().t; };
    CHECK(geom::norm(geom::sub(centre(c), centre(c2))) < 1e-12);
    const auto d = geom::mat_mul(c2.cam_from_cam0(1, conv).r, geom::transpose(c.cam_from_cam0(1, conv).r));
    const auto e = geom::rodrigues(o.rotations[0].rotation_vector);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // the XML rotations are orthonormal to ~1e-10 only
            CHECK(std::fabs(d[i][j] - e[i][j]) < 1e-8);
        }
    }
    // other cameras and the IMU extrinsic untouched
    CHECK(c2.cameras[0] == c.cameras[0]);
    CHECK(c2.cameras[2] == c.cameras[2]);
    CHECK(c2.imu == c.imu);
    // stereo pair follows: X_1' = E X_1
    const geom::Vec3 p{0.3, -0.2, 1.5};
    const auto s = c.stereo(0, 1, conv).apply(p);
    const auto s2 = c2.stereo(0, 1, conv).apply(p);
    CHECK(geom::norm(geom::sub(geom::mat_vec(e, s), s2)) < 1e-12);
}

TEST(refusals) {
    const auto c = calib();
    auto o = sample(c);
    o.device_uid = "OTHER";
    CHECK(!with_overlay(c, o, 3.0));
    o = sample(c);
    o.factory_date = "1999";
    CHECK(!with_overlay(c, o, 3.0));
    o = sample(c);
    o.rotations[0].rotation_vector = {0.1, 0.0, 0.0}; // 5.7 deg
    const auto r = with_overlay(c, o, 3.0);
    CHECK(!r);
    CHECK(r.error().text() == "calibration: overlay refused: camera 1: correction 5.73 deg exceeds the 3 deg limit");
    CHECK(with_overlay(c, o, 6.0).has_value());
    o = sample(c);
    o.rotations[0].id = 7;
    CHECK(!with_overlay(c, o, 3.0));
    o = sample(c);
    o.device_uid.clear();
    o.factory_date.clear();
    CHECK(with_overlay(c, o, 3.0).has_value());
}

TEST(save_rollback_clear) {
    const std::string dir = pibridge::cam::testing::tmp("overlay");
    std::filesystem::remove_all(dir);
    const std::string path = dir + "/sub/cam-calib-overlay.json";
    const auto c = calib();
    auto a = sample(c);
    a.note = "first";
    auto b = sample(c);
    b.note = "second";
    CHECK(save(a, path).has_value());
    CHECK(!path_exists(prev_path(path)));
    CHECK(save(b, path).has_value());
    CHECK(CalibOverlay::load(path)->note == "second");
    CHECK(CalibOverlay::load(prev_path(path))->note == "first");
    CHECK(rollback(path).has_value());
    CHECK(CalibOverlay::load(path)->note == "first");
    CHECK(rollback(path).has_value()); // undo the rollback
    CHECK(CalibOverlay::load(path)->note == "second");
    CHECK(clear(path)->size() == 2);
    CHECK(!rollback(path));
    CHECK(clear(path)->empty());
    std::filesystem::remove_all(dir);
}

PIBRIDGE_TEST_MAIN()
