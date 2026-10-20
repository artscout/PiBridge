// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/calib.rs (MIT OR Apache-2.0). The Rust tests read
// tests/fixtures/headset-calibration.xml; its numbers are the shipped Dream Air reference's (only the deviceUID
// differs), so the twins read the reference that PiBridge embeds.
#include "common.h"
#include "pibridge/cam/calib.h"
#include "pibridge/cam/io.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdio>
#include <filesystem>

using namespace pibridge::cam;
namespace geom = pibridge::cam::geom;

static std::string xml_doc() {
    return std::string(reference_xml());
}

static std::string replace(std::string s, const std::string& a, const std::string& b) {
    const auto p = s.find(a);
    if (p != std::string::npos) {
        s.replace(p, a.size(), b);
    }
    return s;
}

static void write(const std::string& p, const std::string& s) {
    std::filesystem::create_directories(std::filesystem::path(p).parent_path());
    std::FILE* f = std::fopen(p.c_str(), "wb");
    std::fwrite(s.data(), 1, s.size(), f);
    std::fclose(f);
}

TEST(parses_factory_file) {
    const auto c = *DeviceCalib::parse(xml_doc());
    CHECK(c.uid == "reference");
    CHECK(c.sensor == "ov7251");
    CHECK(c.cameras.size() == 4);
    const char* names[] = {"trackingA", "trackingB", "Pi-controllerA", "Pi-controllerB"};
    for (int i = 0; i < 4; i++) {
        CHECK(c.cameras[i].name == names[i]);
    }
    const auto& a = c.cameras[0];
    CHECK(a.model == "FISHEYE_4_PARAMETERS");
    CHECK(a.intr.width == 640 && a.intr.height == 480);
    CHECK(a.intr.cx == 325.3779138659);
    CHECK(a.intr.fy == 234.7063990438);
    CHECK((a.intr.k == std::array<double, 4>{0.0589502951, -0.0603263194, 0.0320281449, -0.0082048556}));
    CHECK((a.intr.extra == std::vector<double>{0.0011599741, -0.0001122173}));
    CHECK(a.rig_r == geom::IDENTITY);
    CHECK((c.cameras[1].rig_t == geom::Vec3{-0.0996975113, 0.0070973858, -0.0497311326}));
    CHECK(c.cameras[3].rig_r[2][2] == 0.1736576490);
    const auto& imu = *c.imu;
    CHECK((imu.ombc == geom::Vec3{-0.07417106183, -2.573029401, 0.6280404502}));
    CHECK(imu.delta_s == 0.001637992156);
    bool wbias = false;
    for (const auto& [k, v] : imu.raw) {
        wbias = wbias || k == "wBias";
    }
    CHECK(wbias);
}

TEST(stereo_pair_a_b) {
    const auto c = *DeviceCalib::parse(xml_doc());
    const auto ab = c.stereo(0, 1, RigConvention::CamFromCam0);
    // camera 0 is the reference: b_from_a is exactly the rig of camera 1.
    CHECK(ab.r == c.cameras[1].rig_r);
    CHECK(ab.t == c.cameras[1].rig_t);
    const double baseline = geom::norm(ab.inverse().t);
    CHECK(std::fabs(baseline - 0.1116) < 0.001);
    // the other convention gives the same baseline length but a different centre
    const auto ab1 = c.stereo(0, 1, RigConvention::Cam0FromCam);
    CHECK(std::fabs(geom::norm(ab1.inverse().t) - baseline) < 1e-9);
    // pair 2-3 composes two rigs
    const auto p23 = c.stereo(2, 3, RigConvention::CamFromCam0);
    const geom::Vec3 x0{0.1, 0.2, 1.0};
    const auto via = c.cam_from_cam0(3, RigConvention::CamFromCam0).apply(x0);
    const auto direct = p23.apply(c.cam_from_cam0(2, RigConvention::CamFromCam0).apply(x0));
    // the XML rotations are orthonormal to ~1e-10 only
    CHECK(geom::norm(geom::sub(via, direct)) < 1e-8);
}

TEST(body_frame_symmetry) {
    // The evidence for the conventions: A-B baseline along body +X, optical axes symmetric about YZ.
    const auto c = *DeviceCalib::parse(xml_doc());
    const auto conv = RigConvention::CamFromCam0;
    const auto a = *c.body_from_cam(0, conv);
    const auto b = *c.body_from_cam(1, conv);
    const auto base = geom::normalize(geom::sub(b.t, a.t));
    CHECK(base[0] > 0.999);
    const auto za = geom::col(a.r, 2);
    const auto zb = geom::col(b.r, 2);
    const auto bis = geom::normalize(geom::add(za, zb));
    CHECK(std::fabs(bis[0]) < 0.03 && bis[2] < -0.85 && bis[1] < -0.4);
    // image "down" (+y_cam) of A points up in the body frame (A is mounted upside down); B is rolled 180 deg
    // relative to A (mirror-symmetric mounting), so B's picture is upright.
    CHECK(geom::col(a.r, 1)[1] > 0.85);
    CHECK(geom::col(b.r, 1)[1] < -0.85);
    // A is the left camera (x < B's x) and both are ~27 deg off the bisector.
    CHECK(geom::angle_deg(za, bis) > 25.0 && geom::angle_deg(za, bis) < 29.0);
    CHECK(!c.geometry_report(conv).empty());
}

TEST(bad_documents) {
    auto e = DeviceCalib::parse("<Foo/>");
    CHECK(!e && e.error().kind == CalibError::Kind::Format);
    e = DeviceCalib::parse("<DeviceConfiguration/>");
    CHECK(!e && e.error().kind == CalibError::Kind::Format);
    e = DeviceCalib::parse("<DeviceConfiguration");
    CHECK(!e && e.error().kind == CalibError::Kind::Xml);
    e = DeviceCalib::parse(replace(xml_doc(), "234.7133343909", "abc"));
    CHECK(!e && e.error().kind == CalibError::Kind::Format);
    CHECK(e.error().text() == "calibration: focal_length: bad number \"abc\"");
    e = DeviceCalib::parse(replace(xml_doc(), "-0.5777754392 -0.1162377714", "-0.9777754392 -0.1162377714"));
    CHECK(!e && e.error().kind == CalibError::Kind::Format);
    CHECK(e.error().text() == "calibration: camera 1: rig rotation is not a rotation");
    e = DeviceCalib::load("/nonexistent/x.xml");
    CHECK(!e && e.error().kind == CalibError::Kind::Io);
    CHECK(e.error().text() == "/nonexistent/x.xml: No such file or directory (os error 2)");
}

TEST(the_shipped_reference_parses_and_carries_no_serial) {
    const auto c = *DeviceCalib::parse(reference_xml());
    CHECK(c.is_reference());
    CHECK(c.cameras.size() == 4);
    CHECK(c.imu.has_value()); // the 6DoF export needs ombc/tbc
    CHECK(c.body_from_cam(0, RigConvention::CamFromCam0).has_value());
    CHECK(reference_xml().find("deviceUID=\"P") == std::string_view::npos);
}

TEST(the_headsets_own_file_wins_and_the_reference_is_the_fallback) {
    const std::string d = pibridge::cam::testing::tmp("ref");
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    const std::string own = d + "/own.xml";
    const std::string missing = d + "/missing.xml";
    // nothing on disk: the reference, reported as built in (no installed copy under this home)
    auto r = *DeviceCalib::load_first({missing}, true, d);
    CHECK(r.first.is_reference());
    CHECK(r.second.find("built-in reference") != std::string::npos);
    // the installed copy is reported by its path
    const std::string inst = path_join(d, REFERENCE_INSTALLED);
    write(inst, xml_doc());
    CHECK(DeviceCalib::load_first({missing}, true, d)->second == inst);
    // fallback off: not found, and the error names the places tried
    auto nf = DeviceCalib::load_first({missing}, false, d);
    CHECK(!nf && nf.error().kind == CalibError::Kind::NotFound);
    CHECK((nf.error().tried == std::vector<std::string>{missing}));
    // the user's own file wins over the reference
    write(own, replace(xml_doc(), "deviceUID=\"reference\"", "deviceUID=\"mine\""));
    r = *DeviceCalib::load_first({missing, own}, true, d);
    CHECK(r.first.uid == "mine" && r.second == own);
    CHECK(!r.first.is_reference());
    std::filesystem::remove_all(d);
}

TEST(the_reference_fallback_can_be_switched_off) {
    CHECK(reference_allowed([](std::string_view) { return std::optional<std::string>{}; }));
    CHECK(reference_allowed([](std::string_view k) {
        return k == "PIMAX_NO_REFERENCE" ? std::optional<std::string>("0") : std::nullopt;
    }));
    CHECK(!reference_allowed([](std::string_view k) {
        return k == "PIMAX_NO_REFERENCE" ? std::optional<std::string>("1") : std::nullopt;
    }));
}

// PiBridge only (docs/CONVENTIONS.md): ~/.config/pibridge/ and ~/.config/pimax-linux/ come before omarchyvr's folder, but
// only when the file is there, so the search (and the "not found" text) is the Rust one otherwise.
TEST(pibridge_config_folders_first_when_present) {
    const std::string h = pibridge::cam::testing::tmp("home");
    std::filesystem::remove_all(h);
    std::filesystem::create_directories(h);
    const char* saved = std::getenv("HOME");
    const std::string saved_home = saved != nullptr ? saved : "";
    ::setenv("HOME", h.c_str(), 1);
    ::unsetenv("PIMAX_CAM_CALIB");
    ::unsetenv("PIMAX_CALIB");
    ::unsetenv("PIMAX_REPO");
    ::unsetenv("PIBRIDGE_CAM_CALIB");
    ::unsetenv("PIBRIDGE_CALIB");
    ::unsetenv("PIBRIDGE_REPO");
    auto c = DeviceCalib::default_candidates();
    CHECK(c.size() == 2);
    CHECK(c[0] == h + "/.config/omarchyvr/headset-calibration.xml");
    CHECK(c[1] == h + "/omarchyvr/analysis/headset-calibration.xml");
    write(h + "/.config/omarchyvr/headset-calibration.xml",
          replace(xml_doc(), "deviceUID=\"reference\"", "deviceUID=\"omarchyvr\""));
    write(h + "/.config/pimax-linux/headset-calibration.xml",
          replace(xml_doc(), "deviceUID=\"reference\"", "deviceUID=\"pimax-linux\""));
    CHECK(DeviceCalib::load_default(std::nullopt)->first.uid == "pimax-linux");
    write(h + "/.config/pibridge/headset-calibration.xml",
          replace(xml_doc(), "deviceUID=\"reference\"", "deviceUID=\"pibridge\""));
    CHECK(DeviceCalib::load_default(std::nullopt)->first.uid == "pibridge");
    c = DeviceCalib::default_candidates();
    CHECK(c.size() == 4);
    // PIBRIDGE_CAM_CALIB wins over PIMAX_CAM_CALIB
    write(h + "/a.xml", replace(xml_doc(), "deviceUID=\"reference\"", "deviceUID=\"a\""));
    write(h + "/b.xml", replace(xml_doc(), "deviceUID=\"reference\"", "deviceUID=\"b\""));
    ::setenv("PIMAX_CAM_CALIB", (h + "/a.xml").c_str(), 1);
    CHECK(DeviceCalib::load_default(std::nullopt)->first.uid == "a");
    ::setenv("PIBRIDGE_CAM_CALIB", (h + "/b.xml").c_str(), 1);
    CHECK(DeviceCalib::load_default(std::nullopt)->first.uid == "b");
    ::unsetenv("PIMAX_CAM_CALIB");
    ::unsetenv("PIBRIDGE_CAM_CALIB");
    if (saved != nullptr) {
        ::setenv("HOME", saved_home.c_str(), 1);
    }
    std::filesystem::remove_all(h);
}

PIBRIDGE_TEST_MAIN()
