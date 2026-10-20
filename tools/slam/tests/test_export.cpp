// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr omarchyvr-slam/src/export.rs, imu.rs and euroc.rs (the load part) (MIT OR
// Apache-2.0).
#include "export.h"

#include "pibridge/cam/io.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdlib>
#include <format>
#include <string>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::slamtool;

namespace {

std::string tmpdir(const std::string& name) {
    const char* d = std::getenv("TMPDIR");
    std::string p = std::string(d != nullptr && *d != 0 ? d : "/tmp") + "/pibridge-slam-test-" +
                    std::to_string(::getpid()) + "-" + name;
    [[maybe_unused]] const int rc_ = std::system(("rm -rf '" + p + "'").c_str());
    return p;
}

cam::DeviceCalib reference() {
    return *cam::DeviceCalib::parse(cam::reference_xml());
}

// imu.rs: deterministic pseudo-random normal-ish noise (sum of 12 uniforms).
double noise(std::uint64_t& seed) {
    double s = 0.0;
    for (int i = 0; i < 12; i++) {
        seed = seed * 6364136223846793005ull + 1442695040888963407ull;
        s += static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53);
    }
    return s - 6.0;
}

} // namespace

TEST(overrides_and_roundtrip) {
    const std::string base =
        "{\"value0\": {\"config.optical_flow_type\": \"frame_to_frame\", \"config.optical_flow_detection_grid_size\": "
        "50, \"config.vio_obs_std_dev\": 0.5, \"config.optical_flow_recall_max_patch_norms\": [1.74, 0.96]}}";
    const auto out = vio_config(base);
    CHECK(out.has_value());
    const auto v = cam::json::parse(*out);
    const auto* m = v->get("value0");
    CHECK(m->get("config.optical_flow_detection_grid_size")->as_f64() == 40.0);
    CHECK(out->find("\"config.optical_flow_detection_grid_size\": 40,") != std::string::npos);
    CHECK(m->get("config.vio_obs_std_dev")->as_f64() == 0.5);
    CHECK(*m->get("config.optical_flow_type")->as_str() == "frame_to_frame");
}

// export.rs export_writes_all_files runs on the headset's own calibration (lab data); the twin uses the shipped
// reference with the same checks.
TEST(export_writes_all_files) {
    const auto cal = reference();
    const std::string dir = tmpdir("export");
    const std::string base = dir + "/base.json";
    CHECK(cam::create_dir_all(dir).has_value());
    CHECK(cam::write_file(base, "{\"value0\": {\"config.vio_max_states\": 3}}").has_value());
    const auto e = export_files(dir, cal, cam::slam::SlamSetup{}, cam::slam::ImuNoise{}, base, false);
    CHECK(e.has_value());
    const auto t = *cam::read_to_string(e->toml);
    CHECK(t.find("show-gui=0") != std::string::npos);
    CHECK(t.find(std::format("cam-calib=\"{}\"", e->calib)) != std::string::npos);
    CHECK(!e->calib.empty() && e->calib.front() == '/');
    CHECK(cam::is_file(e->config));
    [[maybe_unused]] const int rc_ = std::system(("rm -rf '" + dir + "'").c_str());
}

// PiBridge: four cameras keep every value of the two-camera config and switch on Basalt's own corners in the cameras
// beyond cam0's view; two cameras leave the key exactly as the base has it.
TEST(four_cameras_turn_on_nonoverlap_detection) {
    const std::string off = "{\"value0\": {\"config.optical_flow_detection_nonoverlap\": false, \"config.vio_max_states\": 3}}";
    const auto two = vio_config(off, 2);
    const auto four = vio_config(off, 4);
    CHECK(two.has_value() && four.has_value());
    CHECK(*two == *vio_config(off));
    CHECK(two->find("\"config.optical_flow_detection_nonoverlap\": false") != std::string::npos);
    CHECK(four->find("\"config.optical_flow_detection_nonoverlap\": true") != std::string::npos);
    const auto v = cam::json::parse(*four);
    CHECK(v->get("value0")->get("config.vio_max_states")->as_f64() == 3.0);
    // a base without the key gets it with four cameras only
    const std::string none = "{\"value0\": {\"config.vio_max_states\": 3}}";
    CHECK(vio_config(none, 2)->find("nonoverlap") == std::string::npos);
    CHECK(vio_config(none, 4)->find("\"config.optical_flow_detection_nonoverlap\": true") != std::string::npos);
    // the built-in base already has it on: four cameras change nothing there
    CHECK(*vio_config(builtin_base(), 4) == *vio_config(builtin_base(), 2));
}

// PiBridge: a four-camera export holds the two-camera calibration's cameras first, unchanged, then cameras 2 and 3.
TEST(four_camera_export_extends_the_two_camera_calibration) {
    const auto cal = reference();
    if (cal.cameras.size() < 4) {
        return; // the shipped reference has four; nothing to compare otherwise
    }
    const std::string dir = tmpdir("export4");
    CHECK(cam::create_dir_all(dir + "/2").has_value() && cam::create_dir_all(dir + "/4").has_value());
    const std::string base = dir + "/base.json";
    CHECK(cam::write_file(base, builtin_base()).has_value());
    cam::slam::SlamSetup s4;
    s4.cams = {0, 1, 2, 3};
    const auto e2 = export_files(dir + "/2", cal, cam::slam::SlamSetup{}, cam::slam::ImuNoise{}, base, false);
    const auto e4 = export_files(dir + "/4", cal, s4, cam::slam::ImuNoise{}, base, false);
    CHECK(e2.has_value() && e4.has_value());
    const auto c2 = cam::json::parse(*cam::read_to_string(e2->calib));
    const auto c4 = cam::json::parse(*cam::read_to_string(e4->calib));
    for (const char* k : {"T_imu_cam", "intrinsics", "resolution"}) {
        const auto* a2 = c2->get("value0")->get(k);
        const auto* a4 = c4->get("value0")->get(k);
        CHECK(a2->arr.size() == 2 && a4->arr.size() == 4);
        std::string s2, s4x;
        to_json(a2->arr[0], 0, s2);
        to_json(a2->arr[1], 0, s2);
        to_json(a4->arr[0], 0, s4x);
        to_json(a4->arr[1], 0, s4x);
        CHECK(s2 == s4x);
    }
    CHECK(*cam::read_to_string(e2->config) == *cam::read_to_string(e4->config));
    [[maybe_unused]] const int rc_ = std::system(("rm -rf '" + dir + "'").c_str());
}

TEST(the_built_in_base_is_basalts_euroc_config_with_our_override) {
    const auto out = vio_config(builtin_base());
    CHECK(out.has_value());
    const auto v = cam::json::parse(*out);
    const auto* m = v->get("value0");
    CHECK(m->get("config.optical_flow_detection_grid_size")->as_f64() == 40.0);
    CHECK(*m->get("config.optical_flow_type")->as_str() == "frame_to_frame");
}

TEST(a_clean_install_exports_6dof_with_the_shipped_reference_calibration) {
    const auto cal = reference();
    const std::string dir = tmpdir("export-ref");
    CHECK(cam::create_dir_all(dir).has_value());
    const std::string base = dir + "/base.json";
    CHECK(cam::write_file(base, builtin_base()).has_value());
    const auto e = export_files(dir, cal, cam::slam::SlamSetup{}, cam::slam::ImuNoise{}, base, false);
    CHECK(e.has_value() && cam::is_file(e->calib) && cam::is_file(e->config) && cam::is_file(e->toml));
    [[maybe_unused]] const int rc_ = std::system(("rm -rf '" + dir + "'").c_str());
}

TEST(finds_still_window_and_densities) {
    std::uint64_t seed = 7;
    std::vector<ImuRow> v;
    for (std::int64_t i = 0; i < 10'000; i++) {
        const bool moving = i < 4000; // first 4 s: head motion
        const double amp = moving ? 0.5 : 0.002;
        ImuRow r;
        r.ts = i * 1'000'000;
        r.gyro = {amp * noise(seed), amp * noise(seed), amp * noise(seed)};
        r.accel = {0.02 * noise(seed), 9.81 + 0.02 * noise(seed), 0.02 * noise(seed)};
        v.push_back(r);
    }
    const auto e = estimate(v, 2.0, 1.0);
    CHECK(e.has_value());
    CHECK(e->window.first >= 4'000'000'000);
    for (int k = 0; k < 3; k++) {
        CHECK(std::fabs(e->gyro_std[k] - 0.002) < 0.0003);
        const double dens = 0.002 / std::sqrt(1000.0);
        CHECK(std::fabs(e->noise.gyro_noise_std[k] - dens) < dens * 0.2);
    }
    CHECK(std::fabs(e->accel_mean[1] - 9.81) < 0.01);
    CHECK(e->noise.rate_hz == 1000.0);
    // All motion: refused.
    const std::vector<ImuRow> moving(v.begin(), v.begin() + 4000);
    CHECK(!estimate(moving, 2.0, 1.0).has_value());
    CHECK(!still_gyro_bias(moving, 1.0).has_value());
    // Still part found after the motion; mean = bias (zero here) within the noise.
    const auto b = still_gyro_bias(v, 1.0);
    CHECK(b.has_value());
    CHECK(b->second >= 4'000'000'000);
    for (const double x : b->first) {
        CHECK(std::fabs(x) < 0.0005);
    }
}

// euroc.rs good_dataset_passes, the load part (the check itself is the app's core/rec_check).
TEST(good_dataset_loads) {
    const std::string dir = tmpdir("euroc-ok");
    const std::string root = dir + "/euroc_20261014120000";
    CHECK(cam::create_dir_all(root + "/mav0/imu0").has_value());
    std::string imu = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y,w_RS_S_z,a_RS_S_x [m s^-2],a_RS_S_y,a_RS_S_z\n";
    for (std::int64_t i = 0; i < 2000; i++) {
        imu += std::format("{},0.001,0,0,0,9.81,0\n", 1'000'000'000 + i * 1'000'000);
    }
    CHECK(cam::write_file(root + "/mav0/imu0/data.csv", imu).has_value());
    for (int c = 0; c < 2; c++) {
        CHECK(cam::create_dir_all(std::format("{}/mav0/cam{}", root, c)).has_value());
        std::string t = "#timestamp [ns],filename\n";
        for (std::int64_t i = 0; i < 40; i++) {
            const std::int64_t ts = 1'100'000'000 + i * 40'000'000;
            t += std::format("{},{}.png\n", ts, ts);
        }
        CHECK(cam::write_file(std::format("{}/mav0/cam{}/data.csv", root, c), t).has_value());
    }
    const auto ds = Dataset::load(dir);
    CHECK(ds.has_value());
    CHECK(ds->root == root);
    CHECK(ds->imu.size() == 2000);
    CHECK(ds->cams.size() == 2 && ds->cams[1].size() == 40);
    CHECK(!Dataset::load(dir + "/nothing").has_value());
    [[maybe_unused]] const int rc_ = std::system(("rm -rf '" + dir + "'").c_str());
}

TEST(paths_toml_values) {
    CHECK(paths_value("src_dir = \"/a/b\"\n", "src_dir") == std::optional<std::string>("/a/b"));
    CHECK(paths_value("# x\nsrc_dir = '~/s' # c\n", "src_dir") == std::optional<std::string>("~/s"));
    CHECK(paths_value("src_dir = \"\"\nsrc_dir = x\n", "src_dir") == std::optional<std::string>("x"));
    CHECK(!paths_value("lib_dir = x\n", "src_dir"));
    CHECK(legacy_gyro_bias_path("/h", [](std::string_view) { return std::optional<std::string>{}; }) ==
          "/h/.cache/pimax/gyro-bias");
    CHECK(cam::slam::parse_gyro_bias_cache(reference_gyro_bias()).has_value());
}

PIBRIDGE_TEST_MAIN()
