// SPDX-License-Identifier: MIT
// PiBridge's additions to the head pose output layer (not in the Rust code, docs/research/head-tracking-improvements.md):
// the parity switch PIBRIDGE_HEAD_PARITY, the seeded gyro bias, the accelerometer bias, the drift hold default, and the
// SLAM_CONFIG calibration helpers.
#include "pibridge/cam/headpose.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/slam.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

using namespace pibridge::cam;
using namespace pibridge::cam::headpose;
using slam::QUAT_ID;

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
    const std::string d = std::string(t != nullptr && *t != 0 ? t : "/tmp") + "/pibridge-headpibridge-" +
                          std::to_string(::getpid()) + "-" + name;
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    g_scratch.dirs.push_back(d);
    return d;
}

void put(const std::string& path, const std::string& text) {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream(path) << text;
}

// A clean environment for from_env: no 6DoF switch, a scratch HOME / XDG_CACHE_HOME, no SLAM_CONFIG and no factory
// calibration from the user's home.
void clean_env(const std::string& home) {
    for (const char* k : {"PIBRIDGE_HEAD_PARITY", "PIMAX_SLAM_SEED_BIAS", "PIMAX_SLAM_ACCEL_BIAS", "PIMAX_SLAM_DRIFT_HOLD",
                          "SLAM_CONFIG", "PIMAX_CAM_CALIB", "PIMAX_CALIB", "PIBRIDGE_CAM_CALIB", "PIBRIDGE_CALIB",
                          "XDG_CACHE_HOME", "PIMAX_REPO"}) {
        ::unsetenv(k);
    }
    ::setenv("HOME", home.c_str(), 1);
    ::setenv("PIMAX_NO_REFERENCE", "1", 1);
}

constexpr const char* CALIB_JSON =
    R"({"value0": {"calib_accel_bias": [-0.05, 0.34, -0.02, 0, 0, 0, 0, 0, 0], "calib_gyro_bias": [0.001, -0.002, 0.0235, 0, 0, 0, 0, 0, 0, 0, 0, 0]}})";

} // namespace

TEST(parity_switch_values) {
    ::unsetenv("PIBRIDGE_HEAD_PARITY");
    CHECK(!parity_mode());
    for (const char* on : {"1", " yes ", "true", "2"}) {
        ::setenv("PIBRIDGE_HEAD_PARITY", on, 1);
        CHECK(parity_mode());
    }
    for (const char* off : {"0", "off", "no", " ", ""}) {
        ::setenv("PIBRIDGE_HEAD_PARITY", off, 1);
        CHECK(!parity_mode());
    }
    ::unsetenv("PIBRIDGE_HEAD_PARITY");
}

TEST(parity_from_env_is_the_rust_default) {
    const std::string h = scratch("parity");
    clean_env(h);
    put(h + "/.cache/omarchyvr/gyro-bias", "0.065 0.255 -1.341\n");
    ::setenv("PIBRIDGE_HEAD_PARITY", "1", 1);
    const HeadParams p = HeadParams::from_env();
    CHECK(p == HeadParams());
    CHECK(!p.seed_gyro_bias);
    CHECK(p.accel_bias == (Vec3{0.0, 0.0, 0.0}));
    CHECK(p.drift.on);
    CHECK(p.describe().find("pibridge:") == std::string::npos);
    // an explicit switch still works in parity mode
    ::setenv("PIMAX_SLAM_SEED_BIAS", "1", 1);
    const HeadParams q = HeadParams::from_env();
    CHECK(q.seed_gyro_bias.has_value());
    clean_env(h);
}

TEST(pibridge_defaults_seed_accel_and_drift) {
    const std::string h = scratch("pibridge");
    clean_env(h);
    // the driver's cache: raw axes deg/s (x and z negated into the body frame)
    put(h + "/.cache/omarchyvr/gyro-bias", "0.065 0.255 -1.341\n");
    put(h + "/slam/calib.json", CALIB_JSON);
    put(h + "/slam/pimax-basalt.toml", "show-gui=0\ncam-calib=\"calib.json\"\nconfig-path=\"x.json\"\n");
    ::setenv("SLAM_CONFIG", (h + "/slam/pimax-basalt.toml").c_str(), 1);
    const HeadParams p = HeadParams::from_env();
    CHECK(p.seed_gyro_bias.has_value());
    const double d2r = 3.14159265358979323846 / 180.0;
    CHECK(std::fabs((*p.seed_gyro_bias)[0] - (-0.065 * d2r)) < 1e-12);
    CHECK(std::fabs((*p.seed_gyro_bias)[1] - (0.255 * d2r)) < 1e-12);
    CHECK(std::fabs((*p.seed_gyro_bias)[2] - (1.341 * d2r)) < 1e-12);
    CHECK(p.accel_bias == (Vec3{-0.05, 0.34, -0.02}));
    CHECK(!p.drift.on);
    CHECK(p.describe().find("PIMAX_SLAM_DRIFT_HOLD drift hold off") != std::string::npos);
    CHECK(p.describe().find("PIMAX_HEAD_TILT") != std::string::npos);
    CHECK(p.describe().find("pibridge: gyro bias seeded from the driver cache, accel bias from the SLAM_CONFIG calibration") !=
          std::string::npos);
    // switches
    ::setenv("PIMAX_SLAM_DRIFT_HOLD", "1", 1);
    ::setenv("PIMAX_SLAM_SEED_BIAS", "0", 1);
    ::setenv("PIMAX_SLAM_ACCEL_BIAS", "0", 1);
    const HeadParams q = HeadParams::from_env();
    CHECK(q.drift.on);
    CHECK(!q.seed_gyro_bias);
    CHECK(q.accel_bias == (Vec3{0.0, 0.0, 0.0}));
    // without a cache and without any calibration: nothing to seed, said so
    clean_env(h);
    std::filesystem::remove_all(h + "/.cache");
    const HeadParams r = HeadParams::from_env();
    CHECK(!r.seed_gyro_bias);
    CHECK(r.accel_bias == (Vec3{0.0, 0.0, 0.0}));
    CHECK(r.bias_source == "no gyro bias cache, no accel bias");
    clean_env(h);
    ::unsetenv("PIMAX_NO_REFERENCE");
}

TEST(slam_config_calib_path_forms) {
    const std::string h = scratch("toml");
    clean_env(h);
    CHECK(!slam::slam_config_calib_path());
    put(h + "/a.toml", "# c\n  cam-calib = '/abs/c.json'  \n");
    ::setenv("SLAM_CONFIG", (h + "/a.toml").c_str(), 1);
    CHECK(slam::slam_config_calib_path() == std::optional<std::string>("/abs/c.json"));
    put(h + "/b.toml", "cam-calibx=1\ncam-calib=rel/c.json\n");
    ::setenv("SLAM_CONFIG", (h + "/b.toml").c_str(), 1);
    CHECK(slam::slam_config_calib_path() == std::optional<std::string>(h + "/rel/c.json"));
    put(h + "/c.toml", "config-path=\"x\"\n");
    ::setenv("SLAM_CONFIG", (h + "/c.toml").c_str(), 1);
    CHECK(!slam::slam_config_calib_path());
    ::setenv("SLAM_CONFIG", (h + "/missing.toml").c_str(), 1);
    CHECK(!slam::slam_config_calib_path());
    ::unsetenv("SLAM_CONFIG");
    ::unsetenv("PIMAX_NO_REFERENCE");
}

TEST(calib_json_biases_checks) {
    const auto b = slam::calib_json_biases(CALIB_JSON);
    CHECK(b.accel == (std::optional<Vec3>(Vec3{-0.05, 0.34, -0.02})));
    CHECK(b.gyro == (std::optional<Vec3>(Vec3{0.001, -0.002, 0.0235})));
    CHECK(!slam::calib_json_biases("{}").accel);
    CHECK(!slam::calib_json_biases("not json").gyro);
    const auto big = slam::calib_json_biases(R"({"value0": {"calib_accel_bias": [2, 0, 0], "calib_gyro_bias": [0, 0.1, 0]}})");
    CHECK(!big.accel); // above 1 m/s^2
    CHECK(!big.gyro);  // above 5 deg/s
    CHECK(!slam::calib_json_biases(R"({"value0": {"calib_accel_bias": [0, 0]}})").accel);
}

TEST(seeded_bias_is_the_start_value) {
    HeadParams p;
    p.seed_gyro_bias = Vec3{0.001, 0.002, 0.0234};
    HeadTracker t(slam::SupervisorParams{}, p, {0.0, -0.08, 0.04});
    CHECK(t.pred.gyro_bias == *p.seed_gyro_bias);
    HeadTracker u(slam::SupervisorParams{}, HeadParams{}, {0.0, -0.08, 0.04});
    CHECK(u.pred.gyro_bias == (Vec3{0.0, 0.0, 0.0}));
}

namespace {

// A head held perfectly in place (identity attitude) with a biased accelerometer, tracker poses at 25 Hz handed over
// 25 ms late, queries at 90 Hz for 20 ms ahead. Returns the largest distance after the first 2 s between the output and
// where the tracker puts the head.
double held_head_error(bool remove_bias) {
    const Vec3 bias{0.05, 0.34, -0.02};
    const Vec3 head{0.0, -0.08, 0.04};
    slam::SupervisorParams sp;
    sp.acquire_poses = 3;
    HeadParams p;
    p.still.hold = false; // a noiseless IMU would count as lying on a desk
    if (remove_bias) {
        p.accel_bias = bias;
    }
    HeadTracker t(sp, p, head);
    double worst = 0.0;
    std::int64_t next_vio = 0, next_q = 0;
    for (std::int64_t ms = 0; ms < 8000; ms++) {
        const std::int64_t now = ms * 1'000'000;
        t.push_imu(ImuSample{now, geom::add(Vec3{0.0, GRAVITY, 0.0}, bias), {0.0, 0.0, 0.0}});
        if (now >= next_vio + 25'000'000) {
            VioSample v;
            v.ts_ns = next_vio;
            v.q = QUAT_ID;
            v.q3dof = QUAT_ID;
            t.update(now, v);
            next_vio += 40'000'000;
        }
        if (now >= next_q) {
            next_q += 11'111'111;
            t.update(now, std::nullopt);
            Rel r3;
            r3.q = QUAT_ID;
            r3.flags = flags::ORIENTATION_VALID;
            const Rel o = t.head(now, now + 20'000'000, r3);
            if (now > 2'000'000'000 && t.sup.state() == slam::TrackState::Tracking) {
                // where the tracker says the head is (current alignment, no correction)
                const Vec3 truth = t.head_world({0.0, 0.0, 0.0}, QUAT_ID);
                worst = std::max(worst, geom::norm(geom::sub(o.pos, truth)));
            }
        }
    }
    return worst;
}

} // namespace

TEST(accel_bias_removes_the_steady_lag) {
    const double raw = held_head_error(false);
    const double fixed = held_head_error(true);
    std::printf("held head, biased accelerometer: worst output error %.2f mm raw, %.4f mm with the bias removed\n",
                raw * 1000.0, fixed * 1000.0);
    CHECK(raw > 0.002);
    CHECK(fixed < 0.0001);
}

TEST(rotation_prediction_with_angular_acceleration) {
    // The head turns about y with a constant angular acceleration; the pose is asked 40 ms past the newest sample.
    const double alpha = 10.0; // rad/s^2
    auto err = [&](double k) {
        PredictorParams pp;
        pp.ang_accel_k = k;
        pp.bias_tau_s = 0.0;
        Predictor pr(pp);
        std::int64_t te = 0;
        for (std::int64_t ms = 0; ms <= 300; ms++) {
            te = ms * 1'000'000;
            pr.push_imu(ImuSample{te, {0.0, GRAVITY, 0.0}, {0.0, alpha * static_cast<double>(ms) / 1000.0, 0.0}});
        }
        BodyState b;
        b.ts_ns = te;
        b.q = QUAT_ID;
        const double h = 0.040;
        const auto [st, w] = pr.propagate(b, te + 40'000'000);
        (void)w;
        const double w_te = alpha * 0.3;
        const double want = w_te * h + 0.5 * alpha * h * h;
        return std::fabs(slam::qangle_deg(st.q) * 3.14159265358979323846 / 180.0 - want);
    };
    const double e0 = err(0.0), e1 = err(1.0), e75 = err(0.75);
    std::printf("rotation 40 ms ahead under 10 rad/s^2: error %.5f rad (rate held), %.5f (k 0.75), %.5f (k 1)\n", e0,
                e75, e1);
    CHECK(e0 > 0.0075 && e0 < 0.0085); // 0.5 alpha h^2 = 0.008 rad
    CHECK(e1 < 0.1 * e0);
    CHECK(e75 < 0.35 * e0);
}

TEST(prediction_and_backoff_switches) {
    const std::string h = scratch("switches");
    clean_env(h);
    HeadParams p = HeadParams::from_env();
    CHECK(p.predict_accel == 0.75);
    CHECK(p.still.backoff_max == 0); // measured worse on the desk take (closed loop): off by default
    ::setenv("PIMAX_SLAM_PREDICT_ACCEL", "0", 1);
    ::setenv("PIMAX_SLAM_STILL_BACKOFF", "4", 1);
    p = HeadParams::from_env();
    CHECK(p.predict_accel == 0.0);
    CHECK(p.still.backoff_max == 4);
    CHECK(p.describe().find("still re-zero back-off up to 16 s") != std::string::npos);
    ::unsetenv("PIMAX_SLAM_PREDICT_ACCEL");
    ::unsetenv("PIMAX_SLAM_STILL_BACKOFF");
    // the quiet-only drift hold (experiment)
    CHECK(p.drift.quiet_gyro_std == 0.0);
    ::setenv("PIMAX_SLAM_DRIFT_HOLD", "2", 1);
    p = HeadParams::from_env();
    CHECK(p.drift.on);
    CHECK(p.drift.quiet_gyro_std > 0.0 && p.drift.quiet_accel_std == 0.09);
    ::setenv("PIMAX_SLAM_DRIFT_HOLD", "1", 1);
    p = HeadParams::from_env();
    CHECK(p.drift.on && p.drift.quiet_gyro_std == 0.0);
    ::unsetenv("PIMAX_SLAM_DRIFT_HOLD");
    ::setenv("PIBRIDGE_HEAD_PARITY", "1", 1);
    p = HeadParams::from_env();
    CHECK(p.predict_accel == 0.0);
    CHECK(p.still.backoff_max == 0);
    CHECK(p.pibridge_describe().empty());
    clean_env(h);
    ::unsetenv("PIMAX_NO_REFERENCE");
}

TEST(still_backoff_spaces_the_rezeros) {
    // A tracker that drifts 5 cm/s while the IMU says the headset lies still: count the re-zeros in 20 s.
    auto count = [](std::uint32_t backoff) {
        slam::SupervisorParams sp;
        sp.acquire_poses = 3;
        slam::Supervisor s(sp, {0.0, 0.0, 0.0});
        slam::StillParams st;
        st.backoff_max = backoff;
        s.set_still_params(st);
        std::uint64_t n = 0;
        double x = 0.0;
        for (std::uint64_t i = 0; i < 500; i++) { // 25 Hz, 20 s
            const std::uint64_t ts = i * 40'000'000ULL;
            s.set_still(i > 20);
            x += 0.002;
            const auto o = s.update(ts, slam::TrackSample{ts, {x, 0.0, 0.0}, QUAT_ID, QUAT_ID});
            if ((o.events & slam::events::REZEROED) != 0) {
                n++;
                x = 0.0; // the fresh tracker starts at its own origin
            }
        }
        return n;
    };
    const std::uint64_t every = count(0), back = count(4);
    std::printf("re-zeros in 20 s of a drifting still tracker: %llu without back-off, %llu with\n",
                static_cast<unsigned long long>(every), static_cast<unsigned long long>(back));
    CHECK(every >= 15);
    CHECK(back <= 6);
}

PIBRIDGE_TEST_MAIN()
