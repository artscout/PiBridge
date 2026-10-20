// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-slam/src/main.rs (MIT OR Apache-2.0): the `export` and `ae` subcommands.
//
// pibridge-slam: offline tools for 6DoF head tracking (Monado + Basalt). Today `export` and `ae` (the other
// omarchyvr-slam subcommands are analysis tools, plan milestone M10; `check` is the app's core/rec_check).
#include "export.h"
#include "headreplay.h"

#include "pibridge/cam/ae.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <format>
#include <limits>
#include <string>
#include <vector>

namespace {

namespace rf = pibridge::cam::rfmt;
namespace slam = pibridge::cam::slam;
using namespace pibridge::slamtool;

constexpr const char* USAGE =
    "pibridge-slam: offline tools for Pimax 6DoF head tracking (Monado + Basalt).\n"
    "\n"
    "pibridge-slam export     [--out DIR] [--calib XML] [--cams 0,1] [--imu F] [--base CONFIG.json] [--gui] "
    "[--cam-offset-ns N]\n"
    "                      [--gyro-bias X,Y,Z|none] [--bias-from DATASET] [--accel-bias factory|none|X,Y,Z]\n"
    "                      Basalt calibration (kb4), VIO config and SLAM_CONFIG toml (default DIR "
    "~/.config/omarchyvr/slam).\n"
    "                      Gyro bias seed (calib_gyro_bias, body frame): --gyro-bias in deg/s, else the first still\n"
    "                      second of DATASET, else the driver's cache (~/.cache/omarchyvr/gyro-bias), else none.\n"
    "                      Accel bias (calib_accel_bias, body frame, m/s^2): default the factory Stateinit aBias.\n"
    "pibridge-slam ae         REC.pxcam [--frames N] [--model crystal-super]  auto-exposure dry run on a recording "
    "(what it would send)\n"
    "pibridge-slam headreplay DATASET TRACKING.csv [--steps steps.json] [--calib basalt-calib.json] [--latency-ms 25]\n"
    "                      [--predict-ms 20] [--hz 90] [--central-vel] [--head X,Y,Z] [--csv OUT]\n"
    "                      the head pose output layer replayed on a recording + a tracker run, with its numbers\n"
    "                      (prediction error, jitter, mark error, creep, tilt); PIMAX_SLAM_* / PIBRIDGE_HEAD_* apply\n"
    "DATASET: a EuRoC root (holding mav0) or the driver's recording directory (newest euroc_* inside).\n"
    "The same files as omarchyvr-slam export (docs/pibridge-slam.md).";

using R = std::expected<void, std::string>;

struct Args {
    std::vector<std::string> v;

    bool flag(const std::string& n) {
        for (std::size_t i = 0; i < v.size(); i++) {
            if (v[i] == n) {
                v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
                return true;
            }
        }
        return false;
    }
    std::expected<std::optional<std::string>, std::string> opt(const std::string& n) {
        for (std::size_t i = 0; i < v.size(); i++) {
            if (v[i] == n) {
                if (i + 1 < v.size()) {
                    std::string val = v[i + 1];
                    v.erase(v.begin() + static_cast<std::ptrdiff_t>(i), v.begin() + static_cast<std::ptrdiff_t>(i) + 2);
                    return std::optional<std::string>(std::move(val));
                }
                return std::unexpected(n + " needs a value");
            }
        }
        return std::optional<std::string>{};
    }
    R done() const {
        if (!v.empty()) {
            return std::unexpected(std::format("unexpected argument {}", rf::debug_str(v.front())));
        }
        return {};
    }
};

std::string home() {
    return pibridge::cam::env_os("HOME").value_or(".");
}

// Rust `i64::from_str`.
bool parse_i64(std::string_view s, std::int64_t& out) {
    bool neg = false;
    if (!s.empty() && (s[0] == '+' || s[0] == '-')) {
        neg = s[0] == '-';
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return false;
    }
    std::int64_t x = 0;
    for (const char c : s) {
        if (c < '0' || c > '9') {
            return false;
        }
        const int d = c - '0';
        if (__builtin_mul_overflow(x, 10, &x) ||
            (neg ? __builtin_sub_overflow(x, d, &x) : __builtin_add_overflow(x, d, &x))) {
            return false;
        }
    }
    out = x;
    return true;
}

// v.split(',').map(|w| w.trim().parse::<f64>()).collect::<Result<_, _>>()
std::optional<std::vector<double>> parse_list(std::string_view v) {
    std::vector<double> x;
    std::size_t s = 0;
    for (;;) {
        const std::size_t c = v.find(',', s);
        double d = 0;
        if (!rf::parse_f64(rf::trim(v.substr(s, c == std::string_view::npos ? std::string_view::npos : c - s)), d)) {
            return std::nullopt;
        }
        x.push_back(d);
        if (c == std::string_view::npos) {
            break;
        }
        s = c + 1;
    }
    return x;
}

std::expected<pibridge::cam::DeviceCalib, std::string> load_calib(const std::optional<std::string>& p) {
    auto r = pibridge::cam::DeviceCalib::load_default(p);
    if (!r) {
        return std::unexpected(r.error().text());
    }
    std::fprintf(stderr, "calibration: %s (%s)\n", rf::utf8_lossy(r->second).c_str(), r->first.uid.c_str());
    return std::move(r->first);
}

// Gyro bias seed for Basalt (body frame, rad/s) and where it came from.
std::expected<std::pair<Vec3, std::string>, std::string> gyro_bias_seed(const std::optional<std::string>& arg,
                                                                        const std::optional<std::string>& from) {
    if (arg && *arg == "none") {
        return std::pair{Vec3{0.0, 0.0, 0.0}, std::string("none (--gyro-bias none)")};
    }
    if (arg) {
        const auto x = parse_list(*arg);
        if (!x || x->size() != 3) {
            return std::unexpected(std::format("--gyro-bias {}: want X,Y,Z in deg/s", rf::debug_str(*arg)));
        }
        return std::pair{Vec3{rf::to_radians((*x)[0]), rf::to_radians((*x)[1]), rf::to_radians((*x)[2])},
                         std::string("--gyro-bias")};
    }
    if (from) {
        auto ds = Dataset::load(*from);
        if (!ds) {
            return std::unexpected(ds.error());
        }
        if (const auto b = still_gyro_bias(ds->imu, 1.0)) {
            const double at = static_cast<double>(b->second - ds->imu[0].ts) / 1e9;
            return std::pair{b->first, std::format("first still second of {} (at {} s)", rf::utf8_lossy(ds->root),
                                                   rf::fixed(at, 1))};
        }
        std::fprintf(stderr, "%s: the headset never lay still; using the driver's cached bias\n", from->c_str());
    }
    if (const auto b = slam::gyro_bias_cache()) {
        return std::pair{*b, std::format("driver cache {}", rf::utf8_lossy(slam::gyro_bias_cache_path().value_or("")))};
    }
    // nothing in the new cache: the cache of the names before the rename (the driver reads it too), else the value
    // the driver starts with on a clean install (the launcher writes it right after this export; a zero seed lets
    // the tracker drift)
    const std::string legacy = legacy_gyro_bias_path(home(), [](std::string_view k) {
        return pibridge::cam::env_utf8(std::string(k).c_str());
    });
    if (auto t = pibridge::cam::read_to_string(legacy)) {
        if (const auto b = slam::parse_gyro_bias_cache(*t)) {
            return std::pair{*b, std::format("driver cache {}", rf::utf8_lossy(legacy))};
        }
    }
    if (const auto b = slam::parse_gyro_bias_cache(reference_gyro_bias())) {
        return std::pair{*b, std::string("the Dream Air default (no measured bias on this PC yet)")};
    }
    return std::pair{Vec3{0.0, 0.0, 0.0}, std::string("none (no driver cache yet)")};
}

std::string pad_left(const std::string& s, std::size_t width) {
    // Rust {:<width}: pads by chars (the camera names are ASCII)
    std::size_t chars = 0;
    for (const unsigned char c : s) {
        chars += (c & 0xc0) != 0x80 ? 1 : 0;
    }
    return chars >= width ? s : s + std::string(width - chars, ' ');
}

R cmd_export(Args a) {
    auto out_o = a.opt("--out");
    if (!out_o) {
        return std::unexpected(out_o.error());
    }
    const std::string out = out_o->value_or(pibridge::cam::path_join(home(), ".config/omarchyvr/slam"));
    auto calib_o = a.opt("--calib");
    if (!calib_o) {
        return std::unexpected(calib_o.error());
    }
    auto cal = load_calib(*calib_o);
    if (!cal) {
        return std::unexpected(cal.error());
    }
    auto cams_o = a.opt("--cams");
    if (!cams_o) {
        return std::unexpected(cams_o.error());
    }
    auto cams = slam::parse_cams(cams_o->value_or("0,1"), cal->cameras.size());
    if (!cams) {
        return std::unexpected(cams.error());
    }
    auto imu_p = a.opt("--imu");
    if (!imu_p) {
        return std::unexpected(imu_p.error());
    }
    auto base = a.opt("--base");
    if (!base) {
        return std::unexpected(base.error());
    }
    const bool gui = a.flag("--gui");
    std::int64_t offset = 0;
    {
        auto v = a.opt("--cam-offset-ns");
        if (!v) {
            return std::unexpected(v.error());
        }
        if (*v && !parse_i64(**v, offset)) {
            return std::unexpected(std::format("--cam-offset-ns: bad value {}", rf::debug_str(**v)));
        }
    }
    double cam_scale = 1.0;
    {
        // PiBridge: scale of the camera positions (baseline + lever arm); docs/pibridge-slam.md.
        auto v = a.opt("--cam-scale");
        if (!v) {
            return std::unexpected(v.error());
        }
        if (*v && (!rf::parse_f64(rf::trim(**v), cam_scale) || !(cam_scale > 0.5 && cam_scale < 2.0))) {
            return std::unexpected(std::format("--cam-scale: bad value {} (0.5..2)", rf::debug_str(**v)));
        }
    }
    auto bias_arg = a.opt("--gyro-bias");
    if (!bias_arg) {
        return std::unexpected(bias_arg.error());
    }
    auto bias_from = a.opt("--bias-from");
    if (!bias_from) {
        return std::unexpected(bias_from.error());
    }
    auto accel_arg = a.opt("--accel-bias");
    if (!accel_arg) {
        return std::unexpected(accel_arg.error());
    }
    if (auto d = a.done(); !d) {
        return d;
    }
    auto loaded = slam::ImuNoise::load(*imu_p);
    if (!loaded) {
        return std::unexpected(loaded.error());
    }
    const slam::ImuNoise imu = loaded->value_or(slam::ImuNoise{});
    auto seed = gyro_bias_seed(*bias_arg, *bias_from);
    if (!seed) {
        return std::unexpected(seed.error());
    }
    const auto [gyro_bias, bias_src] = *seed;
    Vec3 accel_bias{0.0, 0.0, 0.0};
    const std::string av = accel_arg->value_or("factory");
    if (av == "factory") {
        accel_bias = slam::factory_accel_bias(*cal).value_or(Vec3{0.0, 0.0, 0.0});
    } else if (av != "none") {
        const auto x = parse_list(av);
        if (!x || x->size() != 3) {
            return std::unexpected(
                std::format("--accel-bias {}: want factory, none or X,Y,Z in m/s^2", rf::debug_str(av)));
        }
        accel_bias = {(*x)[0], (*x)[1], (*x)[2]};
    }
    slam::SlamSetup setup;
    setup.cams = std::move(*cams);
    setup.cam_time_offset_ns = offset;
    setup.cam_translation_scale = cam_scale;
    setup.gyro_bias = gyro_bias;
    setup.accel_bias = accel_bias;
    auto e = export_files(out, *cal, setup, imu, *base, gui);
    if (!e) {
        return std::unexpected(e.error());
    }
    std::string o;
    o += std::format("calibration  {}\n", rf::utf8_lossy(e->calib));
    o += std::format("vio config   {} (base {})\n", rf::utf8_lossy(e->config), rf::utf8_lossy(e->base));
    o += std::format("SLAM_CONFIG  {}\n", rf::utf8_lossy(e->toml));
    o += std::format("IMU noise    {}\n", imu.source);
    o += std::format("gyro bias    ({} {} {}) deg/s body, {}\n", rf::fixed(rf::to_degrees(gyro_bias[0]), 3, true),
                     rf::fixed(rf::to_degrees(gyro_bias[1]), 3, true), rf::fixed(rf::to_degrees(gyro_bias[2]), 3, true),
                     bias_src);
    o += std::format("accel bias   ({} {} {}) m/s^2 body\n", rf::fixed(accel_bias[0], 4, true),
                     rf::fixed(accel_bias[1], 4, true), rf::fixed(accel_bias[2], 4, true));
    for (const std::size_t i : setup.cams) {
        const auto p = slam::t_imu_cam(*cal, i);
        const auto q = slam::quat_of(p->r);
        auto f = [](double x) { return rf::fixed(x, 4, true); };
        o += std::format("  cam {} {} T_imu_cam p=({} {} {}) q=({} {} {} {})\n", i, pad_left(cal->cameras[i].name, 15),
                         f(p->t[0]), f(p->t[1]), f(p->t[2]), f(q[0]), f(q[1]), f(q[2]), f(q[3]));
    }
    std::fwrite(o.data(), 1, o.size(), stdout);
    return {};
}

// omarchyvr-slam cmd_ae: the auto-exposure controller open loop over a recording (each tracking frame's first step
// from its own metadata; a recording cannot respond), one line per device second and the medians.
R cmd_ae(Args a) {
    namespace ae = pibridge::cam::ae;
    if (a.v.empty() || a.v[0].starts_with("--")) {
        return std::unexpected(std::string("missing REC.pxcam"));
    }
    const std::string src = a.v.front();
    a.v.erase(a.v.begin());
    std::uint64_t max = 0;
    {
        auto v = a.opt("--frames");
        if (!v) {
            return std::unexpected(v.error());
        }
        if (*v && !rf::parse_u64(**v, max)) {
            return std::unexpected(std::format("--frames: bad value {}", rf::debug_str(**v)));
        }
    }
    ae::AeModel model = ae::AeModel::DreamAir;
    {
        auto v = a.opt("--model");
        if (!v) {
            return std::unexpected(v.error());
        }
        if (*v) {
            const auto m = ae::model_parse(**v);
            if (!m) {
                return std::unexpected(std::format("--model {}: dream-air or crystal-super", **v));
            }
            model = *m;
        }
    }
    if (auto d = a.done(); !d) {
        return d;
    }
    auto opened = pibridge::cam::Camera::open(src, 0, pibridge::cam::ReplayOptions{});
    if (!opened) {
        return std::unexpected(std::format("open {}: {}", src, opened.error().text()));
    }
    pibridge::cam::Camera& cam = **opened;
    const ae::AutoExposure ctl(ae::AeParams::for_model(model));
    const ae::Mask mask;
    std::uint64_t n = 0;
    std::vector<double> means, sats, req_e, req_g;
    std::uint64_t last_print = 0;
    std::string o = "    t     frame exp/gain   lower mean sat   upper mean sat  -> would request lower e/g, upper e/g\n";
    std::fwrite(o.data(), 1, o.size(), stdout);
    for (;;) {
        auto r = cam.next_frame(100);
        if (!r) {
            return std::unexpected(r.error().text());
        }
        if (!*r) {
            if (cam.source().finished()) {
                break;
            }
            continue;
        }
        const pibridge::cam::FrameInfo info = **r;
        if (info.kind != 1) {
            continue;
        }
        std::array<pibridge::cam::TileView, 4> tiles{};
        for (std::size_t t = 0; t < 4; t++) {
            auto v = cam.tile_view(t);
            if (!v) {
                return std::unexpected(v.error().text());
            }
            tiles[t] = *v;
        }
        const auto st = ctl.frame_stats(tiles, mask);
        const ae::AeSetting set = ae::AeSetting::from_info(info, 40, 62);
        const auto [e0, g0, err0] = ctl.step_pair(set.exposure[0], set.gain[0], st[0]);
        const auto [e2, g2, err2] = ctl.step_pair_k(1, set.exposure[2], set.gain[2], st[1]);
        (void)err0;
        (void)err2;
        means.push_back(st[0].trimmed_mean);
        sats.push_back(st[0].sat_frac);
        req_e.push_back(static_cast<double>(e0));
        req_g.push_back(static_cast<double>(g0));
        if (info.dev_ts_us >= last_print + 1'000'000) {
            last_print = info.dev_ts_us;
            o = std::format("{} {:>7}/{:<5} {} {} {} {}  -> {}/{}, {}/{}\n",
                            rf::fixed_w(static_cast<double>(info.dev_ts_us) / 1e6, 7, 1), set.exposure[0], set.gain[0],
                            rf::fixed_w(st[0].trimmed_mean, 8, 1), rf::fixed(st[0].sat_frac, 3),
                            rf::fixed_w(st[1].trimmed_mean, 8, 1), rf::fixed(st[1].sat_frac, 3), e0, g0, e2, g2);
            std::fwrite(o.data(), 1, o.size(), stdout);
        }
        n += 1;
        if (max > 0 && n >= max) {
            break;
        }
    }
    auto med = [](std::vector<double>& v) {
        std::sort(v.begin(), v.end());
        return v.empty() ? std::numeric_limits<double>::quiet_NaN() : v[v.size() / 2];
    };
    o = std::format("{} tracking frames: lower-pair trimmed mean median {}, saturated median {}; first-step request "
                    "median exposure {} gain {} (open loop: a recording cannot respond)\n",
                    n, rf::fixed(med(means), 1), rf::fixed(med(sats), 3), rf::fixed(med(req_e), 0),
                    rf::fixed(med(req_g), 0));
    std::fwrite(o.data(), 1, o.size(), stdout);
    return {};
}

R cmd_headreplay(Args a) {
    HeadReplayOpts o;
    auto num = [&](const char* n, double& dst) -> R {
        auto v = a.opt(n);
        if (!v) {
            return std::unexpected(v.error());
        }
        if (*v) {
            double x = 0;
            if (!rf::parse_f64(rf::trim(**v), x) || !std::isfinite(x)) {
                return std::unexpected(std::format("{}: not a number {}", n, rf::debug_str(**v)));
            }
            dst = x;
        }
        return {};
    };
    auto str = [&](const char* n, std::string& dst) -> R {
        auto v = a.opt(n);
        if (!v) {
            return std::unexpected(v.error());
        }
        if (*v) {
            dst = **v;
        }
        return {};
    };
    for (auto r : {num("--latency-ms", o.latency_ms), num("--predict-ms", o.predict_ms), num("--hz", o.query_hz),
                   str("--steps", o.steps), str("--calib", o.calib), str("--csv", o.csv)}) {
        if (!r) {
            return r;
        }
    }
    auto hv = a.opt("--head");
    if (!hv) {
        return std::unexpected(hv.error());
    }
    if (*hv) {
        slam::Vec3 h{};
        int n = 0;
        std::vector<std::string> parts;
        {
            std::string cur;
            for (const char ch : **hv) {
                if (ch == ',') {
                    parts.push_back(cur);
                    cur.clear();
                } else {
                    cur += ch;
                }
            }
            parts.push_back(cur);
        }
        for (const auto& part : parts) {
            double x = 0;
            if (n >= 3 || !rf::parse_f64(rf::trim(part), x)) {
                return std::unexpected("--head needs X,Y,Z (m)");
            }
            h[static_cast<std::size_t>(n++)] = x;
        }
        if (n != 3) {
            return std::unexpected("--head needs X,Y,Z (m)");
        }
        o.head = h;
    }
    o.central_vel = a.flag("--central-vel");
    if (a.v.size() != 2) {
        return std::unexpected("needs DATASET and TRACKING.csv");
    }
    o.dataset = a.v[0];
    o.traj = a.v[1];
    if (o.query_hz < 1.0) {
        return std::unexpected("--hz must be at least 1");
    }
    auto rep = head_replay(o);
    if (!rep) {
        return std::unexpected(rep.error());
    }
    std::fwrite(rep->data(), 1, rep->size(), stdout);
    return {};
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> v(argv + 1, argv + argc);
    if (v.empty() || v[0] == "-h" || v[0] == "--help") {
        std::fprintf(stderr, "%s\n", USAGE);
        return 2;
    }
    const std::string cmd = v[0];
    Args a{std::vector<std::string>(v.begin() + 1, v.end())};
    R r;
    try {
        if (cmd == "export") {
            r = cmd_export(std::move(a));
        } else if (cmd == "ae") {
            r = cmd_ae(std::move(a));
        } else if (cmd == "headreplay") {
            r = cmd_headreplay(std::move(a));
        } else {
            r = std::unexpected(std::format("unknown command {} (export, ae)", rf::debug_str(cmd)));
        }
    } catch (const std::exception& ex) {
        r = std::unexpected(std::string(ex.what()));
    }
    if (!r) {
        std::fprintf(stderr, "pibridge-slam %s: %s\n", cmd.c_str(), r.error().c_str());
        return 1;
    }
    return 0;
}
