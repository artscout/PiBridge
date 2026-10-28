// SPDX-License-Identifier: MIT
#include "headtrack.h"

#include "pibridge/cam/ae.h"
#include "pibridge/cam/calib.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/frame.h"
#include "pibridge/hmd/ctrl_serials.h"
#include "pibridge/hmd/haptics.h"
#include "pibridge_pause.h"
#include "pibridge_status.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <atomic>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace pibridge::headtrack {

namespace geom = cam::geom;
namespace slam = cam::slam;
namespace ps = hmd::pose_shm;
using namespace std::chrono_literals;

namespace {

std::int64_t monotonic_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
}

/// The engine's world (z up) to y up: -90 degrees about x (pimax_camera.c SLAM_TO_YUP).
const Quat TO_YUP{-std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};
/// Without an engine pose this long: the 3DoF orientation at the last position (pimax_camera.c PT_STALE_NS).
constexpr std::int64_t STALE_NS = 350'000'000;
constexpr double G = 9.80665;
/// Controller gyro: 8.2 LSB per deg/s (PIMAX_CTRL_GYRO_LSB_DPS, matched against Pimax's engine inputs).
constexpr double CTRL_GYRO_SCALE = M_PI / 180.0 / 8.2;

/// DIAG: distinct bright LED dots in a Y8 LED-exposure tile -- 4-connected blobs of pixels >= thr with at least
/// `min_px` pixels (so one count per diode, not per saturated pixel; single-pixel noise is ignored). The Pimax LED
/// engine's matched-diode count is not exposed, so we count what the cameras see ourselves; `peak_out` = brightest px.
int led_blobs(const std::uint8_t* px, int w, int h, std::uint8_t thr, int min_px, int& peak_out) {
    const std::size_t npx = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    std::vector<std::uint8_t> seen(npx, 0);
    std::vector<int> stack;
    int blobs = 0, peak = 0;
    for (std::size_t i = 0; i < npx; i++) {
        peak = px[i] > peak ? px[i] : peak;
    }
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            if (seen[idx] != 0 || px[idx] < thr) {
                continue;
            }
            int size = 0;
            stack.clear();
            stack.push_back(y * w + x);
            seen[idx] = 1;
            while (!stack.empty()) {
                const int p = stack.back();
                stack.pop_back();
                size++;
                const int py = p / w, pxl = p % w;
                const int nb[4][2] = {{py - 1, pxl}, {py + 1, pxl}, {py, pxl - 1}, {py, pxl + 1}};
                for (const auto& q : nb) {
                    if (q[0] < 0 || q[0] >= h || q[1] < 0 || q[1] >= w) {
                        continue;
                    }
                    const std::size_t qi = static_cast<std::size_t>(q[0]) * static_cast<std::size_t>(w) + static_cast<std::size_t>(q[1]);
                    if (seen[qi] == 0 && px[qi] >= thr) {
                        seen[qi] = 1;
                        stack.push_back(q[0] * w + q[1]);
                    }
                }
            }
            blobs += size >= min_px ? 1 : 0;
        }
    }
    peak_out = peak;
    return blobs;
}
/// A controller out of the cameras' view: held this long (PIMAX_CTRL_TRACK_LOST_S).
constexpr std::int64_t CTRL_HOLD_NS = 4'000'000'000;
/// The aim frame: the LED model frame turned -52 degrees about x (PIMAX_CTRLTRACK_AIM_PITCH_DEG; pibridge-ctrltrack's
/// poses and so the fuser's are in it).
const Quat DEVICE_FROM_AIM{std::sin(-52.0 * M_PI / 360.0), 0.0, 0.0, std::cos(-52.0 * M_PI / 360.0)};
/// PIMAX_CTRL_ANGLE (pimax_ctrl.c): degrees the aim is pitched up about its own x axis, -20..+70, default +30; live
/// changes from $XDG_RUNTIME_DIR/pimax-ctrl-angle (the Tracking page's "Controller angle").
constexpr double CTRL_ANGLE_DEFAULT = 52.0, CTRL_ANGLE_MIN = -20.0, CTRL_ANGLE_MAX = 70.0;

double ctrl_angle_clamp(double deg) {
    return std::isnan(deg) ? CTRL_ANGLE_DEFAULT : std::clamp(deg, CTRL_ANGLE_MIN, CTRL_ANGLE_MAX);
}

Vec3 v3(const float a[3]) { return {a[0], a[1], a[2]}; }
Quat q4(const float a[4]) { return slam::qnorm({a[0], a[1], a[2], a[3]}); }

void put(std::array<float, 3>& out, const Vec3& v) {
    out = {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}
void put(std::array<float, 4>& out, const Quat& q) {
    out = {static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]), static_cast<float>(q[3])};
}

/// pibridge-keepalived's status says it let go of the headset (or there is no keepalived).
bool keepalived_released() {
    std::ifstream f(status::runtime_dir() / "keepalived.json");
    if (!f) {
        return true;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str().find("\"state\": \"paused\"") != std::string::npos;
}

} // namespace

// ------------------------------------------------------------------------------------------------ IMU time

std::int64_t ImuTime::map(std::uint32_t dev_us, std::int64_t now_ns) {
    std::lock_guard g(mu_);
    if (!have_) {
        have_ = true;
        ext_us_ = dev_us;
        last_dev_ = dev_us;
        offset_ns_ = now_ns - static_cast<std::int64_t>(ext_us_ * 1000);
    } else {
        ext_us_ += static_cast<std::uint32_t>(dev_us - last_dev_); // the 2^32 wrap
        last_dev_ = dev_us;
    }
    const std::int64_t cand = now_ns - static_cast<std::int64_t>(ext_us_ * 1000);
    // the least delayed offset; relax slowly to follow the clock drift
    if (cand < offset_ns_) {
        offset_ns_ = cand;
    } else {
        offset_ns_ += (cand - offset_ns_) >> 14;
    }
    std::int64_t t = static_cast<std::int64_t>(ext_us_ * 1000) + offset_ns_;
    if (t <= last_ns_) {
        t = last_ns_ + 1;
    }
    last_ns_ = t;
    ref_.dev_ts_u32 = dev_us;
    ref_.dev_ext_us = ext_us_;
    ref_.offset_ns = offset_ns_;
    ref_.valid = 1;
    return t;
}

cam::ImuClockRef ImuTime::ref() const {
    std::lock_guard g(mu_);
    return ref_;
}

// ------------------------------------------------------------------------------------------------ 3DoF

void Orientation::update(std::int64_t ts_ns, const Vec3& accel, const Vec3& gyro) {
    double dt = last_ts_ == 0 ? 0.001 : static_cast<double>(ts_ns - last_ts_) / 1e9;
    last_ts_ = ts_ns;
    dt = std::clamp(dt, 1e-4, 0.01);
    const double an = geom::norm(accel);
    // the gyro bias: the mean over a second of rest (small rotation, gravity alone), refreshed at every rest
    if (geom::norm(gyro) < 0.03 && std::fabs(an - G) < 0.3) {
        rest_sum_ = geom::add(rest_sum_, gyro);
        if (++rest_n_ >= 1000) {
            bias_ = geom::scale(rest_sum_, 1.0 / rest_n_);
            rest_sum_ = {};
            rest_n_ = 0;
        }
    } else {
        rest_sum_ = {};
        rest_n_ = 0;
    }
    w_ = geom::sub(gyro, bias_);
    if (!init_) {
        if (an >= 1.0) {
            const Vec3 u = geom::scale(accel, 1.0 / an);
            q_ = slam::qnorm(Quat{-u[2], 0.0, u[0], 1.0 + u[1]});
            init_ = true;
        }
        return;
    }
    Vec3 w = w_;
    if (std::fabs(an - G) < 1.0) {
        const Vec3 up = slam::qrot(slam::qconj(q_), Vec3{0, 1, 0});
        const Vec3 e = geom::cross(geom::scale(accel, 1.0 / an), up);
        w = geom::add(w, geom::scale(e, 0.5));
    }
    q_ = slam::qnorm(slam::qmul(q_, Quat{w[0] * 0.5 * dt, w[1] * 0.5 * dt, w[2] * 0.5 * dt, 1.0}));
}

// ------------------------------------------------------------------------------------------------ HID

class Daemon::Hid : public hmd::Handler {
public:
    explicit Hid(Daemon& d) : d_(d) {}
    void on_state(hmd::State s) override {
        if (s == hmd::State::Connected) {
            d_.need_video_ = true; // video_prepare blocks: not from the session thread
        }
        d_.say_(s == hmd::State::Connected ? "headset connected" : "headset disconnected");
    }
    void on_imu(const hmd::ImuSample& s) override { d_.on_imu(s); }
    void on_controller(const hmd::ControllerReport& r) override { d_.on_controller(r); }
    void on_log(std::string_view line) override { d_.on_log(line); }

private:
    Daemon& d_;
};

Daemon::Daemon(Options o, std::function<void(std::string_view)> say) : o_(std::move(o)), say_(std::move(say)) {
    if (const char* a = std::getenv("PIMAX_CTRL_ANGLE"); a != nullptr && *a != '\0') {
        ctrl_angle_deg_ = ctrl_angle_clamp(std::strtod(a, nullptr));
    }
    if (const char* p = std::getenv("PIMAX_CTRL_PREDICT_MS"); p != nullptr && *p != '\0') {
        const double ms = std::strtod(p, nullptr);
        ctrl_predict_s_ = std::isnan(ms) ? 0.025 : std::clamp(ms, 0.0, 50.0) * 1e-3;
    }
}

Daemon::~Daemon() { stop(); }

bool Daemon::start() {
    // the head origin in the IMU frame, from the factory calibration (the driver's head_in_imu)
    if (auto c = cam::DeviceCalib::load_default(o_.calibration.empty() ? std::nullopt
                                                                        : std::optional<std::string>(o_.calibration))) {
        head_in_imu_ = slam::default_head_in_imu(c->first);
        say_("head origin from " + c->second);
    } else {
        say_("no headset calibration: head origin at the IMU");
    }
    // the headset's HID: ours while we run (the app's inside-out marker makes pibridge-keepalived let go of it; run
    // by hand, write it ourselves)
    if (!pause::active_marker()) {
        std::error_code ec;
        std::filesystem::create_directories(pause::marker_path().parent_path(), ec);
        std::ofstream(pause::marker_path()) << pause::marker_text(::getpid(), "headtrack");
        own_marker_ = true;
    }
    for (int i = 0; i < 150 && !keepalived_released(); i++) {
        std::this_thread::sleep_for(100ms);
    }
    if (!keepalived_released()) {
        say_("pibridge-keepalived did not let go of the headset in 15 s");
        return false;
    }
    pub_ = ps::Publisher::create(o_.pose_file);
    if (!pub_) {
        say_("cannot write " + o_.pose_file.string());
        return false;
    }
    std::string err;
    vit_ = VitHost::open(o_.vit_library, static_cast<std::uint32_t>(o_.cams.size()), o_.slam_config, &err);
    if (!vit_) {
        say_("SLAM engine: " + err);
        return false;
    }
    say_("SLAM engine " + o_.vit_library + ", " + std::to_string(o_.cams.size()) + " cameras" +
         (vit_->has_controllers() ? ", controllers by Pimax's LED engine" : ", no controller tracking"));
    say_("controller forward-prediction cap: " + std::to_string(static_cast<int>(ctrl_predict_s_ * 1000.0 + 0.5)) +
         " ms (PIMAX_CTRL_PREDICT_MS; 0 = SteamVR predicts all, as for the head)");
    head_hist_.resize(2048);
    world_ = std::make_unique<World>();
    world_->set_log([this](std::string_view l) { say_(l); }); // floor/anchor diagnostics into the headtrack log
    if (o_.engine == "basalt") {
        ht_ = std::make_unique<cam::headpose::HeadTracker>(slam::SupervisorParams{}, cam::headpose::HeadParams::from_env(),
                                                            head_in_imu_);
        for (std::size_t h = 0; h < 2; h++) {
            fuser_[h] = std::make_unique<ctrl::fuse::Fuser>(h, ctrl::fuse::Config::from_env(),
                                                            ctrl::fuse::calib_or_default(ctrl::fuse::calib_path(), h));
        }
        say_("Basalt: PiBridge's output layer (" + ht_->p.describe() + "), controllers from pibridge-ctrltrack");
    }
    hmd::Config cfg;
    cfg.log = [this](std::string_view l) { say_(l); };
    cfg.auto_attach = true; // both controllers at connect, again when one is awake but silent (the driver's attach)
    auto s = hmd::Session::open(std::move(cfg), std::make_unique<Hid>(*this));
    if (!s) {
        say_("headset HID: " + s.error().message());
        return false;
    }
    session_ = std::move(*s);
    running_ = true;
    camera_ = std::thread([this] { camera_loop(); });
    say_(haptic_rx_.ok() ? "vibration: listening on @" + hmd::haptics::socket_name()
                         : "vibration: socket @" + hmd::haptics::socket_name() + " not bound (only the request file)");
    output_ = std::thread([this] { output_loop(); });
    serials_ = std::thread([this] { serials_loop(); });
    return true;
}

void Daemon::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (camera_.joinable()) {
        camera_.join();
    }
    if (output_.joinable()) {
        output_.join();
    }
    if (serials_.joinable()) {
        serials_.join();
    }
    if (pub_) {
        pub_->gone(monotonic_ns());
    }
    session_.reset();
    vit_.reset();
    if (own_marker_) {
        std::error_code ec;
        std::filesystem::remove(pause::marker_path(), ec);
    }
}

// The controllers' own serials, once per connection (ctrl_serials.h; the app shows them): in a thread of its own, as
// the request waits for the headset's reply that the session thread delivers.
void Daemon::serials_loop() {
    std::array<bool, 2> read{false, false};
    bool headset_read = false;
    while (running_) {
        if (!headset_read && session_ && session_->is_connected()) {
            headset_read = true; // the headset's and its optical module's, once
            for (const std::uint8_t slot : {std::uint8_t{0}, std::uint8_t{5}}) {
                if (auto sn = session_->read_serial(slot, std::chrono::milliseconds(800)); sn && !sn->empty()) {
                    hmd::ctrl_serials::store(slot, *sn);
                }
            }
        }
        for (int i = 0; i < 2; i++) {
            std::int64_t last = 0;
            {
                std::lock_guard l(hands_[i].mu);
                last = hands_[i].report ? hands_[i].report_ns : 0;
            }
            const bool on = last != 0 && monotonic_ns() - last < 1'000'000'000;
            if (!on) {
                read[i] = false;
            } else if (!read[i] && session_) {
                read[i] = true;
                const std::uint8_t slot = i == 0 ? 2 : 3;
                if (auto sn = session_->read_serial(slot, std::chrono::milliseconds(800)); sn && !sn->empty()) {
                    hmd::ctrl_serials::store(slot, *sn);
                }
            }
        }
        for (int k = 0; k < 10 && running_; k++) {
            std::this_thread::sleep_for(50ms);
        }
    }
}

void Daemon::on_imu(const hmd::ImuSample& s) {
    const std::int64_t ts = time_.map(s.ts_us, monotonic_ns());
    // OpenXR head axes (pimax_hmd.c map_axes, PIMAX_AXIS_MODE=1): (x, y, z) -> (-x, y, -z)
    const float a[3] = {static_cast<float>(-s.accel_g[0] * G), static_cast<float>(s.accel_g[1] * G),
                        static_cast<float>(-s.accel_g[2] * G)};
    const double r = M_PI / 180.0;
    const float g[3] = {static_cast<float>(-s.gyro_dps[0] * r), static_cast<float>(s.gyro_dps[1] * r),
                        static_cast<float>(-s.gyro_dps[2] * r)};
    {
        std::lock_guard l(imu_mu_);
        orient_.update(ts, v3(a), v3(g));
        last_imu_ns_ = ts;
    }
    if (ht_) {
        std::lock_guard l(ht_mu_);
        ht_->push_imu(cam::headpose::ImuSample{ts, v3(a), v3(g)});
    }
    if (vit_) {
        vit_->push_imu(ts, a, g);
    }
}

void Daemon::on_log(std::string_view line) {
    // the volume buttons to the paused pibridge-keepalived, which changes the headset volume ("<seq> <dir> <pressed>")
    int dir = 0, pressed = 0;
    if (line.find("app: vol up 1") != std::string_view::npos) {
        dir = 1, pressed = 1;
    } else if (line.find("app: vol up 0") != std::string_view::npos) {
        dir = 1;
    } else if (line.find("app: vol down 1") != std::string_view::npos) {
        dir = -1, pressed = 1;
    } else if (line.find("app: vol down 0") != std::string_view::npos) {
        dir = -1;
    } else {
        return;
    }
    static unsigned seq = 0;
    const auto path = status::runtime_dir() / "volume-button";
    const auto tmp = path.string() + ".tmp";
    std::ofstream(tmp) << ++seq << " " << dir << " " << pressed << "\n";
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
}

void Daemon::camera_loop() {
    auto cam_r = cam::Camera::open(o_.camera, 60, cam::ReplayOptions{});
    if (!cam_r) {
        say_("camera: " + cam_r.error().text());
        return;
    }
    auto& cam = **cam_r;
    say_("camera " + cam.describe());
    std::vector<std::vector<std::uint8_t>> tiles(o_.cams.size());
    std::vector<const std::uint8_t*> ptrs(o_.cams.size());
    std::uint64_t sets = 0, untimed = 0;
    // auto-exposure (pimax_camera.c ae_step over ae_capi.cpp): PIMAX_CAM_AE unset/0 off, "dry" compute only, "1" send
    const char* ae_env = std::getenv("PIMAX_CAM_AE");
    const std::string ae_mode = ae_env != nullptr ? ae_env : "";
    std::optional<cam::ae::AutoExposure> ae;
    const cam::ae::Mask ae_mask{};
    std::uint64_t ae_sent = 0, ae_errors = 0, ae_refused = 0;
    if (ae_mode == "1" || ae_mode == "dry") {
        const auto [model, why] = cam::ae::model_detect();
        ae.emplace(cam::ae::AeParams::from_env_model(model));
        say_(cam::ae::model_line(*ae, why));
        say_(cam::ae::start_line(*ae));
        say_(ae_mode == "1" ? "auto-exposure ON: sending the exposure push (HID 0x51) to the headset"
                            : "auto-exposure DRY RUN: computing only, nothing is sent");
    } else if (!ae_mode.empty() && ae_mode != "0") {
        say_("PIMAX_CAM_AE=" + ae_mode + " unknown (0, dry or 1); auto-exposure off");
    }
    auto ae_step = [&](const cam::FrameInfo& fi) {
        if (!ae || (fi.flags & cam::FLAG_META_VALID) == 0 || (fi.flags & cam::FLAG_REPLAY) != 0) {
            return;
        }
        std::array<cam::TileView, 4> views{};
        for (std::size_t t = 0; t < 4; t++) {
            auto v = cam.tile_view(t);
            if (!v) {
                return;
            }
            views[t] = *v;
        }
        const auto setting = cam::ae::AeSetting::from_info(fi, ae->p.led_exposure, ae->p.led_gain);
        const auto stats = ae->frame_stats(views, ae_mask);
        const auto d = ae->update(setting, stats, static_cast<std::uint64_t>(monotonic_ns()));
        for (const auto& ev : ae->take_events()) {
            say_(cam::ae::event_line(ev, *ae));
            if (ev.kind == cam::ae::LightEvent::LowRaised) {
                cam::ae::toast(cam::ae::LOW_LIGHT_TEXT);
            }
        }
        const auto tgt = ae->target().value_or(setting);
        cam::headpose::scene::publish(std::fmin(d.stats[0].trimmed_mean, d.stats[1].trimmed_mean));
        if (!d.send || ae_mode != "1") {
            return;
        }
        if (!cam::ae::model_allows(ae->p.model, tgt)) {
            if (ae_refused++ % 1000 == 0) {
                say_("auto-exposure: a setting outside the Windows ranges, not sent");
            }
            return;
        }
        const auto pl = cam::ae::payload(tgt);
        if (session_->send(cam::ae::AE_CMD, cam::ae::AE_OPT, pl)) {
            ae_sent++;
        } else if (ae_errors++ % 50 == 0) {
            say_("auto-exposure: exposure push failed (" + std::to_string(ae_errors) + " errors)");
        }
    };
    while (running_) {
        auto f = cam.next_frame(100);
        if (!f || !*f) {
            continue;
        }
        const cam::FrameInfo& fi = **f;
        {
            // DIAG: verify the firmware's tracking<->LED exposure interleave on THIS headset directly (not from a code
            // comment): the first 24 frame kinds as a sequence, and the per-second count of each kind.
            static long kLed = 0, kTrk = 0, kUnk = 0;
            static std::string kSeq;
            static std::int64_t kLast = 0;
            const char c = fi.kind == static_cast<std::uint32_t>(cam::FrameKind::Led)        ? 'L'
                           : fi.kind == static_cast<std::uint32_t>(cam::FrameKind::Tracking) ? 'T'
                                                                                             : 'U';
            (c == 'L' ? kLed : c == 'T' ? kTrk : kUnk)++;
            if (kSeq.size() < 24) {
                kSeq += c;
                if (kSeq.size() == 24) {
                    say_("frame kind sequence (first 24): " + kSeq);
                }
            }
            const std::int64_t know = monotonic_ns();
            if (kLast == 0) {
                kLast = know;
            } else if (know - kLast >= 2'000'000'000) {
                const double s = static_cast<double>(know - kLast) / 1e9;
                say_("camera frames/s: LED " + std::to_string(static_cast<int>(kLed / s + 0.5)) + ", Tracking " +
                     std::to_string(static_cast<int>(kTrk / s + 0.5)) + ", other " +
                     std::to_string(static_cast<int>(kUnk / s + 0.5)));
                kLast = know;
                kLed = kTrk = kUnk = 0;
            }
        }
        if (fi.kind == static_cast<std::uint32_t>(cam::FrameKind::Led) && vit_->has_controllers() &&
            (fi.flags & cam::FLAG_NO_DEV_TIME) == 0) {
            // all four tiles of the LED exposure to Pimax's LED engine, as Pimax Play hands them (40 us)
            const auto ref = time_.ref();
            if (ref.valid != 0) {
                std::vector<std::vector<std::uint8_t>> led(4);
                std::vector<const std::uint8_t*> lp(4);
                bool ok = true;
                for (std::size_t t = 0; t < 4 && ok; t++) {
                    auto tile = cam.tile(t);
                    ok = tile.has_value();
                    if (ok) {
                        led[t] = std::move(*tile);
                        lp[t] = led[t].data();
                    }
                }
                if (ok) {
                    vit_->push_led(static_cast<std::int64_t>(cam::imu_dev_to_mono_ns(fi.dev_ts_us, ref)), 40000, lp);
                    // DIAG (~1/s): how many bright LED dots each of the 4 LED cameras sees, and the brightest pixel.
                    // If this drops on fast swings / bad angles, the diodes are the limit (pose stays "optical" but gets
                    // noisy -> the "bending"); if the peak is low, the LED exposure is too dim.
                    static long ledDiagN = 0;
                    if (++ledDiagN % 90 == 0) {
                        int b[4] = {}, pk[4] = {};
                        for (int t = 0; t < 4; t++) {
                            b[t] = led_blobs(lp[static_cast<std::size_t>(t)], 640, 480, 230, 2, pk[t]);
                        }
                        say_("LED dots per camera: " + std::to_string(b[0]) + " " + std::to_string(b[1]) + " " +
                             std::to_string(b[2]) + " " + std::to_string(b[3]) + " (total " +
                             std::to_string(b[0] + b[1] + b[2] + b[3]) + "), peak px " + std::to_string(pk[0]) + "/" +
                             std::to_string(pk[1]) + "/" + std::to_string(pk[2]) + "/" + std::to_string(pk[3]));
                        // DIAG: dump the 4 LED tiles (overwritten ~1/s) so we can look at the controllers' LED dots.
                        const char* rt = std::getenv("XDG_RUNTIME_DIR");
                        const std::string base =
                            std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge-led-cam";
                        for (int t = 0; t < 4; t++) {
                            std::ofstream f(base + std::to_string(t) + ".pgm", std::ios::binary);
                            if (f) {
                                f << "P5\n640 480\n255\n";
                                f.write(reinterpret_cast<const char*>(lp[static_cast<std::size_t>(t)]), 640 * 480);
                            }
                        }
                    }
                }
            }
            continue;
        }
        if (fi.kind != static_cast<std::uint32_t>(cam::FrameKind::Tracking)) {
            continue;
        }
        ae_step(fi);
        // frames on the IMU timeline (pimax_camera.c frame_time); the Motion-JPEG stream has host time only
        std::int64_t ts = 0;
        if ((fi.flags & cam::FLAG_NO_DEV_TIME) != 0) {
            ts = static_cast<std::int64_t>(fi.mono_ts_ns);
        } else {
            const auto ref = time_.ref();
            if (ref.valid == 0) {
                untimed++;
                continue; // no IMU yet: the engine cannot use it
            }
            ts = static_cast<std::int64_t>(cam::imu_dev_to_mono_ns(fi.dev_ts_us, ref));
        }
        bool all = true;
        for (std::size_t i = 0; i < o_.cams.size(); i++) {
            auto t = cam.tile(o_.cams[i]);
            if (!t) {
                all = false; // all or nothing: one frame per camera per timestamp
                break;
            }
            tiles[i] = std::move(*t);
            ptrs[i] = tiles[i].data();
        }
        if (!all) {
            continue;
        }
        // DIAG: dump the tracking-exposure tiles (~every 2s) next to the LED ones, to compare both exposures per camera.
        {
            static long trkDumpN = 0;
            if (++trkDumpN % 50 == 0) {
                const char* rt = std::getenv("XDG_RUNTIME_DIR");
                const std::string base =
                    std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge-trk-cam";
                for (std::size_t i = 0; i < o_.cams.size() && i < 4; i++) {
                    std::ofstream f(base + std::to_string(o_.cams[i]) + ".pgm", std::ios::binary);
                    if (f) {
                        f << "P5\n640 480\n255\n";
                        f.write(reinterpret_cast<const char*>(ptrs[i]), 640 * 480);
                    }
                }
            }
        }
        // Basalt: the mid-exposure (the stamp marks the exposure start; pimax_camera.c push_slam_frames)
        std::int64_t adj = o_.cam_offset_ns;
        if (o_.exposure_mid) {
            std::int64_t sum = 0;
            for (std::uint32_t c : o_.cams) {
                sum += c < fi.exposure.size() ? fi.exposure[c] : 0;
            }
            adj += sum * 500 / static_cast<std::int64_t>(o_.cams.size());
        }
        vit_->push_frames(ts + adj, ptrs, cam::TILE_W, cam::TILE_H);
        if (++sets == 1) {
            say_("first frame set to the engine");
        }
    }
    say_("camera: " + std::to_string(sets) + " frame sets, " + std::to_string(untimed) + " before the IMU" +
         (ae ? ", auto-exposure " + std::to_string(ae->changes) + " changes, " + std::to_string(ae_sent) + " sent" : ""));
}

void Daemon::to_world(ps::Device& d, bool position_valid, std::int64_t now_ns) {
    const RigidPose raw{{d.position[0], d.position[1], d.position[2]},
                        {d.orientation[0], d.orientation[1], d.orientation[2], d.orientation[3]}};
    const double dps = geom::norm(v3(d.angular_velocity.data())) * 180.0 / M_PI;
    const RigidPose t = world_->update(d.t_ns, raw, position_valid, dps, now_ns);
    const RigidPose w = compose(t, raw);
    put(d.position, w.p);
    put(d.orientation, w.q);
    put(d.linear_velocity, slam::qrot(t.q, v3(d.linear_velocity.data())));
    put(d.angular_velocity, slam::qrot(t.q, v3(d.angular_velocity.data())));
}

ps::Device Daemon::head_from_engine(const EnginePose& e) {
    // pimax_camera.c passthrough_head without its prediction step (SteamVR predicts from the velocities and the
    // sample time): the IMU pose of the engine turned y up, the head origin, the world transform
    const Quat q = slam::qnorm(slam::qmul(TO_YUP, q4(e.q)));
    const Vec3 p = slam::qrot(TO_YUP, v3(e.p));
    const Vec3 v = slam::qrot(TO_YUP, v3(e.v));
    Vec3 w_body{};
    Quat q3 = slam::QUAT_ID;
    {
        std::lock_guard l(imu_mu_);
        w_body = orient_.gyro();
        q3 = orient_.q();
    }
    const Vec3 hr = slam::qrot(q, head_in_imu_);
    const Vec3 head = geom::add(p, hr);
    const Vec3 w = slam::qrot(q, w_body);
    if (!zeroed_) {
        // the engine tracks in its saved-map frame, tens of metres from the origin: start near it, keep the height
        zeroed_ = true;
        const Vec3 hw = slam::qrot(world_q_, head);
        world_t_[0] = -hw[0];
        world_t_[2] = -hw[2];
        say_("start near the origin (removed the engine's " +
             std::to_string(std::sqrt(hw[0] * hw[0] + hw[2] * hw[2])) + " m saved-map offset)");
    }
    ps::Device d;
    d.t_ns = e.ts_ns;
    d.state = 1;
    const Vec3 pos = geom::add(slam::qrot(world_q_, head), world_t_);
    const Quat ori = slam::qnorm(slam::qmul(world_q_, q));
    put(d.position, pos);
    put(d.orientation, ori);
    put(d.linear_velocity, slam::qrot(world_q_, geom::add(v, geom::cross(w, hr))));
    put(d.angular_velocity, slam::qrot(world_q_, w));
    // for a later loss: the orientation continuous, the position held
    fix_ = slam::qnorm(slam::qmul(ori, slam::qconj(q3)));
    held_ = pos;
    return d;
}

ps::Device Daemon::lost_head(std::int64_t now_ns) {
    ps::Device d;
    Quat q3 = slam::QUAT_ID;
    Vec3 w_body{};
    bool valid = false;
    {
        std::lock_guard l(imu_mu_);
        valid = orient_.valid();
        q3 = orient_.q();
        w_body = orient_.gyro();
        d.t_ns = last_imu_ns_ != 0 ? last_imu_ns_ : now_ns;
    }
    if (!valid) {
        return d; // state 0: no pose at all yet
    }
    d.state = 1;
    const Quat ori = slam::qnorm(slam::qmul(fix_, q3));
    put(d.position, held_);
    put(d.orientation, ori);
    put(d.angular_velocity, slam::qrot(ori, w_body));
    return d;
}

void Daemon::on_controller(const hmd::ControllerReport& r) {
    if (r.slot != 2 && r.slot != 3) {
        return;
    }
    const int side = r.slot == 2 ? 0 : 1;
    const std::int64_t now = monotonic_ns();
    // the controller IMU in its raw axes: m/s^2 (1024 LSB/g), rad/s (8.2 LSB/(deg/s), PIMAX_CTRL_GYRO_LSB_DPS)
    float a[3], g[3];
    for (int k = 0; k < 3; k++) {
        a[k] = static_cast<float>(r.raw_accel[k] / 1024.0 * G);
        g[k] = static_cast<float>(r.raw_gyro[k] * CTRL_GYRO_SCALE);
    }
    bool buttonsChanged = false;
    if (side == 1) {
        right_menu_.store((r.buttons & hmd::BTN_APP_MENU) != 0);
    }
    {
        std::lock_guard l(hands_[side].mu);
        Hand& hd = hands_[side];
        if (r.buttons != hd.logged_buttons) {
            hd.logged_buttons = r.buttons;
            buttonsChanged = true;
        }
        hd.report = r;
        hd.report_ns = now;
        hd.imu.update(now, v3(a), v3(g));
        // the Pimax button: recenter on a long press (~0.8 s), so a brief touch never recenters (pimax_ctrl.c)
        const bool down = (r.buttons & hmd::BTN_MENU) != 0;
        if (down && hd.pimax_down_ns == 0) {
            hd.pimax_down_ns = now;
            hd.recentered = false;
        } else if (!down) {
            hd.pimax_down_ns = 0;
        }
        // publish the long-press progress (0..1) for the in-game recenter ring (pibridge-overlay reads this file and
        // draws a filling ring in front of the HMD); the recenter fires when it completes.
        {
            // Only the hand that holds the button writes; the OTHER hand's reports (not holding) must never write a 0,
            // or the file thrashes between the hold's progress and 0 at the report rate -- the ring blinks.
            static std::atomic<int> holdHand{-1}; // the hand holding the recenter button (-1 none)
            static std::atomic<int> lastQ{-1};
            const bool holding = down && !hd.recentered;
            double prog = 0.0;
            bool publish = false;
            if (holding) {
                holdHand.store(side);
                prog = std::clamp(static_cast<double>(now - hd.pimax_down_ns) / 800'000'000.0, 0.0, 1.0);
                publish = true;
            } else if (holdHand.load() == side) {
                holdHand.store(-1); // this hand released (or the recenter fired): the ring goes away
                publish = true;
            }
            if (publish) {
                const int q = static_cast<int>(std::lround(prog * 40.0));
                if (lastQ.exchange(q) != q) { // only on a bucket change, so this is not a per-report file write
                    const auto path = status::runtime_dir() / "recenter-progress";
                    const std::string tmp = path.string() + ".tmp";
                    std::ofstream(tmp) << prog << "\n";
                    ::rename(tmp.c_str(), path.string().c_str());
                }
            }
        }
        if (down && !hd.recentered && now - hd.pimax_down_ns > 800'000'000) {
            hd.recentered = true;
            // A recenter needs BOTH parts, because SteamVR splits them:
            //  - the LIVE pose shift (our anchor), which a RUNNING game sees -- a game keeps the tracking origin it got
            //    at launch, and only the live pose stream (as SteamVR's own recenter does) moves it; the chaperone
            //    config does not reach it. This is the in-game recenter.
            //  - the chaperone commit (pibridge-roomsetup --vr-standing, a client -- a driver cannot), which sets the
            //    forward/centre SteamVR Home shows and persists it. It reads the already-anchor-recentered raw head
            //    ~1 s later, so it captures the same recentre (no double shift). Without --radius it keeps the boundary.
            if (world_) {
                world_->request_recenter();
            }
            // --keep-floor: the recenter moves centre / forward ONLY. The floor is the room setup's (its saved plane,
            // put into this session's raw space when the room was recognised) and must never be re-derived from the
            // head at the press: head y - a default eye height at an arbitrary posture put the floor below the real one.
            say_(std::string("Pimax button long press (") + (side == 0 ? "left" : "right") +
                 "): recenter -> anchor (live, in-game) + chaperone (--vr-standing --keep-floor)");
            std::thread([] {
                if (std::system("pibridge-roomsetup --vr-standing --keep-floor >/dev/null 2>&1") != 0) {
                    // best-effort: a failed commit only misses the Home-side update; the live anchor recentre still took
                }
            }).detach();
        }
    }
    if (vit_) {
        vit_->push_controller_imu(static_cast<std::uint32_t>(side), now, a, g); // as Pimax Play feeds its LED engine
    }
    if (fuser_[side]) {
        std::lock_guard l(ht_mu_);
        fuser_[side]->report(static_cast<std::uint64_t>(now), r.raw.data());
    }
    if (buttonsChanged) {
        // which bits each physical button sets, per hand (A 0x2, B 0x1, menu 0x4, stick 0x8, Pimax 0x10)
        char line[96];
        std::snprintf(line, sizeof line, "controller %s buttons 0x%02x", side == 0 ? "left" : "right", r.buttons);
        say_(line);
    }
}

std::optional<Daemon::HeadAt> Daemon::head_at(std::int64_t ts) const {
    // the newest published head pose at or before ts (else the oldest kept)
    std::optional<HeadAt> best;
    for (const HeadAt& h : head_hist_) {
        if (h.ts == 0) {
            continue;
        }
        if (h.ts <= ts && (!best || h.ts > best->ts)) {
            best = h;
        }
    }
    if (!best) {
        for (const HeadAt& h : head_hist_) {
            if (h.ts != 0 && (!best || h.ts < best->ts)) {
                best = h;
            }
        }
    }
    return best;
}

ps::Device Daemon::controller(int side, std::int64_t now_ns) {
    // pimax_ctrl.c engine_pose: the controller relative to the engine's head, put
    // under our head pose at the sample time (so recenter and the room apply as to the head), into the aim frame
    // into the aim frame (rot_x(-52 deg), as pibridge-ctrltrack defines it), on to now. Out of the cameras' view:
    // position held, orientation turned by the controller gyro, for CTRL_HOLD_NS. Every path then pitches the aim by
    // the controller angle (aim_out); CustomHeadsetOpenVR takes it as the tip, as SteamVR took Monado's aim.
    Hand& hd = hands_[side];
    std::optional<hmd::ControllerReport> rep;
    std::int64_t rep_ns = 0;
    Quat imu_now = slam::QUAT_ID;
    {
        std::lock_guard l(hd.mu);
        rep = hd.report;
        rep_ns = hd.report_ns;
        imu_now = hd.imu.q();
    }
    ps::Device d;
    if (!rep || now_ns - rep_ns > 1'000'000'000) {
        return d; // state 0: not connected
    }
    // inputs, in the bits of the original Crystal's report (what CustomHeadsetOpenVR maps to Touch)
    std::uint32_t pressed = 0, touched = 0;
    pressed |= (rep->buttons & hmd::BTN_A) != 0 ? 0x01u : 0u;      // A / X
    pressed |= (rep->buttons & hmd::BTN_B) != 0 ? 0x02u : 0u;      // B / Y
    pressed |= (rep->buttons & hmd::BTN_APP_MENU) != 0 ? 0x04u : 0u;
    if (side == 0 && right_menu_.load()) {
        pressed |= 0x04u; // the right Menu button opens SteamVR's dashboard too (right_menu_ in headtrack.h)
    }
    pressed |= (rep->buttons & hmd::BTN_STICK) != 0 ? 0x08u : 0u;
    // the Pimax button is not passed on: a long press recenters (on_controller)
    const auto tb = rep->touch_bits();
    touched |= tb.thumb() ? 0x03u : 0u;
    touched |= tb.stick ? 0x08u : 0u;
    touched |= rep->trigger > 0 ? 0x10u : 0u;
    touched |= tb.hand_on ? 0x20u : 0u;
    d.buttons = pressed | touched << 8;
    d.axes = {rep->trigger / 255.0f, rep->grip / 255.0f, std::clamp(static_cast<float>(rep->stick_x) / 127.0f, -1.0f, 1.0f),
              std::clamp(static_cast<float>(rep->stick_y) / 127.0f, -1.0f, 1.0f)};
    d.battery = rep->battery;
    d.t_ns = now_ns;
    d.state = 4; // connected, no pose yet

    if (fuser_[side]) {
        // Basalt: the fuser's answer at now (fresh optical + gyro, out of view an arm-model hold, then nothing)
        const auto head = head_at(now_ns);
        if (head) {
            ctrl::fuse::P hp;
            hp.q = head->q;
            hp.p = head->pos;
            std::pair<ctrl::fuse::State, ctrl::fuse::P> got;
            {
                std::lock_guard l(ht_mu_);
                got = fuser_[side]->get(static_cast<std::uint64_t>(now_ns), hp);
            }
            const int fstate = static_cast<int>(got.first);
            if (got.first != ctrl::fuse::State::None) {
                put(d.position, got.second.p);
                put(d.orientation, got.second.q);
                d.state = got.first == ctrl::fuse::State::Optical ? 7 : 5;
            }
            if (fstate != hd.logged_state) {
                static const char* names[] = {"no pose (no optical fix)", "optical (pibridge-ctrltrack)",
                                              "out of view: held relative to the head"};
                say_(std::string(side == 0 ? "left" : "right") + " controller: " + names[fstate]);
                hd.logged_state = fstate;
            }
        }
        aim_out(d, now_ns);
        d.sample = ++ctrl_sample_[side];
        return d;
    }
    const auto e = vit_ ? vit_->controller_pose(static_cast<std::uint32_t>(side)) : std::nullopt;
    const bool seen = e && now_ns - e->ts_ns < 300'000'000;
    const auto head = seen ? head_at(e->ts_ns) : std::nullopt;
    int state = 0;
    if (seen && head) {
        const Quat hq = q4(e->head_q);
        const Quat hqi = slam::qconj(hq);
        const Quat rq = slam::qmul(hqi, q4(e->q));
        Vec3 rt = slam::qrot(hqi, geom::sub(v3(e->p), v3(e->head_p)));
        rt = geom::sub(rt, head_in_imu_); // the head frame is the IMU frame moved to the head origin
        const Quat q = slam::qnorm(slam::qmul(head->q, rq));
        const Vec3 pos = geom::add(slam::qrot(head->q, rt), head->pos);
        const Quat R = slam::qmul(head->q, hqi); // the engine's world -> ours
        const Vec3 v = slam::qrot(R, v3(e->lin_vel));
        const Quat dfi = q4(e->dev_from_imu);
        const Quat qdev = slam::qnorm(slam::qmul(q, slam::qconj(dfi))); // world_from_device (LED model frame)
        Vec3 g_imu{};
        for (int k = 0; k < 3; k++) {
            g_imu[k] = rep->raw_gyro[k] * CTRL_GYRO_SCALE;
        }
        const Vec3 w_body = slam::qrot(dfi, g_imu);
        // on to now, as pimax_ctrl.c engine_pose: the engine's velocity and the controller gyro, at most 25 ms ahead
        // (Pimax's own cap); SteamVR predicts the rest from now as for the head
        const double dt = std::clamp(static_cast<double>(now_ns - e->ts_ns) * 1e-9, -0.05, ctrl_predict_s_);
        const double wn = geom::norm(w_body);
        Quat dq = slam::QUAT_ID;
        if (wn > 1e-9) {
            const double h = 0.5 * wn * dt, k = std::sin(h) / wn;
            dq = {w_body[0] * k, w_body[1] * k, w_body[2] * k, std::cos(h)};
        }
        const Quat qp = slam::qnorm(slam::qmul(qdev, dq));
        const Vec3 pp = geom::add(pos, geom::scale(v, dt));
        const Quat aim = slam::qnorm(slam::qmul(qp, DEVICE_FROM_AIM));
        put(d.position, pp);
        put(d.orientation, aim);
        put(d.linear_velocity, v);
        put(d.angular_velocity, slam::qrot(qp, w_body));
        // diagnostics (~30 Hz): the prediction the controller pose is extrapolated with. engLat is how old the engine
        // pose is (now - e->ts_ns); dt is the clamped forward prediction ([-50, +25] ms). If engLat keeps exceeding
        // 25 ms (CLAMPED), the forward prediction cannot keep up -> the pose lags, which looks like the saber "bending".
        if (now_ns - hd.diag_last_ns >= 33'000'000) {
            hd.diag_last_ns = now_ns;
            const double latMs = static_cast<double>(now_ns - e->ts_ns) * 1e-6;
            const bool clamped = latMs > 25.0 || latMs < -50.0;
            char b[224];
            std::snprintf(b, sizeof b,
                          "ctrl %s pred: engLat %.1f ms, dt %.1f ms%s, |v| %.2f m/s, |w| %.0f deg/s, pos(%.3f %.3f %.3f)",
                          side == 0 ? "L" : "R", latMs, dt * 1000.0, clamped ? " CLAMPED" : "", geom::norm(v),
                          geom::norm(w_body) * 180.0 / M_PI, pp[0], pp[1], pp[2]);
            say_(b);
        }
        // Stamp the pose at the time it is valid for (e->ts_ns + dt), not "now": so when dt < the engine latency
        // (PIMAX_CTRL_PREDICT_MS reduced, or the 25 ms cap hit) SteamVR predicts the remainder from the right age,
        // exactly as it does for the head -- instead of getting an under-predicted pose stamped as current (a lag).
        d.t_ns = e->ts_ns + static_cast<std::int64_t>(dt * 1e9);
        d.state = 7;
        hd.eng_have = true;
        hd.eng_pos = pp;
        hd.eng_aim = aim;
        hd.eng_imu = imu_now;
        hd.eng_dfi = dfi;
        hd.eng_last_ns = now_ns;
        state = 1;
    } else if (hd.eng_have && now_ns - hd.eng_last_ns < CTRL_HOLD_NS) {
        // held: the turn the IMU made since the last engine pose (IMU frame -> device frame -> aim frame)
        const Quat di = slam::qmul(slam::qconj(hd.eng_imu), imu_now);
        const Quat dd = slam::qmul(slam::qmul(hd.eng_dfi, di), slam::qconj(hd.eng_dfi));
        const Quat da = slam::qmul(slam::qmul(slam::qconj(DEVICE_FROM_AIM), dd), DEVICE_FROM_AIM);
        put(d.position, hd.eng_pos);
        put(d.orientation, slam::qnorm(slam::qmul(hd.eng_aim, da)));
        d.state = 5; // orientation only, the position held
        state = 2;
    }
    if (state != hd.logged_state) {
        static const char* names[] = {"no pose (Pimax's engine does not see it)", "Pimax's LED engine",
                                      "held, turned by the gyro (out of the cameras' view)"};
        say_(std::string(side == 0 ? "left" : "right") + " controller: " + names[state]);
        hd.logged_state = state;
    }
    aim_out(d, now_ns);
    d.sample = ++ctrl_sample_[side];
    return d;
}

void Daemon::aim_out(ps::Device& d, std::int64_t now_ns) {
    if (now_ns - angle_polled_ns_ >= 500'000'000) {
        angle_polled_ns_ = now_ns;
        double deg = 0;
        if (std::ifstream f(status::runtime_dir() / "pimax-ctrl-angle"); f >> deg) {
            deg = ctrl_angle_clamp(deg);
            if (deg != ctrl_angle_deg_) {
                ctrl_angle_deg_ = deg;
                char b[64];
                std::snprintf(b, sizeof b, "controller angle %+.1f deg (run-time change)", deg);
                say_(b);
            }
        }
    }
    if (d.state != 5 && d.state != 7) {
        return;
    }
    const double h = ctrl_angle_deg_ * M_PI / 360.0;
    const Quat up{std::sin(h), 0.0, 0.0, std::cos(h)};
    put(d.orientation, slam::qnorm(slam::qmul(q4(d.orientation.data()), up)));
}

ps::Device Daemon::basalt_head(std::int64_t now_ns, std::uint32_t& tracking) {
    // the engine's newest pose (tracker world turned y up) into the supervisor, the head at now from the output layer
    // (pimax_camera.c slam_poll_locked and pimax_camera_head_relation)
    Quat q3 = slam::QUAT_ID;
    Vec3 w_body{};
    bool have3 = false;
    {
        std::lock_guard l(imu_mu_);
        q3 = orient_.q();
        w_body = orient_.gyro();
        have3 = orient_.valid();
    }
    auto poses = vit_->poses();
    std::lock_guard l(ht_mu_);
    if (!poses.empty() || now_ns >= ht_poll_at_) {
        ht_poll_at_ = now_ns + 4'000'000;
        std::optional<cam::headpose::VioSample> vs;
        if (!poses.empty() && have3) {
            const EnginePose& e = poses.back();
            vs = cam::headpose::VioSample{e.ts_ns, slam::qrot(TO_YUP, v3(e.p)), slam::qnorm(slam::qmul(TO_YUP, q4(e.q))),
                                          slam::qrot(TO_YUP, v3(e.v)), q3};
        }
        const auto out = ht_->update(now_ns, vs);
        if ((out.events & slam::events::RESET_WANTED) != 0) {
            vit_->reset();
            say_(std::string("tracker reset (") + out.reason + ")");
        }
        if ((out.events & slam::events::ACQUIRED) != 0) {
            say_(std::string("6DoF tracking (") + out.reason + ")");
        }
        if ((out.events & slam::events::LOST) != 0) {
            say_(std::string("lost (") + out.reason + "); 3DoF at the last position");
        }
    }
    ps::Device d;
    if (!have3) {
        tracking = 0;
        return d;
    }
    namespace fl = cam::headpose::flags;
    cam::headpose::Rel r3;
    r3.q = q3;
    r3.ang_vel = slam::qrot(q3, w_body);
    r3.flags = fl::ORIENTATION_VALID | fl::ORIENTATION_TRACKED | fl::ANGULAR_VELOCITY_VALID;
    const cam::headpose::Rel r = ht_->head(now_ns, now_ns, r3);
    const bool tracked = (r.flags & fl::POSITION_TRACKED) != 0;
    tracking = tracked ? 3 : 1;
    d.t_ns = now_ns;
    d.state = 1;
    put(d.position, r.pos);
    put(d.orientation, r.q);
    put(d.linear_velocity, r.lin_vel);
    put(d.angular_velocity, r.ang_vel);
    return d;
}

void Daemon::optical_fixes(std::int64_t now_ns) {
    // pibridge-ctrltrack's new fixes (controller pose in the head frame at the LED frame time) under the head pose of
    // that time, to the fusers (pimax_ctrl.c track_thread)
    if (!ctrl_reader_ && now_ns >= ctrl_open_at_) {
        ctrl_open_at_ = now_ns + 1'000'000'000;
        if (auto r = ctrl::track::shm::PoseReader::open(ctrl::track::shm::default_path())) {
            ctrl_reader_ = std::move(*r);
            say_("controllers: optical poses from " + ctrl::track::shm::default_path());
        }
    }
    if (!ctrl_reader_) {
        return;
    }
    const auto hands = ctrl_reader_->read();
    if (!hands) {
        return;
    }
    for (std::size_t i = 0; i < 2; i++) {
        const auto& h = (*hands)[i];
        if (!fuser_[i] || h.frame_ns == 0 || h.frame_ns == ctrl_seen_[i]) {
            continue;
        }
        ctrl_seen_[i] = h.frame_ns;
        if (static_cast<std::uint64_t>(now_ns) > h.frame_ns + 2'000'000'000ull) {
            continue; // left over from an earlier tracker run
        }
        const auto head = head_at(static_cast<std::int64_t>(h.frame_ns));
        if (!head) {
            continue;
        }
        ctrl::fuse::P wh;
        wh.q = head->q;
        wh.p = head->pos;
        ctrl::fuse::P body;
        body.q = slam::qnorm({h.quat[0], h.quat[1], h.quat[2], h.quat[3]});
        body.p = {h.pos[0], h.pos[1], h.pos[2]};
        std::lock_guard l(ht_mu_);
        fuser_[i]->optical(h.frame_ns, wh.mul(body), wh, h.rms_px, h.leds);
    }
}

void Daemon::room_state_poll(std::int64_t now_ns) {
    // Pimax's own floor (GetGroundState) and localization/recognition state (GetLocModeState), through winbridge.
    // Polled at ~5 Hz: both change slowly. Observe-first: forward the raw values to room-state.json and the log so we
    // can pin down GetGroundState's float layout and GetLocModeState's byte meaning against a live headset, before
    // wiring them into the room setup (floor step, "room recognized" gate).
    if (!vit_ || now_ns < room_poll_at_) {
        return;
    }
    room_poll_at_ = now_ns + 200'000'000;
    const auto rs = vit_->room_state();
    if (!rs) {
        return; // Basalt engine, or no reply yet
    }
    // GetGroundState out (decoded from Pimax Play's Frida captures): flag byte @0, then two 3D points in the engine's
    // map frame (z up): A @[4,8,12] (live, drifts a little), B @[16,20,24] (constant per session: the locked floor
    // reference). The up axis is z, so a point's floor height in the head's y-up world equals its z component (the
    // engine->y-up turn Rx(-90) maps engine z to y; the x/z origin shift and the yaw recenter do not touch y).
    // The head-relative drop = head.y - floor.z is what the room setup needs (chaperone --eye-height); the raw map-frame
    // value must never be used directly (the "-45 m floor" bug). A live standing run tells which point is the floor
    // (drop ~1.6 m) and confirms the sign.
    const std::uint8_t flag = rs->ground[0];
    float a[3] = {0, 0, 0}, b[3] = {0, 0, 0};
    std::memcpy(a, rs->ground + 4, sizeof(a));
    std::memcpy(b, rs->ground + 16, sizeof(b));
    const bool have_head = room_head_valid_;
    const double drop_live = have_head ? double(room_head_y_) - a[2] : 0.0;
    const double drop_ref = have_head ? double(room_head_y_) - b[2] : 0.0;
    status::StatusWriter w;
    w.integer("ground_ret", rs->ground_ret)
        .integer("ground_flag", flag)
        .num("live_x", a[0]).num("live_y", a[1]).num("live_z", a[2])
        .num("ref_x", b[0]).num("ref_y", b[1]).num("ref_z", b[2])
        .num("head_y", room_head_y_)
        .boolean("head_valid", have_head)
        .num("drop_live_m", drop_live)
        .num("drop_ref_m", drop_ref)
        .integer("loc_ret", rs->loc_ret)
        .integer("loc_state", rs->loc_state);
    w.write("room-state");
    // log on a meaningful change (first reply, or a change of ret / flag / localization byte)
    if (!room_logged_ || rs->loc_ret != room_loc_ret_ || rs->loc_state != room_loc_state_ ||
        rs->ground_ret != room_ground_ret_ || flag != room_ground_flag_) {
        room_logged_ = true;
        room_loc_ret_ = rs->loc_ret;
        room_loc_state_ = rs->loc_state;
        room_ground_ret_ = rs->ground_ret;
        room_ground_flag_ = flag;
        char bbuf[320];
        std::snprintf(bbuf, sizeof bbuf,
                      "room: ground_ret=%d flag=%u drop(live=%.3f ref=%.3f m, head_y=%.3f%s) | loc_ret=%d loc_state=%u",
                      rs->ground_ret, flag, drop_live, drop_ref, room_head_y_, have_head ? "" : " no-head",
                      rs->loc_ret, rs->loc_state);
        say_(bbuf);
    }
}

void Daemon::output_loop() {
    std::int64_t last_pose_ns = 0;
    ps::State st;
    st.source = ps::SOURCE_HEADTRACK;
    while (running_) {
        if (need_video_.exchange(false) && session_) {
            // the panel on (the driver's pimax_video_prepare) and the rate for pibridge-keepalived's later keepalive
            const auto r = session_->video_prepare(o_.fps);
            say_(r ? "video prepared at " + std::to_string(o_.fps) + " Hz" : "video prepare failed: " + r.error().message());
            std::ofstream(status::runtime_dir().parent_path() / "pimax-rate") << int(o_.fps) << "\n";
        }
        const std::int64_t now = monotonic_ns();
        if (ht_) {
            std::uint32_t tracking = 0;
            st.devices[0] = basalt_head(now, tracking);
            st.tracking = tracking;
            if (st.devices[0].state != 0) {
                to_world(st.devices[0], tracking == 3, now);
                st.devices[0].sample = ++sample_;
                head_hist_[head_next_++ % head_hist_.size()] =
                    HeadAt{st.devices[0].t_ns,
                           {st.devices[0].position[0], st.devices[0].position[1], st.devices[0].position[2]},
                           {st.devices[0].orientation[0], st.devices[0].orientation[1], st.devices[0].orientation[2],
                            st.devices[0].orientation[3]}};
            }
            optical_fixes(now);
        }
        auto poses = ht_ ? std::vector<EnginePose>{} : vit_->poses();
        if (ht_) {
            // the Basalt head is done above
        } else if (!poses.empty()) {
            last_pose_ns = now;
            st.tracking = 3; // orientation and position
            st.devices[0] = head_from_engine(poses.back());
            to_world(st.devices[0], true, now);
            st.devices[0].sample = ++sample_;
            head_hist_[head_next_++ % head_hist_.size()] =
                HeadAt{st.devices[0].t_ns,
                       {st.devices[0].position[0], st.devices[0].position[1], st.devices[0].position[2]},
                       {st.devices[0].orientation[0], st.devices[0].orientation[1], st.devices[0].orientation[2],
                        st.devices[0].orientation[3]}};
            if (!tracking_) {
                tracking_ = true;
                acquired_++;
                say_("6DoF tracking (acquisition " + std::to_string(acquired_) + ")");
            }
        } else if (!ht_ && now - last_pose_ns > STALE_NS) {
            if (tracking_) {
                tracking_ = false;
                lost_++;
                say_("no engine pose for 350 ms: 3DoF at the last position (loss " + std::to_string(lost_) + ")");
            }
            st.devices[0] = lost_head(now);
            if (st.devices[0].state != 0) {
                to_world(st.devices[0], false, now);
            }
            st.tracking = st.devices[0].state != 0 ? 1 : 0; // orientation only
            st.devices[0].sample = ++sample_;
        }
        // vibration: CustomHeadsetOpenVR's (a game's) and the app's test pulse, "SLOT AMP MS" in pimax-haptic-request
        // a game's pulses arrive on the socket (every one of them), the app's test pulse in the file
        std::optional<hmd::haptics::Due> pulse = haptic_rx_.next();
        if (!pulse) {
            if (auto req = hmd::haptics::take_request(hmd::haptics::request_path()); req && *req) {
                pulse = **req;
            }
        }
        if (pulse && session_) {
            const auto [slot, amp, ms] = *pulse;
            const bool ok = session_->haptic(slot, hmd::haptics::Pulse{amp, ms});
            haptic_requests_++;
            if (now - haptic_logged_ns_ >= 500'000'000) { // every request, at most two lines a second
                haptic_logged_ns_ = now;
                const auto [sent, replaced] = session_->haptic_counts();
                say_("haptic: slot " + std::to_string(slot) + " " + std::to_string(amp) + " % " + std::to_string(ms) +
                     " ms" + (ok ? "" : " (refused)") + "; " + std::to_string(haptic_requests_) + " requests, " +
                     std::to_string(sent) + " sent, " + std::to_string(replaced) + " replaced");
            }
        }
        st.devices[1] = controller(0, now);
        st.devices[2] = controller(1, now);
        pub_->publish(st, now);
        room_head_valid_ = st.devices[0].state != 0;
        room_head_y_ = st.devices[0].position[1];
        room_state_poll(now);
        std::this_thread::sleep_for(1ms);
    }
}

} // namespace pibridge::headtrack
