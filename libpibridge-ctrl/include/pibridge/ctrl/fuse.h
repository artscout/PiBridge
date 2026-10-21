// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/fuse.rs (MIT OR Apache-2.0).
// Controller pose fusion for the optical (LED) tracking: what to show when the cameras lose the controller, and the
// controller gyro calibration learned from the optical poses.
//
// Inputs, all on the host CLOCK_MONOTONIC (ns): every controller report (raw 64 bytes: one or two gyro samples,
// big-endian i16, stamped with the device microsecond clock) and every optical fix (world_from_aim at the LED frame
// time plus the head pose then). Output for a time t and the head pose at t (Fuser::get): fresh optical (age <=
// extrap_s, extrapolated with a decaying velocity, rotated by the integrated gyro), out of view (age <= lost_s: an arm
// model around the elbow in a body frame that follows the head position and half its yaw), older: none (rest pose).
// Reacquire blends from the prediction to the optical pose; optical outliers vs the prediction are dropped (repeated
// ones accepted). The gyro calibration y = C x + d is fitted online over 9 camera-to-IMU time offsets and kept in
// ~/.config/pibridge/ctrl-imu.json per hand; a built-in one is used until then.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::ctrl::fuse {

using V3 = std::array<double, 3>;
/// Quaternion x, y, z, w.
using Q = std::array<double, 4>;
using M3 = std::array<std::array<double, 3>, 3>;

// ---------- small math ----------
Q qmul(Q a, Q b);
Q qconj(Q a);
Q qnorm(Q a);
V3 qrot(Q q, V3 v);
Q qexp(V3 w);
/// Rotation vector of q (shortest way).
V3 qlog(Q q);
double qangle(Q a, Q b);
double det3(const M3& m);
std::optional<M3> inv3(const M3& m);
M3 polar(const M3& m);
V3 mv(const M3& m, V3 v);
/// Yaw-only part of a head orientation (about +Y, OpenXR axes).
Q yaw_only(Q q);

/// Rigid pose (orientation x y z w, position).
struct P {
    Q q{0.0, 0.0, 0.0, 1.0};
    V3 p{0.0, 0.0, 0.0};
    static P id() { return P{}; }
    P mul(const P& o) const;
    P inv() const;
    bool operator==(const P&) const = default;
};

// ---------- report decoding ----------
struct Gyro {
    std::array<std::uint32_t, 2> ts;
    std::array<std::array<std::int16_t, 3>, 2> g;
    std::size_t n;
};
/// Gyro samples of one raw controller report: device us, raw gyro, 1 or 2 samples (the second, bytes 30..41, is all
/// zero when the report holds only one; it is taken as 2 ms after the first).
Gyro report_gyro(const std::uint8_t* raw64);
/// Accelerometer of the same samples (bytes 16..21 and 30..35, big-endian, about 1024 LSB/g).
std::array<std::array<std::int16_t, 3>, 2> report_accel(const std::uint8_t* raw64);

// ---------- calibration ----------
inline constexpr double ACCEL_LSB_PER_G = 1024.0;
inline constexpr double ACCEL_G = 9.80665;

struct Calib {
    /// raw gyro LSB -> aim-frame rad/s.
    M3 c{};
    /// Gyro bias in LSB.
    V3 bias{0.0, 0.0, 0.0};
    /// IMU time = optical time + offset (ms) for the best fit.
    double time_offset_ms = 0.0;
    std::uint32_t pairs = 0;
    std::uint32_t moving = 0;
    double r2 = 0.0;

    double scale_dps_per_lsb() const;
    /// Nearest signed axis permutation, e.g. "+x -z +y".
    std::string axes() const;
    /// Rotation error of the axis mapping vs the nearest signed permutation (deg).
    double axis_misalignment_deg() const;
    V3 omega(V3 raw) const;
    M3 axis_rotation() const;
    /// Accelerometer sample (raw LSB) as specific force in the aim frame (m/s^2).
    V3 accel(V3 raw) const;
    bool operator==(const Calib&) const = default;
};

/// Time offsets tried by the fit (IMU interval = optical interval + offset).
inline constexpr std::array<double, 9> FIT_OFFSETS_MS = {-64.0, -48.0, -32.0, -16.0, 0.0, 16.0, 32.0, 48.0, 64.0};
inline constexpr std::size_t N_OFF = FIT_OFFSETS_MS.size();

/// Least-squares accumulator for y = C x + d.
struct Acc {
    std::array<std::array<double, 4>, 4> a{};
    std::array<std::array<double, 3>, 4> b{};
    double yy = 0.0;
    V3 ysum{0.0, 0.0, 0.0};
    std::uint32_t n = 0;
    std::uint32_t moving = 0;
    void add(V3 x, V3 y);
    struct Sol {
        M3 c;
        V3 d;
        double r2;
    };
    /// (C, d, r2) or nullopt when singular.
    std::optional<Sol> solve() const;
};

/// The online fit: the pairs (mean raw gyro per time offset, optical body angular velocity), newest last.
struct Fit {
    std::deque<std::pair<std::array<V3, N_OFF>, V3>> pairs;
    std::uint32_t npairs() const { return static_cast<std::uint32_t>(pairs.size()); }
    void add(const std::array<V3, N_OFF>& xs, V3 y);
    struct Robust {
        M3 c;
        V3 d;
        double r2;
        std::uint32_t kept;
        std::uint32_t moving;
    };
    /// Robust fit for one time offset (least squares, then four rounds dropping residuals above the 85th
    /// percentile, at least 0.3 rad/s).
    std::optional<Robust> robust(std::size_t k) const;
    /// Best calibration over the time offsets, if the data supports one.
    std::optional<Calib> best(std::uint32_t min_pairs, std::uint32_t min_moving, double min_r2) const;
};

/// Pairs kept for the fit (about 10 min of tracking).
inline constexpr std::size_t FIT_MAX_PAIRS = 3000;

/// Built-in calibration used when the file has no entry for a hand.
Calib default_calib(std::size_t hand);
/// The saved calibration of `hand`, else the built-in one (unless PIMAX_CTRL_IMU_DEFAULT=0).
std::optional<Calib> calib_or_default(const std::string& path, std::size_t hand);
/// PIMAX_CTRL_IMU_CALIB, else $HOME/.config/pibridge/ctrl-imu.json.
std::string calib_path();
/// Read one hand's calibration from the JSON file.
std::optional<Calib> load_calib(const std::string& path, std::size_t hand);
/// Write one hand's calibration, keeping the other hand's entry (atomic rename). false on an I/O error.
bool save_calib(const std::string& path, std::size_t hand, const Calib& c);
/// The numbers in a text (split on ',', '[', ']' and Unicode whitespace; unparsable tokens skipped).
std::vector<double> nums(std::string_view s);
/// Whether a new online fit replaces the kept one (never a worse one, 0.02 slack).
bool fit_replaces(std::optional<double> old_r2, double new_r2);

// ---------- the fuser ----------
inline constexpr V3 FOREARM_LEFT = {0.01, 0.085, -0.09};

struct Config {
    double extrap_s = 0.22;
    double extrap_tau = 0.06;
    double max_vel = 1.5;
    double lost_s = 4.0;
    double blend_s = 0.15;
    double gap_s = 0.22;
    double max_jump_deg = 35.0;
    double max_jump_m = 0.15;
    std::optional<bool> use_imu;
    bool online_fit = true;
    V3 shoulder = {-0.18, -0.25, 0.10};
    double reach = 0.75;
    double cross = 0.30;
    double min_sep = 0.08;
    bool filter = true;
    bool still_lock = true;
    std::optional<V3> forearm = FOREARM_LEFT;
    double dr_tau_s = 0.0;
    double dr_max_m = 0.25;
    double blend_mps = 0.5;
    double blend_dps = 120.0;
    double blend_max_s = 0.5;
    bool blend_ease = false;
    double follow_yaw = 0.5;
    double follow_pos = 1.0;
    bool hyst = true;
    std::uint32_t confirm_n = 2;
    double confirm_s = 0.15;
    double confirm_m = 0.1;

    // ---- PiBridge's improvements (docs/research/ctrl-tracking-improvements.md); the defaults here are the Rust
    // behaviour, Config::pibridge() switches them on, PIBRIDGE_CTRL_PARITY=1 keeps them off. ----
    /// IMU position (inertial navigation) for this long after the last fix (s); 0 = off. A per-axis Kalman filter
    /// (position, velocity, world-frame accelerometer bias) runs on the accelerometer rotated by the gyro
    /// orientation and is corrected by every optical fix (the bias it learns also absorbs the accelerometer scale
    /// error on gravity); out of view it carries the hand on, the velocity damped after `ins_free_s`, the distance
    /// from the last fix capped at `ins_max_m`; then the arm-model hold takes over from where it ended.
    double ins_s = 0.0;
    double ins_free_s = 0.25;
    double ins_tau_v = 0.3;
    double ins_max_m = 0.35;
    /// A fix farther than this (m, + 1 m/s x age, within 0.2 s of the last fix) from the inertial prediction is a
    /// "jump" (dropped up to three times in a row, as the optical jump test). 0 = off.
    double ins_jump_m = 0.0;
    /// Out of view, pull the gyro orientation's pitch and roll towards the accelerometer's gravity (rate 1/s; only
    /// when the accelerometer reads about 1 g): the gyro bias left over no longer tilts a held controller.
    bool tilt_fix = false;
    /// Keep a held controller after lost_s while its gyro streams (Hold instead of None: no snap to the rest pose);
    /// the hold glides towards the rest pose (`rest_pos`, head-yaw frame, x mirrored for the left hand) with this
    /// time constant (s) from lost_s on. 0 = off (Rust: None after lost_s).
    double keep_tau_s = 0.0;
    V3 rest_pos = {0.15, -0.16, -0.38};
    /// Drop an optical fix whose gravity direction disagrees with this controller's accelerometer by more than this
    /// (deg; only while the accelerometer reads about 1 g, with a fitted or saved gyro calibration): a fix of the other
    /// hand or with the wrong LED identities. It switches itself off when it would drop most fixes (a wrong axis
    /// calibration). 0 = off.
    double grav_check_deg = 0.0;
    /// Write the shown pose, head-relative, to this hint file for the optical tracker (pibridge/ctrl/hint.h); empty =
    /// off. Config::from_env() sets the default path unless PIBRIDGE_CTRL_PARITY=1 or PIBRIDGE_CTRL_HINT=0.
    std::string hint_path;
    /// The shown position is the last fix as measured plus the inertial displacement since (not the display
    /// filter's position): no lag behind a hand that stops hard. Needs ins_s > 0.
    bool raw_anchor = false;
    /// A few-LED fix whose orientation turned against the gyro is dropped (Rust: its position is kept, "pos-only").
    bool turn_drop = false;
    /// Keep the hand-identity state of imu_state() up to date also without a hint file (the offline bench).
    bool imu_state = false;

    /// PiBridge's defaults: the Rust ones (Config{}) plus the improvements of docs/research/ctrl-tracking-improvements.md.
    static Config pibridge();
    /// pibridge() (Config{} with PIBRIDGE_CTRL_PARITY=1) plus the PIMAX_CTRL_TRACK_* / PIMAX_CTRL_IMU_FIT environment.
    static Config from_env();
    /// The pre-2026-10-17 optical window: 80 ms, 100 ms gap, no re-entry confirmation, step weighting.
    void set_legacy_window();
};

/// What Fuser::get returned.
enum class State : std::uint32_t { None = 0, Optical = 1, Hold = 2 };

struct Stats {
    std::uint64_t imu_samples = 0;
    std::uint64_t fixes = 0;
    std::uint64_t rejected = 0;
    std::uint64_t rejected_body = 0;
    std::uint64_t reacquires = 0;
    std::uint64_t fit_updates = 0;
    /// PiBridge: fixes dropped by the gravity check.
    std::uint64_t rejected_tilt = 0;
};

class Fuser {
public:
    Fuser(std::size_t hand, Config cfg, std::optional<Calib> calib);
    ~Fuser();
    Fuser(const Fuser&) = delete;
    Fuser& operator=(const Fuser&) = delete;

    /// Log every gyro sample and optical fix to `path` for `minutes`. false when the file cannot be created.
    bool start_log(const std::string& path, double minutes);
    bool gyro_active() const;
    /// One raw controller report received at host_ns.
    void report(std::uint64_t host_ns, const std::uint8_t* raw64);
    /// One gyro sample: device us stamp, raw LSB.
    void sample(std::uint64_t host_ns, std::uint32_t dev_ts, std::array<std::int16_t, 3> g);
    void sample_acc(std::uint64_t host_ns, std::uint32_t dev_ts, std::array<std::int16_t, 3> g,
                    std::optional<std::array<std::int16_t, 3>> acc);
    /// This hand's shoulder in the world, from the head pose (position + yaw).
    V3 shoulder(const P& head) const;
    /// Why a world position cannot be this hand (nullptr = plausible).
    const char* implausible(V3 p, const P& head) const;
    /// Position of the last fix if it is at most max_age_s older than t.
    std::optional<V3> fresh_fix(std::uint64_t t, double max_age_s) const;
    /// Unconstrained prediction of the position at t (for the other hand's separation rule).
    std::optional<V3> peek(std::uint64_t t, const P& head);
    /// Seconds since the last accepted fix at t (infinity without one).
    double last_fix_age(std::uint64_t t) const;
    /// The best estimate of the true pose at t for the tracker (not the smoothed display pose): the last fix as
    /// measured, turned by the gyro since, moved by the inertial filter's displacement since. nullopt without a fix
    /// or once get() would give up.
    std::optional<P> hint_pose(std::uint64_t t) const;
    /// What the IMU says at t, for the tracker's hand identity (written into the hint file).
    struct ImuState {
        /// Gyro-integrated orientation (aim frame, arbitrary origin): the rotation between two times is
        /// conj(q0) * q1, in the aim frame of the first.
        Q q;
        /// Gravity "up" in the aim frame (unit, low-passed accelerometer); up_ok when it reads about 1 g.
        V3 up;
        bool up_ok;
        /// Seconds the gyro has been quiet (under 0.05 rad/s smoothed over 0.2 s); 0 while it moves.
        double still_s;
        /// A fitted or saved gyro calibration (the axes can be trusted).
        bool calibrated;
    };
    std::optional<ImuState> imu_state(std::uint64_t t) const;
    /// The gyro says the controller is not rotating.
    bool gyro_still(std::uint64_t t) const;
    static double gain(std::uint32_t leds, float rms_px, bool smooth);
    std::pair<State, P> get(std::uint64_t t, const P& head) { return get_with(t, head, std::nullopt); }
    std::pair<State, P> get_with(std::uint64_t t, const P& head, std::optional<V3> other);
    void optical(std::uint64_t t, P w, const P& head, float rms_px, std::uint32_t leds) {
        optical_with(t, w, head, rms_px, leds, std::nullopt);
    }
    /// The verdict: "ok", "pos-only", "confirm" or why the fix was dropped.
    const char* optical_with(std::uint64_t t, P w, const P& head, float rms_px, std::uint32_t leds,
                             std::optional<V3> other);
    /// Save the online fit if it changed and the last save is older than min_interval_s. nullopt = nothing done,
    /// else whether the save worked.
    std::optional<bool> maybe_save(std::uint64_t now, const std::string& path, double min_interval_s);
    void flush_log();

    std::size_t hand;
    Config cfg;
    std::optional<Calib> calib;
    Fit fit;
    Stats stats;

private:
    struct Fix {
        std::uint64_t t;
        P w;
        std::uint32_t leds;
        std::uint32_t seg;
    };
    struct Dr {
        std::uint64_t fix_t;
        V3 g;
        std::uint64_t t;
        V3 v;
        V3 d;
        std::optional<V3> d_anchor;
    };
    struct Blend {
        std::optional<std::uint64_t> t0;
        double dur;
        V3 dp;
        Q dq;
    };
    struct Log;

    void log_line(std::uint64_t t, const std::string& line);
    std::optional<Calib> active_calib() const;
    void update_rest_bias();
    std::optional<Q> q_at(std::uint64_t t) const;
    std::optional<Q> gyro_delta(std::uint64_t t0, std::uint64_t t1) const;
    V3 constrain(V3 p, const P& head, std::optional<V3> other) const;
    P filter_update(const Fix& f, std::uint64_t t, P w, std::uint32_t leds, float rms_px);
    std::pair<State, P> predict(std::uint64_t t, const P& head);
    std::optional<V3> dead_reckon(const Fix& f, std::uint64_t t);
    double blend_weight(double a, double dur) const;
    bool needs_confirm(std::uint64_t t, const P& w);
    const char* judge(std::uint64_t t, const P& w, const P& head, std::optional<V3> other);
    void accept(std::uint64_t t, P w, const P& head, std::uint32_t leds, float rms_px, bool pos_only);
    void fit_pair(const Fix& b);
    void process_pending();
    void fit_eval(Fix a, Fix b);

    bool calib_from_file_;
    std::optional<std::uint32_t> dev_last_;
    std::uint64_t dev_us_ = 0;
    std::optional<std::int64_t> clk_off_;
    std::uint64_t clk_t_ = 0;
    std::deque<std::pair<std::uint64_t, V3>> raw_;
    std::deque<std::pair<std::uint64_t, V3>> acc_;
    std::optional<Dr> dr_;
    std::deque<std::pair<std::uint64_t, Q>> q_ring_;
    Q q_imu_{0.0, 0.0, 0.0, 1.0};
    std::uint32_t rest_n_ = 0;
    V3 rest_sum_{0.0, 0.0, 0.0};
    std::optional<Fix> last_;
    std::deque<Fix> hist_;
    std::uint32_t seg_ = 0;
    V3 vel_{0.0, 0.0, 0.0};
    std::optional<std::pair<P, bool>> hold_;
    std::optional<P> hold_yaw_;
    std::optional<Blend> blend_;
    std::uint32_t rejects_ = 0;
    std::deque<std::pair<Fix, Fix>> pending_;
    bool fit_dirty_ = false;
    std::uint64_t last_save_ = 0;
    Log* log_ = nullptr;
    std::uint64_t last_p_log_ = 0;
    std::uint64_t last_query_t_ = 0;
    std::uint32_t last_leds_ = 0;
    struct Reentry {
        std::uint64_t t;
        V3 p;
        std::uint32_t n;
    };
    std::optional<Reentry> reentry_;

    // PiBridge: the inertial position filter (Config::ins_s), one Kalman filter per world axis: x = (p, v, b).
    struct Kf {
        std::array<double, 3> x{0.0, 0.0, 0.0};
        std::array<std::array<double, 3>, 3> p{};
    };
    struct Ins {
        bool on = false;
        std::uint64_t t = 0;
        std::array<Kf, 3> ax{};
        std::deque<std::pair<std::uint64_t, V3>> hist;
        /// The fix the filter last saw (its time), the position it handed over to the hold.
        std::uint64_t fix_t = 0;
        std::optional<V3> handover;
        /// Gyro rate, smoothed over 0.1 s (rad/s): the zero-velocity test.
        double w_avg = 1.0;
    };
    Ins ins_{};
    /// PiBridge: tilt correction of the held orientation (world-frame rotation applied on top of the gyro).
    Q tilt_{0.0, 0.0, 0.0, 1.0};
    std::uint64_t tilt_t_ = 0;
    void ins_sample(std::uint64_t t, double dt, V3 acc_raw);
    void ins_fix(std::uint64_t t, V3 p, std::uint32_t leds, bool gap);
    std::optional<V3> ins_pos(std::uint64_t t) const;
    std::optional<Q> world_q(std::uint64_t t) const;
    /// Gyro orientation in the world (no tilt correction), re-anchored past 1.5 s out of view (Config::keep_tau_s).
    std::optional<Q> gyro_world(std::uint64_t t) const;
    struct Anchor {
        std::uint64_t fix_t;
        std::uint64_t t;
        Q q;
    };
    std::optional<Anchor> qa_;
    std::uint64_t hint_t_ = 0;
    V3 acc_lp_{0.0, 0.0, 0.0};
    /// Accelerometer sum since the last hint entry (aim frame, m/s^2), and the count.
    V3 hint_acc_{0.0, 0.0, 0.0};
    std::uint32_t hint_acc_n_ = 0;
    double w_lp_ = 1.0;
    std::uint64_t still_since_ = 0;
    bool still_now_ = false;
    /// The last accepted fix as measured (before the display filter): the hint's anchor.
    std::optional<P> last_raw_;
    void write_hint(std::uint64_t t, const P& head, State s, const P& p);
    std::pair<State, P> get_inner(std::uint64_t t, const P& head, std::optional<V3> other);
    /// Gravity check of the last fixes (Config::grav_check_deg): (checked, failed) over a window.
    std::uint32_t grav_n_ = 0;
    std::uint32_t grav_bad_ = 0;
    bool grav_off_ = false;
    /// The angle (deg) between the fix's gravity and the accelerometer's at t, when it can be told.
    std::optional<double> grav_angle(std::uint64_t t, const Q& q) const;
};

} // namespace pibridge::ctrl::fuse
