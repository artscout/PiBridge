// SPDX-License-Identifier: MIT
//
// pibridge-slam headreplay (headreplay.h): the head pose output layer on a recording, with its numbers.
#include "headreplay.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/headpose.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/json.h"
#include "pibridge/cam/slam.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <format>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <vector>

namespace pibridge::slamtool {

namespace {

namespace geom = pibridge::cam::geom;
namespace slam = pibridge::cam::slam;
namespace hp = pibridge::cam::headpose;
namespace json = pibridge::cam::json;
using geom::Vec3;
using slam::Quat;
using slam::qconj;
using slam::qmul;
using slam::qnorm;
using slam::qrot;

constexpr double PI = 3.14159265358979323846;
constexpr double DEG = 180.0 / PI;

struct ImuRow {
    std::int64_t ts;
    Vec3 g, a;
};

struct PoseRow {
    std::int64_t ts;
    Vec3 p;
    Quat q;
};

struct Step {
    int index = 0;
    std::string id, kind;
    std::int64_t from = 0, to = 0; // still part (after the settle time) .. end
};

std::string find_mav(const std::string& d) {
    namespace fs = std::filesystem;
    if (fs::exists(fs::path(d) / "mav0")) {
        return d;
    }
    std::string best;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(d, ec)) {
        if (e.is_directory() && fs::exists(e.path() / "mav0") && e.path().filename().string() > best) {
            best = e.path().filename().string();
        }
    }
    return best.empty() ? d : (fs::path(d) / best).string();
}

std::vector<ImuRow> load_imu(const std::string& root) {
    std::vector<ImuRow> v;
    std::ifstream f(root + "/mav0/imu0/data.csv");
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        ImuRow r{};
        long long ts = 0;
        if (std::sscanf(line.c_str(), "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &ts, &r.g[0], &r.g[1], &r.g[2], &r.a[0], &r.a[1],
                        &r.a[2]) == 7) {
            r.ts = ts;
            v.push_back(r);
        }
    }
    return v;
}

std::vector<PoseRow> load_traj(const std::string& path) {
    std::vector<PoseRow> v;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        PoseRow r{};
        long long ts = 0;
        double w = 0;
        if (std::sscanf(line.c_str(), "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &ts, &r.p[0], &r.p[1], &r.p[2], &w, &r.q[0],
                        &r.q[1], &r.q[2]) == 8) {
            r.ts = ts;
            r.q[3] = w;
            v.push_back(r);
        }
    }
    return v;
}

std::vector<Step> load_steps(const std::string& path) {
    std::vector<Step> v;
    if (path.empty()) {
        return v;
    }
    const auto txt = pibridge::cam::read_to_string(path);
    if (!txt) {
        return v;
    }
    const auto doc = json::parse(*txt);
    if (!doc) {
        return v;
    }
    const json::Value* st = doc->get("steps");
    if (st == nullptr || st->as_array() == nullptr) {
        return v;
    }
    for (const auto& s : *st->as_array()) {
        Step x;
        auto num = [&](const char* k) -> double {
            const json::Value* e = s.get(k);
            return e != nullptr && e->as_f64() ? *e->as_f64() : 0.0;
        };
        auto str = [&](const char* k) -> std::string {
            const json::Value* e = s.get(k);
            return e != nullptr && e->as_str() != nullptr ? *e->as_str() : std::string();
        };
        x.index = static_cast<int>(num("index"));
        x.id = str("id");
        x.kind = str("kind");
        const double still = num("still_from_ns");
        x.from = static_cast<std::int64_t>(still > 0.0 ? still : num("start_ns"));
        x.to = static_cast<std::int64_t>(num("end_ns"));
        if (const json::Value* sk = s.get("skipped"); sk != nullptr && sk->type == json::Value::Type::Bool && sk->b) {
            continue;
        }
        v.push_back(x);
    }
    return v;
}

Vec3 v3(double x, double y, double z) {
    return {x, y, z};
}

double ang_deg(const Quat& a, const Quat& b) {
    return geom::norm(hp::qlog(qmul(a, qconj(b)))) * DEG;
}

double pct(std::vector<double> v, double p) {
    if (v.empty()) {
        return std::nan("");
    }
    std::sort(v.begin(), v.end());
    const std::size_t i = std::min(v.size() - 1, static_cast<std::size_t>(p * static_cast<double>(v.size() - 1) + 0.5));
    return v[i];
}

double rms(const std::vector<double>& v) {
    if (v.empty()) {
        return std::nan("");
    }
    double s = 0;
    for (double x : v) {
        s += x * x;
    }
    return std::sqrt(s / static_cast<double>(v.size()));
}

// Least-squares slope (per second) of each axis over (t, p): the creep of a window.
double creep_mm_s(const std::vector<std::pair<std::int64_t, Vec3>>& w) {
    if (w.size() < 10) {
        return std::nan("");
    }
    const double t0 = static_cast<double>(w.front().first);
    double st = 0, stt = 0;
    Vec3 sp{}, stp{};
    for (const auto& [t, p] : w) {
        const double x = (static_cast<double>(t) - t0) / 1e9;
        st += x;
        stt += x * x;
        for (int k = 0; k < 3; k++) {
            sp[k] += p[k];
            stp[k] += x * p[k];
        }
    }
    const double n = static_cast<double>(w.size());
    const double den = n * stt - st * st;
    if (den <= 0) {
        return std::nan("");
    }
    Vec3 slope{};
    for (int k = 0; k < 3; k++) {
        slope[k] = (n * stp[k] - st * sp[k]) / den;
    }
    return geom::norm(slope) * 1000.0;
}

Vec3 mean_pos(const std::vector<std::pair<std::int64_t, Vec3>>& w) {
    Vec3 m{};
    for (const auto& e : w) {
        m = geom::add(m, e.second);
    }
    return w.empty() ? m : geom::scale(m, 1.0 / static_cast<double>(w.size()));
}

// One fed tracker pose (after the reset emulation's rebase), for the non-causal reference.
struct Fed {
    std::int64_t ts;
    Vec3 p;
    Quat q;
    int seg;
};

// One head query.
struct Query {
    std::int64_t now, at;
    hp::Rel out;
    slam::TrackState state;
    bool still, frozen;
    Vec3 wt;
    Quat wq, tilt;
    double held_m;
};

class Gyro {
public:
    Gyro(const std::vector<ImuRow>& imu, Vec3 bias) : imu_(imu), bias_(bias) {}
    // Bias-corrected rotation from t0 to t1 (zero-order hold, as the predictor integrates).
    Quat delta(std::int64_t t0, std::int64_t t1) const {
        Quat q = slam::QUAT_ID;
        auto it = std::upper_bound(imu_.begin(), imu_.end(), t0, [](std::int64_t t, const ImuRow& r) { return t < r.ts; });
        std::int64_t t = t0;
        for (; it != imu_.end() && t < t1; ++it) {
            const std::int64_t end = std::min(it->ts, t1);
            if (end > t) {
                q = qmul(q, hp::qexp(geom::scale(geom::sub(it->g, bias_), static_cast<double>(end - t) / 1e9)));
                t = end;
            }
        }
        return qnorm(q);
    }
    // Bias-corrected rate at t (the sample covering t).
    Vec3 rate(std::int64_t t) const {
        auto it = std::lower_bound(imu_.begin(), imu_.end(), t, [](const ImuRow& r, std::int64_t x) { return r.ts < x; });
        if (it == imu_.end()) {
            return {0, 0, 0};
        }
        return geom::sub(it->g, bias_);
    }

private:
    const std::vector<ImuRow>& imu_;
    Vec3 bias_;
};

// Reference IMU pose at t in the fed (rebased, y-up) tracker world, from the fed poses around it.
std::optional<std::pair<Vec3, Quat>> reference(const std::vector<Fed>& fed, const Gyro& gy, std::int64_t t) {
    auto it = std::upper_bound(fed.begin(), fed.end(), t, [](std::int64_t x, const Fed& f) { return x < f.ts; });
    if (it == fed.begin() || it == fed.end()) {
        return std::nullopt;
    }
    const std::size_t k1 = static_cast<std::size_t>(it - fed.begin());
    const std::size_t k0 = k1 - 1;
    const Fed& a = fed[k0];
    const Fed& b = fed[k1];
    if (a.seg != b.seg || b.ts - a.ts > 200'000'000) {
        return std::nullopt;
    }
    auto vel = [&](std::size_t k) -> Vec3 {
        const std::size_t lo = (k > 0 && fed[k - 1].seg == fed[k].seg) ? k - 1 : k;
        const std::size_t hi = (k + 1 < fed.size() && fed[k + 1].seg == fed[k].seg) ? k + 1 : k;
        if (hi == lo) {
            return {0, 0, 0};
        }
        return geom::scale(geom::sub(fed[hi].p, fed[lo].p), 1e9 / static_cast<double>(fed[hi].ts - fed[lo].ts));
    };
    const double h = static_cast<double>(b.ts - a.ts) / 1e9;
    const double s = static_cast<double>(t - a.ts) / 1e9 / h;
    const double h00 = 2 * s * s * s - 3 * s * s + 1, h10 = s * s * s - 2 * s * s + s, h01 = -2 * s * s * s + 3 * s * s,
                 h11 = s * s * s - s * s;
    const Vec3 va = vel(k0), vb = vel(k1);
    Vec3 p{};
    for (int k = 0; k < 3; k++) {
        p[k] = h00 * a.p[k] + h10 * h * va[k] + h01 * b.p[k] + h11 * h * vb[k];
    }
    // Orientation: gyro from a, the mismatch at b spread linearly (in a's body frame, right-multiplied).
    const Quat da = gy.delta(a.ts, t);
    const Quat dab = gy.delta(a.ts, b.ts);
    const Quat m = qmul(qconj(qmul(qnorm(a.q), dab)), qnorm(b.q));
    const Vec3 lm = hp::qlog(m);
    // The mismatch m sits at the end (q_b = q_a dab m); spread as q(t) = q_a da exp(s * log m) (small m: order does
    // not matter at the size of the mismatch).
    const Quat q = qnorm(qmul(qmul(qnorm(a.q), da), hp::qexp(geom::scale(lm, s))));
    return std::pair<Vec3, Quat>{p, q};
}

std::string fmt(double v, int d = 1) {
    if (!std::isfinite(v)) {
        return "-";
    }
    return std::format("{:.{}f}", v, d);
}

} // namespace

std::expected<std::string, std::string> head_replay(const HeadReplayOpts& o, PoseSource* src) {
    const std::string root = find_mav(o.dataset);
    std::vector<ImuRow> imu = load_imu(root);
    std::vector<PoseRow> vio = src != nullptr ? std::vector<PoseRow>{} : load_traj(o.traj);
    if (imu.size() < 1000) {
        return std::unexpected(std::format("{}: no IMU (mav0/imu0/data.csv)", root));
    }
    if (src == nullptr && vio.size() < 50) {
        return std::unexpected(std::format("{}: no tracker trajectory", o.traj));
    }
    const std::vector<Step> steps = load_steps(o.steps);

    // Calibration the tracker used: the accel bias for the tilt reference, the gyro bias for the 3DoF filter.
    Vec3 accel_bias{0, 0, 0};
    std::optional<Vec3> gyro3;
    if (!o.calib.empty()) {
        const auto txt = pibridge::cam::read_to_string(o.calib);
        if (!txt) {
            return std::unexpected(std::format("{}: cannot read", o.calib));
        }
        const auto doc = json::parse(*txt);
        if (!doc || doc->get("value0") == nullptr) {
            return std::unexpected(std::format("{}: not a Basalt calibration", o.calib));
        }
        const json::Value* v0 = doc->get("value0");
        if (const json::Value* ab = v0->get("calib_accel_bias"); ab != nullptr && ab->as_array() != nullptr &&
                                                                  ab->as_array()->size() >= 3) {
            for (int k = 0; k < 3; k++) {
                accel_bias[k] = (*ab->as_array())[static_cast<std::size_t>(k)].as_f64().value_or(0.0);
            }
        }
        if (const json::Value* gb = v0->get("calib_gyro_bias"); gb != nullptr && gb->as_array() != nullptr &&
                                                                 gb->as_array()->size() >= 3) {
            Vec3 g{};
            for (int k = 0; k < 3; k++) {
                g[k] = (*gb->as_array())[static_cast<std::size_t>(k)].as_f64().value_or(0.0);
            }
            gyro3 = g;
        }
    }
    if (!gyro3) {
        gyro3 = slam::gyro_bias_cache().value_or(Vec3{0, 0, 0});
    }

    Vec3 head = v3(0.049, -0.015, 0.030);
    std::string head_from = "difftest default";
    if (o.head) {
        head = *o.head;
        head_from = "--head";
    } else if (auto c = pibridge::cam::DeviceCalib::load_default(std::nullopt)) {
        head = slam::default_head_in_imu(c->first);
        head_from = c->first.is_reference() ? "reference calibration" : "factory calibration";
    }

    const Quat to_yup{-std::sqrt(0.5), 0, 0, std::sqrt(0.5)};
    hp::HeadParams prm = hp::HeadParams::from_env();
    const std::string describe = prm.describe();
    hp::HeadTracker t(slam::SupervisorParams{}, prm, head);
    const Gyro gy(imu, *gyro3);

    // 3DoF: Mahony from the bias-corrected gyro and gravity (the difftest's filter, with the driver's bias).
    Quat fus = slam::QUAT_ID;
    bool fus_init = false;
    std::vector<std::pair<std::int64_t, Quat>> qh;
    qh.reserve(imu.size());

    std::vector<Fed> fed;
    std::vector<Query> qs;
    qs.reserve(static_cast<std::size_t>(static_cast<double>(imu.back().ts - imu.front().ts) / 1e9 * o.query_hz) + 16);
    const std::int64_t lat = static_cast<std::int64_t>(o.latency_ms * 1e6);
    const std::int64_t pred = static_cast<std::int64_t>(o.predict_ms * 1e6);
    const std::int64_t q_every = static_cast<std::int64_t>(1e9 / o.query_hz);

    bool pending_reset = false, rebased = false;
    int seg = 0;
    std::int64_t rb_t = 0;
    Vec3 rb_p{}, rb_v{};
    Quat rb_r = slam::QUAT_ID, w0 = slam::QUAT_ID;
    bool have_w0 = false;
    int skip = 0;
    std::size_t k = 0;
    std::int64_t next_q = imu.front().ts + 1'000'000'000;
    std::vector<Vec3> raw_p(vio.size());
    for (std::size_t i = 0; i < vio.size(); i++) {
        raw_p[i] = qrot(to_yup, vio[i].p);
    }
    std::uint64_t n_updates = 0;
    double busy_s = 0;
    using clk = std::chrono::steady_clock;
    std::uint64_t rezero_events = 0, reset_events = 0;
    std::vector<std::pair<std::int64_t, Vec3>> ext_hist; // a live source's last positions (velocity fallback)
    std::string events_log;
    auto note_events = [&](const slam::TrackOutput& out, std::int64_t now) {
        if ((out.events & slam::events::RESET_WANTED) != 0) {
            pending_reset = true;
            reset_events++;
            if ((out.events & slam::events::REZEROED) != 0) {
                rezero_events++;
            }
        }
        if ((out.events & (slam::events::LOST | slam::events::ACQUIRED)) != 0) {
            events_log += std::format("  {:7.2f} s {}{}: {}\n", static_cast<double>(now - imu.front().ts) / 1e9,
                                      (out.events & slam::events::LOST) != 0 ? "lost" : "",
                                      (out.events & slam::events::ACQUIRED) != 0 ? "acquired" : "", out.reason);
        }
    };

    for (std::size_t i = 0; i < imu.size(); i++) {
        const ImuRow& s = imu[i];
        double dt = i == 0 ? 0.001 : static_cast<double>(s.ts - imu[i - 1].ts) / 1e9;
        dt = std::clamp(dt, 1e-4, 0.01);
        const double an = geom::norm(s.a);
        if (!fus_init) {
            if (an >= 1.0) {
                const Vec3 u = geom::scale(s.a, 1.0 / an);
                fus = qnorm(Quat{-u[2], 0.0, u[0], 1.0 + u[1]});
                fus_init = true;
            }
        } else {
            Vec3 w = geom::sub(s.g, *gyro3);
            if (std::fabs(an - 9.81) < 1.0) {
                const Vec3 up = qrot(qconj(fus), v3(0, 1, 0));
                const Vec3 e = geom::cross(geom::scale(s.a, 1.0 / an), up);
                w = geom::add(w, geom::scale(e, 0.5));
            }
            fus = qnorm(qmul(fus, Quat{w[0] * 0.5 * dt, w[1] * 0.5 * dt, w[2] * 0.5 * dt, 1.0}));
        }
        qh.emplace_back(s.ts, fus);
        if (src != nullptr) {
            src->on_imu(s.ts, s.a, s.g);
        }
        auto c0 = clk::now();
        t.push_imu(hp::ImuSample{s.ts, s.a, s.g});
        busy_s += std::chrono::duration<double>(clk::now() - c0).count();

        // One tracker pose (y-up tracker world) to the output layer.
        auto deliver = [&](hp::VioSample sv) {
            Quat q3 = fus;
            for (std::size_t j = qh.size(); j-- > 0 && qh.size() - j <= 2048;) {
                if (qh[j].first <= sv.ts_ns) {
                    q3 = qh[j].second;
                    break;
                }
            }
            sv.q3dof = q3;
            if (!have_w0 && fus_init) {
                w0 = qnorm(qmul(sv.q, qconj(q3)));
                have_w0 = true;
            }
            fed.push_back(Fed{sv.ts_ns, sv.pos, sv.q, seg});
            const auto c1 = clk::now();
            const slam::TrackOutput out = t.update(s.ts, sv);
            busy_s += std::chrono::duration<double>(clk::now() - c1).count();
            n_updates++;
            note_events(out, s.ts);
        };
        if (src != nullptr) {
            if (pending_reset) {
                // a real tracker: reset it; its next poses are a fresh world
                pending_reset = false;
                src->reset(s.ts);
                seg += 1;
                ext_hist.clear();
            }
            for (const TrackerPose& tp : src->poll(s.ts)) {
                hp::VioSample sv;
                sv.ts_ns = tp.ts;
                sv.pos = qrot(to_yup, tp.p);
                sv.q = qnorm(qmul(to_yup, qnorm(tp.q)));
                if (tp.has_vel && !o.central_vel) {
                    sv.vel = qrot(to_yup, tp.v);
                } else if (ext_hist.size() >= 2) {
                    const auto& [t1, p1] = ext_hist[ext_hist.size() - 1];
                    const auto& [t2, p2] = ext_hist[ext_hist.size() - 2];
                    const double d2 = static_cast<double>(tp.ts - t2) / 2e9;
                    if (d2 > 0 && tp.ts > t1) {
                        for (int j = 0; j < 3; j++) {
                            sv.vel[j] = (3.0 * sv.pos[j] - 4.0 * p1[j] + p2[j]) * 0.5 / d2;
                        }
                    }
                }
                ext_hist.emplace_back(tp.ts, sv.pos);
                if (ext_hist.size() > 3) {
                    ext_hist.erase(ext_hist.begin());
                }
                deliver(sv);
                if (pending_reset) {
                    break; // the rest came from the old world before the reset
                }
            }
        }
        while (src == nullptr && k < vio.size() && vio[k].ts + lat <= s.ts) {
            const Vec3 rp = raw_p[k];
            const Quat rq = qnorm(qmul(to_yup, qnorm(vio[k].q)));
            const std::int64_t ts = vio[k].ts;
            Vec3 rv{0, 0, 0};
            if (o.central_vel) {
                // central difference (uses the next pose: a filtered VIO velocity is about this good)
                if (k > 0 && k + 1 < vio.size()) {
                    rv = geom::scale(geom::sub(raw_p[k + 1], raw_p[k - 1]),
                                     1e9 / static_cast<double>(vio[k + 1].ts - vio[k - 1].ts));
                }
            } else if (k >= 2) {
                const double d2 = static_cast<double>(ts - vio[k - 2].ts) / 2e9;
                if (d2 > 0) {
                    for (int j = 0; j < 3; j++) {
                        rv[j] = (3.0 * rp[j] - 4.0 * raw_p[k - 1][j] + raw_p[k - 2][j]) * 0.5 / d2;
                    }
                }
            }
            if (pending_reset) {
                // A fresh tracker world: starts at the current pose with zero velocity, gravity aligned like the
                // first one, another yaw (as the difftest and omarchyvr's stillgate emulate it).
                pending_reset = false;
                const Quat q3r = qh.back().second;
                const Quat d = qnorm(qmul(qmul(have_w0 ? w0 : slam::QUAT_ID, q3r), qconj(rq)));
                const Vec3 f2 = qrot(q3r, v3(0, 0, -1));
                const Quat y = slam::qyaw(std::atan2(-f2[0], -f2[2]) + 0.6);
                rb_r = qnorm(qmul(qconj(y), d));
                rb_t = ts;
                rb_p = rp;
                rb_v = rv;
                rebased = true;
                skip = 2;
                seg += 1;
            }
            k++;
            if (skip > 0) {
                skip--;
                continue;
            }
            hp::VioSample sv;
            sv.ts_ns = ts;
            if (rebased) {
                const double tr = static_cast<double>(ts - rb_t) / 1e9;
                const Vec3 rel = geom::sub(geom::sub(rp, rb_p), geom::scale(rb_v, tr));
                sv.pos = qrot(rb_r, rel);
                sv.q = qnorm(qmul(rb_r, rq));
                sv.vel = qrot(rb_r, geom::sub(rv, rb_v));
            } else {
                sv.pos = rp;
                sv.q = rq;
                sv.vel = rv;
            }
            deliver(sv);
        }
        if (s.ts < next_q) {
            continue;
        }
        next_q += q_every;
        c0 = clk::now();
        const slam::TrackOutput out = t.update(s.ts, std::nullopt);
        note_events(out, s.ts);
        hp::Rel r3;
        r3.q = fus;
        r3.ang_vel = qrot(fus, geom::sub(s.g, *gyro3));
        r3.flags = hp::flags::ORIENTATION_VALID | hp::flags::ORIENTATION_TRACKED | hp::flags::ANGULAR_VELOCITY_VALID;
        const hp::Rel ho = t.head(s.ts, s.ts + pred, r3);
        busy_s += std::chrono::duration<double>(clk::now() - c0).count();
        const auto [wt, wq] = t.sup.world_from_slam();
        qs.push_back(Query{s.ts, s.ts + pred, ho, t.sup.state(), t.is_still(), t.is_frozen(), wt, wq, t.tilt(),
                           t.drift.stats.held_m});
    }

    /*
     * Metrics
     */
    auto in = [](std::int64_t x, const Step& st) { return x >= st.from && x < st.to; };
    std::string r;
    auto line = [&r](const std::string& s) { r += s + "\n"; };
    line(std::format("dataset {}", root));
    if (src != nullptr) {
        line(std::format("tracker {} ({} poses), {} imu, {} queries at {} Hz, predict {} ms", src->describe(),
                         fed.size(), imu.size(), qs.size(), o.query_hz, o.predict_ms));
    } else {
        line(std::format("trajectory {} ({} poses), {} imu, {} queries at {} Hz, latency {} ms, predict {} ms, "
                         "velocity {}",
                         o.traj, vio.size(), imu.size(), qs.size(), o.query_hz, o.latency_ms, o.predict_ms,
                         o.central_vel ? "central" : "backward"));
    }
    line(std::format("head origin ({:.3f} {:.3f} {:.3f}) m from {}", head[0], head[1], head[2], head_from));
    line(describe);
    line(std::format("parity {}", hp::parity_mode() ? "on (exact omarchyvr 5ffbcc5 behaviour)" : "off (PiBridge improvements)"));

    // Per query: reference pose (tracker world) mapped to the output world with the alignment of that moment.
    std::vector<double> e_rot, e_pos, e_rot_slow, e_rot_mid, e_rot_fast, e_pos_slow, e_pos_fast;
    std::vector<std::optional<Vec3>> ref_head_w(qs.size());
    std::vector<std::optional<Quat>> ref_q_w(qs.size());
    std::FILE* csv = o.csv.empty() ? nullptr : std::fopen(o.csv.c_str(), "w");
    if (csv != nullptr) {
        std::fprintf(csv, "now_ns,at_ns,state,still,frozen,px,py,pz,qx,qy,qz,qw,rpx,rpy,rpz,rqx,rqy,rqz,rqw,held_m\n");
    }
    for (std::size_t j = 0; j < qs.size(); j++) {
        const Query& q = qs[j];
        const auto ref = reference(fed, gy, q.at);
        if (ref && q.state == slam::TrackState::Tracking) {
            const Vec3 hs = geom::add(ref->first, qrot(ref->second, head));
            const Vec3 hw = geom::add(qrot(q.wq, hs), q.wt);
            const Quat qw = qnorm(qmul(q.tilt, qmul(q.wq, ref->second)));
            ref_head_w[j] = hw;
            ref_q_w[j] = qw;
            const double er = ang_deg(q.out.q, qw);
            const double ep = geom::norm(geom::sub(q.out.pos, hw)) * 1000.0;
            const double wr = geom::norm(gy.rate(q.at)) * DEG;
            e_rot.push_back(er);
            e_pos.push_back(ep);
            (wr < 30 ? e_rot_slow : wr < 100 ? e_rot_mid : e_rot_fast).push_back(er);
            (wr < 30 ? e_pos_slow : e_pos_fast).push_back(ep);
        }
        if (csv != nullptr) {
            const Vec3 rp = ref_head_w[j].value_or(Vec3{NAN, NAN, NAN});
            const Quat rq = ref_q_w[j].value_or(Quat{NAN, NAN, NAN, NAN});
            std::fprintf(csv, "%lld,%lld,%u,%d,%d,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.4f\n",
                         static_cast<long long>(q.now), static_cast<long long>(q.at), static_cast<unsigned>(q.state),
                         q.still ? 1 : 0, q.frozen ? 1 : 0, q.out.pos[0], q.out.pos[1], q.out.pos[2], q.out.q[0],
                         q.out.q[1], q.out.q[2], q.out.q[3], rp[0], rp[1], rp[2], rq[0], rq[1], rq[2], rq[3], q.held_m);
        }
    }
    if (csv != nullptr) {
        std::fclose(csv);
    }
    line(std::format("prediction vs tracker (output at display time - tracker afterwards): rotation median {} p95 {} "
                     "max {} deg; position median {} p95 {} max {} mm",
                     fmt(pct(e_rot, 0.5), 3), fmt(pct(e_rot, 0.95), 3), fmt(pct(e_rot, 1.0), 2), fmt(pct(e_pos, 0.5)),
                     fmt(pct(e_pos, 0.95)), fmt(pct(e_pos, 1.0))));
    {
        // Dynamic position error: the error minus its own centred 1 s mean (a slowly changing offset between output and
        // tracker is invisible; what is left is lag, overshoot and jitter).
        std::vector<std::size_t> idx;
        std::vector<Vec3> ev;
        for (std::size_t j = 0; j < qs.size(); j++) {
            if (ref_head_w[j]) {
                idx.push_back(j);
                ev.push_back(geom::sub(qs[j].out.pos, *ref_head_w[j]));
            }
        }
        std::vector<double> hp_all, hp_mov;
        const std::int64_t half = 500'000'000;
        std::size_t lo = 0, hi = 0;
        Vec3 sum{};
        for (std::size_t a = 0; a < idx.size(); a++) {
            const std::int64_t tc = qs[idx[a]].at;
            while (hi < idx.size() && qs[idx[hi]].at <= tc + half) {
                sum = geom::add(sum, ev[hi++]);
            }
            while (lo < hi && qs[idx[lo]].at < tc - half) {
                sum = geom::sub(sum, ev[lo++]);
            }
            const Vec3 m = geom::scale(sum, 1.0 / static_cast<double>(hi - lo));
            const double e = geom::norm(geom::sub(ev[a], m)) * 1000.0;
            hp_all.push_back(e);
            if (geom::norm(gy.rate(qs[idx[a]].at)) * DEG >= 30) {
                hp_mov.push_back(e);
            }
        }
        line(std::format("  dynamic position error (1 s offset removed): median {} p95 {} mm; head turning >= 30 deg/s "
                         "p95 {} mm",
                         fmt(pct(hp_all, 0.5), 2), fmt(pct(hp_all, 0.95), 2), fmt(pct(hp_mov, 0.95), 2)));
    }
    line(std::format("  by head turn rate: <30 deg/s rot p95 {} deg pos p95 {} mm (n {}); 30-100 rot p95 {} (n {}); "
                     ">100 rot p95 {} max {} (n {}); >=30 pos p95 {} mm",
                     fmt(pct(e_rot_slow, 0.95), 3), fmt(pct(e_pos_slow, 0.95)), e_rot_slow.size(),
                     fmt(pct(e_rot_mid, 0.95), 3), e_rot_mid.size(), fmt(pct(e_rot_fast, 0.95), 3),
                     fmt(pct(e_rot_fast, 1.0), 2), e_rot_fast.size(), fmt(pct(e_pos_fast, 0.95))));

    // Jitter: 90 Hz output roughness (second difference) and high-pass residual, at rest (still steps) and moving.
    auto rough = [&](const std::function<bool(std::int64_t)>& sel, bool use_ref) {
        std::vector<double> rp, rr, hp_p;
        for (std::size_t j = 1; j + 1 < qs.size(); j++) {
            if (!sel(qs[j].now) || qs[j].state != slam::TrackState::Tracking ||
                qs[j - 1].state != slam::TrackState::Tracking || qs[j + 1].state != slam::TrackState::Tracking) {
                continue;
            }
            Vec3 a, b, c;
            Quat qa, qb, qc;
            if (use_ref) {
                if (!ref_head_w[j - 1] || !ref_head_w[j] || !ref_head_w[j + 1]) {
                    continue;
                }
                a = *ref_head_w[j - 1];
                b = *ref_head_w[j];
                c = *ref_head_w[j + 1];
                qa = *ref_q_w[j - 1];
                qb = *ref_q_w[j];
                qc = *ref_q_w[j + 1];
            } else {
                a = qs[j - 1].out.pos;
                b = qs[j].out.pos;
                c = qs[j + 1].out.pos;
                qa = qs[j - 1].out.q;
                qb = qs[j].out.q;
                qc = qs[j + 1].out.q;
            }
            const Vec3 d2 = geom::add(geom::sub(c, geom::scale(b, 2.0)), a);
            rp.push_back(geom::norm(d2) * 1000.0);
            const Vec3 w1 = hp::qlog(qmul(qb, qconj(qa)));
            const Vec3 w2 = hp::qlog(qmul(qc, qconj(qb)));
            rr.push_back(geom::norm(geom::sub(w2, w1)) * DEG);
        }
        return std::pair{rms(rp), rms(rr)};
    };
    auto at_rest = [&](std::int64_t x) {
        for (const Step& st : steps) {
            if (st.kind == "still" && in(x, st)) {
                return true;
            }
        }
        return false;
    };
    auto moving = [&](std::int64_t x) {
        for (const Step& st : steps) {
            if (st.kind == "move" && in(x, st)) {
                return true;
            }
        }
        return false;
    };
    if (!steps.empty()) {
        const auto [rp_o, rr_o] = rough(at_rest, false);
        const auto [rp_r, rr_r] = rough(at_rest, true);
        const auto [mp_o, mr_o] = rough(moving, false);
        const auto [mp_r, mr_r] = rough(moving, true);
        line(std::format("jitter (90 Hz second difference RMS): at rest output {} mm {} deg (tracker interpolated {} mm "
                         "{} deg); moving output {} mm {} deg (tracker {} mm {} deg)",
                         fmt(rp_o, 3), fmt(rr_o, 4), fmt(rp_r, 3), fmt(rr_r, 4), fmt(mp_o, 3), fmt(mr_o, 4),
                         fmt(mp_r, 3), fmt(mr_r, 4)));
    }

    // Marks, creep and tilt per step.
    std::map<int, Vec3> mark_out, mark_trk;
    std::map<int, std::int64_t> mark_mid;
    std::vector<double> creep_out, creep_trk, tilt_out, tilt_trk, moved_out, moved_trk;
    std::string per_step;
    for (const Step& st : steps) {
        std::vector<std::pair<std::int64_t, Vec3>> wo, wr;
        std::vector<double> to, tr;
        for (std::size_t j = 0; j < qs.size(); j++) {
            const Query& q = qs[j];
            if (!in(q.now, st)) {
                continue;
            }
            wo.emplace_back(q.at, q.out.pos);
            if (ref_head_w[j]) {
                // The tracker's own head position in its world (no alignment: what Basalt alone did).
                const auto ref = reference(fed, gy, q.at);
                if (ref) {
                    wr.emplace_back(q.at, geom::add(ref->first, qrot(ref->second, head)));
                }
            }
            if (st.kind == "still" || st.kind == "tilt-hold") {
                // Tilt: the output's up in the head frame against the accelerometer's (20 ms mean, bias removed).
                auto lo = std::lower_bound(imu.begin(), imu.end(), q.now - 20'000'000,
                                           [](const ImuRow& rr, std::int64_t x) { return rr.ts < x; });
                auto hi = std::upper_bound(imu.begin(), imu.end(), q.now,
                                           [](std::int64_t x, const ImuRow& rr) { return x < rr.ts; });
                Vec3 am{};
                double gmax = 0;
                int n = 0;
                for (auto it = lo; it != hi; ++it, ++n) {
                    am = geom::add(am, geom::sub(it->a, accel_bias));
                    gmax = std::max(gmax, geom::norm(geom::sub(it->g, *gyro3)));
                }
                if (n >= 5) {
                    am = geom::scale(am, 1.0 / n);
                    const double amn = geom::norm(am);
                    if (std::fabs(amn - 9.81) < 0.3 && gmax < 0.15) {
                        const Vec3 up = geom::scale(am, 1.0 / amn);
                        const Vec3 up_o = qrot(qconj(q.out.q), v3(0, 1, 0));
                        to.push_back(std::acos(std::clamp(geom::dot(up, up_o), -1.0, 1.0)) * DEG);
                        if (ref_q_w[j]) {
                            const auto ref = reference(fed, gy, q.now);
                            if (ref) {
                                const Vec3 up_r = qrot(qconj(ref->second), v3(0, 1, 0));
                                tr.push_back(std::acos(std::clamp(geom::dot(up, up_r), -1.0, 1.0)) * DEG);
                            }
                        }
                    }
                }
            }
        }
        tilt_out.insert(tilt_out.end(), to.begin(), to.end());
        tilt_trk.insert(tilt_trk.end(), tr.begin(), tr.end());
        std::string extra;
        if (st.kind == "still") {
            const double co = creep_mm_s(wo), cr = creep_mm_s(wr);
            creep_out.push_back(co);
            creep_trk.push_back(cr);
            auto moved = [](const std::vector<std::pair<std::int64_t, Vec3>>& w) {
                return w.size() < 2 ? std::nan("") : geom::norm(geom::sub(w.back().second, w.front().second)) * 1000.0;
            };
            moved_out.push_back(moved(wo));
            moved_trk.push_back(moved(wr));
            extra = std::format(" creep output {} tracker {} mm/s, moved output {} tracker {} mm", fmt(co), fmt(cr),
                                fmt(moved(wo)), fmt(moved(wr)));
            if (st.id.starts_with("still-mark") || st.id.starts_with("back-on-mark")) {
                mark_out[st.index] = mean_pos(wo);
                mark_trk[st.index] = mean_pos(wr);
                mark_mid[st.index] = (st.from + st.to) / 2;
            }
        }
        if (!to.empty()) {
            extra += std::format(" tilt output median {} tracker {} deg", fmt(pct(to, 0.5), 2), fmt(pct(tr, 0.5), 2));
        }
        per_step += std::format("  step {:2} {:17}{}\n", st.index, st.id, extra);
    }
    if (!steps.empty()) {
        std::string marks;
        std::vector<int> idx;
        for (const auto& [i, p] : mark_out) {
            idx.push_back(i);
        }
        for (std::size_t a = 0; a < idx.size(); a++) {
            for (std::size_t b = a + 1; b < idx.size(); b++) {
                const double eo = geom::norm(geom::sub(mark_out[idx[a]], mark_out[idx[b]])) * 1000.0;
                const double et = geom::norm(geom::sub(mark_trk[idx[a]], mark_trk[idx[b]])) * 1000.0;
                marks += std::format(" {}-{} {} ({})", idx[a], idx[b], fmt(eo, 0), fmt(et, 0));
            }
        }
        line(std::format("mark error output (tracker) mm:{}", marks));
        if (idx.size() >= 2) {
            const double mins = static_cast<double>(mark_mid[idx.back()] - mark_mid[idx.front()]) / 6e10;
            const double eo = geom::norm(geom::sub(mark_out[idx.back()], mark_out[idx.front()])) * 1000.0;
            line(std::format("drift first to last mark: {} mm in {} min = {} mm/min", fmt(eo, 0), fmt(mins, 2),
                             fmt(eo / mins, 0)));
        }
        line(std::format("creep at rest output median {} max {} mm/s (tracker median {} max {}); moved during rests "
                         "output sum {} max {} mm (tracker sum {} max {})",
                         fmt(pct(creep_out, 0.5)), fmt(pct(creep_out, 1.0)), fmt(pct(creep_trk, 0.5)),
                         fmt(pct(creep_trk, 1.0)), fmt(std::accumulate(moved_out.begin(), moved_out.end(), 0.0), 0),
                         fmt(pct(moved_out, 1.0), 0), fmt(std::accumulate(moved_trk.begin(), moved_trk.end(), 0.0), 0),
                         fmt(pct(moved_trk, 1.0), 0)));
        line(std::format("tilt error (still and tilt holds) output median {} p95 {} max {} deg (tracker median {} p95 {})",
                         fmt(pct(tilt_out, 0.5), 2), fmt(pct(tilt_out, 0.95), 2), fmt(pct(tilt_out, 1.0), 2),
                         fmt(pct(tilt_trk, 0.5), 2), fmt(pct(tilt_trk, 0.95), 2)));
    }
    line(std::format("supervisor: {} acquisitions, {} losses, {} resets ({} still re-zeros), {} rejects; drift hold {} "
                     "cm; corrections {} blended {} snapped {} big; frozen queries {}",
                     t.sup.acquisitions, t.sup.losses, t.sup.resets, rezero_events, t.sup.rejects,
                     fmt(t.drift.stats.held_m * 100.0, 0), t.corr.blended, t.corr.snapped, t.corr.big,
                     t.freeze_frames));
    (void)reset_events;
    {
        const Vec3 b = t.pred.gyro_bias;
        line(std::format("output gyro bias at the end ({:.3f} {:.3f} {:.3f}) deg/s; 3DoF bias ({:.3f} {:.3f} {:.3f}) deg/s",
                         b[0] * DEG, b[1] * DEG, b[2] * DEG, (*gyro3)[0] * DEG, (*gyro3)[1] * DEG,
                         (*gyro3)[2] * DEG));
    }
    line(std::format("cpu: {} us per query (all output-layer calls / queries), {} % of one core",
                     fmt(busy_s * 1e6 / static_cast<double>(std::max<std::size_t>(qs.size(), 1)), 2),
                     fmt(busy_s / (static_cast<double>(imu.back().ts - imu.front().ts) / 1e9) * 100.0, 2)));
    if (!events_log.empty()) {
        r += "losses and acquisitions (s from the first IMU sample):\n" + events_log;
    }
    if (!per_step.empty()) {
        r += "per step:\n" + per_step;
    }
    return r;
}

} // namespace pibridge::slamtool
