// SPDX-License-Identifier: MIT
// PiBridge: the sliding-window estimator (window.h). See the header for the design; Pimax's constants are in WindowCfg.
#include "pibridge/ctrl/track/window.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace pibridge::ctrl::track {

namespace g = cam::geom;

namespace {

constexpr double G = 9.80665;
constexpr int NP = 15; // parameters per frame: dtheta, dp, dv, dbg, dba

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Vec9 = Eigen::Matrix<double, 9, 1>;

Matrix3d M(const g::Mat3& m) {
    Matrix3d o;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            o(i, j) = m[i][j];
        }
    }
    return o;
}
g::Mat3 Mg(const Matrix3d& m) {
    g::Mat3 o;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            o[i][j] = m(i, j);
        }
    }
    return o;
}
Vector3d V(const g::Vec3& v) { return Vector3d(v[0], v[1], v[2]); }
g::Vec3 Vg(const Vector3d& v) { return g::Vec3{v[0], v[1], v[2]}; }

Matrix3d skew(const Vector3d& w) {
    Matrix3d s;
    s << 0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0;
    return s;
}

Matrix3d Exp(const Vector3d& w) {
    const double a = w.norm();
    if (a < 1e-12) {
        return Matrix3d::Identity() + skew(w);
    }
    const Matrix3d k = skew(w / a);
    return Matrix3d::Identity() + std::sin(a) * k + (1.0 - std::cos(a)) * k * k;
}

Vector3d Log(const Matrix3d& r) {
    const double c = std::clamp((r.trace() - 1.0) * 0.5, -1.0, 1.0);
    const Vector3d v(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1));
    const double s = 0.5 * v.norm();
    const double a = std::atan2(s, c);
    if (s < 1e-9) {
        return 0.5 * v; // small angle (a ~ s)
    }
    return v * (a / (2.0 * s));
}

Matrix3d orthonormalize(const Matrix3d& r) {
    const Eigen::JacobiSVD<Matrix3d> svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3d o = svd.matrixU() * svd.matrixV().transpose();
    if (o.determinant() < 0.0) {
        Matrix3d f = Matrix3d::Identity();
        f(2, 2) = -1.0;
        o = svd.matrixU() * f * svd.matrixV().transpose();
    }
    return o;
}

/// Robust weight of Huber on a residual norm (already scaled): 1 inside delta, delta / |r| outside.
double huber_w(double rnorm, double delta) { return rnorm <= delta ? 1.0 : delta / rnorm; }

} // namespace

/// The IMU preintegrated over an interval with given biases: dR (device_i from device_j), dv, dp (in device_i), dt,
/// covariance on [dtheta, dv, dp] (VINS midpoint form).
struct Window::Preint {
    Matrix3d dR = Matrix3d::Identity();
    Vector3d dv = Vector3d::Zero();
    Vector3d dp = Vector3d::Zero();
    double dt = 0.0;
    Mat9 P = Mat9::Zero();
    std::size_t n = 0;

    static Preint make(std::span<const ImuStep> steps, const Vector3d& bg, const Vector3d& ba, const WindowCfg& cfg,
                       bool with_cov = true) {
        Preint q;
        for (const ImuStep& s : steps) {
            if (s.dt <= 0.0) {
                continue;
            }
            const double dt = s.dt;
            const Vector3d w = V(s.dtheta) / dt - bg;
            const Vector3d a = V(s.f) - ba;
            const Matrix3d dRk = Exp(w * dt);
            const Matrix3d R0 = q.dR;
            const Matrix3d R1 = R0 * dRk;
            const Vector3d a0 = R0 * a, a1 = R1 * a;
            const Vector3d am = 0.5 * (a0 + a1);
            q.dp += q.dv * dt + 0.5 * am * dt * dt;
            q.dv += am * dt;
            if (!with_cov) {
                q.dR = orthonormalize(R1);
                q.dt += dt;
                q.n += 1;
                continue;
            }
            // error state [dtheta, dv, dp] (dtheta: dR <- dR Exp(dtheta))
            Mat9 F = Mat9::Identity();
            F.block<3, 3>(0, 0) = dRk.transpose();
            F.block<3, 3>(3, 0) = -0.5 * (R0 * skew(a) + R1 * skew(a) * dRk.transpose()) * dt;
            F.block<3, 3>(6, 0) = 0.5 * F.block<3, 3>(3, 0) * dt;
            F.block<3, 3>(6, 3) = Matrix3d::Identity() * dt;
            Eigen::Matrix<double, 9, 6> Gm = Eigen::Matrix<double, 9, 6>::Zero();
            Gm.block<3, 3>(0, 0) = Matrix3d::Identity() * dt;                    // gyro noise -> dtheta
            Gm.block<3, 3>(3, 3) = 0.5 * (R0 + R1) * dt;                        // accel noise -> dv
            Gm.block<3, 3>(6, 3) = 0.25 * (R0 + R1) * dt * dt;                  // accel noise -> dp
            Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
            Q.block<3, 3>(0, 0) = Matrix3d::Identity() * cfg.sigma_g * cfg.sigma_g;
            Q.block<3, 3>(3, 3) = Matrix3d::Identity() * cfg.sigma_a * cfg.sigma_a;
            q.P = F * q.P * F.transpose() + Gm * Q * Gm.transpose();
            q.dR = orthonormalize(R1);
            q.dt += dt;
            q.n += 1;
        }
        return q;
    }
};

Window::Window(WindowCfg c) : cfg(c) {
    if (const char* e = std::getenv("WT_BA_W")) {
        cfg.bias_prior_w_a = std::atof(e);
    }
    if (const char* e = std::getenv("WT_BG_W")) {
        cfg.bias_prior_w_g = std::atof(e);
    }
    if (const char* e = std::getenv("WT_SIGMA_A")) {
        cfg.sigma_a = std::atof(e);
    }
    if (const char* e = std::getenv("WT_SIGMA_G")) {
        cfg.sigma_g = std::atof(e);
    }
}

void Window::reset(const WinState& s, double init_scale) {
    frames_.clear();
    prior_.reset();
    init_ = s;
    init_scale_ = init_scale;
    Frame f;
    f.x = s;
    frames_.push_back(std::move(f));
    last_cost = 0.0;
    last_vis_rms_px = 0.0;
    last_iterations = 0;
}

WinState Window::predict(std::span<const ImuStep> steps, std::uint64_t t) const {
    WinState s = frames_.back().x;
    if (steps.empty()) {
        s.t = t;
        return s;
    }
    const Preint q = Preint::make(steps, V(s.bg), V(s.ba), cfg);
    const Matrix3d Ri = M(s.r);
    const Vector3d gw(0.0, -G, 0.0);
    const Vector3d pi = V(s.p), vi = V(s.v);
    const Vector3d pj = pi + vi * q.dt + 0.5 * gw * q.dt * q.dt + Ri * q.dp;
    const Vector3d vj = vi + gw * q.dt + Ri * q.dv;
    s.r = Mg(orthonormalize(Ri * q.dR));
    s.p = Vg(pj);
    s.v = Vg(vj);
    s.t = t;
    return s;
}

namespace {

/// x (-) x0 on the 15 parameters.
Eigen::Matrix<double, 15, 1> boxminus(const WinState& x, const WinState& x0) {
    Eigen::Matrix<double, 15, 1> d;
    d.segment<3>(0) = Log(M(x0.r).transpose() * M(x.r));
    d.segment<3>(3) = V(x.p) - V(x0.p);
    d.segment<3>(6) = V(x.v) - V(x0.v);
    d.segment<3>(9) = V(x.bg) - V(x0.bg);
    d.segment<3>(12) = V(x.ba) - V(x0.ba);
    return d;
}

void boxplus(WinState& x, const Eigen::Matrix<double, 15, 1>& d) {
    x.r = Mg(orthonormalize(M(x.r) * Exp(d.segment<3>(0))));
    x.p = Vg(V(x.p) + d.segment<3>(3));
    x.v = Vg(V(x.v) + d.segment<3>(6));
    x.bg = Vg(V(x.bg) + d.segment<3>(9));
    x.ba = Vg(V(x.ba) + d.segment<3>(12));
}

/// Normal equations accumulator over a subset of frames (col[k] = column offset of frame k, or -1 to leave it out).
struct Normal {
    MatrixXd H;
    VectorXd b;
    double cost = 0.0;
    std::vector<int> col;
    explicit Normal(std::size_t n, std::vector<int> c) : H(MatrixXd::Zero(n, n)), b(VectorXd::Zero(n)), col(std::move(c)) {}
    /// Adds a factor: residual r (whitened), Jacobian blocks J_k (rows x 15) per frame k, robust weight w (scales
    /// the factor's information; the cost adds the robust loss itself).
    void add(const VectorXd& r, const std::vector<std::pair<std::size_t, MatrixXd>>& J, double w, double loss) {
        cost += loss;
        for (const auto& [ka, Ja] : J) {
            if (col[ka] < 0) {
                continue;
            }
            b.segment(col[ka], NP) += w * Ja.transpose() * r;
            for (const auto& [kb, Jb] : J) {
                if (col[kb] < 0) {
                    continue;
                }
                H.block(col[ka], col[kb], NP, NP) += w * Ja.transpose() * Jb;
            }
        }
    }
};

} // namespace

/// Evaluates every factor of the window at the current states into `nrm` (frames with col < 0 are fixed, their
/// factors still contribute to the others), the IMU preintegrations recomputed at the frames' biases.
namespace {
struct Eval {
    const std::deque<Window::Frame>& fr; // (a deque: never indexed through a raw pointer)
    std::size_t n;
    const WindowCfg& cfg;
    bool fast;
    const std::optional<Window::Prior>* prior;
    bool only_touching_oldest; // marginalization: only factors involving frame 0

    void run(Normal& nrm, double* vis_sq_newest, std::size_t* vis_n_newest) const {
        const Vector3d gw(0.0, -G, 0.0);
        const double hub = fast ? cfg.huber_fast : cfg.huber;
        // prior on frame 0
        if (*prior) {
            const Window::Prior& P = **prior;
            const auto d = boxminus(fr[0].x, P.x0);
            const Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jm(
                P.J.data(), static_cast<Eigen::Index>(P.rows), NP);
            const Eigen::Map<const VectorXd> r0(P.r0.data(), static_cast<Eigen::Index>(P.rows));
            const MatrixXd J = Jm;
            VectorXd r = r0 + J * d;
            nrm.add(r, {{0, J}}, 1.0, 0.5 * r.squaredNorm());
        } else if (n >= 1) {
            // Pimax: a bias prior (weight 5e4) on the first frame while there is no marginal prior; PiBridge: the
            // first frame's pose and velocity too (the acquisition), else the gauge is free before the first IMU link
            {
                MatrixXd J = MatrixXd::Zero(6, NP);
                J.block<3, 3>(0, 9) = Matrix3d::Identity() * cfg.bias_prior_w_g;
                J.block<3, 3>(3, 12) = Matrix3d::Identity() * cfg.bias_prior_w_a;
                VectorXd r(6);
                r.segment<3>(0) = cfg.bias_prior_w_g * (V(fr[0].x.bg) - V(init_bg_));
                r.segment<3>(3) = cfg.bias_prior_w_a * (V(fr[0].x.ba) - V(init_ba_));
                nrm.add(r, {{0, J}}, 1.0, 0.5 * r.squaredNorm());
            }
            if (init_pose_) {
                MatrixXd J = MatrixXd::Zero(9, NP);
                const auto d = boxminus(fr[0].x, *init_pose_);
                VectorXd r(9);
                const double ir = cfg.init_r * init_scale_, ip = cfg.init_p * init_scale_;
                J.block<3, 3>(0, 0) = Matrix3d::Identity() / ir;
                J.block<3, 3>(3, 3) = Matrix3d::Identity() / ip;
                J.block<3, 3>(6, 6) = Matrix3d::Identity() / cfg.init_v;
                r.segment<3>(0) = d.segment<3>(0) / ir;
                r.segment<3>(3) = d.segment<3>(3) / ip;
                r.segment<3>(6) = d.segment<3>(6) / cfg.init_v;
                nrm.add(r, {{0, J}}, 1.0, 0.5 * r.squaredNorm());
            }
        }
        for (std::size_t k = 0; k < n; k++) {
            if (only_touching_oldest && k >= 1) {
                break; // marginalization: frame 0's own factors and its links to frame 1 only
            }
            // visual factors of frame k
            if (!only_touching_oldest || k == 0) {
                const Window::Frame& f = fr[k];
                const Matrix3d Rbw = M(f.body_from_world.r);
                const Vector3d tbw = V(f.body_from_world.t);
                const Matrix3d R = M(f.x.r);
                const Vector3d p = V(f.x.p);
                for (const VisObs& o : f.obs) {
                    const Matrix3d Rcb = M(o.cam->cam_from_body.r);
                    const Vector3d tcb = V(o.cam->cam_from_body.t);
                    const Matrix3d Mm = Rcb * Rbw;
                    const Vector3d c0 = Rcb * tbw + tcb;
                    const Vector3d pl = V(o.p_led);
                    const Vector3d Xc = Mm * (R * pl + p) + c0;
                    const auto uv = o.cam->intr.project(Vg(Xc));
                    if (!uv) {
                        continue;
                    }
                    Eigen::Vector2d r(uv->first - o.u, uv->second - o.v);
                    // d pi / d Xc by forward differences
                    Eigen::Matrix<double, 2, 3> dpi;
                    for (int a = 0; a < 3; a++) {
                        Vector3d Xd = Xc;
                        const double h = 1e-6 + 1e-6 * std::fabs(Xc[a]);
                        Xd[a] += h;
                        const auto uvd = o.cam->intr.project(Vg(Xd));
                        if (!uvd) {
                            dpi.col(a).setZero();
                            continue;
                        }
                        dpi(0, a) = (uvd->first - uv->first) / h;
                        dpi(1, a) = (uvd->second - uv->second) / h;
                    }
                    MatrixXd J = MatrixXd::Zero(2, NP);
                    J.block<2, 3>(0, 0) = cfg.vis_w * dpi * (-Mm * R * skew(pl));
                    J.block<2, 3>(0, 3) = cfg.vis_w * dpi * Mm;
                    r *= cfg.vis_w;
                    const double rn = r.norm();
                    const double w = huber_w(rn, hub);
                    const double loss = rn <= hub ? 0.5 * rn * rn : hub * (rn - 0.5 * hub);
                    nrm.add(r, {{k, J}}, w, loss);
                    if (k + 1 == n && vis_sq_newest != nullptr) {
                        *vis_sq_newest += (rn / cfg.vis_w) * (rn / cfg.vis_w);
                        *vis_n_newest += 1;
                    }
                }
            }
            if (k + 1 >= n) {
                break;
            }
            // frame k -> k+1: IMU, bias walk, stationary
            const Window::Frame& fi = fr[k];
            const Window::Frame& fj = fr[k + 1];
            const Matrix3d Ri = M(fi.x.r), Rj = M(fj.x.r);
            const Vector3d pi = V(fi.x.p), pj = V(fj.x.p), vi = V(fi.x.v), vj = V(fj.x.v);
            const Vector3d bg = V(fi.x.bg), ba = V(fi.x.ba);
            double dt_total = 0.0;
            for (const ImuStep& s : fj.steps) {
                dt_total += s.dt;
            }
            // Pimax links two frames by the IMU only when the interval is at least 15 ms with more than 8 samples
            // (at its 500 Hz); the hint runs at 250 Hz, the bench's log lower still: 2 steps suffice here
            const bool imu_link = dt_total >= 0.015 && fj.steps.size() >= 2;
            if (imu_link) {
                const Window::Preint q = Window::Preint::make(fj.steps, bg, ba, cfg);
                const double dt = q.dt;
                // residual [theta, v, p], the covariance's order
                const auto resid = [&](const Window::Preint& qq) {
                    Vec9 r;
                    r.segment<3>(0) = Log(qq.dR.transpose() * Ri.transpose() * Rj);
                    r.segment<3>(3) = Ri.transpose() * (vj - vi - gw * dt) - qq.dv;
                    r.segment<3>(6) = Ri.transpose() * (pj - pi - vi * dt - 0.5 * gw * dt * dt) - qq.dp;
                    return r;
                };
                const Vec9 r = resid(q);
                MatrixXd Ji = MatrixXd::Zero(9, NP), Jj = MatrixXd::Zero(9, NP);
                // theta
                Ji.block<3, 3>(0, 0) = -(Rj.transpose() * Ri);
                Jj.block<3, 3>(0, 0) = Matrix3d::Identity();
                // v
                Ji.block<3, 3>(3, 0) = skew(Ri.transpose() * (vj - vi - gw * dt));
                Ji.block<3, 3>(3, 6) = -Ri.transpose();
                Jj.block<3, 3>(3, 6) = Ri.transpose();
                // p
                Ji.block<3, 3>(6, 0) = skew(Ri.transpose() * (pj - pi - vi * dt - 0.5 * gw * dt * dt));
                Ji.block<3, 3>(6, 3) = -Ri.transpose();
                Ji.block<3, 3>(6, 6) = -Ri.transpose() * dt;
                Jj.block<3, 3>(6, 3) = Ri.transpose();
                // biases of frame i: by re-integration (central differences)
                for (int a = 0; a < 6; a++) {
                    const double h = a < 3 ? 1e-5 : 1e-4;
                    Vector3d bgp = bg, bap = ba, bgm = bg, bam = ba;
                    if (a < 3) {
                        bgp[a] += h;
                        bgm[a] -= h;
                    } else {
                        bap[a - 3] += h;
                        bam[a - 3] -= h;
                    }
                    const Vec9 rp = resid(Window::Preint::make(fj.steps, bgp, bap, cfg, false));
                    const Vec9 rm = resid(Window::Preint::make(fj.steps, bgm, bam, cfg, false));
                    Ji.block<9, 1>(0, 9 + a) = (rp - rm) / (2.0 * h);
                }
                // whitening: Omega = P^-1 (P floored)
                Mat9 P = q.P;
                for (int d = 0; d < 9; d++) {
                    P(d, d) += d < 3 ? 1e-10 : d < 6 ? 1e-10 : 1e-12;
                }
                const Eigen::LLT<Mat9> llt(P.inverse());
                const Mat9 L = llt.matrixL();
                const Mat9 Lt = L.transpose();
                const Vec9 rw = Lt * r;
                nrm.add(rw, {{k, MatrixXd(Lt * Ji)}, {k + 1, MatrixXd(Lt * Jj)}}, 1.0, 0.5 * rw.squaredNorm());
                // bias random walk
                {
                    MatrixXd Bi = MatrixXd::Zero(6, NP), Bj = MatrixXd::Zero(6, NP);
                    VectorXd rb(6);
                    const double sg = cfg.walk_bg * std::sqrt(dt), sa = cfg.walk_ba * std::sqrt(dt);
                    Bi.block<3, 3>(0, 9) = -Matrix3d::Identity() / sg;
                    Bj.block<3, 3>(0, 9) = Matrix3d::Identity() / sg;
                    Bi.block<3, 3>(3, 12) = -Matrix3d::Identity() / sa;
                    Bj.block<3, 3>(3, 12) = Matrix3d::Identity() / sa;
                    rb.segment<3>(0) = (V(fj.x.bg) - bg) / sg;
                    rb.segment<3>(3) = (V(fj.x.ba) - ba) / sa;
                    nrm.add(rb, {{k, Bi}, {k + 1, Bj}}, 1.0, 0.5 * rb.squaredNorm());
                }
            }
            if (fj.still_strict) {
                MatrixXd Si = MatrixXd::Zero(6, NP), Sj = MatrixXd::Zero(6, NP);
                VectorXd rs(6);
                const Vector3d dth = Log(Ri.transpose() * Rj);
                Si.block<3, 3>(0, 0) = -(Rj.transpose() * Ri) / cfg.static_r;
                Sj.block<3, 3>(0, 0) = Matrix3d::Identity() / cfg.static_r;
                Si.block<3, 3>(3, 3) = -Matrix3d::Identity() / cfg.static_p;
                Sj.block<3, 3>(3, 3) = Matrix3d::Identity() / cfg.static_p;
                rs.segment<3>(0) = dth / cfg.static_r;
                rs.segment<3>(3) = (pj - pi) / cfg.static_p;
                nrm.add(rs, {{k, Si}, {k + 1, Sj}}, 1.0, 0.5 * rs.squaredNorm());
            }
            if (fj.still_loose && !only_touching_oldest) {
                const double sv = fj.still_strict ? cfg.zero_v_strict : cfg.zero_v;
                MatrixXd Z = MatrixXd::Zero(3, NP);
                Z.block<3, 3>(0, 6) = Matrix3d::Identity() / sv;
                VectorXd rz = vj / sv;
                nrm.add(rz, {{k + 1, Z}}, 1.0, 0.5 * rz.squaredNorm());
            }
        }
    }
    g::Vec3 init_bg_{}, init_ba_{};
    std::optional<WinState> init_pose_;
    double init_scale_ = 1.0; // Window::init_scale_: a relock's wider first-frame prior
};
} // namespace

struct WindowAccess {
    static Eval eval(const Window& w, bool marg) {
        Eval e{w.frames_, w.frames_.size(), w.cfg, w.fast_, &w.prior_, marg};
        return e;
    }
    static double evaluate(Window& w, MatrixXd& H, VectorXd& b);
    static bool apply_step(Window& w, const MatrixXd& H, const VectorXd& b, double lambda);
};

namespace {
// the first frame's prior (the acquisition) is kept here per window through a side table keyed by the window
// address: simplest is a member; we store it in the oldest frame when no marginal prior exists
} // namespace

/// One evaluation of the normal equations at the current states (cost, H, b); last_vis_rms_px is set from it.
double WindowAccess::evaluate(Window& w, MatrixXd& H, VectorXd& b) {
    auto& frames_ = w.frames_;
    auto& init_ = w.init_;
    auto& prior_ = w.prior_;
    const std::size_t n = frames_.size();
    std::vector<int> col(n);
    for (std::size_t k = 0; k < n; k++) {
        col[k] = static_cast<int>(k * NP);
    }
    Normal nrm(n * NP, col);
    Eval e = WindowAccess::eval(w, false);
    e.init_bg_ = init_.bg;
    e.init_ba_ = init_.ba;
    e.init_pose_ = prior_ ? std::nullopt : std::optional<WinState>(init_);
    e.init_scale_ = w.init_scale_;
    double vis_sq = 0.0;
    std::size_t vis_n = 0;
    e.run(nrm, &vis_sq, &vis_n);
    w.last_vis_rms_px = vis_n > 0 ? std::sqrt(vis_sq / static_cast<double>(vis_n)) : 0.0;
    H = std::move(nrm.H);
    b = std::move(nrm.b);
    return nrm.cost;
}

/// The damped Gauss-Newton step from (H, b) applied to the states.
bool WindowAccess::apply_step(Window& w, const MatrixXd& H0, const VectorXd& b, double lambda) {
    auto& frames_ = w.frames_;
    MatrixXd H = H0;
    for (Eigen::Index i = 0; i < H.rows(); i++) {
        H(i, i) += lambda * (1.0 + H(i, i));
    }
    const VectorXd dx = H.ldlt().solve(-b);
    if (!dx.allFinite()) {
        return false;
    }
    for (std::size_t k = 0; k < frames_.size(); k++) {
        boxplus(frames_[k].x, dx.segment<NP>(static_cast<Eigen::Index>(k * NP)));
    }
    return true;
}

void Window::push(std::uint64_t t, const g::Pose& body_from_world, std::vector<VisObs> obs, std::vector<ImuStep> steps,
                  bool still_strict, bool still_loose, bool fast) {
    if (frames_.empty()) {
        return;
    }
    fast_ = fast;
    Frame f;
    f.x = predict(steps, t);
    f.x.t = t;
    f.body_from_world = body_from_world;
    f.obs = std::move(obs);
    f.steps = std::move(steps);
    f.still_strict = still_strict;
    f.still_loose = still_loose;
    frames_.push_back(std::move(f));
    // Levenberg-Marquardt, cfg.iterations steps, one evaluation each: the step is taken from the evaluation at the
    // current states; the next evaluation says whether it lowered the cost (else it is undone and the damping raised)
    double lambda = 1e-6;
    MatrixXd H, Hprev;
    VectorXd b, bprev;
    double cost = WindowAccess::evaluate(*this, H, b);
    double rms_prev = last_vis_rms_px;
    last_iterations = 0;
    last_accepted = 0;
    std::deque<Frame> backup = frames_;
    for (std::size_t it = 0; it < cfg.iterations; it++) {
        Hprev = H;
        bprev = b;
        backup = frames_;
        if (!WindowAccess::apply_step(*this, H, b, lambda)) {
            break;
        }
        const double c2 = WindowAccess::evaluate(*this, H, b);
        last_iterations += 1;
        if (c2 < cost) {
            cost = c2;
            rms_prev = last_vis_rms_px;
            lambda = std::max(lambda * 0.3, 1e-9);
            last_accepted += 1;
        } else {
            frames_ = backup;
            H = Hprev;
            b = bprev;
            last_vis_rms_px = rms_prev;
            lambda *= 10.0;
        }
    }
    last_cost = cost;
    while (frames_.size() > cfg.max_frames) {
        marginalize();
    }
}

std::pair<std::array<double, 9>, std::array<double, 9>> Window::imu_residual_debug(const WinState& a, const WinState& b,
                                                                                  std::span<const ImuStep> steps) const {
    const Preint q = Preint::make(steps, V(a.bg), V(a.ba), cfg);
    const Vector3d gw(0.0, -G, 0.0);
    const Matrix3d Ri = M(a.r), Rj = M(b.r);
    const double dt = q.dt;
    Vec9 r;
    r.segment<3>(0) = Log(q.dR.transpose() * Ri.transpose() * Rj);
    r.segment<3>(3) = Ri.transpose() * (V(b.v) - V(a.v) - gw * dt) - q.dv;
    r.segment<3>(6) = Ri.transpose() * (V(b.p) - V(a.p) - V(a.v) * dt - 0.5 * gw * dt * dt) - q.dp;
    std::array<double, 9> ro{}, pd{};
    for (int i = 0; i < 9; i++) {
        ro[i] = r[i];
        pd[i] = q.P(i, i);
    }
    return {ro, pd};
}

void Window::marginalize() {
    if (frames_.size() < 2) {
        return;
    }
    // factors touching frame 0, linearized at the current states, over frames 0 and 1
    std::vector<int> col(frames_.size(), -1);
    col[0] = 0;
    col[1] = NP;
    Normal nrm(2 * NP, col);
    Eval e = WindowAccess::eval(*this, true);
    e.init_bg_ = init_.bg;
    e.init_ba_ = init_.ba;
    e.init_pose_ = prior_ ? std::nullopt : std::optional<WinState>(init_);
    e.init_scale_ = init_scale_;
    e.run(nrm, nullptr, nullptr);
    const MatrixXd H00 = nrm.H.block(0, 0, NP, NP) + MatrixXd::Identity(NP, NP) * 1e-9;
    const MatrixXd H01 = nrm.H.block(0, NP, NP, NP);
    const MatrixXd H11 = nrm.H.block(NP, NP, NP, NP);
    const VectorXd b0 = nrm.b.segment(0, NP), b1 = nrm.b.segment(NP, NP);
    const MatrixXd H00i = H00.ldlt().solve(MatrixXd::Identity(NP, NP));
    MatrixXd Hr = H11 - H01.transpose() * H00i * H01;
    VectorXd br = b1 - H01.transpose() * H00i * b0;
    Hr = 0.5 * (Hr + Hr.transpose());
    const Eigen::SelfAdjointEigenSolver<MatrixXd> es(Hr);
    const VectorXd ev = es.eigenvalues();
    const double emax = ev.cwiseAbs().maxCoeff();
    std::vector<int> keep;
    for (int i = 0; i < ev.size(); i++) {
        if (ev[i] > 1e-12 * std::max(emax, 1.0)) {
            keep.push_back(i);
        }
    }
    Prior P;
    P.rows = keep.size();
    P.J.assign(keep.size() * NP, 0.0);
    P.r0.assign(keep.size(), 0.0);
    for (std::size_t q = 0; q < keep.size(); q++) {
        const int i = keep[q];
        const double s = std::sqrt(ev[i]);
        for (int c = 0; c < NP; c++) {
            P.J[q * NP + c] = s * es.eigenvectors()(c, i);
        }
        P.r0[q] = es.eigenvectors().col(i).dot(br) / s;
    }
    P.x0 = frames_[1].x;
    prior_ = std::move(P);
    frames_.pop_front();
}

} // namespace pibridge::ctrl::track
