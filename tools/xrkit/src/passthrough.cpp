// SPDX-License-Identifier: MIT
#include "pibridge/xrkit/passthrough.h"

#include "pibridge/cam/camera.h"
#include "pibridge/cam/frame.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace pibridge::xrkit::pt {

namespace {

room::Mat34 fromPose(const cam::geom::Pose& p) {
    room::Mat34 m;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            m.m[r][c] = p.r[r][c];
        }
        m.m[r][3] = p.t[r];
    }
    return m;
}

room::Mat34 translation(room::Vec3 t) {
    room::Mat34 m;
    m.m[0][3] = t.x;
    m.m[1][3] = t.y;
    m.m[2][3] = t.z;
    return m;
}

// column-major mat4 -> the 3x4 it holds
struct M {
    const std::array<float, 16>& a;
    double r(int row, int col) const { return a[static_cast<std::size_t>(col * 4 + row)]; }
    room::Vec3 xform(room::Vec3 p) const {
        return {r(0, 0) * p.x + r(0, 1) * p.y + r(0, 2) * p.z + r(0, 3), r(1, 0) * p.x + r(1, 1) * p.y + r(1, 2) * p.z + r(1, 3),
                r(2, 0) * p.x + r(2, 1) * p.y + r(2, 2) * p.z + r(2, 3)};
    }
    room::Vec3 rot(room::Vec3 p) const {
        return {r(0, 0) * p.x + r(0, 1) * p.y + r(0, 2) * p.z, r(1, 0) * p.x + r(1, 1) * p.y + r(1, 2) * p.z,
                r(2, 0) * p.x + r(2, 1) * p.y + r(2, 2) * p.z};
    }
};

// the shader's project(): pixel coordinates (texel centres at integers) of a stage point in slot c, and its weight
struct Proj {
    double px = 0, py = 0, w = 0;
};
std::optional<Proj> project(const Uniform& u, int c, room::Vec3 p) {
    const auto ci = static_cast<std::size_t>(c);
    if (u.kbL[ci][1] < 0.5F) {
        return std::nullopt;
    }
    const room::Vec3 q = M{u.camFromStage[ci]}.xform(p);
    const double r = std::hypot(q.x, q.y);
    const double th = std::atan2(r, q.z);
    if (th > u.kbL[ci][0]) {
        return std::nullopt;
    }
    const auto& k = u.kbK[ci];
    const double t2 = th * th;
    const double td = th * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
    const double dx = r > 1e-9 ? q.x / r : 0.0;
    const double dy = r > 1e-9 ? q.y / r : 0.0;
    const auto& f = u.kbF[ci];
    const double px = f[0] * td * dx + f[2];
    const double py = f[1] * td * dy + f[3];
    const double b = u.roomInfo[3];
    const double edge = std::min({px - b, kW - 1.0 - b - px, py - b, kH - 1.0 - b - py});
    const double vig = u.info[2] - std::hypot(px - f[2], py - f[3]);
    const double w = std::clamp(std::min(edge, vig) / std::max(1.0, static_cast<double>(u.info[1])), 0.0, 1.0);
    if (w <= 0) {
        return std::nullopt;
    }
    return Proj{px, py, w};
}

// texture(): bilinear, clamp to edge
double texel(const std::vector<std::uint8_t>& img, double px, double py) {
    const auto at = [&](int x, int y) {
        x = std::clamp(x, 0, kW - 1);
        y = std::clamp(y, 0, kH - 1);
        return img[static_cast<std::size_t>(y * kW + x)] / 255.0;
    };
    const double fx = std::floor(px), fy = std::floor(py);
    const double ax = px - fx, ay = py - fy;
    const int x = static_cast<int>(fx), y = static_cast<int>(fy);
    return (at(x, y) * (1 - ax) + at(x + 1, y) * ax) * (1 - ay) + (at(x, y + 1) * (1 - ax) + at(x + 1, y + 1) * ax) * ay;
}

room::Vec3 eyeRay(const Uniform& u, double uu, double vv, room::Vec3& origin) {
    const double tx = u.tan[0] + (u.tan[1] - u.tan[0]) * uu;
    const double ty = u.tan[2] + (u.tan[3] - u.tan[2]) * vv;
    const room::Vec3 dEye = room::Vec3{tx, ty, -1.0}.norm();
    const M se{u.stageFromEye};
    origin = {se.r(0, 3), se.r(1, 3), se.r(2, 3)};
    return se.rot(dEye);
}

room::Poly thin(const room::Poly& p) {
    if (p.size() <= kWalls) {
        return p;
    }
    room::Poly o;
    for (std::size_t i = 0; i < kWalls; i++) {
        o.push_back(p[i * p.size() / kWalls]);
    }
    return o;
}

} // namespace

std::vector<std::size_t> Rig::tiles() const {
    std::vector<std::size_t> t;
    for (std::size_t s = 0; s < n; s++) {
        t.push_back(cam[s].tile);
    }
    return t;
}

std::array<std::uint8_t, kCams> preference(const Rig& r, int eye) {
    std::array<std::uint8_t, kCams> ids{};
    std::vector<std::uint8_t> v;
    for (std::size_t s = 0; s < r.n; s++) {
        v.push_back(static_cast<std::uint8_t>(s));
    }
    std::ranges::stable_sort(v, [&](std::uint8_t a, std::uint8_t b) {
        const auto& ca = r.cam[a];
        const auto& cb = r.cam[b];
        if (ca.upper != cb.upper) {
            return !ca.upper;
        }
        const bool sideA = (ca.viewFromCam.pos().x < 0) == (eye == 0);
        const bool sideB = (cb.viewFromCam.pos().x < 0) == (eye == 0);
        return sideA && !sideB;
    });
    for (std::size_t i = 0; i < kCams; i++) {
        ids[i] = i < v.size() ? v[i] : static_cast<std::uint8_t>(255);
    }
    return ids;
}

std::expected<Rig, std::string> makeRig(const cam::DeviceCalib& cal, room::Vec3 headInBody, bool upper) {
    Rig rig;
    for (std::size_t id = 0; id < kCams; id++) {
        const auto it = std::ranges::find_if(cal.cameras, [id](const cam::CameraCalib& c) { return c.id == id; });
        const auto body = it == cal.cameras.end() ? std::nullopt : cal.body_from_cam(id, cam::RigConvention::CamFromCam0);
        const bool ok = it != cal.cameras.end() && body && it->intr.width == static_cast<std::uint32_t>(kW) &&
                        it->intr.height == static_cast<std::uint32_t>(kH);
        if (!ok) {
            if (id < 2) {
                if (it == cal.cameras.end() || !body) {
                    return std::unexpected("the calibration has no camera " + std::to_string(id) + " with its body pose");
                }
                return std::unexpected("camera " + std::to_string(id) + " is not 640x480");
            }
            continue;
        }
        Camera c;
        c.tile = id;
        c.name = it->name;
        c.kb = it->intr;
        c.viewFromCam = room::mul34(translation(headInBody * -1.0), fromPose(*body));
        c.upper = c.viewFromCam.axis(2).y > 0; // the optical axis points up
        if (c.upper && !upper) {
            continue;
        }
        rig.cam[rig.n++] = c;
    }
    for (int e = 0; e < 2; e++) {
        rig.order[static_cast<std::size_t>(e)] = preference(rig, e);
    }
    return rig;
}

EyeView eyeView(const room::Mat34& stageFromEye, double angleLeft, double angleRight, double angleUp, double angleDown) {
    return {stageFromEye, std::tan(angleLeft), std::tan(angleRight), std::tan(angleUp), std::tan(angleDown)};
}

Uniform makeUniform(int eye, const EyeView& e, const Rig& rig, const room::Mat34& stageFromViewAtCapture, const Look& look) {
    Uniform u;
    u.stageFromEye = room::mat4From34(e.stageFromEye);
    u.tan = {static_cast<float>(e.tanL), static_cast<float>(e.tanR), static_cast<float>(e.tanU), static_cast<float>(e.tanD)};
    u.misc = {static_cast<float>(look.floorY), static_cast<float>(look.radius), static_cast<float>(look.gain),
              look.srgb ? 1.0F : 0.0F};
    u.info = {static_cast<float>(rig.n), static_cast<float>(look.feather), static_cast<float>(look.vignette),
              static_cast<float>(kMaxFloor)};
    const auto& ord = rig.order[eye == 1 ? 1 : 0];
    for (std::size_t i = 0; i < kCams; i++) {
        u.order[i] = ord[i] == 255 ? -1.0F : static_cast<float>(ord[i]);
    }
    u.roomInfo = {0, 0, 0, static_cast<float>(kBorderPx)};
    if (look.room && look.room->outline.size() >= 3) {
        const room::Poly p = thin(look.room->outline);
        for (std::size_t i = 0; i < p.size(); i++) {
            const auto& a = p[i];
            const auto& b = p[(i + 1) % p.size()];
            u.walls[i] = {static_cast<float>(a.x), static_cast<float>(a.z), static_cast<float>(b.x), static_cast<float>(b.z)};
        }
        u.roomInfo[0] = static_cast<float>(p.size());
        u.roomInfo[1] = static_cast<float>(look.room->ceilingY);
        u.roomInfo[2] = 1.0F;
    }
    for (std::size_t c = 0; c < rig.n; c++) {
        const room::Mat34 stageFromCam = room::mul34(stageFromViewAtCapture, rig.cam[c].viewFromCam);
        u.camFromStage[c] = room::mat4From34(room::inv34(stageFromCam));
        const auto& k = rig.cam[c].kb;
        u.kbF[c] = {static_cast<float>(k.fx), static_cast<float>(k.fy), static_cast<float>(k.cx), static_cast<float>(k.cy)};
        u.kbK[c] = {static_cast<float>(k.k[0]), static_cast<float>(k.k[1]), static_cast<float>(k.k[2]), static_cast<float>(k.k[3])};
        u.kbL[c] = {static_cast<float>(k.theta_limit), 1.0F, static_cast<float>(look.camGain[c]), 0.0F};
    }
    return u;
}

double surfaceDistance(const Uniform& u, room::Vec3 o, room::Vec3 d) {
    constexpr double kBig = 1e9;
    double tf = kBig;
    if (d.y < -1e-4) {
        const double t = (u.misc[0] - o.y) / d.y;
        if (t > 0) {
            tf = t;
        }
    }
    if (u.roomInfo[2] > 0.5F) {
        double t = tf;
        if (d.y > 1e-4) {
            const double tc = (u.roomInfo[1] - o.y) / d.y;
            if (tc > 0) {
                t = std::min(t, tc);
            }
        }
        const int nw = static_cast<int>(u.roomInfo[0]);
        for (int i = 0; i < nw; i++) {
            const auto& w = u.walls[static_cast<std::size_t>(i)];
            const double ex = w[2] - w[0], ez = w[3] - w[1];
            const double den = d.x * ez - d.z * ex;
            if (std::fabs(den) < 1e-9) {
                continue;
            }
            const double wx = w[0] - o.x, wz = w[1] - o.z;
            const double tw = (wx * ez - wz * ex) / den;
            const double s = (wx * d.z - wz * d.x) / den;
            if (tw > 1e-4 && s >= 0.0 && s <= 1.0) {
                t = std::min(t, tw);
            }
        }
        if (t < kBig) {
            return std::min(t, static_cast<double>(u.info[3]));
        }
        return u.misc[1];
    }
    return tf < kBig ? std::min(tf, static_cast<double>(u.info[3])) : static_cast<double>(u.misc[1]);
}

std::optional<Sample> sampleFull(const Uniform& u, const Frame& f, double uu, double vv) {
    room::Vec3 o;
    const room::Vec3 d = eyeRay(u, uu, vv, o);
    const room::Vec3 p = o + d * surfaceDistance(u, o, d);
    const int n = static_cast<int>(u.info[0]);
    int a = -1, b = -1;
    Proj pa, pb;
    for (int i = 0; i < n; i++) {
        const int c = static_cast<int>(u.order[static_cast<std::size_t>(i)]);
        if (c < 0) {
            continue;
        }
        const auto pr = project(u, c, p);
        if (!pr) {
            continue;
        }
        if (a < 0) {
            a = c;
            pa = *pr;
            if (pa.w >= 1.0) {
                break;
            }
        } else if (b < 0 || pr->w > pb.w) {
            b = c;
            pb = *pr;
        }
    }
    if (a < 0) {
        return std::nullopt;
    }
    const auto grey = [&](int c, const Proj& pr) {
        const auto& img = f.img[static_cast<std::size_t>(c)];
        if (img.size() != static_cast<std::size_t>(kW * kH)) {
            return 0.0;
        }
        return texel(img, pr.px, pr.py) * u.kbL[static_cast<std::size_t>(c)][2];
    };
    Sample s;
    s.a = a;
    s.b = b;
    double g = 0;
    if (b >= 0) {
        s.alpha = static_cast<float>(pa.w / (pa.w + (1.0 - pa.w) * pb.w));
        g = grey(a, pa) * s.alpha + grey(b, pb) * (1.0 - s.alpha);
    } else {
        s.alpha = static_cast<float>(pa.w);
        g = grey(a, pa) * pa.w + (5.0 / 255.0) * (1.0 - pa.w); // fades to the "no camera" grey
    }
    s.grey = static_cast<float>(std::clamp(g * u.misc[2], 0.0, 1.0));
    return s;
}

std::optional<float> sample(const Uniform& u, const Frame& f, double uu, double vv) {
    if (const auto s = sampleFull(u, f, uu, vv)) {
        return s->grey;
    }
    return std::nullopt;
}

room::Image render(const Uniform& u, const Frame& f, int w, int h) {
    room::Image img{w, h, std::vector<std::uint8_t>(static_cast<std::size_t>(w * h * 3))};
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const auto g = sample(u, f, (x + 0.5) / w, (y + 0.5) / h);
            const auto v = static_cast<std::uint8_t>(g ? std::lround(*g * 255.0) : 5);
            const auto i = static_cast<std::size_t>((y * w + x) * 3);
            img.rgb[i] = img.rgb[i + 1] = img.rgb[i + 2] = v;
        }
    }
    return img;
}

room::Image coverage(const Uniform& u, const Frame& f, int w, int h) {
    static constexpr std::array<std::array<double, 3>, kCams> kCol{{{230, 60, 60}, {60, 110, 240}, {235, 210, 60}, {60, 200, 90}}};
    room::Image img{w, h, std::vector<std::uint8_t>(static_cast<std::size_t>(w * h * 3), 12)};
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const auto s = sampleFull(u, f, (x + 0.5) / w, (y + 0.5) / h);
            if (!s) {
                continue;
            }
            const auto i = static_cast<std::size_t>((y * w + x) * 3);
            for (std::size_t k = 0; k < 3; k++) {
                const double ca = kCol[static_cast<std::size_t>(s->a)][k];
                const double cb = s->b >= 0 ? kCol[static_cast<std::size_t>(s->b)][k] : 12.0;
                const double shade = 0.55 + 0.45 * s->grey; // the picture shows through
                img.rgb[i + k] = static_cast<std::uint8_t>(std::clamp((ca * s->alpha + cb * (1 - s->alpha)) * shade, 0.0, 255.0));
            }
        }
    }
    return img;
}

double autoGain(double mean) {
    if (!(mean > 0)) {
        return 1.0;
    }
    return std::clamp(100.0 / mean, 1.0, 4.0);
}

std::array<double, kCams> balanceGains(const Frame& f) {
    std::array<double, kCams> g{1, 1, 1, 1};
    if (!(f.mean > 1.0F)) {
        return g;
    }
    for (std::size_t s = 0; s < std::min(f.n, kCams); s++) {
        if (f.means[s] > 1.0F) {
            g[s] = std::clamp(static_cast<double>(f.mean) / f.means[s], 0.6, 1.6);
        }
    }
    return g;
}

float meanOf(const std::vector<std::uint8_t>& img) {
    if (img.size() < static_cast<std::size_t>(kW * kH)) {
        return 0;
    }
    std::uint64_t sum = 0, n = 0;
    for (int y = 0; y < kH; y += 8) {
        for (int x = 0; x < kW; x += 8) {
            sum += img[static_cast<std::size_t>(y * kW + x)];
            n++;
        }
    }
    return n == 0 ? 0.0F : static_cast<float>(static_cast<double>(sum) / static_cast<double>(n));
}

// ---------------------------------------------------------------- Feed

Feed::Feed(std::string spec, std::vector<std::size_t> tiles, bool loopReplay)
    : spec_(std::move(spec)), tiles_(std::move(tiles)), loop_(loopReplay) {
    if (tiles_.size() > kCams) {
        tiles_.resize(kCams);
    }
}

Feed::~Feed() { stop(); }

void Feed::start() {
    if (!th_.joinable()) {
        stop_.store(false);
        th_ = std::thread([this] { run(); });
    }
}

void Feed::stop() {
    stop_.store(true);
    if (th_.joinable()) {
        th_.join();
    }
}

bool Feed::take(Frame& out, std::uint64_t afterSeq) {
    std::lock_guard l(m_);
    if (!fresh_ || back_.seq <= afterSeq) {
        return false;
    }
    std::swap(out.img, back_.img);
    out.monoNs = back_.monoNs;
    out.seq = back_.seq;
    out.mean = back_.mean;
    out.means = back_.means;
    out.n = back_.n;
    fresh_ = false;
    return true;
}

std::string Feed::error() const {
    std::lock_guard l(m_);
    return error_;
}

void Feed::run() {
    using namespace std::chrono_literals;
    Frame work;
    const std::size_t n = tiles_.size();
    for (std::size_t s = 0; s < n; s++) {
        work.img[s].assign(static_cast<std::size_t>(kW * kH), 0);
    }
    work.n = n;
    while (!stop_.load()) {
        auto cam = cam::Camera::open(spec_, 0, cam::ReplayOptions{loop_, true});
        if (!cam) {
            {
                std::lock_guard l(m_);
                error_ = "no camera picture (" + spec_ + "): " + cam.error().text();
            }
            for (int i = 0; i < 20 && !stop_.load(); i++) {
                std::this_thread::sleep_for(100ms);
            }
            continue;
        }
        (*cam)->verify_rows = false;
        while (!stop_.load()) {
            auto fr = (*cam)->next_frame(200);
            if (!fr) {
                std::lock_guard l(m_);
                error_ = "camera picture lost: " + fr.error().text();
                break;
            }
            if (!*fr) {
                if ((*cam)->source().finished()) {
                    break;
                }
                continue;
            }
            const cam::FrameInfo& info = **fr;
            if (info.kind != static_cast<std::uint32_t>(cam::FrameKind::Tracking)) {
                continue;
            }
            bool ok = true;
            for (std::size_t s = 0; s < n && ok; s++) {
                ok = (*cam)->copy_tile(tiles_[s], work.img[s], static_cast<std::size_t>(kW)).has_value();
            }
            if (!ok) {
                continue;
            }
            work.monoNs = info.mono_ts_ns != 0 ? info.mono_ts_ns : info.host_ts_ns;
            for (std::size_t s = 0; s < n; s++) {
                work.means[s] = meanOf(work.img[s]);
            }
            work.mean = n >= 2 ? 0.5F * (work.means[0] + work.means[1]) : work.means[0];
            work.seq = seq_.load() + 1;
            {
                std::lock_guard l(m_);
                std::swap(work.img, back_.img);
                back_.monoNs = work.monoNs;
                back_.seq = work.seq;
                back_.mean = work.mean;
                back_.means = work.means;
                back_.n = n;
                fresh_ = true;
                error_.clear();
            }
            seq_.store(work.seq);
            for (std::size_t s = 0; s < n; s++) {
                work.img[s].resize(static_cast<std::size_t>(kW * kH));
            }
        }
        if ((*cam)->source().finished() && !loop_) {
            return;
        }
    }
}

} // namespace pibridge::xrkit::pt
