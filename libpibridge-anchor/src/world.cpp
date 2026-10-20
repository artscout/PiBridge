// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/world.rs (MIT OR Apache-2.0). See world.h.
#include "pibridge/anchor/world.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"
#include "pibridge/ctrl/track/solve.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace pibridge::anchor {

namespace g = cam::geom;
namespace rf = cam::rfmt;

std::string runtime_dir() {
    if (auto v = cam::env_os("XDG_RUNTIME_DIR")) {
        return *v;
    }
    return "/tmp";
}

std::string ring_path() {
    if (auto v = cam::env_os("PIMAX_ANCHOR_HEAD_FILE"); v && !v->empty()) {
        return *v;
    }
    return cam::path_join(runtime_dir(), "pimax-anchor.head");
}

std::string offset_path() {
    if (auto v = cam::env_os("PIMAX_ANCHOR_FILE"); v && !v->empty()) {
        return *v;
    }
    return cam::path_join(runtime_dir(), "pimax-anchor.offset");
}

std::pair<Vec3, std::array<double, 4>> pq(const Pose& p) {
    return {p.t, g::quat_from_mat(p.r)};
}

Pose from_pq(const Vec3& t, const std::array<double, 4>& q) {
    return Pose{ctrl::track::quat_to_mat(q), t};
}

namespace {

std::array<double, 4> slerp(const std::array<double, 4>& a, std::array<double, 4> b, double f) {
    double d = -0.0;
    for (int i = 0; i < 4; i++) {
        d += a[i] * b[i];
    }
    if (d < 0.0) {
        d = -d;
        for (auto& x : b) {
            x = -x;
        }
    }
    double wa;
    double wb;
    if (d > 0.9995) {
        wa = 1.0 - f;
        wb = f;
    } else {
        const double th = std::acos(d);
        wa = std::sin((1.0 - f) * th) / std::sin(th);
        wb = std::sin(f * th) / std::sin(th);
    }
    std::array<double, 4> q{};
    for (int i = 0; i < 4; i++) {
        q[i] = wa * a[i] + wb * b[i];
    }
    double s = -0.0;
    for (double x : q) {
        s += x * x;
    }
    const double n = std::sqrt(s);
    for (auto& x : q) {
        x = x / n;
    }
    return q;
}

std::uint32_t u32at(std::string_view b, std::size_t o) {
    std::uint32_t v;
    std::memcpy(&v, b.data() + o, 4);
    return v;
}

double f32at(std::string_view b, std::size_t o) {
    float v;
    std::memcpy(&v, b.data() + o, 4);
    return static_cast<double>(v);
}

} // namespace

Pose interp(const Pose& a, const Pose& b, double f) {
    Vec3 t{};
    for (int i = 0; i < 3; i++) {
        t[i] = a.t[i] * (1.0 - f) + b.t[i] * f;
    }
    return from_pq(t, slerp(g::quat_from_mat(a.r), g::quat_from_mat(b.r), f));
}

std::optional<HeadRing> HeadRing::parse(std::string_view b) {
    if (b.size() < RING_SIZE) {
        return std::nullopt;
    }
    if (u32at(b, 0) != RING_MAGIC || u32at(b, 4) != 1) {
        return std::nullopt;
    }
    HeadRing r;
    for (std::size_t i = 0; i < RING_SLOTS; i++) {
        const std::size_t o = RING_HDR + i * RING_SLOT;
        std::uint64_t ts;
        std::memcpy(&ts, b.data() + o, 8);
        if (ts == 0) {
            continue;
        }
        const Vec3 t{f32at(b, o + 8), f32at(b, o + 12), f32at(b, o + 16)};
        const std::array<double, 4> q{f32at(b, o + 20), f32at(b, o + 24), f32at(b, o + 28), f32at(b, o + 32)};
        if (std::any_of(q.begin(), q.end(), [](double x) { return !std::isfinite(x); })) {
            continue;
        }
        r.samples.push_back(RingSample{ts, from_pq(t, q), (u32at(b, o + 36) & 1u) != 0});
    }
    std::stable_sort(r.samples.begin(), r.samples.end(),
                     [](const RingSample& x, const RingSample& y) { return x.ts_ns < y.ts_ns; });
    r.pid = u32at(b, 12);
    return r;
}

std::optional<HeadRing> HeadRing::read(const std::string& path) {
    for (int k = 0; k < 8; k++) {
        auto b = cam::read_file(path);
        if (!b) {
            return std::nullopt;
        }
        if (b->size() < RING_SIZE) {
            return std::nullopt;
        }
        const std::uint32_t seq = u32at(*b, 8);
        if ((seq & 1u) == 1u) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        auto b2 = cam::read_file(path);
        if (!b2) {
            return std::nullopt;
        }
        if (b2->size() >= 12 && std::memcmp(b2->data() + 8, b->data() + 8, 4) == 0) {
            return parse(*b);
        }
    }
    return std::nullopt;
}

std::uint64_t HeadRing::newest_ns() const {
    return samples.empty() ? 0 : samples.back().ts_ns;
}

std::optional<Pose> HeadRing::at(std::uint64_t ts_ns, std::uint64_t max_gap_ns) const {
    std::vector<const RingSample*> v;
    v.reserve(samples.size());
    for (const auto& s : samples) {
        if (s.valid) {
            v.push_back(&s);
        }
    }
    const std::size_t i = static_cast<std::size_t>(
        std::partition_point(v.begin(), v.end(), [&](const RingSample* s) { return s->ts_ns <= ts_ns; }) - v.begin());
    if (i > 0 && i < v.size()) {
        const RingSample* a = v[i - 1];
        const RingSample* b = v[i];
        if (b->ts_ns - a->ts_ns <= max_gap_ns) {
            const double f = static_cast<double>(ts_ns - a->ts_ns) /
                             static_cast<double>(std::max<std::uint64_t>(b->ts_ns - a->ts_ns, 1));
            return interp(a->pose, b->pose, f);
        }
    }
    const RingSample* best = nullptr;
    std::uint64_t bd = 0;
    for (const RingSample* s : v) {
        const std::uint64_t d = s->ts_ns > ts_ns ? s->ts_ns - ts_ns : ts_ns - s->ts_ns;
        if (best == nullptr || d < bd) {
            best = s;
            bd = d;
        }
    }
    if (best != nullptr && bd <= max_gap_ns / 2) {
        return best->pose;
    }
    return std::nullopt;
}

std::string ring_bytes(std::uint32_t pid, const std::vector<RingSample>& samples) {
    std::string b(RING_SIZE, '\0');
    auto put32 = [&](std::size_t o, std::uint32_t v) { std::memcpy(b.data() + o, &v, 4); };
    auto putf = [&](std::size_t o, float v) { std::memcpy(b.data() + o, &v, 4); };
    put32(0, RING_MAGIC);
    put32(4, 1);
    put32(12, pid);
    for (std::size_t i = 0; i < samples.size() && i < RING_SLOTS; i++) {
        const std::size_t o = RING_HDR + i * RING_SLOT;
        std::memcpy(b.data() + o, &samples[i].ts_ns, 8);
        const auto [t, q] = pq(samples[i].pose);
        for (int k = 0; k < 3; k++) {
            putf(o + 8 + 4 * k, static_cast<float>(t[k]));
        }
        for (int k = 0; k < 4; k++) {
            putf(o + 20 + 4 * k, static_cast<float>(q[k]));
        }
        put32(o + 36, samples[i].valid ? 1u : 0u);
    }
    return b;
}

Pose compute_offset(const Pose& reference, const Pose& world_from_tag_now, bool yaw_only) {
    const g::Mat3 r_full = g::mat_mul(reference.r, g::transpose(world_from_tag_now.r));
    g::Mat3 r = r_full;
    if (yaw_only) {
        const double psi = std::atan2(r_full[0][2] - r_full[2][0], r_full[0][0] + r_full[2][2]);
        r = g::rodrigues({0.0, psi, 0.0});
    }
    const Vec3 t = g::sub(reference.t, g::mat_vec(r, world_from_tag_now.t));
    return Pose{r, t};
}

Pose recenter_offset(const Pose& slam_from_head) {
    const double yaw = cam::slam::yaw_of(cam::slam::quat_of(slam_from_head.r));
    const g::Mat3 r = cam::slam::mat_of(cam::slam::qyaw(-yaw));
    const Vec3 h = g::mat_vec(r, slam_from_head.t);
    return Pose{r, {-h[0], 0.0, -h[2]}};
}

std::string offset_text(std::uint32_t pid, const Pose& o) {
    const auto [t, q] = pq(o);
    return "pimax-anchor-offset 1\npid " + std::to_string(pid) + "\npose " + rf::fixed(t[0], 6) + " " +
           rf::fixed(t[1], 6) + " " + rf::fixed(t[2], 6) + " " + rf::fixed(q[0], 7) + " " + rf::fixed(q[1], 7) + " " +
           rf::fixed(q[2], 7) + " " + rf::fixed(q[3], 7) + "\n";
}

std::optional<std::pair<std::uint32_t, Pose>> parse_offset(std::string_view s) {
    const auto l = rs::lines(s);
    if (l.empty() || rf::trim(l[0]) != "pimax-anchor-offset 1") {
        return std::nullopt;
    }
    if (l.size() < 2 || !l[1].starts_with("pid ")) {
        return std::nullopt;
    }
    std::uint32_t pid = 0;
    if (!rf::parse_u32(rf::trim(l[1].substr(4)), pid)) {
        return std::nullopt;
    }
    if (l.size() < 3 || !l[2].starts_with("pose ")) {
        return std::nullopt;
    }
    std::vector<double> v;
    for (auto w : rf::split_whitespace(l[2].substr(5))) {
        double x = 0;
        if (!rf::parse_f64(w, x)) {
            return std::nullopt;
        }
        v.push_back(x);
    }
    if (v.size() != 7) {
        return std::nullopt;
    }
    return std::pair{pid, from_pq({v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]})};
}

std::optional<std::pair<std::uint32_t, Pose>> read_offset(const std::string& path) {
    auto t = cam::read_to_string(path);
    if (!t) {
        return std::nullopt;
    }
    return parse_offset(*t);
}

namespace {

/// flock on `omarchyvr-anchor.offset.lock` next to the offset: lease decisions and offset writes are serialised.
class LeaseLock {
public:
    static cam::Result<LeaseLock> take(const std::string& offset) {
        const std::string p = rs::with_file_name(offset, "omarchyvr-anchor.offset.lock");
        const int fd = ::open(p.c_str(), O_WRONLY | O_CREAT | O_CLOEXEC, 0666);
        if (fd < 0) {
            return cam::fail(cam::IoError::last_os());
        }
        int r;
        do {
            r = ::flock(fd, LOCK_EX);
        } while (r != 0 && errno == EINTR);
        if (r != 0) {
            const auto e = cam::IoError::last_os();
            ::close(fd);
            return cam::fail(e);
        }
        return LeaseLock(fd);
    }
    LeaseLock(LeaseLock&& o) noexcept : fd_(o.fd_) { o.fd_ = -1; }
    LeaseLock(const LeaseLock&) = delete;
    LeaseLock& operator=(const LeaseLock&) = delete;
    LeaseLock& operator=(LeaseLock&&) = delete;
    ~LeaseLock() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

private:
    explicit LeaseLock(int fd) : fd_(fd) {}
    int fd_;
};

bool lease_held_by(const std::optional<Lease>& cur, std::uint32_t session, std::uint32_t me) {
    return cur && cur->session == session && cur->writer == me && cur->schema == LEASE_SCHEMA;
}

std::string refused_text(const std::optional<Lease>& cur, std::uint32_t session) {
    if (cur && cur->session == session) {
        return "the room offset of this session (Monado pid " + std::to_string(session) + ") belongs to " +
               cur->holder_title() + "; not written";
    }
    return "no writer lease for the room offset of Monado pid " + std::to_string(session) + "; not written";
}

/// Write `text` to `tmp` and rename it to `dst`.
cam::Result<void> write_rename(const std::string& tmp, const std::string& dst, std::string_view text) {
    if (auto w = cam::write_file(tmp, text); !w) {
        return w;
    }
    if (::rename(tmp.c_str(), dst.c_str()) != 0) {
        return cam::fail(cam::IoError::last_os());
    }
    return {};
}

std::string comm_of(std::uint32_t pid) {
    auto t = cam::read_to_string("/proc/" + std::to_string(pid) + "/comm");
    return t ? std::string(rf::trim(*t)) : std::string();
}

/// End an evicted writer by pid: only our own offset writers (comm checked), SIGTERM, wait up to 3 s.
std::expected<void, std::string> end_writer(std::uint32_t pid) {
    const std::string c = comm_of(pid);
    if (!is_offset_writer_comm(c)) {
        return std::unexpected("pid " + std::to_string(pid) + " (" + c +
                               ") holds the room offset lease but is not an offset writer; not ended");
    }
    ::kill(static_cast<pid_t>(pid), SIGTERM);
    for (int i = 0; i < 30; i++) {
        if (!pid_alive(pid) || comm_of(pid) != c) {
            return {};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return std::unexpected("pid " + std::to_string(pid) + " (" + c + ") did not stop within 3 s");
}

} // namespace

cam::Result<void> write_offset(const std::string& path, std::uint32_t pid, const Pose& o) {
    return write_offset_as(path, pid, rs::process_id(), o);
}

cam::Result<void> write_offset_as(const std::string& path, std::uint32_t pid, std::uint32_t me, const Pose& o) {
    auto g_ = LeaseLock::take(path);
    if (!g_) {
        return cam::fail(g_.error());
    }
    const auto cur = read_lease(lease_path_for(path));
    if (!lease_held_by(cur, pid, me)) {
        return cam::fail(cam::IoError::custom(cam::ErrorKind::PermissionDenied, refused_text(cur, pid)));
    }
    return write_rename(rs::with_extension(path, "offset.tmp"), path, offset_text(pid, o));
}

std::pair<double, double> pose_error(const Pose& a, const Pose& b) {
    const double dt = g::norm(g::sub(a.t, b.t));
    const g::Mat3 rel = g::mat_mul(g::transpose(a.r), b.r);
    const double c = rf::clamp((rel[0][0] + rel[1][1] + rel[2][2] - 1.0) / 2.0, -1.0, 1.0);
    return {dt, rf::to_degrees(std::acos(c))};
}

std::optional<std::tuple<Pose, std::size_t, double, double>> robust_mean(const std::vector<Pose>& v, double pos_tol,
                                                                         double rot_tol_deg) {
    (void)rot_tol_deg;
    if (v.empty()) {
        return std::nullopt;
    }
    auto med = [&](int k) {
        std::vector<double> x;
        x.reserve(v.size());
        for (const auto& p : v) {
            x.push_back(p.t[k]);
        }
        if (x.size() > 1 && std::any_of(x.begin(), x.end(), [](double y) { return std::isnan(y); })) {
            // Rust: sort_by(partial_cmp().unwrap()) meets the NaN
            rs::panic("omarchyvr-anchor/src/world.rs", "called `Option::unwrap()` on a `None` value");
        }
        std::stable_sort(x.begin(), x.end());
        return x[x.size() / 2];
    };
    const Vec3 c{med(0), med(1), med(2)};
    std::vector<const Pose*> keep;
    for (const auto& p : v) {
        if (g::norm(g::sub(p.t, c)) <= pos_tol) {
            keep.push_back(&p);
        }
    }
    if (keep.empty()) {
        return std::nullopt;
    }
    Vec3 mean_t{0.0, 0.0, 0.0};
    std::array<double, 4> qs{0.0, 0.0, 0.0, 0.0};
    const auto q0 = g::quat_from_mat(keep[0]->r);
    const double n_keep = static_cast<double>(keep.size());
    for (const Pose* p : keep) {
        for (int i = 0; i < 3; i++) {
            mean_t[i] += p->t[i] / n_keep;
        }
        auto q = g::quat_from_mat(p->r);
        double d = -0.0;
        for (int i = 0; i < 4; i++) {
            d += q[i] * q0[i];
        }
        if (d < 0.0) {
            for (auto& x : q) {
                x = -x;
            }
        }
        for (int i = 0; i < 4; i++) {
            qs[i] += q[i];
        }
    }
    double s = -0.0;
    for (double x : qs) {
        s += x * x;
    }
    const double n = std::sqrt(s);
    std::array<double, 4> qn{};
    for (int i = 0; i < 4; i++) {
        qn[i] = qs[i] / n;
    }
    const Pose mean = from_pq(mean_t, qn);
    double st = -0.0;
    double sr = -0.0;
    for (const Pose* p : keep) {
        const double e = g::norm(g::sub(p->t, mean_t));
        st += e * e;
        const double r = pose_error(mean, *p).second;
        sr += r * r;
    }
    return std::tuple{mean, keep.size(), std::sqrt(st / n_keep), std::sqrt(sr / n_keep)};
}

// ---------------------------------------------------------------------------------------------------------------
// The lease.

const char* role_name(Role r) {
    switch (r) {
    case Role::Setup: return "setup";
    case Role::Auto: return "auto";
    case Role::Follow: return "follow";
    case Role::Reloc: return "reloc";
    case Role::Apply: return "apply";
    case Role::Tag: return "tag";
    }
    return "";
}

std::optional<Role> role_parse(std::string_view s) {
    for (Role r : {Role::Setup, Role::Auto, Role::Follow, Role::Reloc, Role::Apply, Role::Tag}) {
        if (s == role_name(r)) {
            return r;
        }
    }
    return std::nullopt;
}

const char* role_kind(Role r) {
    return r == Role::Tag ? "anchor" : "roommap";
}

const char* role_title(Role r) {
    switch (r) {
    case Role::Setup: return "the room setup";
    case Role::Auto: return "the automatic room placement";
    case Role::Follow: return "the floor follow";
    case Role::Reloc: return "Recognise room";
    case Role::Apply: return "a saved room fix";
    case Role::Tag: return "the AprilTag";
    }
    return "";
}

bool role_preempts(Role r) {
    return r == Role::Setup || r == Role::Reloc || r == Role::Apply || r == Role::Tag;
}

bool role_first_only(Role r) {
    return r == Role::Auto;
}

std::string Lease::text() const {
    std::string t = kind + " " + std::to_string(session) + "\nlease " + std::to_string(schema) + "\nwriter " +
                    std::to_string(writer) + "\n";
    if (role) {
        t += std::string("role ") + role_name(*role) + "\n";
    }
    return t;
}

std::optional<Lease> Lease::parse(std::string_view s) {
    const auto ls = rs::lines(s);
    if (ls.empty()) {
        return std::nullopt;
    }
    const auto w = rf::split_whitespace(ls[0]);
    if (w.size() < 2) {
        return std::nullopt;
    }
    Lease l;
    l.kind = std::string(w[0]);
    if (!rf::parse_u32(w[1], l.session)) {
        return std::nullopt;
    }
    for (std::size_t i = 1; i < ls.size(); i++) {
        const auto kv = rs::split_once(rf::trim(ls[i]), ' ');
        if (!kv) {
            continue;
        }
        if (kv->first == "lease") {
            if (!rf::parse_u32(rf::trim(kv->second), l.schema)) {
                return std::nullopt;
            }
        } else if (kv->first == "writer") {
            if (!rf::parse_u32(rf::trim(kv->second), l.writer)) {
                return std::nullopt;
            }
        } else if (kv->first == "role") {
            l.role = role_parse(rf::trim(kv->second));
        }
    }
    if (l.schema > LEASE_SCHEMA) {
        l.writer = 0;
        l.role = std::nullopt;
    }
    return l;
}

std::string Lease::holder_title() const {
    if (role) {
        return std::string(role_title(*role)) + " (pid " + std::to_string(writer) + ")";
    }
    if (kind == "anchor") {
        return "the AprilTag";
    }
    return "the room map";
}

std::string lease_path_for(const std::string& offset) {
    return rs::with_file_name(offset, "omarchyvr-anchor.offset.owner");
}

std::string lease_path() {
    return lease_path_for(offset_path());
}

std::optional<Lease> read_lease(const std::string& path) {
    auto t = cam::read_to_string(path);
    if (!t) {
        return std::nullopt;
    }
    return Lease::parse(*t);
}

std::expected<std::optional<std::uint32_t>, std::string> decide_lease(const std::optional<Lease>& cur_in,
                                                                      std::uint32_t session, std::uint32_t me,
                                                                      Role role, bool offset_written,
                                                                      const std::function<bool(std::uint32_t)>& alive) {
    if (!cur_in || cur_in->session != session) {
        return std::optional<std::uint32_t>{};
    }
    const Lease& cur = *cur_in;
    if (cur.writer == me && cur.writer != 0) {
        return std::optional<std::uint32_t>{};
    }
    if (cur.writer != 0 && alive(cur.writer)) {
        if (role_preempts(role)) {
            return std::optional<std::uint32_t>{cur.writer};
        }
        return std::unexpected(cur.holder_title() + " writes the room offset of this session; " + role_title(role) +
                               " leaves it alone");
    }
    if (offset_written && cur.kind != role_kind(role)) {
        return std::unexpected("this session's room offset came from " + cur.holder_title() +
                               ": only one of the room map and the AprilTag per session. Press Clear offset on the "
                               "Room page first.");
    }
    if (offset_written && role_first_only(role)) {
        return std::unexpected(cur.holder_title() + " already placed the room in this session; " + role_title(role) +
                               " leaves it alone");
    }
    return std::optional<std::uint32_t>{};
}

std::expected<std::optional<std::string>, std::string> take_lease(const std::string& offset, std::uint32_t session,
                                                                  Role role) {
    return take_lease_as(offset, session, rs::process_id(), role, pid_alive, end_writer);
}

std::expected<std::optional<std::string>, std::string> take_lease_as(
    const std::string& offset, std::uint32_t session, std::uint32_t me, Role role,
    const std::function<bool(std::uint32_t)>& alive,
    const std::function<std::expected<void, std::string>(std::uint32_t)>& evict) {
    const std::string lp = lease_path_for(offset);
    auto g_ = LeaseLock::take(offset);
    if (!g_) {
        return std::unexpected("cannot lock the room offset lease: " + g_.error().text());
    }
    const auto cur = read_lease(lp);
    if (lease_held_by(cur, session, me)) {
        return std::optional<std::string>{};
    }
    const auto off = read_offset(offset);
    const bool written = off && off->first == session;
    const auto d = decide_lease(cur, session, me, role, written, alive);
    if (!d) {
        return std::unexpected(d.error());
    }
    std::optional<std::string> note;
    if (*d) {
        const std::string who = cur ? cur->holder_title() : std::string();
        if (auto e = evict(**d); !e) {
            return std::unexpected(e.error());
        }
        note = std::string(role_title(role)) + " took the room offset from " + who + ", which was ended";
    }
    const Lease l{role_kind(role), session, me, role, LEASE_SCHEMA};
    if (auto w = write_rename(rs::with_extension(lp, "owner.tmp"), lp, l.text()); !w) {
        return std::unexpected("cannot write " + lp + ": " + w.error().text());
    }
    return note;
}

bool holds_lease(const std::string& offset, std::uint32_t session) {
    return lease_held_by(read_lease(lease_path_for(offset)), session, rs::process_id());
}

bool holds_lease_as(const std::string& offset, std::uint32_t session, std::uint32_t me) {
    return lease_held_by(read_lease(lease_path_for(offset)), session, me);
}

bool is_offset_writer_comm(std::string_view c) {
    // comm is the first 15 bytes of the executable name
    return c == "omarchyvr-roomm" || c == "omarchyvr-ancho" || c == "pibridge-roommap" || c == "pibridge-anchor";
}

// ---------------------------------------------------------------------------------------------------------------
// The feed.

std::vector<std::uint32_t> pids_named(std::string_view name) {
    std::vector<std::uint32_t> v;
    DIR* d = ::opendir("/proc");
    if (d == nullptr) {
        return v;
    }
    while (const dirent* e = ::readdir(d)) {
        std::uint32_t pid = 0;
        if (!rf::parse_u32(e->d_name, pid)) {
            continue;
        }
        auto c = cam::read_to_string("/proc/" + std::to_string(pid) + "/comm");
        if (c && rf::trim(*c) == name) {
            v.push_back(pid);
        }
    }
    ::closedir(d);
    std::sort(v.begin(), v.end());
    return v;
}

std::vector<std::uint32_t> tracker_pids() {
    auto m = pids_named("monado-service");
    if (m.empty()) {
        return pids_named("vrserver");
    }
    return m;
}

bool pid_alive(std::uint32_t pid) {
    return pid != 0 && cam::path_exists("/proc/" + std::to_string(pid));
}

std::optional<std::uint32_t> Feed::live_pid() const {
    if (kind == Kind::Live) {
        return pid;
    }
    return std::nullopt;
}

std::string Feed::plain() const {
    switch (kind) {
    case Kind::Missing:
        return "Monado's head-pose feed is missing (Monado does not run with 6DoF and the room anchor; start Work from "
               "the app)";
    case Kind::NoMonado:
        return "Monado is not running (the head-pose feed is left over from pid " + std::to_string(pid) +
               "); start Work from the app";
    case Kind::Foreign:
        return "the head-pose feed was taken over by another program (pid " + std::to_string(pid) +
               (alive ? ", still running" : ", already closed") +
               ", e.g. SteamVR's driver opening the headset) and is not the running Monado's (pid " +
               std::to_string(monado) + "); restart Work from the app to get it back";
    case Kind::Stale:
        if (age_s) {
            return "Monado's head-pose feed (pid " + std::to_string(pid) + ") stopped " + rf::fixed(*age_s, 0) +
                   " s ago (6DoF lost or the headset asleep?)";
        }
        return "Monado's head-pose feed (pid " + std::to_string(pid) + ") has no poses yet (6DoF not tracking?)";
    case Kind::Live:
        return "Monado's head-pose feed is live (pid " + std::to_string(pid) + ")";
    }
    return "";
}

Feed check_feed(const std::optional<HeadRing>& ring, const std::vector<std::uint32_t>& monado, std::uint64_t now_ns,
                const std::function<bool(std::uint32_t)>& alive) {
    Feed f;
    if (!ring) {
        return f;
    }
    f.pid = ring->pid;
    if (std::find(monado.begin(), monado.end(), ring->pid) == monado.end()) {
        if (monado.empty()) {
            f.kind = Feed::Kind::NoMonado;
            return f;
        }
        f.kind = Feed::Kind::Foreign;
        f.monado = monado.front();
        f.alive = alive(ring->pid);
        return f;
    }
    const std::uint64_t newest = ring->newest_ns();
    if (newest == 0) {
        f.kind = Feed::Kind::Stale;
        return f;
    }
    const double age = static_cast<double>(now_ns > newest ? now_ns - newest : 0) / 1e9;
    if (age > RING_STALE_S) {
        f.kind = Feed::Kind::Stale;
        f.age_s = age;
        return f;
    }
    f.kind = Feed::Kind::Live;
    return f;
}

Feed feed_now() {
    return check_feed(HeadRing::read(ring_path()), tracker_pids(), rs::monotonic_ns(), pid_alive);
}

} // namespace pibridge::anchor
