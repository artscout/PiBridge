// SPDX-License-Identifier: MIT
#include "world.h"

#include <filesystem>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::headtrack {

namespace slam = cam::slam;
namespace geom = cam::geom;

namespace {

constexpr std::uint32_t MAGIC = 0x434E4150u;
constexpr std::size_t SLOTS = 128, HDR = 16, SLOT = 40, SIZE = HDR + SLOTS * SLOT;
constexpr double SLEW_M_S = 0.05, SLEW_DEG_S = 2.0;

// The anchor files of the SteamVR backend (libpibridge-anchor backend.h; pibridge-headtrack is that backend by
// definition): <runtime>/pibridge/steamvr/<leaf>, the explicit PIMAX_ANCHOR_* file first. Monado's files are its own.
std::string path_of(const char* env, const char* leaf) {
    if (const char* e = std::getenv(env); e != nullptr && *e != '\0') {
        return e;
    }
    const char* d = std::getenv("XDG_RUNTIME_DIR");
    const std::string dir = std::string(d != nullptr && *d != '\0' ? d : "/tmp") + "/pibridge/steamvr";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir + "/" + leaf;
}

bool pid_alive(std::uint32_t pid) {
    return pid != 0 && pid <= static_cast<std::uint32_t>(INT32_MAX) &&
           (::kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM);
}

std::int64_t monotonic_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
}

/// Another live process keeping the ring at `path` fresh (newest sample at most 10 s old), else 0.
std::uint32_t live_owner(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return 0;
    }
    std::uint8_t buf[SIZE];
    const ssize_t n = ::pread(fd, buf, sizeof buf, 0);
    ::close(fd);
    if (n != static_cast<ssize_t>(sizeof buf)) {
        return 0;
    }
    std::uint32_t hdr[4];
    std::memcpy(hdr, buf, sizeof hdr);
    if (hdr[0] != MAGIC || hdr[1] != 1 || hdr[3] == static_cast<std::uint32_t>(::getpid()) || !pid_alive(hdr[3])) {
        return 0;
    }
    std::uint64_t newest = 0;
    for (std::size_t i = 0; i < SLOTS; i++) {
        std::uint64_t ts;
        std::memcpy(&ts, buf + HDR + i * SLOT, 8);
        newest = std::max(newest, ts);
    }
    return newest == 0 || monotonic_ns() - static_cast<std::int64_t>(newest) > 10'000'000'000 ? 0 : hdr[3];
}

double quat_angle(const Quat& a, const Quat& b) {
    const double d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    return 2.0 * std::acos(std::min(d, 1.0));
}

Quat slerp(const Quat& a, Quat b, double t) {
    double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (d < 0) {
        d = -d;
        for (double& c : b) {
            c = -c;
        }
    }
    if (d > 0.9995) {
        return slam::qnorm({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2]),
                            a[3] + t * (b[3] - a[3])});
    }
    const double th = std::acos(d), s = std::sin(th);
    const double wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    return slam::qnorm({wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2], wa * a[3] + wb * b[3]});
}

} // namespace

RigidPose compose(const RigidPose& a, const RigidPose& b) {
    return {geom::add(slam::qrot(a.q, b.p), a.p), slam::qnorm(slam::qmul(a.q, b.q))};
}

World::World() : ring_path_(path_of("PIMAX_ANCHOR_HEAD_FILE", "anchor.head")),
                 offset_path_(path_of("PIMAX_ANCHOR_FILE", "anchor.offset")) {
    // When the floor is carried by the anchor (apply_vertical_), hold it: placed at once, NO glide (step() snaps y),
    // changed only when a new offset is written (setup / the long-press recenter), never chased live so crouching
    // cannot move it. Default OFF: today the chaperone's universe carries the floor (its standing translation y), so
    // applying it here too would double it (the old "2 m above the floor" bug). Moving the floor onto this held anchor
    // needs the chaperone written floor-neutral in the same change -- see the plan. PIMAX_ANCHOR_VERTICAL=1 to try it.
    apply_vertical_ = false;
    if (const char* e = std::getenv("PIMAX_ANCHOR_VERTICAL"); e != nullptr && e[0] == '1') {
        apply_vertical_ = true;
    }
    if (live_owner(ring_path_) != 0) {
        return; // another live tracker owns the room anchor: its ring and offset stay (room anchor off here)
    }
    ::unlink(offset_path_.c_str()); // the tracker world is new: an old offset would be wrong
    create_ring();
}

World::~World() {
    if (ring_ != nullptr) {
        ::munmap(ring_, SIZE);
    }
}

void World::create_ring() {
    const std::string tmp = ring_path_ + "." + std::to_string(::getpid()) + ".tmp";
    const int fd = ::open(tmp.c_str(), O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0 || ::ftruncate(fd, SIZE) != 0) {
        if (fd >= 0) {
            ::close(fd);
            ::unlink(tmp.c_str());
        }
        return;
    }
    struct stat st{};
    void* p = ::fstat(fd, &st) == 0 ? ::mmap(nullptr, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0) : MAP_FAILED;
    ::close(fd);
    if (p == MAP_FAILED) {
        ::unlink(tmp.c_str());
        return;
    }
    const std::uint32_t hdr[4] = {MAGIC, 1, 0, static_cast<std::uint32_t>(::getpid())};
    std::memcpy(p, hdr, sizeof hdr);
    if (::rename(tmp.c_str(), ring_path_.c_str()) != 0) {
        ::munmap(p, SIZE);
        ::unlink(tmp.c_str());
        return;
    }
    if (ring_ != nullptr) {
        ::munmap(ring_, SIZE);
    }
    ring_ = static_cast<std::uint8_t*>(p);
    ring_dev_ = st.st_dev;
    ring_ino_ = st.st_ino;
    next_ = 0;
    last_ring_ns_ = 0;
}

void World::keep_ring(std::int64_t now_ns) {
    // once a second: the path is still our ring; take it back when it was replaced or removed and nobody keeps it
    if (now_ns - last_ring_check_ns_ < 1'000'000'000) {
        return;
    }
    last_ring_check_ns_ = now_ns;
    struct stat st{};
    if (::stat(ring_path_.c_str(), &st) == 0 && st.st_dev == ring_dev_ && st.st_ino == ring_ino_) {
        return;
    }
    if (live_owner(ring_path_) == 0) {
        create_ring();
    }
}

void World::record(std::int64_t ts_ns, const RigidPose& raw, bool position_valid) {
    const std::int64_t d = ts_ns - last_ring_ns_;
    if (ring_ == nullptr || ts_ns <= 0 || (d < 3'000'000 && d > -3'000'000)) {
        return;
    }
    last_ring_ns_ = ts_ns;
    std::uint8_t* slot = ring_ + HDR + static_cast<std::size_t>(next_++ % SLOTS) * SLOT;
    auto* seq = reinterpret_cast<std::atomic<std::uint32_t>*>(ring_ + 8);
    const std::uint32_t q0 = seq->load(std::memory_order_relaxed);
    seq->store(q0 + 1, std::memory_order_release);
    const auto ts = static_cast<std::uint64_t>(ts_ns);
    const float f[7] = {static_cast<float>(raw.p[0]), static_cast<float>(raw.p[1]), static_cast<float>(raw.p[2]),
                        static_cast<float>(raw.q[0]), static_cast<float>(raw.q[1]), static_cast<float>(raw.q[2]),
                        static_cast<float>(raw.q[3])};
    const std::uint32_t flags = position_valid ? 1u : 0u;
    std::memcpy(slot, &ts, 8);
    std::memcpy(slot + 8, f, 28);
    std::memcpy(slot + 36, &flags, 4);
    seq->store(q0 + 2, std::memory_order_release);
}

void World::refresh_offset(std::int64_t now_ns) {
    if (now_ns - last_check_ns_ < 250'000'000) {
        return;
    }
    last_check_ns_ = now_ns;
    auto cleared = [this] {
        const bool had = have_offset_;
        have_offset_ = false;
        have_applied_ = false;
        if (placed_) {
            // cleared (Room page): the raw tracker world now; a later offset glides from there
            applied_ = RigidPose{};
            have_applied_ = true;
        }
        if (had) {
            say("floor/anchor: offset cleared -> raw tracker world (no vertical/yaw shift)");
        }
    };
    struct stat st{};
    if (::stat(offset_path_.c_str(), &st) != 0) {
        mtime_s_ = 0;
        cleared();
        return;
    }
    if (have_offset_ && st.st_mtim.tv_sec == mtime_s_ && st.st_mtim.tv_nsec == mtime_ns_) {
        return;
    }
    mtime_s_ = st.st_mtim.tv_sec;
    mtime_ns_ = st.st_mtim.tv_nsec;
    std::ifstream f(offset_path_);
    std::string l1, l2, l3;
    unsigned pid = 0;
    double v[7];
    const bool ok = std::getline(f, l1) && l1.rfind("pimax-anchor-offset 1", 0) == 0 && std::getline(f, l2) &&
                    std::sscanf(l2.c_str(), "pid %u", &pid) == 1 && std::getline(f, l3) &&
                    std::sscanf(l3.c_str(), "pose %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4],
                                &v[5], &v[6]) == 7;
    if (!ok || pid != static_cast<unsigned>(::getpid())) {
        cleared();
        return;
    }
    offset_ = RigidPose{{v[0], v[1], v[2]}, slam::qnorm({v[3], v[4], v[5], v[6]})};
    have_offset_ = true;
    generation_++;
    if (log_) {
        const Vec3 fwd = slam::qrot(offset_.q, Vec3{0, 0, -1});
        const double yaw = std::atan2(-fwd[0], -fwd[2]) * 180.0 / M_PI;
        char b[260];
        std::snprintf(b, sizeof b,
                      "floor/anchor: new offset from %s: p(%.3f %.3f %.3f) m, yaw %.1f deg (gen %u) -- vertical y=%+.3f m %s",
                      offset_path_.c_str(), offset_.p[0], offset_.p[1], offset_.p[2], yaw, generation_, offset_.p[1],
                      apply_vertical_ ? "(applied)" : "(IGNORED: floor from chaperone)");
        say(b);
    }
}

void World::step(std::int64_t now_ns, double head_dps) {
    double dt = last_step_ns_ > 0 ? static_cast<double>(now_ns - last_step_ns_) * 1e-9 : 0.0;
    last_step_ns_ = now_ns;
    dt = std::clamp(dt, 0.0, 0.1);
    if (!have_applied_) {
        applied_ = offset_; // the first offset of the session: placed at once
        have_applied_ = true;
        placed_ = true;
        say("floor/anchor: offset placed at once (y=" + std::to_string(applied_.p[1]) + " m, no glide)");
        return;
    }
    // faster while the head turns: 1x up to 20 deg/s, 3x from 60 deg/s
    const double boost = 1.0 + 2.0 * std::clamp((head_dps - 20.0) / 40.0, 0.0, 1.0);
    const Vec3 dp = geom::sub(offset_.p, applied_.p);
    const double dist = geom::norm(dp);
    const double max_d = SLEW_M_S * boost * dt;
    applied_.p = dist <= max_d ? offset_.p : geom::add(applied_.p, geom::scale(dp, max_d / dist));
    const double ang = quat_angle(applied_.q, offset_.q);
    const double max_a = SLEW_DEG_S * boost * M_PI / 180.0 * dt;
    applied_.q = ang <= max_a || ang < 1e-6 ? offset_.q : slerp(applied_.q, offset_.q, max_a / ang);
    if (apply_vertical_) {
        applied_.p[1] = offset_.p[1]; // the floor (y) is held: snap to target, never glide; only x/z glide
    }
    // diagnostics: a glide is the floor visibly moving. Log it rate-limited while it runs, once when it settles.
    const double gap = geom::norm(geom::sub(offset_.p, applied_.p));
    if (gap > 1e-4 || quat_angle(applied_.q, offset_.q) > 1e-4) {
        gliding_ = true;
        if (log_ && now_ns - last_glide_log_ns_ > 500'000'000) {
            last_glide_log_ns_ = now_ns;
            char b[220];
            std::snprintf(b, sizeof b,
                          "floor/anchor: gliding -- applied y=%.3f -> target y=%.3f (gap %.3f m, %.1fx, head %.0f deg/s)",
                          applied_.p[1], offset_.p[1], gap, boost, head_dps);
            say(b);
        }
    } else if (gliding_) {
        gliding_ = false;
        say("floor/anchor: glide settled at y=" + std::to_string(applied_.p[1]) + " m");
    }
}

RigidPose World::update(std::int64_t ts_ns, const RigidPose& raw, bool position_valid, double head_dps,
                        std::int64_t now_ns) {
    keep_ring(now_ns);
    record(ts_ns, raw, position_valid);
    refresh_offset(now_ns);
    if (have_offset_) {
        step(now_ns, head_dps);
    }
    const RigidPose off = have_applied_ ? applied_ : RigidPose{};
    // a new room offset (recognised or set up) replaces an earlier recenter: it would shift the room
    if (generation_ != seen_generation_) {
        seen_generation_ = generation_;
        const bool had = recenter_.p[0] != 0 || recenter_.p[1] != 0 || recenter_.p[2] != 0;
        recenter_ = RigidPose{};
        if (had) {
            say("floor/anchor: recenter cleared by a new room offset");
        }
    }
    if (recenter_wanted_.exchange(false)) {
        // the head now: its forward (-Z) onto -Z, its position over the origin, the floor kept
        const RigidPose w = compose(recenter_, compose(off, raw));
        const Vec3 fwd = slam::qrot(w.q, Vec3{0, 0, -1});
        const double yaw = std::atan2(-fwd[0], -fwd[2]);
        RigidPose t;
        t.q = {0, std::sin(-yaw / 2), 0, std::cos(-yaw / 2)};
        t.p = slam::qrot(t.q, Vec3{-w.p[0], 0, -w.p[2]});
        recenter_ = compose(t, recenter_);
        char b[200];
        std::snprintf(b, sizeof b, "floor/anchor: recenter applied -- yaw %.1f deg, origin over head, floor kept",
                      yaw * 180.0 / M_PI);
        say(b);
    }
    RigidPose out = compose(recenter_, off);
    if (!apply_vertical_) {
        // the anchor's vertical floor shift is disabled: keep the raw head height, let SteamVR's chaperone set the floor
        out.p[1] = 0.0;
    }
    // floor diagnostic (~1 Hz): the raw head height (SLAM), the stored offset vertical, what we apply, and the final
    // head y we hand out. Shows whether the start drift is the raw SLAM y settling, and whether applying the anchor
    // floor would double with the chaperone's own standing height.
    if (log_ && now_ns - last_floor_log_ns_ > 1'000'000'000) {
        last_floor_log_ns_ = now_ns;
        char b[240];
        std::snprintf(b, sizeof b,
                      "floor diag: raw_y=%+.3f offset_y=%+.3f applied_y=%+.3f out_head_y=%+.3f have_offset=%d apply_vert=%d",
                      raw.p[1], offset_.p[1], applied_.p[1], raw.p[1] + out.p[1], have_offset_ ? 1 : 0,
                      apply_vertical_ ? 1 : 0);
        say(b);
    }
    return out;
}

} // namespace pibridge::headtrack
