// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/imulog.rs (MIT OR Apache-2.0).
#include "pibridge/cam/imulog.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <format>
#include <limits>
#include <mutex>
#include <sys/stat.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <variant>

namespace pibridge::cam::imulog {

namespace rf = pibridge::cam::rfmt;

namespace {

struct ImuMsg {
    std::int64_t t;
    std::array<double, 3> a;
    std::array<double, 3> g;
};
struct VioMsg {
    std::int64_t t;
    std::array<double, 3> p;
    std::array<double, 4> q;
};
using Msg = std::variant<ImuMsg, VioMsg>;

std::int64_t sat_sub(std::int64_t a, std::int64_t b) {
    std::int64_t r = 0;
    if (__builtin_sub_overflow(a, b, &r)) {
        return b < 0 ? std::numeric_limits<std::int64_t>::max() : std::numeric_limits<std::int64_t>::min();
    }
    return r;
}

// Rust `x as i64` for an f64: saturating, NaN -> 0.
std::int64_t f64_to_i64(double x) {
    if (std::isnan(x)) {
        return 0;
    }
    if (x <= -9223372036854775808.0) {
        return std::numeric_limits<std::int64_t>::min();
    }
    if (x >= 9223372036854775808.0) {
        return std::numeric_limits<std::int64_t>::max();
    }
    return static_cast<std::int64_t>(x);
}

// Rust `s.parse::<i64>()`.
bool parse_i64(std::string_view s, std::int64_t& out) {
    bool neg = false;
    if (!s.empty() && (s[0] == '+' || s[0] == '-')) {
        neg = s[0] == '-';
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return false;
    }
    std::int64_t v = 0;
    for (const char c : s) {
        if (c < '0' || c > '9') {
            return false;
        }
        const int d = c - '0';
        if (__builtin_mul_overflow(v, 10, &v) || (neg ? __builtin_sub_overflow(v, d, &v) : __builtin_add_overflow(v, d, &v))) {
            return false;
        }
    }
    out = v;
    return true;
}

std::string file_name(const std::string& p) {
    std::string s = p;
    while (s.size() > 1 && s.back() == '/') {
        s.pop_back();
    }
    const auto i = s.rfind('/');
    return i == std::string::npos ? s : s.substr(i + 1);
}

struct Mtime {
    std::int64_t sec = 0;
    std::int64_t nsec = 0;
    auto operator<=>(const Mtime&) const = default;
};

std::optional<Mtime> mtime_of(const std::string& p) {
    struct stat st{};
    if (::stat(p.c_str(), &st) != 0) {
        return std::nullopt;
    }
    return Mtime{st.st_mtim.tv_sec, st.st_mtim.tv_nsec};
}

// Local time stamp YYYYmmdd-HHMMSS of a file's modification time (localtime_r).
std::optional<std::string> stamp_of(const std::string& path) {
    const auto m = mtime_of(path);
    if (!m || m->sec < 0) {
        return std::nullopt;
    }
    const time_t secs = static_cast<time_t>(m->sec);
    struct tm tm{};
    if (::localtime_r(&secs, &tm) == nullptr) {
        return std::nullopt;
    }
    return std::format("{:04}{:02}{:02}-{:02}{:02}{:02}", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                       tm.tm_min, tm.tm_sec);
}

// dir/name.csv.prev for dir/name.csv.
std::string prev_link(const std::string& path) {
    return path + ".prev";
}

// Point link at target (a symlink; relative when they share a directory).
void relink(const std::string& link, const std::string& target) {
    const std::string t = path_parent(link) == path_parent(target) ? file_name(target) : target;
    ::unlink(link.c_str());
    if (!t.empty()) {
        if (::symlink(t.c_str(), link.c_str()) != 0) {
            // let _ = symlink(..): a failed link only loses the convenience pointer
        }
    }
}

// Step 1 of keeping a log (instant, same directory): rename a non-empty dir/name.csv to dir/name.<stamp>.keeping and
// point name.csv.prev at it. Returns the staged file.
std::optional<std::string> stage(const std::string& path) {
    struct stat st{};
    if (!is_file(path) || ::stat(path.c_str(), &st) != 0 || st.st_size == 0) {
        return std::nullopt;
    }
    const auto dir = path_parent(path);
    if (!dir) {
        return std::nullopt;
    }
    const std::string name = file_name(path);
    if (name.empty() || name == ".." || !rf::utf8_valid(name)) {
        return std::nullopt;
    }
    const std::string stem = name.ends_with(".csv") ? name.substr(0, name.size() - 4) : name;
    const auto stamp = stamp_of(path);
    if (!stamp) {
        return std::nullopt;
    }
    std::string staged = path_join(*dir, std::format("{}.{}.keeping", stem, *stamp));
    int n = 1;
    while (path_exists(staged)) {
        staged = path_join(*dir, std::format("{}.{}-{}.keeping", stem, *stamp, n));
        n += 1;
    }
    if (::rename(path.c_str(), staged.c_str()) != 0) {
        return std::nullopt;
    }
    relink(prev_link(path), staged);
    return staged;
}

// std::fs::copy: the bytes and the permissions.
bool copy_file(const std::string& from, const std::string& to) {
    const int in = ::open(from.c_str(), O_RDONLY | O_CLOEXEC);
    if (in < 0) {
        return false;
    }
    struct stat st{};
    if (::fstat(in, &st) != 0) {
        ::close(in);
        return false;
    }
    const int out = ::open(to.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, st.st_mode & 07777);
    if (out < 0) {
        ::close(in);
        return false;
    }
    bool ok = true;
    char buf[65536];
    for (;;) {
        const ssize_t r = ::read(in, buf, sizeof buf);
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            ok = false;
            break;
        }
        if (r == 0) {
            break;
        }
        ssize_t off = 0;
        while (off < r) {
            const ssize_t w = ::write(out, buf + off, static_cast<std::size_t>(r - off));
            if (w < 0) {
                if (errno == EINTR) {
                    continue;
                }
                ok = false;
                break;
            }
            off += w;
        }
        if (!ok) {
            break;
        }
    }
    if (ok) {
        (void)::fchmod(out, st.st_mode & 07777);
    }
    ::close(out);
    ::close(in);
    return ok;
}

// Delete all but the newest keep copies stem.<stamp>.csv in dir.
void prune(const std::string& dir, const std::string& stem, std::size_t keep) {
    DIR* d = ::opendir(dir.c_str());
    if (d == nullptr) {
        return;
    }
    const std::string prefix = stem + ".";
    std::vector<std::string> old;
    while (const dirent* e = ::readdir(d)) {
        const std::string f = e->d_name;
        if (f == "." || f == ".." || !rf::utf8_valid(f)) {
            continue;
        }
        if (f.starts_with(prefix) && f.ends_with(".csv") && f.size() > prefix.size() &&
            f[prefix.size()] >= '0' && f[prefix.size()] <= '9') {
            old.push_back(path_join(dir, f));
        }
    }
    ::closedir(d);
    // Oldest first: by write time, then by the collision number of copies stamped in the same second.
    auto key = [](const std::string& p) {
        const auto t = mtime_of(p);
        std::string f = file_name(p);
        while (f.ends_with(".csv")) {
            f.resize(f.size() - 4);
        }
        std::uint32_t n = 0;
        if (const auto i = f.rfind('-'); i != std::string::npos) {
            const std::string x = f.substr(i + 1);
            std::uint32_t v = 0;
            if (x.size() < 6 && rf::parse_u32(x, v)) {
                n = v;
            }
        }
        return std::pair{t, n};
    };
    std::stable_sort(old.begin(), old.end(), [&](const std::string& a, const std::string& b) { return key(a) < key(b); });
    while (old.size() > std::max<std::size_t>(keep, 1)) {
        ::unlink(old.front().c_str());
        old.erase(old.begin());
    }
}

// Step 2: move a staged name.<stamp>.keeping into keep_dir as name.<stamp>.csv (a rename, or a copy across
// filesystems), re-point link at it and delete all but the newest keep stamped copies there. Returns the kept file.
// On an error the staged file is renamed to name.<stamp>.csv where it is (still linked).
std::optional<std::string> finish(const std::string& staged, const std::string& kdir, const std::string& link,
                                  std::size_t keep) {
    const std::string fname = file_name(staged);
    if (!rf::utf8_valid(fname) || !fname.ends_with(".keeping")) {
        return std::nullopt;
    }
    const std::string base = fname.substr(0, fname.size() - 8);
    const auto dot = base.rfind('.');
    if (dot == std::string::npos) {
        return std::nullopt;
    }
    const std::string stem = base.substr(0, dot);
    auto moved = [&]() -> std::optional<std::string> {
        if (!create_dir_all(kdir)) {
            return std::nullopt;
        }
        std::string dest = path_join(kdir, base + ".csv");
        int n = 1;
        while (path_exists(dest)) {
            dest = path_join(kdir, std::format("{}-{}.csv", base, n));
            n += 1;
        }
        if (::rename(staged.c_str(), dest.c_str()) != 0) {
            // tmpfs -> disk: copy, keep the write time (the copies are sorted by it), then drop the RAM copy
            struct stat st{};
            const bool have_mtime = ::stat(staged.c_str(), &st) == 0;
            if (!copy_file(staged, dest)) {
                ::unlink(dest.c_str());
                return std::nullopt;
            }
            if (have_mtime) {
                const int fd = ::open(dest.c_str(), O_WRONLY | O_CLOEXEC);
                if (fd >= 0) {
                    const struct timespec ts[2] = {{0, UTIME_OMIT}, st.st_mtim};
                    (void)::futimens(fd, ts);
                    ::close(fd);
                }
            }
            ::unlink(staged.c_str());
        }
        return dest;
    }();
    if (!moved) {
        const auto dir = path_parent(staged);
        const std::string here = path_join(dir.value_or(""), base + ".csv");
        if (::rename(staged.c_str(), here.c_str()) == 0) {
            relink(link, here);
        }
        return std::nullopt;
    }
    relink(link, *moved);
    prune(kdir, stem, keep);
    return moved;
}

} // namespace

std::optional<std::string> flag_path() {
    const auto d = env_os("XDG_RUNTIME_DIR");
    if (!d || d->empty()) {
        return std::nullopt;
    }
    return path_join(*d, "omarchyvr-imu-log.on");
}

// The bounded queue between the IMU thread and the writer (std::sync::mpsc::sync_channel(8192)).
struct Channel {
    std::mutex m;
    std::condition_variable cv;
    std::deque<Msg> q;
    bool sender_gone = false;
    bool receiver_gone = false;
    static constexpr std::size_t CAP = 8192;
};

namespace {

void writer(std::shared_ptr<Channel> ch, int fd, std::int64_t span_ns, std::optional<std::string> flag) {
    std::string buf;
    bool ok = true;
    auto flush = [&]() {
        std::size_t off = 0;
        while (ok && off < buf.size()) {
            const ssize_t n = ::write(fd, buf.data() + off, buf.size() - off);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                ok = false;
            } else {
                off += static_cast<std::size_t>(n);
            }
        }
        buf.clear();
    };
    // BufWriter::with_capacity(1 << 16): writes reach the file in 64 KiB pieces.
    auto put = [&](std::string_view s) {
        if (buf.size() + s.size() > (1u << 16)) {
            flush();
        }
        buf.append(s);
        return ok;
    };
    auto finish_thread = [&]() {
        std::lock_guard<std::mutex> g(ch->m);
        ch->receiver_gone = true;
        ch->q.clear();
    };
    if (!put(std::string(HEADER) + "\n")) {
        finish_thread();
        ::close(fd);
        return;
    }
    std::optional<std::int64_t> first;
    bool flag_on = false;
    std::optional<std::int64_t> flag_checked;
    for (;;) {
        Msg m;
        {
            std::unique_lock<std::mutex> l(ch->m);
            ch->cv.wait(l, [&] { return !ch->q.empty() || ch->sender_gone; });
            if (ch->q.empty()) {
                break;
            }
            m = ch->q.front();
            ch->q.pop_front();
        }
        const std::int64_t ts = std::visit([](const auto& x) { return x.t; }, m);
        if (!first) {
            first = ts;
        }
        const std::int64_t t0 = *first;
        if (sat_sub(ts, t0) > span_ns) {
            if (!flag_checked || sat_sub(ts, *flag_checked) > 1'000'000'000) {
                flag_checked = ts;
                const bool on = flag && path_exists(*flag);
                if (flag_on && !on) {
                    flush();
                }
                flag_on = on;
            }
            if (!flag_on) {
                continue;
            }
        }
        std::string line;
        if (const auto* i = std::get_if<ImuMsg>(&m)) {
            line = std::format("I,{},{},{},{},{},{},{}\n", i->t, rf::fixed(i->a[0], 5), rf::fixed(i->a[1], 5),
                               rf::fixed(i->a[2], 5), rf::fixed(i->g[0], 6), rf::fixed(i->g[1], 6),
                               rf::fixed(i->g[2], 6));
        } else {
            const auto& v = std::get<VioMsg>(m);
            line = std::format("V,{},{},{},{},{},{},{},{}\n", v.t, rf::fixed(v.p[0], 5), rf::fixed(v.p[1], 5),
                               rf::fixed(v.p[2], 5), rf::fixed(v.q[0], 6), rf::fixed(v.q[1], 6), rf::fixed(v.q[2], 6),
                               rf::fixed(v.q[3], 6));
        }
        if (!put(line)) {
            finish_thread();
            ::close(fd);
            return;
        }
    }
    flush();
    finish_thread();
    ::close(fd);
}

} // namespace

std::unique_ptr<HeadImuLog> HeadImuLog::from_env() {
    const auto p = env_os("PIMAX_HEAD_IMU_LOG");
    if (!p || p->empty()) {
        return nullptr;
    }
    double minutes = 10.0;
    if (const auto v = env_utf8("PIMAX_HEAD_IMU_LOG_MIN")) {
        double x = 0;
        if (rf::parse_f64(rf::trim(*v), x)) {
            minutes = x;
        }
    }
    return start(*p, minutes, flag_path());
}

std::unique_ptr<HeadImuLog> HeadImuLog::start(const std::string& path, double minutes,
                                              std::optional<std::string> flag) {
    return start_keeping(path, minutes, std::move(flag), keep_dir([](std::string_view k) {
                             return env_utf8(std::string(k).c_str());
                         }));
}

std::unique_ptr<HeadImuLog> HeadImuLog::start_keeping(const std::string& path, double minutes,
                                                      std::optional<std::string> flag,
                                                      std::optional<std::string> keep) {
    if (const auto staged = stage(path)) {
        const std::string k = keep ? *keep : path_parent(path).value_or("");
        const std::string link = prev_link(path);
        try {
            std::thread([s = *staged, k, link] { (void)finish(s, k, link, KEEP); }).detach();
        } catch (...) {
        }
    }
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
    if (fd < 0) {
        return nullptr;
    }
    auto ch = std::make_shared<Channel>();
    const std::int64_t span_ns = f64_to_i64(rf::fmax(minutes, 0.0) * 60e9);
    try {
        std::thread(writer, ch, fd, span_ns, std::move(flag)).detach();
    } catch (...) {
        ::close(fd);
        return nullptr;
    }
    return std::unique_ptr<HeadImuLog>(new HeadImuLog(std::move(ch)));
}

HeadImuLog::~HeadImuLog() {
    {
        std::lock_guard<std::mutex> g(ch_->m);
        ch_->sender_gone = true;
    }
    ch_->cv.notify_all();
}

namespace {
// try_send: a full queue drops the message (counted), a writer that has ended swallows it.
bool try_send(Channel& ch, Msg m) {
    {
        std::lock_guard<std::mutex> g(ch.m);
        if (ch.receiver_gone) {
            return true;
        }
        if (ch.q.size() >= Channel::CAP) {
            return false;
        }
        ch.q.push_back(std::move(m));
    }
    ch.cv.notify_one();
    return true;
}
} // namespace

void HeadImuLog::imu(std::int64_t ts_ns, const std::array<double, 3>& accel, const std::array<double, 3>& gyro) {
    if (!try_send(*ch_, ImuMsg{ts_ns, accel, gyro})) {
        dropped += 1;
    }
}

void HeadImuLog::vio(std::int64_t ts_ns, const std::array<double, 3>& pos, const std::array<double, 4>& q) {
    if (!try_send(*ch_, VioMsg{ts_ns, pos, q})) {
        dropped += 1;
    }
}

std::optional<std::string> keep_dir(const std::function<std::optional<std::string>(std::string_view)>& env) {
    std::optional<std::string> state;
    if (auto x = env("XDG_STATE_HOME"); x && !x->empty()) {
        state = *x;
    } else if (auto h = env("HOME"); h && !h->empty()) {
        state = path_join(*h, ".local/state");
    }
    if (!state) {
        return std::nullopt;
    }
    return path_join(*state, "omarchyvr/head-imu");
}

std::optional<std::string> rotate(const std::string& path, const std::string& kdir, std::size_t keep) {
    const auto staged = stage(path);
    if (!staged) {
        return std::nullopt;
    }
    return finish(*staged, kdir, prev_link(path), keep);
}

HeadImu parse(std::string_view text) {
    HeadImu h;
    std::size_t pos = 0;
    while (pos < text.size()) {
        std::size_t nl = text.find('\n', pos);
        std::string_view l = text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        pos = nl == std::string_view::npos ? text.size() : nl + 1;
        if (!l.empty() && l.back() == '\r') {
            l.remove_suffix(1);
        }
        std::vector<std::string_view> f;
        std::size_t s = 0;
        for (;;) {
            const std::size_t c = l.find(',', s);
            f.push_back(l.substr(s, c == std::string_view::npos ? std::string_view::npos : c - s));
            if (c == std::string_view::npos) {
                break;
            }
            s = c + 1;
        }
        std::optional<std::vector<double>> n;
        if (f.size() >= 2) {
            std::vector<double> v;
            for (std::size_t i = 2; i < f.size(); i++) {
                double x = 0;
                if (rf::parse_f64(rf::trim(f[i]), x)) {
                    v.push_back(x);
                }
            }
            n = std::move(v);
        }
        std::optional<std::int64_t> ts;
        if (f.size() >= 2) {
            std::int64_t t = 0;
            if (parse_i64(rf::trim(f[1]), t)) {
                ts = t;
            }
        }
        if (f[0] == "I" && ts && n && n->size() == 6 && f.size() == 8) {
            const auto& v = *n;
            h.imu.push_back({*ts, {v[0], v[1], v[2]}, {v[3], v[4], v[5]}});
        } else if (f[0] == "V" && ts && n && n->size() == 7 && f.size() == 9) {
            h.vio_ts.push_back(*ts);
        }
    }
    return h;
}

} // namespace pibridge::cam::imulog
