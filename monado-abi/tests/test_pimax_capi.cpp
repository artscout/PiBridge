// SPDX-License-Identifier: MIT
// Twins of omarchyvr pimax-capi tests/abi.rs, src/eye.rs, tests/eye_export.rs and tests/hands_export.rs (MIT OR
// Apache-2.0), through the archive (libpibridge_pimax.a) the way the Monado driver links it, with a fake headset over
// SOCK_SEQPACKET socketpairs (no hardware). Plus the allowlist gate (PiBridge only): every command the C++ libpimax writes
// passes the connected model's description, and with a description that lists nothing, nothing at all is written.
#include "pimax.h"
#include "pimax_eye.h"
#include "pimax_hands.h"

#include "pibridge_test.h"

#include "pibridge/hmd/descriptor.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

extern "C" {
pimax_t* pimax_open_with_fds(const pimax_callbacks* cb, const int read_fds[4], int ctl_write_fd);
pimax_t* pimax_open_with_fds_pid(const pimax_callbacks* cb, const int read_fds[4], int ctl_write_fd, uint16_t pid);
}

namespace fs = std::filesystem;
using namespace std::chrono;
using Report = std::vector<std::uint8_t>;

namespace {

void msleep(int ms) { std::this_thread::sleep_for(milliseconds(ms)); }

std::string read_text(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// Function names declared at column 0 of a header (`type name(...)`), as the Rust export tests read them.
std::vector<std::string> header_functions(const std::string& file, const std::string& prefix) {
    std::vector<std::string> names;
    std::istringstream in(read_text(fs::path(PIBRIDGE_ABI_INCLUDE_DIR) / file));
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || std::isspace(static_cast<unsigned char>(line[0])) || line[0] == '#' || line[0] == '/' ||
            line[0] == '*' || line[0] == '}' || line.starts_with("typedef") || line.find('(') == std::string::npos) {
            continue;
        }
        std::string head = line.substr(0, line.find('('));
        std::size_t e = head.size();
        std::size_t b = e;
        while (b > 0 && (std::isalnum(static_cast<unsigned char>(head[b - 1])) || head[b - 1] == '_')) {
            b--;
        }
        const std::string name = head.substr(b, e - b);
        if (name.starts_with(prefix)) {
            names.push_back(name);
        }
    }
    return names;
}

std::string defined_globals() {
    const std::string cmd = std::string("nm -g --defined-only ") + PIBRIDGE_ABI_ARCHIVE;
    FILE* p = ::popen(cmd.c_str(), "r");
    std::string out;
    if (p == nullptr) {
        return out;
    }
    char b[4096];
    std::size_t n;
    while ((n = std::fread(b, 1, sizeof b, p)) > 0) {
        out.append(b, n);
    }
    ::pclose(p);
    return out;
}

bool exported(const std::string& syms, const std::string& name) {
    return syms.find(" T " + name + "\n") != std::string::npos;
}

// ---------------------------------------------------------------------------------------------- the fake headset

struct Fake {
    int dev_if[4]{}, host_if[4]{}, ctl_dev = -1, ctl_host = -1;
    std::mutex mu;
    std::vector<Report> cmds; // everything the session wrote (report id stripped), in order
    std::vector<std::size_t> bad_lengths;
    std::atomic<bool> stop{false};
    std::thread reader;

    Fake() {
        for (int i = 0; i < 4; i++) {
            int sv[2];
            CHECK(::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) == 0);
            host_if[i] = sv[0];
            dev_if[i] = sv[1];
        }
        int cv[2];
        CHECK(::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, cv) == 0);
        ctl_host = cv[0];
        ctl_dev = cv[1];
        reader = std::thread([this] {
            while (!stop.load()) {
                pollfd p{ctl_dev, POLLIN, 0};
                if (::poll(&p, 1, 10) <= 0) {
                    continue;
                }
                std::uint8_t b[80];
                const ssize_t n = ::recv(ctl_dev, b, sizeof b, MSG_DONTWAIT);
                if (n <= 0) {
                    continue;
                }
                std::lock_guard g(mu);
                if (n != 65 || b[0] != 0) {
                    bad_lengths.push_back(static_cast<std::size_t>(n));
                    continue;
                }
                cmds.emplace_back(b + 1, b + 65);
            }
        });
    }
    ~Fake() {
        stop = true;
        reader.join();
        for (int i = 0; i < 4; i++) {
            ::close(dev_if[i]);
        }
        ::close(ctl_dev);
    }
    std::vector<Report> take() {
        std::lock_guard g(mu);
        return std::exchange(cmds, {});
    }
    void send(int iface, std::initializer_list<std::uint8_t> bytes) {
        std::uint8_t r[64] = {};
        std::copy(bytes.begin(), bytes.end(), r);
        CHECK(::send(dev_if[iface], r, 64, 0) == 64);
    }
};

bool has(const std::vector<Report>& v, std::initializer_list<std::uint8_t> prefix) {
    for (const auto& r : v) {
        if (std::equal(prefix.begin(), prefix.end(), r.begin())) {
            return true;
        }
    }
    return false;
}

const pibridge::hmd::Descriptor& desc(const char* id) {
    static std::map<std::string, pibridge::hmd::Descriptor> cache;
    auto it = cache.find(id);
    if (it == cache.end()) {
        auto d = pibridge::hmd::Descriptor::load(fs::path(PIBRIDGE_SOURCE_DEVICES_DIR) / (std::string(id) + ".toml"));
        if (!d) {
            std::fprintf(stderr, "%s: %s\n", id, d.error().msg.c_str());
            std::abort();
        }
        it = cache.emplace(id, std::move(*d)).first;
    }
    return it->second;
}

// ---------------------------------------------------------------------------------------------- session callbacks

std::atomic<int> g_state{-1}, g_imu{0}, g_ctrl{0}, g_status{0}, g_log{0};
pimax_imu_sample g_imu0{};
pimax_controller_report g_ctrl0{};
std::string g_logline;
int g_tag;

void on_state(void* u, pimax_state s) {
    CHECK(u == &g_tag);
    g_state = s;
}
void on_imu(void* u, const pimax_imu_sample* s) {
    CHECK(u == &g_tag);
    if (g_imu.load() == 0) {
        g_imu0 = *s;
    }
    g_imu++;
}
void on_ctrl(void* u, const pimax_controller_report* r) {
    CHECK(u == &g_tag);
    g_ctrl0 = *r;
    g_ctrl++;
}
void on_status(void* u, const pimax_controller_status*) {
    CHECK(u == &g_tag);
    g_status++;
}
void on_log(void* u, const char* l) {
    CHECK(u == &g_tag);
    g_logline = l;
    g_log++;
}

const pimax_callbacks k_cb = {&g_tag, on_state, on_imu, on_ctrl, on_status, on_log, nullptr};

template <class T>
bool wait_until(T pred, int ms = 2000) {
    for (int i = 0; i < ms; i++) {
        if (pred()) {
            return true;
        }
        msleep(1);
    }
    return pred();
}

} // namespace

// ---------------------------------------------------------------------------------------------- abi.rs, exports

TEST(every_header_function_is_exported) {
    const auto syms = defined_globals();
    const auto lib = header_functions("pimax.h", "pimax_");
    CHECK(lib.size() == 17); // 14 of the C library + 3 haptics
    for (const auto& n : lib) {
        if (!exported(syms, n)) {
            std::fprintf(stderr, "missing export %s\n", n.c_str());
        }
        CHECK(exported(syms, n));
    }
    CHECK(exported(syms, "pimax_open_with_fds") && exported(syms, "pimax_open_with_fds_pid"));
}

TEST(every_eye_header_function_is_exported) {
    const auto syms = defined_globals();
    const auto names = header_functions("pimax_eye.h", "pimax_eye_");
    CHECK(names.size() == 7);
    for (const auto& n : names) {
        CHECK(exported(syms, n));
    }
}

TEST(every_hands_header_function_is_exported) {
    const auto syms = defined_globals();
    const auto names = header_functions("pimax_hands.h", "pimax_hands_");
    CHECK(names.size() == 13);
    for (const auto& n : names) {
        CHECK(exported(syms, n));
    }
    // the same constant on both sides: a stale archive would make the Monado driver switch hand tracking off
    CHECK(pimax_hands_abi_version() == PIMAX_HANDS_ABI_VERSION);
}

TEST(the_eye_abi_version_matches_the_header) {
    CHECK(read_text(fs::path(PIBRIDGE_ABI_INCLUDE_DIR) / "pimax_eye.h").find("#define PIMAX_EYE_ABI_VERSION 1u") !=
          std::string::npos);
    CHECK(pimax_eye_abi_version() == PIMAX_EYE_ABI_VERSION);
}

// eye.rs
TEST(eye_struct_layout_matches_the_header) {
    CHECK(sizeof(pimax_eye_sample) == 8 + 8 + 8 + 4 + 4 + 12 + 12 + 24 + 24 + 8);
    CHECK(alignof(pimax_eye_sample) == 8);
}

TEST(eye_null_arguments_are_safe) {
    CHECK(pimax_eye_latest(nullptr, nullptr) == PIMAX_EYE_BAD_ARG);
    CHECK(pimax_eye_state(nullptr) == PIMAX_EYE_STATE_STOPPED);
    CHECK(pimax_eye_observe_target(nullptr, 0, nullptr) == PIMAX_EYE_BAD_ARG);
    pimax_eye_stop(nullptr);
    CHECK(pimax_eye_abi_version() == PIMAX_EYE_ABI_VERSION);
}

TEST(struct_layout_matches_the_c_header) {
    CHECK(sizeof(pimax_imu_sample) == 48 && offsetof(pimax_imu_sample, sample_in_report) == 44);
    CHECK(offsetof(pimax_controller_report, raw) + PIMAX_REPORT_SIZE <= sizeof(pimax_controller_report));
    CHECK(offsetof(pimax_callbacks, on_reply) == 6 * sizeof(void*));
    CHECK(PIMAX_DISCONNECTED == 0 && PIMAX_CONNECTED == 1 && PIMAX_NUM_IF == 4);
}

// tests/c/session.c as a C++ twin; the two commands that are not in the Dream Air description are refused (-EPERM)
// where the Rust library writes them (`58 3f 16 00` with 60 data bytes, `58 03 2c 00`).
TEST(a_full_session_against_a_fake_headset) {
    g_state = -1;
    g_imu = g_ctrl = g_status = g_log = 0;
    Fake f;
    pimax_t* h = pimax_open_with_fds(&k_cb, f.host_if, f.ctl_host);
    CHECK(h != nullptr);
    CHECK(pimax_wait_connected(h, 3000) == 0);
    CHECK(pimax_is_connected(h) == 1);
    CHECK(g_state.load() == PIMAX_CONNECTED);
    CHECK(wait_until([&] { std::lock_guard g(f.mu); return !f.cmds.empty(); }));
    {
        const auto c = f.take();
        const std::uint8_t wake[] = {0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00};
        CHECK(!c.empty() && std::equal(std::begin(wake), std::end(wake), c[0].begin()));
    }
    pimax_set_video_rate(h, 72);
    CHECK(wait_until([&] { std::lock_guard g(f.mu); return has(f.cmds, {0x58, 0x04, 0x39, 0x00, 72}); }, 3000));

    f.send(2, {0x53, 0x31, 0x08, 0x00, 0x40, 0x4b, 0x4c, 0x00, 0, 0, 0, 0, 0, 0, 0x08, 0x00, 0xfc, 0x00, 0, 0, 0x00, 0xa4,
               0, 0, 0xff, 0x5c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6590 & 255, 6590 >> 8});
    CHECK(wait_until([] { return g_imu.load() >= 2; }));
    CHECK(g_imu0.ts_us == 5000000 && g_imu0.sample_in_report == 0);
    CHECK(g_imu0.raw_accel[0] == 2048 && g_imu0.accel_g[0] == 1.0f && g_imu0.accel_g[1] == -0.5f);
    CHECK(g_imu0.raw_gyro[0] == 164 && g_imu0.gyro_dps[0] == 10.0f && g_imu0.raw_gyro[2] == -164);
    CHECK(g_imu0.ipd_mm > 65.89f && g_imu0.ipd_mm < 65.91f);

    std::uint8_t cr[64] = {0x53, 0x32, 0x08, 0x03, 0x43, 255, 1, 200};
    cr[44] = 99;
    cr[46] = 0xda;
    cr[47] = 0x0a;
    cr[48] = 1;
    cr[49] = 0x08;
    CHECK(::send(f.dev_if[1], cr, 64, 0) == 64);
    CHECK(wait_until([] { return g_ctrl.load() >= 1; }));
    CHECK(g_ctrl0.slot == PIMAX_SLOT_RIGHT && g_ctrl0.stick_x == 127 && g_ctrl0.stick_y == -127 &&
          g_ctrl0.trigger == 200 && g_ctrl0.grip == 99);
    CHECK(g_ctrl0.battery == 90 && g_ctrl0.raw[46] == 0xda);
    CHECK(g_ctrl0.buttons == (PIMAX_BTN_A | PIMAX_BTN_STICK | PIMAX_BTN_MENU) && g_ctrl0.touch == PIMAX_TOUCH_STICK);

    f.send(0, {0x4c, 0x0f, 0x37, 0x02, 0x48, 1, 2, 3, 4, 0x49, 0, 2, 1, 0, 0x08, 0x2d});
    CHECK(wait_until([] { return g_status.load() >= 1; }));
    f.send(3, {'h', 'e', 'l', 'l', 'o', ' ', 'f', 'w', '\r', '\n'});
    CHECK(wait_until([] { return g_log.load() >= 1; }));
    CHECK(g_logline == "hello fw");

    // the gate: what the description does not list is refused before anything is written
    (void)f.take();
    std::uint8_t big[61] = {};
    CHECK(pimax_send(h, 0x16, 0, big, sizeof big) == -EMSGSIZE);
    CHECK(pimax_send(h, 0x16, 0, big, 60) == -EPERM);
    std::uint8_t rep[64];
    CHECK(pimax_request(h, 0x2c, 0, nullptr, 0, rep, sizeof rep, 120) == -EPERM);
    CHECK(pimax_request(h, 0x50, 0, nullptr, 0, rep, sizeof rep, 120) == -ETIMEDOUT); // listed, never answered
    msleep(30);
    for (const auto& c : f.take()) {
        CHECK(c[2] == 0x39 || c[2] == 0x1a || c[2] == 0x50); // keepalives, the wake re-send, the listed read
    }

    CHECK(pimax_controller_attach(h, PIMAX_SLOT_LEFT) == 0);
    CHECK(pimax_video_prepare(h, 72) == 0);
    msleep(50);
    const auto c = f.take();
    for (const auto& want : {std::vector<std::uint8_t>{0x58, 0x04, 0x16, 2, 0x1e}, {0x58, 0x03, 0x50, 2, 0},
                             {0x58, 0x07, 0x0a, 2, 0}, {0x58, 0x03, 0x03, 2, 0}, {0x58, 0x05, 0x5b, 0x05, 2, 2},
                             {0x58, 0x04, 0x29, 0x00, 1, 0}, {0x58, 0x0d, 0x1a, 0x05, 2, 72},
                             {0x58, 0x0d, 0x1a, 0x05, 5, 0}}) {
        bool found = false;
        for (const auto& r : c) {
            found = found || std::equal(want.begin(), want.end(), r.begin());
        }
        CHECK(found);
    }

    for (int i = 0; i < 4; i++) {
        ::shutdown(f.dev_if[i], SHUT_RDWR);
    }
    ::shutdown(f.ctl_dev, SHUT_RDWR);
    CHECK(wait_until([] { return g_state.load() == PIMAX_DISCONNECTED; }));
    CHECK(pimax_is_connected(h) == 0);
    CHECK(pimax_send(h, 3, 0, nullptr, 0) == -ENOTCONN);
    pimax_close(h);
}

// abi.rs close_returns_promptly_while_caller_holds_callback_lock
namespace {
std::mutex g_lock;
void on_state_locking(void*, pimax_state s) {
    if (s == PIMAX_DISCONNECTED) {
        std::lock_guard g(g_lock); // disconnect during close
    }
}
} // namespace

TEST(close_returns_promptly_while_caller_holds_callback_lock) {
    int sv[4][2];
    for (auto& p : sv) {
        CHECK(::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, p) == 0);
    }
    const int reads[4] = {sv[0][0], sv[1][0], sv[2][0], sv[3][0]};
    pimax_callbacks cb{};
    cb.on_state = on_state_locking;
    pimax_t* h = pimax_open_with_fds(&cb, reads, sv[0][1]);
    CHECK(h != nullptr);
    CHECK(pimax_wait_connected(h, 2000) == 0);
    std::unique_lock g(g_lock);
    const auto t = steady_clock::now();
    pimax_close(h);
    g.unlock();
    CHECK(steady_clock::now() - t < milliseconds(500));
}

TEST(null_handles_are_refused_not_dereferenced) {
    CHECK(pimax_is_connected(nullptr) == 0);
    CHECK(pimax_wait_connected(nullptr, 0) == -EINVAL);
    CHECK(pimax_send(nullptr, 3, 0, nullptr, 0) == -EINVAL);
    CHECK(pimax_haptic(nullptr, 2, 0.5f, 0) == -EINVAL);
    CHECK(pimax_haptic_poll_request(nullptr) == -EINVAL);
    std::uint64_t s = 7;
    pimax_haptic_counts(nullptr, &s, nullptr);
    CHECK(s == 7);
    pimax_set_video_rate(nullptr, 72);
    pimax_close(nullptr);
    pimax_imu_sample out[2];
    CHECK(pimax_decode_imu(nullptr, out) == 0);
    CHECK(pimax_open_with_fds_pid(nullptr, nullptr, -1, 0x1234) == nullptr && errno == ENODEV);
}

// ---------------------------------------------------------------------------------------------- the allowlist gate

namespace {

// Everything the Monado driver (and pimax-cli) asks of libpimax, and what the session does by itself.
void drive_everything(pimax_t* h, const fs::path& runtime) {
    for (const std::uint8_t r : std::initializer_list<std::uint8_t>{0, 72, 90, 60}) {
        pimax_set_video_rate(h, r);
        msleep(240);
    }
    for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{2, 3}) {
        (void)pimax_controller_attach(h, slot);
    }
    for (const std::uint8_t fps : std::initializer_list<std::uint8_t>{72, 90, 60}) {
        (void)pimax_video_prepare(h, fps);
        msleep(240);
    }
    for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{2, 3, 1, 4}) {
        for (const auto& [a, ns] : std::vector<std::pair<float, std::int64_t>>{
                 {0.25f, 300'000'000}, {1.0f, -1}, {0.001f, 1}, {2.0f, 1'000'000'000'000LL}, {0.5f, INT64_MAX}}) {
            (void)pimax_haptic(h, slot, a, ns);
            msleep(12);
        }
    }
    std::ofstream(runtime / "pimax-haptic-request") << "3 25 300\n";
    (void)pimax_haptic_poll_request(h);
    msleep(12);
    std::uint8_t rep[64];
    char ser[32];
    for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{0, 2, 3}) {
        (void)pimax_read_serial(h, slot, ser, sizeof ser, 30);
        (void)pimax_read_version(h, slot, rep, sizeof rep, 30);
        (void)pimax_send(h, 0x50, slot, nullptr, 0);
    }
    // commands nobody listed: the IPD motor, pairing, a calibration opcode, a raw camera push with zero exposure
    const std::uint8_t ipd[] = {0x01, 0xaf, 0x18};
    (void)pimax_send(h, 0x5c, 0x05, ipd, sizeof ipd);
    const std::uint8_t pair[] = {2};
    (void)pimax_send(h, 0x04, 0x00, pair, 1);
    (void)pimax_send(h, 0x3f, 0x02, nullptr, 0);
    const std::uint8_t ae[22] = {0x01, 0x32};
    (void)pimax_send(h, 0x51, 0x00, ae, sizeof ae);
    msleep(250);
}

} // namespace

TEST(the_gate_sees_every_command) {
    const fs::path runtime = std::getenv("XDG_RUNTIME_DIR") ? std::getenv("XDG_RUNTIME_DIR") : "/tmp";
    for (const auto& [pid, id] : std::vector<std::pair<std::uint16_t, const char*>>{{0x0044, "dream-air"},
                                                                                 {0x0040, "crystal-super"}}) {
        Fake f;
        pimax_t* h = pimax_open_with_fds_pid(nullptr, f.host_if, f.ctl_host, pid);
        CHECK(h != nullptr);
        CHECK(pimax_wait_connected(h, 2000) == 0);
        drive_everything(h, runtime);
        // refused at the ABI: what the description does not list
        const std::uint8_t ipd[] = {0x01, 0xaf, 0x18};
        // the Crystal Super does not list the IPD motor; the Dream Air does, at most once per second (TooSoon here)
        CHECK(pimax_send(h, 0x5c, 0x05, ipd, sizeof ipd) == -EPERM);
        CHECK(pimax_send(h, 0x3f, 0x02, nullptr, 0) == -EPERM);
        CHECK(pimax_video_prepare(h, 60) == -EPERM); // fps 60 was never captured
        msleep(50);
        pimax_close(h);
        const auto cmds = f.take();
        CHECK(f.bad_lengths.empty());
        std::set<std::string> ids;
        const auto& al = desc(id).allowlist;
        for (const auto& c : cmds) {
            const auto r = al.check(0, c);
            if (!r) {
                std::fprintf(stderr, "%s: a command outside the allowlist reached the device: %02x %02x %02x %02x\n",
                             id, c[0], c[1], c[2], c[3]);
            }
            CHECK(r.has_value());
            if (r) {
                ids.insert((*r)->id);
            }
        }
        std::string list;
        for (const auto& s : ids) {
            list += " " + s;
        }
        std::printf("%s: %zu commands, all listed:%s\n", id, cmds.size(), list.c_str());
        for (const char* want : {"wake", "keepalive", "read_serial", "read_info", "read_doc_controller", "read_version",
                                 "app_unlock", "panel_on", "announce_refresh", "announce_gpu", "haptic"}) {
            CHECK(ids.contains(want));
        }
        // never: a keepalive or an announce with 60, a haptic for slot 1/4, AMP 0
        for (const auto& c : cmds) {
            CHECK(!(c[2] == 0x39 && c[4] == 60));
            CHECK(!(c[2] == 0x1a && c[3] == 0x05 && c[4] == 0x02 && c[5] == 60));
            CHECK(!(c[2] == 0x07 && (c[3] < 2 || c[3] > 3 || c[4] == 0)));
        }
    }
}

TEST(a_description_that_lists_nothing_sends_nothing) {
    const fs::path dir = fs::path(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") /
                         ("pibridge-pimax-empty-" + std::to_string(::getpid()));
    fs::create_directories(dir);
    std::ofstream(dir / "dream-air.toml")
        << "[device]\nid = \"dream-air\"\nname = \"X\"\nvendor = \"v\"\nstatus = \"mock\"\nusb_vid = 1\n"
           "[display]\n[tracking]\n[controllers]\n[eye_tracking]\n[ipd]\n[cameras]\n[audio]\n[calibration]\n"
           "[onboarding]\nsteps = [\"detect\"]\n";
    const char* old = std::getenv("PIBRIDGE_DEVICES_DIR");
    const std::string keep = old ? old : "";
    ::setenv("PIBRIDGE_DEVICES_DIR", dir.c_str(), 1);
    {
        Fake f;
        pimax_t* h = pimax_open_with_fds(nullptr, f.host_if, f.ctl_host);
        CHECK(h != nullptr);
        CHECK(pimax_wait_connected(h, 2000) == 0);
        drive_everything(h, dir);
        CHECK(pimax_send(h, 0x03, 0x00, nullptr, 0) == -EPERM);
        CHECK(pimax_controller_attach(h, 2) == -EPERM);
        CHECK(pimax_video_prepare(h, 90) == -EPERM);
        std::uint8_t rep[8];
        CHECK(pimax_request(h, 0x03, 0, nullptr, 0, rep, sizeof rep, 50) == -EPERM);
        msleep(600); // past the wake re-send
        pimax_close(h);
        CHECK(f.take().empty());
        CHECK(f.bad_lengths.empty());
    }
    if (old) {
        ::setenv("PIBRIDGE_DEVICES_DIR", keep.c_str(), 1);
    } else {
        ::unsetenv("PIBRIDGE_DEVICES_DIR");
    }
    fs::remove_all(dir);
}

// The descriptions are found without a PiBridge devices folder too (the copies compiled into the archive, or an
// installed copy): a Monado build that links the archive still talks to the headset.
TEST(the_built_in_descriptions_are_the_fallback) {
    const fs::path dir = fs::path(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") /
                         ("pibridge-pimax-nodesc-" + std::to_string(::getpid()));
    fs::create_directories(dir);
    const char* old = std::getenv("PIBRIDGE_DEVICES_DIR");
    const std::string keep = old ? old : "";
    ::setenv("PIBRIDGE_DEVICES_DIR", dir.c_str(), 1);
    for (const std::uint16_t pid : {std::uint16_t{0x0044}, std::uint16_t{0x0040}}) {
        Fake f;
        pimax_t* h = pimax_open_with_fds_pid(nullptr, f.host_if, f.ctl_host, pid);
        CHECK(pimax_wait_connected(h, 2000) == 0);
        CHECK(pimax_haptic(h, 3, 0.25f, 300'000'000) == 0);
        msleep(60);
        pimax_close(h);
        const auto c = f.take();
        CHECK(has(c, {0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00}));
        CHECK(has(c, {0x58, 0x09, 0x07, 0x03, 0x19, 0x2c, 0x01}));
    }
    if (old) {
        ::setenv("PIBRIDGE_DEVICES_DIR", keep.c_str(), 1);
    } else {
        ::unsetenv("PIBRIDGE_DEVICES_DIR");
    }
    fs::remove_all(dir);
}

// The haptic request file (pimax_haptic_poll_request): taken, sent on the session, removed.
TEST(the_test_pulse_request_goes_out_on_the_session) {
    const fs::path runtime = std::getenv("XDG_RUNTIME_DIR") ? std::getenv("XDG_RUNTIME_DIR") : "/tmp";
    Fake f;
    pimax_t* h = pimax_open_with_fds_pid(nullptr, f.host_if, f.ctl_host, 0x0040);
    CHECK(pimax_wait_connected(h, 2000) == 0);
    CHECK(pimax_haptic_poll_request(h) == 0);
    std::ofstream(runtime / "pimax-haptic-request") << "2 40 200\n";
    CHECK(pimax_haptic_poll_request(h) == 2);
    CHECK(!fs::exists(runtime / "pimax-haptic-request"));
    std::ofstream(runtime / "pimax-haptic-request") << "junk";
    CHECK(pimax_haptic_poll_request(h) == -EINVAL);
    CHECK(!fs::exists(runtime / "pimax-haptic-request"));
    msleep(40);
    std::uint64_t sent = 0, replaced = 9;
    pimax_haptic_counts(h, &sent, &replaced);
    CHECK(sent == 1 && replaced == 0);
    pimax_close(h);
    CHECK(has(f.take(), {0x58, 0x09, 0x07, 0x02, 40, 200, 0}));
}

PIBRIDGE_TEST_MAIN()
