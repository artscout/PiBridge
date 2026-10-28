// SPDX-License-Identifier: MIT
#include "dash_sys.h"
#include "pibridge/record/cli.h"
#include "pibridge/record/run.h"
#include <iterator>

#include "backgrounds.h"

#include "pibridge/hmd/audio.h"
#include "pibridge_status.h"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <poll.h>
#include <spawn.h>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace pibridge::home::dash {

namespace fs = std::filesystem;

namespace {

bool executable(const fs::path& p) {
    std::error_code ec;
    return fs::is_regular_file(p, ec) && ::access(p.c_str(), X_OK) == 0;
}

std::vector<char*> cstrings(std::vector<std::string>& v) {
    std::vector<char*> out;
    out.reserve(v.size() + 1);
    for (auto& s : v) {
        out.push_back(s.data());
    }
    out.push_back(nullptr);
    return out;
}

std::vector<std::string> environment(const std::vector<std::string>& extra) {
    std::map<std::string, std::string> kv;
    std::vector<std::string> order;
    const auto add = [&](const std::string& e) {
        const std::size_t eq = e.find('=');
        const std::string k = e.substr(0, eq);
        if (!kv.contains(k)) {
            order.push_back(k);
        }
        kv[k] = e;
    };
    for (char** e = environ; e != nullptr && *e != nullptr; ++e) {
        add(*e);
    }
    for (const auto& e : extra) {
        add(e);
    }
    std::vector<std::string> out;
    out.reserve(order.size());
    for (const auto& k : order) {
        out.push_back(kv[k]);
    }
    return out;
}

std::optional<int> percent(const std::optional<std::string>& v) {
    if (!v || v->empty()) {
        return std::nullopt;
    }
    return std::atoi(v->c_str()); // "42%"
}

} // namespace

Paths defaultPaths() {
    Paths p;
    p.conf = bg::confPath();
    p.recordConf = p.conf.parent_path() / "record.toml";
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    p.runtimeDir = rt != nullptr && *rt != '\0' ? fs::path(rt) : fs::path("/tmp");
    p.statusDir = status::runtime_dir();
    std::error_code ec;
    const fs::path self = fs::read_symlink("/proc/self/exe", ec);
    // a PiBridge tool: next to this binary (install) or in its build-tree folder, else PATH
    const auto find = [&](const char* name, const char* buildDir) -> std::string {
        if (!ec) {
            for (const fs::path& c : {self.parent_path() / name, self.parent_path() / ".." / buildDir / name}) {
                if (executable(c)) {
                    std::error_code e2;
                    return fs::weakly_canonical(c, e2).string();
                }
            }
        }
        if (const char* path = std::getenv("PATH"); path != nullptr) {
            std::stringstream ss(path);
            std::string dir;
            while (std::getline(ss, dir, ':')) {
                if (!dir.empty() && executable(fs::path(dir) / name)) {
                    return (fs::path(dir) / name).string();
                }
            }
        }
        return {};
    };
    p.roomsetup = find("pibridge-roomsetup", "roomsetup");
    p.record = find("pibridge-record", "record");
    return p;
}

std::optional<std::string> readFile(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool writeFileAtomic(const fs::path& p, const std::string& text) {
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    const fs::path tmp = p.string() + ".tmp-" + std::to_string(::getpid());
    {
        std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
        if (!o || !(o << text) || !o.flush()) {
            fs::remove(tmp, ec);
            return false;
        }
    }
    fs::rename(tmp, p, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

std::optional<long> spawnDetached(const std::vector<std::string>& argv, const std::vector<std::string>& env) {
    if (argv.empty()) {
        return std::nullopt;
    }
    std::vector<std::string> a = argv;
    std::vector<std::string> e = environment(env);
    auto av = cstrings(a);
    auto ev = cstrings(e);
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
    pid_t pid = 0;
    const int r = posix_spawnp(&pid, av[0], nullptr, &attr, av.data(), ev.data());
    posix_spawnattr_destroy(&attr);
    if (r != 0) {
        return std::nullopt;
    }
    return static_cast<long>(pid);
}

std::optional<int> reap(long pid) {
    int st = 0;
    const pid_t r = ::waitpid(static_cast<pid_t>(pid), &st, WNOHANG);
    if (r == 0) {
        return std::nullopt;
    }
    if (r < 0) {
        return -1; // not ours any more
    }
    return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

std::optional<std::string> usageOf(const std::string& program) {
    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) != 0) {
        return std::nullopt;
    }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, fds[1], 1);
    posix_spawn_file_actions_adddup2(&fa, fds[1], 2);
    std::vector<std::string> a{program, "--help"};
    auto av = cstrings(a);
    pid_t pid = 0;
    const int r = posix_spawnp(&pid, av[0], &fa, nullptr, av.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    ::close(fds[1]);
    if (r != 0) {
        ::close(fds[0]);
        return std::nullopt;
    }
    std::string out;
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    char buf[4096];
    while (std::chrono::steady_clock::now() < until) {
        pollfd pf{fds[0], POLLIN, 0};
        if (::poll(&pf, 1, 100) <= 0) {
            continue;
        }
        const ssize_t n = ::read(fds[0], buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        out.append(buf, static_cast<std::size_t>(n));
    }
    ::close(fds[0]);
    int st = 0;
    if (::waitpid(pid, &st, WNOHANG) == 0) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &st, 0);
    }
    return out;
}

VolumeWorker::VolumeWorker() : th_([this] { run(); }) {}

VolumeWorker::~VolumeWorker() {
    {
        std::lock_guard g(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    th_.join();
}

std::optional<int> VolumeWorker::current() const {
    std::lock_guard g(mu_);
    return current_;
}

bool VolumeWorker::set(int p) {
    {
        std::lock_guard g(mu_);
        if (!current_) {
            return false;
        }
        target_ = p;
        current_ = p; // shown at once; the next read says what it really is
    }
    cv_.notify_all();
    return true;
}

void VolumeWorker::run() {
    std::unique_lock lk(mu_);
    while (!stop_) {
        const std::optional<int> target = std::exchange(target_, std::nullopt);
        lk.unlock();
        const auto sink = hmd::audio::headset_sink();
        std::optional<int> now;
        if (sink) {
            if (target) {
                (void)hmd::audio::set_sink_volume(*sink, std::to_string(*target) + "%");
            }
            now = percent(hmd::audio::sink_volume(*sink));
        }
        lk.lock();
        if (!target_) {
            current_ = now;
        }
        cv_.wait_for(lk, std::chrono::seconds(2), [this] { return stop_ || target_.has_value(); });
    }
}

Sinks realSinks(VolumeWorker& volume) {
    Sinks s;
    s.read = readFile;
    s.write = writeFileAtomic;
    s.volume = [&volume] { return volume.current(); };
    s.setVolume = [&volume](int p) { return volume.set(p); };
    s.spawn = spawnDetached;
    s.usage = usageOf;
    s.recording = [] {
        // the recorder's own sign, as the app's Record card reads it: the pid in its status file runs
        return pibridge::record::runningPid(pibridge::record::Roots::fromEnv()).has_value();
    };
    return s;
}

} // namespace pibridge::home::dash
