// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-components/src/components/proc.rs (MIT OR Apache-2.0).
#include "pibridge/record/proc.h"

#include "pibridge_env.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>

#include <fcntl.h>
#include <sys/statvfs.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;

fs::path procRoot() {
    if (const char* p = pibridge::env("PROC_ROOT"); p != nullptr && *p != '\0') {
        return p;
    }
    return "/proc";
}

namespace {
std::optional<std::string> readFile(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        return std::nullopt;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
std::string trimmed(std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ' || s.back() == '\r' || s.back() == '\t')) {
        s.pop_back();
    }
    std::size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
        i++;
    }
    return s.substr(i);
}
} // namespace

bool isAlive(const fs::path& proc, std::uint32_t pid) {
    if (pid == 0) {
        return false;
    }
    const auto stat = readFile(proc / std::to_string(pid) / "stat");
    if (!stat) {
        return false;
    }
    // "pid (comm) S ...": the state follows the last ')'
    const std::size_t r = stat->rfind(')');
    if (r == std::string::npos || r + 2 >= stat->size()) {
        return true; // a fake tree may only have comm-less stat lines
    }
    const char st = (*stat)[r + 2];
    return st != 'Z' && st != 'X';
}

std::optional<std::string> comm(const fs::path& proc, std::uint32_t pid) {
    auto c = readFile(proc / std::to_string(pid) / "comm");
    if (!c) {
        return std::nullopt;
    }
    return trimmed(*c);
}

bool anyWithComm(const fs::path& proc, std::string_view name) {
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(proc, ec)) {
        const std::string n = e.path().filename().string();
        if (n.empty() || n.find_first_not_of("0123456789") != std::string::npos) {
            continue;
        }
        const auto pid = static_cast<std::uint32_t>(std::stoul(n));
        if (comm(proc, pid) == std::optional<std::string>(std::string(name)) && isAlive(proc, pid)) {
            return true;
        }
    }
    return false;
}

std::optional<std::uint32_t> readPid(const fs::path& pidFile) {
    const auto t = readFile(pidFile);
    if (!t) {
        return std::nullopt;
    }
    const std::string s = trimmed(*t);
    if (s.empty() || s.find_first_not_of("0123456789") != std::string::npos || s.size() > 10) {
        return std::nullopt;
    }
    const unsigned long v = std::stoul(s);
    if (v > UINT32_MAX) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(v);
}

std::string commOf(const fs::path& program) { return program.filename().string().substr(0, 15); }

std::optional<std::uint32_t> spawnLogged(const fs::path& program, const std::vector<std::string>& args,
                                         const fs::path& log, std::string* error) {
    std::error_code ec;
    if (fs::exists(log, ec)) {
        fs::rename(log, fs::path(log.string() + ".prev"), ec);
    }
    const int out = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (out < 0) {
        if (error) {
            *error = std::strerror(errno);
        }
        return std::nullopt;
    }
    std::vector<std::string> store;
    store.push_back(program.string());
    store.insert(store.end(), args.begin(), args.end());
    std::vector<char*> argv;
    for (auto& s : store) {
        argv.push_back(s.data());
    }
    argv.push_back(nullptr);
    // a pipe that the exec closes (CLOEXEC): an exec failure writes errno into it
    int pfd[2];
    if (pipe2(pfd, O_CLOEXEC) != 0) {
        ::close(out);
        if (error) {
            *error = std::strerror(errno);
        }
        return std::nullopt;
    }
    const pid_t pid = fork();
    if (pid < 0) {
        const int e = errno;
        ::close(out);
        ::close(pfd[0]);
        ::close(pfd[1]);
        if (error) {
            *error = std::strerror(e);
        }
        return std::nullopt;
    }
    if (pid == 0) {
        setsid();
        const int nul = ::open("/dev/null", O_RDONLY);
        dup2(nul, 0);
        dup2(out, 1);
        dup2(out, 2);
        const int keep = pfd[1];
        // close everything but 0-2 and the error pipe (CLOEXEC closes that one at exec)
        for (int fd = 3; fd < 1024; fd++) {
            if (fd != keep) {
                ::close(fd);
            }
        }
        signal(SIGPIPE, SIG_DFL);
        sigset_t none;
        sigemptyset(&none);
        sigprocmask(SIG_SETMASK, &none, nullptr);
        execvp(argv[0], argv.data());
        const int e = errno;
        (void)!write(keep, &e, sizeof e);
        _exit(127);
    }
    ::close(out);
    ::close(pfd[1]);
    int childErr = 0;
    ssize_t n;
    do {
        n = read(pfd[0], &childErr, sizeof childErr);
    } while (n < 0 && errno == EINTR);
    ::close(pfd[0]);
    if (n == sizeof childErr) {
        waitpid(pid, nullptr, 0);
        if (error) {
            *error = std::strerror(childErr);
        }
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(pid);
}

void spawnQuiet(const std::vector<std::string>& args) {
    std::vector<std::string> store = args;
    std::vector<char*> argv;
    for (auto& s : store) {
        argv.push_back(s.data());
    }
    argv.push_back(nullptr);
    const pid_t pid = fork();
    if (pid < 0) {
        return;
    }
    if (pid == 0) {
        if (fork() != 0) {
            _exit(0);
        }
        setsid();
        const int nul = ::open("/dev/null", O_RDWR);
        dup2(nul, 0);
        dup2(nul, 1);
        dup2(nul, 2);
        for (int fd = 3; fd < 1024; fd++) {
            ::close(fd);
        }
        signal(SIGPIPE, SIG_DFL);
        execvp(argv[0], argv.data());
        _exit(127);
    }
    waitpid(pid, nullptr, 0);
}

std::string exitStatusText(int st) {
    if (WIFEXITED(st)) {
        return std::format("exit status: {}", WEXITSTATUS(st));
    }
    if (WIFSIGNALED(st)) {
        const int sig = WTERMSIG(st);
        const char* abbrev = sigabbrev_np(sig);
        return std::format("signal: {} (SIG{}){}", sig, abbrev ? abbrev : "?", WCOREDUMP(st) ? " (core dumped)" : "");
    }
    return std::format("unknown wait status {:#x}", st);
}

std::optional<std::uint64_t> freeBytes(const fs::path& dir) {
    struct statvfs st{};
    if (statvfs(dir.c_str(), &st) != 0) {
        return std::nullopt;
    }
    return std::uint64_t(st.f_bavail) * st.f_frsize;
}

} // namespace pibridge::record
