// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/audio.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/audio.h"

#include <cerrno>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace pibridge::hmd::audio {

namespace {

std::string trim(std::string s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string join(const std::vector<std::string>& v, std::string_view sep) {
    std::string o;
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i) {
            o += sep;
        }
        o += v[i];
    }
    return o;
}

} // namespace

std::optional<std::string> run_pactl(const std::vector<std::string>& args) {
    int pipefd[2];
    if (::pipe2(pipefd, O_CLOEXEC) != 0) {
        return std::nullopt;
    }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, pipefd[1], STDOUT_FILENO);
    posix_spawn_file_actions_addopen(&fa, STDERR_FILENO, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    std::vector<std::string> store{"pactl"};
    store.insert(store.end(), args.begin(), args.end());
    std::vector<char*> argv;
    for (auto& s : store) {
        argv.push_back(s.data());
    }
    argv.push_back(nullptr);
    pid_t pid = 0;
    const int rc = posix_spawnp(&pid, "pactl", &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    ::close(pipefd[1]);
    if (rc != 0) {
        ::close(pipefd[0]);
        return std::nullopt;
    }
    std::string out;
    char buf[4096];
    while (true) {
        const auto n = ::read(pipefd[0], buf, sizeof buf);
        if (n > 0) {
            out.append(buf, static_cast<std::size_t>(n));
        } else if (n < 0 && errno == EINTR) {
            continue;
        } else {
            break;
        }
    }
    ::close(pipefd[0]);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        return std::nullopt;
    }
    return trim(out);
}

std::optional<std::string> pick(std::string_view list, std::string_view prefix) {
    while (!list.empty()) {
        const auto nl = list.find('\n');
        const std::string_view line = list.substr(0, nl);
        list = nl == std::string_view::npos ? std::string_view{} : list.substr(nl + 1);
        const auto t1 = line.find('\t');
        if (t1 == std::string_view::npos) {
            continue;
        }
        const auto rest = line.substr(t1 + 1);
        std::string_view name = rest.substr(0, rest.find('\t'));
        if (name.ends_with('\r')) {
            name.remove_suffix(1);
        }
        if (name.starts_with(prefix) && !name.ends_with(".monitor")) {
            return std::string(name);
        }
    }
    return std::nullopt;
}

std::pair<std::string, Before> to_headset(const Runner& pactl) {
    Before b;
    b.sink = pactl({"get-default-sink"});
    b.source = pactl({"get-default-source"});
    std::optional<std::string> sink;
    if (const auto l = pactl({"list", "short", "sinks"})) {
        sink = pick(*l, SINK_PREFIX);
    }
    std::optional<std::string> source;
    if (const auto l = pactl({"list", "short", "sources"})) {
        source = pick(*l, SOURCE_PREFIX);
    }
    std::vector<std::string> done;
    if (sink && pactl({"set-default-sink", *sink})) {
        done.emplace_back("output");
        b.our_sink = sink;
    }
    if (source && pactl({"set-default-source", *source})) {
        done.emplace_back("input");
        b.our_source = source;
    }
    std::string msg = done.empty() ? "audio: no headset output/input found (pactl list short sinks)"
                                   : "audio: headset is the default " + join(done, " and ");
    return {msg, b};
}

std::string restore(const Before& b, const Runner& pactl) {
    std::vector<std::string> done;
    const auto cur_sink = pactl({"get-default-sink"});
    if (b.sink && b.our_sink && cur_sink == b.our_sink && *b.sink != *b.our_sink &&
        pactl({"set-default-sink", *b.sink})) {
        done.push_back("output " + *b.sink);
    }
    const auto cur_source = pactl({"get-default-source"});
    if (b.source && b.our_source && cur_source == b.our_source && *b.source != *b.our_source &&
        pactl({"set-default-source", *b.source})) {
        done.push_back("input " + *b.source);
    }
    return done.empty() ? "audio: nothing to give back" : "audio: back to " + join(done, ", ");
}

} // namespace pibridge::hmd::audio
