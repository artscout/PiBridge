// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/audio.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/audio.h"

#include <cerrno>
#include <cstdlib>
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
    // the C locale: `pactl list cards` is parsed by its English keys
    std::vector<std::string> env_store;
    for (char** e = environ; *e != nullptr; e++) {
        const std::string_view v(*e);
        if (!v.starts_with("LC_ALL=") && !v.starts_with("LANGUAGE=")) {
            env_store.emplace_back(v);
        }
    }
    env_store.emplace_back("LC_ALL=C");
    std::vector<char*> envp;
    for (auto& e : env_store) {
        envp.push_back(e.data());
    }
    envp.push_back(nullptr);
    pid_t pid = 0;
    const int rc = posix_spawnp(&pid, "pactl", &fa, nullptr, argv.data(), envp.data());
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

std::string DpPort::sink() const {
    constexpr std::string_view card_prefix = "alsa_card.";
    constexpr std::string_view profile_prefix = "output:";
    const std::string_view c = std::string_view(card).starts_with(card_prefix)
                                   ? std::string_view(card).substr(card_prefix.size()) : std::string_view(card);
    const std::string_view p = std::string_view(profile).starts_with(profile_prefix)
                                   ? std::string_view(profile).substr(profile_prefix.size()) : std::string_view(profile);
    return "alsa_output." + std::string(c) + "." + std::string(p);
}

std::optional<DpPort> parse_dp_port(std::string_view list) {
    // Card blocks: "\tName: <card>", "\tActive Profile: <p>", then "\tPorts:" with a port per "\t\t<port>: ...", its
    // "\t\t\t\tdevice.product.name = \"<monitor>\"" and "\t\t\tPart of profile(s): <p>, <p>".
    std::string card, active;
    bool in_ports = false, pimax = false;
    const std::string want = "device.product.name = \"" + std::string(DP_MONITOR_NAME) + "\"";
    while (!list.empty()) {
        const auto nl = list.find('\n');
        std::string_view line = list.substr(0, nl);
        list = nl == std::string_view::npos ? std::string_view{} : list.substr(nl + 1);
        std::size_t depth = 0;
        while (depth < line.size() && line[depth] == '\t') {
            depth++;
        }
        const std::string_view t = line.substr(depth);
        if (depth == 0) {
            card.clear();
            active.clear();
            in_ports = pimax = false;
        } else if (depth == 1) {
            in_ports = t == "Ports:";
            if (t.starts_with("Name: ")) {
                card = trim(std::string(t.substr(6)));
            } else if (t.starts_with("Active Profile: ")) {
                active = trim(std::string(t.substr(16)));
            }
        } else if (in_ports && depth == 2) {
            pimax = false;
        } else if (in_ports && depth >= 3 && trim(std::string(t)) == want) {
            pimax = true;
        } else if (in_ports && pimax && t.starts_with("Part of profile(s): ")) {
            std::string first = trim(std::string(t.substr(20)));
            first = first.substr(0, first.find(','));
            if (!card.empty() && !first.empty()) {
                // the card's Active Profile line comes before its ports
                return DpPort{card, trim(first), active};
            }
        }
    }
    return std::nullopt;
}

namespace {

std::optional<DpPort> dp_port(const Runner& pactl) {
    if (const auto l = pactl({"list", "cards"})) {
        return parse_dp_port(*l);
    }
    return std::nullopt;
}

bool has_node(const std::optional<std::string>& list, const std::string& name) {
    return list && pick(*list, name) == name;
}

} // namespace

std::pair<std::string, Before> to_headset(const Runner& pactl, bool output, bool input) {
    Before b;
    b.sink = pactl({"get-default-sink"});
    b.source = pactl({"get-default-source"});
    std::optional<std::string> sink;
    if (!output) {
        // input only
    } else if (const auto l = pactl({"list", "short", "sinks"})) {
        sink = pick(*l, SINK_PREFIX);
    }
    if (!sink && output) {
        if (const auto dp = dp_port(pactl)) {
            if (dp->active_profile == dp->profile || pactl({"set-card-profile", dp->card, dp->profile})) {
                b.card = dp->card;
                b.card_profile = dp->active_profile;
                b.our_card_profile = dp->profile;
                sink = dp->sink();
            }
        }
    }
    std::optional<std::string> source;
    if (!input) {
        // output only
    } else if (const auto l = pactl({"list", "short", "sources"})) {
        source = pick(*l, SOURCE_PREFIX);
        if (!source) {
            source = pick(*l, CRYSTAL_SOURCE_PREFIX);
        }
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
    if (b.card && b.card_profile && b.our_card_profile && *b.card_profile != *b.our_card_profile) {
        if (const auto dp = dp_port(pactl); dp && dp->card == *b.card && dp->active_profile == *b.our_card_profile &&
                                            pactl({"set-card-profile", *b.card, *b.card_profile})) {
            done.push_back("card " + *b.card_profile);
        }
    }
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

std::optional<std::string> headset_sink(const Runner& pactl) {
    const auto l = pactl({"list", "short", "sinks"});
    if (l) {
        if (auto s = pick(*l, SINK_PREFIX)) {
            return s;
        }
    }
    if (const auto dp = dp_port(pactl); dp && has_node(l, dp->sink())) {
        return dp->sink();
    }
    return std::nullopt;
}

std::optional<std::string> parse_volume(std::string_view text) {
    const auto pct = text.find('%');
    if (pct == std::string_view::npos) {
        return std::nullopt;
    }
    std::size_t b = pct;
    while (b > 0 && text[b - 1] >= '0' && text[b - 1] <= '9') {
        b--;
    }
    if (b == pct) {
        return std::nullopt;
    }
    return std::string(text.substr(b, pct - b + 1));
}

std::optional<std::string> sink_volume(const std::string& sink, const Runner& pactl) {
    if (const auto out = pactl({"get-sink-volume", sink})) {
        return parse_volume(*out);
    }
    return std::nullopt;
}

bool set_sink_volume(const std::string& sink, const std::string& volume, const Runner& pactl) {
    return pactl({"set-sink-volume", sink, volume}).has_value();
}

VolumeButton volume_button(std::string_view log_line) {
    for (const auto& [text, b] : {std::pair{"app: vol up 1", VolumeButton{1, true}},
                                  std::pair{"app: vol up 0", VolumeButton{1, false}},
                                  std::pair{"app: vol down 1", VolumeButton{-1, true}},
                                  std::pair{"app: vol down 0", VolumeButton{-1, false}}}) {
        if (log_line.find(text) != std::string_view::npos) {
            return b;
        }
    }
    return {};
}

std::optional<std::string> step_sink_volume(const std::string& sink, int steps, const Runner& pactl) {
    const auto cur = sink_volume(sink, pactl);
    if (!cur) {
        return std::nullopt;
    }
    int pct = std::atoi(cur->c_str()) + steps * VOLUME_STEP;
    pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
    const std::string v = std::to_string(pct) + "%";
    if (!set_sink_volume(sink, v, pactl)) {
        return std::nullopt;
    }
    return v;
}

} // namespace pibridge::hmd::audio
