// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/cli.rs (MIT OR Apache-2.0).
// Not ported: `tracking` (PiBridge records tracking on its Development page) and the GPU gate.
#include "pibridge/record/cli.h"

#include "pibridge/record/proc.h"
#include "pibridge/record/status.h"
#include "pibridge_env.h"

#include <cerrno>
#include <cstring>
#include <csignal>
#include <cstdio>
#include <format>
#include <fstream>
#include <thread>

#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

const char* const kUsage =
    "pibridge-record - record what the headset shows to ~/Videos/pimax/pimax-YYYYmmdd-HHMMSS.mp4\n"
    "\n"
    "usage: pibridge-record start [options] [--save]   start recording (\"Monado directly\" or SteamVR must run)\n"
    "       pibridge-record stop                       stop and finish the file (always playable)\n"
    "       pibridge-record status                     what is being recorded\n"
    "options (defaults from ~/.config/pibridge/record.toml; --save writes the given ones there):\n"
    "  --eyes left|both          one eye, lens-undistorted (default) or both side by side\n"
    "  --fps 30|60|72            frame rate (default 30)\n"
    "  --quality low|medium|high (default high)\n"
    "  --scale F                 0 = fit --max-height (default); 0.1-1.0 = fraction of the eye's resolution\n"
    "  --max-height N            per-eye height cap (default 1080)\n"
    "  --codec h264|hevc         (default h264)\n"
    "  --encoder auto|nvenc|cpu  auto = NVENC, else x264 on the CPU (default auto)\n"
    "  --dir DIR                 output directory (default ~/Videos/pimax)\n"
    "  --audio on|off            record what you hear into the video (default on)\n"
    "  --mic on|off              also mix in your microphone (default off)\n"
    "  --source auto|tap|mirror  auto (default): SteamVR's mirror texture while SteamVR runs, else Monado's tap\n"
    "pibridge-record start --brief / stop --brief: the saved settings, and one short line (\"Recording\", \"Saved: name.mp4\")\n"
    "Design and reliability notes: docs/record.md";

std::expected<Cmd, std::string> parse(const std::vector<std::string>& args) {
    if (args.empty()) {
        return Cmd{};
    }
    const std::string& first = args[0];
    if (first == "tracking") {
        return std::unexpected(std::string(
            "tracking recordings are not part of pibridge-record: use Record tracking on the app's Development page"));
    }
    if (args.size() == 2 && args[1] == "--brief" && (first == "start" || first == "stop")) {
        Cmd c;
        c.kind = Cmd::Kind::Brief;
        c.stop = first == "stop";
        return c;
    }
    Overrides overrides;
    bool save = false;
    std::string source = "auto";
    for (std::size_t i = 1; i < args.size(); i++) {
        const std::string& a = args[i];
        if ((a == "--source" || a.starts_with("--source=")) && (first == "start" || first == "run")) {
            std::string v;
            if (a == "--source") {
                if (++i >= args.size()) {
                    return std::unexpected(a + " needs a value");
                }
                v = args[i];
            } else {
                v = a.substr(9);
            }
            if (v != "auto" && !parseSource(v)) {
                return std::unexpected("source = " + v + ": expected auto, tap or mirror");
            }
            source = v;
        } else if (a == "--save") {
            save = true;
        } else if (a == "-h" || a == "--help") {
            return Cmd{};
        } else if (a.starts_with("--")) {
            std::string k, v;
            if (const auto eq = a.find('=', 2); eq != std::string::npos) {
                k = a.substr(2, eq - 2);
                v = a.substr(eq + 1);
            } else {
                if (++i >= args.size()) {
                    return std::unexpected(a + " needs a value");
                }
                k = a.substr(2);
                v = args[i];
            }
            Settings probe;
            if (auto r = probe.set(k, v); !r) {
                return std::unexpected(r.error());
            }
            overrides.emplace_back(k, v);
        } else {
            return std::unexpected("unexpected argument " + a);
        }
    }
    Cmd c;
    c.overrides = overrides;
    c.source = source;
    if (first == "start") {
        c.kind = Cmd::Kind::Start;
        c.save = save;
    } else if (first == "stop" && overrides.empty()) {
        c.kind = Cmd::Kind::Stop;
        c.overrides.clear();
    } else if (first == "status" && overrides.empty()) {
        c.kind = Cmd::Kind::Status;
    } else if (first == "run") {
        c.kind = Cmd::Kind::Run;
    } else if (first == "help" || first == "-h" || first == "--help") {
        return Cmd{};
    } else if (first == "stop" || first == "status") {
        return std::unexpected(first + " takes no options");
    } else {
        return std::unexpected("unknown record command " + first);
    }
    return c;
}

std::expected<Settings, std::string> effective(const Roots& r, const Overrides& overrides) {
    auto s = Settings::load(settingsReadPath(r.config, r.legacyConfig));
    if (!s) {
        return s;
    }
    for (const auto& [k, v] : overrides) {
        if (auto e = s->set(k, v); !e) {
            return std::unexpected(e.error());
        }
    }
    return s;
}

fs::path recorderBin() {
    if (const char* p = pibridge::env("RECORD_BIN"); p != nullptr && *p != '\0') {
        return p;
    }
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        const fs::path sib = exe.parent_path() / "pibridge-record";
        if (fs::exists(sib, ec)) {
            return sib;
        }
    }
    return "pibridge-record";
}

fs::path pidFile(const fs::path& sessionDir) { return sessionDir / (std::string(kPidName) + ".pid"); }

std::optional<std::uint32_t> runningPid(const Roots& r) {
    const auto pid = readPid(pidFile(r.session));
    if (!pid || !isAlive(r.proc, *pid) || comm(r.proc, *pid) != std::optional<std::string>(kComm)) {
        return std::nullopt;
    }
    return pid;
}

std::vector<std::string> runArgs(const Settings& s, Source source) {
    std::vector<std::string> a{"run"};
    const auto more = s.toArgs();
    a.insert(a.end(), more.begin(), more.end());
    a.insert(a.end(), {"--source", std::string(name(source))});
    return a;
}

std::expected<Source, std::string> pickSource(const Roots& r, const std::string& wanted) {
    const bool steamvr = steamvrRunning(r);
    const bool monado = monadoRunning(r);
    const auto want = parseSource(wanted);
    if (want == Source::Mirror || (!want && steamvr)) {
        if (!steamvr) {
            return std::unexpected(std::string("SteamVR is not running: its mirror cannot be recorded"));
        }
        return Source::Mirror;
    }
    if (monado) {
        return Source::Tap;
    }
    if (steamvr) {
        // --source tap under SteamVR + driver_monado: SteamVR's compositor draws, Monado's tap sees nothing
        return std::unexpected(std::string(
            "SteamVR runs: its compositor draws the headset image, which Monado's recorder tap cannot see. Record "
            "SteamVR's mirror instead (--source mirror, the default)."));
    }
    return std::unexpected(
        std::string("no VR session: start one first (\"Monado directly\" on the Development page, or SteamVR)"));
}

namespace {
Status loadStatus(const Roots& r) { return Status::load(statusPath(r.session)).value_or(Status{}); }

std::vector<std::string> logTail(const fs::path& p, std::size_t n) {
    std::ifstream f(p);
    std::vector<std::string> lines;
    std::string l;
    while (std::getline(f, l)) {
        lines.push_back(l);
    }
    if (lines.size() > n) {
        lines.erase(lines.begin(), lines.end() - static_cast<std::ptrdiff_t>(n));
    }
    return lines;
}

// Wait until pid has stayed alive for `settle` (polled every 250 ms, as omarchyvr's stays_alive).
bool staysAlive(const Roots& r, std::uint32_t pid, std::chrono::milliseconds settle) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        if (!isAlive(r.proc, pid)) {
            return false;
        }
        if (std::chrono::steady_clock::now() - t0 >= settle) {
            return true;
        }
        std::this_thread::sleep_for(250ms);
    }
}
} // namespace

std::expected<std::string, std::string> start(const Roots& r, const Overrides& overrides, bool save,
                                              const std::string& source) {
    auto s = effective(r, overrides);
    if (!s) {
        return std::unexpected(s.error());
    }
    if (save) {
        if (const std::string e = s->save(settingsPath(r.config)); !e.empty()) {
            return std::unexpected("cannot save the settings: " + e);
        }
    }
    if (const auto pid = runningPid(r)) {
        return std::unexpected(std::format("already recording (pid {}): {}", *pid, loadStatus(r).summary()));
    }
    const auto src = pickSource(r, source);
    if (!src) {
        return std::unexpected(src.error());
    }
    std::error_code ec;
    fs::create_directories(r.session, ec);
    const fs::path bin = recorderBin();
    const fs::path log = r.session / "record.log";
    const auto t0 = std::chrono::steady_clock::now();
    std::string err;
    const auto pid = spawnLogged(bin, runArgs(*s, *src), log, &err);
    if (!pid) {
        return std::unexpected(std::format("cannot start {}: {}", bin.string(), err));
    }
    const fs::path pf = pidFile(r.session);
    {
        std::ofstream f(pf, std::ios::trunc);
        f << *pid << '\n';
    }
    if (!staysAlive(r, *pid, 300ms)) {
        fs::remove(pf, ec);
        const Status st = loadStatus(r);
        if (st.state == "error") {
            return std::unexpected(st.summary());
        }
        std::string tail;
        for (const auto& l : logTail(log, 12)) {
            tail += (tail.empty() ? "" : " | ") + l;
        }
        return std::unexpected("the recorder exited at once: " + tail);
    }
    // wait until it records (the tap attaches within about 1 s) or gives up
    while (std::chrono::steady_clock::now() - t0 < 10s) {
        const Status st = loadStatus(r);
        if (st.pid == *pid && st.state == "recording") {
            return st.summary();
        }
        if (!isAlive(r.proc, *pid)) {
            fs::remove(pf, ec);
            return std::unexpected(st.pid == *pid ? st.summary() : std::string("the recorder exited"));
        }
        std::this_thread::sleep_for(100ms);
    }
    return std::format("started (pid {}), still waiting for the first frame: pibridge-record status", *pid);
}

std::expected<std::string, std::string> stop(const Roots& r) {
    const fs::path pf = pidFile(r.session);
    std::error_code ec;
    const auto notRunning = [&] {
        fs::remove(pf, ec);
        const auto st = Status::load(statusPath(r.session));
        return std::format("not recording ({})", st ? st->summary() : std::string());
    };
    const auto pid = readPid(pf);
    if (!pid || !isAlive(r.proc, *pid) || comm(r.proc, *pid) != std::optional<std::string>(kComm)) {
        return notRunning();
    }
    const auto exited = [&] {
        fs::remove(pf, ec);
        const auto st = Status::load(statusPath(r.session));
        return st ? st->summary() : std::string("stopped");
    };
    if (kill(static_cast<pid_t>(*pid), SIGINT) != 0) {
        if (errno == ESRCH) {
            return exited();
        }
        return std::unexpected(std::format("SIGINT to record pid {}: {}", *pid, std::strerror(errno)));
    }
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < kStopGrace) {
        if (!isAlive(r.proc, *pid)) {
            return exited();
        }
        std::this_thread::sleep_for(250ms);
    }
    if (!isAlive(r.proc, *pid)) {
        return exited();
    }
    return std::unexpected(std::format(
        "the recorder is still finishing after {} s (it never blocks the session; pibridge-record status)", kStopGrace.count()));
}

std::expected<std::string, std::string> stopBrief(const Roots& r) {
    if (!runningPid(r)) {
        return std::string("Not recording");
    }
    if (auto s = stop(r); !s) {
        return s;
    }
    const Status st = loadStatus(r);
    if (!st.file.empty()) {
        const auto slash = st.file.rfind('/');
        return "Saved: " + (slash == std::string::npos ? st.file : st.file.substr(slash + 1));
    }
    if (st.state == "error") {
        return "Not saved: " + st.note;
    }
    return std::string("Nothing was recorded");
}

std::string statusLine(const Roots& r) {
    const Status st = loadStatus(r);
    if (runningPid(r)) {
        return st.summary();
    }
    if (st.state == "recording" || st.state == "starting" || st.state == "finalizing") {
        return std::format("not recording (the recorder exited without finishing its status; last file: {})",
                           st.file.empty() ? std::string("none") : st.file);
    }
    return st.summary();
}

int main(const std::vector<std::string>& args) {
    const Roots r = Roots::fromEnv();
    const auto cmd = parse(args);
    if (!cmd) {
        std::fprintf(stderr, "pibridge-record: %s\n\n%s\n", cmd.error().c_str(), kUsage);
        return 2;
    }
    std::expected<std::string, std::string> res;
    switch (cmd->kind) {
    case Cmd::Kind::Help: std::printf("%s\n", kUsage); return 0;
    case Cmd::Kind::Run: {
        const auto s = effective(r, cmd->overrides);
        if (!s) {
            std::fprintf(stderr, "pibridge-record: %s\n", s.error().c_str());
            return 2;
        }
        return run(*s, r, parseSource(cmd->source).value_or(Source::Tap));
    }
    case Cmd::Kind::Start: res = start(r, cmd->overrides, cmd->save, cmd->source); break;
    case Cmd::Kind::Stop: res = stop(r); break;
    case Cmd::Kind::Brief:
        res = cmd->stop ? stopBrief(r) : start(r, {}, false).transform([](const std::string&) { return std::string("Recording"); });
        break;
    case Cmd::Kind::Status: res = statusLine(r); break;
    }
    if (res) {
        std::printf("%s\n", res->c_str());
        return 0;
    }
    std::fprintf(stderr, "pibridge-record: %s\n", res.error().c_str());
    return 1;
}

} // namespace pibridge::record
