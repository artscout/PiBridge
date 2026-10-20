// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/run.rs (MIT OR Apache-2.0).
// Not ported: omarchyvrd's GPU gate (PiBridge has none): NVENC is always allowed and the CPU encoder is the fallback.
#include "pibridge/record/run.h"

#include "pibridge/record/encode.h"
#include "pibridge/record/proc.h"
#include "pibridge/record/shm.h"
#include "pibridge/record/status.h"
#include "pibridge_env.h"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <memory>
#include <fstream>
#include <thread>

#include <sys/wait.h>

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {
std::atomic<bool> g_stop{false};
extern "C" void onSignal(int) { g_stop.store(true); }

fs::path envPath(const char* var) {
    const char* v = std::getenv(var);
    return v != nullptr && *v != '\0' ? fs::path(v) : fs::path();
}
} // namespace

Roots Roots::fromEnv() {
    Roots r;
    r.home = envPath("HOME");
    if (r.home.empty()) {
        r.home = "/";
    }
    r.runtime = envPath("XDG_RUNTIME_DIR");
    if (r.runtime.empty()) {
        r.runtime = "/tmp";
    }
    if (const char* s = pibridge::env("VR_SESSION_DIR"); s != nullptr && *s != '\0') {
        r.session = s;
    } else {
        r.session = r.runtime / "pibridge";
    }
    fs::path cfg = envPath("XDG_CONFIG_HOME");
    if (cfg.empty()) {
        cfg = r.home / ".config";
    }
    r.config = cfg / "pibridge";
    r.legacyConfig = cfg / "omarchyvr";
    r.proc = procRoot();
    return r;
}

std::uint64_t Pacer::copies(std::uint64_t ts) {
    if (!haveT0_) {
        haveT0_ = true;
        t0_ = ts;
    }
    if (ts < t0_) {
        early++;
        return 0;
    }
    const auto n = static_cast<std::uint64_t>(std::round(double(ts - t0_) * double(fps_) / 1e9));
    if (n < next_) {
        early++;
        return 0;
    }
    const std::uint64_t gap = n - next_ + 1;
    const std::uint64_t c = std::min(gap, kMaxCopies);
    if (gap > c) {
        lost_s += double(gap - c) / double(fps_);
    }
    duplicated += c - 1;
    next_ = n + 1;
    return c;
}

std::string_view name(Source s) { return s == Source::Mirror ? "mirror" : "tap"; }

std::optional<Source> parseSource(std::string_view s) {
    if (s == "tap" || s == "monado") {
        return Source::Tap;
    }
    if (s == "mirror" || s == "steamvr") {
        return Source::Mirror;
    }
    return std::nullopt;
}

fs::path mirrorBin() {
    if (const char* p = pibridge::env("RECORD_MIRROR_BIN"); p != nullptr && *p != '\0') {
        return p;
    }
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        const fs::path sib = exe.parent_path() / "pibridge-record-mirror";
        if (fs::exists(sib, ec)) {
            return sib;
        }
    }
    return "pibridge-record-mirror";
}

bool monadoRunning(const Roots& r) { return anyWithComm(r.proc, "monado-service"); }
bool steamvrRunning(const Roots& r) { return anyWithComm(r.proc, "vrserver"); }

fs::path uniquePath(const fs::path& dir, const std::string& name) {
    std::error_code ec;
    fs::path p = dir / name;
    if (!fs::exists(p, ec)) {
        return p;
    }
    std::string stem = name;
    while (stem.ends_with(".mp4")) {
        stem.resize(stem.size() - 4);
    }
    for (int i = 2;; i++) {
        p = dir / std::format("{}-{}.mp4", stem, i);
        if (!fs::exists(p, ec)) {
            return p;
        }
    }
}

namespace {

// Lower our own CPU and I/O priority (the encoder child sets its own).
void beNice() {
    setpriority(PRIO_PROCESS, 0, 10);
    syscall(SYS_ioprio_set, 1, 0, (2 << 13) | 7);
}

struct Ctx {
    fs::path sessionDir;
    fs::path statusPath;
    Status st;
    void save() {
        st.updated_unix = unixNow();
        st.save(statusPath);
    }
    // A clean refusal or stop: status error, an event, a notification. Exit code 0.
    int fail(const std::string& text) {
        st.state = "error";
        st.note = text;
        save();
        event(sessionDir, "critical", text, true);
        return 0;
    }
};

} // namespace

namespace {
// The last non-empty line of a log ("" when none).
std::string lastLine(const fs::path& p) {
    std::ifstream f(p);
    std::string l, last;
    while (std::getline(f, l)) {
        if (!l.empty()) {
            last = l;
        }
    }
    const std::string prefix = "pibridge-record-mirror: ";
    return last.starts_with(prefix) ? last.substr(prefix.size()) : last;
}

// The mirror child: reaped without waiting; stopped (it notices the ring's stop first) when the recorder ends.
struct MirrorChild {
    pid_t pid = -1;
    std::optional<int> status;
    MirrorChild() = default;
    MirrorChild(const MirrorChild&) = delete;
    MirrorChild& operator=(const MirrorChild&) = delete;
    ~MirrorChild() { end(); }
    bool exited() {
        if (pid > 0 && !status) {
            int st = 0;
            if (waitpid(pid, &st, WNOHANG) == pid) {
                status = st;
            }
        }
        return status.has_value();
    }
    void end() {
        const auto t0 = std::chrono::steady_clock::now();
        while (pid > 0 && !exited() && std::chrono::steady_clock::now() - t0 < 3s) {
            std::this_thread::sleep_for(20ms);
        }
        if (pid > 0 && !exited()) {
            kill(pid, SIGTERM);
            while (!exited() && std::chrono::steady_clock::now() - t0 < 6s) {
                std::this_thread::sleep_for(20ms);
            }
            if (!exited()) {
                kill(pid, SIGKILL);
                int st = 0;
                waitpid(pid, &st, 0);
                status = st;
            }
        }
    }
};
} // namespace

int run(const Settings& s, const Roots& r, Source source) {
    beNice();
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sa.sa_flags = SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN); // a dead encoder is an EPIPE, not our death

    std::error_code ec;
    fs::create_directories(r.session, ec);
    Ctx cx{r.session, statusPath(r.session), {}};
    cx.st.state = "starting";
    cx.st.pid = static_cast<std::uint32_t>(getpid());
    cx.st.started_unix = unixNow();
    cx.st.eyes = name(s.eyes);
    cx.st.fps = s.fps;
    cx.st.quality = name(s.quality);
    cx.st.source = name(source);
    cx.save();
    const bool mirror = source == Source::Mirror;

    // Preconditions.
    if (pibridge::env("RECORD_NO_MONADO_CHECK") == nullptr) {
        if (mirror && !steamvrRunning(r)) {
            return cx.fail("no VR session (SteamVR is not running)");
        }
        if (!mirror && !monadoRunning(r)) {
            return cx.fail("no VR session (monado-service is not running)");
        }
    }
    const fs::path ffmpeg = ffmpegPath();
    const auto avail = Available::probe(ffmpeg);
    if (!avail) {
        return cx.fail(std::format("cannot run {} (install ffmpeg)", ffmpeg.string()));
    }
    auto planned = plan(s.encoder, s.codec, *avail, true);
    if (!planned) {
        return cx.fail(planned.error());
    }
    std::vector<Encoder> encs = *planned;
    const fs::path dir = s.outDir(r.home);
    fs::create_directories(dir, ec);
    if (ec) {
        return cx.fail(std::format("cannot create {}: {}", dir.string(), ec.message()));
    }
    if (const auto f = freeBytes(dir); f && *f < kStartFree) {
        return cx.fail(std::format("less than 2 GiB free in {}", dir.string()));
    }

    // The ring, and a heartbeat that keeps going while the encoder write blocks.
    const auto [maxH, scale] = s.request();
    const Request req = Request::make(eyeCount(s.eyes), s.fps, maxH, scale);
    const fs::path ringPath = defaultShmPath(r.runtime);
    auto ringE = Ring::create(ringPath, req);
    if (!ringE) {
        return cx.fail(std::format("cannot create {}: {}", ringPath.string(), ringE.error()));
    }
    const auto ring = std::make_shared<Ring>(std::move(*ringE));
    std::atomic<bool> hbQuit{false};
    std::atomic<std::uint64_t> progress{monotonicNs()};
    std::atomic<int> encoderPid{0};
    std::jthread hb([&] {
        while (!hbQuit.load()) {
            ring->heartbeat();
            // encoder watchdog: a write blocked for 20 s (or 5 s after a stop request) means ffmpeg hung
            const double stalled = double(monotonicNs() - std::min(progress.load(), monotonicNs())) / 1e9;
            const int pid = encoderPid.load();
            if (pid > 0 && (stalled > 20.0 || (g_stop.load() && stalled > 5.0))) {
                kill(pid, SIGKILL); // our own ffmpeg child; the blocked write then fails with EPIPE
                encoderPid.store(0);
            }
            std::this_thread::sleep_for(200ms);
        }
    });
    const auto finishRing = [&] {
        ring->requestStop();
        // give Monado's worker a moment to detach before the file goes away
        const auto t = std::chrono::steady_clock::now();
        while (ring->producerPid() != 0 && std::chrono::steady_clock::now() - t < 1s) {
            std::this_thread::sleep_for(20ms);
        }
        hbQuit.store(true);
    };

    // SteamVR: our own producer, pibridge-record-mirror, on the same ring.
    MirrorChild mchild;
    const fs::path mirrorLog = r.session / "record-mirror.log";
    if (mirror) {
        std::string err;
        const auto pid = spawnLogged(mirrorBin(), {ringPath.string()}, mirrorLog, &err);
        if (!pid) {
            finishRing();
            hb.join();
            return cx.fail(std::format("cannot start {}: {}", mirrorBin().string(), err));
        }
        mchild.pid = static_cast<pid_t>(*pid);
    }

    // Wait for the producer (Monado's tap, or the mirror) to attach.
    const auto t0 = std::chrono::steady_clock::now();
    while (ring->producerPid() == 0) {
        if (mirror && mchild.exited()) {
            finishRing();
            hb.join();
            const std::string why = lastLine(mirrorLog);
            return cx.fail("SteamVR's mirror could not start: " + (why.empty() ? std::string("see record-mirror.log") : why));
        }
        if (g_stop.load()) {
            finishRing();
            hb.join();
            cx.st.state = "idle";
            cx.st.note = "stopped before the first frame";
            cx.save();
            return 0;
        }
        if (std::chrono::steady_clock::now() - t0 > (mirror ? 15s : 6s)) {
            finishRing();
            hb.join();
            if (mirror) {
                mchild.end();
                return cx.fail("SteamVR's mirror did not answer within 15 s (see record-mirror.log)");
            }
            return cx.fail("Monado did not attach: this monado-service has no recorder tap (install the build with "
                           "comp_recorder, docs/record.md)");
        }
        std::this_thread::sleep_for(50ms);
    }
    const auto [w, h] = ring->size();
    std::fprintf(stderr, "pibridge-record: tap attached: %s (%ux%u)\n", ring->producerNote().c_str(), w, h);

    const fs::path out = uniquePath(dir, fileName(static_cast<std::int64_t>(unixNow())));
    const fs::path ffLog = r.session / "record-ffmpeg.log";
    std::size_t encI = 0;
    // Sound: only what can be read right now goes in; a missing output or mic never stops the video.
    Settings es = s;
    std::string audioNote;
    if (es.audio) {
        if (const auto e = probeAudio(ffmpeg, kMonitorSrc)) {
            es.audio = false;
            es.mic = false;
            audioNote = std::format("recording without sound: the audio output cannot be read ({})", *e);
        } else if (es.mic) {
            if (const auto m = probeAudio(ffmpeg, kMicSrc)) {
                es.mic = false;
                audioNote = std::format("the microphone cannot be read, recording the output sound only ({})", *m);
            }
        }
    }
    const auto spawn = [&](Encoder e) { return Ffmpeg::spawn(ffmpeg, e, es, w, h, out, ffLog); };
    auto first = spawn(encs[0]);
    if (!first) {
        finishRing();
        hb.join();
        return cx.fail(std::format("cannot start ffmpeg: {}", first.error()));
    }
    std::optional<Ffmpeg> ff(std::move(*first));
    encoderPid.store(ff->pid());
    cx.st.state = "recording";
    cx.st.file = out.string();
    cx.st.encoder = encoderName(encs[0], s.codec);
    cx.st.audio = !es.audio ? "" : es.mic ? "output+mic" : "output";
    if (!audioNote.empty()) {
        event(r.session, "notice", audioNote, true);
    }
    cx.st.width = w;
    cx.st.height = h;
    cx.save();
    event(r.session, "notice", std::format("recording to {}", out.string()), true);

    Pacer pacer(s.fps);
    std::string reason = "stopped";
    bool critical = false;
    auto lastStatus = std::chrono::steady_clock::now();
    bool writeFailed = false;
    for (;;) {
        progress.store(monotonicNs());
        if (g_stop.load()) {
            break;
        }
        const auto age = ring->producerAgeS();
        if (ring->producerPid() == 0 || (age && *age > 3.0)) {
            if (mirror) {
                mchild.end();
                const std::string why = lastLine(mirrorLog);
                reason = "SteamVR's mirror stopped" + (why.empty() ? std::string() : " (" + why + ")");
            } else {
                reason = "Monado stopped (or detached the tap)";
            }
            break;
        }
        std::optional<std::string> how = ff->exited();
        if (!how && writeFailed) {
            how = "its input closed";
        }
        if (how) {
            // an encoder that dies at once (NVENC unavailable, ...): the next one in the plan, same file name
            if (std::chrono::steady_clock::now() - ff->started < 5s && encI + 1 < encs.size()) {
                encI++;
                fs::remove(out, ec);
                event(r.session, "notice",
                      std::format("{} failed ({}), falling back to {}", encoderName(ff->encoder, s.codec), *how,
                                  encoderName(encs[encI], s.codec)),
                      false);
                auto next = spawn(encs[encI]);
                if (next) {
                    ff.emplace(std::move(*next));
                    encoderPid.store(ff->pid());
                    writeFailed = false;
                    cx.st.encoder = encoderName(encs[encI], s.codec);
                    pacer = Pacer(s.fps);
                    continue;
                }
                reason = std::format("cannot start ffmpeg: {}", next.error());
                critical = true;
                break;
            }
            reason = std::format("the encoder stopped ({}; see {})", *how, ffLog.string());
            critical = true;
            break;
        }
        if (std::chrono::steady_clock::now() - lastStatus >= 1s) {
            lastStatus = std::chrono::steady_clock::now();
            if (const auto f = freeBytes(dir); f && *f < kStopFree) {
                reason = std::format("the disk is almost full (less than 1 GiB free in {})", dir.string());
                critical = true;
                break;
            }
            const Counters k = ring->counters();
            cx.st.frames = ff->frames;
            cx.st.dropped_tap = k.dropped_busy;
            cx.st.dropped_ring = k.dropped_full;
            cx.st.duplicated = pacer.duplicated;
            cx.save();
        }
        if (auto f = ring->take()) {
            const std::uint64_t n = pacer.copies(f->timestamp_ns);
            for (std::uint64_t i = 0; i < n; i++) {
                if (!ff->write(f->pixels())) {
                    writeFailed = true;
                    break;
                }
            }
        } else {
            std::this_thread::sleep_for(2ms);
        }
    }

    // Finalize: the tap detaches, the encoder writes its last fragment.
    cx.st.state = "finalizing";
    cx.save();
    finishRing();
    mchild.end();
    const std::uint64_t frames = ff->frames;
    const std::string how = ff->finish(15s);
    encoderPid.store(0);
    hb.join();
    const Counters k = ring->counters();
    cx.st.frames = frames;
    cx.st.dropped_tap = k.dropped_busy;
    cx.st.dropped_ring = k.dropped_full;
    cx.st.duplicated = pacer.duplicated;
    const auto size = fs::file_size(out, ec);
    const double secs = double(frames) / double(s.fps);
    std::string text = std::format("saved {} ({:.0f} s, {:.1f} MB; {} dropped by the tap, {} repeated){}", out.string(),
                                   secs, double(ec ? 0 : size) / 1e6, k.dropped_busy + k.dropped_full, pacer.duplicated,
                                   how == "ok" ? std::string() : "; encoder: " + how);
    if (reason != "stopped") {
        text = reason + ": " + text;
    }
    if (frames == 0) {
        fs::remove(out, ec);
        text = reason + ": nothing recorded";
        cx.st.file.clear();
    }
    cx.st.state = critical ? "error" : "idle";
    cx.st.note = reason == "stopped" ? std::string() : reason;
    cx.save();
    event(r.session, critical ? "critical" : "notice", text, true);
    return 0;
}

} // namespace pibridge::record
