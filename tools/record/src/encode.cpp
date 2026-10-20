// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/encode.rs (MIT OR Apache-2.0).
#include "pibridge/record/encode.h"

#include "pibridge/record/proc.h"
#include "pibridge_env.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <format>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

std::string_view encoderName(Encoder e, Codec c) {
    if (e == Encoder::Nvenc) {
        return c == Codec::H264 ? "h264_nvenc" : "hevc_nvenc";
    }
    return c == Codec::H264 ? "libx264" : "libx265";
}

Available Available::parse(std::string_view encoders) {
    const auto has = [&](std::string_view n) {
        std::size_t start = 0;
        while (start < encoders.size()) {
            const std::size_t end = encoders.find('\n', start);
            const std::string_view line =
                encoders.substr(start, end == std::string_view::npos ? std::string_view::npos : end - start);
            start = end == std::string_view::npos ? encoders.size() : end + 1;
            // the second whitespace-separated word
            std::size_t i = line.find_first_not_of(" \t\r");
            if (i == std::string_view::npos) {
                continue;
            }
            i = line.find_first_of(" \t\r", i);
            if (i == std::string_view::npos) {
                continue;
            }
            i = line.find_first_not_of(" \t\r", i);
            if (i == std::string_view::npos) {
                continue;
            }
            const std::size_t j = line.find_first_of(" \t\r", i);
            if (line.substr(i, j == std::string_view::npos ? std::string_view::npos : j - i) == n) {
                return true;
            }
        }
        return false;
    };
    return {has("h264_nvenc"), has("hevc_nvenc"), has("libx264"), has("libx265")};
}

namespace {
// Run argv with stdin /dev/null, collect stdout (or stderr), wait up to `timeout`. nullopt: could not run.
struct RunOut {
    int status = 0;
    std::string out;
    bool timedOut = false;
};
std::optional<RunOut> runCapture(const std::vector<std::string>& args, bool wantStderr, std::chrono::milliseconds timeout) {
    std::vector<std::string> store = args;
    std::vector<char*> argv;
    for (auto& s : store) {
        argv.push_back(s.data());
    }
    argv.push_back(nullptr);
    int p[2], e[2];
    if (pipe2(p, O_CLOEXEC) != 0) {
        return std::nullopt;
    }
    if (pipe2(e, O_CLOEXEC) != 0) {
        ::close(p[0]);
        ::close(p[1]);
        return std::nullopt;
    }
    const pid_t pid = fork();
    if (pid < 0) {
        ::close(p[0]);
        ::close(p[1]);
        ::close(e[0]);
        ::close(e[1]);
        return std::nullopt;
    }
    if (pid == 0) {
        const int nul = ::open("/dev/null", O_RDWR);
        dup2(nul, 0);
        dup2(wantStderr ? nul : p[1], 1);
        dup2(wantStderr ? p[1] : nul, 2);
        signal(SIGPIPE, SIG_DFL);
        execvp(argv[0], argv.data());
        const int err = errno;
        (void)!write(e[1], &err, sizeof err);
        _exit(127);
    }
    ::close(p[1]);
    ::close(e[1]);
    int err = 0;
    const ssize_t en = read(e[0], &err, sizeof err);
    ::close(e[0]);
    if (en == sizeof err) {
        ::close(p[0]);
        waitpid(pid, nullptr, 0);
        return std::nullopt;
    }
    RunOut r;
    const auto t0 = std::chrono::steady_clock::now();
    char buf[4096];
    for (;;) {
        const auto left = timeout - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
        if (left <= 0ms) {
            r.timedOut = true;
            break;
        }
        pollfd pf{p[0], POLLIN, 0};
        const int pr = poll(&pf, 1, static_cast<int>(left.count()));
        if (pr < 0 && errno == EINTR) {
            continue;
        }
        if (pr <= 0) {
            continue;
        }
        const ssize_t n = read(p[0], buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        r.out.append(buf, static_cast<std::size_t>(n));
    }
    ::close(p[0]);
    if (r.timedOut) {
        kill(pid, SIGKILL);
    }
    int st = 0;
    while (waitpid(pid, &st, 0) < 0 && errno == EINTR) {
    }
    r.status = st;
    return r;
}
} // namespace

std::optional<Available> Available::probe(const fs::path& ffmpeg) {
    const auto r = runCapture({ffmpeg.string(), "-hide_banner", "-encoders"}, false, 20s);
    if (!r) {
        return std::nullopt;
    }
    return parse(r->out);
}

std::expected<std::vector<Encoder>, std::string> plan(EncoderPref pref, Codec codec, const Available& avail, bool gpuOk) {
    std::vector<Encoder> v;
    const bool nv = avail.nvenc(codec) && gpuOk;
    if (pref == EncoderPref::Nvenc) {
        if (!gpuOk) {
            return std::unexpected(std::string("the GPU safety gate is not ok: NVENC is not used (try --encoder cpu)"));
        }
        if (!avail.nvenc(codec)) {
            return std::unexpected(std::format("ffmpeg has no {}", encoderName(Encoder::Nvenc, codec)));
        }
        v.push_back(Encoder::Nvenc);
    } else if (pref == EncoderPref::Auto && nv) {
        v.push_back(Encoder::Nvenc);
    }
    if (pref != EncoderPref::Nvenc && avail.cpu(codec)) {
        v.push_back(Encoder::Cpu);
    }
    if (v.empty()) {
        return std::unexpected(std::format("ffmpeg has neither {} nor {}", encoderName(Encoder::Nvenc, codec),
                                           encoderName(Encoder::Cpu, codec)));
    }
    return v;
}

std::vector<std::string> qualityArgs(Encoder enc, Codec codec, Quality q) {
    const bool h264 = codec == Codec::H264;
    if (enc == Encoder::Nvenc) {
        const char* cq = nullptr;
        const char* max = nullptr;
        switch (q) {
        case Quality::Low: cq = h264 ? "30" : "32"; max = h264 ? "8M" : "6M"; break;
        case Quality::Medium: cq = h264 ? "25" : "27"; max = h264 ? "20M" : "15M"; break;
        case Quality::High: cq = h264 ? "20" : "22"; max = h264 ? "50M" : "40M"; break;
        }
        return {"-preset", "p4", "-tune", "hq", "-rc", "vbr", "-cq", cq, "-b:v", "0", "-maxrate", max, "-bufsize", max};
    }
    const char* crf = nullptr;
    switch (q) {
    case Quality::Low: crf = h264 ? "28" : "30"; break;
    case Quality::Medium: crf = h264 ? "23" : "26"; break;
    case Quality::High: crf = h264 ? "19" : "22"; break;
    }
    std::vector<std::string> v{"-preset", "veryfast", "-crf", crf, "-threads", "4"};
    if (!h264) {
        v.insert(v.end(), {"-x265-params", "log-level=error"});
    }
    return v;
}

std::vector<std::string> audioInputs(const Settings& s) {
    std::vector<std::string> a;
    if (!s.audio) {
        return a;
    }
    std::vector<const char*> src{kMonitorSrc};
    if (s.mic) {
        src.push_back(kMicSrc);
    }
    for (const char* d : src) {
        a.insert(a.end(), {"-thread_queue_size", "1024", "-f", "pulse", "-i", d});
    }
    return a;
}

std::vector<std::string> audioOutput(const Settings& s) {
    if (!s.audio) {
        return {"-map", "0:v:0", "-an"};
    }
    std::vector<std::string> a;
    if (s.mic) {
        a = {"-filter_complex", "[1:a][2:a]amix=inputs=2:duration=longest:normalize=0,aresample=async=1000,apad[a]",
             "-map", "0:v:0", "-map", "[a]"};
    } else {
        a = {"-af", "aresample=async=1000,apad", "-map", "0:v:0", "-map", "1:a:0"};
    }
    a.insert(a.end(), {"-c:a", "aac", "-b:a", "160k", "-ar", "48000", "-ac", "2", "-shortest"});
    return a;
}

std::optional<std::string> probeAudio(const fs::path& ffmpeg, const std::string& src) {
    const auto r = runCapture({ffmpeg.string(), "-hide_banner", "-loglevel", "error", "-nostdin", "-f", "pulse", "-i", src,
                               "-t", "0.3", "-f", "null", "-"},
                              true, 4s);
    if (!r) {
        return std::string("cannot run ") + ffmpeg.string();
    }
    if (r->timedOut) {
        return std::string("timed out");
    }
    if (WIFEXITED(r->status) && WEXITSTATUS(r->status) == 0) {
        return std::nullopt;
    }
    // the last line of stderr, trimmed
    std::string_view t = r->out;
    while (!t.empty() && (t.back() == '\n' || t.back() == '\r')) {
        t.remove_suffix(1);
    }
    const std::size_t nl = t.rfind('\n');
    std::string_view last = nl == std::string_view::npos ? t : t.substr(nl + 1);
    while (!last.empty() && (last.front() == ' ' || last.front() == '\t')) {
        last.remove_prefix(1);
    }
    while (!last.empty() && (last.back() == ' ' || last.back() == '\t' || last.back() == '\r')) {
        last.remove_suffix(1);
    }
    return last.empty() ? std::string("ffmpeg failed") : std::string(last);
}

std::vector<std::string> ffmpegArgs(Encoder enc, const Settings& s, std::uint32_t w, std::uint32_t h, const fs::path& out) {
    std::vector<std::string> a{"-hide_banner", "-loglevel", "warning", "-nostdin", "-y",
                               "-f", "rawvideo", "-pix_fmt", "rgba", "-video_size", std::format("{}x{}", w, h),
                               "-framerate", std::to_string(s.fps), "-i", "pipe:0"};
    const auto add = [&](const std::vector<std::string>& v) { a.insert(a.end(), v.begin(), v.end()); };
    add(audioInputs(s));
    add({"-c:v", std::string(encoderName(enc, s.codec))});
    add(qualityArgs(enc, s.codec, s.quality));
    add(audioOutput(s));
    // HEVC in mp4 must be tagged hvc1 for most players
    add({"-pix_fmt", "yuv420p", "-g", std::to_string(s.fps * 2), "-tag:v", s.codec == Codec::Hevc ? "hvc1" : "avc1",
         "-movflags", "+frag_keyframe+empty_moov+default_base_moof", "-f", "mp4", out.string()});
    return a;
}

fs::path ffmpegPath() {
    if (const char* p = pibridge::env("FFMPEG"); p != nullptr && *p != '\0') {
        return p;
    }
    return "ffmpeg";
}

std::expected<Ffmpeg, std::string> Ffmpeg::spawn(const fs::path& ffmpeg, Encoder enc, const Settings& s, std::uint32_t w,
                                                 std::uint32_t h, const fs::path& out, const fs::path& log) {
    const int logfd = ::open(log.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (logfd < 0) {
        return std::unexpected(std::string(std::strerror(errno)));
    }
    std::vector<std::string> store{ffmpeg.string()};
    const auto args = ffmpegArgs(enc, s, w, h, out);
    store.insert(store.end(), args.begin(), args.end());
    std::vector<char*> argv;
    for (auto& x : store) {
        argv.push_back(x.data());
    }
    argv.push_back(nullptr);
    int in[2], e[2];
    if (pipe2(in, O_CLOEXEC) != 0) {
        ::close(logfd);
        return std::unexpected(std::string(std::strerror(errno)));
    }
    if (pipe2(e, O_CLOEXEC) != 0) {
        ::close(logfd);
        ::close(in[0]);
        ::close(in[1]);
        return std::unexpected(std::string(std::strerror(errno)));
    }
    const pid_t pid = fork();
    if (pid < 0) {
        const int err = errno;
        ::close(logfd);
        ::close(in[0]);
        ::close(in[1]);
        ::close(e[0]);
        ::close(e[1]);
        return std::unexpected(std::string(std::strerror(err)));
    }
    if (pid == 0) {
        dup2(in[0], 0);
        const int nul = ::open("/dev/null", O_WRONLY);
        dup2(nul, 1);
        dup2(logfd, 2);
        setpriority(PRIO_PROCESS, 0, 10);
        // IOPRIO_CLASS_BE (2) level 7: the lowest best-effort I/O priority
        syscall(SYS_ioprio_set, 1, 0, (2 << 13) | 7);
        // die with the recorder (a crashed recorder must not leave an encoder behind)
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        signal(SIGPIPE, SIG_DFL);
        signal(SIGINT, SIG_DFL);
        signal(SIGTERM, SIG_DFL);
        sigset_t none;
        sigemptyset(&none);
        sigprocmask(SIG_SETMASK, &none, nullptr);
        execvp(argv[0], argv.data());
        const int err = errno;
        (void)!::write(e[1], &err, sizeof err);
        _exit(127);
    }
    ::close(logfd);
    ::close(in[0]);
    ::close(e[1]);
    int err = 0;
    ssize_t n;
    do {
        n = read(e[0], &err, sizeof err);
    } while (n < 0 && errno == EINTR);
    ::close(e[0]);
    if (n == sizeof err) {
        ::close(in[1]);
        waitpid(pid, nullptr, 0);
        return std::unexpected(std::string(std::strerror(err)));
    }
    Ffmpeg f;
    f.pid_ = pid;
    f.stdin_ = in[1];
    f.started = std::chrono::steady_clock::now();
    f.encoder = enc;
    f.out = out;
    f.audio_ = s.audio;
    return f;
}

Ffmpeg::Ffmpeg(Ffmpeg&& o) noexcept { *this = std::move(o); }

Ffmpeg& Ffmpeg::operator=(Ffmpeg&& o) noexcept {
    if (this != &o) {
        started = o.started;
        encoder = o.encoder;
        out = std::move(o.out);
        frames = o.frames;
        pid_ = std::exchange(o.pid_, -1);
        stdin_ = std::exchange(o.stdin_, -1);
        audio_ = o.audio_;
        status_ = o.status_;
    }
    return *this;
}

Ffmpeg::~Ffmpeg() {
    if (stdin_ >= 0) {
        ::close(stdin_);
    }
    if (pid_ > 0 && !status_) {
        // a replaced encoder (the NVENC fallback) or an early return: reap it, never leave it running
        int st = 0;
        if (waitpid(pid_, &st, WNOHANG) == 0) {
            kill(pid_, SIGKILL);
            waitpid(pid_, &st, 0);
        }
    }
}

bool Ffmpeg::write(std::span<const std::uint8_t> rgba) {
    if (stdin_ < 0) {
        return false;
    }
    std::size_t off = 0;
    while (off < rgba.size()) {
        const ssize_t n = ::write(stdin_, rgba.data() + off, rgba.size() - off);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        off += static_cast<std::size_t>(n);
    }
    frames++;
    return true;
}

std::optional<std::string> Ffmpeg::exited() {
    if (!status_) {
        int st = 0;
        if (waitpid(pid_, &st, WNOHANG) == pid_) {
            status_ = st;
        }
    }
    if (status_) {
        return exitStatusText(*status_);
    }
    return std::nullopt;
}

std::string Ffmpeg::finish(std::chrono::milliseconds grace) {
    if (stdin_ >= 0) {
        ::close(stdin_);
        stdin_ = -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    bool sentInt = false;
    if (audio_) {
        grace = std::min(grace, std::chrono::milliseconds(2000));
    }
    for (;;) {
        if (exited()) {
            return WIFEXITED(*status_) && WEXITSTATUS(*status_) == 0 ? std::string("ok") : exitStatusText(*status_);
        }
        const auto el = std::chrono::steady_clock::now() - t0;
        if (el > grace && !sentInt) {
            kill(pid_, SIGINT);
            sentInt = true;
        }
        if (el > grace + 5s) {
            kill(pid_, SIGKILL);
            int st = 0;
            waitpid(pid_, &st, 0);
            status_ = st;
            return "killed after the grace (the file is playable up to its last fragment)";
        }
        std::this_thread::sleep_for(50ms);
    }
}

} // namespace pibridge::record
