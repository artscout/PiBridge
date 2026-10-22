// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/main.rs (MIT OR Apache-2.0).
//
// pibridge-cam: the tracking-camera tool (C++ twin of `pimax-cam`). Replays and calib need nothing; the live camera is a
// plain UVC stream (nothing is sent to the headset).
//
//   pibridge-cam info      [--source S] [--frames N]           device, first frames' metadata
//   pibridge-cam stats     [--source S] [--seconds N] [--fps F]
//   pibridge-cam record    OUT.pxcam [--seconds N] [--fps F] [--zlib L] [--frames N] [--source S]
//   pibridge-cam calib     [XML] [--conv 1|2] [--overlay F]     parsed calibration and derived geometry
//   pibridge-cam import    IN.raw OUT.pxcam [--zlib L]          raw YUYV frames (v4l2-ctl dump) -> recording
//   pibridge-cam broker    [--source S] [--fps F] [--socket P] [--slots N] [--loop]   share one camera stream with any
//                       number of processes (they open the source `broker`; PXCB v1, compatible with pimax-cam's
//                       broker and clients). The live camera is reopened after an unplug or a 5 s stall.
//   pibridge-cam broker-status [--socket P] [--window S] [--json]   probe a running broker (exit 0 streaming,
//                       1 no broker, 2 camera reopening / no frames)
//
// SRC / S: empty or `auto` (live camera), `/dev/videoN`, `broker[:SOCKET]`, or a `.pxcam` recording. Standard output
// is the same text as pimax-cam's. Not ported: tiles, rectify, hypotheses, cpu.
#include "pibridge/cam/broker.h"
#include "pibridge/cam/calib.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/clock.h"
#include "pibridge/cam/overlay.h"
#include "pibridge/cam/record.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/stats.h"
#include "pibridge/cam/v4l2.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <expected>
#include <format>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace cam = pibridge::cam;
namespace rfmt = pibridge::cam::rfmt;

namespace {

using R = std::expected<void, std::string>;

std::unexpected<std::string> err(std::string s) {
    return std::unexpected(std::move(s));
}

void out(const std::string& s) {
    std::fwrite(s.data(), 1, s.size(), stdout);
}

void eout(const std::string& s) {
    std::fwrite(s.data(), 1, s.size(), stderr);
}

// The Rust Args helper: options are removed as they are read; what is left must not start with "--".
struct Args {
    std::vector<std::string> v;

    bool flag(const std::string& n) {
        for (std::size_t i = 0; i < v.size(); i++) {
            if (v[i] == n) {
                v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
                return true;
            }
        }
        return false;
    }
    std::expected<std::optional<std::string>, std::string> opt(const std::string& n) {
        for (std::size_t i = 0; i < v.size(); i++) {
            if (v[i] == n) {
                if (i + 1 >= v.size()) {
                    return err(n + " needs a value");
                }
                std::string val = v[i + 1];
                v.erase(v.begin() + static_cast<std::ptrdiff_t>(i), v.begin() + static_cast<std::ptrdiff_t>(i + 2));
                return std::optional{val};
            }
        }
        const std::string pre = n + "=";
        for (std::size_t i = 0; i < v.size(); i++) {
            if (v[i].starts_with(pre)) {
                std::string val = v[i].substr(pre.size());
                v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
                return std::optional{val};
            }
        }
        return std::optional<std::string>{};
    }
    template <class T>
    std::expected<T, std::string> num(const std::string& n, T d) {
        auto o = opt(n);
        if (!o) {
            return std::unexpected(o.error());
        }
        if (!*o) {
            return d;
        }
        const std::string& s = **o;
        bool ok = false;
        T val{};
        if constexpr (std::is_same_v<T, double>) {
            ok = rfmt::parse_f64(s, val);
        } else {
            std::uint64_t u = 0;
            ok = rfmt::parse_u64(s, u) && u <= static_cast<std::uint64_t>(std::numeric_limits<T>::max());
            val = static_cast<T>(u);
        }
        if (!ok) {
            return err(std::format("bad value for {}: {}", n, s));
        }
        return val;
    }
    std::optional<std::string> pos(std::size_t i) const {
        std::size_t k = 0;
        for (const auto& a : v) {
            if (a.starts_with("--")) {
                continue;
            }
            if (k == i) {
                return a;
            }
            k++;
        }
        return std::nullopt;
    }
    void remove(const std::string& x) { std::erase(v, x); }
    R done() const {
        for (const auto& a : v) {
            if (a.starts_with("--")) {
                return err("unknown option " + a);
            }
        }
        return {};
    }
};

#define TRY(var, expr)                                                                                                 \
    auto var = (expr);                                                                                                 \
    if (!var) {                                                                                                        \
        return std::unexpected(var.error());                                                                           \
    }

std::expected<cam::RigConvention, std::string> conv_of(std::uint32_t v) {
    if (v == 1) {
        return cam::RigConvention::Cam0FromCam;
    }
    if (v == 2) {
        return cam::RigConvention::CamFromCam0;
    }
    return err("--conv must be 1 or 2");
}

std::expected<std::unique_ptr<cam::Camera>, std::string> open(const std::string& spec, std::uint32_t fps) {
    auto c = cam::Camera::open(spec, fps, cam::ReplayOptions{});
    if (!c) {
        return err(std::format("open {}: {}", rfmt::debug_str(spec), c.error().text()));
    }
    eout(std::format("source: {}\n", (*c)->describe()));
    return std::move(*c);
}

const char* kind_name(std::uint32_t k) {
    return k == 1 ? "tracking" : (k == 2 ? "led" : "unknown");
}

void print_info(const cam::FrameInfo& i) {
    out(std::format("frame ctr {} seq {} kind {} type {:#04x} idx {} dev {} us host {} ns mono {} ns exp {} gain {} rows "
                    "{} dropped {} flags {:#x}\n",
                    i.counter, i.sequence, kind_name(i.kind), i.frame_type, i.stream_index, i.dev_ts_us, i.host_ts_ns,
                    i.mono_ts_ns, rfmt::debug_u16s(i.exposure), rfmt::debug_u16s(i.gain),
                    rfmt::debug_u16s(i.y_offset), i.dropped, i.flags));
}

R cmd_info(Args a) {
    TRY(src, a.opt("--source"));
    TRY(n, a.num<std::size_t>("--frames", 6));
    TRY(d, a.done());
    const std::string s = src->value_or("");
    if (s.empty()) {
        const auto dev = cam::find_device();
        out(std::format("device: {}\n", dev ? "Some(" + rfmt::debug_str(*dev) + ")" : std::string("None")));
    }
    TRY(c, open(s, 0));
    for (std::size_t k = 0; k < *n; k++) {
        auto f = (*c)->next_frame(1000);
        if (!f) {
            return err(f.error().text());
        }
        if (!*f) {
            return err("no frame within 1 s");
        }
        print_info(**f);
    }
    return {};
}

R cmd_stats(Args a) {
    TRY(src, a.opt("--source"));
    TRY(secs, a.num<double>("--seconds", 10.0));
    TRY(fps, a.num<std::uint32_t>("--fps", 0));
    TRY(d, a.done());
    TRY(c, open(src->value_or(""), *fps));
    cam::CaptureStats st;
    const std::uint64_t t0 = cam::monotonic_ns();
    std::uint64_t timeouts = 0;
    double next_print = 1.0;
    while (static_cast<double>(cam::monotonic_ns() - t0) / 1e9 < *secs) {
        auto f = (*c)->next_frame(500);
        if (!f) {
            return err(f.error().text());
        }
        if (*f) {
            st.add(**f);
        } else if ((*c)->source().finished()) {
            break;
        } else {
            timeouts++;
        }
        const double el = static_cast<double>(cam::monotonic_ns() - t0) / 1e9;
        if (el >= next_print) {
            eout(std::format("  {} s: {} frames ({} tracking, {} led), lost {}\n", rfmt::fixed_w(el, 5, 1), st.frames,
                             (*c)->stats.tracking, (*c)->stats.led, st.dropped));
            next_print += 5.0;
        }
    }
    st.finish();
    out(st.report());
    const auto& s = (*c)->stats;
    out(std::format("  kinds: tracking {} led {} unknown {}; non-alternating {}; rows scanned {}; row/picture mismatch "
                    "{}; corrupt buffers {}; poll timeouts {}\n",
                    s.tracking, s.led, s.unknown, s.kind_repeats, s.scanned_rows, s.row_mismatch,
                    (*c)->corrupt_frames(), timeouts));
    return {};
}

std::atomic<bool> g_stop{false};

extern "C" void on_signal(int) {
    g_stop.store(true, std::memory_order_relaxed);
}

R cmd_record(Args a) {
    TRY(src, a.opt("--source"));
    TRY(secs, a.num<double>("--seconds", 20.0));
    TRY(fps, a.num<std::uint32_t>("--fps", 0));
    TRY(zlib, a.num<std::uint8_t>("--zlib", 0));
    TRY(max_frames, a.num<std::uint64_t>("--frames", UINT64_MAX));
    const auto o = a.pos(0);
    if (!o) {
        return err("record needs an output path");
    }
    a.remove(*o);
    TRY(d, a.done());
    TRY(c, open(src->value_or(""), *fps));
    std::optional<std::uint8_t> level;
    if (*zlib > 0) {
        level = *zlib;
    }
    if (auto r = (*c)->start_recording(*o, level); !r) {
        return err(std::format("{}: {}", *o, r.error().text()));
    }
    cam::CaptureStats st;
    const std::uint64_t t0 = cam::monotonic_ns();
    // SIGINT/SIGTERM end the recording cleanly (the file is flushed)
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    while (static_cast<double>(cam::monotonic_ns() - t0) / 1e9 < *secs && st.frames < *max_frames &&
           !g_stop.load(std::memory_order_relaxed)) {
        auto f = (*c)->next_frame(500);
        if (!f) {
            return err(f.error().text());
        }
        if (*f) {
            st.add(**f);
        } else if ((*c)->source().finished()) {
            break;
        }
    }
    auto r = (*c)->stop_recording();
    if (!r) {
        return err("recording failed: " + r.error().text());
    }
    const auto [frames, bytes] = r->value_or(std::pair<std::uint64_t, std::uint64_t>{0, 0});
    st.finish();
    out(st.report());
    out(std::format("recorded {} frames ({} not queued: disk too slow), {} MB -> {}\n", frames,
                    (*c)->stats.record_dropped, rfmt::fixed(static_cast<double>(bytes) / 1e6, 1), *o));
    return {};
}

R cmd_import(Args a) {
    TRY(zlib, a.num<std::uint8_t>("--zlib", 6));
    const auto inp = a.pos(0);
    const auto outp = a.pos(1);
    if (!inp || !outp) {
        return err("import needs IN.raw OUT.pxcam");
    }
    a.remove(*inp);
    a.remove(*outp);
    TRY(d, a.done());
    auto data = cam::read_file(*inp);
    if (!data) {
        return err(std::format("{}: {}", *inp, data.error().text()));
    }
    const std::size_t fb = cam::FRAME_BYTES;
    if (data->size() % fb != 0 || data->empty()) {
        return err(std::format("{}: {} bytes is not a multiple of {}", *inp, data->size(), fb));
    }
    std::optional<std::uint8_t> level;
    if (*zlib > 0) {
        level = *zlib;
    }
    auto rec = cam::Recorder::create(*outp, cam::FRAME_W, level);
    if (!rec) {
        return err(std::format("{}: {}", *outp, rec.error().text()));
    }
    for (std::size_t i = 0; i * fb < data->size(); i++) {
        cam::RawFrame raw;
        raw.data = std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(data->data()) + i * fb, fb);
        raw.stride = cam::FRAME_W;
        raw.host_ts_ns = 1'000'000'000ull + i * 19'988'000ull;
        raw.sequence = static_cast<std::uint32_t>(i);
        if (auto w = (*rec)->write(raw); !w) {
            return err(w.error().text());
        }
    }
    auto fin = (*rec)->finish();
    if (!fin) {
        return err(fin.error().text());
    }
    out(std::format("imported {} frames, {} MB -> {}\n", fin->first, rfmt::fixed(static_cast<double>(fin->second) / 1e6, 1),
                    *outp));
    return {};
}

std::expected<cam::DeviceCalib, std::string> load_calib(const std::optional<std::string>& p,
                                                        const std::optional<std::string>& overlay) {
    auto r = cam::DeviceCalib::load_default(p);
    if (!r) {
        return err(r.error().text());
    }
    eout(std::format("calibration: {}\n", rfmt::utf8_lossy(r->second)));
    if (!overlay) {
        return std::move(r->first);
    }
    std::string path;
    if (*overlay == "default") {
        const auto d = cam::overlay::default_path();
        if (!d) {
            return err("no default overlay path (HOME unset)");
        }
        path = *d;
    } else {
        path = *overlay;
    }
    auto o = cam::overlay::CalibOverlay::load(path);
    if (!o) {
        return err(o.error().text());
    }
    auto c2 = cam::overlay::with_overlay(r->first, *o, cam::overlay::DEFAULT_MAX_ANGLE_DEG);
    if (!c2) {
        return err(std::format("{}: {}", path, c2.error().text()));
    }
    for (const auto& rot : o->rotations) {
        eout(std::format("overlay {}: camera {} rotated {} deg (rotation vector {} rad)\n", path, rot.id,
                         rfmt::fixed(rot.angle_deg(), 3), rfmt::debug_f64s(rot.rotation_vector)));
    }
    return std::move(*c2);
}

R cmd_calib(Args a) {
    TRY(cv, a.num<std::uint32_t>("--conv", 2));
    TRY(conv, conv_of(*cv));
    TRY(ovp, a.opt("--overlay"));
    const auto p = a.pos(0);
    TRY(d, a.done());
    TRY(cal, load_calib(p, *ovp));
    out(cal->geometry_report(*conv));
    return {};
}

R cmd_broker(Args a) {
    namespace broker = cam::broker;
    TRY(src, a.opt("--source"));
    TRY(fps, a.num<std::uint32_t>("--fps", 0));
    TRY(slots, a.num<std::uint32_t>("--slots", broker::DEFAULT_SLOTS));
    TRY(sock, a.opt("--socket"));
    const std::string socket = sock->value_or(broker::default_socket());
    const bool looping = a.flag("--loop");
    TRY(d, a.done());
    const std::string s = src->value_or("");
    if (rfmt::trim(s).starts_with("broker")) {
        return err("a camera broker cannot serve another broker");
    }
    cam::ReplayOptions replay;
    replay.looping = looping;
    replay.realtime = true;
    auto source = cam::open_source(s, *fps, replay);
    if (!source) {
        return err(std::format("open {}: {}", rfmt::debug_str(s), source.error().text()));
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    eout(std::format("camera broker: {} -> {} ({} slots); clients open the source `broker`{}; Ctrl-C stops\n",
                     (*source)->describe(), socket, *slots,
                     socket == broker::default_socket() ? std::string() : std::format(" (here: `broker:{}`)", socket)));
    broker::ServeOptions opts;
    opts.socket = socket;
    opts.slots = *slots;
    std::uint64_t last = 0;
    bool last_up = true;
    const double every = std::chrono::duration<double>(opts.status_every).count();
    auto st = broker::serve(std::move(*source), opts, g_stop, [&](const broker::ServeStats& x) {
        if (x.camera_up != last_up) {
            last_up = x.camera_up;
            eout(std::format("  camera {} (reopens so far: {}); the broker keeps serving, clients stay connected\n",
                             x.camera_up ? "streaming again" : "lost, reopening", x.camera_reopens));
            return;
        }
        eout(std::format("  camera {}, {} frames/s, {} frames, {} client(s) now ({} ever), {} notices dropped, {} source "
                         "timeouts\n",
                         x.camera_up ? "up" : "REOPENING", rfmt::fixed(static_cast<double>(x.frames - last) / every, 1),
                         x.frames, x.clients, x.clients_total, x.notices_dropped, x.timeouts));
        last = x.frames;
    });
    if (!st) {
        return err(st.error().text());
    }
    eout(std::format("camera broker stopped: {} frames, {} client(s) served, {} notices dropped\n", st->frames,
                     st->clients_total, st->notices_dropped));
    return {};
}

int cmd_broker_status(Args a, R& r) {
    namespace broker = cam::broker;
    auto sock = a.opt("--socket");
    if (!sock) {
        r = std::unexpected(sock.error());
        return 1;
    }
    const std::string socket = sock->value_or(broker::default_socket());
    auto window = a.num<double>("--window", 1.0);
    if (!window) {
        r = std::unexpected(window.error());
        return 1;
    }
    const bool json = a.flag("--json");
    if (auto d = a.done(); !d) {
        r = d;
        return 1;
    }
    const double w = rfmt::clamp(*window, 0.05, 30.0);
    auto p = broker::probe(socket, std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(w)));
    if (!p) {
        std::string e = p.error().text();
        if (json) {
            std::replace(e.begin(), e.end(), '"', '\'');
            out(std::format("{{\"broker\":false,\"error\":\"{}\"}}\n", e));
        } else {
            eout(std::format("no broker: {}\n", e));
        }
        return 1;
    }
    if (json) {
        out(p->to_json() + "\n");
    } else {
        const char* cam_state = p->camera_up ? (*p->camera_up ? "streaming" : "REOPENING") : "unknown (older broker)";
        out(std::format("broker {}: camera {}, {} frames/s, {} frames published, {} camera reopens, heartbeat {}\n",
                        socket, cam_state, rfmt::fixed(p->fps, 1), p->published, p->camera_reopens,
                        p->heartbeat_age_s ? rfmt::fixed(*p->heartbeat_age_s, 2) + " s ago" : std::string("n/a")));
    }
    if (p->camera_up == std::optional<bool>(false) || p->fps <= 0.0) {
        return 2;
    }
    return 0;
}

const char* usage() {
    return "usage: pibridge-cam info|stats|record|calib|import|broker|broker-status ... (see the header of "
           "tools/cam/src/main.cpp)";
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> argv_s(argv + 1, argv + argc);
    if (argv_s.empty() || argv_s[0] == "-h" || argv_s[0] == "--help") {
        eout(std::format("{}\n", usage()));
        return 2;
    }
    const std::string cmd = argv_s[0];
    Args a{std::vector<std::string>(argv_s.begin() + 1, argv_s.end())};
    R r;
    int code = 0;
    if (cmd == "info") {
        r = cmd_info(std::move(a));
    } else if (cmd == "stats") {
        r = cmd_stats(std::move(a));
    } else if (cmd == "record") {
        r = cmd_record(std::move(a));
    } else if (cmd == "calib") {
        r = cmd_calib(std::move(a));
    } else if (cmd == "import") {
        r = cmd_import(std::move(a));
    } else if (cmd == "broker") {
        r = cmd_broker(std::move(a));
    } else if (cmd == "broker-status") {
        code = cmd_broker_status(std::move(a), r);
        if (r) {
            std::fflush(stdout);
            return code;
        }
    } else if (cmd == "tiles" || cmd == "rectify" || cmd == "hypotheses" || cmd == "cpu") {
        r = err(std::format("`{}` is not ported to pibridge-cam yet (omarchyvr's pimax-cam has it)", cmd));
    } else {
        r = err(usage());
    }
    std::fflush(stdout);
    if (!r) {
        eout(std::format("pibridge-cam {}: {}\n", cmd, r.error()));
        return 1;
    }
    return 0;
}
