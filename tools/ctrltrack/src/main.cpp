// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/main.rs (MIT OR Apache-2.0).
//
// pibridge-ctrltrack: the Pimax controller LED tracker. `live` publishes the poses for Monado (the session runs
// `live --source broker:<socket> --factory`), `track` solves a recording offline, `scan` lists the blobs of the LED
// frames, `show`/`health` read the pose file. Options, output and exit codes are the Rust tool's. The analysis
// commands (lockrate, fusereplay, imufit, occlusion) and `dump` are not ported: use the Rust pimax-ctrltrack.
#include "cli.h"

#include "pibridge/cam/frame.h"
#include "pibridge/cam/record.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/blob.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <fstream>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>
#include <zlib.h>

namespace pibridge::ctrltool {

namespace rf = cam::rfmt;
namespace tr = ctrl::track;

void eprintln(std::string_view s) {
    std::string l(s);
    l += '\n';
    std::fwrite(l.data(), 1, l.size(), stderr);
}

void println(std::string_view s) {
    std::string l(s);
    l += '\n';
    std::fwrite(l.data(), 1, l.size(), stdout);
}

void panic(std::string_view msg) {
    std::fflush(stdout);
    const long tid = ::syscall(SYS_gettid);
    std::fprintf(stderr,
                 "\nthread 'main' (%ld) panicked at pibridge-ctrltrack:\n%.*s\nnote: run with `RUST_BACKTRACE=1` "
                 "environment variable to display a backtrace\n",
                 tid, static_cast<int>(msg.size()), msg.data());
    std::exit(101);
}

const std::string& at(std::span<const std::string> a, std::size_t i) {
    if (i >= a.size()) {
        panic(std::format("index out of bounds: the len is {} but the index is {}", a.size(), i));
    }
    return a[i];
}

std::size_t parse_usize(std::string_view s) {
    const char* kind = nullptr;
    std::string_view d = s;
    if (d.empty()) {
        kind = "Empty";
    } else {
        if (d[0] == '+' && d.size() > 1) {
            d.remove_prefix(1);
        }
        std::size_t v = 0;
        for (const char c : d) {
            if (c < '0' || c > '9') {
                kind = "InvalidDigit";
                break;
            }
            const std::size_t dig = static_cast<std::size_t>(c - '0');
            if (v > (SIZE_MAX - dig) / 10) {
                kind = "PosOverflow";
                break;
            }
            v = v * 10 + dig;
        }
        if (kind == nullptr) {
            return v;
        }
    }
    panic(std::format("called `Result::unwrap()` on an `Err` value: ParseIntError {{ kind: {} }}", kind));
}

std::optional<double> parse_f64_ok(std::string_view s) {
    double v = 0;
    if (!rf::parse_f64(s, v)) {
        return std::nullopt;
    }
    return v;
}

double parse_f64(std::string_view s) {
    if (const auto v = parse_f64_ok(s)) {
        return *v;
    }
    panic(std::format("called `Result::unwrap()` on an `Err` value: ParseFloatError {{ kind: {} }}",
                      s.empty() ? "Empty" : "Invalid"));
}

std::string io_debug(const cam::IoError& e) {
    const auto kind_name = [](cam::ErrorKind k) {
        switch (k) {
        case cam::ErrorKind::NotFound: return "NotFound";
        case cam::ErrorKind::PermissionDenied: return "PermissionDenied";
        case cam::ErrorKind::InvalidData: return "InvalidData";
        case cam::ErrorKind::InvalidInput: return "InvalidInput";
        case cam::ErrorKind::TimedOut: return "TimedOut";
        case cam::ErrorKind::ConnectionAborted: return "ConnectionAborted";
        case cam::ErrorKind::AddrInUse: return "AddrInUse";
        case cam::ErrorKind::WouldBlock: return "WouldBlock";
        case cam::ErrorKind::Interrupted: return "Interrupted";
        case cam::ErrorKind::Unsupported: return "Unsupported";
        case cam::ErrorKind::Other: break;
        }
        return "Other";
    };
    if (e.os != 0) {
        // Rust's decode_error_kind for the codes a file open or read meets
        const char* k = "Uncategorized";
        switch (e.os) {
        case ENOENT: k = "NotFound"; break;
        case EPERM:
        case EACCES: k = "PermissionDenied"; break;
        case EISDIR: k = "IsADirectory"; break;
        case ENOTDIR: k = "NotADirectory"; break;
        case EINVAL: k = "InvalidInput"; break;
        case ELOOP: k = "FilesystemLoop"; break;
        case ENAMETOOLONG: k = "InvalidFilename"; break;
        case EINTR: k = "Interrupted"; break;
        default: break;
        }
        return std::format("Os {{ code: {}, kind: {}, message: {} }}", e.os, k, rf::debug_str(cam::os_error_text(e.os)));
    }
    return std::format("Custom {{ kind: {}, error: {} }}", kind_name(e.kind), rf::debug_str(e.msg));
}

cam::Result<void> write_png_rgb(const std::string& path, std::size_t width, std::size_t height,
                                std::span<const std::uint8_t> rgb) {
    const std::size_t rowlen = width * 3;
    std::vector<std::uint8_t> filtered;
    filtered.reserve((rowlen + 1) * height);
    for (std::size_t y = 0; y < height; y++) {
        const std::uint8_t* row = rgb.data() + y * rowlen;
        filtered.push_back(1); // Sub
        for (std::size_t x = 0; x < rowlen; x++) {
            const std::uint8_t left = x >= 3 ? row[x - 3] : 0;
            filtered.push_back(static_cast<std::uint8_t>(row[x] - left));
        }
    }
    uLongf zlen = compressBound(static_cast<uLong>(filtered.size()));
    std::vector<std::uint8_t> z(zlen);
    if (compress2(z.data(), &zlen, filtered.data(), static_cast<uLong>(filtered.size()), 6) != Z_OK) {
        return cam::fail(cam::IoError::other("zlib"));
    }
    z.resize(zlen);
    std::vector<std::uint8_t> out{0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
    const auto be32 = [&](std::uint32_t v) {
        for (int s = 24; s >= 0; s -= 8) {
            out.push_back(static_cast<std::uint8_t>(v >> s));
        }
    };
    const auto chunk = [&](const char* type, std::span<const std::uint8_t> data) {
        be32(static_cast<std::uint32_t>(data.size()));
        const std::size_t start = out.size();
        out.insert(out.end(), type, type + 4);
        out.insert(out.end(), data.begin(), data.end());
        const uLong crc = crc32(0L, out.data() + start, static_cast<uInt>(out.size() - start));
        be32(static_cast<std::uint32_t>(crc));
    };
    std::vector<std::uint8_t> ihdr;
    for (const std::uint32_t v : {static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)}) {
        for (int s = 24; s >= 0; s -= 8) {
            ihdr.push_back(static_cast<std::uint8_t>(v >> s));
        }
    }
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});
    chunk("IHDR", ihdr);
    chunk("IDAT", z);
    chunk("IEND", {});
    return cam::write_file(path, std::string_view(reinterpret_cast<const char*>(out.data()), out.size()));
}

std::size_t available_parallelism() {
    std::size_t quota = SIZE_MAX;
    // cgroup v2: the smallest cpu.max quota from this process's cgroup up to the root
    if (const auto cg = cam::read_to_string("/proc/self/cgroup")) {
        std::string_view t = *cg;
        const std::size_t p = t.find("0::");
        if (p != std::string_view::npos && (p == 0 || t[p - 1] == '\n')) {
            std::string_view rel = t.substr(p + 3);
            rel = rel.substr(0, rel.find('\n'));
            std::string dir = "/sys/fs/cgroup" + std::string(rel);
            for (;;) {
                if (const auto m = cam::read_to_string(dir + "/cpu.max")) {
                    const auto parts = rf::split_whitespace(*m);
                    std::uint64_t q = 0;
                    std::uint64_t per = 0;
                    if (parts.size() == 2 && rf::parse_u64(parts[0], q) && rf::parse_u64(parts[1], per) && per > 0) {
                        quota = std::min<std::size_t>(quota, static_cast<std::size_t>(q / per));
                    }
                }
                if (dir == "/sys/fs/cgroup") {
                    break;
                }
                dir = dir.substr(0, dir.rfind('/'));
                if (dir.size() < std::string("/sys/fs/cgroup").size()) {
                    break;
                }
            }
        }
    }
    quota = std::max<std::size_t>(quota, 1);
    cpu_set_t set;
    CPU_ZERO(&set);
    if (::sched_getaffinity(0, sizeof set, &set) == 0) {
        const auto count = std::min<std::size_t>(static_cast<std::size_t>(CPU_COUNT(&set)), quota);
        if (count > 0) {
            return count;
        }
    }
    const long n = ::sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? static_cast<std::size_t>(n) : 4;
}

namespace {

[[noreturn]] void usage() {
    std::fputs(
        "usage: pimax-ctrltrack scan FILE.pxcam [--every N]\n       pimax-ctrltrack dump FILE.pxcam FRAME OUT.png   "
        "(LED frame mosaic, blobs marked red)\n       pimax-ctrltrack track FILE.pxcam [--start N] [--count N] "
        "[--every N] [--calib XML] [--left JSON] [--right JSON] [--csv OUT] [--no-prior] [--overlay DIR "
        "[--overlay-every N]] [--fit-extrinsics [--save-extrinsics P]] [--extrinsics P | --factory]\n       "
        "pimax-ctrltrack live [--source broker|FILE] [--out POSEFILE] [--seconds N] [--aim-pitch-deg D] "
        "[--extrinsics P | --factory] [--baseline] [--no-body] [--thr T] [--peak P]\n       pimax-ctrltrack lockrate "
        "FILE.pxcam [--baseline] [--oracle] [--start N] [--count N] [--csv OUT] [--few N] [--wide PX] [--no-predict] "
        "[--rot-w W] [--thr T] [--peak P]   (offline lock-rate replay through the live tracker)\n       "
        "pimax-ctrltrack show [--out POSEFILE] [--seconds N]\n       pimax-ctrltrack health [--out POSEFILE] "
        "[--max-age S]   (heartbeat as JSON; exit 0 fresh, 1 no file / no heartbeat, 2 stale or source down)\n       "
        "pimax-ctrltrack imufit LOG.csv... [--pool] [--save [PATH]]   (controller gyro calibration from a "
        "PIMAX_CTRL_IMU_LOG log; --save writes ~/.config/omarchyvr/ctrl-imu.json)\n       pimax-ctrltrack "
        "fusereplay IN.csv OUT.csv [--no-filter] [--no-still-lock] [--legacy]   (replay a log through the fuser, "
        "A/B)\n       pimax-ctrltrack occlusion LOG.csv... [--gaps 0.5,1,2] [--fit-forearm]   (masked replay: hide "
        "visible fixes, score the out-of-view pose)\n",
        stderr);
    std::exit(2);
}

/// `scan FILE.pxcam [--every N]`: blob counts and the brightest pixel per camera of every LED frame.
int scan(const std::string& path, std::size_t every) {
    auto r = cam::Reader::open(path);
    if (!r) {
        panic("open: " + io_debug(r.error()));
    }
    const tr::BlobParams prm;
    std::vector<std::uint8_t> tile(cam::TILE_W * cam::TILE_H);
    std::size_t n_led = 0;
    std::array<std::array<std::size_t, 8>, cam::TILE_COUNT> hist{};
    std::size_t i = 0;
    cam::Record rec;
    for (;;) {
        auto got = (*r)->next_record(rec);
        if (!got) {
            panic("read: " + io_debug(got.error()));
        }
        if (!*got) {
            break;
        }
        i += 1;
        const auto m = cam::Meta::parse(std::span(rec.data).first(cam::META_LEN));
        if (!m) {
            continue;
        }
        if (m->kind() != cam::FrameKind::Led) {
            continue;
        }
        n_led += 1;
        if (every == 0) {
            panic("attempt to calculate the remainder with a divisor of zero");
        }
        if (n_led % every != 0) {
            continue;
        }
        const cam::TileRows rows = cam::tile_rows(&*m, rec.data, rec.stride);
        std::string line =
            std::format("frame {:5} ts {} exp {} gain {}", i, rf::fixed(static_cast<double>(m->ts_us) * 1e-6, 3),
                        rf::debug_u16s(m->exposure), rf::debug_u16s(m->gain));
        for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
            cam::copy_tile(rec.data, rec.stride, t, rows.y[t], tile, cam::TILE_W);
            const auto b = tr::detect(tile, cam::TILE_W, cam::TILE_H, cam::TILE_W, prm);
            hist[t][std::min<std::size_t>(b.size(), 7)] += 1;
            const unsigned mx = tile.empty() ? 0 : *std::max_element(tile.begin(), tile.end());
            line += std::format(" | c{} n={:2} max={:3}", t, b.size(), mx);
        }
        println(line);
    }
    eprintln(std::format("LED frames {}; blob-count histogram per camera (0..6, 7+):", n_led));
    for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
        std::string h = "[";
        for (std::size_t k = 0; k < 8; k++) {
            h += (k > 0 ? ", " : "") + std::to_string(hist[t][k]);
        }
        eprintln(std::format("  cam{}: {}]", t, h));
    }
    return 0;
}

int not_ported(std::string_view cmd) {
    eprintln(std::format("pibridge-ctrltrack: `{}` is not ported to PiBridge (an offline analysis tool); use the Rust "
                         "pimax-ctrltrack {} from omarchyvr",
                         cmd, cmd));
    return 2;
}

} // namespace

} // namespace pibridge::ctrltool

int main(int argc, char** argv) {
    using namespace pibridge::ctrltool;
    // Rust's stdout is line-buffered whatever it is connected to: the same interleaving with stderr in a log
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::vector<std::string> a(argv, argv + argc);
    if (a.size() < 2) {
        usage();
    }
    const std::string& cmd = a[1];
    const std::span<const std::string> rest = std::span<const std::string>(a).subspan(2);
    if (cmd == "scan") {
        const std::string& path = at(a, 2);
        std::size_t every = 1;
        for (std::size_t i = 0; i < a.size(); i++) {
            if (a[i] == "--every") {
                every = parse_usize(at(a, i + 1));
                break;
            }
        }
        return scan(path, every);
    }
    if (cmd == "track") {
        return cmd_track(rest);
    }
    if (cmd == "live") {
        return cmd_live(rest);
    }
    if (cmd == "show") {
        return cmd_show(rest);
    }
    if (cmd == "health") {
        return cmd_health(rest);
    }
    if (cmd == "lockrate" || cmd == "fusereplay" || cmd == "imufit" || cmd == "occlusion" || cmd == "dump") {
        return not_ported(cmd);
    }
    usage();
}
