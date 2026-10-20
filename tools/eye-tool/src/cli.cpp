// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/cli.rs (MIT OR Apache-2.0).
//
// pibridge-eye pcap    <capture.pcap> [--device N] [--plan]
// pibridge-eye plan    <capture.pcap> [--device N]
// pibridge-eye frames  <capture.pcap> -o FILE [--device N] [--keep-auth]
// pibridge-eye profile <calibration.setpm> -o FILE
//
// Nothing here talks to a device: these commands read files and write files.
#include "cli.h"

#include "pibridge/eye/analysis.h"
#include "pibridge/eye/frames.h"

#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace eyetool {

namespace {

constexpr const char* USAGE = R"(pibridge-eye: offline tools for the Tobii EyeChip (2104:0561) of Pimax headsets

  pibridge-eye pcap    <capture.pcap> [--device N] [--plan]
        decode a Windows USBPcap capture: TTP messages, gaze notification counts, replay plan completeness
  pibridge-eye plan    <capture.pcap> [--device N]
        list the exact transfers of the Windows session start and which ones the capture cuts
  pibridge-eye frames  <capture.pcap> -o FILE [--device N] [--keep-auth]
        write the eye chip start list of a Windows capture of YOUR headset (every byte as Pimax Play sent it, auth
        steps left out) for the eye daemon (eye_start_list in pibridge.conf): headsets without a built-in list
        (the Crystal Super). The file holds your eye profile upload: keep it private (it is written mode 0600).
  pibridge-eye profile <calibration.setpm> -o FILE
        take the eye profile out of the file Pimax's calibration saved under Windows (ProgramData/Tobii/Tobii
        Platform Runtime/XR5EYECHIP/<headset>/calibration.setpm); the eye daemon uploads it at every start in place
        of the start list's older one (eye_profile in pibridge.conf). Private, like the start list.

Nothing in this tool talks to a device. The live eye tracker is pibridge-eyed.
)";

struct Fail {
    int code;
    std::string msg;
};

Fail usage(std::string m) {
    return {1, std::move(m)};
}

// Write `data` to `dest` with mode 0600 from the first byte on (the file holds private eye data).
std::string write_private(const std::string& dest, const void* data, std::size_t size) {
    const int fd = ::open(dest.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) {
        return std::format("{}: {}", dest, std::strerror(errno));
    }
    std::string error;
    if (::fchmod(fd, 0600) != 0) {
        error = std::format("{}: {}", dest, std::strerror(errno));
    }
    const auto* p = static_cast<const char*>(data);
    std::size_t left = size;
    while (error.empty() && left > 0) {
        const ssize_t w = ::write(fd, p, left);
        if (w < 0 && errno == EINTR) {
            continue;
        }
        if (w <= 0) {
            error = std::format("{}: {}", dest, std::strerror(w < 0 ? errno : EIO));
            break;
        }
        p += w;
        left -= static_cast<std::size_t>(w);
    }
    if (::close(fd) != 0 && error.empty()) {
        error = std::format("{}: {}", dest, std::strerror(errno));
    }
    return error;
}

std::optional<Fail> cmd_pcap(Args& a, std::ostream& out, int& code) {
    std::string e;
    const auto device = a.number<std::uint16_t>("--device", e);
    if (!e.empty()) {
        return usage(e);
    }
    const auto csv = a.value("--csv", e);
    if (!e.empty()) {
        return usage(e);
    }
    const auto head = a.number<std::size_t>("--head", e);
    if (!e.empty()) {
        return usage(e);
    }
    const bool verbose = a.flag("--plan");
    const auto file = a.positional();
    if (!file) {
        return usage("pcap needs a capture file");
    }
    if (auto f = a.finish(); !f.empty()) {
        return usage(f);
    }
    if (csv || head) {
        return usage("--csv and --head need the gaze decoder, which is not in this tool yet");
    }
    const auto report = pibridge::eye::analysis::analyze_file(*file, device);
    if (!report) {
        return Fail{2, report.error().to_string()};
    }
    out << pibridge::eye::analysis::format_report(*report, verbose);
    code = 0;
    return std::nullopt;
}

std::optional<Fail> cmd_plan(Args& a, std::ostream& out, int& code) {
    std::string e;
    const auto device = a.number<std::uint16_t>("--device", e);
    if (!e.empty()) {
        return usage(e);
    }
    const auto file = a.positional();
    if (!file) {
        return usage("plan needs a capture file");
    }
    if (auto f = a.finish(); !f.empty()) {
        return usage(f);
    }
    const auto report = pibridge::eye::analysis::analyze_file(*file, device);
    if (!report) {
        return Fail{2, report.error().to_string()};
    }
    if (report->devices.empty()) {
        return Fail{2, "no device with TTP traffic in the capture"};
    }
    for (const auto& d : report->devices) {
        out << d.plan.describe();
        out << std::format("=> {} ({} bytes missing)\n",
                           d.plan.is_complete() ? "complete: every byte is in the capture" : "NOT replayable verbatim",
                           d.plan.missing_bytes());
    }
    code = 0;
    return std::nullopt;
}

std::optional<Fail> cmd_frames(Args& a, std::ostream& out, int& code) {
    std::string e;
    const auto device = a.number<std::uint16_t>("--device", e);
    if (!e.empty()) {
        return usage(e);
    }
    const bool keep_auth = a.flag("--keep-auth");
    const auto dest = a.value("-o", e);
    if (!e.empty()) {
        return usage(e);
    }
    if (!dest) {
        return usage("frames needs -o FILE");
    }
    const auto file = a.positional();
    if (!file) {
        return usage("frames needs a capture file");
    }
    if (auto f = a.finish(); !f.empty()) {
        return usage(f);
    }
    const auto report = pibridge::eye::analysis::analyze_file(*file, device);
    if (!report) {
        return Fail{2, report.error().to_string()};
    }
    // the device whose start reaches the gaze stream, with the most steps (a capture may hold several sessions)
    const pibridge::eye::analysis::DeviceReport* d = nullptr;
    for (const auto& r : report->devices) {
        if (r.plan.stream_start && !r.plan.requests().empty() && (d == nullptr || r.plan.steps.size() >= d->plan.steps.size())) {
            d = &r;
        }
    }
    if (d == nullptr) {
        return Fail{2, "no eye chip start that reaches the gaze stream in the capture"};
    }
    const std::string name = std::filesystem::path(*file).filename().string();
    const auto text = pibridge::eye::frames::from_plan(d->plan, name.empty() ? *file : name, keep_auth);
    if (!text) {
        return Fail{2, text.error()};
    }
    // the list must load exactly as the tracker loads it
    const auto steps = pibridge::eye::frames::parse(*text);
    if (!steps) {
        return Fail{2, "made list does not parse: " + steps.error()};
    }
    if (auto g = pibridge::eye::frames::Guard::make_with(*steps, false); !g) {
        return Fail{2, g.error()};
    }
    if (auto w = write_private(*dest, text->data(), text->size()); !w.empty()) {
        return Fail{2, w};
    }
    out << std::format("{}: {} steps from device address {} of {} (stream at t={:.3} s){}\n", *dest, steps->size(),
                       d->device, name, d->plan.stream_start.value_or(0.0),
                       keep_auth ? "" : ", auth steps left out");
    code = 0;
    return std::nullopt;
}

std::optional<Fail> cmd_profile(Args& a, std::ostream& out, int& code) {
    std::string e;
    const auto dest = a.value("-o", e);
    if (!e.empty()) {
        return usage(e);
    }
    if (!dest) {
        return usage("profile needs -o FILE");
    }
    const auto file = a.positional();
    if (!file) {
        return usage("profile needs a calibration.setpm file");
    }
    if (auto f = a.finish(); !f.empty()) {
        return usage(f);
    }
    std::ifstream in(*file, std::ios::binary);
    if (!in) {
        return Fail{2, std::format("{}: {}", *file, std::strerror(errno != 0 ? errno : ENOENT))};
    }
    const std::vector<std::uint8_t> raw((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (in.bad()) {
        return Fail{2, std::format("{}: read failed", *file)};
    }
    const auto body = pibridge::eye::frames::profile_from_setpm(raw);
    if (!body) {
        return Fail{2, std::format("{}: {}", *file, body.error())};
    }
    if (auto w = write_private(*dest, body->data(), body->size()); !w.empty()) {
        return Fail{2, w};
    }
    out << std::format("{}: eye profile of {} bytes (the eye daemon uploads it when eye_profile names it)\n", *dest,
                       body->size());
    code = 0;
    return std::nullopt;
}

} // namespace

bool Args::flag(const std::string& name) {
    for (auto it = items_.begin(); it != items_.end(); ++it) {
        if (*it == name) {
            items_.erase(it);
            return true;
        }
    }
    return false;
}

std::optional<std::string> Args::value(const std::string& name, std::string& error) {
    for (std::size_t i = 0; i < items_.size(); i++) {
        if (items_[i] == name) {
            if (i + 1 >= items_.size()) {
                error = name + " needs a value";
                return std::nullopt;
            }
            std::string v = items_[i + 1];
            items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(i), items_.begin() + static_cast<std::ptrdiff_t>(i + 2));
            return v;
        }
    }
    const std::string prefix = name + "=";
    for (std::size_t i = 0; i < items_.size(); i++) {
        if (items_[i].starts_with(prefix)) {
            std::string v = items_[i].substr(prefix.size());
            items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(i));
            return v;
        }
    }
    return std::nullopt;
}

template <class T>
std::optional<T> Args::number(const std::string& name, std::string& error) {
    const auto v = value(name, error);
    if (!v) {
        return std::nullopt;
    }
    T n{};
    const auto r = std::from_chars(v->data(), v->data() + v->size(), n);
    if (v->empty() || r.ec != std::errc{} || r.ptr != v->data() + v->size()) {
        error = std::format("{}: bad number '{}'", name, *v);
        return std::nullopt;
    }
    return n;
}

template std::optional<std::uint16_t> Args::number<std::uint16_t>(const std::string&, std::string&);
template std::optional<std::size_t> Args::number<std::size_t>(const std::string&, std::string&);
template std::optional<double> Args::number<double>(const std::string&, std::string&);

std::optional<std::string> Args::positional() {
    for (auto it = items_.begin(); it != items_.end(); ++it) {
        if (!it->starts_with("--")) {
            std::string v = *it;
            items_.erase(it);
            return v;
        }
    }
    return std::nullopt;
}

std::string Args::finish() const {
    if (items_.empty()) {
        return {};
    }
    return std::format("unexpected argument '{}'", items_.front());
}

int run(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    if (args.empty()) {
        err << USAGE;
        return 1;
    }
    const std::string& cmd = args.front();
    Args a(std::vector<std::string>(args.begin() + 1, args.end()));
    int code = 1;
    std::optional<Fail> fail;
    if (cmd == "pcap") {
        fail = cmd_pcap(a, out, code);
    } else if (cmd == "plan") {
        fail = cmd_plan(a, out, code);
    } else if (cmd == "frames") {
        fail = cmd_frames(a, out, code);
    } else if (cmd == "profile") {
        fail = cmd_profile(a, out, code);
    } else if (cmd == "help" || cmd == "--help" || cmd == "-h") {
        out << USAGE;
        return 0;
    } else if (cmd == "probe" || cmd == "stream" || cmd == "replay" || cmd == "live" || cmd == "watch" ||
               cmd == "power-on" || cmd == "ipd") {
        fail = usage(std::format("'{}' talks to the eye chip or the headset and is not part of this tool", cmd));
    } else {
        fail = usage(std::format("unknown command '{}'", cmd));
    }
    if (!fail) {
        return code;
    }
    err << "pibridge-eye: " << fail->msg << "\n";
    if (fail->code == 1) {
        err << "\n" << USAGE;
    }
    return fail->code;
}

} // namespace eyetool
