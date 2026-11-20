// SPDX-License-Identifier: MIT
#include "sysmon.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string_view>
#include <vector>

namespace pibridge::headtrack {

namespace fs = std::filesystem;

namespace {

std::int64_t mono_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

std::string slurp(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

std::string first_line(const fs::path& p) {
    std::ifstream f(p);
    std::string l;
    std::getline(f, l);
    return l;
}

/// The short name of a process we watch, from its command line; empty for the others.
std::string watched(const std::string& cmdline) {
    static constexpr std::pair<std::string_view, std::string_view> names[] = {
        {"winbridge", "winbridge"},       {"pibridge-ctrltrack", "ctrltrack"}, {"pibridge-headtrack", "headtrack"},
        {"pibridge-cam", "broker"},        {"vrcompositor", "vrcompositor"},   {"vrserver", "vrserver"},           {"wineserver", "wineserver"},
    };
    // only the program (the first argument), so a wrapper naming another program is not taken for it
    const std::string prog = cmdline.substr(0, cmdline.find('\0'));
    for (const auto& [needle, name] : names) {
        if (prog.find(needle) != std::string::npos) {
            return std::string(name);
        }
    }
    return {};
}

struct Sched {
    std::uint64_t cpu = 0, wait = 0, slices = 0;
};

} // namespace

bool SysMon::start(const std::string& path) {
    f_ = std::fopen(path.c_str(), "w");
    if (f_ == nullptr) {
        return false;
    }
    std::fputs("t_ns,kind,proc,pid,tid,thread,cpu_ms,wait_ms,slices,value\n", f_);
    run_ = true;
    th_ = std::thread([this] { loop(); });
    return true;
}

void SysMon::stop() {
    if (run_.exchange(false) && th_.joinable()) {
        th_.join();
    }
    if (f_ != nullptr) {
        std::fclose(f_);
        f_ = nullptr;
    }
}

void SysMon::loop() {
    std::map<std::pair<int, int>, Sched> last;
    std::map<int, std::string> procs; // pid -> short name
    std::int64_t scan_at = 0;
    // the machine's sensors, found once: k10temp/coretemp (CPU), amdgpu (GPU) temperatures; GPU busy
    std::vector<std::pair<std::string, fs::path>> temps;
    std::vector<fs::path> gpu_busy;
    std::error_code ec;
    for (const auto& h : fs::directory_iterator("/sys/class/hwmon", ec)) {
        const std::string n = first_line(h.path() / "name");
        if (n != "k10temp" && n != "coretemp" && n != "amdgpu" && n != "zenpower") {
            continue;
        }
        for (const auto& e : fs::directory_iterator(h.path(), ec)) {
            const std::string f = e.path().filename().string();
            if (f.starts_with("temp") && f.ends_with("_input")) {
                std::string label = first_line(h.path() / (f.substr(0, f.size() - 6) + "_label"));
                temps.emplace_back(n + ":" + (label.empty() ? f : label), e.path());
            }
        }
    }
    for (const auto& c : fs::directory_iterator("/sys/class/drm", ec)) {
        const fs::path b = c.path() / "device/gpu_busy_percent";
        if (c.path().filename().string().find('-') == std::string::npos && fs::exists(b, ec)) {
            gpu_busy.push_back(b);
        }
    }
    auto next = std::chrono::steady_clock::now();
    while (run_) {
        next += std::chrono::seconds(1);
        std::this_thread::sleep_until(next);
        if (!run_) {
            break;
        }
        const std::int64_t now = mono_ns();
        if (now >= scan_at) {
            scan_at = now + 5'000'000'000;
            procs.clear();
            for (const auto& e : fs::directory_iterator("/proc", ec)) {
                const std::string d = e.path().filename().string();
                if (d.empty() || d.find_first_not_of("0123456789") != std::string::npos) {
                    continue;
                }
                const std::string n = watched(slurp(e.path() / "cmdline"));
                if (!n.empty()) {
                    procs[std::stoi(d)] = n;
                }
            }
        }
        std::map<std::pair<int, int>, Sched> cur;
        for (const auto& [pid, name] : procs) {
            const fs::path task = fs::path("/proc") / std::to_string(pid) / "task";
            for (const auto& t : fs::directory_iterator(task, ec)) {
                const int tid = std::atoi(t.path().filename().c_str());
                Sched s;
                std::istringstream in(first_line(t.path() / "schedstat"));
                if (!(in >> s.cpu >> s.wait >> s.slices)) {
                    continue;
                }
                cur[{pid, tid}] = s;
                const auto it = last.find({pid, tid});
                if (it == last.end()) {
                    continue;
                }
                const double cpu = static_cast<double>(s.cpu - it->second.cpu) * 1e-6;
                const double wait = static_cast<double>(s.wait - it->second.wait) * 1e-6;
                if (cpu < 5.0 && wait < 2.0) {
                    continue; // idle threads: no row
                }
                std::string comm = first_line(t.path() / "comm");
                for (char& c : comm) {
                    if (c == ',' || c == '"') {
                        c = '_';
                    }
                }
                std::fprintf(f_, "%lld,thread,%s,%d,%d,%s,%.2f,%.2f,%llu,\n", static_cast<long long>(now), name.c_str(),
                             pid, tid, comm.c_str(), cpu, wait,
                             static_cast<unsigned long long>(s.slices - it->second.slices));
                rows_++;
            }
        }
        last = std::move(cur);
        // resident memory per process (a leak shows as a steady climb)
        for (const auto& [pid, name] : procs) {
            std::ifstream st(fs::path("/proc") / std::to_string(pid) / "status");
            std::string line;
            while (std::getline(st, line)) {
                if (line.rfind("VmRSS:", 0) == 0) {
                    std::fprintf(f_, "%lld,rss,%s,%d,,,,,,%.1f\n", static_cast<long long>(now), name.c_str(), pid,
                                 std::atof(line.c_str() + 6) / 1024.0);
                    rows_++;
                    break;
                }
            }
        }
        const auto put = [&](const char* kind, const std::string& name, double v) {
            std::fprintf(f_, "%lld,%s,%s,,,,,,,%.2f\n", static_cast<long long>(now), kind, name.c_str(), v);
            rows_++;
        };
        {
            std::istringstream in(first_line("/proc/loadavg"));
            double l1 = 0;
            if (in >> l1) {
                put("load", "1min", l1);
            }
        }
        {
            const std::string l = first_line("/proc/pressure/cpu");
            if (const auto p = l.find("avg10="); p != std::string::npos) {
                put("psi_cpu", "some_avg10", std::atof(l.c_str() + p + 6));
            }
        }
        for (const auto& [name, p] : temps) {
            const std::string v = first_line(p);
            if (!v.empty()) {
                put("temp", name, std::atof(v.c_str()) / 1000.0);
            }
        }
        double fsum = 0, fmax = 0;
        int fn = 0;
        for (int c = 0;; c++) {
            const std::string v =
                first_line("/sys/devices/system/cpu/cpu" + std::to_string(c) + "/cpufreq/scaling_cur_freq");
            if (v.empty()) {
                break;
            }
            const double mhz = std::atof(v.c_str()) / 1000.0;
            fsum += mhz;
            fmax = std::max(fmax, mhz);
            fn++;
        }
        if (fn > 0) {
            put("freq_mean", "cpu_mhz", fsum / fn);
            put("freq_max", "cpu_mhz", fmax);
        }
        for (std::size_t i = 0; i < gpu_busy.size(); i++) {
            const std::string v = first_line(gpu_busy[i]);
            if (!v.empty()) {
                put("gpu_busy", "gpu" + std::to_string(i), std::atof(v.c_str()));
            }
        }
        std::fflush(f_);
    }
}

} // namespace pibridge::headtrack
