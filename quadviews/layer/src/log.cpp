// SPDX-License-Identifier: MIT
#include "log.h"

#include <cstdarg>
#include <cstring>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <mutex>

#include "settings.h"

namespace qv {

namespace {
std::mutex g_mutex;
FILE* g_file = nullptr;
bool g_stderr = false;
} // namespace

void logOpen() {
    std::lock_guard lock(g_mutex);
    if (g_file) {
        return; // one log per process: a game may create several instances
    }
    const char* e = std::getenv("QV_VULKAN_LOG_STDERR");
    g_stderr = e && *e == '1';
    const auto dir = stateDir();
    if (dir.empty()) {
        return;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    // The loader unloads the layer with each instance, so "once per process" is told by the pid on the first line:
    // the same process appends, another one starts the log anew.
    const auto path = dir / "qv-vulkan.log";
    char want[64];
    std::snprintf(want, sizeof(want), "qv-vulkan log of process %d\n", int(getpid()));
    char first[64] = {};
    if (FILE* f = std::fopen(path.c_str(), "r")) {
        if (!std::fgets(first, sizeof(first), f)) first[0] = 0;
        std::fclose(f);
    }
    const bool same = std::strcmp(first, want) == 0;
    g_file = std::fopen(path.c_str(), same ? "a" : "w");
    if (g_file && !same) {
        std::fputs(want, g_file);
    }
}

void logf(const char* fmt, ...) {
    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    tm t{};
    localtime_r(&ts.tv_sec, &t);
    char stamp[32];
    std::snprintf(stamp, sizeof(stamp), "%02d:%02d:%02d.%03ld", t.tm_hour, t.tm_min, t.tm_sec, ts.tv_nsec / 1000000);

    std::lock_guard lock(g_mutex);
    if (g_file) {
        std::fprintf(g_file, "%s %s\n", stamp, msg);
        std::fflush(g_file);
    }
    if (g_stderr) {
        std::fprintf(stderr, "[qv-vulkan] %s\n", msg);
    }
}

} // namespace qv
