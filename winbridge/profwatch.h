// SPDX-License-Identifier: MIT
// profwatch: where the Pimax engines' threads spend their time (PIBRIDGE_WB_PROFILE=1).
//
// A sampling profiler inside winbridge.exe: every few ms (PIBRIDGE_WB_PROFILE_MS, default 5) the engine threads whose
// description matches PIBRIDGE_WB_PROFILE_THREADS (default "LEDObject,SLAMManager", "*" = every named thread) are
// suspended for a moment and their instruction pointer read (GetThreadContext); the address is resolved to the start of
// its function through the module's unwind table (RtlLookupFunctionEntry) and counted. Once a minute
// (PIBRIDGE_WB_PROFILE_REPORT_MS) the thread's top functions go to stdout (winbridge.log) as module+offset with their
// share of the minute's samples, and the functions whose share grew most since the previous minute: that is where a
// thread whose CPU grows over a session (the LED engine's, measured 180 -> 550 ms/s over 15 min of continuous tracking)
// spends the extra time. Diagnostics only: ~1 % overhead on the sampled threads.
#pragma once

#include <windows.h>
#include <tlhelp32.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace profwatch {

struct Thread {
    DWORD tid = 0;
    HANDLE h = nullptr;
    std::string name;
    std::unordered_map<std::uint64_t, long> hist;  // function start -> samples this minute
    long samples = 0;
    long waiting = 0;                              // samples inside ntdll / kernelbase / the Unix side (a wait)
    std::unordered_map<std::uint64_t, double> prev_share;
    long prev_samples = 0;
};

inline std::vector<Thread> threads;
inline DWORD period_ms = 5;
inline DWORD report_ms = 60000;
inline std::string filter = "LEDObject,SLAMManager";
inline DWORD self_tid = 0;

typedef HRESULT(WINAPI* GetThreadDescriptionFn)(HANDLE, PWSTR*);
inline GetThreadDescriptionFn p_GetThreadDescription = nullptr;

inline std::string thread_name(HANDLE h) {
    if (!p_GetThreadDescription) return {};
    PWSTR w = nullptr;
    if (FAILED(p_GetThreadDescription(h, &w)) || !w) return {};
    std::string s;
    for (int i = 0; w[i] && i < 64; ++i) s.push_back(w[i] < 128 ? static_cast<char>(w[i]) : '?');
    LocalFree(w);
    return s;
}

inline bool wanted(const std::string& name) {
    if (name.empty()) return false;
    if (filter == "*") return true;
    std::size_t pos = 0;
    while (pos <= filter.size()) {
        const std::size_t e = filter.find(',', pos);
        const std::string part = filter.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
        if (!part.empty() && name.find(part) != std::string::npos) return true;
        if (e == std::string::npos) break;
        pos = e + 1;
    }
    return false;
}

/// Adds the process's threads that match the filter and are not yet watched.
inline void refresh_threads() {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te{};
    te.dwSize = sizeof(te);
    const DWORD pid = GetCurrentProcessId();
    for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self_tid) continue;
        bool known = false;
        for (const auto& t : threads)
            if (t.tid == te.th32ThreadID) { known = true; break; }
        if (known) continue;
        HANDLE h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE,
                              te.th32ThreadID);
        if (!h) continue;
        const std::string name = thread_name(h);
        if (!wanted(name)) { CloseHandle(h); continue; }
        Thread t;
        t.tid = te.th32ThreadID;
        t.h = h;
        t.name = name;
        threads.push_back(std::move(t));
        printf("profwatch: watching thread %lu \"%s\"\n", static_cast<unsigned long>(te.th32ThreadID), name.c_str());
    }
    CloseHandle(snap);
}

/// The start of the function holding `rip` (unwind table), else rip rounded to 64 bytes.
inline std::uint64_t resolve(std::uint64_t rip) {
    DWORD64 base = 0;
    PRUNTIME_FUNCTION f = RtlLookupFunctionEntry(rip, &base, nullptr);
    if (f && base) return base + f->BeginAddress;
    return rip & ~static_cast<std::uint64_t>(63);
}

inline std::string module_of(std::uint64_t addr, std::uint64_t* base_out) {
    HMODULE hm = nullptr;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCSTR>(addr), &hm) && hm) {
        char path[MAX_PATH] = {};
        GetModuleFileNameA(hm, path, sizeof(path));
        const char* b = strrchr(path, '\\');
        *base_out = reinterpret_cast<std::uint64_t>(hm);
        return b ? b + 1 : path;
    }
    *base_out = 0;
    return "?";
}

inline std::string sym(std::uint64_t addr) {
    std::uint64_t base = 0;
    const std::string m = module_of(addr, &base);
    char buf[300];
    if (base) snprintf(buf, sizeof(buf), "%s+0x%llx", m.c_str(), static_cast<unsigned long long>(addr - base));
    else snprintf(buf, sizeof(buf), "0x%llx", static_cast<unsigned long long>(addr));
    return buf;
}

inline bool is_wait_module(const std::string& m) {
    std::string l = m;
    std::transform(l.begin(), l.end(), l.begin(), [](unsigned char c) { return static_cast<char>(tolower(c)); });
    return l == "ntdll.dll" || l == "kernelbase.dll" || l == "kernel32.dll" || l == "win32u.dll" || l == "?";
}

inline void sample_all() {
    for (auto& t : threads) {
        if (SuspendThread(t.h) == static_cast<DWORD>(-1)) continue;
        CONTEXT ctx{};
        ctx.ContextFlags = CONTEXT_CONTROL;
        const BOOL ok = GetThreadContext(t.h, &ctx);
        ResumeThread(t.h);
        if (!ok) continue;
        const std::uint64_t key = resolve(ctx.Rip);
        t.hist[key]++;
        t.samples++;
    }
}

inline void report() {
    for (auto& t : threads) {
        if (t.samples == 0) continue;
        std::vector<std::pair<std::uint64_t, long>> v(t.hist.begin(), t.hist.end());
        std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
        long waiting = 0;
        std::unordered_map<std::uint64_t, double> share;
        for (const auto& [k, n] : v) {
            std::uint64_t base = 0;
            if (is_wait_module(module_of(k, &base))) waiting += n;
            share[k] = 100.0 * n / t.samples;
        }
        printf("profwatch %s: %ld samples, %.0f%% in Windows waits; top:", t.name.c_str(), t.samples,
               100.0 * waiting / t.samples);
        int shown = 0;
        for (const auto& [k, n] : v) {
            std::uint64_t base = 0;
            if (is_wait_module(module_of(k, &base))) continue;
            const double s = share[k];
            const auto p = t.prev_share.find(k);
            printf(" %s %.1f%%(%+.1f)", sym(k).c_str(), s, s - (p == t.prev_share.end() ? 0.0 : p->second));
            if (++shown >= 14) break;
        }
        printf("\n");
        if (t.prev_samples > 0) {
            // the functions whose share grew most since the previous minute
            std::vector<std::pair<std::uint64_t, double>> g;
            for (const auto& [k, s] : share) {
                const auto p = t.prev_share.find(k);
                const double d = s - (p == t.prev_share.end() ? 0.0 : p->second);
                if (d > 0.3) g.emplace_back(k, d);
            }
            std::sort(g.begin(), g.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
            if (!g.empty()) {
                printf("profwatch %s grew:", t.name.c_str());
                int k = 0;
                for (const auto& [addr, d] : g) {
                    printf(" %s %+.1f%% (now %.1f%%)", sym(addr).c_str(), d, share[addr]);
                    if (++k >= 8) break;
                }
                printf("\n");
            }
        }
        t.prev_share = std::move(share);
        t.prev_samples = t.samples;
        t.hist.clear();
        t.samples = 0;
    }
}

inline DWORD WINAPI loop(LPVOID) {
    self_tid = GetCurrentThreadId();
    Sleep(2000);  // the engines' threads exist by then
    refresh_threads();
    DWORD last = GetTickCount();
    DWORD last_refresh = last;
    while (true) {
        sample_all();
        Sleep(period_ms);
        const DWORD now = GetTickCount();
        if (now - last_refresh >= 10000) { refresh_threads(); last_refresh = now; }
        if (now - last >= report_ms) { report(); last = now; }
    }
    return 0;
}

inline void start() {
    HMODULE k = GetModuleHandleA("kernelbase.dll");
    if (k) p_GetThreadDescription = reinterpret_cast<GetThreadDescriptionFn>(GetProcAddress(k, "GetThreadDescription"));
    if (!p_GetThreadDescription) {
        k = GetModuleHandleA("kernel32.dll");
        if (k) p_GetThreadDescription =
                   reinterpret_cast<GetThreadDescriptionFn>(GetProcAddress(k, "GetThreadDescription"));
    }
    if (const char* e = getenv("PIBRIDGE_WB_PROFILE_MS"); e && atoi(e) > 0) period_ms = static_cast<DWORD>(atoi(e));
    if (const char* e = getenv("PIBRIDGE_WB_PROFILE_REPORT_MS"); e && atoi(e) > 0)
        report_ms = static_cast<DWORD>(atoi(e));
    if (const char* e = getenv("PIBRIDGE_WB_PROFILE_THREADS"); e && *e) filter = e;
    printf("profwatch: sampling threads matching \"%s\" every %lu ms, a report every %lu s%s\n", filter.c_str(),
           static_cast<unsigned long>(period_ms), static_cast<unsigned long>(report_ms / 1000),
           p_GetThreadDescription ? "" : " (no GetThreadDescription: threads cannot be named)");
    CreateThread(nullptr, 0, loop, nullptr, 0, nullptr);
}

} // namespace profwatch
