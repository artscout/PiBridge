// SPDX-License-Identifier: MIT
// allocwatch: which code in the Pimax DLLs keeps heap memory (PIBRIDGE_WB_ALLOC=1).
//
// The import tables of every loaded module except winbridge itself and Windows/Wine's own get their heap functions
// (malloc, calloc, realloc, free, _aligned_malloc, _aligned_realloc, _aligned_free from the UCRT / msvcr*) pointed at
// wrappers that record each live block with a short call stack (RtlCaptureStackBackTrace). Once a minute the live bytes
// per call site go to stdout (winbridge.log): the sites holding the most, and the ones that grew most since the last
// report. A block freed that was allocated before the hooks went in is ignored. Diagnostics only: it slows every
// allocation of those DLLs a little.
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

namespace allocwatch {

constexpr int kFrames = 10;

struct Site {
    void* f[kFrames] = {};
    int n = 0;
    long long live = 0;       // bytes now
    long long count = 0;      // blocks now
    long long allocs = 0;     // allocations ever
    long long last_live = 0;  // live at the last report
};

struct Block {
    size_t size;
    uint64_t site;
};

using MallocFn = void* (__cdecl*)(size_t);
using CallocFn = void* (__cdecl*)(size_t, size_t);
using ReallocFn = void* (__cdecl*)(void*, size_t);
using FreeFn = void(__cdecl*)(void*);
using AMallocFn = void* (__cdecl*)(size_t, size_t);
using AReallocFn = void* (__cdecl*)(void*, size_t, size_t);

inline MallocFn o_malloc;
inline CallocFn o_calloc;
inline ReallocFn o_realloc;
inline FreeFn o_free;
inline AMallocFn o_amalloc;
inline AReallocFn o_arealloc;
inline FreeFn o_afree;

inline SRWLOCK lock = SRWLOCK_INIT;
inline std::unordered_map<void*, Block>* blocks;
inline std::unordered_map<uint64_t, Site>* sites;
inline long long total_live = 0;
/// Every `every`-th allocation is tracked (the stack walk under Wine is slow); the report scales by it.
inline unsigned every = 16;
inline volatile LONG counter = 0;

inline uint64_t site_of(int skip) {
    void* f[kFrames + 1] = {};
    const USHORT n = RtlCaptureStackBackTrace(static_cast<DWORD>(skip), kFrames, f, nullptr);
    uint64_t h = 1469598103934665603ull;
    for (USHORT i = 0; i < n; i++) {
        h = (h ^ reinterpret_cast<uint64_t>(f[i])) * 1099511628211ull;
    }
    auto it = sites->find(h);
    if (it == sites->end()) {
        Site s;
        s.n = n;
        std::memcpy(s.f, f, sizeof(void*) * n);
        it = sites->emplace(h, s).first;
    }
    return h;
}

inline void add(void* p, size_t size) {
    if (p == nullptr || (static_cast<unsigned>(InterlockedIncrement(&counter)) % every) != 0) {
        return;
    }
    AcquireSRWLockExclusive(&lock);
    const uint64_t s = site_of(1);
    (*blocks)[p] = Block{size, s};
    Site& st = (*sites)[s];
    st.live += static_cast<long long>(size);
    st.count++;
    st.allocs++;
    total_live += static_cast<long long>(size);
    ReleaseSRWLockExclusive(&lock);
}

inline void remove(void* p) {
    if (p == nullptr) {
        return;
    }
    AcquireSRWLockExclusive(&lock);
    auto it = blocks->find(p);
    if (it != blocks->end()) {
        Site& st = (*sites)[it->second.site];
        st.live -= static_cast<long long>(it->second.size);
        st.count--;
        total_live -= static_cast<long long>(it->second.size);
        blocks->erase(it);
    }
    ReleaseSRWLockExclusive(&lock);
}

inline void* __cdecl w_malloc(size_t n) {
    void* p = o_malloc(n);
    add(p, n);
    return p;
}
inline void* __cdecl w_calloc(size_t a, size_t b) {
    void* p = o_calloc(a, b);
    add(p, a * b);
    return p;
}
inline void* __cdecl w_realloc(void* q, size_t n) {
    remove(q);
    void* p = o_realloc(q, n);
    add(p, n);
    return p;
}
inline void __cdecl w_free(void* p) {
    remove(p);
    o_free(p);
}
inline void* __cdecl w_amalloc(size_t n, size_t al) {
    void* p = o_amalloc(n, al);
    add(p, n);
    return p;
}
inline void* __cdecl w_arealloc(void* q, size_t n, size_t al) {
    remove(q);
    void* p = o_arealloc(q, n, al);
    add(p, n);
    return p;
}
inline void __cdecl w_afree(void* p) {
    remove(p);
    o_afree(p);
}

inline bool is_heap_dll(const char* name) {
    char lower[128];
    size_t i = 0;
    for (; name[i] && i < sizeof(lower) - 1; i++) {
        lower[i] = static_cast<char>(tolower(static_cast<unsigned char>(name[i])));
    }
    lower[i] = 0;
    // the UCRT only (one heap): an older msvcr* has its own, and one set of originals cannot serve both
    return strstr(lower, "api-ms-win-crt-heap") || strstr(lower, "ucrtbase");
}

// Point one module's heap imports at the wrappers; returns how many entries were patched.
inline int patch_module(HMODULE mod) {
    auto* base = reinterpret_cast<uint8_t*>(mod);
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE) {
        return 0;
    }
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (dir.VirtualAddress == 0) {
        return 0;
    }
    int patched = 0;
    for (auto* imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress); imp->Name; imp++) {
        const char* dll = reinterpret_cast<const char*>(base + imp->Name);
        if (!is_heap_dll(dll) || imp->OriginalFirstThunk == 0) {
            continue;
        }
        auto* names = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->OriginalFirstThunk);
        auto* addrs = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->FirstThunk);
        for (; names->u1.AddressOfData; names++, addrs++) {
            if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal)) {
                continue;
            }
            const char* fn = reinterpret_cast<const char*>(
                reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData)->Name);
            void* repl = nullptr;
            void** orig = nullptr;
            if (!strcmp(fn, "malloc")) {
                repl = reinterpret_cast<void*>(&w_malloc), orig = reinterpret_cast<void**>(&o_malloc);
            } else if (!strcmp(fn, "calloc")) {
                repl = reinterpret_cast<void*>(&w_calloc), orig = reinterpret_cast<void**>(&o_calloc);
            } else if (!strcmp(fn, "realloc")) {
                repl = reinterpret_cast<void*>(&w_realloc), orig = reinterpret_cast<void**>(&o_realloc);
            } else if (!strcmp(fn, "free")) {
                repl = reinterpret_cast<void*>(&w_free), orig = reinterpret_cast<void**>(&o_free);
            } else if (!strcmp(fn, "_aligned_malloc")) {
                repl = reinterpret_cast<void*>(&w_amalloc), orig = reinterpret_cast<void**>(&o_amalloc);
            } else if (!strcmp(fn, "_aligned_realloc")) {
                repl = reinterpret_cast<void*>(&w_arealloc), orig = reinterpret_cast<void**>(&o_arealloc);
            } else if (!strcmp(fn, "_aligned_free")) {
                repl = reinterpret_cast<void*>(&w_afree), orig = reinterpret_cast<void**>(&o_afree);
            }
            if (!repl) {
                continue;
            }
            void* cur = reinterpret_cast<void*>(addrs->u1.Function);
            if (cur == repl) {
                continue;
            }
            if (*orig == nullptr) {
                *orig = cur;
            }
            DWORD old;
            if (VirtualProtect(&addrs->u1.Function, sizeof(void*), PAGE_READWRITE, &old)) {
                addrs->u1.Function = reinterpret_cast<ULONG_PTR>(repl);
                VirtualProtect(&addrs->u1.Function, sizeof(void*), old, &old);
                patched++;
            }
        }
    }
    return patched;
}

inline std::string where(void* a) {
    HMODULE m = nullptr;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            static_cast<LPCSTR>(a), &m) ||
        !m) {
        char b[32];
        snprintf(b, sizeof b, "%p", a);
        return b;
    }
    char path[MAX_PATH] = {};
    GetModuleFileNameA(m, path, MAX_PATH);
    const char* name = strrchr(path, '\\');
    name = name ? name + 1 : path;
    char b[MAX_PATH + 32];
    snprintf(b, sizeof b, "%s+0x%llx", name,
             static_cast<unsigned long long>(reinterpret_cast<uint8_t*>(a) - reinterpret_cast<uint8_t*>(m)));
    return b;
}

// Patch every loaded module except this exe and the system's (Windows/Wine dlls); returns the patched entries.
inline int install() {
    blocks = new std::unordered_map<void*, Block>();
    sites = new std::unordered_map<uint64_t, Site>();
    blocks->reserve(1 << 20);
    HMODULE self = GetModuleHandleA(nullptr);
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
    int total = 0;
    if (snap == INVALID_HANDLE_VALUE) {
        return 0;
    }
    MODULEENTRY32 me;
    me.dwSize = sizeof(me);
    for (BOOL ok = Module32First(snap, &me); ok; ok = Module32Next(snap, &me)) {
        if (me.hModule == self) {
            continue;
        }
        char lower[MAX_PATH];
        size_t i = 0;
        for (; me.szExePath[i] && i < sizeof(lower) - 1; i++) {
            lower[i] = static_cast<char>(tolower(static_cast<unsigned char>(me.szExePath[i])));
        }
        lower[i] = 0;
        if (strstr(lower, "\\windows\\")) {
            continue; // the system's (Wine's builtins live there): only the Pimax kit and its libraries
        }
        const int n = patch_module(me.hModule);
        if (n > 0) {
            printf("allocwatch: %s: %d heap imports hooked\n", me.szModule, n);
        }
        total += n;
    }
    CloseHandle(snap);
    fflush(stdout);
    return total;
}

// A site's frames past winbridge's own (the wrappers), at most six.
inline std::string stack(const Site& s) {
    std::string out;
    int shown = 0;
    for (int f = 0; f < s.n && shown < 6; f++) {
        const std::string w = where(s.f[f]);
        if (w.rfind("winbridge.exe", 0) == 0) {
            continue;
        }
        out += (shown ? " < " : "") + w;
        shown++;
    }
    return out;
}

// The report: the sites that grew most since the last one and those holding the most.
inline void report() {
    struct Row {
        uint64_t key;
        Site s;
    };
    std::vector<Row> rows;
    long long live;
    size_t nblocks;
    AcquireSRWLockExclusive(&lock);
    rows.reserve(sites->size());
    for (auto& [k, s] : *sites) {
        rows.push_back({k, s});
        s.last_live = s.live;
    }
    live = total_live;
    nblocks = blocks->size();
    ReleaseSRWLockExclusive(&lock);
    printf("allocwatch: ~%lld bytes live (1 in %u allocations tracked, %zu blocks, scaled), %zu call sites\n",
           live * every, every, nblocks, rows.size());
    std::sort(rows.begin(), rows.end(),
              [](const Row& a, const Row& b) { return a.s.live - a.s.last_live > b.s.live - b.s.last_live; });
    for (size_t i = 0; i < rows.size() && i < 12; i++) {
        const Site& s = rows[i].s;
        if (s.live - s.last_live <= 0) {
            break;
        }
        const std::string st = stack(s);
        printf("allocwatch grew ~%+lld B (live ~%lld B in ~%lld blocks, ~%lld allocs ever): %s\n",
               (s.live - s.last_live) * every, s.live * every, s.count * every, s.allocs * every, st.c_str());
    }
    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.s.live > b.s.live; });
    for (size_t i = 0; i < rows.size() && i < 6; i++) {
        const Site& s = rows[i].s;
        const std::string st = stack(s);
        printf("allocwatch holds ~%lld B in ~%lld blocks: %s\n", s.live * every, s.count * every, st.c_str());
    }
    fflush(stdout);
}

inline void start_reporter(DWORD period_ms) {
    CreateThread(
        nullptr, 0,
        [](LPVOID p) -> DWORD {
            const DWORD ms = static_cast<DWORD>(reinterpret_cast<uintptr_t>(p));
            for (;;) {
                Sleep(ms);
                report();
            }
        },
        reinterpret_cast<LPVOID>(static_cast<uintptr_t>(period_ms)), 0, nullptr);
}

} // namespace allocwatch
