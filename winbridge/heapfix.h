// SPDX-License-Identifier: MIT
// heapfix: a size-class caching allocator in front of Wine's heap for the Pimax DLLs (default on; PIBRIDGE_WB_HEAP=0 off).
//
// Why. Live on 2026-11-20 (profwatch, 15 min of continuous controller tracking) the LED engine's thread spent a share
// growing from 1.8 % to 37 % of its samples in Wine's ntdll heap_allocate_block: the first-fit walk over the free lists
// of the standard heap. Wine 10's heap moves a size bin to its Low Fragmentation Heap only after the bin's live count
// passes 4 MB / size for blocks of 1 KB and more (bin_try_enable), which a per-frame allocate-then-free pattern never
// reaches, and never for blocks over 32 KB; those sizes are the engines' per-frame temporaries (OpenCV Mats, Eigen
// / Ceres matrices, 640x480 images). As the SLAM map's long-lived keyframes interleave with them, the free lists
// fragment and every malloc walks a longer chain, so the engines get slower by the minute (the "Pimax slowdown";
// memory flat, so it is not a leak). Pimax Play on Windows does not see this: Windows' LFH takes those sizes.
//
// What. The import tables of the kit's modules (every loaded module except winbridge and the system's, as allocwatch
// does) get malloc / calloc / realloc / free / _aligned_malloc / _aligned_realloc / _aligned_free pointed at an
// allocator that rounds a request up to a size class (16-byte steps to 1 KB, then 8 classes per power of two: at most
// 12.5 % slack) and keeps freed blocks of each class on its own stack: a block is reused for the next request of its
// class without touching Wine's heap. The cache is capped (PIBRIDGE_WB_HEAP_CACHE_MB, default 512): beyond it a
// freed block really goes back. Blocks the DLLs took before the hooks went in carry no header of ours and are passed
// on to the real free / realloc. A one-line report every minute (PIBRIDGE_WB_HEAP_REPORT_MS, 0 = none).
#pragma once

#include <windows.h>
#include <tlhelp32.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace heapfix {

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

constexpr uint32_t kMagic = 0x42486950u;        // "PiHB": a plain block of ours
constexpr uint32_t kMagicAligned = 0x41486950u; // "PiHA": an _aligned_malloc block (header before the aligned pointer)
constexpr size_t kHdr = 16;

/// Before every user pointer of ours: 16 bytes, so the user pointer keeps malloc's 16-byte alignment.
struct Hdr {
    uint32_t magic;
    uint32_t cls;  // size class (plain) / unused (aligned)
    uint64_t size; // the request (plain) / offset from the base block's user pointer (aligned)
};

// --- size classes -------------------------------------------------------------------------------------------------
constexpr size_t kSmallMax = 1024;      // 16-byte classes up to here
constexpr unsigned kSmallClasses = 64;  // 16 .. 1024
constexpr unsigned kStepsPerPow2 = 8;   // above: 8 classes per power of two
constexpr unsigned kMaxPow2 = 36;       // classes to 2^36 (64 GB): beyond, pass through
constexpr unsigned kClasses = kSmallClasses + (kMaxPow2 - 10) * kStepsPerPow2 + 1;

inline unsigned class_of(size_t n) {
    if (n <= 16) return 0;
    if (n <= kSmallMax) return static_cast<unsigned>((n + 15) / 16 - 1);
    // n in (2^p, 2^(p+1)]: 8 steps of 2^(p-3)
    const unsigned p = 63u - static_cast<unsigned>(__builtin_clzll(static_cast<unsigned long long>(n - 1))); // 2^p < n
    const size_t step = static_cast<size_t>(1) << (p - 3);
    const unsigned sub = static_cast<unsigned>((n - 1 - (static_cast<size_t>(1) << p)) / step); // 0..7
    return kSmallClasses + (p - 10) * kStepsPerPow2 + sub;
}

inline size_t class_size(unsigned c) {
    if (c < kSmallClasses) return static_cast<size_t>(c + 1) * 16;
    const unsigned k = c - kSmallClasses;
    const unsigned p = 10 + k / kStepsPerPow2;
    const unsigned sub = k % kStepsPerPow2;
    return (static_cast<size_t>(1) << p) + static_cast<size_t>(sub + 1) * (static_cast<size_t>(1) << (p - 3));
}

// --- the cache ----------------------------------------------------------------------------------------------------
struct FreeNode {
    FreeNode* next;
};

struct Class {
    volatile LONG lock = 0;
    FreeNode* head = nullptr;
    LONG64 cached = 0;    // blocks on the stack
    LONG64 hits = 0;      // requests served from the stack
    LONG64 misses = 0;    // requests that went to Wine's heap
};

inline Class classes[kClasses];
inline volatile LONG64 cached_bytes = 0;
inline LONG64 cache_cap = static_cast<LONG64>(512) << 20;
inline volatile LONG64 released = 0;     // frees passed on to Wine's heap for the cap
inline volatile LONG64 foreign_frees = 0; // frees of blocks allocated before the hooks
inline volatile LONG64 passthrough = 0;   // requests too large / too aligned for the cache
inline bool enabled = false;

inline void lock(Class& c) {
    unsigned spins = 0;
    while (InterlockedExchange(&c.lock, 1) != 0) {
        if (++spins < 64) YieldProcessor();
        else { spins = 0; SwitchToThread(); }
    }
}
inline void unlock(Class& c) { InterlockedExchange(&c.lock, 0); }

inline Hdr* hdr_of(void* p) { return reinterpret_cast<Hdr*>(static_cast<uint8_t*>(p) - kHdr); }

/// A block of class c with n bytes requested: from the stack, else from Wine's heap.
inline void* take(unsigned c, size_t n) {
    Class& k = classes[c];
    lock(k);
    FreeNode* node = k.head;
    if (node) {
        k.head = node->next;
        k.cached--;
        k.hits++;
    } else {
        k.misses++;
    }
    unlock(k);
    const size_t sz = class_size(c);
    if (node) {
        InterlockedExchangeAdd64(&cached_bytes, -static_cast<LONG64>(sz + kHdr));
    } else {
        void* raw = o_malloc(sz + kHdr);
        if (!raw) return nullptr;
        node = reinterpret_cast<FreeNode*>(static_cast<uint8_t*>(raw) + kHdr);
    }
    Hdr* h = hdr_of(node);
    h->magic = kMagic;
    h->cls = c;
    h->size = n;
    return node;
}

/// A plain block of ours back to its class's stack (or to Wine's heap when the cache is at its cap).
inline void give(void* p, Hdr* h) {
    const unsigned c = h->cls;
    const size_t sz = class_size(c);
    h->magic = 0; // no longer a live block of ours
    if (cached_bytes + static_cast<LONG64>(sz + kHdr) > cache_cap) {
        InterlockedIncrement64(&released);
        o_free(h);
        return;
    }
    InterlockedExchangeAdd64(&cached_bytes, static_cast<LONG64>(sz + kHdr));
    Class& k = classes[c];
    FreeNode* node = static_cast<FreeNode*>(p);
    lock(k);
    node->next = k.head;
    k.head = node;
    k.cached++;
    unlock(k);
}

// --- the hooked functions -----------------------------------------------------------------------------------------
inline void* __cdecl h_malloc(size_t n) {
    if (n > (static_cast<size_t>(1) << kMaxPow2)) {
        InterlockedIncrement64(&passthrough);
        return o_malloc(n);
    }
    return take(class_of(n), n);
}

inline void* __cdecl h_calloc(size_t a, size_t b) {
    if (b != 0 && a > SIZE_MAX / b) return nullptr;
    const size_t n = a * b;
    void* p = h_malloc(n);
    if (p) memset(p, 0, n);
    return p;
}

inline void __cdecl h_free(void* p) {
    if (!p) return;
    Hdr* h = hdr_of(p);
    if (h->magic == kMagic) {
        give(p, h);
    } else if (h->magic == kMagicAligned) {
        void* base = static_cast<uint8_t*>(p) - h->size;
        Hdr* bh = hdr_of(base);
        if (bh->magic == kMagic) { give(base, bh); return; }
        o_afree(p);
    } else {
        InterlockedIncrement64(&foreign_frees);
        o_free(p);
    }
}

inline void* __cdecl h_realloc(void* q, size_t n) {
    if (!q) return h_malloc(n);
    Hdr* h = hdr_of(q);
    if (h->magic != kMagic) {
        return o_realloc(q, n); // a block from before the hooks stays Wine's
    }
    if (n <= class_size(h->cls) && n > (class_size(h->cls) >> 2)) {
        h->size = n; // still fits, and is not shrinking to a quarter: keep the block
        return q;
    }
    void* p = h_malloc(n);
    if (!p) return nullptr;
    memcpy(p, q, h->size < n ? h->size : n);
    give(q, h);
    return p;
}

inline void* __cdecl h_amalloc(size_t n, size_t al) {
    if (al <= 16) return h_malloc(n);
    if (al > 4096 || (al & (al - 1)) != 0) {
        InterlockedIncrement64(&passthrough);
        return o_amalloc(n, al);
    }
    // room for the alignment and a header of ours right before the aligned pointer
    uint8_t* base = static_cast<uint8_t*>(h_malloc(n + al + kHdr));
    if (!base) return nullptr;
    uintptr_t a = reinterpret_cast<uintptr_t>(base) + kHdr;
    a = (a + al - 1) & ~(static_cast<uintptr_t>(al) - 1);
    uint8_t* p = reinterpret_cast<uint8_t*>(a);
    Hdr* ah = hdr_of(p);
    ah->magic = kMagicAligned;
    ah->cls = 0;
    ah->size = static_cast<uint64_t>(p - base);
    return p;
}

inline void __cdecl h_afree(void* p) {
    if (!p) return;
    Hdr* h = hdr_of(p);
    if (h->magic == kMagicAligned || h->magic == kMagic) { h_free(p); return; }
    InterlockedIncrement64(&foreign_frees);
    o_afree(p);
}

inline void* __cdecl h_arealloc(void* q, size_t n, size_t al) {
    if (!q) return h_amalloc(n, al);
    Hdr* h = hdr_of(q);
    size_t old = 0;
    if (h->magic == kMagicAligned) {
        Hdr* bh = hdr_of(static_cast<uint8_t*>(q) - h->size);
        if (bh->magic != kMagic) return o_arealloc(q, n, al);
        old = bh->size - h->size; // the base request minus what the alignment took
    } else if (h->magic == kMagic) {
        old = h->size;
    } else {
        return o_arealloc(q, n, al);
    }
    void* p = h_amalloc(n, al);
    if (!p) return nullptr;
    memcpy(p, q, old < n ? old : n);
    h_free(q);
    return p;
}

// --- install ------------------------------------------------------------------------------------------------------
inline bool is_heap_dll(const char* name) {
    char lower[128];
    size_t i = 0;
    for (; name[i] && i < sizeof(lower) - 1; i++) lower[i] = static_cast<char>(tolower(static_cast<unsigned char>(name[i])));
    lower[i] = 0;
    return strstr(lower, "api-ms-win-crt-heap") || strstr(lower, "ucrtbase");
}

inline int patch_module(HMODULE mod) {
    auto* base = reinterpret_cast<uint8_t*>(mod);
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE) return 0;
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (dir.VirtualAddress == 0) return 0;
    int patched = 0;
    for (auto* imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress); imp->Name; imp++) {
        const char* dll = reinterpret_cast<const char*>(base + imp->Name);
        if (!is_heap_dll(dll) || imp->OriginalFirstThunk == 0) continue;
        auto* names = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->OriginalFirstThunk);
        auto* addrs = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->FirstThunk);
        for (; names->u1.AddressOfData; names++, addrs++) {
            if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal)) continue;
            const char* fn =
                reinterpret_cast<const char*>(reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData)->Name);
            void* repl = nullptr;
            void** orig = nullptr;
            if (!strcmp(fn, "malloc")) repl = reinterpret_cast<void*>(&h_malloc), orig = reinterpret_cast<void**>(&o_malloc);
            else if (!strcmp(fn, "calloc")) repl = reinterpret_cast<void*>(&h_calloc), orig = reinterpret_cast<void**>(&o_calloc);
            else if (!strcmp(fn, "realloc")) repl = reinterpret_cast<void*>(&h_realloc), orig = reinterpret_cast<void**>(&o_realloc);
            else if (!strcmp(fn, "free")) repl = reinterpret_cast<void*>(&h_free), orig = reinterpret_cast<void**>(&o_free);
            else if (!strcmp(fn, "_aligned_malloc")) repl = reinterpret_cast<void*>(&h_amalloc), orig = reinterpret_cast<void**>(&o_amalloc);
            else if (!strcmp(fn, "_aligned_realloc")) repl = reinterpret_cast<void*>(&h_arealloc), orig = reinterpret_cast<void**>(&o_arealloc);
            else if (!strcmp(fn, "_aligned_free")) repl = reinterpret_cast<void*>(&h_afree), orig = reinterpret_cast<void**>(&o_afree);
            if (!repl) continue;
            void* cur = reinterpret_cast<void*>(addrs->u1.Function);
            if (cur == repl) continue;
            if (*orig == nullptr) *orig = cur;
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

/// The originals the hooks need, resolved from the UCRT itself (a module may import only some of the seven).
inline bool resolve_originals() {
    HMODULE u = GetModuleHandleA("ucrtbase.dll");
    if (!u) u = LoadLibraryA("ucrtbase.dll");
    if (!u) return false;
    auto get = [&](const char* n) { return reinterpret_cast<void*>(GetProcAddress(u, n)); };
    if (!o_malloc) o_malloc = reinterpret_cast<MallocFn>(get("malloc"));
    if (!o_calloc) o_calloc = reinterpret_cast<CallocFn>(get("calloc"));
    if (!o_realloc) o_realloc = reinterpret_cast<ReallocFn>(get("realloc"));
    if (!o_free) o_free = reinterpret_cast<FreeFn>(get("free"));
    if (!o_amalloc) o_amalloc = reinterpret_cast<AMallocFn>(get("_aligned_malloc"));
    if (!o_arealloc) o_arealloc = reinterpret_cast<AReallocFn>(get("_aligned_realloc"));
    if (!o_afree) o_afree = reinterpret_cast<FreeFn>(get("_aligned_free"));
    return o_malloc && o_calloc && o_realloc && o_free && o_amalloc && o_arealloc && o_afree;
}

/// Hooks every loaded module except this exe and the system's; returns the patched entries (0: nothing done).
inline int install() {
    if (!resolve_originals()) {
        printf("heapfix: the UCRT's heap functions were not found; not installed\n");
        return 0;
    }
    if (const char* e = getenv("PIBRIDGE_WB_HEAP_CACHE_MB"); e && atoi(e) > 0) cache_cap = static_cast<LONG64>(atoi(e)) << 20;
    HMODULE self = GetModuleHandleA(nullptr);
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
    if (snap == INVALID_HANDLE_VALUE) return 0;
    int total = 0;
    MODULEENTRY32 me;
    me.dwSize = sizeof(me);
    for (BOOL ok = Module32First(snap, &me); ok; ok = Module32Next(snap, &me)) {
        if (me.hModule == self) continue;
        char lower[MAX_PATH];
        size_t i = 0;
        for (; me.szExePath[i] && i < sizeof(lower) - 1; i++) lower[i] = static_cast<char>(tolower(static_cast<unsigned char>(me.szExePath[i])));
        lower[i] = 0;
        if (strstr(lower, "\\windows\\")) continue; // the system's (Wine's builtins): only the kit and its libraries
        total += patch_module(me.hModule);
    }
    CloseHandle(snap);
    enabled = total > 0;
    return total;
}

/// One line: what the cache did since the start.
inline void report() {
    LONG64 hits = 0, misses = 0, blocks = 0;
    unsigned top_c[4] = {};
    LONG64 top_b[4] = {};
    for (unsigned c = 0; c < kClasses; c++) {
        const Class& k = classes[c];
        hits += k.hits;
        misses += k.misses;
        blocks += k.cached;
        const LONG64 b = k.cached * static_cast<LONG64>(class_size(c));
        for (int j = 0; j < 4; j++) {
            if (b > top_b[j]) {
                for (int m = 3; m > j; m--) { top_b[m] = top_b[m - 1]; top_c[m] = top_c[m - 1]; }
                top_b[j] = b;
                top_c[j] = c;
                break;
            }
        }
    }
    printf("heapfix: %lld allocations, %.1f%% from the cache; cached %lld blocks / %.1f MB (cap %lld MB), released %lld, "
           "foreign frees %lld, pass-through %lld; largest classes:",
           static_cast<long long>(hits + misses), hits + misses ? 100.0 * hits / (hits + misses) : 0.0,
           static_cast<long long>(blocks), cached_bytes / 1048576.0, static_cast<long long>(cache_cap >> 20),
           static_cast<long long>(released), static_cast<long long>(foreign_frees), static_cast<long long>(passthrough));
    for (int j = 0; j < 4 && top_b[j] > 0; j++)
        printf(" %zu B x %lld", class_size(top_c[j]), static_cast<long long>(classes[top_c[j]].cached));
    printf("\n");
}

inline DWORD WINAPI reporter(LPVOID arg) {
    const DWORD ms = static_cast<DWORD>(reinterpret_cast<uintptr_t>(arg));
    for (;;) {
        Sleep(ms);
        report();
    }
    return 0;
}

inline void start_reporter(DWORD ms) {
    if (ms == 0) return;
    CreateThread(nullptr, 0, reporter, reinterpret_cast<LPVOID>(static_cast<uintptr_t>(ms)), 0, nullptr);
}

} // namespace heapfix
