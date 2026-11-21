// SPDX-License-Identifier: MIT
// heapfix_test: the caching allocator (heapfix.h) hammered from several threads under Wine, contents checked.
// Build: x86_64-w64-mingw32-g++ -O2 -static -std=c++20 heapfix_test.cpp -o heapfix_test.exe
// Run:   wine heapfix_test.exe [seconds]
#include "heapfix.h"

#include <cstdlib>
#include <vector>

namespace {

struct Slot {
    void* p = nullptr;
    size_t n = 0;
    uint8_t fill = 0;
    bool aligned = false;
    bool foreign = false; // allocated with the real malloc, freed through the hook (a pre-hook block)
};

volatile LONG g_errors = 0;
volatile LONG g_ops = 0;
DWORD g_seconds = 20;

size_t pick_size(uint64_t& rng) {
    rng = rng * 6364136223846793005ull + 1442695040888963407ull;
    const unsigned k = static_cast<unsigned>(rng >> 60);
    const uint64_t r = rng >> 20;
    switch (k) {
    case 0: return 0;
    case 1: case 2: case 3: case 4: return 1 + r % 256;         // small
    case 5: case 6: case 7: case 8: return 256 + r % 4096;      // the 1-32 KB band
    case 9: case 10: case 11: return 4096 + r % (64 << 10);
    case 12: case 13: return 64 << 10;                         // repeated same size
    case 14: return 640 * 480;                                 // an image
    default: return r % (2 << 20);
    }
}

void check(Slot& s) {
    if (!s.p) return;
    const uint8_t* b = static_cast<const uint8_t*>(s.p);
    for (size_t i = 0; i < s.n; i++) {
        if (b[i] != static_cast<uint8_t>(s.fill + i)) {
            InterlockedIncrement(&g_errors);
            printf("content error: slot size %zu at %zu\n", s.n, i);
            return;
        }
    }
}

void fill(Slot& s) {
    uint8_t* b = static_cast<uint8_t*>(s.p);
    for (size_t i = 0; i < s.n; i++) b[i] = static_cast<uint8_t>(s.fill + i);
}

DWORD WINAPI worker(LPVOID arg) {
    uint64_t rng = 0x9e3779b97f4a7c15ull ^ (reinterpret_cast<uintptr_t>(arg) * 0x100000001b3ull);
    std::vector<Slot> slots(400);
    const DWORD end = GetTickCount() + g_seconds * 1000;
    while (GetTickCount() < end) {
        rng = rng * 6364136223846793005ull + 1442695040888963407ull;
        Slot& s = slots[(rng >> 33) % slots.size()];
        const unsigned op = static_cast<unsigned>((rng >> 24) % 16);
        if (s.p) {
            check(s);
            if (op < 10) {
                if (s.aligned) heapfix::h_afree(s.p);
                else heapfix::h_free(s.p);
                s.p = nullptr;
            } else if (op < 13 && !s.aligned && !s.foreign) {
                const size_t n2 = pick_size(rng);
                void* q = heapfix::h_realloc(s.p, n2);
                if (!q) { InterlockedIncrement(&g_errors); printf("realloc failed\n"); continue; }
                // the common prefix survives
                const uint8_t* b = static_cast<const uint8_t*>(q);
                const size_t keep = n2 < s.n ? n2 : s.n;
                for (size_t i = 0; i < keep; i++)
                    if (b[i] != static_cast<uint8_t>(s.fill + i)) { InterlockedIncrement(&g_errors); printf("realloc content error\n"); break; }
                s.p = q;
                s.n = n2;
                fill(s);
            } else if (op < 15 && s.aligned) {
                const size_t n2 = pick_size(rng);
                void* q = heapfix::h_arealloc(s.p, n2, 64);
                if (!q || (reinterpret_cast<uintptr_t>(q) & 63)) { InterlockedIncrement(&g_errors); printf("aligned realloc bad\n"); continue; }
                const uint8_t* b = static_cast<const uint8_t*>(q);
                const size_t keep = n2 < s.n ? n2 : s.n;
                for (size_t i = 0; i < keep; i++)
                    if (b[i] != static_cast<uint8_t>(s.fill + i)) { InterlockedIncrement(&g_errors); printf("aligned realloc content error\n"); break; }
                s.p = q;
                s.n = n2;
                fill(s);
            }
        } else {
            s.n = pick_size(rng);
            s.fill = static_cast<uint8_t>(rng >> 40);
            s.aligned = false;
            s.foreign = false;
            if (op < 2) {
                s.aligned = true;
                const size_t al = (op == 0) ? 32 : 4096;
                s.p = heapfix::h_amalloc(s.n, al);
                if (s.p && (reinterpret_cast<uintptr_t>(s.p) & (al - 1))) { InterlockedIncrement(&g_errors); printf("misaligned\n"); }
            } else if (op == 2) {
                s.p = heapfix::h_calloc(s.n, 1);
                if (s.p) {
                    const uint8_t* b = static_cast<const uint8_t*>(s.p);
                    for (size_t i = 0; i < s.n; i++) if (b[i]) { InterlockedIncrement(&g_errors); printf("calloc not zero\n"); break; }
                }
            } else if (op == 3) {
                s.foreign = true;
                s.p = heapfix::o_malloc(s.n);
            } else {
                s.p = heapfix::h_malloc(s.n);
            }
            if (!s.p) { InterlockedIncrement(&g_errors); printf("alloc failed (%zu)\n", s.n); continue; }
            if (reinterpret_cast<uintptr_t>(s.p) & 15) { InterlockedIncrement(&g_errors); printf("not 16-aligned\n"); }
            fill(s);
        }
        InterlockedIncrement(&g_ops);
    }
    for (Slot& s : slots) {
        check(s);
        if (s.p) { if (s.aligned) heapfix::h_afree(s.p); else heapfix::h_free(s.p); }
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1) g_seconds = static_cast<DWORD>(atoi(argv[1]));
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!heapfix::resolve_originals()) { printf("no UCRT originals\n"); return 1; }
    // size class sanity
    for (size_t n = 0; n < 300000; n = n < 2048 ? n + 1 : n + 37) {
        const unsigned c = heapfix::class_of(n);
        const size_t sz = heapfix::class_size(c);
        if (sz < n || sz > (n < 1024 ? n + 16 : n + n / 8 + 16) || c >= heapfix::kClasses) {
            printf("class error: n %zu -> class %u size %zu\n", n, c, sz);
            return 1;
        }
    }
    printf("classes ok (%u)\n", heapfix::kClasses);
    HANDLE th[6];
    const DWORD t0 = GetTickCount();
    for (int i = 0; i < 6; i++) th[i] = CreateThread(nullptr, 0, worker, reinterpret_cast<LPVOID>(static_cast<uintptr_t>(i + 1)), 0, nullptr);
    WaitForMultipleObjects(6, th, TRUE, INFINITE);
    const DWORD dt = GetTickCount() - t0;
    heapfix::report();
    printf("%ld ops in %lu ms (%.0f k/s), %ld errors\n", g_ops, dt, g_ops / (dt / 1000.0) / 1000.0, g_errors);
    return g_errors ? 1 : 0;
}
