// SPDX-License-Identifier: MIT
// A minimal test harness shared by every part: TEST(name) { CHECK(...); } and one PIBRIDGE_TEST_MAIN() per test binary.
// No dependencies. A failing CHECK prints file:line and the expression, and the binary exits non-zero.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace pibridge::test {

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& cases() {
    static std::vector<Case> c;
    return c;
}

inline int& failures() {
    static int f = 0;
    return f;
}

struct Reg {
    Reg(const char* n, std::function<void()> f) { cases().push_back({n, std::move(f)}); }
};

inline int run_all(int argc, char** argv) {
    const std::string only = argc > 1 ? argv[1] : "";
    int ran = 0;
    for (const auto& c : cases()) {
        if (!only.empty() && std::string(c.name).find(only) == std::string::npos) {
            continue;
        }
        const int before = failures();
        c.fn();
        std::printf("%s %s\n", failures() == before ? "ok  " : "FAIL", c.name);
        ran++;
    }
    std::printf("%d tests, %d failed checks\n", ran, failures());
    return failures() == 0 ? 0 : 1;
}

} // namespace pibridge::test

#define PIBRIDGE_CAT2(a, b) a##b
#define PIBRIDGE_CAT(a, b) PIBRIDGE_CAT2(a, b)
#define TEST(name)                                                                \
    static void name();                                                           \
    static const pibridge::test::Reg PIBRIDGE_CAT(pibridge_reg_, name){#name, name};                 \
    static void name()
#define CHECK(expr)                                                               \
    do {                                                                          \
        if (!(expr)) {                                                            \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #expr); \
            pibridge::test::failures()++;                                                \
        }                                                                         \
    } while (0)
#define CHECK_NEAR(a, b, eps)                                                     \
    do {                                                                          \
        const double pibridge_a = (a), pibridge_b = (b);                                      \
        if (!(std::fabs(pibridge_a - pibridge_b) <= (eps))) {                                 \
            std::fprintf(stderr, "%s:%d: CHECK_NEAR(%s, %s) failed: %g vs %g\n", __FILE__, __LINE__, #a, #b, pibridge_a, pibridge_b); \
            pibridge::test::failures()++;                                                \
        }                                                                         \
    } while (0)
#define PIBRIDGE_TEST_MAIN()                                                            \
    int main(int argc, char** argv) { return pibridge::test::run_all(argc, argv); }
