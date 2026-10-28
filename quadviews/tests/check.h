// SPDX-License-Identifier: MIT
// A minimal test harness: TEST(name) { CHECK(...); } and main() from QV_TEST_MAIN.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

namespace qvtest {
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
} // namespace qvtest

#define TEST(name)                                                                                                     \
    static void name();                                                                                                \
    static qvtest::Reg reg_##name(#name, name);                                                                        \
    static void name()

#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                              \
            qvtest::failures()++;                                                                                      \
        }                                                                                                              \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                                                          \
    do {                                                                                                               \
        const double a_ = (a), b_ = (b);                                                                               \
        if (!(std::fabs(a_ - b_) <= (eps))) {                                                                          \
            std::printf("  FAIL %s:%d: %s = %.6f, %s = %.6f\n", __FILE__, __LINE__, #a, a_, #b, b_);                   \
            qvtest::failures()++;                                                                                      \
        }                                                                                                              \
    } while (0)

#define QV_TEST_MAIN                                                                                                   \
    int main() {                                                                                                       \
        for (auto& c : qvtest::cases()) {                                                                              \
            const int before = qvtest::failures();                                                                     \
            c.fn();                                                                                                    \
            std::printf("%s %s\n", qvtest::failures() == before ? "ok  " : "FAIL", c.name);                            \
        }                                                                                                              \
        return qvtest::failures() ? 1 : 0;                                                                             \
    }
