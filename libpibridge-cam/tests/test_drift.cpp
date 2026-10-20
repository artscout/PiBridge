// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/drift.rs (MIT OR Apache-2.0).
#include "pibridge/cam/drift.h"
#include "pibridge_test.h"

#include <cmath>
#include <string>

using namespace pibridge::cam::drift;

TEST(removal_band_is_continuous) {
    const DriftGov g(DriftParams{}, {0.0, 0.0, 0.0});
    CHECK(g.removed_speed(0.02, 0.03) == 0.0);
    CHECK(std::fabs(g.removed_speed(0.03, 0.0) - 0.03) < 1e-12);
    CHECK(std::fabs(g.removed_speed(0.05, 0.0) - 0.05) < 1e-12);
    CHECK(g.removed_speed(0.1, 0.0) < 0.05 && g.removed_speed(0.1, 0.0) > 0.0);
    CHECK(g.removed_speed(0.15, 0.0) == 0.0);
    CHECK(g.removed_speed(0.5, 0.0) == 0.0);
}

TEST(report_line_has_the_words_the_app_reads) {
    const std::string l = report_line(DriftReport{0.123, 10.0, 1.5});
    CHECK(l == "6DoF drift: head tracking slid 12 cm without movement in 10 s; held (session total 150 cm)");
    DriftGov g(DriftParams{}, {0.0, 0.0, 0.0});
    CHECK(!g.take_report(0));
    g.report_m = 0.2;
    CHECK(!g.take_report(5'000'000'000)); // not before 10 s
    const auto r = g.take_report(10'000'000'000);
    CHECK(r && r->m == 0.2);
    CHECK(!g.take_report(20'000'000'000)); // nothing new held
}

PIBRIDGE_TEST_MAIN()
