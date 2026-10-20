// SPDX-License-Identifier: MIT
// Twin of the test in omarchyvr pimax-cam/src/stats.rs (MIT OR Apache-2.0), plus the report text.
#include "pibridge/cam/stats.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::cam;

TEST(series_and_capture_stats) {
    Series s;
    for (const double x : {1.0, 2.0, 3.0, 4.0}) {
        s.push(x);
    }
    CHECK(s.mean() == 2.5);
    CHECK(std::fabs(s.std() - 1.118) < 1e-3);
    CHECK(s.quantile(0.0) == 1.0);
    CHECK(s.quantile(1.0) == 4.0);
    CaptureStats c;
    for (std::uint64_t n = 0; n < 51; n++) {
        FrameInfo i;
        i.host_ts_ns = 1'000'000'000 + n * 20'000'000 + (n % 3) * 1'000'000;
        i.dev_ts_us = 10'000 + n * 20'000;
        i.mono_ts_ns = 1;
        i.kind = n % 2 == 0 ? 1 : 2;
        c.add(i);
    }
    c.finish();
    CHECK(std::fabs(c.fps() - 50.0) < 0.2);
    CHECK(std::fabs(c.dev_dt_ms.mean() - 20.0) < 1e-9);
    CHECK(std::fabs(c.tracking_dt_ms.mean() - 40.0) < 1e-9);
    CHECK(std::fabs(c.latency_ms.max() - 2.0) < 1e-6);
    CHECK(c.settings.size() == 2);
    CHECK(c.report().find("fps") != std::string::npos);
}

TEST(empty_report_is_the_rust_text) {
    CaptureStats c;
    c.finish();
    CHECK(c.report() ==
          "frames 0 in 0.00 s = 0.000 fps, lost 0 (0.000%)\n"
          "  host interval   -0.000 ± -0.000 [inf .. -inf] ms (n=0)\n"
          "  device interval -0.000 ± -0.000 [inf .. -inf] ms (n=0)\n"
          "  tracking-frame interval (device) -0.000 ± -0.000 [inf .. -inf] ms (n=0)\n"
          "  arrival latency above minimum: p50 NaN p90 NaN p99 NaN max -inf ms\n"
          "  fallback clock: mapped time - arrival -0.000 ± -0.000 [inf .. -inf] ms (n=0)\n");
}

PIBRIDGE_TEST_MAIN()
