// SPDX-License-Identifier: MIT
// Test stub: Monado's monotonic clock (os/os_time.h), with an offset the test can move to step past the hook's
// 250 ms re-read interval without sleeping.
#pragma once
#include <stdint.h>
#include <time.h>

extern int64_t pibridge_test_clock_shift_ns;

static inline uint64_t os_monotonic_get_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec + (uint64_t)pibridge_test_clock_shift_ns;
}
