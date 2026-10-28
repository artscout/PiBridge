// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tests for the process-wide GPU wedge flag (util/u_gpu_wedge.h).
 */

#include "util/u_gpu_wedge.h"

#include "catch_amalgamated.hpp"

#include <string>

TEST_CASE("gpu_wedge")
{
	u_gpu_wedge_clear();
	CHECK_FALSE(u_gpu_wedge_is_set());
	CHECK(std::string(u_gpu_wedge_reason()).empty());

	SECTION("the first reason is kept")
	{
		u_gpu_wedge_set("first");
		u_gpu_wedge_set("second");
		CHECK(u_gpu_wedge_is_set());
		CHECK(std::string(u_gpu_wedge_reason()) == "first");
	}

	SECTION("clear resets the flag and the reason")
	{
		u_gpu_wedge_set("stuck");
		u_gpu_wedge_clear();
		CHECK_FALSE(u_gpu_wedge_is_set());
		u_gpu_wedge_set("again");
		CHECK(std::string(u_gpu_wedge_reason()) == "again");
	}

	SECTION("a NULL reason is reported as unknown")
	{
		u_gpu_wedge_set(nullptr);
		CHECK(std::string(u_gpu_wedge_reason()) == "unknown");
	}

	u_gpu_wedge_clear();
}
