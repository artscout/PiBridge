// SPDX-License-Identifier: MIT
#include "pibridge/room/chap.h"
#include "pibridge/room/json.h"
#include "pibridge_test.h"
#include "selftest.h"

using namespace pibridge::roomsetup;

TEST(synthetic_file_parses) {
    auto j = pibridge::room::parseJson(selfTestVrchap());
    CHECK(j.has_value());
    CHECK(pibridge::room::universesOf(*j).size() == 2);
}

TEST(self_test_passes) { CHECK(runSelfTest("") == 0); }

PIBRIDGE_TEST_MAIN()
