// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/capi_ae.rs (MIT OR Apache-2.0), through libpibridge_pimax.a and the
// verbatim header (the text tests are in libpibridge-cam/tests/test_ae.cpp).
#include "pimax_cam.h"

#include "pibridge_test.h"

#include <cstddef>
#include <cstdlib>
#include <string>

TEST(create_update_destroy_with_null_camera) {
    pimax_cam_ae* a = pimax_cam_ae_create();
    CHECK(a != nullptr);
    pimax_cam_ae_out o{};
    CHECK(pimax_cam_ae_update(a, nullptr, 0, &o) == -1);
    CHECK(std::string(pimax_cam_last_error()) == "null camera");
    pimax_cam_ae_destroy(a);
}

TEST(create_for_a_named_model) {
    for (const char* name : {"crystal-super", "dream-air", ""}) {
        pimax_cam_ae* a = pimax_cam_ae_create_model(name);
        CHECK(a != nullptr);
        pimax_cam_ae_destroy(a);
    }
    pimax_cam_ae* a = pimax_cam_ae_create_model(nullptr);
    CHECK(a != nullptr);
    pimax_cam_ae_destroy(a);
    if (std::getenv("PIMAX_CAM_AE_MODEL") == nullptr) {
        CHECK(pimax_cam_ae_create_model("8kx") == nullptr);
        CHECK(std::string(pimax_cam_last_error()) == "unknown headset model \"8kx\" (dream-air or crystal-super)");
    }
}

TEST(out_layout_is_stable) {
    CHECK(sizeof(pimax_cam_ae_out) == 8 + 24 + 24 + 8 + 8 + 4 + 8 + 8 + 8 + 8 + 8 + 4);
    CHECK(offsetof(pimax_cam_ae_out, payload) == 32);
    CHECK(offsetof(pimax_cam_ae_out, mean) == 92);
}

PIBRIDGE_TEST_MAIN()
