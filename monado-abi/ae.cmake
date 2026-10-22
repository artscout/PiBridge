# SPDX-License-Identifier: MIT
# The auto-exposure part of monado-abi (M3): pimax_cam_ae_* of pimax_cam.h. The twins of the Rust capi_ae.rs tests
# through the archive, and the payloads against PiBridge's allowlist gate (devices/crystal-super.toml `camera_ae`).
# Included from CMakeLists.txt inside PIBRIDGE_BUILD_TESTS.

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

add_executable(monado_abi_test_ae_capi tests/test_ae_capi.cpp)
target_link_libraries(monado_abi_test_ae_capi PRIVATE pibridge_pimax pibridge_common)
add_test(NAME monado_abi_ae_capi COMMAND monado_abi_test_ae_capi)

# libpibridge-cam's ae (the code inside the archive) against libpibridge-hmd's gate: the Rust allowlist.rs twin and the
# property test (random rooms x every PIMAX_CAM_AE_* override).
if(TARGET pibridge_hmd)
    add_executable(monado_abi_test_ae_gate tests/test_ae_gate.cpp)
    target_link_libraries(monado_abi_test_ae_gate PRIVATE pibridge_cam pibridge_hmd pibridge_common)
    target_compile_options(monado_abi_test_ae_gate PRIVATE -ffp-contract=off)
    target_compile_definitions(monado_abi_test_ae_gate PRIVATE PIBRIDGE_SOURCE_DEVICES_DIR="${PIBRIDGE_SOURCE_DEVICES_DIR}")
    add_test(NAME monado_abi_ae_gate COMMAND monado_abi_test_ae_gate)
    set_tests_properties(monado_abi_ae_gate PROPERTIES TIMEOUT 600)
endif()
