# SPDX-License-Identifier: MIT
# The SLAM / 6DoF part of monado-abi (M4): its C ABI tests through the archive. Included from CMakeLists.txt inside
# PIBRIDGE_BUILD_TESTS.

# Twins of the capi_slam.rs tests through the archive (PIBRIDGE_HEAD_PARITY=1: the output layer as the Rust original,
# without PiBridge's 6DoF improvements, docs/research/head-tracking-improvements.md).
add_executable(monado_abi_test_capi_slam tests/test_capi_slam.cpp)
target_link_libraries(monado_abi_test_capi_slam PRIVATE pibridge_pimax pibridge_cam pibridge_common)
target_compile_definitions(monado_abi_test_capi_slam PRIVATE PIBRIDGE_CAM_REFERENCE_XML="${PIBRIDGE_CAM_REFERENCE_XML}")
add_test(NAME monado_abi_capi_slam COMMAND monado_abi_test_capi_slam)
set_tests_properties(monado_abi_capi_slam PROPERTIES ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR};PIBRIDGE_HEAD_PARITY=1")

# PiBridge's own 6DoF additions through the archive (not in the Rust code): the recorder's calibration in use.
add_executable(monado_abi_test_capi_slam_pibridge tests/test_capi_slam_pibridge.cpp)
target_link_libraries(monado_abi_test_capi_slam_pibridge PRIVATE pibridge_pimax pibridge_common)
target_compile_definitions(monado_abi_test_capi_slam_pibridge PRIVATE PIBRIDGE_CAM_REFERENCE_XML="${PIBRIDGE_CAM_REFERENCE_XML}")
add_test(NAME monado_abi_capi_slam_pibridge COMMAND monado_abi_test_capi_slam_pibridge)
set_tests_properties(monado_abi_capi_slam_pibridge PROPERTIES ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
