# SPDX-License-Identifier: MIT
# The controller part of monado-abi (M6): pimax_hands.h, pimax_ctrlfuse.h, pimax_inhand.h. The twins of the Rust ffi /
# c_abi tests through the archive. Included from CMakeLists.txt inside PIBRIDGE_BUILD_TESTS when libpibridge-ctrl's
# hands half is built.

find_package(ZLIB REQUIRED)

add_executable(monado_abi_test_hands_capi tests/test_hands_capi.cpp)
target_link_libraries(monado_abi_test_hands_capi PRIVATE pibridge_pimax pibridge_common)
target_compile_definitions(monado_abi_test_hands_capi PRIVATE
    PIBRIDGE_HANDS_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/pimax_hands.h")
add_test(NAME monado_abi_hands_capi COMMAND monado_abi_test_hands_capi)
set_tests_properties(monado_abi_hands_capi PROPERTIES
    ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR};HOME=${CMAKE_CURRENT_BINARY_DIR}/home;XDG_RUNTIME_DIR=${CMAKE_CURRENT_BINARY_DIR}")
# the Rust twins expect the Rust fuser: PiBridge's fuser improvements off (docs/research/ctrl-tracking-improvements.md)
set_tests_properties(monado_abi_hands_capi PROPERTIES ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
