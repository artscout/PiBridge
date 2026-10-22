# SPDX-License-Identifier: MIT
# The lens part of monado-abi (M5): twins of the Rust capi.rs tests through the archive. Included from CMakeLists.txt
# inside PIBRIDGE_BUILD_TESTS when libpibridge-lens is built.

add_executable(monado_abi_test_lens_capi tests/test_lens_capi.cpp)
target_link_libraries(monado_abi_test_lens_capi PRIVATE pibridge_pimax pibridge_common)
add_test(NAME monado_abi_lens_capi COMMAND monado_abi_test_lens_capi)
set_tests_properties(monado_abi_lens_capi PROPERTIES ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
