# SPDX-License-Identifier: MIT
# The lens part of monado-abi (M5): twins of the Rust capi.rs tests through the archive, and the lens difftest
# (difftest/lens_difftest.c against libpibridge_pimax.a and against omarchyvr's libpimax_lens.a). Included from
# CMakeLists.txt inside PIBRIDGE_BUILD_TESTS when libpibridge-lens is built; uses its variables (d, run, w, ...).

add_executable(monado_abi_test_lens_capi tests/test_lens_capi.cpp)
target_link_libraries(monado_abi_test_lens_capi PRIVATE pibridge_pimax pibridge_common)
add_test(NAME monado_abi_lens_capi COMMAND monado_abi_test_lens_capi)
set_tests_properties(monado_abi_lens_capi PROPERTIES ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")

add_executable(lens_difftest_pibridge ${d}/lens_difftest.c)
target_link_libraries(lens_difftest_pibridge PRIVATE pibridge_pimax ZLIB::ZLIB)
set_target_properties(lens_difftest_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
target_compile_options(lens_difftest_pibridge PRIVATE ${pibridge_difftest_c_opts} -Wno-format-truncation -ffp-contract=off)
if(EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_lens.a")
    add_executable(lens_difftest_rust ${d}/lens_difftest.c)
    target_include_directories(lens_difftest_rust PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(lens_difftest_rust PRIVATE "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_lens.a" ZLIB::ZLIB pthread dl m
                          rt gcc_s)
    set_target_properties(lens_difftest_rust PROPERTIES C_STANDARD 11)
    target_compile_options(lens_difftest_rust PRIVATE ${pibridge_difftest_c_opts} -Wno-format-truncation -ffp-contract=off)
    set(pibridge_lens_difftest_rust $<TARGET_FILE:lens_difftest_rust>)
else()
    set(pibridge_lens_difftest_rust "${CMAKE_CURRENT_BINARY_DIR}/no-rust-oracle")
endif()

set(lf $<TARGET_FILE:lens_difftest_pibridge>)
set(lr ${pibridge_lens_difftest_rust})
set(crystal_lens ${PIBRIDGE_LAB_LENS_DIR}/crystal.conf)
add_test(NAME monado_abi_diff_lens_synth COMMAND ${run} lens-synth ${w} ${lr} ${lf} -- synth @W@)
add_test(NAME monado_abi_diff_lens_crystal COMMAND ${run} lens-crystal ${w} ${lr} ${lf}
         --need ${crystal_lens} --need ${PIBRIDGE_LAB_LENS_DIR}/crystal-lut.bin -- conf @W@ ${crystal_lens} 513)
add_test(NAME monado_abi_diff_lens_reference COMMAND ${run} lens-reference ${w} ${lr} ${lf}
         -- conf @W@ ${PIBRIDGE_LENS_REFERENCE_CONF} 513)
set(lens_diff_tests monado_abi_diff_lens_synth monado_abi_diff_lens_crystal monado_abi_diff_lens_reference)
set_tests_properties(${lens_diff_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
