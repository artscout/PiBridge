# SPDX-License-Identifier: MIT
# libpibridge-anchor: the world anchor files shared with Monado's room-anchor hook (PIMAX_ANCHOR=1): the offset file
# (writer and reader), the one-writer lease with its flock, the head-pose ring reader and the feed check, the offset
# maths, and anchor.toml. Ported from omarchyvr omarchyvr-anchor (world.rs, config.rs; MIT OR Apache-2.0); the
# AprilTag half (detect.rs, tag.rs, rig.rs) is not ported (docs/libpibridge-anchor.md). Namespace pibridge::anchor, target
# pibridge_anchor. Needs libpibridge-cam (geom, io, rustfmt) and libpibridge-ctrl's tracker half (quat_to_mat).

add_library(pibridge_anchor_obj OBJECT
    src/rustx.cpp
    src/world.cpp
    src/config.cpp
)
target_include_directories(pibridge_anchor_obj PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(pibridge_anchor_obj PUBLIC $<TARGET_PROPERTY:pibridge_cam_obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(pibridge_anchor_obj PUBLIC $<TARGET_PROPERTY:pibridge_ctrltrack_obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_features(pibridge_anchor_obj PUBLIC cxx_std_23)
# -fno-math-errno: the libm calls stay the same (bit-identical values), only errno is not set, so sin/cos/exp of
# a loop-invariant argument can be hoisted as LLVM does for the Rust code.
target_compile_options(pibridge_anchor_obj PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS} -fno-math-errno)

add_library(pibridge_anchor STATIC $<TARGET_OBJECTS:pibridge_anchor_obj>)
target_include_directories(pibridge_anchor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_anchor PUBLIC cxx_std_23)
target_link_libraries(pibridge_anchor PUBLIC pibridge_ctrltrack pibridge_cam)

if(PIBRIDGE_BUILD_TESTS)
    # Monado's own hook (pimax_anchor.h, included from the omarchyvr checkout, not copied) as the other side of the
    # files: it writes the head ring and reads the offset this library writes. Built with stub headers for the few
    # Monado helpers it calls (math, clock, log); skipped when the checkout is not next to PiBridge.
    set(anchor_monado_dir "${PIBRIDGE_OMARCHYVR_DIR}/forks/monado/src/xrt")
    set(anchor_test_defs)
    set(anchor_test_libs)
    if(EXISTS "${anchor_monado_dir}/drivers/pimax/pimax_anchor.h")
        add_library(anchor_monado_side STATIC tests/monado_side.c)
        target_include_directories(anchor_monado_side PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests/monado_stub
            ${anchor_monado_dir}/include ${anchor_monado_dir}/auxiliary ${anchor_monado_dir}/drivers/pimax)
        set_target_properties(anchor_monado_side PROPERTIES C_STANDARD 11)
        target_compile_options(anchor_monado_side PRIVATE -w)
        target_link_libraries(anchor_monado_side PRIVATE m)
        set(anchor_test_defs PIBRIDGE_HAVE_MONADO_ANCHOR=1)
        set(anchor_test_libs anchor_monado_side)
    endif()
    # the parity probe against the oracle's output (tests/golden, see anchor_probe.cpp)
    add_executable(pibridge_anchor_probe tests/anchor_probe.cpp)
    target_link_libraries(pibridge_anchor_probe PRIVATE pibridge_anchor)
    target_compile_options(pibridge_anchor_probe PRIVATE -ffp-contract=off)
    add_test(NAME pibridge_anchor_probe COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/cmp_golden.sh
        $<TARGET_FILE:pibridge_anchor_probe> ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/anchor_probe.txt.gz)
    set_tests_properties(pibridge_anchor_probe PROPERTIES ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
    foreach(t world config monado)
        add_executable(pibridge_anchor_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_anchor_test_${t} PRIVATE pibridge_anchor pibridge_common ${anchor_test_libs})
        target_compile_options(pibridge_anchor_test_${t} PRIVATE -ffp-contract=off)
        target_compile_definitions(pibridge_anchor_test_${t} PRIVATE ${anchor_test_defs})
        add_test(NAME pibridge_anchor_${t} COMMAND pibridge_anchor_test_${t})
        set_tests_properties(pibridge_anchor_${t} PROPERTIES SKIP_RETURN_CODE 77
            ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR};LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    endforeach()
endif()
