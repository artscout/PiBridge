# SPDX-License-Identifier: MIT
# The auto-exposure part of monado-abi (M3): pimax_cam_ae_* of pimax_cam.h. The difftest (difftest/ae_difftest.c,
# with its own in-process camera broker so that every frame is "live") against libpibridge_pimax.a and against
# omarchyvr's libpimax_cam.a (the Rust oracle), the twins of the Rust capi_ae.rs tests through the archive, and the
# payloads against PiBridge's allowlist gate (devices/crystal-super.toml `camera_ae`). Included from CMakeLists.txt inside
# PIBRIDGE_BUILD_TESTS; uses its variables (d, run, w, pibridge_difftest_c_opts).

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

add_executable(ae_difftest_pibridge ${d}/ae_difftest.c)
target_link_libraries(ae_difftest_pibridge PRIVATE pibridge_pimax ZLIB::ZLIB Threads::Threads m)
set_target_properties(ae_difftest_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
target_compile_options(ae_difftest_pibridge PRIVATE ${pibridge_difftest_c_opts} -ffp-contract=off -Wno-format-truncation)
if(EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a")
    add_executable(ae_difftest_rust ${d}/ae_difftest.c)
    target_include_directories(ae_difftest_rust PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(ae_difftest_rust PRIVATE "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a" ZLIB::ZLIB pthread dl m rt
                          gcc_s)
    set_target_properties(ae_difftest_rust PROPERTIES C_STANDARD 11)
    target_compile_options(ae_difftest_rust PRIVATE ${pibridge_difftest_c_opts} -ffp-contract=off -Wno-format-truncation)
    set(ae_r $<TARGET_FILE:ae_difftest_rust>)
else()
    set(ae_r "${CMAKE_CURRENT_BINARY_DIR}/no-rust-oracle")
endif()
set(ae_f $<TARGET_FILE:ae_difftest_pibridge>)

set(ae_tests)
# The Rust panic text (an override that crosses f64::clamp's bounds) is not part of the ABI: the C++ library returns
# the same NULL and "panic inside pimax_cam" without it.
set(ae_panic_text "^$|panicked at|^min > max|^assertion failed|RUST_BACKTRACE")
macro(ae_diff name)
    add_test(NAME monado_abi_diff_${name} COMMAND ${run} ${name} ${w} ${ae_r} ${ae_f} ${ARGN})
    list(APPEND ae_tests monado_abi_diff_${name})
endmacro()

ae_diff(ae_layout -- layout)
ae_diff(ae_create -- create)
ae_diff(ae_create_env_crystal --env PIMAX_CAM_AE_MODEL=crystal-super -- create)
ae_diff(ae_create_env_dream --env "PIMAX_CAM_AE_MODEL= Dream_Air " -- create)
ae_diff(ae_create_env_bogus --env PIMAX_CAM_AE_MODEL=8kx -- create)
ae_diff(ae_create_overrides --env PIMAX_CAM_AE_EXP_MAX=3000 --env PIMAX_CAM_AE_GAIN_MAX=100 --env PIMAX_CAM_AE_EXTEND=0
        -- create)
ae_diff(ae_create_old_law --env "PIMAX_CAM_AE_LAW= OLD" --env PIMAX_CAM_AE_EXP_MIN=10 -- create)
ae_diff(ae_create_panic --ignore "${ae_panic_text}" --env PIMAX_CAM_AE_EXP_MAX=nan --env PIMAX_CAM_AE_EXP_MIN=5
        -- create)
ae_diff(ae_errors -- errors @W@)
# The scene test drives the head tracker: compared with the Rust oracle in parity mode (no PiBridge 6DoF changes).
ae_diff(ae_scene --env PIMAX_FUSE_AUTO=1 --env PIBRIDGE_HEAD_PARITY=1 -- scene @W@)
# Recordings (lab data), open loop: every tracking frame's own metadata.
foreach(rec first ctrl1 ctrl2)
    set(px ${PIBRIDGE_LAB_CAM_DIR}/${rec}.pxcam)
    ae_diff(ae_pxcam_${rec}_crystal --need ${px} -- pxcam @W@ crystal-super ${px})
    ae_diff(ae_pxcam_${rec}_dream --need ${px} -- pxcam @W@ dream-air ${px})
endforeach()
ae_diff(ae_pxcam_first_detect_env --need ${PIBRIDGE_LAB_CAM_DIR}/first.pxcam --env PIMAX_CAM_AE_MODEL=0x0040
        -- pxcam @W@ - ${PIBRIDGE_LAB_CAM_DIR}/first.pxcam)
ae_diff(ae_pxcam_first_old_law --need ${PIBRIDGE_LAB_CAM_DIR}/first.pxcam --env PIMAX_CAM_AE_LAW=v1
        -- pxcam @W@ crystal-super ${PIBRIDGE_LAB_CAM_DIR}/first.pxcam)
# The simulated closed loop: random rooms; a Crystal Super run fails (exit 3) on any payload outside the ranges.
foreach(seed 1 2 3 4 5 6)
    ae_diff(ae_synth_crystal_${seed} -- synth @W@ crystal-super ${seed} 60)
endforeach()
foreach(seed 1 2 3)
    ae_diff(ae_synth_dream_${seed} -- synth @W@ dream-air ${seed} 60)
endforeach()
# Every PIMAX_CAM_AE_* override on the Crystal Super (and a few on the Dream Air), extreme values included.
set(ae_overrides
    "PIMAX_CAM_AE_TARGET=200" "PIMAX_CAM_AE_TARGET=1" "PIMAX_CAM_AE_EXP_MIN=100" "PIMAX_CAM_AE_EXP_MIN=9000"
    "PIMAX_CAM_AE_EXP_MAX=100" "PIMAX_CAM_AE_EXP_MAX=65535" "PIMAX_CAM_AE_GAIN_MIN=0" "PIMAX_CAM_AE_GAIN_MIN=400"
    "PIMAX_CAM_AE_GAIN_MAX=1" "PIMAX_CAM_AE_GAIN_MAX=900" "PIMAX_CAM_AE_PERIOD_MS=1" "PIMAX_CAM_AE_PERIOD_MS=2000"
    "PIMAX_CAM_AE_K=1" "PIMAX_CAM_AE_DEADBAND=0" "PIMAX_CAM_AE_UPPER=0" "PIMAX_CAM_AE_QUANTIZE=0"
    "PIMAX_CAM_AE_EXTEND=1" "PIMAX_CAM_AE_LAW=old" "PIMAX_CAM_AE_TARGET=nan" "PIMAX_CAM_AE_GAIN_MAX=nan"
    "PIMAX_CAM_AE_EXP_MAX=inf")
set(i 0)
foreach(kv ${ae_overrides})
    math(EXPR i "${i} + 1")
    ae_diff(ae_synth_crystal_env${i} --env ${kv} -- synth @W@ crystal-super ${i}0 30)
endforeach()
ae_diff(ae_synth_crystal_envmix --env PIMAX_CAM_AE_QUANTIZE=0 --env PIMAX_CAM_AE_UPPER=0 --env PIMAX_CAM_AE_K=0.9
        --env PIMAX_CAM_AE_DEADBAND=0.01 --env PIMAX_CAM_AE_EXP_MAX=12000 --env PIMAX_CAM_AE_GAIN_MAX=1000
        --env PIMAX_CAM_AE_LAW=0 -- synth @W@ crystal-super 77 40)
ae_diff(ae_synth_crystal_detect --env PIMAX_CAM_AE_MODEL=crystal -- synth @W@ - 78 30)
ae_diff(ae_synth_dream_env1 --env PIMAX_CAM_AE_EXTEND=0 --env PIMAX_CAM_AE_EXP_MIN=100 -- synth @W@ dream-air 81 30)
ae_diff(ae_synth_dream_env2 --env PIMAX_CAM_AE_LAW=old --env PIMAX_CAM_AE_QUANTIZE=0 -- synth @W@ dream-air 82 30)
set_tests_properties(${ae_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900 ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
