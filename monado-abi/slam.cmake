# SPDX-License-Identifier: MIT
# The SLAM / 6DoF difftests (M4): slam_difftest.c against libpibridge_pimax.a and against the Rust libpimax_cam.a,
# compared by run_difftest.sh. Included from CMakeLists.txt inside PIBRIDGE_BUILD_TESTS (uses d, run, w, ref, crystal,
# pibridge_difftest_c_opts from there).
set(PIBRIDGE_LAB_SLAMREC_DIR "${PROJECT_SOURCE_DIR}/../ext/slamrec" CACHE PATH
    "Recorded EuRoC datasets of the Crystal Super (private lab data, read only)")
set(PIBRIDGE_LAB_SLAM_DIR "${PROJECT_SOURCE_DIR}/../ext/slam" CACHE PATH
    "SLAM study folder with Basalt trajectories of the recordings (private lab data, read only)")
set(PIBRIDGE_LAB_RECORDINGS_DIR "$ENV{HOME}/.local/share/pibridge/recordings" CACHE PATH
    "PiBridge RECORD TRACKING takes (private lab data, read only)")

add_executable(slam_difftest_pibridge ${d}/slam_difftest.c)
target_link_libraries(slam_difftest_pibridge PRIVATE pibridge_pimax m)
set_target_properties(slam_difftest_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
target_compile_options(slam_difftest_pibridge PRIVATE ${pibridge_difftest_c_opts} -ffp-contract=off -Wno-format-truncation)
if(EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a")
    add_executable(slam_difftest_rust ${d}/slam_difftest.c)
    target_include_directories(slam_difftest_rust PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(slam_difftest_rust PRIVATE "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a" pthread dl m rt gcc_s)
    set_target_properties(slam_difftest_rust PROPERTIES C_STANDARD 11)
    target_compile_options(slam_difftest_rust PRIVATE ${pibridge_difftest_c_opts} -ffp-contract=off -Wno-format-truncation)
    set(slam_r $<TARGET_FILE:slam_difftest_rust>)
else()
    set(slam_r "${CMAKE_CURRENT_BINARY_DIR}/no-rust-oracle")
endif()
set(slam_f $<TARGET_FILE:slam_difftest_pibridge>)

set(slam_tests)
macro(slam_diff name)
    add_test(NAME monado_abi_diff_${name} COMMAND ${run} ${name} ${w} ${slam_r} ${slam_f} ${ARGN})
    list(APPEND slam_tests monado_abi_diff_${name})
endmacro()

slam_diff(slam_layout -- layout)
slam_diff(slam_reference --home -- slam ${ref} @W@)
slam_diff(slam_crystal --home --need ${crystal} -- slam ${crystal} @W@ private)
slam_diff(slam_imu -- imu @W@)
slam_diff(slam_errors -- errors)
slam_diff(slam_imulog -- imulog @W@)
slam_diff(slam_synth -- synth)
# Each switch the output layer reads (HeadParams::from_env), alone or in groups.
slam_diff(slam_synth_raw -- -e PIMAX_SLAM_SMOOTH=0 -e PIMAX_SLAM_RESNAP=0 synth)
slam_diff(slam_synth_nodrift -- -e PIMAX_SLAM_DRIFT_HOLD=0 -e PIMAX_SLAM_FREEZE=0 synth)
slam_diff(slam_synth_nofuse -- -e PIMAX_SLAM_FUSE=off -e PIMAX_HEAD_TILT=0 -e PIMAX_SLAM_TAU_REST_MS=0 synth)
slam_diff(slam_synth_fuse -- -e "PIMAX_SLAM_FUSE= 0.3, 0.1,0.05 ,0.7,2" -e PIMAX_FUSE_AUTO=0 -e PIMAX_HEAD_TILT=400
          -e PIMAX_SLAM_TAU_MS=40 -e PIMAX_SLAM_BIG_CORR_MM=5 synth)
slam_diff(slam_synth_still -- -e PIMAX_SLAM_STILL=5 -e PIMAX_SLAM_STILL_REZERO=0 -e PIMAX_SLAM_TAU_REST_MS=9000
          -e PIMAX_HEAD_TILT=-1 synth)
slam_diff(slam_synth_nostill -- -e PIMAX_SLAM_STILL=0 -e "PIMAX_SLAM_FUSE=1.5,0,0,0,0" -e PIMAX_SLAM_TAU_MS=x
          -e PIMAX_SLAM_BIG_CORR_MM=1e9 synth)

# Recorded IMU + recorded Basalt trajectory (lab data): a worn take with auto exposure on (study3) and the desk /
# dark take of 05:27 (study2), each with the default switches and with the drift hold off.
set(take_a ${PIBRIDGE_LAB_RECORDINGS_DIR}/20261019-070409/euroc_20261019070409)
set(traj_a ${PIBRIDGE_LAB_SLAM_DIR}/study3/runs/L/out/tracking.csv)
set(take_b ${PIBRIDGE_LAB_SLAMREC_DIR}/euroc_20261019052737)
set(traj_b ${PIBRIDGE_LAB_SLAM_DIR}/study2/runs/L/out/tracking.csv)
slam_diff(slam_replay_worn --need ${take_a}/mav0/imu0/data.csv --need ${traj_a} -- replay ${take_a} ${traj_a})
slam_diff(slam_replay_worn_nodrift --need ${take_a}/mav0/imu0/data.csv --need ${traj_a}
          -- -e PIMAX_SLAM_DRIFT_HOLD=0 replay ${take_a} ${traj_a})
slam_diff(slam_replay_desk --need ${take_b}/mav0/imu0/data.csv --need ${traj_b} -- replay ${take_b} ${traj_b})
slam_diff(slam_replay_desk_rezero0 --need ${take_b}/mav0/imu0/data.csv --need ${traj_b}
          -- -e PIMAX_SLAM_STILL_REZERO=0 -e PIMAX_SLAM_FREEZE=0 replay ${take_b} ${traj_b})

set_tests_properties(${slam_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
# The difftests compare against the Rust oracle: PiBridge's 6DoF improvements (seeded gyro bias, accel bias, drift hold off,
# the recorder's calibration in use; docs/research/head-tracking-improvements.md) are off in parity mode.
set_property(TEST ${slam_tests} APPEND PROPERTY ENVIRONMENT "PIBRIDGE_HEAD_PARITY=1")

# Twins of the capi_slam.rs tests through the archive.
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
