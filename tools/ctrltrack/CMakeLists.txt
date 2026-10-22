# SPDX-License-Identifier: MIT
# pibridge-ctrltrack: the Pimax controller LED tracker (port of omarchyvr's `pimax-ctrltrack`: live, track, scan, show,
# health). Options, output and exit codes are the Rust tool's; the pose file is byte-compatible with Monado's reader.

find_package(ZLIB REQUIRED)
add_executable(pibridge-ctrltrack src/main.cpp src/track.cpp src/live.cpp)
target_link_libraries(pibridge-ctrltrack PRIVATE pibridge_ctrltrack ZLIB::ZLIB)
target_compile_options(pibridge-ctrltrack PRIVATE -ffp-contract=off -fwrapv)
install(TARGETS pibridge-ctrltrack RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    include(${CMAKE_CURRENT_SOURCE_DIR}/tests/tests.cmake)
endif()

# pibridge-ctrlbench: PiBridge's offline measurement harness of the tracker and the fuser (not installed; no Rust twin).
if(TARGET pibridge_ctrl)
    add_executable(pibridge-ctrlbench src/bench.cpp src/bench_fuse.cpp)
    target_link_libraries(pibridge-ctrlbench PRIVATE pibridge_ctrltrack pibridge_ctrl)
    if(PIBRIDGE_BUILD_TESTS)
        # the real-IMU-log replay on a synthetic log written by the Rust fuser: every logged verdict comes back
        add_test(NAME pibridge_ctrlbench_imu_log COMMAND pibridge-ctrlbench --selftest-imu-log ${CMAKE_CURRENT_BINARY_DIR})
        set_tests_properties(pibridge_ctrlbench_imu_log PROPERTIES PASS_REGULAR_EXPRESSION "588 of 588 verdicts as logged"
                             ENVIRONMENT "PIBRIDGE_CTRL_HINT=0")
    endif()
endif()
if(PIBRIDGE_BUILD_TESTS)
    # the controller take's guided steps (ctrl-take-steps.json) parse like the app's rec_steps
    add_test(NAME pibridge_ctrltrack_take_steps
             COMMAND ${CMAKE_COMMAND} -DFILE=${CMAKE_CURRENT_SOURCE_DIR}/ctrl-take-steps.json
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_steps.cmake)
endif()
