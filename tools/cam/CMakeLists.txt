# SPDX-License-Identifier: MIT
# pibridge-cam: the tracking-camera tool (port of omarchyvr's `pimax-cam`: info, stats, record, calib, import, broker,
# broker-status). Its output is the Rust tool's text; the broker speaks PXCB v1 with pimax-cam's clients and servers.

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
add_executable(pibridge-cam src/main.cpp)
target_link_libraries(pibridge-cam PRIVATE pibridge_cam)
target_compile_options(pibridge-cam PRIVATE -ffp-contract=off)
install(TARGETS pibridge-cam RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    # Tool parity against the Rust pimax-cam (the oracle, see monado-abi/difftest/README) and the broker cross tests
    # (Rust broker <-> PiBridge client, PiBridge broker <-> Rust client) on a recorded .pxcam. Skipped (exit 77) without the
    # oracle or the lab recording.
    set(pibridge_cam_rust_bin "${PIBRIDGE_RUST_ORACLE_DIR}/pimax-cam")
    set(pibridge_cam_first "${PIBRIDGE_LAB_CAM_DIR}/first.pxcam")
    set(pibridge_cam_runner ${PROJECT_SOURCE_DIR}/monado-abi/difftest/run_difftest.sh)
    set(pibridge_cam_work ${CMAKE_CURRENT_BINARY_DIR}/difftest)
    add_test(NAME pibridge_cam_tool_calib_reference COMMAND ${pibridge_cam_runner} tool-calib-ref ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> -- calib ${PIBRIDGE_CAM_REFERENCE_XML})
    add_test(NAME pibridge_cam_tool_calib_conv1 COMMAND ${pibridge_cam_runner} tool-calib-conv1 ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> -- calib ${PIBRIDGE_CAM_REFERENCE_XML} --conv 1)
    add_test(NAME pibridge_cam_tool_calib_crystal COMMAND ${pibridge_cam_runner} tool-calib-crystal ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> --need ${PIBRIDGE_LAB_CALIB_DIR}/crystal-calibration.xml
             -- calib ${PIBRIDGE_LAB_CALIB_DIR}/crystal-calibration.xml)
    add_test(NAME pibridge_cam_tool_calib_home COMMAND ${pibridge_cam_runner} tool-calib-home ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> --home -- calib)
    foreach(rec first ctrl1 ctrl2)
        # the latency line depends on the absolute time of the replay (rebased onto the clock at its start): two
        # runs of the Rust tool can differ there by one in the last digit, so it is left out
        add_test(NAME pibridge_cam_tool_stats_${rec} COMMAND ${pibridge_cam_runner} tool-stats-${rec} ${pibridge_cam_work}
                 ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> --need ${PIBRIDGE_LAB_CAM_DIR}/${rec}.pxcam
                 --ignore "arrival latency above minimum"
                 -- stats --source ${PIBRIDGE_LAB_CAM_DIR}/${rec}.pxcam --seconds 1000)
    endforeach()
    set(pibridge_cam_dump $<TARGET_FILE:cam_difftest_pibridge>)
    add_test(NAME pibridge_cam_tool_record COMMAND ${pibridge_cam_runner} tool-record ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> --need ${pibridge_cam_first} --ignore "arrival latency"
             --post "${pibridge_cam_dump} pxcam @W@/out.pxcam"
             -- record @W@/out.pxcam --source ${pibridge_cam_first} --frames 20 --seconds 1000)
    add_test(NAME pibridge_cam_tool_import COMMAND ${pibridge_cam_runner} tool-import ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> --need ${pibridge_cam_first}
             --pre "${pibridge_cam_dump} extract ${pibridge_cam_first} 5 @W@/in.raw"
             --post "${pibridge_cam_dump} pxcam @W@/out.pxcam"
             -- import @W@/in.raw @W@/out.pxcam --zlib 0)
    add_test(NAME pibridge_cam_tool_errors COMMAND ${pibridge_cam_runner} tool-errors ${pibridge_cam_work}
             ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam> --exit 1 -- stats --source /nonexistent.pxcam)
    add_test(NAME pibridge_cam_broker_cross COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/broker_cross.sh
             ${CMAKE_CURRENT_BINARY_DIR}/broker-cross ${pibridge_cam_rust_bin} $<TARGET_FILE:pibridge-cam>
             ${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a $<TARGET_FILE:cam_difftest_pibridge>
             "$<$<TARGET_EXISTS:cam_difftest_rust>:$<TARGET_FILE:cam_difftest_rust>>" ${pibridge_cam_first})
    set_tests_properties(pibridge_cam_tool_calib_reference pibridge_cam_tool_calib_conv1 pibridge_cam_tool_calib_crystal
        pibridge_cam_tool_calib_home pibridge_cam_tool_stats_first pibridge_cam_tool_stats_ctrl1 pibridge_cam_tool_stats_ctrl2
        pibridge_cam_tool_record pibridge_cam_tool_import pibridge_cam_tool_errors pibridge_cam_broker_cross
        PROPERTIES SKIP_RETURN_CODE 77)
    set_tests_properties(pibridge_cam_broker_cross PROPERTIES TIMEOUT 300)
endif()
