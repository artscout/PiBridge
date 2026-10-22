# SPDX-License-Identifier: MIT
# pibridge-ctrltrack's tool tests. Lab recordings (PIBRIDGE_LAB_CAM_DIR) and the Crystal calibration are read only;
# the tests that need them skip (77) without them.

set(ct_dir ${CMAKE_CURRENT_SOURCE_DIR}/tests)
set(cam_dir ${PIBRIDGE_LAB_CAM_DIR})
set(crystal ${PIBRIDGE_LAB_CALIB_DIR}/crystal-calibration.xml)

# Monado's reader of the pose file (third_party/monado's Pimax driver, included, not copied).
set(ct_monado_shm_dir "${PROJECT_SOURCE_DIR}/third_party/monado/src/xrt/drivers/pimax")
set(ct_shm_dump "${CMAKE_CURRENT_BINARY_DIR}/no-monado-reader")
if(EXISTS "${ct_monado_shm_dir}/pimax_ctrltrack_shm.h")
    add_executable(ctrltrack_shm_dump ${ct_dir}/shm_dump.c)
    target_include_directories(ctrltrack_shm_dump PRIVATE ${ct_monado_shm_dir})
    set_target_properties(ctrltrack_shm_dump PROPERTIES C_STANDARD 11)
    target_compile_options(ctrltrack_shm_dump PRIVATE -Wno-pedantic -Wno-conversion -Wno-sign-conversion)
    set(ct_shm_dump $<TARGET_FILE:ctrltrack_shm_dump>)
endif()

# the analysis commands that were not ported: a plain message and exit 2
foreach(cmd lockrate fusereplay imufit occlusion dump)
    add_test(NAME pibridge_ctrltrack_not_ported_${cmd} COMMAND ${CMAKE_COMMAND} -E env
             sh -c "$<TARGET_FILE:pibridge-ctrltrack> ${cmd} x 2>&1; echo exit $?")
    set_tests_properties(pibridge_ctrltrack_not_ported_${cmd} PROPERTIES PASS_REGULAR_EXPRESSION
        "is not ported to PiBridge.*use the Rust pimax-ctrltrack ${cmd}.*exit 2")
endforeach()

# live over a camera broker, as the session runs it (`live --source broker:<socket> --factory`): a pibridge-cam broker
# serving a recording
if(TARGET pibridge-cam)
    add_test(NAME pibridge_ctrltrack_live_broker COMMAND ${ct_dir}/live_broker.sh ${CMAKE_CURRENT_BINARY_DIR}/broker
             $<TARGET_FILE:pibridge-cam> $<TARGET_FILE:pibridge-ctrltrack> ${cam_dir}/ctrl1.pxcam ${ct_shm_dump})
    set_tests_properties(pibridge_ctrltrack_live_broker PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
endif()

# The recorded controller takes ~/gamedev/ext/cam/ctrl{1,2}.pxcam are Crystal Super recordings: their reference CSVs
# (ctrl{1,2}.csv, lab data) are exactly what `track --calib <the Crystal's XML>` writes. pibridge-ctrltrack must write
# the same bytes.
foreach(rec ctrl1 ctrl2)
    add_test(NAME pibridge_ctrltrack_csv_${rec} COMMAND ${ct_dir}/csv_golden.sh ${CMAKE_CURRENT_BINARY_DIR}/csv-${rec}
             $<TARGET_FILE:pibridge-ctrltrack> ${cam_dir}/${rec}.pxcam ${crystal} ${cam_dir}/${rec}.csv)
    set_tests_properties(pibridge_ctrltrack_csv_${rec} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600
                         ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
endforeach()
