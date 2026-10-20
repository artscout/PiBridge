# SPDX-License-Identifier: MIT
# pibridge-ctrltrack against `pimax-ctrltrack` of the pinned omarchyvr commit (the oracle, monado-abi/difftest/
# build-oracle.sh): every command run by both tools through tests/ct_run.sh (stderr merged into stdout, run-dependent
# values masked: durations, ages, pids), compared byte for byte by monado-abi/difftest/run_difftest.sh, with the files
# written (CSV, extrinsics, overlay pixels, the `live --trace` with relative times, the pose file as Monado's own
# reader sees it) appended. A scratch HOME for every run (no own models / extrinsics of the user). Lab recordings
# (PIBRIDGE_LAB_CAM_DIR) and the Crystal calibration (by hash) are read only; the tests skip (77) without them or the
# oracle.

set(ct_dir ${CMAKE_CURRENT_SOURCE_DIR}/tests)
set(ct_rust_exe "${PIBRIDGE_RUST_ORACLE_DIR}/pimax-ctrltrack")
set(ct_wrap_rust "${CMAKE_CURRENT_BINARY_DIR}/ct-rust")
set(ct_wrap_pibridge "${CMAKE_CURRENT_BINARY_DIR}/ct-pibridge")
if(EXISTS "${ct_rust_exe}")
    file(GENERATE OUTPUT ${ct_wrap_rust} CONTENT "#!/bin/sh\nexec ${ct_dir}/ct_run.sh ${ct_rust_exe} \"$@\"\n"
         FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endif()
file(GENERATE OUTPUT ${ct_wrap_pibridge}
     CONTENT "#!/bin/sh\nexec ${ct_dir}/ct_run.sh $<TARGET_FILE:pibridge-ctrltrack> \"$@\"\n"
     FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# Monado's reader of the pose file (included from the omarchyvr checkout, not copied).
set(ct_monado_shm_dir "${PIBRIDGE_OMARCHYVR_DIR}/forks/monado/src/xrt/drivers/pimax")
set(ct_shm_dump "${CMAKE_CURRENT_BINARY_DIR}/no-monado-reader")
if(EXISTS "${ct_monado_shm_dir}/pimax_ctrltrack_shm.h")
    add_executable(ctrltrack_shm_dump ${ct_dir}/shm_dump.c)
    target_include_directories(ctrltrack_shm_dump PRIVATE ${ct_monado_shm_dir})
    set_target_properties(ctrltrack_shm_dump PROPERTIES C_STANDARD 11)
    target_compile_options(ctrltrack_shm_dump PRIVATE -Wno-pedantic -Wno-conversion -Wno-sign-conversion)
    set(ct_shm_dump $<TARGET_FILE:ctrltrack_shm_dump>)
endif()
# The overlay pictures, decoded (libpibridge-lens's PNG decoder).
set(ct_png_dump "true")
if(TARGET pibridge_lens)
    add_executable(ctrltrack_png_dump ${ct_dir}/png_dump.cpp)
    target_link_libraries(ctrltrack_png_dump PRIVATE pibridge_lens)
    set(ct_png_dump $<TARGET_FILE:ctrltrack_png_dump>)
endif()

set(run ${PROJECT_SOURCE_DIR}/monado-abi/difftest/run_difftest.sh)
set(w ${CMAKE_CURRENT_BINARY_DIR}/difftest)
set(r ${ct_wrap_rust})
set(f ${ct_wrap_pibridge})
set(cam_dir ${PIBRIDGE_LAB_CAM_DIR})
set(crystal ${PIBRIDGE_LAB_CALIB_DIR}/crystal-calibration.xml)
set(refdir ${PIBRIDGE_SOURCE_DEVICES_DIR}/dream-air/reference)
set(ct_tests)
macro(ct_diff name)
    add_test(NAME pibridge_ctrltrack_diff_${name} COMMAND ${run} ${name} ${w} ${r} ${f} --home ${ARGN})
    list(APPEND ct_tests pibridge_ctrltrack_diff_${name})
endmacro()
set(csv "cat @W@/out.csv")
set(xdg --env XDG_RUNTIME_DIR=@W@)
set(live_post "${ct_dir}/trace_rel.sh @W@/trace && ${ct_shm_dump} @W@/pose")

# track: the three recordings with the defaults; ctrl1 with every option
foreach(rec first ctrl1 ctrl2)
    set(px ${cam_dir}/${rec}.pxcam)
    ct_diff(track_${rec} --need ${px} --post "${csv}" -- track ${px} --csv @W@/out.csv)
    ct_diff(scan_${rec} --need ${px} -- scan ${px})
endforeach()
set(px1 ${cam_dir}/ctrl1.pxcam)
set(px2 ${cam_dir}/ctrl2.pxcam)
ct_diff(track_ctrl1_factory --need ${px1} --post "${csv}" -- track ${px1} --factory --csv @W@/out.csv)
ct_diff(track_ctrl2_factory --need ${px2} --post "${csv}" -- track ${px2} --factory --csv @W@/out.csv)
ct_diff(track_ctrl1_window --need ${px1} --post "${csv}"
        -- track ${px1} --no-prior --start 100 --count 150 --every 2 --csv @W@/out.csv)
ct_diff(track_ctrl1_prior_window --need ${px1} --post "${csv}"
        -- track ${px1} --start 37 --count 61 --every 3 --csv @W@/out.csv)
# swapped models: the right model as left and the other way round (the user's own files given on the command line)
ct_diff(track_ctrl1_models --need ${px1} --post "${csv}"
        -- track ${px1} --left ${refdir}/controllers/right.json --right ${refdir}/controllers/left.json
        --count 120 --csv @W@/out.csv)
# the camera extrinsics refit, saved and read back
ct_diff(track_ctrl1_fit --need ${px1} --post "cat @W@/ext.json && ${csv}"
        -- track ${px1} --fit-extrinsics --save-extrinsics @W@/sub/ext.json --csv @W@/out.csv)
ct_diff(track_ctrl2_fit_factory --need ${px2} --post "cat @W@/sub/ext.json"
        -- track ${px2} --factory --fit-extrinsics --save-extrinsics @W@/sub/ext.json --count 200)
ct_diff(track_ctrl1_fit_badsave --need ${px1} --pre "printf x > @W@/file"
        -- track ${px1} --fit-extrinsics --save-extrinsics @W@/file/ext.json --count 100)
ct_diff(track_ctrl1_extrinsics --need ${px1} --post "${csv}"
        --pre "sed 's/\"id\": 2/\"id\": 0/' ${refdir}/ctrl-cam-extrinsics.json > @W@/ext.json"
        -- track ${px1} --extrinsics @W@/ext.json --count 120 --csv @W@/out.csv)
ct_diff(track_ctrl1_extrinsics_other --need ${px1}
        --pre "sed 's/\"device_uid\": \"reference\"/\"device_uid\": \"OTHER\"/' ${refdir}/ctrl-cam-extrinsics.json > @W@/ext.json"
        -- track ${px1} --extrinsics @W@/ext.json --count 30)
ct_diff(track_ctrl1_extrinsics_broken --need ${px1} --pre "printf '{\"format\": 1' > @W@/ext.json"
        -- track ${px1} --extrinsics @W@/ext.json --count 30)
ct_diff(track_ctrl1_extrinsics_home --need ${px1}
        --pre "mkdir -p @W@/home/.config/omarchyvr && printf '{\"format\": \"pimax-ctrltrack-extrinsics\", \"cameras\": [{\"id\": 3, \"cam_from_body_r\": [1,0,0,0,1,0,0,0,1], \"cam_from_body_t\": [0,0,0]}, {\"id\": 9, \"cam_from_body_r\": [1,0,0,0,1,0,0,0,1], \"cam_from_body_t\": [0,0,0.1]}]}' > @W@/home/.config/omarchyvr/ctrl-cam-extrinsics.json"
        -- track ${px1} --count 30)
ct_diff(track_ctrl1_overlay --need ${px1} --post "${ct_png_dump} @W@/ov"
        -- track ${px1} --count 101 --overlay @W@/ov --overlay-every 50)
# the Crystal Super's own calibration (lab data, compared by hash): the Dream Air recording through the Crystal rig
ct_diff(track_ctrl1_crystal --hash --need ${px1} --need ${crystal} --post "${csv}"
        -- track ${px1} --calib ${crystal} --factory --count 200 --csv @W@/out.csv)
# the extrinsics refit with real multi-camera fixes (the Crystal rig): rounds, saved file, corrections
ct_diff(track_ctrl1_crystal_fit --hash --need ${px1} --need ${crystal} --post "cat @W@/ext.json && ${csv}"
        -- track ${px1} --calib ${crystal} --factory --fit-extrinsics --save-extrinsics @W@/ext.json --csv @W@/out.csv)
ct_diff(track_ctrl2_crystal_overlay --hash --need ${px2} --need ${crystal} --post "${ct_png_dump} @W@/ov"
        -- track ${px2} --calib ${crystal} --count 120 --overlay @W@/ov --overlay-every 40)
ct_diff(live_ctrl2_crystal --hash --need ${px2} --need ${crystal} ${xdg} --post "${live_post}"
        -- live --source ${px2} --calib ${crystal} --factory --out @W@/pose --stats 0 --trace @W@/trace)
ct_diff(track_ctrl1_crystal_ext --hash --need ${px1} --need ${crystal}
        -- track ${px1} --calib ${crystal} --count 50)
# own models in the scratch HOME (the Pimax per-serial layout, the Hand field deciding), the model directory variable
ct_diff(track_ctrl1_home_models --need ${px1}
        --pre "mkdir -p @W@/home/.config/omarchyvr/controllers/A @W@/home/.config/omarchyvr/controllers/B && cp ${refdir}/controllers/right.json @W@/home/.config/omarchyvr/controllers/A/controller_params.json && cp ${refdir}/controllers/left.json @W@/home/.config/omarchyvr/controllers/B/controller_params.json"
        -- track ${px1} --count 40)
ct_diff(track_ctrl1_model_dir --need ${px1} --env PIMAX_CTRL_MODEL_DIR=@W@/m
        --pre "mkdir -p @W@/m && cp ${refdir}/controllers/left.json @W@/m/left.json && cp ${refdir}/controllers/right.json @W@/m/right.json"
        -- track ${px1} --count 40)
# errors (exit codes and messages)
ct_diff(err_usage --exit 2 --)
ct_diff(err_command --exit 2 -- frobnicate)
ct_diff(err_track_noargs --exit 101 -- track)
ct_diff(err_track_missing --exit 101 -- track @W@/missing.pxcam)
ct_diff(err_track_notpxcam --exit 101 --pre "printf 'not a recording' > @W@/x.pxcam" -- track @W@/x.pxcam)
ct_diff(err_track_start --exit 101 -- track x --start abc)
ct_diff(err_track_count --exit 101 -- track x --count -1)
ct_diff(err_track_every_empty --exit 101 -- track x --every "")
ct_diff(err_track_every_zero --need ${px1} --exit 101 -- track ${px1} --every 0)
ct_diff(err_track_calib --exit 2 -- track x --calib @W@/missing.xml)
ct_diff(err_track_left --exit 2 -- track x --left @W@/missing.json)
ct_diff(err_track_right_broken --exit 2 --pre "printf '{\"Hand\": \"Right\", \"ModelPoints\": {\"Point0\": [1,2,3]}}' > @W@/r.json"
        -- track x --right @W@/r.json)
ct_diff(err_track_few_leds --exit 2 --pre "printf '{\"ModelPoints\": {\"Point0\": [1,2,3,0,0,1]}}' > @W@/r.json"
        -- track x --right @W@/r.json)
ct_diff(err_track_noref --exit 2 --env PIMAX_NO_REFERENCE=1 -- track x)
ct_diff(err_track_noref_models --exit 2 --env PIMAX_NO_REFERENCE=1 --env PIMAX_CAM_CALIB=${refdir}/headset-calibration.xml
        -- track x)
ct_diff(err_scan_noargs --exit 101 -- scan)
ct_diff(err_scan_every --exit 101 -- scan x --every)
ct_diff(err_live_seconds --exit 101 -- live --seconds x)
ct_diff(err_live_source --exit 1 --env XDG_RUNTIME_DIR=@W@ -- live --source @W@/missing.pxcam --out @W@/pose)
ct_diff(err_live_out --need ${cam_dir}/first.pxcam --exit 1 -- live --source ${cam_dir}/first.pxcam --out @W@/no/pose)
ct_diff(err_show_missing --exit 1 -- show --out @W@/missing.pose)
ct_diff(err_show_short --exit 1 --pre "printf 'PXCT' > @W@/pose" -- show --out @W@/pose)

# live: the replays at their recorded pace, stats after every LED frame, the trace and the pose file
foreach(rec first ctrl1 ctrl2)
    set(px ${cam_dir}/${rec}.pxcam)
    ct_diff(live_${rec} --need ${px} ${xdg} --post "${live_post}"
            -- live --source ${px} --out @W@/pose --stats 0 --trace @W@/trace)
    # as the session runs it (factory extrinsics)
    ct_diff(live_${rec}_factory --need ${px} ${xdg} --post "${live_post}"
            -- live --source ${px} --out @W@/pose --stats 0 --trace @W@/trace --factory)
endforeach()
ct_diff(live_ctrl1_baseline --need ${px1} ${xdg} --post "${live_post}"
        -- live --source ${px1} --out @W@/pose --stats 0 --trace @W@/trace --baseline --no-body)
ct_diff(live_ctrl2_options --need ${px2} ${xdg} --env PIMAX_CTRLTRACK_AIM_PITCH_DEG=30.5 --post "${live_post}"
        -- live --source ${px2} --out @W@/pose --stats 0 --trace @W@/trace --thr 70 --peak 140 --no-body)
ct_diff(live_ctrl1_pitch --need ${px1} ${xdg} --post "${live_post}"
        -- live --source ${px1} --stats 0 --trace @W@/trace --aim-pitch-deg -12.25 --out @W@/pose)
# Monado's in-hand status: the left controller lies on the desk (writer pid 1 is always alive), then both
ct_diff(live_ctrl1_inhand_left --need ${px1} ${xdg} --post "${live_post}"
        --pre "printf 'pid=1\\nmode=auto\\nleft=down\\nright=held\\n' > @W@/pimax-ctrl-inhand"
        -- live --source ${px1} --out @W@/pose --stats 0 --trace @W@/trace)
ct_diff(live_first_inhand_both --need ${cam_dir}/first.pxcam ${xdg} --post "${live_post}"
        --pre "printf 'pid=1\\nleft=off\\nright=down\\n' > @W@/pimax-ctrl-inhand"
        -- live --source ${cam_dir}/first.pxcam --out @W@/pose --stats 0 --trace @W@/trace)
ct_diff(live_ctrl1_crystal --hash --need ${px1} --need ${crystal} ${xdg} --post "${live_post}"
        -- live --source ${px1} --calib ${crystal} --factory --out @W@/pose --stats 0 --trace @W@/trace)

# show / health on pose files written by each tool (cross: each tool reads the file the other wrote)
foreach(writer rust pibridge)
    if(writer STREQUAL "rust")
        set(wexe "${ct_rust_exe}")
    else()
        set(wexe "$<TARGET_FILE:pibridge-ctrltrack>")
    endif()
    set(first ${cam_dir}/first.pxcam)
    set(mk "XDG_RUNTIME_DIR=@W@ HOME=@W@/home ${wexe} live --source ${first} --out @W@/pose")
    ct_diff(show_from_${writer} --need ${first} --need ${wexe} --pre "${mk}" -- show --out @W@/pose --seconds 0.1)
    ct_diff(health_from_${writer} --need ${first} --need ${wexe} --pre "${mk}" -- health --out @W@/pose --max-age 1e9)
    ct_diff(health_stale_from_${writer} --exit 2 --need ${first} --need ${wexe} --pre "${mk}"
            -- health --out @W@/pose --max-age 0)
endforeach()
ct_diff(health_missing --exit 1 -- health --out @W@/missing.pose)
ct_diff(health_old_writer --exit 1 --pre "${ct_dir}/old_pose.sh @W@/pose"
        -- health --out @W@/pose)
ct_diff(health_default_path --exit 1 ${xdg} -- health)
ct_diff(show_old_writer --pre "${ct_dir}/old_pose.sh @W@/pose"
        -- show --out @W@/pose --seconds 0.1)

set_tests_properties(${ct_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
# byte-for-byte against the Rust tool: PiBridge's tracker improvements off (docs/research/ctrl-tracking-improvements.md)
set_tests_properties(${ct_tests} PROPERTIES ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")

# the analysis commands that stay in Rust: a plain message and exit 2
foreach(cmd lockrate fusereplay imufit occlusion dump)
    add_test(NAME pibridge_ctrltrack_not_ported_${cmd} COMMAND ${CMAKE_COMMAND} -E env
             sh -c "$<TARGET_FILE:pibridge-ctrltrack> ${cmd} x 2>&1; echo exit $?")
    set_tests_properties(pibridge_ctrltrack_not_ported_${cmd} PROPERTIES PASS_REGULAR_EXPRESSION
        "is not ported to PiBridge.*use the Rust pimax-ctrltrack ${cmd}.*exit 2")
endforeach()

# live over a camera broker, as the session runs it (`live --source broker:<socket> --factory`): a pibridge-cam broker
# serving a recording to each tool in turn
if(TARGET pibridge-cam)
    add_test(NAME pibridge_ctrltrack_live_broker COMMAND ${ct_dir}/live_broker.sh ${CMAKE_CURRENT_BINARY_DIR}/broker
             $<TARGET_FILE:pibridge-cam> ${ct_rust_exe} $<TARGET_FILE:pibridge-ctrltrack> ${cam_dir}/ctrl1.pxcam
             ${ct_shm_dump})
    set_tests_properties(pibridge_ctrltrack_live_broker PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300
                         ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
endif()

# The recorded controller takes ~/gamedev/ext/cam/ctrl{1,2}.pxcam are Crystal Super recordings: their reference CSVs
# (ctrl{1,2}.csv, lab data) are exactly what `track --calib <the Crystal's XML>` writes. pibridge-ctrltrack must write
# the same bytes (no oracle needed).
foreach(rec ctrl1 ctrl2)
    add_test(NAME pibridge_ctrltrack_csv_${rec} COMMAND ${ct_dir}/csv_golden.sh ${CMAKE_CURRENT_BINARY_DIR}/csv-${rec}
             $<TARGET_FILE:pibridge-ctrltrack> ${cam_dir}/${rec}.pxcam ${crystal} ${cam_dir}/${rec}.csv)
    set_tests_properties(pibridge_ctrltrack_csv_${rec} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600
                         ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
endforeach()
