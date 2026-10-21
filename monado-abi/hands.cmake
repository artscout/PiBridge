# SPDX-License-Identifier: MIT
# The controller part of monado-abi (M6): pimax_hands.h, pimax_ctrlfuse.h, pimax_inhand.h. The difftest
# (difftest/hands_difftest.c) against libpibridge_pimax.a and against omarchyvr's libpimax_hands.a (the Rust oracle, built
# by difftest/build-oracle.sh), and the twins of the Rust ffi / c_abi tests through the archive. Included from
# CMakeLists.txt inside PIBRIDGE_BUILD_TESTS when libpibridge-ctrl's hands half is built; uses its variables (d, run, w, ...).

find_package(ZLIB REQUIRED)

add_executable(monado_abi_test_hands_capi tests/test_hands_capi.cpp)
target_link_libraries(monado_abi_test_hands_capi PRIVATE pibridge_pimax pibridge_common)
target_compile_definitions(monado_abi_test_hands_capi PRIVATE
    PIBRIDGE_HANDS_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/pimax_hands.h")
add_test(NAME monado_abi_hands_capi COMMAND monado_abi_test_hands_capi)
set_tests_properties(monado_abi_hands_capi PROPERTIES
    ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR};HOME=${CMAKE_CURRENT_BINARY_DIR}/home;XDG_RUNTIME_DIR=${CMAKE_CURRENT_BINARY_DIR}")

add_executable(hands_difftest_pibridge ${d}/hands_difftest.c)
target_link_libraries(hands_difftest_pibridge PRIVATE pibridge_pimax ZLIB::ZLIB m)
set_target_properties(hands_difftest_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
target_compile_options(hands_difftest_pibridge PRIVATE ${pibridge_difftest_c_opts} -Wno-format-truncation -ffp-contract=off)
target_compile_definitions(hands_difftest_pibridge PRIVATE _GNU_SOURCE)
if(EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_hands.a")
    add_executable(hands_difftest_rust ${d}/hands_difftest.c)
    target_include_directories(hands_difftest_rust PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(hands_difftest_rust PRIVATE "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_hands.a" ZLIB::ZLIB pthread dl
                          m rt gcc_s)
    set_target_properties(hands_difftest_rust PROPERTIES C_STANDARD 11)
    target_compile_options(hands_difftest_rust PRIVATE ${pibridge_difftest_c_opts} -Wno-format-truncation -ffp-contract=off)
    target_compile_definitions(hands_difftest_rust PRIVATE _GNU_SOURCE)
    set(pibridge_hands_difftest_rust $<TARGET_FILE:hands_difftest_rust>)
else()
    set(pibridge_hands_difftest_rust "${CMAKE_CURRENT_BINARY_DIR}/no-rust-oracle")
endif()

set(hf $<TARGET_FILE:hands_difftest_pibridge>)
set(hr ${pibridge_hands_difftest_rust})
set(fx ${PIBRIDGE_OMARCHYVR_DIR}/rust/fixtures)
# Every run gets a scratch HOME and XDG_RUNTIME_DIR: the calibration file, the status and request files land there,
# never in the user's.
set(sandbox --home --env XDG_RUNTIME_DIR=@W@)
# The files the fuser writes: the calibration (on destroy, every 2000 IMU samples) and the IMU logs.
set(fdump "${d}/hands_dump.sh @W@")
set(hands_diff_tests)
macro(hands_diff name)
    add_test(NAME monado_abi_diff_hands_${name} COMMAND ${run} hands-${name} ${w} ${hr} ${hf} ${sandbox} ${ARGN})
    list(APPEND hands_diff_tests monado_abi_diff_hands_${name})
endmacro()

hands_diff(layout -- layout)
hands_diff(session -- session)
foreach(seed 1 2 3 4)
    hands_diff(synth_${seed} -- hands ${seed} 60000)
endforeach()
# the placement switches (read once per process)
hands_diff(poses -- poses)
hands_diff(poses_legacy_head --env PIMAX_CTRL_REST=legacy --env PIMAX_CTRL_AIM=head:2 -- poses)
hands_diff(poses_numbers --env "PIMAX_CTRL_REST= 0.1, 0.2,0.3,-10 " --env PIMAX_CTRL_AIM=head -- poses)
hands_diff(poses_three --env PIMAX_CTRL_REST=0.3,0.1,0.5 --env "PIMAX_CTRL_AIM=head: 7.25" -- poses)
hands_diff(poses_bad --env PIMAX_CTRL_REST=nope --env PIMAX_CTRL_AIM=sideways -- poses)
hands_diff(poses_bad_range --env PIMAX_CTRL_REST=0.1,0.2,9 --env PIMAX_CTRL_AIM=head:0.01 -- poses)
hands_diff(poses_bad_count --env PIMAX_CTRL_REST=1,2,3,4,5 --env PIMAX_CTRL_AIM=level -- poses)
hands_diff(poses_bad_nan --env PIMAX_CTRL_REST=0.1,nan,0.3 --env PIMAX_CTRL_AIM=head:x -- poses)
hands_diff(poses_pitch --env PIMAX_CTRL_REST=0,0,0,400 --env PIMAX_CTRL_AIM=head:inf -- poses)
hands_diff(poses_names --env PIMAX_CTRL_REST=default --env PIMAX_CTRL_AIM=level -- poses)
hands_diff(poses_two --env PIMAX_CTRL_REST=0.1,0.2 --env PIMAX_CTRL_AIM=head: -- poses)
hands_diff(poses_empty --env PIMAX_CTRL_REST= --env PIMAX_CTRL_AIM= -- poses)
hands_diff(poses_round1 --env PIMAX_CTRL_REST=0.125,0.135,0.145,0.5 --env PIMAX_CTRL_AIM=head:2.675 -- poses)
hands_diff(poses_round2 --env PIMAX_CTRL_REST=0.1,-0.005,1e-3,-0.5 --env PIMAX_CTRL_AIM=head:0.205 -- poses)
hands_diff(poses_round3 --env PIMAX_CTRL_REST=-1,1.5,-0.5,2.5 --env PIMAX_CTRL_AIM=head:20 -- poses)
hands_diff(poses_spaces --env "PIMAX_CTRL_REST=1,2, x y" -- poses)
# recorded controller reports (the pimax-hands replay fixtures, read only)
foreach(rec ctrl_buttons_hub2 p82 p83)
    hands_diff(replay_${rec} --need ${fx}/${rec}.rec.gz --post "${fdump}" -- replay ${fx}/${rec}.rec.gz)
endforeach()
hands_diff(replay_hub2_log --need ${fx}/ctrl_buttons_hub2.rec.gz --env PIMAX_CTRL_TRACK_IMU=0
           --env PIMAX_CTRL_IMU_LOG=@W@/log --env PIMAX_CTRL_IMU_LOG_MIN=0.05 --post "${fdump}"
           -- replay ${fx}/ctrl_buttons_hub2.rec.gz)
# the in-hand fixtures of the session 2026-10-16-1818 (inhand_fixtures.rs), through pimax_inhand_report
foreach(cut desk desk_bump pickup held putdown)
    hands_diff(inhand_${cut} --need ${fx}/inhand_${cut}.csv.gz -- inhand_csv ${fx}/inhand_${cut}.csv.gz free)
    hands_diff(inhand_${cut}_picked --need ${fx}/inhand_${cut}.csv.gz -- inhand_csv ${fx}/inhand_${cut}.csv.gz picked)
endforeach()
hands_diff(inhand_desk_typing --need ${fx}/inhand_desk.csv.gz -- inhand_csv ${fx}/inhand_desk.csv.gz typing)
hands_diff(inhand_held_half --need ${fx}/inhand_held.csv.gz -- inhand_csv ${fx}/inhand_held.csv.gz half)
hands_diff(inhand_desk_down2 --need ${fx}/inhand_desk.csv.gz --env PIMAX_CTRL_INHAND_DOWN_S=2
           -- inhand_csv ${fx}/inhand_desk.csv.gz picked)
hands_diff(inhand_putdown_down13 --need ${fx}/inhand_putdown.csv.gz --env "PIMAX_CTRL_INHAND_DOWN_S= 1.3 "
           -- inhand_csv ${fx}/inhand_putdown.csv.gz picked)
# synthetic in-hand sessions: modes, requests, the status file, the down-time switch
hands_diff(inhand_synth_1 -- inhand_synth 1 @W@)
hands_diff(inhand_synth_2 --env PIMAX_CTRL_INHAND=always -- inhand_synth 2 @W@)
hands_diff(inhand_synth_3 --env "PIMAX_CTRL_INHAND= OFF " --env PIMAX_CTRL_INHAND_DOWN_S=1 -- inhand_synth 3 @W@)
hands_diff(inhand_synth_4 --env PIMAX_CTRL_INHAND=when_held --env PIMAX_CTRL_INHAND_DOWN_S=60 -- inhand_synth 4 @W@)
hands_diff(inhand_synth_5 --env PIMAX_CTRL_INHAND=bogus --env PIMAX_CTRL_INHAND_DOWN_S=0.5 -- inhand_synth 5 @W@)
hands_diff(inhand_synth_6 --env PIMAX_CTRL_INHAND=never --env PIMAX_CTRL_INHAND_DOWN_S=7.7 -- inhand_synth 6 @W@)
hands_diff(inhand_synth_7 --env PIMAX_CTRL_INHAND_DOWN_S=x -- inhand_synth 7 @W@)
# the fuser: synthetic optical + IMU sessions with every switch, the calibration file and the IMU log
hands_diff(fuse_nulls --post "${fdump}" -- fuse_nulls)
foreach(sc 0 1 2 3 4 5 6)
    hands_diff(fuse_${sc} --post "${fdump}" -- fuse ${sc} ${sc}7 90)
endforeach()
hands_diff(fuse_long --post "${fdump}" -- fuse 0 11 400)
hands_diff(fuse_fit --post "${fdump}" -- fuse 6 31 300)
hands_diff(fuse_fit_file --env PIMAX_CTRL_IMU_CALIB=@W@/calib.json --pre "cp ${d}/hands_calib_ok.json @W@/calib.json"
           --post "${fdump}" -- fuse 6 32 300)
hands_diff(fuse_fit_nofit --env PIMAX_CTRL_IMU_FIT=0 --post "${fdump}" -- fuse 6 33 200)
hands_diff(fuse_fit_scratch --env PIMAX_CTRL_IMU_DEFAULT=0 --env PIMAX_CTRL_TRACK_IMU=1 --post "${fdump}" -- fuse 6 34 200)
hands_diff(fuse_log --env PIMAX_CTRL_IMU_LOG=@W@/log --post "${fdump}" -- fuse 0 5 60)
hands_diff(fuse_log_short --env PIMAX_CTRL_IMU_LOG=@W@/log --env PIMAX_CTRL_IMU_LOG_MIN=0.2 --post "${fdump}"
           --pre "touch @W@/omarchyvr-imu-log.on" -- fuse 4 6 40)
hands_diff(fuse_log_nan --env PIMAX_CTRL_IMU_LOG=@W@/log --env PIMAX_CTRL_IMU_LOG_MIN=NaN --post "${fdump}" -- fuse 1 7 20)
hands_diff(fuse_noimu --env PIMAX_CTRL_TRACK_IMU=0 --post "${fdump}" -- fuse 0 8 90)
hands_diff(fuse_forceimu --env PIMAX_CTRL_TRACK_IMU=on --env PIMAX_CTRL_IMU_DEFAULT=0 --post "${fdump}" -- fuse 0 9 90)
hands_diff(fuse_nodefault --env "PIMAX_CTRL_IMU_DEFAULT= 0" --post "${fdump}" -- fuse 0 10 120)
hands_diff(fuse_legacy --env PIMAX_CTRL_TRACK_HYST=off --post "${fdump}" -- fuse 0 12 90)
hands_diff(fuse_switches1 --env PIMAX_CTRL_TRACK_LOST_S=1.5 --env PIMAX_CTRL_TRACK_EXTRAP_MS=40
           --env PIMAX_CTRL_TRACK_BLEND_MS=300 --env PIMAX_CTRL_IMU_FIT=0 --env PIMAX_CTRL_TRACK_FILTER=0
           --post "${fdump}" -- fuse 0 13 90)
hands_diff(fuse_switches2 --env PIMAX_CTRL_TRACK_STILL_LOCK=0 --env PIMAX_CTRL_TRACK_FOLLOW_YAW=1
           --env PIMAX_CTRL_TRACK_BLEND_MPS=0 --env PIMAX_CTRL_TRACK_FOREARM=0 --post "${fdump}" -- fuse 1 14 90)
hands_diff(fuse_switches3 --env PIMAX_CTRL_TRACK_DR_MS=300 --env "PIMAX_CTRL_TRACK_FOREARM=[0.02, 0.1, -0.2]"
           --env PIMAX_CTRL_TRACK_BLEND_MPS=0.2 --env PIMAX_CTRL_TRACK_FOLLOW_YAW=0.25 --post "${fdump}" -- fuse 0 15 90)
hands_diff(fuse_switches4 --env PIMAX_CTRL_TRACK_DR_MS=80 --env PIMAX_CTRL_TRACK_LOST_S=-1
           --env PIMAX_CTRL_TRACK_EXTRAP_MS=-5 --env PIMAX_CTRL_TRACK_FOLLOW_YAW=2 --env PIMAX_CTRL_TRACK_FOREARM=1,2
           --env PIMAX_CTRL_TRACK_IMU=maybe --env PIMAX_CTRL_TRACK_HYST=1 --post "${fdump}" -- fuse 4 16 90)
hands_diff(fuse_dr_desk --env PIMAX_CTRL_TRACK_DR_MS=500 --env PIMAX_CTRL_TRACK_STILL_LOCK=0 --post "${fdump}" -- fuse 1 17 90)
# a calibration file: both hands, one hand, damaged, without the format marker, a directory, odd number forms, NaN
# (the Rust library panics in describe: the process aborts, the C++ does the same), a relative path
foreach(cf ok left damaged noformat unicode)
    hands_diff(fuse_calib_${cf} --env PIMAX_CTRL_IMU_CALIB=@W@/calib.json --pre "cp ${d}/hands_calib_${cf}.json @W@/calib.json"
               --post "${fdump}" -- fuse 0 18 120)
endforeach()
hands_diff(fuse_calib_nan --exit 134 --env PIMAX_CTRL_IMU_CALIB=@W@/calib.json --pre "cp ${d}/hands_calib_nan.json @W@/calib.json"
           -- fuse 0 18 10)
hands_diff(fuse_calib_dir --env PIMAX_CTRL_IMU_CALIB=@W@/calib.json --pre "mkdir -p @W@/calib.json"
           --post "${fdump}" -- fuse 0 22 60)
hands_diff(fuse_home_calib --pre "mkdir -p @W@/home/.config/pibridge && cp ${d}/hands_calib_ok.json @W@/home/.config/pibridge/ctrl-imu.json"
           --post "${fdump}" -- fuse 0 24 120)
set_tests_properties(${hands_diff_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
# the Rust twins compare against the Rust library: PiBridge's fuser improvements off (docs/research/ctrl-tracking-improvements.md)
set_tests_properties(${hands_diff_tests} monado_abi_hands_capi PROPERTIES ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
