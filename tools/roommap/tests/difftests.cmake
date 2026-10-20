# SPDX-License-Identifier: MIT
# The room-map tools against omarchyvr's (the oracle: monado-abi/difftest/build-oracle.sh at the pinned commit)
# through tests/rm_run.sh (stderr merged, run-dependent values masked) and monado-abi/difftest/run_difftest.sh
# (stdout + the files written, byte for byte). A scratch HOME and XDG_RUNTIME_DIR for every run. The tests skip (77)
# without the oracle or the lab data they name.

set(rm_dir ${CMAKE_CURRENT_SOURCE_DIR}/tests)
set(run ${PROJECT_SOURCE_DIR}/monado-abi/difftest/run_difftest.sh)
set(w ${CMAKE_CURRENT_BINARY_DIR}/difftest)
set(rm_tests)
function(rm_wrapper var tool exe)
    set(p "${CMAKE_CURRENT_BINARY_DIR}/${tool}")
    file(GENERATE OUTPUT ${p} CONTENT "#!/bin/sh\nexec ${rm_dir}/rm_run.sh ${exe} \"$@\"\n"
         FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
    set(${var} ${p} PARENT_SCOPE)
endfunction()
rm_wrapper(anchor_r anchor-rust "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-anchor")
rm_wrapper(anchor_f anchor-pibridge $<TARGET_FILE:pibridge-anchor>)
if(NOT EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-anchor")
    set(anchor_r "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-anchor")
endif()
set(ringgen $<TARGET_FILE:roommap_ringgen>)
set(rt --env XDG_RUNTIME_DIR=@W@ --env PIMAX_ANCHOR_CONFIG=@W@/anchor.toml)

macro(anchor_diff name)
    add_test(NAME pibridge_anchor_diff_${name} COMMAND ${run} ${name} ${w} ${anchor_r} ${anchor_f} --home ${rt} ${ARGN})
    list(APPEND rm_tests pibridge_anchor_diff_${name})
endmacro()

set(cfg_ok "printf '[tag]\\nid = 3\\nsize_m = 0.152\\n[reference]\\npos = [0.31, -0.72, -0.45]\\nquat = [0.1, 0.7, -0.05, 0.7]\\nsaved = \"unix 1792000000\"\\n[[window]]\\nname = \"term\"\\npos = [0, 0.2, 0]\\nquat = [0, 0, 0, 1]\\n' > @W@/anchor.toml")
set(off_ok "printf 'pimax-anchor-offset 1\\npid 4242\\npose 0.25 -1.4 0.6 0 0.3428978 0 0.9393727\\n' > @W@/pimax-anchor.offset")
anchor_diff(status_empty -- status)
anchor_diff(status_config --pre "${cfg_ok}" -- status)
anchor_diff(status_config_bad --pre "printf '[tag]\\nid = x\\n' > @W@/anchor.toml" -- status)
anchor_diff(status_offset --pre "${off_ok}" -- status)
anchor_diff(status_offset_bad --pre "printf 'pimax-anchor-offset 1\\npid 4242\\npose 1 2 3\\n' > @W@/pimax-anchor.offset" -- status)
# rings: an old one of a closed writer, an empty one, a fresh one of this machine's "Monado" (none runs: no Monado)
anchor_diff(status_ring_old --pre "${cfg_ok} && ${off_ok} && ${ringgen} @W@/pimax-anchor.head 999999999 1000 50" -- status)
anchor_diff(status_ring_empty --pre "${ringgen} @W@/pimax-anchor.head 4242 0 0" -- status)
anchor_diff(status_ring_fresh --pre "${ringgen} @W@/pimax-anchor.head 1 0.5 200" -- status)
anchor_diff(status_ring_short --pre "printf 'PANC' > @W@/pimax-anchor.head" -- status)
anchor_diff(status_ring_env --env PIMAX_ANCHOR_HEAD_FILE=@W@/h --env PIMAX_ANCHOR_FILE=@W@/o
            --pre "${ringgen} @W@/h 77 30 3 && printf 'pimax-anchor-offset 1\\npid 77\\npose 0 0 0 0 0 0 1\\n' > @W@/o" -- status)
anchor_diff(clear --pre "${off_ok}" --post "ls @W@" -- clear)
anchor_diff(clear_none --post "ls @W@" -- clear)
anchor_diff(clear_dir --pre "mkdir @W@/pimax-anchor.offset" --post "ls @W@" -- clear)


# ---------------------------------------------------------------------------------------------------------------
# pibridge-room = omarchyvr-room. Lab data: the Crystal Super recordings (EuRoC) with their Basalt trajectories and the
# Crystal calibration, read only; the outputs that carry the headset's own values (calibration uid, map files) are
# compared by hash (--hash).
set(PIBRIDGE_LAB_SLAMREC_DIR "${PROJECT_SOURCE_DIR}/../ext/slamrec" CACHE PATH "EuRoC recordings (private lab data, read only)")
set(PIBRIDGE_LAB_SLAM_DIR "${PROJECT_SOURCE_DIR}/../ext/slam" CACHE PATH "Basalt runs of the recordings (private lab data)")
set(PIBRIDGE_LAB_RECORDINGS_DIR "$ENV{HOME}/.local/share/pibridge/recordings" CACHE PATH "PiBridge RECORD TRACKING takes (private)")
rm_wrapper(room_r room-rust "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-room")
rm_wrapper(room_f room-pibridge $<TARGET_FILE:pibridge-room>)
if(NOT EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-room")
    set(room_r "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-room")
endif()
set(room_oracle "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-room")
set(crystal ${PIBRIDGE_LAB_CALIB_DIR}/crystal-calibration.xml)
set(desk ${PIBRIDGE_LAB_SLAMREC_DIR}/euroc_20261019052737)
set(desk_tr ${PIBRIDGE_LAB_SLAM_DIR}/study2/runs/L/out/tracking.csv)
set(worn ${PIBRIDGE_LAB_RECORDINGS_DIR}/20261019-070409)
set(worn_tr ${PIBRIDGE_LAB_SLAM_DIR}/study3/runs/L/out/tracking.csv)
set(refxml ${PIBRIDGE_SOURCE_DEVICES_DIR}/dream-air/reference/headset-calibration.xml)
set(mapdump "sha256sum < @W@/m/map.bin && cat @W@/m/room.json")
macro(room_diff name)
    add_test(NAME pibridge_room_diff_${name} COMMAND ${run} ${name} ${w} ${room_r} ${room_f} --home ${ARGN})
    list(APPEND rm_tests pibridge_room_diff_${name})
endmacro()
# the synthetic two-session demo through the reference calibration (the session-1 time is masked)
room_diff(demo --env PIMAX_CAM_CALIB=${refxml} -- demo)
# build: the desk take and the worn take (every 3rd / 2nd frame set)
room_diff(build_desk --need ${desk} --need ${crystal} --hash --post "${mapdump}"
          -- build ${desk} ${desk_tr} @W@/m --calib ${crystal} --every 3)
room_diff(build_worn --need ${worn} --need ${crystal} --hash --post "${mapdump}"
          -- build ${worn} ${worn_tr} @W@/m --calib ${crystal} --every 2)
# reloc: the desk take in a map of the worn take (made by the oracle in --pre), and the worn take in its own map,
# --save (the grown map)
set(pre_worn_map "${room_oracle} build ${worn} ${worn_tr} @W@/m --calib ${crystal} --every 4")
room_diff(reloc_desk_in_worn --need ${worn} --need ${desk} --need ${crystal} --hash --pre "${pre_worn_map}"
          -- reloc @W@/m ${desk} ${desk_tr} --calib ${crystal} --every 6)
room_diff(reloc_worn_self_save --need ${worn} --need ${crystal} --hash --pre "${pre_worn_map}" --post "${mapdump}"
          -- reloc @W@/m ${worn} ${worn_tr} --calib ${crystal} --every 3 --save)
# info and anchors on that map
room_diff(info --need ${worn} --need ${crystal} --hash --pre "${pre_worn_map}" -- info @W@/m)
room_diff(anchor_set --need ${worn} --need ${crystal} --hash --pre "${pre_worn_map}" --post "${mapdump}"
          -- anchor @W@/m set desk 0.5 0.75 -0.4 33.5)
room_diff(anchor_set3 --need ${worn} --need ${crystal} --hash --pre "${pre_worn_map}" --post "${mapdump}"
          -- anchor @W@/m set "a b\"c" 1 2 3)
room_diff(anchor_list --need ${worn} --need ${crystal} --pre "${pre_worn_map} && ${room_oracle} anchor @W@/m set k 1 2 3 90 && ${room_oracle} anchor @W@/m set j -1 0 2"
          -- anchor @W@/m list)
room_diff(anchor_rm --need ${worn} --need ${crystal} --hash --pre "${pre_worn_map} && ${room_oracle} anchor @W@/m set k 1 2 3 90"
          --post "${mapdump}" -- anchor @W@/m rm k)
# errors (exit 1; the tool name in the message is masked)
room_diff(err_unknown --exit 1 -- frobnicate)
room_diff(err_missing --exit 1 -- build)
room_diff(err_nomap --exit 1 -- info @W@/none)
room_diff(err_badmap --exit 1 --pre "printf PXRM > @W@/map.bin" -- info @W@)
room_diff(err_badmap2 --exit 1 --pre "printf UFhSTQIAAAA= | base64 -d > @W@/map.bin" -- info @W@)
room_diff(err_anchor_rm --exit 1 --need ${worn} --need ${crystal} --pre "${pre_worn_map}" -- anchor @W@/m rm nothere)
room_diff(err_anchor_what --exit 1 --need ${worn} --need ${crystal} --pre "${pre_worn_map}" -- anchor @W@/m show)
room_diff(err_anchor_num --exit 1 --need ${worn} --need ${crystal} --pre "${pre_worn_map}" -- anchor @W@/m set x 1 y 3)
room_diff(err_novalue --exit 1 -- build a b c --every)
room_diff(err_notrack --exit 1 --need ${desk} --need ${crystal} --hash -- build ${desk} @W@/none.csv @W@/m --calib ${crystal})
room_diff(err_nodataset --exit 1 --need ${crystal} --hash --pre "mkdir -p @W@/d/x" -- build @W@/d ${desk_tr} @W@/m --calib ${crystal})

# ---------------------------------------------------------------------------------------------------------------
# pibridge-roommap = omarchyvr-roommap (offline paths on the Crystal takes; the live paths need a broker and a head
# ring). Files written are appended; their wall-clock fields (roommap.toml created, floor.toml when) are masked.
rm_wrapper(rmap_r rmap-rust "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-roommap")
rm_wrapper(rmap_f rmap-pibridge $<TARGET_FILE:pibridge-roommap>)
if(NOT EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-roommap")
    set(rmap_r "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-roommap")
endif()
set(rmap_oracle "${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-roommap")
set(files "${rm_dir}/dump_map.sh @W@/m")
set(rmenv --env XDG_RUNTIME_DIR=@W@ --env PIMAX_ROOMMAP_DIR=@W@/maps)
macro(rmap_diff name)
    add_test(NAME pibridge_roommap_diff_${name} COMMAND ${run} ${name} ${w} ${rmap_r} ${rmap_f} --home ${rmenv} ${ARGN})
    list(APPEND rm_tests pibridge_roommap_diff_${name})
endmacro()
set(lab --need ${worn} --need ${crystal} --hash)
set(rec_dir "mkdir -p @W@/rec/replay-1 && ln -s ${worn}/euroc_20261019070409 @W@/rec/ && ln -s ${worn_tr} @W@/rec/replay-1/tracking.csv")
set(pre_map "${rmap_oracle} build --rec ${worn} --tracking ${worn_tr} --map @W@/m --calib ${crystal} --every 2 > /dev/null 2>&1")
rmap_diff(build_worn ${lab} --post "${files}" -- build --rec ${worn} --tracking ${worn_tr} --map @W@/m --calib ${crystal})
rmap_diff(build_worn_refine ${lab} --post "${files}"
          -- build --rec ${worn} --tracking ${worn_tr} --map @W@/m --calib ${crystal} --every 2 --refine)
rmap_diff(build_desk_noloops --need ${desk} --need ${crystal} --hash --post "${files}"
          -- build --rec ${desk} --tracking ${desk_tr} --map @W@/m --calib ${crystal} --every 3 --no-loops)
rmap_diff(reloc_worn_self ${lab} --pre "${pre_map}" --post "${files}"
          -- relocalize --rec ${worn} --tracking ${worn_tr} --map @W@/m --calib ${crystal} --every 3)
rmap_diff(reloc_desk_in_worn ${lab} --need ${desk} --exit 1 --pre "${pre_map}"
          -- relocalize --rec ${desk} --tracking ${desk_tr} --map @W@/m --calib ${crystal} --every 6)
rmap_diff(floor_detect ${lab} --pre "${pre_map}" --post "${files}" -- floor detect --map @W@/m --save)
rmap_diff(floor_detect_rec ${lab} --pre "${pre_map} && ${rec_dir}" -- floor detect --rec @W@/rec --map @W@/m --calib ${crystal} --every 6)
rmap_diff(floor_drift ${lab} --pre "${rec_dir}" --post "cat @W@/d.csv" -- floor drift --rec @W@/rec --calib ${crystal} --every 4 --csv @W@/d.csv)
rmap_diff(export ${lab} --pre "${pre_map}" --post "${files}" -- export --map @W@/m)
rmap_diff(view ${lab} --pre "${pre_map}" --post "${files}" -- view --map @W@/m)
rmap_diff(info ${lab} --pre "${pre_map} && sed -i -E 's/unix [0-9]+/unix T/' @W@/m/roommap.toml" -- info --map @W@/m)
rmap_diff(floor_set ${lab} --pre "${pre_map}" --post "${files}" -- floor set --map @W@/m --y -1.25)
rmap_diff(apply_dry ${lab} --pre "${pre_map} && printf 'map = \"x\"\\nsource = \"s\"\\nyaw_deg = 12.5\\nt = [0.1, -0.2, 0.3]\\npid = 0\\n' > @W@/m/last-reloc.toml"
          -- apply --map @W@/m --dry-run)
rmap_diff(apply_write ${lab} --pre "${pre_map} && printf 'map = \"x\"\\nsource = \"s\"\\nyaw_deg = 12.5\\nt = [0.1, -0.2, 0.3]\\npid = 77\\n' > @W@/m/last-reloc.toml"
          --post "cat @W@/pimax-anchor.offset && sed -E 's/^writer [0-9]+/writer W/' @W@/omarchyvr-anchor.offset.owner"
          -- apply --map @W@/m)
rmap_diff(cycle --pre "printf 'yaw_deg = 10\\nt = [1, 2, 3]\\n' > @W@/a.toml && printf 'yaw_deg = -10\\nt = [-1, -1.5, -2.5]\\n' > @W@/b.toml"
          -- cycle @W@/a.toml @W@/b.toml)
rmap_diff(err_unknown --exit 1 -- nope)
rmap_diff(err_nomap --exit 1 -- info --map @W@/none)
rmap_diff(err_build --exit 1 --need ${crystal} --hash -- build --map @W@/m --calib ${crystal})
rmap_diff(err_badnum --exit 1 --need ${crystal} --hash -- relocalize --map @W@/m --calib ${crystal} --window x)
rmap_diff(err_extra --exit 1 -- info --map @W@/m extra)
rmap_diff(err_apply --exit 1 --pre "mkdir -p @W@/m" -- apply --map @W@/m)

# live-side commands that need no running Monado: light on a recorded .pxcam, setup show, auto without a map or
# without Monado (status file, its time masked)
set(px1 ${PIBRIDGE_LAB_CAM_DIR}/ctrl1.pxcam)
set(status_post "sed -E 's/^when = [0-9]+/when = T/' @W@/omarchyvr-room.status")
rmap_diff(light --need ${px1} --need ${crystal} --hash -- light --source ${px1} --calib ${crystal} --seconds 999)
rmap_diff(light_dark --need ${px1} --need ${crystal} --hash
          -- light --source ${px1} --calib ${crystal} --seconds 999 --every 5 --darken 0.15 --noise 3)
rmap_diff(light_nostretch --need ${px1} --need ${crystal} --hash
          -- light --source ${px1} --calib ${crystal} --seconds 999 --every 7 --no-stretch)
rmap_diff(setup_show --pre "mkdir -p @W@/m && printf 'label = \"Living\"\\nmode = \"full\"\\ncentre = [0.25, -1.5]\\nyaw_deg = 28.6\\nplay_area = [1.8, 2.6]\\nhead_m = 1.65\\nwhen = 1\\nboundary = [[0, 0], [2, 0], [2, 3], [0, 3]]\\n' > @W@/m/room-setup.toml"
          -- setup show --map @W@/m)
rmap_diff(auto_nomap --post "${status_post}" -- auto --map @W@/m)
rmap_diff(auto_nomonado --pre "${pre_map}" --need ${worn} --need ${crystal} --post "${status_post}" -- auto --map @W@/m --wait 0)
rmap_diff(setup_nofeed --exit 1 -- setup fast --map @W@/m --no-hud)

set_tests_properties(${rm_tests} PROPERTIES SKIP_RETURN_CODE 77)
