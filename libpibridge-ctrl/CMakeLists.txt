# SPDX-License-Identifier: MIT
# libpibridge-ctrl: the Pimax controllers without hardware access (docs/research/rust-port-plan.md, M6 and M7).
#   hands.cmake  (M6): controller-driven hands, the optical + IMU fuser and the in-hand detector, ported from
#                omarchyvr pimax-hands (namespace pibridge::ctrl, target pibridge_ctrl / pibridge_ctrl_obj; the C ABI
#                pimax_hands.h / pimax_ctrlfuse.h / pimax_inhand.h is in monado-abi).
#   track.cmake  (M7): the LED constellation tracker (blobs, LED models, P3P, joint solve, assignment, pose file),
#                ported from omarchyvr pimax-ctrltrack (namespace pibridge::ctrl::track, target pibridge_ctrltrack /
#                pibridge_ctrltrack_obj); the tool pibridge-ctrltrack is in tools/ctrltrack.
# Each half can be left out with PIBRIDGE_SKIP_PARTS (libpibridge-ctrl/hands, libpibridge-ctrl/track). Both need
# libpibridge-cam (PIBRIDGE_CAM_PARITY_FLAGS, rustfmt.h, geom).

if(NOT "libpibridge-ctrl/hands" IN_LIST PIBRIDGE_SKIP_PARTS)
    include(${CMAKE_CURRENT_SOURCE_DIR}/hands.cmake OPTIONAL)
endif()
if(NOT "libpibridge-ctrl/track" IN_LIST PIBRIDGE_SKIP_PARTS)
    include(${CMAKE_CURRENT_SOURCE_DIR}/track.cmake OPTIONAL)
endif()
