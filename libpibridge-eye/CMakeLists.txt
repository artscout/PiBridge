# SPDX-License-Identifier: MIT
# libpibridge-eye: the Tobii EyeChip protocol (usbfs transport, TTP, TLV, session, frame lists + guard + profile, power
# policy, gaze shared memory, fake chip, pcap reader, replay plan) and the live tracker (src/live.cpp), which runs
# libpibridge-eyemath's gaze math on the chip's stream; hence the public link to eyemath. See docs/libpibridge-eye.md. The
# Tobii chip's own calibration (src/calibration.cpp, docs/eyecal.md) is the one place calibration opcodes may go out.

# The public base frame list is compiled in (frames::base_list()), from the same file the tests read (data/, not
# tests/: it is a shipped input of the library).
set(PIBRIDGE_EYE_BASE_FRAMES ${CMAKE_CURRENT_SOURCE_DIR}/data/base-frames.txt)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PIBRIDGE_EYE_BASE_FRAMES})
file(READ ${PIBRIDGE_EYE_BASE_FRAMES} PIBRIDGE_EYE_BASE_HEX HEX)
string(LENGTH "${PIBRIDGE_EYE_BASE_HEX}" PIBRIDGE_EYE_BASE_HEX_LEN)
math(EXPR PIBRIDGE_EYE_BASE_SIZE "${PIBRIDGE_EYE_BASE_HEX_LEN} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," PIBRIDGE_EYE_BASE_BYTES "${PIBRIDGE_EYE_BASE_HEX}")
configure_file(src/base_frames.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/base_frames.cpp @ONLY)

# The nine captured Tobii calibration target requests (calibration::targets()), compiled in the same way.
set(PIBRIDGE_EYE_CAL_TARGETS ${CMAKE_CURRENT_SOURCE_DIR}/data/tobii-calibration-targets.txt)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PIBRIDGE_EYE_CAL_TARGETS})
file(READ ${PIBRIDGE_EYE_CAL_TARGETS} PIBRIDGE_EYE_CAL_TARGET_HEX HEX)
string(LENGTH "${PIBRIDGE_EYE_CAL_TARGET_HEX}" PIBRIDGE_EYE_CAL_TARGET_HEX_LEN)
math(EXPR PIBRIDGE_EYE_CAL_TARGET_SIZE "${PIBRIDGE_EYE_CAL_TARGET_HEX_LEN} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," PIBRIDGE_EYE_CAL_TARGET_BYTES "${PIBRIDGE_EYE_CAL_TARGET_HEX}")
configure_file(src/calibration_targets.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/calibration_targets.cpp @ONLY)

add_library(pibridge_eye STATIC
    src/error.cpp
    src/tlv.cpp
    src/ttp.cpp
    src/pcap.cpp
    src/replay.cpp
    src/frames.cpp
    src/session.cpp
    src/usbfs.cpp
    src/power.cpp
    src/shm.cpp
    src/fake.cpp
    src/analysis.cpp
    src/live.cpp
    src/calibration.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/base_frames.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/calibration_targets.cpp
)
target_include_directories(pibridge_eye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(pibridge_eye PUBLIC cxx_std_23)
target_link_libraries(pibridge_eye PRIVATE pibridge_common) # pibridge_env.h
target_link_libraries(pibridge_eye PUBLIC pibridge_eyemath)

if(PIBRIDGE_BUILD_TESTS)
    foreach(t tlv ttp pcap replay frames session usbfs power shm fake analysis live calibration)
        add_executable(pibridge_eye_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_eye_test_${t} PRIVATE pibridge_eye pibridge_common)
        target_compile_definitions(pibridge_eye_test_${t} PRIVATE PIBRIDGE_EYE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
        add_test(NAME pibridge_eye_${t} COMMAND pibridge_eye_test_${t})
    endforeach()
endif()
