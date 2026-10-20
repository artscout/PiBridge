# SPDX-License-Identifier: MIT
# libpibridge-eyemath: gaze math without I/O (gaze samples, device-to-head conversion, calibration, correction, filter, drift,
# device clock). Ported from omarchyvr pimax-eye.

add_library(pibridge_eyemath STATIC
    src/gaze.cpp
    src/track.cpp
    src/correction.cpp
    src/calib.cpp
    src/filter.cpp
    src/drift.cpp
    src/clock.cpp
    src/num.cpp
)
target_include_directories(pibridge_eyemath
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(pibridge_eyemath PUBLIC cxx_std_23)
target_link_libraries(pibridge_eyemath PRIVATE pibridge_common) # pibridge_env.h

if(PIBRIDGE_BUILD_TESTS)
    foreach(t gaze track correction calib filter drift clock rng num)
        add_executable(pibridge_eyemath_test_${t} tests/test_${t}.cpp)
        # the tests also use the private helpers (the small TLV writer, the number format)
        target_include_directories(pibridge_eyemath_test_${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(pibridge_eyemath_test_${t} PRIVATE pibridge_eyemath pibridge_common)
        add_test(NAME pibridge_eyemath_${t} COMMAND pibridge_eyemath_test_${t})
    endforeach()
endif()

