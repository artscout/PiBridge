# SPDX-License-Identifier: MIT
# The hands half of libpibridge-ctrl (M6): controller-driven hands (mapping, filter, pose, touch, rest, report, demo),
# the optical + IMU fuser and the in-hand detector, ported from omarchyvr pimax-hands (MIT OR Apache-2.0). The C ABI
# (pimax_hands.h, pimax_ctrlfuse.h, pimax_inhand.h) is in monado-abi (libpibridge_pimax.a). Included from CMakeLists.txt.

add_library(pibridge_ctrl_obj OBJECT
    src/rust.cpp
    src/filter.cpp
    src/mapping.cpp
    src/pose.cpp
    src/touch.cpp
    src/rest.cpp
    src/report.cpp
    src/demo.cpp
    src/inhand.cpp
    src/fuse.cpp
)
target_include_directories(pibridge_ctrl_obj PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# libpibridge-cam's headers (rustfmt.h, io.h): its objects are in the same archive.
target_include_directories(pibridge_ctrl_obj PUBLIC $<TARGET_PROPERTY:pibridge_cam_obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_features(pibridge_ctrl_obj PUBLIC cxx_std_23)
target_compile_options(pibridge_ctrl_obj PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS})

# The library for PiBridge's own tools and tests.
add_library(pibridge_ctrl STATIC $<TARGET_OBJECTS:pibridge_ctrl_obj>)
target_include_directories(pibridge_ctrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_ctrl PUBLIC cxx_std_23)
target_link_libraries(pibridge_ctrl PUBLIC pibridge_cam)

if(PIBRIDGE_BUILD_TESTS)
    # The pimax-hands fixtures stay in the omarchyvr checkout (read only); the tests that need them skip without it.
    set(pibridge_ctrl_fixtures "${PIBRIDGE_OMARCHYVR_DIR}/rust/fixtures")
    find_package(ZLIB REQUIRED)
    foreach(t filter mapping pose touch rest report demo inhand fuse fuse_pibridge hint pimax_button inhand_fixtures properties replay)
        add_executable(pibridge_ctrl_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_ctrl_test_${t} PRIVATE pibridge_ctrl pibridge_common ZLIB::ZLIB)
        if(t STREQUAL "replay" OR t STREQUAL "pimax_button")
            target_link_libraries(pibridge_ctrl_test_${t} PRIVATE pibridge_hmd)
        endif()
        target_compile_options(pibridge_ctrl_test_${t} PRIVATE -ffp-contract=off -fwrapv)
        target_compile_definitions(pibridge_ctrl_test_${t} PRIVATE PIBRIDGE_CTRL_FIXTURES="${pibridge_ctrl_fixtures}")
        add_test(NAME pibridge_ctrl_${t} COMMAND pibridge_ctrl_test_${t})
        set_tests_properties(pibridge_ctrl_${t} PROPERTIES SKIP_RETURN_CODE 77
            ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR};HOME=${CMAKE_CURRENT_BINARY_DIR}/home;XDG_RUNTIME_DIR=${CMAKE_CURRENT_BINARY_DIR}")
    endforeach()
endif()
