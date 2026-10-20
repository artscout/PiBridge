# SPDX-License-Identifier: MIT
# libpibridge-roommap: the persistent room map (docs/libpibridge-roommap.md). Two halves, ported from omarchyvr (MIT OR
# Apache-2.0) at the pinned commit:
#   room (R2): ORB features, stereo keyframes from the factory calibration, gravity-aware 4-DoF PnP, the map file and
#              relocalisation, the session, the synthetic room, the EuRoC PNG reader (omarchyvr-room);
#   roommap (R3): recordings, mapping pipeline, pooled relocalisation, loop closure, refinement, floor, room setup,
#              files, light, export (omarchyvr-roommap).
# Namespace pibridge::roommap, target pibridge_roommap. Needs libpibridge-cam, libpibridge-anchor (and through it libpibridge-ctrl's
# tracker half) and zlib.

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
set(roommap_sources src/orb.cpp src/pngread.cpp src/pnp.cpp src/stereo.cpp src/map.cpp src/session.cpp src/synth.cpp)
foreach(f rec pipeline align loopclose refine floor setup store light exportmap live guide_ipc)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${f}.cpp)
        list(APPEND roommap_sources src/${f}.cpp)
    endif()
endforeach()
# the map viewer page, embedded like the Rust include_str!
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/data/viewer.html" roommap_viewer_hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," roommap_viewer_bytes "${roommap_viewer_hex}")
string(LENGTH "${roommap_viewer_hex}" roommap_viewer_len)
math(EXPR roommap_viewer_len "${roommap_viewer_len} / 2")
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/viewer_html.cpp" CONTENT
"// SPDX-License-Identifier: MIT
// Generated from libpibridge-roommap/data/viewer.html by libpibridge-roommap/CMakeLists.txt.
#include \"pibridge/roommap/exportmap.h\"
namespace pibridge::roommap::exportmap {
namespace { const char k_viewer[] = {@roommap_viewer_bytes@ 0}; }
std::string_view viewer_html() { return std::string_view(k_viewer, @roommap_viewer_len@); }
}
" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/data/viewer.html")
list(APPEND roommap_sources ${CMAKE_CURRENT_BINARY_DIR}/viewer_html.cpp)
add_library(pibridge_roommap_obj OBJECT ${roommap_sources})
target_include_directories(pibridge_roommap_obj PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(pibridge_roommap_obj PUBLIC $<TARGET_PROPERTY:pibridge_anchor_obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_features(pibridge_roommap_obj PUBLIC cxx_std_23)
# -fno-math-errno: the libm calls stay the same (bit-identical values), only errno is not set, so sin/cos/exp of
# a loop-invariant argument can be hoisted as LLVM does for the Rust code.
target_compile_options(pibridge_roommap_obj PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS} -fno-math-errno)
target_link_libraries(pibridge_roommap_obj PUBLIC ZLIB::ZLIB)

add_library(pibridge_roommap STATIC $<TARGET_OBJECTS:pibridge_roommap_obj>)
target_include_directories(pibridge_roommap PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_roommap PUBLIC cxx_std_23)
target_link_libraries(pibridge_roommap PUBLIC pibridge_anchor ZLIB::ZLIB Threads::Threads)

if(PIBRIDGE_BUILD_TESTS)
    # The Dream Air reference calibration (devices/dream-air/reference): the Rust tests use the lab copy
    # (analysis/headset-calibration.xml) or the default search; the twins use the reference, which is public.
    add_executable(pibridge_roommap_probe tests/room_probe.cpp)
    target_link_libraries(pibridge_roommap_probe PRIVATE pibridge_roommap)
    target_compile_options(pibridge_roommap_probe PRIVATE -ffp-contract=off)
    # the synthetic room through the public reference calibration: ORB, features, stereo against the oracle
    set(roommap_ref ${PIBRIDGE_SOURCE_DEVICES_DIR}/dream-air/reference/headset-calibration.xml)
    add_test(NAME pibridge_roommap_probe_room COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/cmp_sha.sh
        $<TARGET_FILE:pibridge_roommap_probe> ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/room_probe.sha256 room ${roommap_ref})
    add_test(NAME pibridge_roommap_probe_trace COMMAND ${PROJECT_SOURCE_DIR}/libpibridge-anchor/tests/cmp_golden.sh
        $<TARGET_FILE:pibridge_roommap_probe> ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/trace_probe.txt.gz trace)
    set_tests_properties(pibridge_roommap_probe_room pibridge_roommap_probe_trace PROPERTIES
        ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
    foreach(t orb pnp stereo room pngread roommap guide)
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_${t}.cpp)
            add_executable(pibridge_roommap_test_${t} tests/test_${t}.cpp)
            target_link_libraries(pibridge_roommap_test_${t} PRIVATE pibridge_roommap pibridge_common)
            target_compile_options(pibridge_roommap_test_${t} PRIVATE -ffp-contract=off)
            target_compile_definitions(pibridge_roommap_test_${t} PRIVATE
                PIBRIDGE_REFERENCE_XML="${PIBRIDGE_SOURCE_DEVICES_DIR}/dream-air/reference/headset-calibration.xml")
            add_test(NAME pibridge_roommap_${t} COMMAND pibridge_roommap_test_${t})
            set_tests_properties(pibridge_roommap_${t} PROPERTIES SKIP_RETURN_CODE 77 ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
        endif()
    endforeach()
endif()
