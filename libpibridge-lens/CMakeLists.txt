# SPDX-License-Identifier: MIT
# libpibridge-lens: the lens model of the Monado Pimax driver (lens.conf parser, per-channel radial polynomial, the LUT
# of the Crystal Super, visibility mask meshes, the PIMAX_LENS_DEBUG_GRID pictures). Ported from omarchyvr pimax-lens
# (MIT OR Apache-2.0). No Qt: Monado links it through monado-abi (libpibridge_pimax.a, pimax_lens_rs.h).

find_package(ZLIB REQUIRED)

set(PIBRIDGE_LAB_LENS_DIR "${PROJECT_SOURCE_DIR}/../ext/lens" CACHE PATH
    "Lens configurations and LUTs of the lab headsets (crystal.conf, crystal-lut.bin; private, read only)")

# The Dream Air reference lens profile (devices/dream-air/reference/lens.conf), embedded like the Rust include_str!.
set(PIBRIDGE_LENS_REFERENCE_CONF "${PIBRIDGE_SOURCE_DEVICES_DIR}/dream-air/reference/lens.conf" CACHE INTERNAL
    "the embedded reference lens profile")
file(READ "${PIBRIDGE_LENS_REFERENCE_CONF}" pibridge_lens_ref_hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," pibridge_lens_ref_bytes "${pibridge_lens_ref_hex}")
string(LENGTH "${pibridge_lens_ref_hex}" pibridge_lens_ref_len)
math(EXPR pibridge_lens_ref_len "${pibridge_lens_ref_len} / 2")
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/reference_lens_conf.cpp" CONTENT
"// SPDX-License-Identifier: MIT
// Generated from devices/dream-air/reference/lens.conf by libpibridge-lens/CMakeLists.txt.
#include \"pibridge/lens/lens.h\"
namespace pibridge::lens {
namespace { const char k_ref[] = {@pibridge_lens_ref_bytes@ 0}; }
std::string_view reference_lens_conf() { return std::string_view(k_ref, @pibridge_lens_ref_len@); }
}
" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${PIBRIDGE_LENS_REFERENCE_CONF}")

add_library(pibridge_lens_obj OBJECT
    src/fmt.cpp
    src/config.cpp
    src/png.cpp
    src/font.cpp
    src/lut.cpp
    src/pvrdump.cpp
    src/model.cpp
    src/mask.cpp
    src/grid.cpp
    src/lens.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/reference_lens_conf.cpp
)
target_include_directories(pibridge_lens_obj PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# libpibridge-cam's headers (rustfmt.h, io.h, json.h): its objects are in the same archive.
target_include_directories(pibridge_lens_obj PUBLIC $<TARGET_PROPERTY:pibridge_cam_obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_features(pibridge_lens_obj PUBLIC cxx_std_23)
target_compile_options(pibridge_lens_obj PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS})
target_link_libraries(pibridge_lens_obj PUBLIC ZLIB::ZLIB)

# The library for PiBridge's own tools and tests.
add_library(pibridge_lens STATIC $<TARGET_OBJECTS:pibridge_lens_obj>)
target_include_directories(pibridge_lens PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_lens PUBLIC cxx_std_23)
target_link_libraries(pibridge_lens PUBLIC pibridge_cam ZLIB::ZLIB)

if(PIBRIDGE_BUILD_TESTS)
    set(pibridge_lens_fixture "${PIBRIDGE_OMARCHYVR_DIR}/rust/pimax-lens/tests/data/pvr-dump-sub.json")
    foreach(t config model lut mask png font grid lens)
        add_executable(pibridge_lens_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_lens_test_${t} PRIVATE pibridge_lens pibridge_common)
        target_compile_options(pibridge_lens_test_${t} PRIVATE -ffp-contract=off)
        target_compile_definitions(pibridge_lens_test_${t} PRIVATE
            PIBRIDGE_LENS_DUMP_FIXTURE="${pibridge_lens_fixture}")
        add_test(NAME pibridge_lens_${t} COMMAND pibridge_lens_test_${t})
        set_tests_properties(pibridge_lens_${t} PROPERTIES SKIP_RETURN_CODE 77
            ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
    endforeach()
endif()
