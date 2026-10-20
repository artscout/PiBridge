# SPDX-License-Identifier: MIT
# libpibridge-cam: the Pimax tracking cameras (UVC 34a4:0014): frame layout and metadata, V4L2 capture, .pxcam
# recording/replay, the camera broker (PXCB v1), factory calibration (KB4), clock mapping. Ported from omarchyvr
# pimax-cam (MIT OR Apache-2.0). No Qt: Monado links it through monado-abi (libpibridge_pimax.a).

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

# Parity data (read only; the tests that need them are skipped without them). Relative to the source tree, no home
# paths: PiBridge, omarchyvr and the lab data folder (ext) sit next to each other.
set(PIBRIDGE_OMARCHYVR_DIR "${PROJECT_SOURCE_DIR}/../omarchyvr" CACHE PATH
    "omarchyvr checkout (read only): the pinned headers and the Rust oracle are taken from it")
set(PIBRIDGE_RUST_ORACLE_DIR "${PIBRIDGE_OMARCHYVR_DIR}/rust/target-port2/release" CACHE PATH
    "Rust oracle build of the pinned commit (libpimax_cam.a, pimax-cam; monado-abi/difftest/build-oracle.sh)")
set(PIBRIDGE_LAB_CAM_DIR "${PROJECT_SOURCE_DIR}/../ext/cam" CACHE PATH
    "Recorded camera streams (*.pxcam, private lab data, read only)")
set(PIBRIDGE_LAB_CALIB_DIR "${PROJECT_SOURCE_DIR}/../ext/calib" CACHE PATH
    "Factory calibration XMLs (private lab data, read only)")

# The Dream Air reference calibration (devices/dream-air/reference/headset-calibration.xml, serial scrubbed), embedded
# like the Rust include_str!.
set(PIBRIDGE_CAM_REFERENCE_XML "${PIBRIDGE_SOURCE_DEVICES_DIR}/dream-air/reference/headset-calibration.xml" CACHE INTERNAL
    "the embedded reference calibration (tests use it too)")
file(READ "${PIBRIDGE_CAM_REFERENCE_XML}" pibridge_cam_ref_hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," pibridge_cam_ref_bytes "${pibridge_cam_ref_hex}")
string(LENGTH "${pibridge_cam_ref_hex}" pibridge_cam_ref_len)
math(EXPR pibridge_cam_ref_len "${pibridge_cam_ref_len} / 2")
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/reference_xml.cpp" CONTENT
"// SPDX-License-Identifier: MIT
// Generated from devices/dream-air/reference/headset-calibration.xml by libpibridge-cam/CMakeLists.txt.
#include \"pibridge/cam/calib.h\"
namespace pibridge::cam {
namespace { const char k_ref[] = {@pibridge_cam_ref_bytes@ 0}; }
std::string_view reference_xml() { return std::string_view(k_ref, @pibridge_cam_ref_len@); }
}
" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${PIBRIDGE_CAM_REFERENCE_XML}")

# Bit parity with the Rust code: no contraction into fma, no fast-math. Hidden symbols: only the C ABI in
# monado-abi is exported from a shared object that links the archive.
# -fwrapv: signed overflow wraps, as Rust's release build does (no overflow checks).
set(PIBRIDGE_CAM_PARITY_FLAGS -ffp-contract=off -fno-fast-math -fwrapv -fvisibility=hidden -fvisibility-inlines-hidden)
# GCC marks inline variables of libstdc++ (std::__unicode tables, ...) STB_GNU_UNIQUE, which objcopy cannot make
# local; as plain weak symbols they are localised with the rest (monado-abi/CMakeLists.txt).
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-fno-gnu-unique PIBRIDGE_HAVE_NO_GNU_UNIQUE)
if(PIBRIDGE_HAVE_NO_GNU_UNIQUE)
    list(APPEND PIBRIDGE_CAM_PARITY_FLAGS -fno-gnu-unique)
endif()
set(PIBRIDGE_CAM_PARITY_FLAGS "${PIBRIDGE_CAM_PARITY_FLAGS}" CACHE INTERNAL "compile flags of the ported code (monado-abi too)")

add_library(pibridge_cam_obj OBJECT
    src/rustfmt.cpp
    src/io.cpp
    src/frame.cpp
    src/clock.cpp
    src/geom.cpp
    src/kb4.cpp
    src/xml.cpp
    src/calib.cpp
    src/json.cpp
    src/overlay.cpp
    src/source.cpp
    src/record.cpp
    src/v4l2.cpp
    src/resilient.cpp
    src/camera.cpp
    src/stats.cpp
    src/broker.cpp
    src/slam.cpp
    src/drift.cpp
    src/headpose.cpp
    src/imulog.cpp
    src/ae.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/reference_xml.cpp
)
target_include_directories(pibridge_cam_obj PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_cam_obj PUBLIC cxx_std_23)
target_compile_options(pibridge_cam_obj PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS})
target_link_libraries(pibridge_cam_obj PUBLIC ZLIB::ZLIB)

# The library for PiBridge's own tools and tests.
add_library(pibridge_cam STATIC $<TARGET_OBJECTS:pibridge_cam_obj>)
target_include_directories(pibridge_cam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_cam PUBLIC cxx_std_23)
target_link_libraries(pibridge_cam PUBLIC ZLIB::ZLIB Threads::Threads)

if(PIBRIDGE_BUILD_TESTS)
    foreach(t rustfmt frame clock geom kb4 xml calib json overlay record stats camera resilient v4l2 broker
            slam headpose headpibridge drift imulog ae)
        add_executable(pibridge_cam_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_cam_test_${t} PRIVATE pibridge_cam pibridge_common)
        target_compile_options(pibridge_cam_test_${t} PRIVATE -ffp-contract=off)
        target_compile_definitions(pibridge_cam_test_${t} PRIVATE PIBRIDGE_LAB_CAM_DIR="${PIBRIDGE_LAB_CAM_DIR}")
        add_test(NAME pibridge_cam_${t} COMMAND pibridge_cam_test_${t})
        set_tests_properties(pibridge_cam_${t} PROPERTIES SKIP_RETURN_CODE 77
            ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}")
    endforeach()
endif()
