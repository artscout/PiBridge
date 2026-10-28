# SPDX-License-Identifier: MIT
# The libpimax part of monado-abi (M8): pimax.h over libpibridge-hmd's Session (+ haptics) and pimax_eye.h over
# libpibridge-eye's live tracker. Included from CMakeLists.txt before the archive is made: it adds the C ABI sources and
# the library objects (libpibridge-hmd, libpibridge-eyemath, libpibridge-eye compiled once more with the parity flags and hidden
# visibility) to pibridge_pimax_sources, and the device descriptions compiled in as the allowlist fallback. With
# PIBRIDGE_BUILD_TESTS: the C++ ABI tests and the driver import check.

if(NOT (TARGET pibridge_hmd AND TARGET pibridge_eyemath AND TARGET pibridge_eye))
    return()
endif()

# One OBJECT library per PiBridge library, from that library's own source list, include dirs and definitions.
function(pibridge_pimax_objects name lib)
    get_target_property(srcs ${lib} SOURCES)
    get_target_property(dir ${lib} SOURCE_DIR)
    set(abs)
    foreach(s ${srcs})
        if(IS_ABSOLUTE "${s}")
            list(APPEND abs "${s}")
        else()
            list(APPEND abs "${dir}/${s}")
        endif()
    endforeach()
    add_library(${name} OBJECT ${abs})
    target_include_directories(${name} PRIVATE $<TARGET_PROPERTY:${lib},INCLUDE_DIRECTORIES>)
    target_compile_definitions(${name} PRIVATE $<TARGET_PROPERTY:${lib},COMPILE_DEFINITIONS>)
    target_compile_features(${name} PRIVATE cxx_std_23)
    target_compile_options(${name} PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS})
endfunction()
pibridge_pimax_objects(pibridge_pimax_hmd_obj pibridge_hmd)
pibridge_pimax_objects(pibridge_pimax_eyemath_obj pibridge_eyemath)
pibridge_pimax_objects(pibridge_pimax_eye_obj pibridge_eye)
target_include_directories(pibridge_pimax_eye_obj PRIVATE $<TARGET_PROPERTY:pibridge_eyemath,INTERFACE_INCLUDE_DIRECTORIES>)

# The device descriptions (the HID allowlist), compiled in: used when no PiBridge devices folder has them.
set(pibridge_pimax_dev_cpp "${CMAKE_CURRENT_BINARY_DIR}/devices_embedded.cpp")
set(pibridge_pimax_dev_body "")
foreach(id dream-air crystal-super)
    set(f "${PIBRIDGE_SOURCE_DEVICES_DIR}/${id}.toml")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${f}")
    file(READ "${f}" hexs HEX)
    string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${hexs}")
    string(LENGTH "${hexs}" n)
    math(EXPR n "${n} / 2")
    string(MAKE_C_IDENTIFIER "${id}" cid)
    string(APPEND pibridge_pimax_dev_body
        "const char k_${cid}[] = {${bytes} 0};\nconstexpr std::size_t n_${cid} = ${n};\n")
endforeach()
file(CONFIGURE OUTPUT "${pibridge_pimax_dev_cpp}" CONTENT
"// SPDX-License-Identifier: MIT
// Generated from devices/dream-air.toml and devices/crystal-super.toml by monado-abi/libpimax.cmake.
#include <cstddef>
#include <string_view>
namespace pibridge::abi {
namespace {
@pibridge_pimax_dev_body@}
std::string_view embedded_device_toml(std::string_view id) {
    if (id == \"dream-air\") return std::string_view(k_dream_air, n_dream_air);
    if (id == \"crystal-super\") return std::string_view(k_crystal_super, n_crystal_super);
    return {};
}
}
" @ONLY)

list(APPEND pibridge_pimax_sources src/pimax_capi.cpp src/eye_capi.cpp "${pibridge_pimax_dev_cpp}"
     $<TARGET_OBJECTS:pibridge_pimax_hmd_obj> $<TARGET_OBJECTS:pibridge_pimax_eyemath_obj>
     $<TARGET_OBJECTS:pibridge_pimax_eye_obj>)
list(APPEND pibridge_pimax_includes $<TARGET_PROPERTY:pibridge_hmd,INTERFACE_INCLUDE_DIRECTORIES>
     $<TARGET_PROPERTY:pibridge_eye,INTERFACE_INCLUDE_DIRECTORIES>
     $<TARGET_PROPERTY:pibridge_eyemath,INTERFACE_INCLUDE_DIRECTORIES> ${PROJECT_SOURCE_DIR}/common)
set(pibridge_pimax_have_libpimax ON)

if(NOT PIBRIDGE_BUILD_TESTS)
    return()
endif()

set(lp_d ${CMAKE_CURRENT_SOURCE_DIR}/tests)
set(lp_inc ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

# The C++ twins of pimax-capi tests/abi.rs, eye.rs, eye_export.rs and hands_export.rs, and the allowlist gate check.
add_executable(monado_abi_test_pimax_capi tests/test_pimax_capi.cpp)
target_link_libraries(monado_abi_test_pimax_capi PRIVATE pibridge_pimax pibridge_hmd pibridge_common Threads::Threads)
target_include_directories(monado_abi_test_pimax_capi PRIVATE ${lp_inc})
target_compile_definitions(monado_abi_test_pimax_capi PRIVATE
    PIBRIDGE_ABI_INCLUDE_DIR="${lp_inc}" PIBRIDGE_SOURCE_DEVICES_DIR="${PIBRIDGE_SOURCE_DEVICES_DIR}"
    PIBRIDGE_ABI_ARCHIVE="${CMAKE_CURRENT_BINARY_DIR}/libpibridge_pimax.a")
add_test(NAME monado_abi_pimax_capi COMMAND monado_abi_test_pimax_capi)
# a scratch HOME and XDG_RUNTIME_DIR: the haptic request file and the descriptions folder are never the user's
set_tests_properties(monado_abi_pimax_capi PROPERTIES
    ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR};HOME=${CMAKE_CURRENT_BINARY_DIR}/home;XDG_RUNTIME_DIR=${CMAKE_CURRENT_BINARY_DIR};PIBRIDGE_DEVICES_DIR=${PIBRIDGE_SOURCE_DEVICES_DIR}")

# The Monado Pimax driver's objects (read only): every ABI function they import is in the archive.
set(PIBRIDGE_MONADO_DRIVER_BUILD_DIR "${PROJECT_SOURCE_DIR}/../pibridge-monado-build" CACHE PATH
    "A built Monado tree (PIBRIDGE_MONADO_SRC_DIR) with the Pimax driver (read only; the import check is skipped without it)")
add_test(NAME monado_abi_driver_imports COMMAND ${lp_d}/check_driver_imports.sh ${PIBRIDGE_MONADO_DRIVER_BUILD_DIR}
         ${CMAKE_CURRENT_BINARY_DIR}/libpibridge_pimax.a ${lp_inc})
set_tests_properties(monado_abi_driver_imports PROPERTIES SKIP_RETURN_CODE 77)
