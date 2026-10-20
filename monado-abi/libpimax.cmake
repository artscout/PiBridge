# SPDX-License-Identifier: MIT
# The libpimax part of monado-abi (M8): pimax.h over libpibridge-hmd's Session (+ haptics) and pimax_eye.h over
# libpibridge-eye's live tracker. Included from CMakeLists.txt before the archive is made: it adds the C ABI sources and
# the library objects (libpibridge-hmd, libpibridge-eyemath, libpibridge-eye compiled once more with the parity flags and hidden
# visibility) to pibridge_pimax_sources, and the device descriptions compiled in as the allowlist fallback. With
# PIBRIDGE_BUILD_TESTS: the C++ ABI tests and the difftest (difftest/pimax_difftest.c) against libpibridge_pimax.a and
# omarchyvr's libpimax_capi.a (the Rust oracle, difftest/build-oracle.sh).

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

set(lp_d ${CMAKE_CURRENT_SOURCE_DIR}/difftest)
set(lp_run ${lp_d}/run_difftest.sh)
set(lp_w ${CMAKE_CURRENT_BINARY_DIR}/difftest)
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

set(lp_copts -Wno-conversion -Wno-sign-conversion -Wno-pedantic -Wno-format-truncation -Wno-unused-result
    -ffp-contract=off)
add_executable(pimax_difftest_pibridge ${lp_d}/pimax_difftest.c)
target_link_libraries(pimax_difftest_pibridge PRIVATE pibridge_pimax ZLIB::ZLIB Threads::Threads m)
set_target_properties(pimax_difftest_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
target_compile_options(pimax_difftest_pibridge PRIVATE ${lp_copts})
if(EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_capi.a")
    add_executable(pimax_difftest_rust ${lp_d}/pimax_difftest.c)
    target_include_directories(pimax_difftest_rust PRIVATE ${lp_inc})
    target_link_libraries(pimax_difftest_rust PRIVATE "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_capi.a" ZLIB::ZLIB pthread dl m
                          rt gcc_s)
    set_target_properties(pimax_difftest_rust PROPERTIES C_STANDARD 11)
    target_compile_options(pimax_difftest_rust PRIVATE ${lp_copts})
    set(lp_r $<TARGET_FILE:pimax_difftest_rust>)
else()
    set(lp_r "${CMAKE_CURRENT_BINARY_DIR}/no-rust-oracle")
endif()
set(lp_f $<TARGET_FILE:pimax_difftest_pibridge>)
set(lp_fx ${PIBRIDGE_OMARCHYVR_DIR}/rust/fixtures)
# every run: a scratch HOME and XDG_RUNTIME_DIR, the PiBridge descriptions of this tree (the C++ gate), an empty sysfs
set(lp_sandbox --home --env XDG_RUNTIME_DIR=@W@ --env PIBRIDGE_DEVICES_DIR=${PIBRIDGE_SOURCE_DEVICES_DIR}
    --env PIMAX_SYSFS_ROOT=@W@/nosys --env PIBRIDGE_SYSFS_ROOT=@W@/nosys)
set(pimax_diff_tests)
macro(pimax_diff name)
    add_test(NAME monado_abi_diff_pimax_${name} COMMAND ${lp_run} pimax-${name} ${lp_w} ${lp_r} ${lp_f} ${lp_sandbox}
             ${ARGN})
    list(APPEND pimax_diff_tests monado_abi_diff_pimax_${name})
endmacro()

pimax_diff(layout -- layout)
pimax_diff(decode --need ${lp_fx}/p83.rec.gz -- decode ${lp_fx}/p83.rec.gz ${lp_fx}/p82.rec.gz
           ${lp_fx}/ctrl_buttons_hub2.rec.gz ${lp_fx}/if3_log_coldstart.rec.gz)
pimax_diff(session -- session @W@)
pimax_diff(replay_p83 --need ${lp_fx}/p83.rec.gz -- replay ${lp_fx}/p83.rec.gz 2)
pimax_diff(replay_p82 --need ${lp_fx}/p82.rec.gz -- replay ${lp_fx}/p82.rec.gz 1)
pimax_diff(replay_hub2 --need ${lp_fx}/ctrl_buttons_hub2.rec.gz -- replay ${lp_fx}/ctrl_buttons_hub2.rec.gz 1)
pimax_diff(replay_if3 --need ${lp_fx}/if3_log_coldstart.rec.gz -- replay ${lp_fx}/if3_log_coldstart.rec.gz 3)
pimax_diff(replay_if3_on_if0 --need ${lp_fx}/if3_log_coldstart.rec.gz -- replay ${lp_fx}/if3_log_coldstart.rec.gz 0)
pimax_diff(status37 --need ${lp_fx}/status37_golden.txt -- status37 ${lp_fx}/status37_golden.txt)
pimax_diff(mahony --need ${lp_fx}/p83.rec.gz --need ${PIBRIDGE_CAM_REFERENCE_XML}
           -- mahony ${lp_fx}/p83.rec.gz ${PIBRIDGE_CAM_REFERENCE_XML} ${lp_fx}/mahony_golden.txt)
pimax_diff(find --pre "${lp_d}/pimax_fake_sysfs.sh @W@" -- find @W@/sys @W@/dev)
pimax_diff(find_long --pre "${lp_d}/pimax_fake_sysfs.sh @W@"
           -- find @W@/sys @W@/dev/a-device-root-whose-name-is-long-enough-to-be-cut-at-sixty-three-bytes)
pimax_diff(find_crystal --pre "${lp_d}/pimax_fake_sysfs.sh @W@ 0040" -- find @W@/sys @W@/dev)
pimax_diff(find_partial --pre "${lp_d}/pimax_fake_sysfs.sh @W@ 0044 0 2" -- find @W@/sys @W@/dev)
pimax_diff(find_none -- find @W@/nosys @W@/dev)
pimax_diff(nodevice --pre "mkdir -p @W@/nosys/class/hidraw" -- nodevice @W@/nosys)
pimax_diff(close -- close)
pimax_diff(eye -- eye @W@)
pimax_diff(eye_cal --env PIMAX_EYE_CAL=1 --env PIMAX_EYE_CALIBRATION=none -- eye @W@)
set_tests_properties(${pimax_diff_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)

# Every command the C++ libpimax writes in a session (attach, video prepare at 72/90, rates 0/72/90, haptics on both
# hands, the request file) checked against each model's allowlist by the C++ test (monado_abi_pimax_capi
# `the_gate_sees_every_command`); here the same wire, Rust against C++, for the record.
pimax_diff(commands --ignore "^cmd (580439|58061a0501$)" -- commands @W@)
set_tests_properties(monado_abi_diff_pimax_commands PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)

# omarchyvr's unmodified pimax-cli (libpimax/tools/pimax-cli.c) against the archive: its paths without hardware.
set(lp_cli_src ${PIBRIDGE_OMARCHYVR_DIR}/libpimax/tools/pimax-cli.c)
if(EXISTS "${lp_cli_src}")
    add_executable(pimax_cli_pibridge ${lp_cli_src})
    target_link_libraries(pimax_cli_pibridge PRIVATE pibridge_pimax m)
    set_target_properties(pimax_cli_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
    target_compile_options(pimax_cli_pibridge PRIVATE -w)
    add_test(NAME monado_abi_pimax_cli COMMAND ${lp_d}/pimax_cli_check.sh $<TARGET_FILE:pimax_cli_pibridge> ${lp_w}/cli)
    set_tests_properties(monado_abi_pimax_cli PROPERTIES TIMEOUT 60
        ENVIRONMENT "XDG_RUNTIME_DIR=${CMAKE_CURRENT_BINARY_DIR};PIBRIDGE_DEVICES_DIR=${PIBRIDGE_SOURCE_DEVICES_DIR}")
endif()

# The Monado Pimax driver's objects (read only): every ABI function they import is in the archive.
set(PIBRIDGE_MONADO_BUILD_DIR "${PROJECT_SOURCE_DIR}/../ovr-build/monado" CACHE PATH
    "A built Monado tree with the Pimax driver (read only; the import check is skipped without it)")
add_test(NAME monado_abi_driver_imports COMMAND ${lp_d}/check_driver_imports.sh ${PIBRIDGE_MONADO_BUILD_DIR}
         ${CMAKE_CURRENT_BINARY_DIR}/libpibridge_pimax.a ${lp_inc})
set_tests_properties(monado_abi_driver_imports PROPERTIES SKIP_RETURN_CODE 77)
# the Rust difftests compare the exact Rust decode (PiBridge decodes byte 47 bit 2, the Menu button, only off parity)
set_tests_properties(${pimax_diff_tests} PROPERTIES ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
