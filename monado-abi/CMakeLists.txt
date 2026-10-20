# SPDX-License-Identifier: MIT
# monado-abi: the C ABI the Monado Pimax driver links (docs/research/rust-port-plan.md). The headers in include/ are
# verbatim copies of omarchyvr's at the pinned commit (spec.txt, include/SHA256SUMS); the archive libpibridge_pimax.a
# holds the implementation and every library object it needs, with only pimax_* visible.
# Today (M0-M7): pimax_cam.h (with the AE entry points, M3), pimax_lens_rs.h, pimax_hands.h, pimax_ctrlfuse.h,
# pimax_inhand.h.

# PIBRIDGE_OMARCHYVR_DIR, PIBRIDGE_RUST_ORACLE_DIR, PIBRIDGE_LAB_CAM_DIR, PIBRIDGE_LAB_CALIB_DIR: see libpibridge-cam/CMakeLists.txt.

# Step 1: the objects (the C ABI and the libraries), compiled with hidden visibility.
# The C ABI sources and the library objects behind them: the camera (M1-M2) and SLAM/6DoF (M4) parts over
# libpibridge-cam (with the auto-exposure part, M3), the lens (M5) over libpibridge-lens.
set(pibridge_pimax_sources src/cam_capi.cpp src/slam_capi.cpp src/ae_capi.cpp $<TARGET_OBJECTS:pibridge_cam_obj>)
# The PIBRIDGE_PIMAX_ABI version string (src/abi_id.cpp), with the pinned spec commit from spec.txt.
file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/spec.txt pibridge_pimax_spec_line REGEX "^commit ")
string(REGEX REPLACE "^commit +([0-9a-f]+).*" "\\1" pibridge_pimax_spec "${pibridge_pimax_spec_line}")
list(APPEND pibridge_pimax_sources src/abi_id.cpp)
set_source_files_properties(src/abi_id.cpp PROPERTIES COMPILE_DEFINITIONS "PIBRIDGE_PIMAX_ABI_SPEC=\"${pibridge_pimax_spec}\""
                            OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/spec.txt)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/spec.txt)
set(pibridge_pimax_includes $<TARGET_PROPERTY:pibridge_cam_obj,INTERFACE_INCLUDE_DIRECTORIES>)
if(TARGET pibridge_lens_obj)
    list(APPEND pibridge_pimax_sources src/lens_capi.cpp $<TARGET_OBJECTS:pibridge_lens_obj>)
    list(APPEND pibridge_pimax_includes $<TARGET_PROPERTY:pibridge_lens_obj,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
# The controller hands, fuser and in-hand ABIs (M6) over libpibridge-ctrl's hands half.
if(TARGET pibridge_ctrl_obj)
    list(APPEND pibridge_pimax_sources src/hands_capi.cpp src/ctrlfuse_capi.cpp src/inhand_capi.cpp
         $<TARGET_OBJECTS:pibridge_ctrl_obj>)
    list(APPEND pibridge_pimax_includes $<TARGET_PROPERTY:pibridge_ctrl_obj,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
# libpimax + eye (M8) over libpibridge-hmd / libpibridge-eye, with its tests.
include(${CMAKE_CURRENT_SOURCE_DIR}/libpimax.cmake OPTIONAL)
add_library(pibridge_pimax_objs STATIC ${pibridge_pimax_sources})
set_target_properties(pibridge_pimax_objs PROPERTIES OUTPUT_NAME pibridge_pimax_objs)
target_include_directories(pibridge_pimax_objs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(pibridge_pimax_objs PRIVATE ${pibridge_pimax_includes})
target_compile_features(pibridge_pimax_objs PRIVATE cxx_std_23)
target_compile_options(pibridge_pimax_objs PRIVATE ${PIBRIDGE_CAM_PARITY_FLAGS})

# Step 2: one relocatable object with only pimax_* global. -fvisibility=hidden alone leaves the std:: template
# instantiations (std::string, std::format internals, ...) exported from a shared object such as driver_monado.so;
# a partial link that dissolves the COMDAT groups (--force-group-allocation) and then localises every other symbol
# keeps them private, so they can neither clash with Monado's own C++ nor be discarded in favour of its copies.
set(pibridge_pimax_o ${CMAKE_CURRENT_BINARY_DIR}/pibridge_pimax.o)
set(pibridge_pimax_a ${CMAKE_CURRENT_BINARY_DIR}/libpibridge_pimax.a)
add_custom_command(OUTPUT ${pibridge_pimax_a}
    COMMAND ${CMAKE_LINKER} -r --force-group-allocation --whole-archive $<TARGET_FILE:pibridge_pimax_objs>
            --no-whole-archive -o ${pibridge_pimax_o}
    COMMAND ${CMAKE_OBJCOPY} -w --keep-global-symbol=pimax_* ${pibridge_pimax_o}
    COMMAND ${CMAKE_COMMAND} -E rm -f ${pibridge_pimax_a}
    COMMAND ${CMAKE_AR} rcs ${pibridge_pimax_a} ${pibridge_pimax_o}
    DEPENDS pibridge_pimax_objs
    COMMENT "libpibridge_pimax.a: one object, only pimax_* global"
    VERBATIM)
add_custom_target(pibridge_pimax_archive ALL DEPENDS ${pibridge_pimax_a})
add_library(pibridge_pimax STATIC IMPORTED GLOBAL)
set_target_properties(pibridge_pimax PROPERTIES IMPORTED_LOCATION ${pibridge_pimax_a}
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
    IMPORTED_LINK_INTERFACE_LANGUAGES CXX)
add_dependencies(pibridge_pimax pibridge_pimax_archive)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
set_property(TARGET pibridge_pimax PROPERTY INTERFACE_LINK_LIBRARIES ZLIB::ZLIB Threads::Threads)

if(PIBRIDGE_BUILD_TESTS)
    set(d ${CMAKE_CURRENT_SOURCE_DIR}/difftest)
    add_test(NAME monado_abi_headers COMMAND ${d}/check_headers.sh ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME monado_abi_headers_pin COMMAND ${d}/check_headers.sh ${CMAKE_CURRENT_SOURCE_DIR}/include
             ${PIBRIDGE_OMARCHYVR_DIR})

    add_executable(monado_abi_test_capi tests/test_capi.cpp)
    target_link_libraries(monado_abi_test_capi PRIVATE pibridge_pimax pibridge_common)
    target_compile_definitions(monado_abi_test_capi PRIVATE PIBRIDGE_CAM_REFERENCE_XML="${PIBRIDGE_CAM_REFERENCE_XML}")
    add_test(NAME monado_abi_capi COMMAND monado_abi_test_capi)

    # only pimax_* leaves a shared object built from the whole archive
    add_library(pibridge_pimax_symcheck SHARED ${d}/empty.c)
    target_link_libraries(pibridge_pimax_symcheck PRIVATE "-Wl,--whole-archive" pibridge_pimax "-Wl,--no-whole-archive"
                          ZLIB::ZLIB Threads::Threads)
    set_target_properties(pibridge_pimax_symcheck PROPERTIES LINKER_LANGUAGE CXX)
    add_test(NAME monado_abi_symbols COMMAND ${d}/check_symbols.sh $<TARGET_FILE:pibridge_pimax_symcheck>)
    # the PIBRIDGE_PIMAX_ABI marker survives into a shared object built from the archive (plan M9)
    add_test(NAME monado_abi_marker COMMAND sh -c "grep -aq 'PIBRIDGE_PIMAX_ABI 1 spec omarchyvr ${pibridge_pimax_spec}' \"$1\"" sh
             $<TARGET_FILE:pibridge_pimax_symcheck>)

    # The difftest program, once against libpibridge_pimax.a and once against the Rust archive.
    set(pibridge_difftest_c_opts -Wno-conversion -Wno-sign-conversion -Wno-pedantic)
    add_executable(cam_difftest_pibridge ${d}/cam_difftest.c)
    target_link_libraries(cam_difftest_pibridge PRIVATE pibridge_pimax)
    set_target_properties(cam_difftest_pibridge PROPERTIES LINKER_LANGUAGE CXX C_STANDARD 11)
    target_compile_options(cam_difftest_pibridge PRIVATE ${pibridge_difftest_c_opts})
    if(EXISTS "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a")
        add_executable(cam_difftest_rust ${d}/cam_difftest.c)
        target_include_directories(cam_difftest_rust PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        target_link_libraries(cam_difftest_rust PRIVATE "${PIBRIDGE_RUST_ORACLE_DIR}/libpimax_cam.a" pthread dl m rt gcc_s)
        set_target_properties(cam_difftest_rust PROPERTIES C_STANDARD 11)
        target_compile_options(cam_difftest_rust PRIVATE ${pibridge_difftest_c_opts})
        set(pibridge_difftest_rust $<TARGET_FILE:cam_difftest_rust>)
    else()
        set(pibridge_difftest_rust "${CMAKE_CURRENT_BINARY_DIR}/no-rust-oracle")
    endif()

    set(run ${d}/run_difftest.sh)
    set(w ${CMAKE_CURRENT_BINARY_DIR}/difftest)
    set(f $<TARGET_FILE:cam_difftest_pibridge>)
    set(r ${pibridge_difftest_rust})
    set(ref ${PIBRIDGE_CAM_REFERENCE_XML})
    set(crystal ${PIBRIDGE_LAB_CALIB_DIR}/crystal-calibration.xml)
    add_test(NAME monado_abi_diff_layout COMMAND ${run} layout ${w} ${r} ${f} -- layout)
    add_test(NAME monado_abi_diff_clock COMMAND ${run} clock ${w} ${r} ${f} -- clock)
    add_test(NAME monado_abi_diff_errors COMMAND ${run} errors ${w} ${r} ${f} -- errors @W@)
    add_test(NAME monado_abi_diff_calib_reference COMMAND ${run} calib-ref ${w} ${r} ${f} -- calib ${ref})
    add_test(NAME monado_abi_diff_calib_crystal COMMAND ${run} calib-crystal ${w} ${r} ${f} --need ${crystal}
             -- calib ${crystal})
    add_test(NAME monado_abi_diff_calib_load COMMAND ${run} calib-load ${w} ${r} ${f} -- load @W@ ${ref})
    set(diff_tests monado_abi_diff_layout monado_abi_diff_clock monado_abi_diff_errors monado_abi_diff_calib_reference
        monado_abi_diff_calib_crystal monado_abi_diff_calib_load)
    foreach(rec first ctrl1 ctrl2)
        set(px ${PIBRIDGE_LAB_CAM_DIR}/${rec}.pxcam)
        add_test(NAME monado_abi_diff_replay_${rec} COMMAND ${run} replay-${rec} ${w} ${r} ${f} --need ${px}
                 -- replay ${px})
        list(APPEND diff_tests monado_abi_diff_replay_${rec})
    endforeach()
    set(first ${PIBRIDGE_LAB_CAM_DIR}/first.pxcam)
    add_test(NAME monado_abi_diff_replay_loop COMMAND ${run} replay-loop ${w} ${r} ${f} --need ${first}
             -- replay ${first} 400)
    add_test(NAME monado_abi_diff_record COMMAND ${run} record ${w} ${r} ${f} --need ${first}
             -- record ${first} @W@ 40)
    list(APPEND diff_tests monado_abi_diff_replay_loop monado_abi_diff_record)
    set_tests_properties(monado_abi_headers_pin ${diff_tests} PROPERTIES SKIP_RETURN_CODE 77)
    set_tests_properties(${diff_tests} PROPERTIES TIMEOUT 600)

    # The SLAM/6DoF (M4), auto-exposure (M3) and lens (M5) difftests, each in its own file.
    include(${CMAKE_CURRENT_SOURCE_DIR}/slam.cmake OPTIONAL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/ae.cmake OPTIONAL)
    if(TARGET pibridge_lens_obj)
        include(${CMAKE_CURRENT_SOURCE_DIR}/lens.cmake OPTIONAL)
    endif()
    # The hands / ctrlfuse / inhand tests and difftests (M6).
    if(TARGET pibridge_ctrl_obj)
        include(${CMAKE_CURRENT_SOURCE_DIR}/hands.cmake OPTIONAL)
    endif()
endif()
