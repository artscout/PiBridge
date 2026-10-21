# SPDX-License-Identifier: MIT
# pibridge-roomsetup: the room setup in the headset (port of ZoneTweak's editor, MIT, Copyright (c) 2026 killdano; see
# NOTICE and docs/ROOM-SETUP.md). The editor itself is libpibridge-room; this part is the OpenXR + Vulkan app, the
# OpenVR helper processes (universe, reload, zone settings) and the self test. Built with OpenXR when tools/xrkit is
# (the loader, Vulkan and glslangValidator are there; else only --selftest, --list and --use). Nothing is downloaded.

add_library(pibridge_roomsetup_core STATIC src/options.cpp src/host.cpp src/selftest.cpp src/recenter.cpp src/mapflow.cpp)
target_include_directories(pibridge_roomsetup_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# the map flow speaks pibridge-roommap's guide files (libpibridge-roommap guide_ipc.h); the self test draws the passthrough
# of the synthetic room (libpibridge-roommap synth.h) with the CPU reference (tools/xrkit passthrough.h)
target_link_libraries(pibridge_roomsetup_core PUBLIC pibridge_room pibridge_roommap pibridge_passthrough pibridge_ptsynth)

# the OpenVR SDK header only (app/third_party/openvr, BSD-3); libopenvr_api.so is loaded with dlopen at run time
set(PIBRIDGE_ROOMSETUP_OPENVR_INCLUDE ${PROJECT_SOURCE_DIR}/app/third_party/openvr)

set(PIBRIDGE_ROOMSETUP_SOURCES src/main.cpp src/openvr_helper.cpp)
# the OpenXR + Vulkan plumbing and the scene shaders are shared with pibridge-home (tools/xrkit)
if(PIBRIDGE_XRKIT)
    # src/rendermodel.cpp loads the SteamVR controller render models (OpenVR, dlopen'd); it needs the OpenVR header
    add_executable(pibridge-roomsetup ${PIBRIDGE_ROOMSETUP_SOURCES} src/xrapp.cpp src/rendermodel.cpp)
    target_compile_definitions(pibridge-roomsetup PRIVATE PIBRIDGE_ROOMSETUP_XR=1)
    target_link_libraries(pibridge-roomsetup PRIVATE pibridge_xrkit)
else()
    message(STATUS "pibridge-roomsetup: no pibridge_xrkit (OpenXR loader, Vulkan or glslangValidator missing): no headset mode")
    add_executable(pibridge-roomsetup ${PIBRIDGE_ROOMSETUP_SOURCES})
endif()
target_include_directories(pibridge-roomsetup SYSTEM PRIVATE ${PIBRIDGE_ROOMSETUP_OPENVR_INCLUDE})
target_link_libraries(pibridge-roomsetup PRIVATE pibridge_roomsetup_core ${CMAKE_DL_LIBS})
install(TARGETS pibridge-roomsetup RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    foreach(t options host selftest recenter mapflow passthrough)
        add_executable(pibridge_roomsetup_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_roomsetup_test_${t} PRIVATE pibridge_roomsetup_core pibridge_common)
        add_test(NAME pibridge_roomsetup_${t} COMMAND pibridge_roomsetup_test_${t})
        set_tests_properties(pibridge_roomsetup_${t} PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    endforeach()
    # the binary's own self test (no runtime, no GPU)
    add_test(NAME pibridge_roomsetup_selftest_binary COMMAND pibridge-roomsetup --selftest)
endif()
