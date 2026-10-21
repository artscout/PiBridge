# SPDX-License-Identifier: MIT
# pibridge-overlay: the PiBridge window as a SteamVR dashboard tab (docs/OVERLAY.md). It reads the app's shared frame
# (common/pibridge_vrpanel.h), uploads it to a dashboard overlay with IVROverlay::SetOverlayRaw, and writes the
# controller laser back as the pointer the app turns into mouse events. libopenvr_api.so is loaded with dlopen at run
# time (the OpenVR SDK header only, app/third_party/openvr, as tools/roomsetup does); nothing is downloaded.

# The pure parts (no OpenVR, no I/O): the byte swizzle, the laser->pixel mapping, the placeholder and the manifest text.
add_library(pibridge_overlay_core STATIC src/overlay_core.cpp src/options.cpp)
target_include_directories(pibridge_overlay_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pibridge_overlay_core PUBLIC pibridge_common)

# the OpenVR glue reads openvrpaths.vrpath through libpibridge-room (same as tools/roomsetup)
add_executable(pibridge-overlay src/main.cpp src/overlay.cpp)
target_include_directories(pibridge-overlay SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/app/third_party/openvr)
target_link_libraries(pibridge-overlay PRIVATE pibridge_overlay_core pibridge_room ${CMAKE_DL_LIBS})
# the install paths baked in as the defaults (the user never types them; the app and the installer pass the rest)
target_compile_definitions(pibridge-overlay PRIVATE
    PIBRIDGE_OVERLAY_ICON="${CMAKE_INSTALL_FULL_DATADIR}/icons/hicolor/256x256/apps/pibridge.png"
    PIBRIDGE_OVERLAY_BINARY="${CMAKE_INSTALL_FULL_BINDIR}/pibridge-overlay"
    PIBRIDGE_OVERLAY_MANIFEST="${CMAKE_INSTALL_FULL_DATADIR}/pibridge/pibridge.vrmanifest")
install(TARGETS pibridge-overlay RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# The SteamVR application manifest (one dashboard overlay). Installed so the installer can register it once with
# `pibridge-overlay --register` (or vrpathreg), with SteamVR running (the main thread, between sessions). The icon it
# names is installed by the app (app/CMakeLists.txt, hicolor/256x256/apps/pibridge.png).
set(PIBRIDGE_OVERLAY_INSTALL_BINARY "${CMAKE_INSTALL_FULL_BINDIR}/pibridge-overlay")
set(PIBRIDGE_OVERLAY_INSTALL_ICON "${CMAKE_INSTALL_FULL_DATADIR}/icons/hicolor/256x256/apps/pibridge.png")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pibridge.vrmanifest.in ${CMAKE_CURRENT_BINARY_DIR}/pibridge.vrmanifest @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pibridge.vrmanifest DESTINATION ${CMAKE_INSTALL_DATADIR}/pibridge)

if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_overlay_test tests/test_overlay.cpp)
    target_link_libraries(pibridge_overlay_test PRIVATE pibridge_overlay_core pibridge_common)
    add_test(NAME pibridge_overlay COMMAND pibridge_overlay_test)
    set_tests_properties(pibridge_overlay PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # the binary's own self test (no runtime, no GPU)
    add_test(NAME pibridge_overlay_selftest_binary COMMAND pibridge-overlay --selftest)
endif()
