# SPDX-License-Identifier: MIT
# pibridge-desktop: the desktop in the headset (docs/desktop.md): the monitors through the desktop portal (ScreenCast +
# RemoteDesktop, KDE asks once) and PipeWire into $XDG_RUNTIME_DIR/pibridge/desktop.shm, the overlay's pointer back
# as the desktop's pointer. Needs Qt 6 DBus and libpipewire-0.3; without them only the pointer logic and its tests.

add_library(pibridge_desktop_core STATIC src/input.cpp)
target_include_directories(pibridge_desktop_core PUBLIC src)
target_link_libraries(pibridge_desktop_core PUBLIC pibridge_common)

find_package(Qt6 QUIET COMPONENTS Core DBus)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(PIPEWIRE QUIET IMPORTED_TARGET libpipewire-0.3)
endif()
if(Qt6DBus_FOUND AND PIPEWIRE_FOUND)
    add_executable(pibridge-desktop src/main.cpp src/portal.cpp src/service.cpp src/pw_capture.c)
    set_target_properties(pibridge-desktop PROPERTIES AUTOMOC ON)
    # PipeWire's headers are C with GNU extensions: no pedantic warnings for the C part
    set_source_files_properties(src/pw_capture.c PROPERTIES COMPILE_OPTIONS "-Wno-pedantic;-Wno-conversion;-Wno-shadow")
    target_link_libraries(pibridge-desktop PRIVATE pibridge_desktop_core Qt6::Core Qt6::DBus PkgConfig::PIPEWIRE)
    install(TARGETS pibridge-desktop RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
else()
    message(STATUS "pibridge-desktop: no Qt6 DBus or libpipewire-0.3: not built (no desktop in the headset)")
endif()

if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_desktop_tests tests/test_desktop.cpp)
    target_link_libraries(pibridge_desktop_tests PRIVATE pibridge_desktop_core pibridge_common)
    add_test(NAME pibridge_desktop_tests COMMAND pibridge_desktop_tests)
endif()
