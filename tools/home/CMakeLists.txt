# SPDX-License-Identifier: MIT
# pibridge-home: the home scene in the headset while PiBridge's Tracking session runs with apps through Monado directly
# (docs/HOME.md): a floor grid, the horizon, the active saved room and the controllers, drawn through tools/xrkit.
# The scene and the session rules are a library of their own (tested without a runtime); the OpenXR part is built
# when pibridge_xrkit is (else only --selftest). Nothing is downloaded.

find_package(Threads REQUIRED)

# the backgrounds' list, paths and settings: also the app's (its Home page)
add_library(pibridge_home_backgrounds STATIC src/backgrounds.cpp)
target_include_directories(pibridge_home_backgrounds PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_library(pibridge_home_core STATIC src/scene.cpp src/loop.cpp src/options.cpp src/selftest.cpp src/sky.cpp src/panel.cpp
    src/plan.cpp src/dash_settings.cpp src/dash_view.cpp src/dash_sys.cpp)
target_include_directories(pibridge_home_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# the self test draws the passthrough of the synthetic room (tools/xrkit ptsynth.h) with the CPU reference; the
# dashboard's receivers use libpibridge-hmd (the headset volume through pactl, the vibration request's format)
target_link_libraries(pibridge_home_core PUBLIC pibridge_record pibridge_room pibridge_common pibridge_home_backgrounds pibridge_ptsynth
    pibridge_hmd Threads::Threads)

# The dashboard's picture, painted with QPainter (dash_paint.*; Qt 6 Gui is already a PiBridge dependency: the app,
# tools/roomsetup's panel). Without Qt the home has no dashboard and shows the window or the text panel as before.
find_package(Qt6 QUIET COMPONENTS Gui)
if(Qt6Gui_FOUND)
    add_library(pibridge_home_dash_paint STATIC src/dash_paint.cpp)
    target_include_directories(pibridge_home_dash_paint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(pibridge_home_dash_paint PUBLIC Qt6::Gui)
    set(PIBRIDGE_HOME_DASH 1)
else()
    message(STATUS "pibridge-home: no Qt6 Gui: no dashboard (the PiBridge window or the text panel only)")
endif()

if(PIBRIDGE_XRKIT)
    add_executable(pibridge-home src/main.cpp src/xrhome.cpp src/monado_clients.cpp)
    target_compile_definitions(pibridge-home PRIVATE PIBRIDGE_HOME_XR=1)
    target_link_libraries(pibridge-home PRIVATE pibridge_xrkit ${CMAKE_DL_LIBS})
else()
    message(STATUS "pibridge-home: no pibridge_xrkit (OpenXR loader, Vulkan or glslangValidator missing): only --selftest")
    add_executable(pibridge-home src/main.cpp)
endif()
target_link_libraries(pibridge-home PRIVATE pibridge_home_core)
if(PIBRIDGE_HOME_DASH)
    target_compile_definitions(pibridge-home PRIVATE PIBRIDGE_HOME_DASH=1)
    target_link_libraries(pibridge-home PRIVATE pibridge_home_dash_paint)
endif()
install(TARGETS pibridge-home RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_home_tests tests/test_home.cpp)
    target_link_libraries(pibridge_home_tests PRIVATE pibridge_home_core pibridge_common)
    add_test(NAME pibridge_home_tests COMMAND pibridge_home_tests)
    set_tests_properties(pibridge_home_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # the dashboard: settings and their receivers (fakes), layout, hits, clicks; its painter when Qt is there
    add_executable(pibridge_home_dash_tests tests/test_dash.cpp)
    target_link_libraries(pibridge_home_dash_tests PRIVATE pibridge_home_core pibridge_common)
    if(PIBRIDGE_HOME_DASH)
        target_compile_definitions(pibridge_home_dash_tests PRIVATE PIBRIDGE_HOME_DASH=1)
        target_link_libraries(pibridge_home_dash_tests PRIVATE pibridge_home_dash_paint)
    endif()
    add_test(NAME pibridge_home_dash_tests COMMAND pibridge_home_dash_tests)
    set_tests_properties(pibridge_home_dash_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # the binary's own self test (no runtime, no GPU)
    add_test(NAME pibridge_home_selftest_binary COMMAND pibridge-home --selftest)
endif()
