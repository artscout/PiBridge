# SPDX-License-Identifier: MIT
# The Qt 6 app: PiBridge (binary `pibridge`). The non-GUI logic (settings writer, status readers, gaze reader, presets, commands)
# is a static library of its own so the tests check it without a display.
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(Threads REQUIRED)
find_path(PIBRIDGE_OPENXR_INCLUDE_DIR openxr/openxr.h REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(pibridge_app_core STATIC
    src/core/paths.cpp src/core/conf.cpp src/core/status.cpp src/core/gazeshm.cpp src/core/presets.cpp
    src/core/runner.cpp src/core/system.cpp src/core/actions.cpp src/core/state.cpp
    src/core/steamvr.cpp src/core/steamvr_openvr.cpp src/core/steamvr_ready.cpp src/core/steamvr_ready.h
    src/core/steamvr_reset.cpp src/core/steamvr_reset.h src/core/chovr.cpp src/core/chovr.h
    src/core/apps_vdf.cpp src/core/apps_steam.cpp src/core/apps_own.cpp src/core/apps_launch.cpp src/core/apps_launch.h
    src/core/apps_runtime.cpp src/core/apps_runtime.h src/core/monado_devices.cpp src/core/monado_devices.h
    src/core/runner.h src/core/state.h
    src/core/rec_setup.cpp src/core/rec_setup.h src/core/rec_steps.cpp src/core/rec_steps.h
    src/core/rec_check.cpp src/core/rec_check.h src/core/rec_session.cpp src/core/rec_session.h
    src/core/roomsetup.cpp src/core/roomsetup.h src/core/inside_out.cpp src/core/inside_out.h src/core/io_session.cpp src/core/io_session.h
    src/core/monado_build.cpp src/core/monado_build.h src/core/anchor_offset.cpp src/core/anchor_offset.h src/core/roommap_link.cpp src/core/roommap_link.h src/core/steamvr_drivers.cpp src/core/steamvr_drivers.h
    src/core/device_info.cpp src/core/device_info.h src/core/sound_test.cpp src/core/sound_test.h
    src/core/vr_backgrounds.cpp src/core/vr_backgrounds.h src/core/vr_panel.cpp src/core/vr_panel.h
    src/core/overlay_control.cpp src/core/overlay_control.h
    src/core/video_record.cpp src/core/video_record.h
    src/core/controllers.cpp src/core/controllers.h
    src/qv/settings.cpp src/qv/fovmath.cpp)
target_include_directories(pibridge_app_core PUBLIC src)
target_include_directories(pibridge_app_core SYSTEM PUBLIC ${PIBRIDGE_OPENXR_INCLUDE_DIR})
# OpenVR: the SDK header only (third_party/openvr, BSD-3); libopenvr_api.so of the installed SteamVR is loaded with
# dlopen at run time, so there is no link-time dependency on it.
target_include_directories(pibridge_app_core SYSTEM PRIVATE third_party/openvr)
target_link_libraries(pibridge_app_core PUBLIC Qt6::Core pibridge_common pibridge_room pibridge_record PRIVATE pibridge_hmd ${CMAKE_DL_LIBS} Threads::Threads)
# the VR home (core/vr_panel.h: the window picture and the pointer; core/vr_backgrounds.h: the thumbnails, the
# backgrounds' list of tools/home)
find_package(Qt6 REQUIRED COMPONENTS Gui)
target_link_libraries(pibridge_app_core PUBLIC Qt6::Gui Qt6::Widgets pibridge_home_backgrounds)

add_executable(pibridge-app
    src/main.cpp src/mainwindow.cpp src/mainwindow.h
    src/ui/brand.h src/ui/theme.cpp src/ui/widgets.cpp src/ui/widgets.h src/ui/icons.cpp
    src/pages/dashboard.cpp src/pages/dashboard.h src/pages/performance.cpp src/pages/performance.h
    src/pages/logs.cpp src/pages/logs.h src/pages/about.cpp src/pages/about.h
    src/pages/apps.cpp src/pages/apps.h src/pages/tracking.cpp src/pages/tracking.h
    src/pages/room.cpp src/pages/room.h src/pages/help_window.cpp src/pages/help_window.h
    src/pages/sound_tests.cpp src/pages/sound_tests.h
    src/pages/display.cpp src/pages/display.h src/pages/recording.cpp src/pages/recording.h
    src/pages/record_card.cpp src/pages/record_card.h
    src/qv/preview.cpp src/qv/preview.h src/qv/layerinstall.cpp)
set_target_properties(pibridge-app PROPERTIES OUTPUT_NAME pibridge)
# the logo as PNGs inside the binary (window icon, sidebar), rendered from data/pibridge.svg
set(PIBRIDGE_ICON_SIZES 16 24 32 48 64 128 256 512)
set(PIBRIDGE_ICON_FILES)
foreach(s ${PIBRIDGE_ICON_SIZES})
    list(APPEND PIBRIDGE_ICON_FILES data/icons/pibridge-${s}.png)
endforeach()
qt_add_resources(pibridge-app pibridge_brand PREFIX /pibridge/icons BASE data/icons FILES ${PIBRIDGE_ICON_FILES})
target_compile_definitions(pibridge-app PRIVATE
    PIBRIDGE_VERSION="${PROJECT_VERSION}"
    PIBRIDGE_SOURCE_DOCS="${PROJECT_SOURCE_DIR}/docs"
    PIBRIDGE_INSTALLED_DOCS="${CMAKE_INSTALL_FULL_DOCDIR}")
target_link_libraries(pibridge-app PRIVATE pibridge_app_core Qt6::Widgets)
install(TARGETS pibridge-app RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${PROJECT_SOURCE_DIR}/docs/app.md ${PROJECT_SOURCE_DIR}/docs/INSTALL.md ${PROJECT_SOURCE_DIR}/docs/TRACKING.md
    ${PROJECT_SOURCE_DIR}/docs/DEVELOPMENT.md ${PROJECT_SOURCE_DIR}/docs/ROOM-SETUP.md ${PROJECT_SOURCE_DIR}/docs/home.md
    ${PROJECT_SOURCE_DIR}/docs/record.md ${PROJECT_SOURCE_DIR}/docs/OVERLAY.md
    DESTINATION ${CMAKE_INSTALL_DOCDIR} OPTIONAL)
# The desktop entry names the installed binary by its full path (~/.local/bin is not on every session's PATH).
set(PIBRIDGE_APP_EXEC "${CMAKE_INSTALL_FULL_BINDIR}/pibridge")
configure_file(data/pibridge.desktop.in ${CMAKE_CURRENT_BINARY_DIR}/pibridge.desktop @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pibridge.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
install(FILES data/pibridge-crystal.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps RENAME pibridge.svg)
foreach(s ${PIBRIDGE_ICON_SIZES})
    install(FILES data/icons/pibridge-${s}.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/${s}x${s}/apps
            RENAME pibridge.png)
endforeach()

if(PIBRIDGE_BUILD_TESTS)
    # the VR home: the window picture, the pointer, the backgrounds' download steps, the shared file
    add_executable(pibridge_vrhome_tests tests/test_vrhome.cpp)
    target_link_libraries(pibridge_vrhome_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_vrhome_tests COMMAND pibridge_vrhome_tests)
    set_tests_properties(pibridge_vrhome_tests PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
    add_executable(pibridge_app_tests tests/test_app_core.cpp)
    target_link_libraries(pibridge_app_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_app_tests COMMAND pibridge_app_tests)
    # The SteamVR dashboard panel: the status words, the chip, finding the binary, and the persisted switch (fake mode:
    # no process is started).
    add_executable(pibridge_overlay_control_tests tests/test_overlay_control.cpp)
    target_link_libraries(pibridge_overlay_control_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_overlay_control_tests COMMAND pibridge_overlay_control_tests)
    set_tests_properties(pibridge_overlay_control_tests PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
    # The Apps page's logic: the VDF/ACF reader, the Steam library, apps.toml, the launcher, SteamVR readiness.
    add_executable(pibridge_apps_tests tests/test_apps.cpp)
    target_link_libraries(pibridge_apps_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_apps_tests COMMAND pibridge_apps_tests)
    set_tests_properties(pibridge_apps_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # Reset SteamVR: detection over a fake /proc tree, the TERM/KILL sequence (fake signals, injected clock), the
    # dialog words, the Steam restart (FakeRunner). No real process is ever signalled.
    add_executable(pibridge_steamvr_reset_tests tests/test_steamvr_reset.cpp)
    target_link_libraries(pibridge_steamvr_reset_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_steamvr_reset_tests COMMAND pibridge_steamvr_reset_tests)
    set_tests_properties(pibridge_steamvr_reset_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # Record tracking: setup lookup and detection over fake /sys trees, readiness words, the step runner's timing,
    # steps.json, the EuRoC check on synthetic datasets, the session (start order, lease retry, PID-exact stop) with
    # FakeRecProcesses and an injected clock. Nothing is ever started or signalled.
    add_executable(pibridge_record_tests tests/test_record.cpp)
    target_link_libraries(pibridge_record_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_record_tests COMMAND pibridge_record_tests)
    set_tests_properties(pibridge_record_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # The Room page's logic: the gates (SteamVR on / off), the fake files, the universe and room lists, the launch of
    # pibridge-roomsetup (FakeRunner). Every file is in a temporary directory.
    add_executable(pibridge_room_page_tests tests/test_room.cpp)
    target_link_libraries(pibridge_room_page_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_room_page_tests COMMAND pibridge_room_page_tests)
    set_tests_properties(pibridge_room_page_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # Tracking (inside-out): settings, the one env builder, PiBridge's Basalt config, START TRACKING (FakeRecProcesses).
    add_executable(pibridge_tracking_tests tests/test_tracking.cpp)
    target_link_libraries(pibridge_tracking_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_tracking_tests COMMAND pibridge_tracking_tests)
    set_tests_properties(pibridge_tracking_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # The Dashboard's controllers from Monado (libmonado through dlopen): link, words, worker, all with a fake source.
    add_executable(pibridge_monado_tests tests/test_monado.cpp)
    target_link_libraries(pibridge_monado_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_monado_tests COMMAND pibridge_monado_tests)
    set_tests_properties(pibridge_monado_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # SteamVR + Monado: the driver switch on made-up SteamVR files, the session with fake processes and signals, the
    # Monado build's markers, cap_sys_nice and Fix permissions (FakeRunner). Nothing is started or signalled.
    add_executable(pibridge_steamvr_monado_tests tests/test_steamvr_monado.cpp)
    target_link_libraries(pibridge_steamvr_monado_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_steamvr_monado_tests COMMAND pibridge_steamvr_monado_tests)
    set_tests_properties(pibridge_steamvr_monado_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    add_executable(pibridge_chovr_tests tests/test_chovr.cpp)
    target_link_libraries(pibridge_chovr_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_chovr_tests COMMAND pibridge_chovr_tests)
    # The Device page's words and files, the sound tests (FakeRunner + FakeCapture: nothing is played or recorded).
    add_executable(pibridge_device_tests tests/test_device.cpp)
    target_link_libraries(pibridge_device_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_device_tests COMMAND pibridge_device_tests)
    set_tests_properties(pibridge_device_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # The Dashboard's Record card: its words, record.toml, pibridge-record start/stop (FakeRunner), the tap's env.
    add_executable(pibridge_video_record_tests tests/test_video_record.cpp)
    target_link_libraries(pibridge_video_record_tests PRIVATE pibridge_app_core)
    add_test(NAME pibridge_video_record_tests COMMAND pibridge_video_record_tests)
    set_tests_properties(pibridge_video_record_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    # The tests point every XDG dir at a temporary directory themselves; this keeps a stray default safe too.
    set_tests_properties(pibridge_app_tests PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
endif()
