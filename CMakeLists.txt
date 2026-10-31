# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.22)
project(pibridge VERSION 2.0.2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(PIBRIDGE_BUILD_APP "Build the Qt 6 app" ON)
option(PIBRIDGE_BUILD_TESTS "Build the tests" ON)
# The app's last fallback in every lookup: the development machine's folders (~/gamedev/...), so an uninstalled checkout
# runs. The package turns it off: it looks only in the user's own folders and its prefix.
option(PIBRIDGE_DEV_PATHS "Let the app look in the development folders (~/gamedev/...) last" ON)
set(PIBRIDGE_SKIP_PARTS "" CACHE STRING "Parts (directories) to leave out of this build, e.g. libpibridge-lens")
# PiBridge's Monado fork (the Monado backend, docs/monado.md): its sources in third_party/monado. Read only here: the
# tests compile the driver's shared-file headers against ours; the fork is built on its own (docs/INSTALL.md).
set(PIBRIDGE_MONADO_SRC_DIR "${PROJECT_SOURCE_DIR}/third_party/monado" CACHE PATH "PiBridge's Monado fork (sources)")

include(GNUInstallDirs)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

# Device descriptions (the allowlist of HID commands lives in them): installed next to the binaries' data.
set(PIBRIDGE_DEVICES_DIR "${CMAKE_INSTALL_FULL_DATADIR}/pibridge/devices")
set(PIBRIDGE_SOURCE_DEVICES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/devices")
# systemd user units: <prefix>/share/systemd/user is on systemd's user search path for the usual prefixes
# ($XDG_DATA_HOME/systemd/user for ~/.local, $XDG_DATA_DIRS/systemd/user for /usr/local and /usr).
set(PIBRIDGE_SYSTEMD_USER_UNIT_DIR "${CMAKE_INSTALL_DATADIR}/systemd/user" CACHE STRING
    "Where the systemd user units are installed (relative to the prefix, or absolute)")

if(PIBRIDGE_BUILD_TESTS)
    enable_testing()
endif()

add_library(pibridge_common INTERFACE)
target_include_directories(pibridge_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/common)

# Each part owns its directory (docs/CONVENTIONS.md). A part that is not written yet is skipped.
# tools/roomsetup is the in-headset passthrough VIEW for SETUP PLAYSPACE: it shows the room through the cameras with
# the step text while pibridge-roommap setup fast does the mapping/floor/boundary (user 2026-10-20). Its boundary EDITOR
# (the janky controller interactions) is not used from fast mode.
foreach(part libpibridge-hmd libpibridge-eyemath libpibridge-eye libpibridge-room libpibridge-cam libpibridge-lens libpibridge-ctrl libpibridge-anchor libpibridge-roommap monado-abi
        tools/keepalived tools/record tools/eyed tools/eye-tool tools/eyecal tools/cam tools/slam tools/ctrltrack tools/roommap tools/xrkit tools/roomsetup
        quadviews tools/home tools/overlay tools/xroverlay tools/record-layer tools/headtrack tools/posetap tools/lens app)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${part}/CMakeLists.txt)
        if(part STREQUAL "app" AND NOT PIBRIDGE_BUILD_APP)
            continue()
        endif()
        if(part IN_LIST PIBRIDGE_SKIP_PARTS)
            continue()
        endif()
        add_subdirectory(${part})
    endif()
endforeach()

# Only what runs is installed: the binaries, the device descriptions, the units, the desktop entry and icon, the
# docs and the udev rule (to copy to /etc/udev/rules.d by hand, see docs/INSTALL.md). The libraries are static and
# internal: no headers or archives are installed.
install(DIRECTORY devices/ DESTINATION ${CMAKE_INSTALL_DATADIR}/pibridge/devices FILES_MATCHING PATTERN "*.toml")
# PiBridge World (world/, a Godot 4 project run by Godot as the Monado backend's home; docs/world.md)
install(DIRECTORY world/ DESTINATION ${CMAKE_INSTALL_DATADIR}/pibridge/world PATTERN ".godot" EXCLUDE PATTERN ".gitignore" EXCLUDE)
# Godot itself is not shipped: pibridge-get-godot fetches the official build for the user (checked by its checksum)
install(PROGRAMS tools/get-godot/pibridge-get-godot DESTINATION ${CMAKE_INSTALL_BINDIR})
# Or the world exported into one standalone executable, <prefix>/lib/pibridge/pibridge-world (Godot's Linux release
# template with the project embedded): the package does this, so its users need no Godot. It needs Godot
# (PIBRIDGE_GODOT) and that version's export templates on the build machine (docs/world.md).
option(PIBRIDGE_EXPORT_WORLD "Export PiBridge World to a standalone pibridge-world (needs Godot + export templates)" OFF)
set(PIBRIDGE_GODOT "" CACHE FILEPATH "The Godot 4.7 binary that exports PiBridge World")
if(PIBRIDGE_EXPORT_WORLD)
    if(NOT EXISTS "${PIBRIDGE_GODOT}")
        message(FATAL_ERROR "PIBRIDGE_EXPORT_WORLD=ON needs PIBRIDGE_GODOT, the Godot binary (\"${PIBRIDGE_GODOT}\")")
    endif()
    file(GLOB_RECURSE pibridge_world_files CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/world/*.gd ${PROJECT_SOURCE_DIR}/world/*.tscn
         ${PROJECT_SOURCE_DIR}/world/*.tres ${PROJECT_SOURCE_DIR}/world/*.godot ${PROJECT_SOURCE_DIR}/world/*.cfg)
    set(pibridge_world_copy ${CMAKE_BINARY_DIR}/world-export/project)
    set(pibridge_world_bin ${CMAKE_BINARY_DIR}/world-export/pibridge-world)
    # exported from a copy, so Godot's import cache (.godot/) never lands in the source tree
    add_custom_command(OUTPUT ${pibridge_world_bin}
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${pibridge_world_copy}
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/world ${pibridge_world_copy}
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${pibridge_world_copy}/.godot
        COMMAND ${PIBRIDGE_GODOT} --headless --path ${pibridge_world_copy} --export-release Linux ${pibridge_world_bin}
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${pibridge_world_copy}
        DEPENDS ${pibridge_world_files}
        COMMENT "Exporting PiBridge World with ${PIBRIDGE_GODOT}"
        VERBATIM)
    add_custom_target(pibridge_world_export ALL DEPENDS ${pibridge_world_bin})
    install(PROGRAMS ${pibridge_world_bin} DESTINATION lib/pibridge)
endif()
install(FILES udev/70-pibridge.rules DESTINATION ${CMAKE_INSTALL_DATADIR}/pibridge/udev)
# The NVIDIA EDID override scripts (run by hand with sudo, docs/INSTALL.md 5.2) and the Pimax EDIDs they merge.
install(PROGRAMS tools/headset-edid/pibridge-headset-edid tools/headset-edid/make_crystal_edid.py
        DESTINATION ${CMAKE_INSTALL_DATADIR}/pibridge/edid)
install(DIRECTORY tools/headset-edid/known/ DESTINATION ${CMAKE_INSTALL_DATADIR}/pibridge/edid/known FILES_MATCHING PATTERN "*.bin")
