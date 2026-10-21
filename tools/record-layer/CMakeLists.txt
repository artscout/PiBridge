# SPDX-License-Identifier: MIT
# pibridge-record-layer: a Vulkan implicit layer that captures the VR eye images a game shares with the SteamVR
# compositor, for PiBridge's video recording ([memory] pibridge-video-capture-plan, docs/record.md). A Vulkan layer
# loads into the game through the host Linux Vulkan loader, so it works for Proton games too (DXVK/VKD3D render on the
# host Vulkan), on any desktop and X11/Wayland, without the compositor/portal. Built only when Vulkan is present;
# nothing is downloaded. It is enabled per process by ENABLE_VK_LAYER_PIBRIDGE_RECORD=1 (the manifest's
# enable_environment), so it stays out of every other Vulkan app. M1-vk is a logger (see src/layer.cpp).

find_package(Vulkan QUIET)
find_package(Threads REQUIRED)

set(PIBRIDGE_RECORD_LAYER OFF CACHE INTERNAL "pibridge-record-layer is built")
if(NOT Vulkan_FOUND)
    message(STATUS "pibridge-record-layer: Vulkan not found: the video capture layer is not built")
    return()
endif()

add_library(pibridge_record_layer MODULE src/layer.cpp)
set_target_properties(pibridge_record_layer PROPERTIES
    OUTPUT_NAME pibridge-record-layer
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
target_compile_definitions(pibridge_record_layer PRIVATE PIBRIDGE_VERSION="${PROJECT_VERSION}")
# The shared IPC/control headers (pibridge_record_capture_ipc.h, pibridge_record_capture_control.h) live in common/.
target_include_directories(pibridge_record_layer PRIVATE ${CMAKE_SOURCE_DIR}/common)
target_link_libraries(pibridge_record_layer PRIVATE Vulkan::Vulkan Threads::Threads ${CMAKE_DL_LIBS})
# Only the loader negotiation entry point is exported; no undefined symbols.
target_link_options(pibridge_record_layer PRIVATE -Wl,--no-undefined -Wl,--exclude-libs,ALL
    -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/layer.map)
set_property(TARGET pibridge_record_layer APPEND PROPERTY LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/layer.map)

# Where the .so and the Vulkan implicit-layer manifest are installed (the loader searches
# $XDG_DATA_HOME/vulkan/implicit_layer.d, i.e. ~/.local/share/vulkan/implicit_layer.d by default).
set(PIBRIDGE_RECORD_LAYER_LIBDIR ${CMAKE_INSTALL_LIBDIR}/pibridge)
set(PIBRIDGE_RECORD_LAYER_MANIFEST_DIR ${CMAKE_INSTALL_DATADIR}/vulkan/implicit_layer.d)

# The installed manifest points at the installed .so by absolute path.
set(PIBRIDGE_RECORD_LAYER_LIBRARY_PATH "${CMAKE_INSTALL_FULL_LIBDIR}/pibridge/libpibridge-record-layer.so")
configure_file(manifest/layer.json.in
    ${CMAKE_CURRENT_BINARY_DIR}/install-manifest/VK_LAYER_PIBRIDGE_record.json @ONLY)

# A build-tree manifest too, to test before install via
#   VK_ADD_LAYER_PATH=<build>/tools/record-layer/build-manifest ENABLE_VK_LAYER_PIBRIDGE_RECORD=1 <game>
set(PIBRIDGE_RECORD_LAYER_LIBRARY_PATH "$<TARGET_FILE:pibridge_record_layer>")
configure_file(manifest/layer.json.in ${CMAKE_CURRENT_BINARY_DIR}/build-manifest.json.in @ONLY)
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/build-manifest/VK_LAYER_PIBRIDGE_record.json
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/build-manifest.json.in)

install(TARGETS pibridge_record_layer LIBRARY DESTINATION ${PIBRIDGE_RECORD_LAYER_LIBDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/install-manifest/VK_LAYER_PIBRIDGE_record.json
    DESTINATION ${PIBRIDGE_RECORD_LAYER_MANIFEST_DIR})

set(PIBRIDGE_RECORD_LAYER ON CACHE INTERNAL "pibridge-record-layer is built")
message(STATUS "pibridge-record-layer: building the Vulkan video capture layer (M1-vk logger)")
