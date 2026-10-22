# SPDX-License-Identifier: MIT
# pibridge-record: records what the headset shows (port of omarchyvr's `omarchyvr record`): Monado's recorder tap
# (comp_recorder.c) -> shared memory -> ffmpeg (NVENC, x264 fallback) -> a fragmented mp4. The app's Record card
# links pibridge_record for the settings and status files.

find_package(Threads REQUIRED)
add_library(pibridge_record STATIC src/settings.cpp src/shm.cpp src/status.cpp src/proc.cpp src/encode.cpp src/run.cpp
            src/cli.cpp src/mirror.cpp src/capture_client.cpp)
target_include_directories(pibridge_record PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pibridge_record PUBLIC pibridge_common Threads::Threads)

# The Vulkan capture-layer client (src/capture_client.cpp, `--source layer`). Built with Vulkan when present; without
# it, runLayer() still compiles and fails cleanly (so the one build always has the symbol — DoD: one whole build).
find_package(Vulkan QUIET)
if(Vulkan_FOUND)
    target_link_libraries(pibridge_record PUBLIC Vulkan::Vulkan)
    target_compile_definitions(pibridge_record PRIVATE PIBRIDGE_HAVE_VULKAN=1)
    message(STATUS "pibridge-record: Vulkan found: --source layer (video capture) is built")
else()
    message(STATUS "pibridge-record: Vulkan not found: --source layer fails cleanly at run time")
endif()

add_executable(pibridge-record src/main.cpp)
target_link_libraries(pibridge-record PRIVATE pibridge_record)
install(TARGETS pibridge-record RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# pibridge-record-mirror: the SteamVR source (OpenVR mirror texture in a headless EGL/OpenGL context). The OpenVR
# header is the app's copy (BSD-3); libopenvr_api.so of the installed SteamVR is loaded with dlopen at run time and
# every GL entry point comes from eglGetProcAddress, so it links libEGL only.
find_library(PIBRIDGE_EGL_LIBRARY EGL REQUIRED)
find_path(PIBRIDGE_EGL_INCLUDE_DIR EGL/egl.h REQUIRED)
add_executable(pibridge-record-mirror src/mirror_main.cpp src/mirror_openvr.cpp)
target_include_directories(pibridge-record-mirror SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/app/third_party/openvr ${PIBRIDGE_EGL_INCLUDE_DIR})
target_link_libraries(pibridge-record-mirror PRIVATE pibridge_record ${PIBRIDGE_EGL_LIBRARY} ${CMAKE_DL_LIBS})
install(TARGETS pibridge-record-mirror RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    # Our Monado fork's sources: the test also compiles the real comp_recorder_shm.h (when present) and checks that
    # both layouts agree field for field.
    set(PIBRIDGE_MONADO_SRC_DIR "${PROJECT_SOURCE_DIR}/third_party/monado" CACHE PATH "PiBridge's Monado fork (sources)")
    add_executable(pibridge_record_fakeproducer tests/fake_producer.cpp)
    target_link_libraries(pibridge_record_fakeproducer PRIVATE pibridge_record)
    # a fake libopenvr_api.so: pibridge-record-mirror's start errors without SteamVR, EGL or a GPU
    add_library(pibridge_record_fake_openvr MODULE tests/fake_openvr.cpp)
    # Twins of the Rust unit tests, the ring with an in-process fake producer, and the whole recorder (the real
    # pibridge-record binary) with a fake producer and a fake ffmpeg (tests/fake_ffmpeg.sh): nothing touches a GPU,
    # a session or the sound server.
    add_executable(pibridge_record_tool_tests tests/test_record.cpp)
    target_link_libraries(pibridge_record_tool_tests PRIVATE pibridge_record)
    target_compile_definitions(pibridge_record_tool_tests PRIVATE
        PIBRIDGE_RECORD_BIN="$<TARGET_FILE:pibridge-record>"
        PIBRIDGE_RECORD_MIRROR="$<TARGET_FILE:pibridge-record-mirror>"
        PIBRIDGE_FAKE_PRODUCER="$<TARGET_FILE:pibridge_record_fakeproducer>"
        PIBRIDGE_FAKE_OPENVR="$<TARGET_FILE:pibridge_record_fake_openvr>"
        PIBRIDGE_FAKE_FFMPEG="${CMAKE_CURRENT_SOURCE_DIR}/tests/fake_ffmpeg.sh"
        PIBRIDGE_MONADO_SHM_HEADER="${PIBRIDGE_MONADO_SRC_DIR}/src/xrt/compositor/main/comp_recorder_shm.h")
    add_dependencies(pibridge_record_tool_tests pibridge-record pibridge-record-mirror pibridge_record_fakeproducer pibridge_record_fake_openvr)
    add_test(NAME pibridge_record_tool COMMAND pibridge_record_tool_tests)
    set_tests_properties(pibridge_record_tool PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
endif()
