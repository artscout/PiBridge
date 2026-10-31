# SPDX-License-Identifier: MIT
# pibridge-xroverlay: the Monado backend's overlay over every app (XR_EXTX_overlay): the recenter ring and the loading
# dots and the volume indicator (docs/xroverlay.md).
add_library(pibridge_xroverlay_core STATIC src/loading.cpp src/volume.cpp)
target_include_directories(pibridge_xroverlay_core PUBLIC src)
target_link_libraries(pibridge_xroverlay_core PUBLIC pibridge_common)
if(PIBRIDGE_XRKIT)
    add_executable(pibridge-xroverlay src/main.cpp)
    target_link_libraries(pibridge-xroverlay PRIVATE pibridge_xrkit pibridge_overlay_core pibridge_xroverlay_core pibridge_common)
    install(TARGETS pibridge-xroverlay RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_xroverlay_tests tests/test_loading.cpp)
    target_link_libraries(pibridge_xroverlay_tests PRIVATE pibridge_xroverlay_core)
    add_test(NAME pibridge_xroverlay_loading COMMAND pibridge_xroverlay_tests)
    add_executable(pibridge_xroverlay_volume_tests tests/test_volume.cpp)
    target_link_libraries(pibridge_xroverlay_volume_tests PRIVATE pibridge_xroverlay_core)
    add_test(NAME pibridge_xroverlay_volume COMMAND pibridge_xroverlay_volume_tests)
endif()
