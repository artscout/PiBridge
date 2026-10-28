# SPDX-License-Identifier: MIT
# quadviews: the OpenXR API layer for quad views and eye-tracked foveated rendering of Vulkan games (formerly the
# separate qv-vulkan project, merged into PiBridge; its settings are the app's Performance page). On SteamVR the layer
# composes the four views into stereo itself; on Monado, which has quad views, it moves the focus views with the gaze
# and Monado's compositor puts them in (README.md, "On Monado").
set(QV_VERSION_STRING "${PROJECT_VERSION}")
set(QV_LAYER_NAME "XR_APILAYER_ARTSCOUT_quad_views_vulkan")
set(QV_LAYER_LIBDIR "${CMAKE_INSTALL_FULL_LIBDIR}/qv-vulkan")
set(QV_MANIFEST_DIR "${CMAKE_INSTALL_FULL_DATADIR}/openxr/1/api_layers/implicit.d")

# the layer keeps its own warning set (it is C++20 code written against the OpenXR C structs)
set_property(DIRECTORY PROPERTY COMPILE_OPTIONS -Wall -Wextra -Wno-missing-field-initializers)

find_package(Vulkan REQUIRED)
find_path(OPENXR_INCLUDE_DIR openxr/openxr_loader_negotiation.h REQUIRED)
add_library(qv_openxr_headers INTERFACE)
target_include_directories(qv_openxr_headers INTERFACE ${OPENXR_INCLUDE_DIR})
target_compile_definitions(qv_openxr_headers INTERFACE XR_USE_GRAPHICS_API_VULKAN)

add_subdirectory(layer)
if(PIBRIDGE_BUILD_TESTS)
    set(QV_BUILD_MANIFEST ${CMAKE_CURRENT_BINARY_DIR}/layer/build-manifest/${QV_LAYER_NAME}.json)
    add_subdirectory(tests)
endif()
