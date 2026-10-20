# SPDX-License-Identifier: MIT
# pibridge_xrkit: the OpenXR + Vulkan plumbing shared by pibridge-roomsetup and pibridge-home (session, swapchains, the two
# scene pipelines over libpibridge-room's draw lists). Internal static library; built only when the OpenXR loader, Vulkan
# and glslangValidator are there (PIBRIDGE_XRKIT is then ON for the tools that use it). Nothing is downloaded.

# the passthrough's calibration, uniform block, CPU reference and camera feed (no OpenXR, no Vulkan): always built,
# so the tests and the snapshots run everywhere
add_library(pibridge_passthrough STATIC src/passthrough.cpp)
target_include_directories(pibridge_passthrough PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pibridge_passthrough PUBLIC pibridge_cam pibridge_room Threads::Threads)
# the synthetic room through the reference cameras (self tests, snapshots, tests)
add_library(pibridge_ptsynth STATIC src/ptsynth.cpp)
target_link_libraries(pibridge_ptsynth PUBLIC pibridge_passthrough pibridge_roommap)

find_package(OpenXR CONFIG QUIET)
find_package(Vulkan QUIET)
find_program(PIBRIDGE_GLSLANG glslangValidator)
set(PIBRIDGE_XRKIT OFF CACHE INTERNAL "pibridge_xrkit is built")
if(TARGET OpenXR::openxr_loader AND Vulkan_FOUND AND PIBRIDGE_GLSLANG)
    set(PIBRIDGE_XRKIT_SHADER_OUT ${CMAKE_CURRENT_BINARY_DIR}/shaders)
    file(MAKE_DIRECTORY ${PIBRIDGE_XRKIT_SHADER_OUT})
    set(PIBRIDGE_XRKIT_SHADERS)
    foreach(shader scene passthrough sky panel)
        foreach(stage vert frag)
            set(src ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${shader}.${stage})
            set(out ${PIBRIDGE_XRKIT_SHADER_OUT}/${shader}_${stage}.h)
            add_custom_command(OUTPUT ${out}
                COMMAND ${PIBRIDGE_GLSLANG} -V --target-env vulkan1.0 --vn ${shader}_${stage} -o ${out} ${src}
                DEPENDS ${src}
                COMMENT "SPIR-V ${shader}.${stage}")
            list(APPEND PIBRIDGE_XRKIT_SHADERS ${out})
        endforeach()
    endforeach()
    add_library(pibridge_xrkit STATIC src/xrkit.cpp src/texpass.cpp ${PIBRIDGE_XRKIT_SHADERS})
    target_include_directories(pibridge_xrkit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include PRIVATE ${PIBRIDGE_XRKIT_SHADER_OUT})
    target_link_libraries(pibridge_xrkit PUBLIC pibridge_room pibridge_passthrough OpenXR::openxr_loader Vulkan::Vulkan)
    set(PIBRIDGE_XRKIT ON CACHE INTERNAL "pibridge_xrkit is built")
else()
    message(STATUS "pibridge_xrkit: OpenXR loader, Vulkan or glslangValidator not found: no headset mode for the XR tools")
endif()
