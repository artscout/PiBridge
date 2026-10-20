# SPDX-License-Identifier: MIT
# pibridge-eyecal: the eye calibration (the chip's own Tobii calibration as Pimax Play runs it, the host fit, recentre),
# port of omarchyvr `pimax-eyecal`. See docs/eyecal.md.
#
# The procedure, the fits, the report, the simulation and the HUD texture are a library (pibridge_eyecal_core) the tests
# use; the OpenXR + Vulkan app is built when the OpenXR loader, Vulkan and glslangValidator are there (without them
# pibridge-eyecal only runs --simulate). Nothing is downloaded.

add_library(pibridge_eyecal_core STATIC
    src/layout.cpp
    src/fixation.cpp
    src/calfit.cpp
    src/collect.cpp
    src/refit.cpp
    src/procedure.cpp
    src/report.cpp
    src/sim.cpp
    src/hud.cpp
    src/marks.cpp
    src/chip.cpp
    src/options.cpp
    src/runner.cpp
)
target_include_directories(pibridge_eyecal_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pibridge_eyecal_core PUBLIC pibridge_eye pibridge_hmd pibridge_common)

find_package(OpenXR CONFIG QUIET)
find_package(Vulkan QUIET)
find_program(PIBRIDGE_GLSLANG glslangValidator)
if(TARGET OpenXR::openxr_loader AND Vulkan_FOUND AND PIBRIDGE_GLSLANG)
    # SPIR-V of the marks pass, as C arrays (glslangValidator --vn), like the quad views layer builds its shaders
    set(PIBRIDGE_EYECAL_SHADER_OUT ${CMAKE_CURRENT_BINARY_DIR}/shaders)
    file(MAKE_DIRECTORY ${PIBRIDGE_EYECAL_SHADER_OUT})
    set(PIBRIDGE_EYECAL_SHADERS)
    foreach(stage vert frag)
        set(src ${CMAKE_CURRENT_SOURCE_DIR}/shaders/marks.${stage})
        set(out ${PIBRIDGE_EYECAL_SHADER_OUT}/marks_${stage}.h)
        add_custom_command(OUTPUT ${out}
            COMMAND ${PIBRIDGE_GLSLANG} -V --target-env vulkan1.0 --vn marks_${stage} -o ${out} ${src}
            DEPENDS ${src}
            COMMENT "SPIR-V marks.${stage}")
        list(APPEND PIBRIDGE_EYECAL_SHADERS ${out})
    endforeach()
    add_executable(pibridge-eyecal src/main.cpp src/xrapp.cpp ${PIBRIDGE_EYECAL_SHADERS})
    target_include_directories(pibridge-eyecal PRIVATE ${PIBRIDGE_EYECAL_SHADER_OUT})
    target_compile_definitions(pibridge-eyecal PRIVATE PIBRIDGE_EYECAL_XR=1)
    target_link_libraries(pibridge-eyecal PRIVATE pibridge_eyecal_core pibridge_eyed_core OpenXR::openxr_loader Vulkan::Vulkan)
else()
    message(STATUS "pibridge-eyecal: OpenXR loader, Vulkan or glslangValidator not found: built for --simulate only")
    add_executable(pibridge-eyecal src/main.cpp)
    target_link_libraries(pibridge-eyecal PRIVATE pibridge_eyecal_core pibridge_eyed_core)
endif()
install(TARGETS pibridge-eyecal RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    foreach(t layout fixation calfit collect procedure sim report hud marks chip options runner)
        add_executable(pibridge_eyecal_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_eyecal_test_${t} PRIVATE pibridge_eyecal_core)
        add_test(NAME pibridge_eyecal_${t} COMMAND pibridge_eyecal_test_${t})
    endforeach()
    set_tests_properties(pibridge_eyecal_sim PROPERTIES TIMEOUT 600)
    # the std140 layout test reads the shader source
    target_compile_definitions(pibridge_eyecal_test_marks PRIVATE PIBRIDGE_EYECAL_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
endif()
