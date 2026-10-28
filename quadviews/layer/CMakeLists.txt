# The settings and the geometry: shared by the layer, the settings app and the tests.
add_library(qv_common STATIC src/settings.cpp src/fovmath.cpp src/log.cpp)
target_include_directories(qv_common PUBLIC src)
target_link_libraries(qv_common PUBLIC qv_openxr_headers)

# SPIR-V of the composition shaders, as C arrays (glslangValidator --vn).
find_program(GLSLANG glslangValidator REQUIRED)
set(QV_SHADER_OUT ${CMAKE_CURRENT_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${QV_SHADER_OUT})
set(QV_SHADER_HEADERS)
foreach(stage vert frag)
  set(src ${CMAKE_CURRENT_SOURCE_DIR}/shaders/compose.${stage})
  set(out ${QV_SHADER_OUT}/compose_${stage}.h)
  add_custom_command(OUTPUT ${out}
    COMMAND ${GLSLANG} -V --target-env vulkan1.0 --vn compose_${stage} -o ${out} ${src}
    DEPENDS ${src}
    COMMENT "SPIR-V compose.${stage}")
  list(APPEND QV_SHADER_HEADERS ${out})
endforeach()
add_custom_target(qv_shaders DEPENDS ${QV_SHADER_HEADERS})

add_library(qv_layer MODULE src/layer.cpp src/compositor_vk.cpp ${QV_SHADER_HEADERS})
set_target_properties(qv_layer PROPERTIES
  OUTPUT_NAME ${QV_LAYER_NAME}
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
target_include_directories(qv_layer PRIVATE ${QV_SHADER_OUT})
target_compile_definitions(qv_layer PRIVATE QV_VERSION="${QV_VERSION_STRING}")
target_link_libraries(qv_layer PRIVATE qv_common Vulkan::Vulkan)
# Only the negotiation entry point is exported; no undefined symbols.
target_link_options(qv_layer PRIVATE -Wl,--no-undefined -Wl,--exclude-libs,ALL
  -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/layer.map)
set_property(TARGET qv_layer APPEND PROPERTY LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/layer.map)

# The manifest for the build tree (tests, QV_LAYER_PATH) and the one that is installed.
set(QV_LAYER_LIBRARY_PATH "$<TARGET_FILE:qv_layer>")
configure_file(manifest/layer.json.in ${CMAKE_CURRENT_BINARY_DIR}/manifest.json.in @ONLY)
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/build-manifest/${QV_LAYER_NAME}.json
  INPUT ${CMAKE_CURRENT_BINARY_DIR}/manifest.json.in)

set(QV_LAYER_LIBRARY_PATH "${QV_LAYER_LIBDIR}/lib${QV_LAYER_NAME}.so")
configure_file(manifest/layer.json.in ${CMAKE_CURRENT_BINARY_DIR}/install-manifest/${QV_LAYER_NAME}.json @ONLY)

install(TARGETS qv_layer LIBRARY DESTINATION ${QV_LAYER_LIBDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/install-manifest/${QV_LAYER_NAME}.json DESTINATION ${QV_MANIFEST_DIR})
