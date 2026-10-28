add_executable(qv_test_common test_common.cpp)
target_link_libraries(qv_test_common PRIVATE qv_common)
add_test(NAME qv_common COMMAND qv_test_common)

# End to end: the real OpenXR loader, the layer from the build tree, a fake runtime; Vulkan on lavapipe (CPU).
find_package(OpenXR REQUIRED)
add_library(qv_fake_runtime MODULE fake_runtime.cpp)
target_link_libraries(qv_fake_runtime PRIVATE qv_openxr_headers Vulkan::Vulkan)
set_target_properties(qv_fake_runtime PROPERTIES CXX_VISIBILITY_PRESET hidden)

add_executable(qv_test_e2e test_e2e.cpp)
target_link_libraries(qv_test_e2e PRIVATE qv_openxr_headers OpenXR::openxr_loader Vulkan::Vulkan ${CMAKE_DL_LIBS})
add_dependencies(qv_test_e2e qv_layer qv_fake_runtime)
add_test(NAME qv_e2e COMMAND qv_test_e2e $<TARGET_FILE:qv_fake_runtime>
  ${QV_BUILD_MANIFEST})
set(QV_LAVAPIPE /usr/share/vulkan/icd.d/lvp_icd.json CACHE FILEPATH "Vulkan ICD for the end-to-end test (CPU)")
if(EXISTS ${QV_LAVAPIPE})
  set_tests_properties(qv_e2e PROPERTIES ENVIRONMENT "VK_DRIVER_FILES=${QV_LAVAPIPE}")
endif()

# The same with the Khronos validation layer and its synchronisation checks: any message fails the test.
set(QV_VALIDATION_DIR /usr/share/vulkan/explicit_layer.d CACHE PATH "Where VkLayer_khronos_validation.json is")
if(EXISTS ${QV_LAVAPIPE} AND EXISTS ${QV_VALIDATION_DIR}/VkLayer_khronos_validation.json)
  add_test(NAME qv_e2e_validation COMMAND qv_test_e2e $<TARGET_FILE:qv_fake_runtime>
    ${QV_BUILD_MANIFEST})
  set_tests_properties(qv_e2e_validation PROPERTIES
    ENVIRONMENT "VK_DRIVER_FILES=${QV_LAVAPIPE};VK_ADD_LAYER_PATH=${QV_VALIDATION_DIR};VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation;VK_VALIDATION_FEATURE_ENABLE=VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT"
    FAIL_REGULAR_EXPRESSION "VUID-|SYNC-|Validation Error|Validation Warning|FAIL")
endif()
