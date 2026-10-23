# SPDX-License-Identifier: MIT
# pibridge-headtrack: the Crystal Super's / Dream Air's inside-out head tracking without Monado.

add_library(pibridge_headtrack_core STATIC src/headtrack.cpp src/vit_host.cpp src/world.cpp)
target_include_directories(pibridge_headtrack_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
                                                   PRIVATE ${PROJECT_SOURCE_DIR}/winbridge)
target_link_libraries(pibridge_headtrack_core PUBLIC pibridge_hmd pibridge_cam pibridge_ctrl pibridge_ctrltrack pibridge_common ${CMAKE_DL_LIBS})

add_executable(pibridge-headtrack src/main.cpp)
target_link_libraries(pibridge-headtrack PRIVATE pibridge_headtrack_core)

install(TARGETS pibridge-headtrack RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_headtrack_test tests/test_headtrack.cpp)
    target_link_libraries(pibridge_headtrack_test PRIVATE pibridge_headtrack_core)
    add_test(NAME pibridge_headtrack COMMAND pibridge_headtrack_test)
endif()
