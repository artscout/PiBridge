# SPDX-License-Identifier: MIT
# pibridge-eye: offline tools for the eye chip (profile, frames, pcap, plan). See docs/eye-tool.md.

add_library(pibridge_eye_cli STATIC src/cli.cpp)
target_include_directories(pibridge_eye_cli PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pibridge_eye_cli PUBLIC pibridge_eye)

add_executable(pibridge-eye src/main.cpp)
target_link_libraries(pibridge-eye PRIVATE pibridge_eye_cli)
install(TARGETS pibridge-eye RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_eye_test_cli tests/test_cli.cpp)
    target_link_libraries(pibridge_eye_test_cli PRIVATE pibridge_eye_cli pibridge_common)
    # the packet builders shared with the libpibridge-eye tests
    target_include_directories(pibridge_eye_test_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../libpibridge-eye/tests)
    add_test(NAME pibridge_eye_cli COMMAND pibridge_eye_test_cli)
endif()
