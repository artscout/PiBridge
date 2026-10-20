# SPDX-License-Identifier: MIT
# pibridge-eyed: the eye tracker daemon, gaze to shared memory (port of `omarchyvr eyed`). See docs/eyed.md.
# It links libpibridge-hmd for the headset command (the eye-module enable through the allowlist gate) and libpibridge-eye for
# the tracker; libpibridge-eye itself stays free of libpibridge-hmd.

add_library(pibridge_eyed_core STATIC src/eyed.cpp src/hid_commands.cpp)
target_include_directories(pibridge_eyed_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pibridge_eyed_core PUBLIC pibridge_hmd pibridge_eye pibridge_common)

add_executable(pibridge-eyed src/main.cpp)
target_link_libraries(pibridge-eyed PRIVATE pibridge_eyed_core)

install(TARGETS pibridge-eyed RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# the shipped unit names ~/.local/bin; the installed copy names the real bindir
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/pibridge-eyed.service PIBRIDGE_EYED_UNIT)
string(REPLACE "%h/.local/bin/pibridge-eyed" "${CMAKE_INSTALL_FULL_BINDIR}/pibridge-eyed" PIBRIDGE_EYED_UNIT "${PIBRIDGE_EYED_UNIT}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/pibridge-eyed.service "${PIBRIDGE_EYED_UNIT}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/pibridge-eyed.service)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pibridge-eyed.service DESTINATION ${PIBRIDGE_SYSTEMD_USER_UNIT_DIR})

if(PIBRIDGE_BUILD_TESTS)
    foreach(t eyed hid_commands)
        add_executable(pibridge_eyed_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_eyed_test_${t} PRIVATE pibridge_eyed_core)
        target_compile_definitions(pibridge_eyed_test_${t} PRIVATE PIBRIDGE_SOURCE_DEVICES_DIR="${PIBRIDGE_SOURCE_DEVICES_DIR}")
        add_test(NAME pibridge_eyed_${t} COMMAND pibridge_eyed_test_${t})
    endforeach()
endif()
