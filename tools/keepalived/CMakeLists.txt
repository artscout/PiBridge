# SPDX-License-Identifier: MIT
# pibridge-keepalived: keeps the headset awake while the followed VR runtime runs (port of `omarchyvr keepalived`).

add_library(pibridge_keepalived_core STATIC src/keepalived.cpp src/crystal.cpp)
target_include_directories(pibridge_keepalived_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pibridge_keepalived_core PUBLIC pibridge_hmd pibridge_common)

add_executable(pibridge-keepalived src/main.cpp)
target_link_libraries(pibridge-keepalived PRIVATE pibridge_keepalived_core)

install(TARGETS pibridge-keepalived RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# the shipped unit names ~/.local/bin; the installed copy names the real bindir
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/pibridge-keepalived.service PIBRIDGE_KA_UNIT)
string(REPLACE "%h/.local/bin/pibridge-keepalived" "${CMAKE_INSTALL_FULL_BINDIR}/pibridge-keepalived" PIBRIDGE_KA_UNIT "${PIBRIDGE_KA_UNIT}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/pibridge-keepalived.service "${PIBRIDGE_KA_UNIT}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/pibridge-keepalived.service)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pibridge-keepalived.service DESTINATION ${PIBRIDGE_SYSTEMD_USER_UNIT_DIR})

if(PIBRIDGE_BUILD_TESTS)
    add_executable(pibridge_keepalived_test tests/test_keepalived.cpp)
    target_link_libraries(pibridge_keepalived_test PRIVATE pibridge_keepalived_core)
    target_compile_definitions(pibridge_keepalived_test PRIVATE PIBRIDGE_SOURCE_DEVICES_DIR="${PIBRIDGE_SOURCE_DEVICES_DIR}")
    add_test(NAME pibridge_keepalived COMMAND pibridge_keepalived_test)
endif()
