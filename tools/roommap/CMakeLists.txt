# SPDX-License-Identifier: MIT
# The room-map tools, ports of omarchyvr's (MIT OR Apache-2.0): pibridge-anchor (omarchyvr-anchor status/clear),
# pibridge-room (omarchyvr-room) and pibridge-roommap (omarchyvr-roommap). Options, output and files are the Rust tools'
# (docs/pibridge-roommap.md).

add_executable(pibridge-anchor src/anchor_main.cpp)
target_link_libraries(pibridge-anchor PRIVATE pibridge_anchor)
target_compile_options(pibridge-anchor PRIVATE -ffp-contract=off -fwrapv)
install(TARGETS pibridge-anchor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(TARGET pibridge_roommap)
    add_executable(pibridge-room src/room_main.cpp)
    target_link_libraries(pibridge-room PRIVATE pibridge_roommap)
    target_compile_options(pibridge-room PRIVATE -ffp-contract=off -fwrapv)
    install(TARGETS pibridge-room RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    add_executable(pibridge-roommap src/roommap_main.cpp src/roommap_floor.cpp src/roommap_setup.cpp)
    target_link_libraries(pibridge-roommap PRIVATE pibridge_roommap pibridge_ctrltrack)
    target_compile_options(pibridge-roommap PRIVATE -ffp-contract=off -fwrapv)
    install(TARGETS pibridge-roommap RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
