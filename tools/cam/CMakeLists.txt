# SPDX-License-Identifier: MIT
# pibridge-cam: the tracking-camera tool (port of omarchyvr's `pimax-cam`: info, stats, record, calib, import, broker,
# broker-status). Its output is the Rust tool's text; the broker speaks PXCB v1 with pimax-cam's clients and servers.

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
add_executable(pibridge-cam src/main.cpp)
target_link_libraries(pibridge-cam PRIVATE pibridge_cam)
target_compile_options(pibridge-cam PRIVATE -ffp-contract=off)
install(TARGETS pibridge-cam RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
