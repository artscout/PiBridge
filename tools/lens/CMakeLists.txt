# SPDX-License-Identifier: MIT
# pibridge-lens: build the Crystal Super lens profile (and other LUT turns) from a like-Pimax LUT (docs/pibridge-lens.md).
add_executable(pibridge-lens src/main.cpp)
target_link_libraries(pibridge-lens PRIVATE pibridge_lens pibridge_common)
install(TARGETS pibridge-lens RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
