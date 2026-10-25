# SPDX-License-Identifier: MIT
# pibridge-posetap: a read-only diagnostic that samples the pose shared memory (libpibridge-hmd pose_shm.h) and reports
# the head device's true update rate, latency (age), jitter and per-update orientation/position steps. Used to see
# where inside-out output is worse than it was with the Monado SLAM driver. It changes nothing and is safe on a live
# session.
add_executable(pibridge-posetap src/main.cpp)
target_link_libraries(pibridge-posetap PRIVATE pibridge_hmd)
install(TARGETS pibridge-posetap RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
