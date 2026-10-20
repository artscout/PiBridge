# SPDX-License-Identifier: MIT
# libpibridge-room: the room setup (boundary, play area, saved rooms) without graphics or runtime. Shared by the app's
# Room page and pibridge-roomsetup (the in-headset editor). Large parts are ported from ZoneTweak (MIT,
# Copyright (c) 2026 killdano; see NOTICE and docs/ROOM-SETUP.md).

add_library(pibridge_room STATIC
    src/geom.cpp
    src/json.cpp
    src/files.cpp
    src/chap.cpp
    src/rooms.cpp
    src/zone.cpp
    src/vmath.cpp
    src/draw.cpp
    src/pose.cpp
    src/editor.cpp
    src/scene.cpp
    src/raster.cpp
)
target_include_directories(pibridge_room PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_room PUBLIC cxx_std_23)

if(PIBRIDGE_BUILD_TESTS)
    foreach(t geom json chap rooms zone editor pose scene)
        add_executable(pibridge_room_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_room_test_${t} PRIVATE pibridge_room pibridge_common)
        add_test(NAME pibridge_room_${t} COMMAND pibridge_room_test_${t})
        # numbers must not follow the locale
        set_tests_properties(pibridge_room_${t} PROPERTIES ENVIRONMENT "LC_ALL=de_DE.UTF-8;LC_NUMERIC=de_DE.UTF-8")
    endforeach()
endif()
