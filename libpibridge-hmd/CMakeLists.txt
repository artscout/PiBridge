# SPDX-License-Identifier: MIT
# libpibridge-hmd: HID I/O for Pimax headsets (protocol frames, discovery, hidraw, allowlist gate, session, audio, follow,
# settings). Ported from omarchyvr pimax-proto, pimax, pimax-device and omarchyvr-tools (MIT OR Apache-2.0).

find_package(ZLIB QUIET)

add_library(pibridge_hmd STATIC
    src/proto.cpp
    src/sys.cpp
    src/discovery.cpp
    src/toml_lite.cpp
    src/allowlist.cpp
    src/descriptor.cpp
    src/haptics.cpp
    src/pairing.cpp
    src/ipd.cpp
    src/session.cpp
    src/fake.cpp
    src/audio.cpp
    src/follow.cpp
    src/settings.cpp
)
target_include_directories(pibridge_hmd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pibridge_hmd PUBLIC cxx_std_23)
target_link_libraries(pibridge_hmd PRIVATE pibridge_common) # pibridge_env.h
target_compile_definitions(pibridge_hmd PRIVATE PIBRIDGE_INSTALLED_DEVICES_DIR="${PIBRIDGE_DEVICES_DIR}")
if(ZLIB_FOUND)
    # calibration document inflate (calib::inflate_blob / build_blob)
    target_link_libraries(pibridge_hmd PRIVATE ZLIB::ZLIB)
    target_compile_definitions(pibridge_hmd PRIVATE PIBRIDGE_HAVE_ZLIB=1)
else()
    target_compile_definitions(pibridge_hmd PRIVATE PIBRIDGE_HAVE_ZLIB=0)
endif()

if(PIBRIDGE_BUILD_TESTS)
    # Public captures that are too big to copy (p82/p83/ctrl_*): read from an omarchyvr checkout next to this one when
    # present; those tests report "skipped" without it.
    set(PIBRIDGE_FIXTURES_DIR "${PROJECT_SOURCE_DIR}/../omarchyvr/rust/fixtures" CACHE PATH
        "omarchyvr rust/fixtures (optional, for the real-capture tests)")
    foreach(t proto fixtures discovery toml allowlist session close audio follow settings haptics pairing)
        add_executable(pibridge_hmd_test_${t} tests/test_${t}.cpp)
        target_link_libraries(pibridge_hmd_test_${t} PRIVATE pibridge_hmd pibridge_common)
        target_compile_definitions(pibridge_hmd_test_${t} PRIVATE
            PIBRIDGE_SOURCE_DEVICES_DIR="${PIBRIDGE_SOURCE_DEVICES_DIR}"
            PIBRIDGE_HMD_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
            PIBRIDGE_FIXTURES_DIR="${PIBRIDGE_FIXTURES_DIR}"
            PIBRIDGE_HAVE_ZLIB=$<BOOL:${ZLIB_FOUND}>)
        if(ZLIB_FOUND)
            target_link_libraries(pibridge_hmd_test_${t} PRIVATE ZLIB::ZLIB)
        endif()
        add_test(NAME pibridge_hmd_${t} COMMAND pibridge_hmd_test_${t})
    endforeach()
    # the decoder's twins of the Rust tests: without PiBridge's Menu-button bit (byte 47 bit 2)
    if(TEST pibridge_hmd_proto)
        set_tests_properties(pibridge_hmd_proto PROPERTIES ENVIRONMENT_MODIFICATION "PIBRIDGE_CTRL_PARITY=set:1")
    endif()
endif()
