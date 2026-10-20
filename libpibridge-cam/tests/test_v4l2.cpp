// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/v4l2.rs (MIT OR Apache-2.0). No device is opened: pick_node is pure
// and the open test uses a path that does not exist.
#include "pibridge/cam/v4l2.h"
#include "pibridge_test.h"

#include <linux/videodev2.h>

using namespace pibridge::cam;

TEST(abi_matches_videodev2_h) {
    // The Rust code declares these structs by hand; libpibridge-cam uses <linux/videodev2.h> itself (x86_64 values).
    CHECK(sizeof(v4l2_capability) == 104);
    CHECK(offsetof(v4l2_capability, device_caps) == 88);
    CHECK(sizeof(v4l2_format) == 208);
    CHECK(offsetof(v4l2_format, fmt) == 8);
    CHECK(sizeof(v4l2_pix_format) == 48);
    CHECK(sizeof(v4l2_requestbuffers) == 20);
    CHECK(sizeof(v4l2_buffer) == 88);
    CHECK(offsetof(v4l2_buffer, timestamp) == 24);
    CHECK(offsetof(v4l2_buffer, timecode) == 40);
    CHECK(offsetof(v4l2_buffer, sequence) == 56);
    CHECK(offsetof(v4l2_buffer, memory) == 60);
    CHECK(offsetof(v4l2_buffer, m) == 64);
    CHECK(offsetof(v4l2_buffer, length) == 72);
    CHECK(sizeof(v4l2_streamparm) == 204);
    CHECK(offsetof(v4l2_streamparm, parm) == 4);
    CHECK(sizeof(v4l2_captureparm) == 40);
    CHECK(offsetof(v4l2_captureparm, timeperframe) == 8);
    CHECK(sizeof(v4l2_frmivalenum) == 52);
    // the ioctl numbers encode the struct sizes (the Rust constants)
    CHECK(VIDIOC_QUERYCAP == 0x80685600ul);
    CHECK(VIDIOC_G_FMT == 0xc0d05604ul);
    CHECK(VIDIOC_S_FMT == 0xc0d05605ul);
    CHECK(VIDIOC_REQBUFS == 0xc0145608ul);
    CHECK(VIDIOC_QUERYBUF == 0xc0585609ul);
    CHECK(VIDIOC_QBUF == 0xc058560ful);
    CHECK(VIDIOC_DQBUF == 0xc0585611ul);
    CHECK(VIDIOC_STREAMON == 0x40045612ul);
    CHECK(VIDIOC_STREAMOFF == 0x40045613ul);
    CHECK(VIDIOC_G_PARM == 0xc0cc5615ul);
    CHECK(VIDIOC_S_PARM == 0xc0cc5616ul);
    CHECK(VIDIOC_ENUM_FRAMEINTERVALS == 0xc034564bul);
}

static NodeEntry entry(std::uint32_t n, std::optional<std::uint32_t> caps, std::optional<std::uint32_t> index,
                       std::string_view pid) {
    return NodeEntry{"/dev/video" + std::to_string(n), std::string(CAM_VID), std::string(pid), caps, index};
}

TEST(pick_node_prefers_capture_over_metadata) {
    const std::uint32_t cap = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;
    const std::uint32_t meta = 0x0000'0008 | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS; // V4L2_CAP_META_CAPTURE
    // metadata node listed first must be skipped
    CHECK(pick_node({entry(0, meta, 0, CAM_PID), entry(1, cap, 1, CAM_PID)}) == std::optional<std::string>("/dev/video1"));
    CHECK(pick_node({entry(0, cap, 0, CAM_PID), entry(1, meta, 1, CAM_PID)}) == std::optional<std::string>("/dev/video0"));
    // unknown caps: sysfs index 0
    CHECK(pick_node({entry(1, std::nullopt, 1, CAM_PID), entry(0, std::nullopt, 0, CAM_PID)}) ==
          std::optional<std::string>("/dev/video0"));
    // other devices ignored, nothing capable = None
    CHECK(!pick_node({entry(2, cap, 0, "ffff")}));
    CHECK(!pick_node({entry(0, meta, 0, CAM_PID)}));
    CHECK(!pick_node({}));
}

TEST(open_nonexistent_fails) {
    const auto r = V4l2Capture::open("/dev/does-not-exist-pimax", 0, 4);
    CHECK(!r);
    CHECK(r.error().text() == "No such file or directory (os error 2)");
}

PIBRIDGE_TEST_MAIN()
