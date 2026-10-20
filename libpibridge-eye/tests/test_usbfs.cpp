// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/usbfs.rs tests (MIT OR Apache-2.0). No device node is ever opened here.
#include "pibridge/eye/usbfs.h"

#include "pibridge_test.h"
#include "testutil.h"

using namespace pibridge::eye::usbfs;
namespace fs = std::filesystem;

TEST(ioctl_numbers_match_linux_usbdevice_fs_h) {
    // values printed by a C program including <linux/usbdevice_fs.h> on x86_64
    if (sizeof(void*) == 8) {
        CHECK(ctrl_transfer_size() == 24);
        CHECK(bulk_transfer_size() == 24);
        CHECK(ioctl_control() == 0xc0185500ul);
        CHECK(ioctl_bulk() == 0xc0185502ul);
    }
    CHECK(ioctl_claim_interface() == 0x8004550ful);
    CHECK(ioctl_release_interface() == 0x80045510ul);
    CHECK(ioctl_reset() == 0x5514ul);
}

TEST(sysfs_scan_reads_a_fake_tree) {
    const auto dir = testutil::temp_dir("sysfs");
    const auto dev = dir / "1-5.1.4.3.4";
    fs::create_directories(dev);
    const std::pair<const char*, const char*> attrs[] = {
        {"idVendor", "2104\n"}, {"idProduct", "0561\n"}, {"busnum", "1\n"},     {"devnum", "12\n"},
        {"product", "EyeChip\n"}, {"serial", "TEST0000\n"}, {"speed", "480\n"},
    };
    for (const auto& [k, v] : attrs) {
        const std::string s = v;
        testutil::write_bytes(dev / k, std::span(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()));
    }
    fs::create_directories(dir / "usb1"); // no idVendor: skipped
    const auto found = scan_sysfs(dir, "/dev/bus/usb");
    CHECK(found.size() == 1);
    const auto& d = found[0];
    CHECK(d.vid == 0x2104 && d.pid == 0x0561 && d.bus == 1 && d.devnum == 12);
    CHECK(d.devnode == fs::path("/dev/bus/usb/001/012"));
    CHECK(d.product == std::optional<std::string>("EyeChip"));
    CHECK(!d.interface_driver());
    fs::remove_all(dir);
}

TEST(missing_sysfs_root_is_empty) {
    CHECK(scan_sysfs("/nonexistent-pibridge-eye", "/dev").empty());
}

TEST(opening_a_missing_node_is_an_error) {
    // a path that cannot exist: nothing real is opened
    const auto r = UsbFs::open("/nonexistent-pibridge-eye/001/001");
    CHECK(!r && r.error().code == pibridge::eye::Errc::not_found);
}

PIBRIDGE_TEST_MAIN()
