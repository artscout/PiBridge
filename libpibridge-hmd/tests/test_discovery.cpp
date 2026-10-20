// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/tests/discovery.rs (MIT OR Apache-2.0). A fake sysfs tree in a temporary folder.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/discovery.h"

using namespace pibridge::hmd;
namespace fs = std::filesystem;

static void write(const fs::path& p, const std::string& s) {
    std::ofstream f(p);
    f << s;
}

static void add(const fs::path& root, const std::string& hidraw, const std::string& vid_pid, const std::string& iface,
                const std::string& bus) {
    const fs::path ifdir = root / ("sys/devices/pci/usb1/" + bus + "/" + bus + ":1." + iface);
    const fs::path hid = ifdir / ("0003:" + vid_pid + ".0001");
    fs::create_directories(hid);
    write(ifdir / "bInterfaceNumber", "0" + iface + "\n");
    write(hid / "uevent", "DRIVER=hid-generic\nHID_ID=0003:" + vid_pid + "\nHID_NAME=x\n");
    const fs::path cls = root / "sys/class/hidraw" / hidraw;
    fs::create_directories(cls);
    fs::create_directory_symlink(hid, cls / "device");
}

static std::string name(const std::optional<fs::path>& p) { return p ? p->string() : "-"; }

TEST(finds_four_interfaces_and_bridge_in_fake_sysfs) {
    const auto root = testutil::tmpdir("sysfs");
    for (const auto& [n, i] : std::vector<std::pair<int, std::string>>{{5, "0"}, {6, "1"}, {7, "2"}, {8, "3"}}) {
        add(root, "hidraw" + std::to_string(n), "000034A4:00000044", i, "1-1");
    }
    add(root, "hidraw2", "000034A4:00000043", "0", "1-2");
    add(root, "hidraw0", "0000046D:0000C52B", "0", "1-3"); // unrelated mouse
    fs::create_directories(root / "sys/class/hidraw/notahid");
    const discovery::Roots roots{root / "sys/class/hidraw", "/dev"};
    const auto f = discovery::find_interfaces(roots);
    CHECK(name(f[0]) == "/dev/hidraw5");
    CHECK(name(f[1]) == "/dev/hidraw6");
    CHECK(name(f[2]) == "/dev/hidraw7");
    CHECK(name(f[3]) == "/dev/hidraw8");
    CHECK(discovery::mask(f) == 0b1111);
    CHECK(name(discovery::find_bridge_log(roots)) == "/dev/hidraw2");
    fs::remove_all(root);
}

TEST(empty_or_missing_sysfs_finds_nothing) {
    const auto root = testutil::tmpdir("sysfs-empty");
    const discovery::Roots roots{root / "nope", "/dev"};
    CHECK(discovery::mask(discovery::find_interfaces(roots)) == 0);
    CHECK(!discovery::find_bridge_log(roots));
    fs::remove_all(root);
}

TEST(partial_interfaces_mask) {
    const auto root = testutil::tmpdir("sysfs-partial");
    add(root, "hidraw3", "000034A4:00000044", "0", "1-1");
    add(root, "hidraw4", "000034A4:00000044", "2", "1-1");
    const discovery::Roots roots{root / "sys/class/hidraw", root / "dev"};
    const auto f = discovery::find_interfaces(roots);
    CHECK(discovery::mask(f) == 0b0101);
    CHECK(f[2] && *f[2] == root / "dev/hidraw4");
    fs::remove_all(root);
}

TEST(finds_the_crystal_super_and_its_bridge) {
    const auto root = testutil::tmpdir("sysfs-crystal");
    for (const auto& [n, i] : std::vector<std::pair<int, std::string>>{{10, "0"}, {11, "1"}, {12, "2"}, {13, "3"}}) {
        add(root, "hidraw" + std::to_string(n), "000034A4:00000040", i, "3-3");
    }
    add(root, "hidraw14", "000034A4:00000041", "0", "3-4");
    const discovery::Roots roots{root / "sys/class/hidraw", "/dev"};
    const auto found = discovery::find_headset(roots);
    CHECK(found.has_value());
    CHECK(found->model->id == "crystal-super");
    CHECK(found->model->default_fps == 90);
    CHECK(discovery::mask(found->nodes) == 0b1111);
    CHECK(name(found->nodes[0]) == "/dev/hidraw10");
    CHECK(discovery::find_interfaces(roots) == found->nodes);
    CHECK(name(discovery::find_bridge_log(roots)) == "/dev/hidraw14");
    fs::remove_all(root);
}

TEST(usb_devnum_of_a_fake_bus) {
    const auto root = testutil::tmpdir("usbbus");
    fs::create_directories(root / "3-3");
    write(root / "3-3/idVendor", "34a4\n");
    write(root / "3-3/idProduct", "0040\n");
    write(root / "3-3/devnum", "17\n");
    fs::create_directories(root / "3-4");
    write(root / "3-4/idVendor", "34a4\n");
    write(root / "3-4/idProduct", "0041\n"); // the bridge is not the headset
    write(root / "3-4/devnum", "18\n");
    CHECK(discovery::usb_devnum(root) == std::optional<std::string>("17"));
    fs::remove_all(root / "3-3");
    CHECK(!discovery::usb_devnum(root));
    fs::remove_all(root);
}

PIBRIDGE_TEST_MAIN()
