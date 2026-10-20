// SPDX-License-Identifier: MIT
// HidHeadsetCommands on a temporary sysfs tree with a byte-vector writer: the eye-module enable goes out as the
// allowlisted report, anything else is refused before a write. Nothing opens a device node.
#include "hid_commands.h"

#include "pibridge_test.h"

#include <fstream>
#include <unistd.h>

using namespace pibridge::hmd;
using namespace pibridge::eyed;
namespace fs = std::filesystem;

namespace {

fs::path temp_dir(const std::string& name) {
    const auto d = fs::temp_directory_path() / ("pibridge-eyed-" + name + "-" + std::to_string(::getpid()));
    fs::remove_all(d);
    fs::create_directories(d);
    return d;
}

void put(const fs::path& p, const std::string& s) {
    fs::create_directories(p.parent_path());
    std::ofstream(p) << s;
}

/// hidrawN of interface `iface` of a headset with product id `pid`.
void hidraw(const fs::path& root, const char* name, const char* pid, const char* iface) {
    const auto hid = root / "devices" / (std::string(name) + "-if") / "hid";
    fs::create_directories(hid);
    put(hid.parent_path() / "bInterfaceNumber", std::string(iface) + "\n");
    put(hid / "uevent", std::string("HID_ID=0003:000034A4:0000") + pid + "\nHID_NAME=Pimax\n");
    fs::create_directories(root / "class" / name);
    fs::create_directory_symlink(hid, root / "class" / name / "device");
}

struct Recorder {
    std::vector<fs::path> opened;
    std::vector<std::vector<std::uint8_t>> writes;
    HidHeadsetCommands::Opener opener() {
        return [this](const fs::path& node) -> std::expected<Gate::Writer, std::string> {
            opened.push_back(node);
            return Gate::Writer([this](std::span<const std::uint8_t> b) -> long {
                writes.emplace_back(b.begin(), b.end());
                return static_cast<long>(b.size());
            });
        };
    }
};

} // namespace

TEST(the_enable_goes_to_interface_0_of_the_crystal_super_as_the_allowlisted_report) {
    const auto dir = temp_dir("hid-cs");
    hidraw(dir, "hidraw4", "0040", "01");
    hidraw(dir, "hidraw5", "0040", "00");
    Recorder rec;
    HidHeadsetCommands h(discovery::Roots{dir / "class", dir / "dev"}, {PIBRIDGE_SOURCE_DEVICES_DIR}, rec.opener());
    CHECK(h.send("eye_power_on").has_value());
    CHECK(rec.opened == std::vector<fs::path>{dir / "dev/hidraw5"});
    CHECK(rec.writes.size() == 1);
    std::vector<std::uint8_t> want(65, 0);
    const std::uint8_t head[] = {0x00, 0x58, 0x05, 0x3c, 0x05, 0x00, 0x01};
    std::copy(std::begin(head), std::end(head), want.begin());
    CHECK(rec.writes[0] == want);
    CHECK(h.last_target() == "Pimax Crystal Super, " + (dir / "dev/hidraw5").string());
    fs::remove_all(dir);
}

TEST(the_dream_air_takes_the_same_enable) {
    const auto dir = temp_dir("hid-da");
    hidraw(dir, "hidraw0", "0044", "00");
    Recorder rec;
    HidHeadsetCommands h(discovery::Roots{dir / "class", dir / "dev"}, {PIBRIDGE_SOURCE_DEVICES_DIR}, rec.opener());
    CHECK(h.send("eye_power_on").has_value());
    CHECK(rec.writes.size() == 1 && rec.writes[0][1] == 0x58 && rec.writes[0][6] == 0x01);
    fs::remove_all(dir);
}

TEST(unknown_commands_and_missing_headsets_write_nothing) {
    const auto dir = temp_dir("hid-refuse");
    Recorder rec;
    HidHeadsetCommands none(discovery::Roots{dir / "class", dir / "dev"}, {PIBRIDGE_SOURCE_DEVICES_DIR}, rec.opener());
    const auto r = none.send("eye_power_on");
    CHECK(!r.has_value() && r.error().code == pibridge::eye::Errc::not_found);
    hidraw(dir, "hidraw5", "0040", "00");
    HidHeadsetCommands h(discovery::Roots{dir / "class", dir / "dev"}, {PIBRIDGE_SOURCE_DEVICES_DIR}, rec.opener());
    const auto u = h.send("eye_power_off"); // never captured, never listed
    CHECK(!u.has_value() && u.error().code == pibridge::eye::Errc::permission_denied);
    CHECK(rec.opened.empty() && rec.writes.empty());
    // no device description: nothing can be sent
    HidHeadsetCommands nodesc(discovery::Roots{dir / "class", dir / "dev"}, {dir / "no-devices"}, rec.opener());
    CHECK(!nodesc.send("eye_power_on").has_value());
    CHECK(rec.writes.empty());
    // a failing write is an OS error with the node
    HidHeadsetCommands bad(discovery::Roots{dir / "class", dir / "dev"}, {PIBRIDGE_SOURCE_DEVICES_DIR},
                           [](const fs::path&) -> std::expected<Gate::Writer, std::string> {
                               return Gate::Writer([](std::span<const std::uint8_t>) -> long { return -EPIPE; });
                           });
    const auto w = bad.send("eye_power_on");
    CHECK(!w.has_value() && w.error().os_errno == EPIPE);
    fs::remove_all(dir);
}

PIBRIDGE_TEST_MAIN()
