// SPDX-License-Identifier: MIT
// power::HeadsetCommands over libpibridge-hmd: the eye-module enable (and only allowlisted commands) to the headset.
//
// libpibridge-eye never writes HID itself (power.h). This is the one implementation: it finds the headset with
// pibridge::hmd::discovery::find_headset, takes that model's allowlist (`devices/<id>.toml`, pibridge::hmd::load_allowlist), builds
// the named command from it, and writes it through pibridge::hmd::Gate to the command interface (interface 0, the node
// power::headset_command_node finds). A command the description does not list is refused and nothing is written.
// The node is opened for that one write and closed again (the keepalive daemon keeps its own handle; hidraw allows
// several writers), like the Rust `power::send_enable`.
#pragma once

#include "pibridge/hmd/allowlist.h"
#include "pibridge/hmd/discovery.h"

#include "pibridge/eye/power.h"

#include <expected>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace pibridge::eyed {

using namespace pibridge::hmd; // built on libpibridge-hmd (discovery, allowlist gate, hidraw)

class HidHeadsetCommands final : public pibridge::eye::power::HeadsetCommands {
public:
    /// Opens a node for one write; the writer returns bytes written or -errno (pibridge::hmd::Gate::Writer). The default
    /// opens the hidraw node read-write.
    using Opener = std::function<std::expected<Gate::Writer, std::string>(const std::filesystem::path& node)>;

    explicit HidHeadsetCommands(discovery::Roots roots = discovery::Roots::from_env(),
                                std::vector<std::filesystem::path> devices_dirs = {}, Opener open = {});

    /// Send the allowlist entry `id` once to the headset that is on the bus now.
    pibridge::eye::Result<void> send(std::string_view id) override;

    /// "Pimax Crystal Super, /dev/hidraw3" of the last successful send (for the log); empty before.
    std::string last_target() const;

private:
    discovery::Roots roots_;
    std::vector<std::filesystem::path> dirs_;
    Opener open_;
    mutable std::mutex mu_;
    std::string last_;
};

/// The default Opener: pibridge::hmd::hid::open(node, write = true).
std::expected<Gate::Writer, std::string> open_hidraw(const std::filesystem::path& node);

} // namespace pibridge::eyed
