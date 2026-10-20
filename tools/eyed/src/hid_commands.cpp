// SPDX-License-Identifier: MIT
#include "hid_commands.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/sys.h"

#include <cerrno>
#include <format>
#include <memory>

#include <unistd.h>

namespace pibridge::eyed {

namespace fs = std::filesystem;
using pibridge::eye::Errc;
using PError = pibridge::eye::Error;

std::expected<Gate::Writer, std::string> open_hidraw(const fs::path& node) {
    auto fd = hid::open(node, true);
    if (!fd) {
        return std::unexpected(std::format("open {}: {}", node.string(), fd.error().message()));
    }
    auto shared = std::make_shared<Fd>(std::move(*fd));
    return Gate::Writer([shared](std::span<const std::uint8_t> b) -> long {
        const ssize_t n = ::write(shared->get(), b.data(), b.size());
        return n < 0 ? -static_cast<long>(errno) : static_cast<long>(n);
    });
}

HidHeadsetCommands::HidHeadsetCommands(discovery::Roots roots, std::vector<fs::path> devices_dirs, Opener open)
    : roots_(std::move(roots)), dirs_(devices_dirs.empty() ? pibridge::hmd::devices_dirs() : std::move(devices_dirs)),
      open_(open ? std::move(open) : Opener(open_hidraw)) {}

pibridge::eye::Result<void> HidHeadsetCommands::send(std::string_view id) {
    const auto found = discovery::find_headset(roots_);
    if (!found || found->model == nullptr) {
        return std::unexpected(PError::make(Errc::not_found, "no Pimax headset (34a4:0040 or 34a4:0044) on hidraw"));
    }
    const Model& model = *found->model;
    const auto allow = load_allowlist(model, dirs_);
    if (!allow) {
        return std::unexpected(PError::make(
            Errc::not_found, std::format("no device description for the {}: {}", model.name, allow.error().msg)));
    }
    const CommandSpec* spec = (*allow)->get(id);
    if (spec == nullptr) {
        return std::unexpected(PError::make(
            Errc::permission_denied, std::format("'{}' is not in the allowlist of the {}", id, model.name)));
    }
    const auto report = (*allow)->build(id);
    if (!report) {
        return std::unexpected(PError::make(Errc::permission_denied, report.error().message()));
    }
    std::optional<fs::path> node;
    if (spec->interface == 0) {
        node = pibridge::eye::power::headset_command_node(roots_.hidraw_class, roots_.dev);
    }
    if (!node && spec->interface < found->nodes.size()) {
        node = found->nodes[spec->interface];
    }
    if (!node) {
        return std::unexpected(PError::make(
            Errc::not_found, std::format("no hidraw node for interface {} of the {}", spec->interface, model.name)));
    }
    auto writer = open_(*node);
    if (!writer) {
        return std::unexpected(PError::make(Errc::io, writer.error()));
    }
    Gate gate(**allow, spec->interface, std::move(*writer));
    if (auto r = gate.send(*report); !r) {
        if (const auto* ref = r.error().refusal()) {
            return std::unexpected(PError::make(Errc::permission_denied, ref->message()));
        }
        return std::unexpected(PError::os(std::get<int>(r.error().e), "write " + node->string()));
    }
    std::lock_guard l(mu_);
    last_ = std::format("{}, {}", model.name, node->string());
    return {};
}

std::string HidHeadsetCommands::last_target() const {
    std::lock_guard l(mu_);
    return last_;
}

} // namespace pibridge::eyed
