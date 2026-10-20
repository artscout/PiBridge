// SPDX-License-Identifier: MIT
// The calibration tool's side of the eye chip: who else holds it, the tracker it runs itself (with the calibration
// channel of pibridge::eye::calibration between the tracker and the chip), the requests of the procedure, and the
// profile file (saved atomically with a backup, kept or put back at the end).
#pragma once

#include "pibridge/eye/calibration.h"
#include "pibridge/eye/live.h"
#include "pibridge/eye/power.h"

#include <cstdint>
#include <deque>
#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace pibridge::eyecal {

/// The programs that own the eye chip while they run (one owner only): pibridge-eyed, its earlier name pimax-eyed,
/// omarchyvr-eyed, and `omarchyvr eyed` (the omarchyvr binary with the eyed command).
/// The first one running under `proc_root`, by name; empty when none runs.
std::optional<std::string> chip_owner(const std::filesystem::path& proc_root = "/proc");

/// The plain-words refusal when `owner` holds the chip.
std::string owner_message(std::string_view owner);

/// A live::Device that puts a calibration::Channel between the tracker and every chip it opens (the inner device does
/// the opening, the USB reset and the eye-module power, like RealDevice for pibridge-eyed).
class ChannelDevice final : public pibridge::eye::live::Device {
public:
    ChannelDevice(std::unique_ptr<pibridge::eye::live::Device> inner, std::shared_ptr<pibridge::eye::calibration::Link> link);
    std::expected<std::unique_ptr<pibridge::eye::Transport>, std::string> open(const pibridge::eye::live::Config& cfg,
                                                                           std::stop_token stop) override;
    std::expected<std::string, std::string> reset(const pibridge::eye::live::Config& cfg) override;
    std::optional<std::string> absent(const pibridge::eye::live::Config& cfg, pibridge::eye::Clock::time_point now) override;
    std::optional<std::string> silent(const pibridge::eye::live::Config& cfg, pibridge::eye::Clock::time_point now) override;

private:
    std::unique_ptr<pibridge::eye::live::Device> inner_;
    std::shared_ptr<pibridge::eye::calibration::Link> link_;
};

/// Carries the procedure's chip requests over a Link: request() submits (or fails at once), poll() hands back the
/// answers in order, with the sequence number each was sent with.
class ChipDriver {
public:
    explicit ChipDriver(std::shared_ptr<pibridge::eye::calibration::Link> link) : link_(std::move(link)) {}

    using Result = std::pair<std::uint32_t, std::expected<pibridge::eye::calibration::Answer, std::string>>;

    /// Send `step` (the session must be open).
    void request(const pibridge::eye::calibration::Step& step);
    /// The next result, once there is one.
    std::optional<Result> poll();
    /// Requests out and not answered.
    std::size_t outstanding() const { return queue_.size() + immediate_.size(); }
    /// The calibration session is open and the chip link is up.
    bool ready() const { return link_->attached() && link_->session_open(); }
    /// Open the session when the tracker streams (idempotent); close it otherwise.
    void follow(pibridge::eye::live::State tracker);

private:
    std::shared_ptr<pibridge::eye::calibration::Link> link_;
    std::deque<std::uint64_t> queue_;
    std::deque<Result> immediate_;
};

/// The eye profile file eyed uploads at every chip start: saving the chip's new profile, with a way back.
class ProfileStore {
public:
    explicit ProfileStore(std::filesystem::path path) : path_(std::move(path)) {}

    /// Check `body` (frames::check_profile; a bad one leaves the file alone) and write it atomically with mode 0600.
    /// The first save of a run copies the previous file to `<name>.bak-<stamp>` (0600) and remembers it; later saves
    /// of the same run overwrite the unkept new one. A save after keep() starts over: the kept file is the previous
    /// one (backed up, put back by rollback()). Returns the backup made by this call, if any.
    std::expected<std::optional<std::filesystem::path>, std::string> save(pibridge::eye::ByteView body,
                                                                        std::string_view stamp);
    /// The new profile stays.
    void keep() { kept_ = true; }
    /// Put the previous profile back (or remove the new file when there was none before). Nothing happens when
    /// nothing was saved or the new one is kept. The value says what was done.
    std::expected<std::string, std::string> rollback();

    const std::filesystem::path& path() const { return path_; }
    bool saved() const { return saved_; }
    bool kept() const { return kept_; }
    /// The copy of the previous profile (empty when there was none, or nothing was saved yet).
    const std::optional<std::filesystem::path>& backup() const { return backup_; }

private:
    std::filesystem::path path_;
    bool saved_ = false;
    bool kept_ = false;
    bool had_previous_ = false;
    std::optional<std::filesystem::path> backup_;
};

/// Write `bytes` to `path` through a temporary file in the same folder and a rename, with mode 0600.
std::expected<void, std::string> write_private(const std::filesystem::path& path, pibridge::eye::ByteView bytes);

} // namespace pibridge::eyecal
