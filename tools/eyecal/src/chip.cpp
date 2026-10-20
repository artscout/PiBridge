// SPDX-License-Identifier: MIT
#include "chip.h"

#include "pibridge/eye/frames.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::eyecal {

namespace fs = std::filesystem;
namespace cal = pibridge::eye::calibration;
namespace live = pibridge::eye::live;

namespace {

std::string read_small(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool all_digits(const std::string& s) { return !s.empty() && std::ranges::all_of(s, [](char c) { return c >= '0' && c <= '9'; }); }

} // namespace

std::optional<std::string> chip_owner(const fs::path& proc_root) {
    static constexpr std::array<std::string_view, 3> owners = {"pibridge-eyed", "pimax-eyed", "omarchyvr-eyed"};
    std::error_code ec;
    for (const auto& d : fs::directory_iterator(proc_root, ec)) {
        const std::string pid = d.path().filename().string();
        if (!all_digits(pid) || pid == std::to_string(::getpid())) {
            continue;
        }
        std::string comm = read_small(d.path() / "comm");
        while (!comm.empty() && (comm.back() == '\n' || comm.back() == ' ')) {
            comm.pop_back();
        }
        for (const auto o : owners) {
            // the kernel cuts comm at 15 characters
            if (!comm.empty() && comm == o.substr(0, 15)) {
                return std::string(o);
            }
        }
        if (comm == "omarchyvr") {
            // `omarchyvr eyed ...`: the second argument names the command
            const std::string cmd = read_small(d.path() / "cmdline");
            const auto z = cmd.find('\0');
            if (z != std::string::npos && cmd.compare(z + 1, 4, "eyed") == 0 && (cmd.size() == z + 5 || cmd[z + 5] == '\0')) {
                return std::string("omarchyvr eyed");
            }
        }
    }
    return std::nullopt;
}

std::string owner_message(std::string_view owner) {
    return std::format("{} is using the eye tracker. The eye chip has one owner at a time: stop {} first (the PiBridge "
                       "app does this for you when you press Calibrate), then start the calibration again.",
                       owner, owner);
}

// ------------------------------------------------------------------------------------------------ device

ChannelDevice::ChannelDevice(std::unique_ptr<live::Device> inner, std::shared_ptr<cal::Link> link)
    : inner_(std::move(inner)), link_(std::move(link)) {}

std::expected<std::unique_ptr<pibridge::eye::Transport>, std::string> ChannelDevice::open(const live::Config& cfg,
                                                                                    std::stop_token stop) {
    auto t = inner_->open(cfg, stop);
    if (!t) {
        return std::unexpected(t.error());
    }
    std::shared_ptr<pibridge::eye::Transport> inner(std::move(*t));
    return std::make_unique<cal::Channel>(std::move(inner), link_);
}

std::expected<std::string, std::string> ChannelDevice::reset(const live::Config& cfg) { return inner_->reset(cfg); }

std::optional<std::string> ChannelDevice::absent(const live::Config& cfg, pibridge::eye::Clock::time_point now) {
    return inner_->absent(cfg, now);
}

std::optional<std::string> ChannelDevice::silent(const live::Config& cfg, pibridge::eye::Clock::time_point now) {
    return inner_->silent(cfg, now);
}

// ------------------------------------------------------------------------------------------------ driver

void ChipDriver::request(const cal::Step& step) {
    auto t = link_->submit(step);
    if (!t) {
        immediate_.push_back(Result{0, std::unexpected(t.error())});
        return;
    }
    queue_.push_back(*t);
}

std::optional<ChipDriver::Result> ChipDriver::poll() {
    if (!immediate_.empty()) {
        auto r = std::move(immediate_.front());
        immediate_.pop_front();
        return r;
    }
    if (queue_.empty()) {
        return std::nullopt;
    }
    const std::uint64_t ticket = queue_.front();
    const std::uint32_t seq = link_->seq_of(ticket);
    auto r = link_->poll(ticket);
    if (!r) {
        return std::nullopt;
    }
    queue_.pop_front();
    return Result{seq, std::move(*r)};
}

void ChipDriver::follow(live::State tracker) {
    const bool streaming = tracker == live::State::Streaming;
    if (streaming && link_->attached() && !link_->session_open()) {
        (void)link_->open_session();
    } else if (!streaming && link_->session_open() && queue_.empty()) {
        link_->close_session();
    }
}

// ------------------------------------------------------------------------------------------------ profile file

std::expected<void, std::string> write_private(const fs::path& path, pibridge::eye::ByteView bytes) {
    std::error_code ec;
    if (path.has_parent_path()) {
        fs::create_directories(path.parent_path(), ec);
        if (ec) {
            return std::unexpected(std::format("{}: {}", path.parent_path().string(), ec.message()));
        }
    }
    const fs::path tmp = path.parent_path() / std::format(".{}.tmp-{}", path.filename().string(), ::getpid());
    const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) {
        return std::unexpected(std::format("{}: {}", tmp.string(), std::strerror(errno)));
    }
    bool ok = ::fchmod(fd, 0600) == 0;
    std::size_t off = 0;
    while (ok && off < bytes.size()) {
        const ssize_t n = ::write(fd, bytes.data() + off, bytes.size() - off);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        ok = n > 0;
        off += ok ? static_cast<std::size_t>(n) : 0;
    }
    ok = ok && ::fsync(fd) == 0;
    const int err = errno;
    ::close(fd);
    if (!ok) {
        fs::remove(tmp, ec);
        return std::unexpected(std::format("{}: {}", tmp.string(), std::strerror(err)));
    }
    if (::rename(tmp.c_str(), path.c_str()) != 0) {
        const int e = errno;
        fs::remove(tmp, ec);
        return std::unexpected(std::format("{}: {}", path.string(), std::strerror(e)));
    }
    return {};
}

std::expected<std::optional<fs::path>, std::string> ProfileStore::save(pibridge::eye::ByteView body, std::string_view stamp) {
    if (auto c = pibridge::eye::frames::check_profile(body); !c) {
        return std::unexpected("the new profile is not valid (" + c.error() + "); the old file stays");
    }
    std::optional<fs::path> made;
    std::error_code ec;
    if (kept_) {
        // a new calibration after a kept one (the check grid's A after B): the kept profile is "the previous one" now
        saved_ = false;
        kept_ = false;
        backup_.reset();
    }
    if (!saved_) {
        had_previous_ = fs::is_regular_file(path_, ec);
        if (had_previous_) {
            const std::string name = path_.filename().string();
            fs::path b = path_.parent_path() / std::format("{}.bak-{}", name, stamp);
            for (int n = 1; fs::exists(b, ec); n++) {
                b = path_.parent_path() / std::format("{}.bak-{}-{}", name, stamp, n);
            }
            const std::string old = read_small(path_);
            if (auto w = write_private(b, pibridge::eye::ByteView(reinterpret_cast<const std::uint8_t*>(old.data()), old.size()));
                !w) {
                return std::unexpected("backup of the old profile: " + w.error());
            }
            backup_ = b;
            made = b;
        }
    }
    if (auto w = write_private(path_, body); !w) {
        return std::unexpected(w.error());
    }
    saved_ = true;
    return made;
}

std::expected<std::string, std::string> ProfileStore::rollback() {
    if (!saved_ || kept_) {
        return std::string("nothing to undo");
    }
    std::error_code ec;
    if (had_previous_ && backup_) {
        const std::string old = read_small(*backup_);
        if (auto w = write_private(path_, pibridge::eye::ByteView(reinterpret_cast<const std::uint8_t*>(old.data()), old.size()));
            !w) {
            return std::unexpected("putting the previous profile back: " + w.error());
        }
        saved_ = false;
        return std::format("the previous profile is back in {} (copy kept as {})", path_.string(), backup_->string());
    }
    fs::remove(path_, ec);
    if (ec) {
        return std::unexpected(std::format("{}: {}", path_.string(), ec.message()));
    }
    saved_ = false;
    return std::format("{} removed (there was no profile before)", path_.string());
}

} // namespace pibridge::eyecal
