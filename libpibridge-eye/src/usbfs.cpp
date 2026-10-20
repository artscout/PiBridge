// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/usbfs.rs (MIT OR Apache-2.0).
#include "pibridge/eye/usbfs.h"

#include "pibridge/eye/replay.h"

#include "sysfs.h"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <format>
#include <limits>

#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace pibridge::eye::usbfs {

namespace fs = std::filesystem;

unsigned long ioctl_control() { return USBDEVFS_CONTROL; }
unsigned long ioctl_bulk() { return USBDEVFS_BULK; }
unsigned long ioctl_claim_interface() { return USBDEVFS_CLAIMINTERFACE; }
unsigned long ioctl_release_interface() { return USBDEVFS_RELEASEINTERFACE; }
unsigned long ioctl_reset() { return USBDEVFS_RESET; }
std::size_t ctrl_transfer_size() { return sizeof(usbdevfs_ctrltransfer); }
std::size_t bulk_transfer_size() { return sizeof(usbdevfs_bulktransfer); }

namespace {

// ioctl with EINTR retry; returns the non-negative result or errno.
std::expected<int, int> xioctl(int fd, unsigned long request, void* arg) {
    for (;;) {
        const int rc = ::ioctl(fd, request, arg);
        if (rc >= 0) {
            return rc;
        }
        if (errno != EINTR) {
            return std::unexpected(errno);
        }
    }
}

unsigned millis(Duration t) {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t).count();
    if (ms <= 1) {
        return 1;
    }
    return ms > std::numeric_limits<unsigned>::max() ? std::numeric_limits<unsigned>::max()
                                                     : static_cast<unsigned>(ms);
}

std::expected<int, int> open_rw(const fs::path& path) {
    for (;;) {
        const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd >= 0) {
            return fd;
        }
        if (errno != EINTR) {
            return std::unexpected(errno);
        }
    }
}

} // namespace

Result<UsbFs> UsbFs::open(const fs::path& path) {
    auto fd = open_rw(path);
    if (!fd) {
        return std::unexpected(Error::os(fd.error(), path.string()));
    }
    return UsbFs(*fd);
}

UsbFs::UsbFs(UsbFs&& o) noexcept : fd_(o.fd_), claimed_(o.claimed_) {
    o.fd_ = -1;
    o.claimed_.reset();
}

UsbFs& UsbFs::operator=(UsbFs&& o) noexcept {
    if (this != &o) {
        close_fd();
        fd_ = o.fd_;
        claimed_ = o.claimed_;
        o.fd_ = -1;
        o.claimed_.reset();
    }
    return *this;
}

UsbFs::~UsbFs() {
    close_fd();
}

void UsbFs::close_fd() {
    if (fd_ < 0) {
        return;
    }
    if (claimed_) {
        unsigned int n = *claimed_;
        (void)xioctl(fd_, USBDEVFS_RELEASEINTERFACE, &n);
        claimed_.reset();
    }
    ::close(fd_);
    fd_ = -1;
}

Result<void> UsbFs::claim_interface(std::uint32_t interface) {
    unsigned int n = interface;
    if (auto r = xioctl(fd_, USBDEVFS_CLAIMINTERFACE, &n); !r) {
        return std::unexpected(Error::os(r.error(), std::format("claim interface {}", interface)));
    }
    claimed_ = interface;
    return {};
}

Result<Bytes> UsbFs::control(const Setup& setup, ByteView out, Duration timeout) {
    const bool is_in = (setup[0] & 0x80) != 0;
    const auto w_length = static_cast<std::uint16_t>(setup[6] | (setup[7] << 8));
    if (!is_in && out.size() != w_length) {
        return std::unexpected(Error::make(Errc::invalid_input, "control OUT data length differs from wLength"));
    }
    Bytes buf = is_in ? Bytes(w_length) : Bytes(out.begin(), out.end());
    usbdevfs_ctrltransfer t{};
    t.bRequestType = setup[0];
    t.bRequest = setup[1];
    t.wValue = static_cast<std::uint16_t>(setup[2] | (setup[3] << 8));
    t.wIndex = static_cast<std::uint16_t>(setup[4] | (setup[5] << 8));
    t.wLength = w_length;
    t.timeout = millis(timeout);
    t.data = buf.data();
    auto n = xioctl(fd_, USBDEVFS_CONTROL, &t);
    if (!n) {
        return std::unexpected(Error::os(n.error(), "control transfer"));
    }
    if (is_in) {
        buf.resize(std::min<std::size_t>(buf.size(), static_cast<std::size_t>(*n)));
        return buf;
    }
    return Bytes{};
}

Result<void> UsbFs::bulk_out(std::uint8_t ep, ByteView data, Duration timeout) {
    if (data.size() > std::numeric_limits<unsigned>::max()) {
        return std::unexpected(Error::make(Errc::invalid_input, "bulk OUT too large"));
    }
    Bytes buf(data.begin(), data.end());
    usbdevfs_bulktransfer t{};
    t.ep = ep;
    t.len = static_cast<unsigned>(buf.size());
    t.timeout = millis(timeout);
    t.data = buf.data();
    auto n = xioctl(fd_, USBDEVFS_BULK, &t);
    if (!n) {
        return std::unexpected(Error::os(n.error(), "bulk OUT"));
    }
    if (static_cast<std::size_t>(*n) != data.size()) {
        return std::unexpected(
            Error::make(Errc::short_transfer, std::format("short bulk OUT: {} of {} bytes", *n, data.size())));
    }
    return {};
}

Result<std::optional<Bytes>> UsbFs::bulk_in(std::uint8_t ep, std::size_t max_len, Duration timeout) {
    if (max_len > std::numeric_limits<unsigned>::max()) {
        return std::unexpected(Error::make(Errc::invalid_input, "bulk IN too large"));
    }
    Bytes buf(max_len);
    usbdevfs_bulktransfer t{};
    t.ep = ep;
    t.len = static_cast<unsigned>(max_len);
    t.timeout = millis(timeout);
    t.data = buf.data();
    auto n = xioctl(fd_, USBDEVFS_BULK, &t);
    if (!n) {
        if (n.error() == ETIMEDOUT) {
            return std::optional<Bytes>{};
        }
        return std::unexpected(Error::os(n.error(), "bulk IN"));
    }
    buf.resize(std::min<std::size_t>(buf.size(), static_cast<std::size_t>(*n)));
    return std::optional<Bytes>(std::move(buf));
}

Result<void> reset_node(const fs::path& path) {
    auto fd = open_rw(path);
    if (!fd) {
        return std::unexpected(Error::os(fd.error(), path.string()));
    }
    auto r = xioctl(*fd, USBDEVFS_RESET, nullptr);
    ::close(*fd);
    if (!r) {
        return std::unexpected(Error::os(r.error(), "USB reset"));
    }
    return {};
}

std::optional<std::string> SysDevice::interface_driver() const {
    const std::string name = sysfs.filename().string();
    std::error_code ec;
    const auto link = fs::read_symlink(sysfs / (name + ":1.0") / "driver", ec);
    if (ec || link.filename().empty()) {
        return std::nullopt;
    }
    return link.filename().string();
}

std::vector<SysDevice> scan_sysfs(const fs::path& sysfs_root, const fs::path& dev_root) {
    std::vector<SysDevice> out;
    std::error_code ec;
    fs::directory_iterator it(sysfs_root, ec);
    if (ec) {
        return out;
    }
    for (const auto& entry : it) {
        const fs::path p = entry.path();
        const auto hex = [&](const char* name) -> std::optional<std::uint16_t> {
            return detail::parse_uint<std::uint16_t>(detail::read_trim(p / name), 16);
        };
        const auto dec = [&](const char* name) -> std::optional<std::uint32_t> {
            return detail::parse_uint<std::uint32_t>(detail::read_trim(p / name), 10);
        };
        const auto vid = hex("idVendor");
        const auto pid = hex("idProduct");
        const auto bus = dec("busnum");
        const auto devnum = dec("devnum");
        if (!vid || !pid || !bus || !devnum) {
            continue;
        }
        SysDevice d;
        d.vid = *vid;
        d.pid = *pid;
        d.bus = *bus;
        d.devnum = *devnum;
        d.product = detail::read_trim(p / "product");
        d.serial = detail::read_trim(p / "serial");
        d.speed = detail::read_trim(p / "speed");
        d.devnode = dev_root / std::format("{:03}", *bus) / std::format("{:03}", *devnum);
        d.sysfs = p;
        out.push_back(std::move(d));
    }
    std::ranges::sort(out, [](const SysDevice& a, const SysDevice& b) {
        return std::pair(a.bus, a.devnum) < std::pair(b.bus, b.devnum);
    });
    return out;
}

std::vector<SysDevice> find_eye_chips() {
    std::vector<SysDevice> out;
    for (auto& d : scan_sysfs("/sys/bus/usb/devices", "/dev/bus/usb")) {
        if (d.vid == replay::EYE_VID && d.pid == replay::EYE_PID) {
            out.push_back(std::move(d));
        }
    }
    return out;
}

} // namespace pibridge::eye::usbfs
