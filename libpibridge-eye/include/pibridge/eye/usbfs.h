// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/usbfs.rs (MIT OR Apache-2.0).
//
// Linux usbfs (`/dev/bus/usb/BBB/DDD`) access to one device, and sysfs discovery.
//
// A few ioctls on a file descriptor we own. It never detaches a kernel driver (USBDEVFS_DISCONNECT is not used) and
// never changes the configuration; claiming an interface that a driver owns simply fails with EBUSY. reset_node()
// issues USBDEVFS_RESET, a host-side USB port reset (the hub drives a bus reset on the device's port and the kernel
// re-enumerates it, as on every plug-in): no request or data is sent to the device firmware. The live tracker uses it
// only to recover a chip that stopped answering.
#pragma once

#include "pibridge/eye/error.h"
#include "pibridge/eye/transport.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::eye::usbfs {

/// The ioctl request numbers used (from <linux/usbdevice_fs.h>); exposed so a test can pin them.
unsigned long ioctl_control();
unsigned long ioctl_bulk();
unsigned long ioctl_claim_interface();
unsigned long ioctl_release_interface();
unsigned long ioctl_reset();
/// sizeof the kernel's usbdevfs_ctrltransfer / usbdevfs_bulktransfer.
std::size_t ctrl_transfer_size();
std::size_t bulk_transfer_size();

/// One opened usbfs node.
class UsbFs final : public Transport {
public:
    /// Open the node read/write.
    static Result<UsbFs> open(const std::filesystem::path& path);

    UsbFs(UsbFs&& o) noexcept;
    UsbFs& operator=(UsbFs&& o) noexcept;
    UsbFs(const UsbFs&) = delete;
    UsbFs& operator=(const UsbFs&) = delete;
    /// Releases the claimed interface and closes the node.
    ~UsbFs() override;

    /// Claim an interface. Fails with EBUSY when a kernel driver is bound; nothing is detached.
    Result<void> claim_interface(std::uint32_t interface);

    Result<Bytes> control(const Setup& setup, ByteView out, Duration timeout) override;
    Result<void> bulk_out(std::uint8_t ep, ByteView data, Duration timeout) override;
    Result<std::optional<Bytes>> bulk_in(std::uint8_t ep, std::size_t max_len, Duration timeout) override;

private:
    explicit UsbFs(int fd) : fd_(fd) {}
    void close_fd();

    int fd_ = -1;
    std::optional<std::uint32_t> claimed_;
};

/// USB port reset of the device behind a usbfs node (USBDEVFS_RESET). The kernel re-enumerates the device; interfaces
/// claimed through usbfs are released, so every handle has to be reopened afterwards.
Result<void> reset_node(const std::filesystem::path& path);

/// A USB device as seen in sysfs.
struct SysDevice {
    /// `/sys/bus/usb/devices/<name>`.
    std::filesystem::path sysfs;
    std::uint16_t vid = 0;
    std::uint16_t pid = 0;
    /// Bus number.
    std::uint32_t bus = 0;
    /// Device number on the bus.
    std::uint32_t devnum = 0;
    /// Product string.
    std::optional<std::string> product;
    /// Serial string (never logged by this library).
    std::optional<std::string> serial;
    /// Negotiated speed in Mb/s as text.
    std::optional<std::string> speed;
    /// `/dev/bus/usb/BBB/DDD`.
    std::filesystem::path devnode;

    /// Name of the kernel driver bound to interface 0, if any.
    std::optional<std::string> interface_driver() const;

    bool operator==(const SysDevice&) const = default;
};

/// List the USB devices under `sysfs_root` (normally `/sys/bus/usb/devices`); nodes are placed under `dev_root`
/// (normally `/dev/bus/usb`). Sorted by (bus, devnum).
std::vector<SysDevice> scan_sysfs(const std::filesystem::path& sysfs_root, const std::filesystem::path& dev_root);

/// EyeChips (2104:0561) currently on the bus (reads sysfs only).
std::vector<SysDevice> find_eye_chips();

} // namespace pibridge::eye::usbfs
