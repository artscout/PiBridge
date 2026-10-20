// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/{sys,hid,lib}.rs (MIT OR Apache-2.0).
//
// Thin wrappers over the Linux syscalls the I/O layer needs: an owning file descriptor, ppoll, eventfd, seqpacket
// socketpairs (the fake device), hidraw report reads and writes, and the error type of the I/O layer.
#pragma once

#include "pibridge/hmd/proto.h"

#include <chrono>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <poll.h>

namespace pibridge::hmd {

// ------------------------------------------------------------------------------------------------ errors

/// What went wrong in the I/O layer.
enum class Errc {
    /// No device session at the moment (also when the device vanishes during a request).
    NotConnected,
    /// No reply within the timeout, or wait_connected timed out.
    Timeout,
    /// Payload too long for one command.
    TooLong,
    /// The reply could not be used.
    BadReply,
    /// The report is not in the headset's allowlist (devices/*.toml); nothing was written.
    Refused,
    /// Operating system error (`os_errno`).
    Io,
};

struct Error {
    Errc code = Errc::Io;
    int os_errno = 0;
    /// Plain words (the refusal reason, the bad reply).
    std::string detail;

    /// Negative errno, the convention of the C API.
    int to_errno() const;
    std::string message() const;
};

template <class T>
using Result = std::expected<T, Error>;

/// An `Error` with code `Io` for errno `e`.
Error io_error(int e);

// ------------------------------------------------------------------------------------------------ file descriptor

/// Owning file descriptor (closed on destruction, movable).
class Fd {
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& o) noexcept : fd_(std::exchange(o.fd_, -1)) {}
    Fd& operator=(Fd&& o) noexcept;
    ~Fd();
    int get() const { return fd_; }
    explicit operator bool() const { return fd_ >= 0; }
    void reset();
    /// `dup` with close-on-exec.
    Result<Fd> try_clone() const;

private:
    int fd_ = -1;
};

// ------------------------------------------------------------------------------------------------ poll / eventfd

/// One entry for ppoll() waiting for input.
pollfd pollin(int fd);
/// ppoll with a timeout (nullopt = block). Returns the number of ready fds; EINTR is reported as 0.
Result<int> ppoll(std::span<pollfd> fds, std::optional<std::chrono::nanoseconds> timeout);
/// Non-blocking, close-on-exec eventfd used to wake a thread.
Result<Fd> eventfd();
/// Add 1 to an eventfd (ignores errors: a full counter still wakes the reader).
void eventfd_signal(int fd);
/// Read an eventfd back to zero (non-blocking).
void eventfd_drain(int fd);
/// Non-blocking SOCK_SEQPACKET socketpair: message boundaries like hidraw.
Result<std::pair<Fd, Fd>> seqpacket_pair();

// ------------------------------------------------------------------------------------------------ hidraw

namespace hid {
/// Open a hidraw node non-blocking and close-on-exec (read only, or read-write for IF0).
Result<Fd> open(const std::filesystem::path& path, bool write);
/// Read one report. nullopt when nothing is queued (EAGAIN/EINTR); short reads are zero padded.
Result<std::optional<std::size_t>> read_report(int fd, Report& out);
/// Write a command with the report id 0 prefix in one write().
Result<void> write_report(int fd, const Report& r);
/// True for errno values that mean the device is gone and the session should be torn down.
bool is_gone(int err);
} // namespace hid

// ------------------------------------------------------------------------------------------------ misc

/// CPU seconds (utime+stime) of a `/proc/<pid>/stat`-style file.
std::optional<double> proc_cpu_seconds(const std::filesystem::path& path);
/// Local wall-clock time as `HH:MM:SS`.
std::string local_hms();

} // namespace pibridge::hmd
