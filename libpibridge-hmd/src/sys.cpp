// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/{sys,hid,lib}.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/sys.h"

#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>

#include <fcntl.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

namespace pibridge::hmd {

int Error::to_errno() const {
    switch (code) {
    case Errc::NotConnected: return -ENOTCONN;
    case Errc::Timeout: return -ETIMEDOUT;
    case Errc::TooLong: return -EMSGSIZE;
    case Errc::BadReply: return -EPROTO;
    case Errc::Refused: return -EPERM;
    case Errc::Io: return -(os_errno ? os_errno : EIO);
    }
    return -EIO;
}

std::string Error::message() const {
    switch (code) {
    case Errc::NotConnected: return "headset not connected";
    case Errc::Timeout: return "timed out";
    case Errc::TooLong: return "payload longer than " + std::to_string(MAX_DATA) + " bytes";
    case Errc::BadReply: return "bad reply: " + detail;
    case Errc::Refused: return detail.empty() ? std::string("refused by the allowlist") : detail;
    case Errc::Io: return "I/O error: " + std::string(std::strerror(os_errno ? os_errno : EIO));
    }
    return "error";
}

Error io_error(int e) { return Error{Errc::Io, e, {}}; }

Fd& Fd::operator=(Fd&& o) noexcept {
    if (this != &o) {
        reset();
        fd_ = std::exchange(o.fd_, -1);
    }
    return *this;
}

Fd::~Fd() { reset(); }

void Fd::reset() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

Result<Fd> Fd::try_clone() const {
    const int d = ::fcntl(fd_, F_DUPFD_CLOEXEC, 0);
    if (d < 0) {
        return std::unexpected(io_error(errno));
    }
    return Fd(d);
}

pollfd pollin(int fd) { return pollfd{fd, POLLIN, 0}; }

Result<int> ppoll(std::span<pollfd> fds, std::optional<std::chrono::nanoseconds> timeout) {
    timespec ts{};
    const timespec* tsp = nullptr;
    if (timeout) {
        const auto ns = std::max<std::chrono::nanoseconds::rep>(timeout->count(), 0);
        ts.tv_sec = static_cast<time_t>(ns / 1'000'000'000);
        ts.tv_nsec = static_cast<long>(ns % 1'000'000'000);
        tsp = &ts;
    }
    const int rc = ::ppoll(fds.data(), static_cast<nfds_t>(fds.size()), tsp, nullptr);
    if (rc < 0) {
        if (errno == EINTR) {
            return 0;
        }
        return std::unexpected(io_error(errno));
    }
    return rc;
}

Result<Fd> eventfd() {
    const int fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (fd < 0) {
        return std::unexpected(io_error(errno));
    }
    return Fd(fd);
}

void eventfd_signal(int fd) {
    const std::uint64_t one = 1;
    [[maybe_unused]] const auto n = ::write(fd, &one, sizeof one);
}

void eventfd_drain(int fd) {
    std::uint64_t v = 0;
    [[maybe_unused]] const auto n = ::read(fd, &v, sizeof v);
}

Result<std::pair<Fd, Fd>> seqpacket_pair() {
    int sv[2] = {-1, -1};
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, sv) < 0) {
        return std::unexpected(io_error(errno));
    }
    return std::pair{Fd(sv[0]), Fd(sv[1])};
}

namespace hid {

Result<Fd> open(const std::filesystem::path& path, bool write) {
    const int fd = ::open(path.c_str(), (write ? O_RDWR : O_RDONLY) | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return std::unexpected(io_error(errno));
    }
    return Fd(fd);
}

Result<std::optional<std::size_t>> read_report(int fd, Report& out) {
    Report buf{};
    const auto n = ::read(fd, buf.data(), buf.size());
    if (n > 0) {
        out = buf;
        return std::optional<std::size_t>(static_cast<std::size_t>(n));
    }
    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return std::optional<std::size_t>{};
    }
    return std::unexpected(io_error(errno));
}

Result<void> write_report(int fd, const Report& r) {
    const auto buf = to_write_buf(r);
    const auto n = ::write(fd, buf.data(), buf.size());
    if (n < 0) {
        return std::unexpected(io_error(errno));
    }
    if (static_cast<std::size_t>(n) != buf.size()) {
        return std::unexpected(io_error(EIO));
    }
    return {};
}

bool is_gone(int err) {
    switch (err) {
    case ENODEV:
    case EIO:
    case ENOTCONN:
    case EPIPE:
    case ENOENT:
    case ECONNRESET:
    case EBADF: return true;
    default: return false;
    }
}

} // namespace hid

std::optional<double> proc_cpu_seconds(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) {
        return std::nullopt;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string s = ss.str();
    const auto p = s.rfind(')');
    if (p == std::string::npos || p + 2 > s.size()) {
        return std::nullopt;
    }
    std::vector<std::string> fields;
    std::string cur;
    for (std::size_t i = p + 2; i < s.size(); i++) {
        if (s[i] == ' ') {
            fields.push_back(cur);
            cur.clear();
        } else {
            cur += s[i];
        }
    }
    fields.push_back(cur);
    // after the comm field: state is index 0, utime index 11, stime index 12
    if (fields.size() < 13) {
        return std::nullopt;
    }
    try {
        const double ut = std::stod(fields[11]);
        const double st = std::stod(fields[12]);
        return (ut + st) / static_cast<double>(::sysconf(_SC_CLK_TCK));
    } catch (...) {
        return std::nullopt;
    }
}

std::string local_hms() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    ::localtime_r(&t, &tm);
    char b[16];
    std::snprintf(b, sizeof b, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return b;
}

} // namespace pibridge::hmd
