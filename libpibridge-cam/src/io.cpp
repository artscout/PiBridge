// SPDX-License-Identifier: MIT
#include "pibridge/cam/io.h"

#include "pibridge/cam/rustfmt.h"

#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <format>

namespace pibridge::cam {

std::string os_error_text(int e) {
    char buf[256];
    // the GNU strerror_r returns the message (maybe not in buf)
    const char* s = strerror_r(e, buf, sizeof buf);
    return s;
}

IoError IoError::from_errno(int e) {
    ErrorKind k = ErrorKind::Other;
    switch (e) {
    case EPERM:
    case EACCES: k = ErrorKind::PermissionDenied; break;
    case ENOENT: k = ErrorKind::NotFound; break;
    case EINTR: k = ErrorKind::Interrupted; break;
    case EAGAIN: k = ErrorKind::WouldBlock; break;
    case EADDRINUSE: k = ErrorKind::AddrInUse; break;
    case ECONNABORTED: k = ErrorKind::ConnectionAborted; break;
    case ETIMEDOUT: k = ErrorKind::TimedOut; break;
    case EINVAL: k = ErrorKind::InvalidInput; break;
    case ENOSYS:
    case EOPNOTSUPP: k = ErrorKind::Unsupported; break;
    default: break;
    }
    return IoError{k, e, {}};
}

IoError IoError::last_os() {
    return from_errno(errno);
}

std::string IoError::text() const {
    if (os != 0) {
        return std::format("{} (os error {})", os_error_text(os), os);
    }
    return msg;
}

Result<std::string> read_file(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return fail(IoError::last_os());
    }
    std::string out;
    char buf[65536];
    for (;;) {
        const ssize_t n = ::read(fd, buf, sizeof buf);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            const IoError e = IoError::last_os();
            ::close(fd);
            return fail(e);
        }
        if (n == 0) {
            break;
        }
        out.append(buf, static_cast<std::size_t>(n));
    }
    ::close(fd);
    return out;
}

bool is_file(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool path_exists(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0;
}

std::string path_join(const std::string& base, std::string_view rel) {
    if (!rel.empty() && rel.front() == '/') {
        return std::string(rel);
    }
    if (base.empty()) {
        return std::string(rel);
    }
    if (base.back() == '/') {
        return base + std::string(rel);
    }
    return base + "/" + std::string(rel);
}

std::optional<std::string> env_os(const char* name) {
    const char* v = std::getenv(name);
    if (v == nullptr) {
        return std::nullopt;
    }
    return std::string(v);
}

std::optional<std::string> env_utf8(const char* name) {
    auto v = env_os(name);
    if (v && !rfmt::utf8_valid(*v)) {
        return std::nullopt;
    }
    return v;
}

Result<std::string> read_to_string(const std::string& path) {
    auto b = read_file(path);
    if (!b) {
        return b;
    }
    if (!rfmt::utf8_valid(*b)) {
        return fail(IoError::custom(ErrorKind::InvalidData, "stream did not contain valid UTF-8"));
    }
    return b;
}

bool is_dir(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::optional<std::string> path_parent(const std::string& path) {
    // Path::parent over the components (trailing slashes and "." components are not components).
    std::string p = path;
    while (p.size() > 1 && p.back() == '/') {
        p.pop_back();
    }
    while (p.size() > 2 && p.ends_with("/.")) {
        p.resize(p.size() - 2);
        while (p.size() > 1 && p.back() == '/') {
            p.pop_back();
        }
    }
    if (p.empty() || p == "/" || p == ".") {
        return std::nullopt;
    }
    const auto i = p.rfind('/');
    if (i == std::string::npos) {
        return std::string();
    }
    std::string d = p.substr(0, i);
    while (d.size() > 1 && d.back() == '/') {
        d.pop_back();
    }
    return d.empty() ? std::string("/") : d;
}

Result<void> create_dir_all(const std::string& path) {
    if (path.empty()) {
        return {};
    }
    if (::mkdir(path.c_str(), 0777) == 0) {
        return {};
    }
    const int e = errno;
    if (e != ENOENT) {
        if (is_dir(path)) {
            return {};
        }
        return fail(IoError::from_errno(e));
    }
    const auto parent = path_parent(path);
    if (!parent) {
        return fail(IoError::custom(ErrorKind::Other, "failed to create whole tree"));
    }
    if (auto r = create_dir_all(*parent); !r) {
        return r;
    }
    if (::mkdir(path.c_str(), 0777) == 0) {
        return {};
    }
    const int e2 = errno;
    if (is_dir(path)) {
        return {};
    }
    return fail(IoError::from_errno(e2));
}

Result<void> write_file(const std::string& path, std::string_view data) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
    if (fd < 0) {
        return fail(IoError::last_os());
    }
    std::size_t off = 0;
    while (off < data.size()) {
        const ssize_t n = ::write(fd, data.data() + off, data.size() - off);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            const IoError e = IoError::last_os();
            ::close(fd);
            return fail(e);
        }
        if (n == 0) {
            ::close(fd);
            return fail(IoError::custom(ErrorKind::Other, "failed to write whole buffer"));
        }
        off += static_cast<std::size_t>(n);
    }
    ::close(fd);
    return {};
}

} // namespace pibridge::cam
