// SPDX-License-Identifier: MIT
// The error type of the camera library: the C++ twin of Rust's std::io::Error as pimax-cam uses it. text() is the
// Rust Display form ("No such file or directory (os error 2)" for an OS error, the message for a custom one), which
// is what pimax_cam_last_error() and the tool print.
#pragma once

#include <expected>
#include <optional>
#include <string>
#include <string_view>

namespace pibridge::cam {

/// The io::ErrorKind values the camera code tests or creates.
enum class ErrorKind {
    Other,
    NotFound,
    PermissionDenied,
    InvalidData,
    InvalidInput,
    TimedOut,
    ConnectionAborted,
    AddrInUse,
    WouldBlock,
    Interrupted,
    Unsupported,
};

struct IoError {
    ErrorKind kind = ErrorKind::Other;
    int os = 0;      ///< errno for an OS error, 0 for a custom one
    std::string msg; ///< custom message

    /// An OS error from errno (kind derived the way Rust's decode_error_kind does for the codes we meet).
    static IoError from_errno(int e);
    /// errno now.
    static IoError last_os();
    /// io::Error::new(kind, msg).
    static IoError custom(ErrorKind k, std::string m) { return IoError{k, 0, std::move(m)}; }
    /// io::Error::other(msg).
    static IoError other(std::string m) { return IoError{ErrorKind::Other, 0, std::move(m)}; }

    /// Rust Display.
    std::string text() const;
};

template <class T>
using Result = std::expected<T, IoError>;

/// std::unexpected shorthand.
inline std::unexpected<IoError> fail(IoError e) {
    return std::unexpected<IoError>(std::move(e));
}

/// strerror text of errno `e`, as Rust prints it (glibc's message).
std::string os_error_text(int e);

/// std::fs::read: the whole file.
Result<std::string> read_file(const std::string& path);
/// Path::is_file (follows symlinks).
bool is_file(const std::string& path);
/// Path::exists (follows symlinks).
bool path_exists(const std::string& path);
/// Path::join for a relative `rel` ("" + rel = rel; no doubled separator).
std::string path_join(const std::string& base, std::string_view rel);
/// The value of an environment variable, nullopt when unset (std::env::var_os).
std::optional<std::string> env_os(const char* name);
/// std::env::var: nullopt when unset or not valid UTF-8.
std::optional<std::string> env_utf8(const char* name);
/// std::fs::read_to_string: the file, or the io error ("stream did not contain valid UTF-8" for invalid UTF-8).
Result<std::string> read_to_string(const std::string& path);
/// std::fs::create_dir_all (the same recursion and the same "already a directory" rule as Rust's).
Result<void> create_dir_all(const std::string& path);
/// std::fs::write: create or truncate (mode 0666 before the umask) and write everything.
Result<void> write_file(const std::string& path, std::string_view data);
/// Path::is_dir (follows symlinks).
bool is_dir(const std::string& path);
/// Path::parent as a string: nullopt for "" and "/", "" for a bare name ("a" -> "").
std::optional<std::string> path_parent(const std::string& path);

} // namespace pibridge::cam
