// SPDX-License-Identifier: MIT
// The error type of libpibridge-eye. Every fallible call returns std::expected<T, Error> (docs/CONVENTIONS.md): no
// exception crosses the library boundary. Modules with their own failure taxonomy (TTP framing, TLV, pcap, replay)
// have their own error structs; this one is for I/O and policy refusals, like io::Error in the Rust original.
#pragma once

#include <expected>
#include <string>

namespace pibridge::eye {

/// What went wrong, in the classes the Rust code distinguishes (io::ErrorKind).
enum class Errc {
    io,                ///< an OS call failed (see os_errno)
    permission_denied, ///< refused by a guard (not in the frame list, denied opcode) or by the OS
    invalid_input,     ///< the caller passed something impossible (control OUT length differs from wLength)
    invalid_data,      ///< the device or a file sent bytes that do not frame
    short_transfer,    ///< a write moved fewer bytes than asked
    not_found,         ///< no such device or file
};

struct Error {
    Errc code = Errc::io;
    /// errno of the failed OS call, 0 when the error did not come from the OS.
    int os_errno = 0;
    /// Plain words; may be empty for a bare OS error.
    std::string message;

    /// An OS error: `context` (may be empty) plus errno.
    static Error os(int err, std::string context = {});
    /// A non-OS error.
    static Error make(Errc code, std::string message);

    /// Message and, for OS errors, strerror.
    std::string to_string() const;
};

template <class T>
using Result = std::expected<T, Error>;

} // namespace pibridge::eye
