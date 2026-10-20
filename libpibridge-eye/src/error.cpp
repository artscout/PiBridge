// SPDX-License-Identifier: MIT
#include "pibridge/eye/error.h"

#include <cerrno>
#include <cstring>

namespace pibridge::eye {

Error Error::os(int err, std::string context) {
    Errc c = Errc::io;
    if (err == EACCES || err == EPERM) {
        c = Errc::permission_denied;
    } else if (err == ENOENT || err == ENODEV) {
        c = Errc::not_found;
    }
    return Error{c, err, std::move(context)};
}

Error Error::make(Errc code, std::string message) {
    return Error{code, 0, std::move(message)};
}

std::string Error::to_string() const {
    if (os_errno == 0) {
        return message;
    }
    const std::string sys = std::strerror(os_errno);
    return message.empty() ? sys : message + ": " + sys;
}

} // namespace pibridge::eye
