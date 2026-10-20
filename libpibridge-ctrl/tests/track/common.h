// SPDX-License-Identifier: MIT
// Helpers shared by the tracker tests.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>

namespace pibridge::ctrl::track::testing {

/// rm -rf of a scratch path.
inline void rmrf(const std::string& p) {
    if (std::system(("rm -rf '" + p + "'").c_str()) != 0) {
        std::fprintf(stderr, "could not remove %s\n", p.c_str());
    }
}

/// A fresh path in $TMPDIR (ctest sets it to the build directory).
inline std::string tmp(const std::string& name) {
    const char* d = std::getenv("TMPDIR");
    std::string p = std::string(d != nullptr && *d != 0 ? d : "/tmp") + "/pibridge-ctrltrack-test-" +
                    std::to_string(::getpid()) + "-" + name;
    rmrf(p);
    return p;
}

} // namespace pibridge::ctrl::track::testing
