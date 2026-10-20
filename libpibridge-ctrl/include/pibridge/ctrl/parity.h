// SPDX-License-Identifier: MIT
//
// The parity switch of the controller code (docs/research/ctrl-tracking-improvements.md). PiBridge improves on the Rust
// tracker and fuser it was ported from; `PIBRIDGE_CTRL_PARITY=1` (or `on`/`true`) brings back the exact Rust behaviour,
// bit for bit, which the difftests against the Rust oracle use. Read once per process.
#pragma once

#include <cstdlib>
#include <string_view>

namespace pibridge::ctrl {

inline bool parity_mode() {
    static const bool on = [] {
        const char* v = std::getenv("PIBRIDGE_CTRL_PARITY");
        if (v == nullptr) {
            return false;
        }
        const std::string_view s(v);
        return s == "1" || s == "on" || s == "true";
    }();
    return on;
}

namespace track {
using ctrl::parity_mode;
}

} // namespace pibridge::ctrl
