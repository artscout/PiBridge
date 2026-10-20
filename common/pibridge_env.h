// SPDX-License-Identifier: MIT
// Environment overrides: `PIBRIDGE_<NAME>` is the name; the older `PIMAX_<NAME>` (the names omarchyvr and pimax-eyecal
// use, e.g. PIMAX_EYE_FLIP_X) is still accepted when the PIBRIDGE_ one is not set. See docs/CONVENTIONS.md.
#pragma once

#include <cstdlib>
#include <string>
#include <string_view>

namespace pibridge {

/// The value of `PIBRIDGE_<name>`, else of `PIMAX_<name>`, else nullptr. `name` is given without the prefix
/// ("EYE_FLIP_X"). Like std::getenv, the pointer is valid until the environment changes.
inline const char* env(std::string_view name) {
    const std::string n(name);
    if (const char* v = std::getenv(("PIBRIDGE_" + n).c_str())) {
        return v;
    }
    return std::getenv(("PIMAX_" + n).c_str());
}

/// The full name a value of env(name) came from ("PIBRIDGE_EYE_FRAMES" or "PIMAX_EYE_FRAMES"), for messages; the
/// PIBRIDGE_ name when neither is set.
inline std::string env_name(std::string_view name) {
    const std::string n(name);
    if (std::getenv(("PIBRIDGE_" + n).c_str()) == nullptr && std::getenv(("PIMAX_" + n).c_str()) != nullptr) {
        return "PIMAX_" + n;
    }
    return "PIBRIDGE_" + n;
}

} // namespace pibridge
