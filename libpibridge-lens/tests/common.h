// SPDX-License-Identifier: MIT
// Helpers shared by the libpibridge-lens tests (twins of the Rust test helpers in pimax-lens).
#pragma once

#include "pibridge/cam/io.h"
#include "pibridge/lens/config.h"
#include "pibridge/lens/lut.h"
#include "pibridge/lens/pvrdump.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::lens::testing {

inline constexpr float ASPECT = 3840.0f / 3552.0f;

/// A fresh directory in $TMPDIR (ctest sets it to the build directory).
inline std::string tmpdir(const std::string& name) {
    const char* d = std::getenv("TMPDIR");
    std::string p = std::string(d != nullptr && *d != 0 ? d : "/tmp") + "/pibridge-lens-test-" +
                    std::to_string(::getpid()) + "-" + name;
    if (std::system(("rm -rf '" + p + "'").c_str()) != 0) {
    }
    ::mkdir(p.c_str(), 0755);
    return p;
}

inline void rmrf(const std::string& p) {
    if (std::system(("rm -rf '" + p + "'").c_str()) != 0) {
    }
}

inline void write(const std::string& path, std::string_view text) {
    std::ofstream f(path, std::ios::binary);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
}

inline Env no_env() {
    return [](std::string_view) -> std::optional<std::string> { return std::nullopt; };
}

inline Env env_of(std::map<std::string, std::string> m) {
    return [m](std::string_view k) -> std::optional<std::string> {
        const auto it = m.find(std::string(k));
        if (it == m.end()) {
            return std::nullopt;
        }
        return it->second;
    };
}

/// The public PVR dump fixture of omarchyvr (rust/pimax-lens/tests/data/pvr-dump-sub.json, a 17x17 subsample);
/// nullopt (the test is skipped) when the checkout is not next to PiBridge.
inline std::optional<Dump> fixture_dump() {
    const auto text = cam::read_file(PIBRIDGE_LENS_DUMP_FIXTURE);
    if (!text) {
        std::printf("skipped: no %s\n", PIBRIDGE_LENS_DUMP_FIXTURE);
        return std::nullopt;
    }
    auto d = parse_dump(*text);
    if (!d) {
        std::printf("fixture: %s\n", d.error().c_str());
        return std::nullopt;
    }
    return std::move(*d);
}

/// The fixture's LUT file in `dir` (sub.bin).
inline std::optional<std::string> sub_lut_file(const std::string& dir) {
    const auto d = fixture_dump();
    if (!d) {
        return std::nullopt;
    }
    const auto l = Lut::from_dump(*d);
    if (!l) {
        return std::nullopt;
    }
    const auto b = l->to_bytes();
    const std::string p = dir + "/sub.bin";
    write(p, std::string_view(reinterpret_cast<const char*>(b.data()), b.size()));
    return p;
}

} // namespace pibridge::lens::testing
