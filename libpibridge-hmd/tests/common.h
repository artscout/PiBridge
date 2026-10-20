// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-proto/tests/common/mod.rs and pimax/tests/common/mod.rs (MIT OR Apache-2.0).
// Fixture loading shared by the tests: real captures as `[f64 time][64 B]` records, gzip compressed.
#pragma once

#include "pibridge/hmd/proto.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#if PIBRIDGE_HAVE_ZLIB
#include <zlib.h>
#endif

namespace testutil {

namespace fs = std::filesystem;

inline fs::path data_dir() { return PIBRIDGE_HMD_TEST_DATA_DIR; }
inline fs::path fixtures_dir() { return PIBRIDGE_FIXTURES_DIR; }

/// The path of a capture: tests/data first, then the omarchyvr fixtures dir. nullopt when neither has it.
inline std::optional<fs::path> fixture(const std::string& name) {
    std::error_code ec;
    for (const auto& d : {data_dir(), fixtures_dir()}) {
        if (fs::exists(d / name, ec)) {
            return d / name;
        }
    }
    return std::nullopt;
}

inline std::optional<std::vector<std::uint8_t>> gunzip(const fs::path& p) {
#if PIBRIDGE_HAVE_ZLIB
    gzFile f = gzopen(p.c_str(), "rb");
    if (!f) {
        return std::nullopt;
    }
    std::vector<std::uint8_t> out;
    std::uint8_t buf[65536];
    int n = 0;
    while ((n = gzread(f, buf, sizeof buf)) > 0) {
        out.insert(out.end(), buf, buf + n);
    }
    gzclose(f);
    return out;
#else
    (void)p;
    return std::nullopt;
#endif
}

inline std::optional<std::string> read_text(const fs::path& p) {
    std::ifstream f(p);
    if (!f) {
        return std::nullopt;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

using Rec = std::vector<std::pair<double, pibridge::hmd::Report>>;

/// `[f64 time][64 B]` records; nullopt (and a "skipped" line) when the capture is not available.
inline std::optional<Rec> load_rec(const std::string& name) {
    const auto p = fixture(name);
    if (!p) {
        std::fprintf(stderr, "  skipped: capture %s not found (set PIBRIDGE_FIXTURES_DIR)\n", name.c_str());
        return std::nullopt;
    }
    const auto d = gunzip(*p);
    if (!d) {
        std::fprintf(stderr, "  skipped: %s needs zlib\n", name.c_str());
        return std::nullopt;
    }
    Rec v;
    for (std::size_t i = 0; i + 72 <= d->size(); i += 72) {
        double t = 0;
        std::memcpy(&t, d->data() + i, 8);
        pibridge::hmd::Report r{};
        std::memcpy(r.data(), d->data() + i + 8, 64);
        v.emplace_back(t, r);
    }
    return v;
}

/// A fresh temporary directory (removed and recreated).
inline fs::path tmpdir(const std::string& name) {
    const fs::path d = fs::temp_directory_path() / ("pibridge-test-" + name + "-" + std::to_string(::getpid()));
    std::error_code ec;
    fs::remove_all(d, ec);
    fs::create_directories(d);
    return d;
}

inline pibridge::hmd::Report bytes(std::initializer_list<std::uint8_t> b) {
    pibridge::hmd::Report r{};
    std::copy(b.begin(), b.end(), r.begin());
    return r;
}

inline std::string hex(std::span<const std::uint8_t> r) {
    std::string s;
    char b[4];
    for (std::size_t i = 0; i < r.size(); i++) {
        std::snprintf(b, sizeof b, "%02x", r[i]);
        if (i) {
            s += ' ';
        }
        s += b;
    }
    return s;
}

inline std::vector<std::uint8_t> pad(std::initializer_list<std::uint8_t> b) {
    std::vector<std::uint8_t> r(b);
    r.resize(64, 0);
    return r;
}

} // namespace testutil
