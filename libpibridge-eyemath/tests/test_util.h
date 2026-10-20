// SPDX-License-Identifier: MIT
// Test helpers: a private temporary directory per test binary (never the user's real config).
#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include <unistd.h>

namespace eyemath_test {

/// A fresh directory `<tmp>/<name>-<pid>`, removed again by the destructor.
struct TempDir {
    std::filesystem::path path;
    explicit TempDir(std::string_view name)
        : path(std::filesystem::temp_directory_path() / (std::string(name) + "-" + std::to_string(::getpid()))) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
};

inline void write_file(const std::filesystem::path& p, std::string_view text) {
    std::filesystem::create_directories(p.parent_path());
    std::ofstream f(p, std::ios::binary);
    f << text;
}

} // namespace eyemath_test
