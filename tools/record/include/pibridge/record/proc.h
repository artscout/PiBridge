// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-components/src/components/proc.rs (the parts pibridge-record uses; MIT OR Apache-2.0).
// Processes by pid file and comm, never by name alone: a pid is only signalled when /proc/<pid>/comm is ours.
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::record {

// $PIBRIDGE_PROC_ROOT / $PIMAX_PROC_ROOT, else /proc (tests point it at a fake tree).
std::filesystem::path procRoot();
// Alive (exists and is not a zombie).
bool isAlive(const std::filesystem::path& proc, std::uint32_t pid);
std::optional<std::string> comm(const std::filesystem::path& proc, std::uint32_t pid);
// Some live process has this comm.
bool anyWithComm(const std::filesystem::path& proc, std::string_view name);
std::optional<std::uint32_t> readPid(const std::filesystem::path& pidFile);
// The comm the kernel gives a binary: its file name cut at 15 characters.
std::string commOf(const std::filesystem::path& program);

// Spawn in its own session, stdin /dev/null, stdout+stderr into `log` (the previous log kept as <log>.prev), the
// other descriptors closed. The pid, or why it could not start.
std::optional<std::uint32_t> spawnLogged(const std::filesystem::path& program, const std::vector<std::string>& args,
                                         const std::filesystem::path& log, std::string* error = nullptr);
// Fire and forget, all output to /dev/null, never a zombie (double fork). `argv[0]` is looked up on the PATH.
void spawnQuiet(const std::vector<std::string>& argv);
// Rust's ExitStatus Display: "exit status: N" or "signal: N (SIGNAME)".
std::string exitStatusText(int waitStatus);
// Free bytes on the filesystem of `dir` (nullopt when unknown).
std::optional<std::uint64_t> freeBytes(const std::filesystem::path& dir);

} // namespace pibridge::record
