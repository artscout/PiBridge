// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/cli.rs (MIT OR Apache-2.0).
// `pibridge-record start|stop|status` (the same options as `omarchyvr record`) and `pibridge-record run` (the recorder).
#pragma once

#include "pibridge/record/run.h"
#include "pibridge/record/settings.h"

#include <chrono>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace pibridge::record {

extern const char* const kUsage;

// The session's recorder pid file name and comm.
inline constexpr char kPidName[] = "record";
inline constexpr char kComm[] = "pibridge-record";
// How long `stop` waits for the file to be finished.
inline constexpr std::chrono::seconds kStopGrace{25};

using Overrides = std::vector<std::pair<std::string, std::string>>;

struct Cmd {
    enum class Kind { Start, Stop, Status, Run, Help, Brief };
    Kind kind = Kind::Help;
    Overrides overrides; // Start, Run
    bool save = false;   // Start
    bool stop = false;   // Brief: `stop --brief` (else `start --brief`)
    std::string source = "auto"; // Start, Run: --source auto | tap | mirror (not a setting: the session decides)
    bool operator==(const Cmd&) const = default;
};

// Parse `start|stop|status|run [--key value ...]`. Every value is validated here (a typo must fail here, not in the
// detached recorder).
std::expected<Cmd, std::string> parse(const std::vector<std::string>& args);
// The file settings (PiBridge's record.toml, else omarchyvr's) with the flags on top.
std::expected<Settings, std::string> effective(const Roots& r, const Overrides& overrides);
// The recorder binary: $PIBRIDGE_RECORD_BIN / $PIMAX_RECORD_BIN, else next to this executable, else `pibridge-record`.
std::filesystem::path recorderBin();
std::filesystem::path pidFile(const std::filesystem::path& sessionDir);
// The running recorder's pid (pid file + comm), if any.
std::optional<std::uint32_t> runningPid(const Roots& r);
// The argv of the recorder (`run` + the settings).
std::vector<std::string> runArgs(const Settings& s, Source source = Source::Tap);
// The source for `start`: `wanted` (auto | tap | mirror) against what runs. auto: SteamVR's mirror while vrserver
// runs, else Monado's tap while monado-service runs. Err: why there is nothing to record.
std::expected<Source, std::string> pickSource(const Roots& r, const std::string& wanted);

// `start`: the status line once recording, else why not.
std::expected<std::string, std::string> start(const Roots& r, const Overrides& overrides, bool save,
                                              const std::string& source = "auto");
// `stop`: SIGINT by pid, wait for the file to be finished. The last status line.
std::expected<std::string, std::string> stop(const Roots& r);
// `stop --brief`: "Saved: pimax-20261016-120000.mp4", or why nothing was saved, in plain words.
std::expected<std::string, std::string> stopBrief(const Roots& r);
// `status`: one line.
std::string statusLine(const Roots& r);

// Run a command; the process exit code (2 for usage errors, 1 for failures).
int main(const std::vector<std::string>& args);

} // namespace pibridge::record
