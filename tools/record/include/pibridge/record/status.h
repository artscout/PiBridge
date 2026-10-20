// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/status.rs (MIT OR Apache-2.0).
// What the recorder tells the world: `<session dir>/record.status` (key=value, rewritten atomically about every
// second and at every state change; read by `pibridge-record status` and the app's Record card), `<session
// dir>/record.events` (one JSON object per line, omarchyvrd's event shape) and a desktop notification for the
// important ones.
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace pibridge::record {

struct Status {
    std::string state; // starting | recording | finalizing | idle | error
    std::uint32_t pid = 0;
    std::string file;
    double started_unix = 0;
    std::string eyes;
    std::uint32_t fps = 0;
    std::string quality;
    std::string encoder;
    std::string audio; // what is mixed into the sound: "" (none), "output" or "output+mic"
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint64_t frames = 0;       // written to the encoder (duplicates included: the file is constant frame rate)
    std::uint64_t dropped_tap = 0;  // the tap's GPU readback was still busy (the compositor never waits)
    std::uint64_t dropped_ring = 0; // the ring was full (recorder or encoder behind)
    std::uint64_t duplicated = 0;   // repeated to keep the frame rate
    std::string note;               // last error or stop reason (empty while recording normally)
    double updated_unix = 0;
    std::string source; // tap | mirror (PiBridge only; omarchyvr's readers ignore the key)

    bool operator==(const Status&) const = default;

    std::string toText() const;
    static Status parse(std::string_view text);
    static std::optional<Status> load(const std::filesystem::path& p);
    void save(const std::filesystem::path& p) const;
    // Seconds recorded so far (or in total, once idle).
    double seconds() const;
    // One line for humans.
    std::string summary() const;
};

std::filesystem::path statusPath(const std::filesystem::path& sessionDir);
std::filesystem::path eventsPath(const std::filesystem::path& sessionDir);
// The event line shape: {"event":E,"t":T,"id":"record","text":...} (t is unix time).
std::string eventLine(std::string_view event, double unix, std::string_view text);
// Append an event (notice | critical) and, if `notify` (and PIBRIDGE_/PIMAX_RECORD_NO_NOTIFY is not set), show a
// desktop notification. Also printed to stderr (the recorder's log).
void event(const std::filesystem::path& sessionDir, std::string_view event, std::string_view text, bool notify);

// Write atomically (a temporary name next to it, then rename).
bool writeAtomic(const std::filesystem::path& p, std::string_view data);
double unixNow();

} // namespace pibridge::record
