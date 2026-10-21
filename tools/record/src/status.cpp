// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/status.rs (MIT OR Apache-2.0).
#include "pibridge/record/status.h"

#include "pibridge/record/proc.h"
#include "pibridge/record/settings.h"
#include "pibridge_env.h"

#include <chrono>
#include <cstdio>
#include <format>
#include <fstream>
#include <sstream>

#include <unistd.h>

namespace pibridge::record {

namespace fs = std::filesystem;

double unixNow() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

bool writeAtomic(const fs::path& p, std::string_view data) {
    std::error_code ec;
    if (p.has_parent_path()) {
        fs::create_directories(p.parent_path(), ec);
    }
    fs::path tmp = p;
    tmp.replace_extension(std::format("tmp{}", getpid()));
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) {
            return false;
        }
        f.write(data.data(), static_cast<std::streamsize>(data.size()));
        if (!f.flush()) {
            return false;
        }
    }
    fs::rename(tmp, p, ec);
    return !ec;
}

std::string Status::toText() const {
    std::string n = note;
    for (char& c : n) {
        if (c == '\n') {
            c = ' ';
        }
    }
    return std::format("state={}\npid={}\nfile={}\nstarted_unix={:.1f}\neyes={}\nfps={}\nquality={}\nencoder={}\n"
                       "audio={}\nwidth={}\nheight={}\nframes={}\ndropped_tap={}\ndropped_ring={}\nduplicated={}\n"
                       "note={}\nupdated_unix={:.1f}\nsource={}\n",
                       state, pid, file, started_unix, eyes, fps, quality, encoder, audio, width, height, frames,
                       dropped_tap, dropped_ring, duplicated, n, updated_unix, source);
}

namespace {
std::string_view trimWs(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
        s.remove_suffix(1);
    }
    return s;
}
} // namespace

Status Status::parse(std::string_view text) {
    Status s;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find('\n', start);
        std::string_view line = text.substr(start, end == std::string_view::npos ? std::string_view::npos : end - start);
        start = end == std::string_view::npos ? text.size() : end + 1;
        if (!line.empty() && line.back() == '\r') {
            line.remove_suffix(1);
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos) {
            continue;
        }
        const std::string_view k = line.substr(0, eq);
        const std::string_view v = line.substr(eq + 1);
        const auto n = [&] { return parseU64(trimWs(v)).value_or(0); };
        const auto f = [&] { return parseF64(trimWs(v)).value_or(0.0); };
        if (k == "state") {
            s.state = v;
        } else if (k == "pid") {
            s.pid = static_cast<std::uint32_t>(n());
        } else if (k == "file") {
            s.file = v;
        } else if (k == "started_unix") {
            s.started_unix = f();
        } else if (k == "eyes") {
            s.eyes = v;
        } else if (k == "fps") {
            s.fps = static_cast<std::uint32_t>(n());
        } else if (k == "quality") {
            s.quality = v;
        } else if (k == "encoder") {
            s.encoder = v;
        } else if (k == "audio") {
            s.audio = v;
        } else if (k == "width") {
            s.width = static_cast<std::uint32_t>(n());
        } else if (k == "height") {
            s.height = static_cast<std::uint32_t>(n());
        } else if (k == "frames") {
            s.frames = n();
        } else if (k == "dropped_tap") {
            s.dropped_tap = n();
        } else if (k == "dropped_ring") {
            s.dropped_ring = n();
        } else if (k == "duplicated") {
            s.duplicated = n();
        } else if (k == "note") {
            s.note = v;
        } else if (k == "updated_unix") {
            s.updated_unix = f();
        } else if (k == "source") {
            s.source = v;
        }
    }
    return s;
}

std::optional<Status> Status::load(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        return std::nullopt;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return parse(ss.str());
}

void Status::save(const fs::path& p) const { writeAtomic(p, toText()); }

double Status::seconds() const { return fps == 0 ? 0.0 : double(frames) / double(fps); }

std::string Status::summary() const {
    const double secs = seconds();
    const std::string len = std::format("{}:{:02}", std::uint64_t(secs / 60.0), std::uint64_t(secs) % 60);
    if (state == "recording") {
        return std::format("recording {} to {} ({} eye(s), {}x{} at {} fps, {} quality, {}, sound: {}; dropped {} + {}, "
                           "repeated {})",
                           len, file, eyes == "both" ? 2 : 1, width, height, fps, quality, encoder,
                           audio.empty() ? std::string("none") : audio, dropped_tap, dropped_ring, duplicated);
    }
    if (state == "starting") {
        if (source == "mirror") {
            return "starting (waiting for SteamVR's mirror)";
        }
        if (source == "layer") {
            return "starting (waiting for the Vulkan capture layer)";
        }
        return "starting (waiting for Monado's recorder tap)";
    }
    if (state == "finalizing") {
        return "finishing " + file;
    }
    if (state == "error") {
        return "not recording: " + note;
    }
    if (!file.empty()) {
        return std::format("not recording (last: {} {}{})", file, len, note.empty() ? std::string() : ", " + note);
    }
    return "not recording";
}

fs::path statusPath(const fs::path& sessionDir) { return sessionDir / "record.status"; }
fs::path eventsPath(const fs::path& sessionDir) { return sessionDir / "record.events"; }

namespace {
// Rust's chars(): code points; control characters below 0x20 as \u00XX.
std::string jsonStr(std::string_view s) {
    std::string o;
    o.reserve(s.size() + 2);
    o.push_back('"');
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        if (c == '"') {
            o += "\\\"";
        } else if (c == '\\') {
            o += "\\\\";
        } else if (c == '\n') {
            o += "\\n";
        } else if (c < 0x20) {
            o += std::format("\\u{:04x}", unsigned(c));
        } else {
            o.push_back(ch);
        }
    }
    o.push_back('"');
    return o;
}
} // namespace

std::string eventLine(std::string_view ev, double unix, std::string_view text) {
    return std::format("{{\"event\":{},\"t\":{:.1f},\"id\":\"record\",\"text\":{}}}", jsonStr(ev), unix, jsonStr(text));
}

void event(const fs::path& sessionDir, std::string_view ev, std::string_view text, bool notify) {
    std::fprintf(stderr, "pibridge-record: %.*s\n", int(text.size()), text.data());
    {
        std::ofstream f(eventsPath(sessionDir), std::ios::app);
        if (f) {
            f << eventLine(ev, unixNow(), text) << '\n';
        }
    }
    if (notify && pibridge::env("RECORD_NO_NOTIFY") == nullptr) {
        spawnQuiet({"notify-send", "-a", "pibridge", "-u", ev == "critical" ? "critical" : "normal", "-i", "media-record",
                    "PiBridge recording", std::string(text)});
    }
}

} // namespace pibridge::record
