// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-autod/src/{pairing,unpair}.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/pairing.h"

#include <charconv>
#include <fstream>
#include <iterator>
#include <system_error>
#include <vector>

#include <unistd.h>

namespace pibridge::hmd::pairing {

std::optional<LogEvent> parse_log(std::string_view line) {
    const auto has = [&](std::string_view w) { return line.find(w) != std::string_view::npos; };
    if (has("radio_pair_success_process") || has("pair finished")) {
        return LogEvent::Success;
    }
    if (has("radio_pair_cancel_or_timeout_process")) {
        return LogEvent::TimeoutOrCancel;
    }
    if (has("eSpiCmdStartPair") || has("Pairing device")) {
        return LogEvent::Started;
    }
    return std::nullopt;
}

std::optional<std::uint8_t> slot_of(std::string_view side) {
    if (side == "left") {
        return std::uint8_t{2};
    }
    if (side == "right") {
        return std::uint8_t{3};
    }
    return std::nullopt;
}

std::string_view side_of(std::uint8_t slot) { return slot == 2 ? "left" : "right"; }

std::string_view op_name(Op op) { return op == Op::Pair ? "pair" : "unpair"; }

std::string format_request(const Request& r) {
    return std::to_string(r.id) + " " + std::string(op_name(r.op)) + " " + std::string(side_of(r.slot)) + "\n";
}

std::optional<Request> parse_request(std::string_view s) {
    std::vector<std::string_view> t;
    std::size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
            i++;
        }
        const std::size_t b = i;
        while (i < s.size() && !(s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
            i++;
        }
        if (i > b) {
            t.push_back(s.substr(b, i - b));
        }
    }
    if (t.size() != 3) {
        return std::nullopt;
    }
    Request r;
    const auto [p, ec] = std::from_chars(t[0].data(), t[0].data() + t[0].size(), r.id);
    if (ec != std::errc{} || p != t[0].data() + t[0].size() || r.id == 0) {
        return std::nullopt;
    }
    if (t[1] == "pair") {
        r.op = Op::Pair;
    } else if (t[1] == "unpair") {
        r.op = Op::Unpair;
    } else {
        return std::nullopt;
    }
    const auto slot = slot_of(t[2]);
    if (!slot) {
        return std::nullopt;
    }
    r.slot = *slot;
    return r;
}

std::expected<std::optional<Request>, std::string> take_request(const std::filesystem::path& path) {
    // rename first: the request is ours alone, and a half-written file cannot be read (the app renames into place)
    const std::filesystem::path mine = path.string() + ".taken." + std::to_string(::getpid());
    std::error_code ec;
    std::filesystem::rename(path, mine, ec);
    if (ec) {
        if (ec == std::errc::no_such_file_or_directory) {
            return std::optional<Request>{};
        }
        return std::unexpected(ec.message());
    }
    std::string s;
    {
        std::ifstream f(mine, std::ios::binary);
        s.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    std::filesystem::remove(mine, ec);
    if (s.size() > 256) {
        return std::unexpected(std::string("controller request too long"));
    }
    if (auto r = parse_request(s)) {
        return std::optional<Request>(*r);
    }
    return std::unexpected("bad controller request \"" + s + "\"");
}

} // namespace pibridge::hmd::pairing
